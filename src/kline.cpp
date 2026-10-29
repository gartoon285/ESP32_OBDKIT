#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "kline.h"
#include "pins.h"        /* pin assignments for ESP32_OBDKIT V1 */
#include "live_data.h"   /* central protocol-agnostic struct the dashboard reads */
#include "wdt.h"

#define KLINE_TX_PIN   PIN_KLINE_TX   /* IO8 -> TLP291 (U1) input  */
#define KLINE_RX_PIN   PIN_KLINE_RX   /* IO9 <- TLP291 (U2) output */
#define KLINE_BAUD     10400
#define KLINE_SERIAL   Serial2

/* ── live-poll tuning ──────────────────────────────────────────────────────────
 * The K-line session only requires a poll at least every ~2-3 s to stay alive —
 * that is a *ceiling*, not a floor, so we poll as fast as the bus allows and let
 * the ECU's own P2/P3 timing set the real cadence.
 *   KLINE_POLL_GAP_MS — min gap between polls; small so we yield to other tasks
 *                       without throttling the sample rate.
 *   KLINE_O2_EVERY    — read the O2 table (group B) once per N engine polls, so
 *                       the engine gauges refresh at full rate instead of being
 *                       halved by an extra round-trip every loop.
 *   KLINE_O2_TIMEOUT_MS — response timeout for the O2 read only. Kept short (vs
 *                       the 1000ms engine timeout) so a missed O2 frame doesn't
 *                       stall the fast poll loop; B is retried next cycle.
 *   KLINE_VERBOSE     — 1 dumps every frame + decoded line to Serial (blocking
 *                       USB writes, tens of ms/loop). Keep 0 for max throughput. */
#define KLINE_POLL_GAP_MS   5
#define KLINE_O2_EVERY      1
#define KLINE_O2_TIMEOUT_MS 200
#define KLINE_VERBOSE       0

volatile kline_data_t kline_data = {};

/* ── helpers ─────────────────────────────────────────────────────────────── */

static uint8_t kline_checksum(const uint8_t *buf, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += buf[i];
    return (uint8_t)(0x100u - (sum & 0xFFu));
}

static bool kline_recv(uint8_t *buf, size_t len, uint32_t timeout_ms) {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    size_t got = 0;
    static uint32_t last_feed = 0;   // this task's only caller of wdt_feed()
    while (got < len) {
        // Time-boxed, unconditional: a BUSY RX line (continuous noise with RX
        // floating) keeps `available()` true forever, so the fast path below
        // never falls into the old "waiting" branch where the feed used to
        // live — that's exactly what still starved this task into a watchdog
        // panic after the first fix. Checking here, every iteration
        // regardless of which path is taken, closes that gap. esp_task_wdt_reset()
        // touches a lock shared across cores, so keep this throttled (once
        // every 500ms is still nowhere near the 8s timeout) rather than
        // calling it every iteration.
        uint32_t now = millis();
        if (now - last_feed >= 500) { wdt_feed(); last_feed = now; }

        /* Drain what's already buffered BEFORE checking the deadline — if this
         * task was preempted past the timeout, the bytes are sitting in the RX
         * buffer and must not be reported as a timeout. */
        if (KLINE_SERIAL.available()) {
            buf[got++] = (uint8_t)KLINE_SERIAL.read();
        } else {
            if (xTaskGetTickCount() > deadline) return false;
            vTaskDelay(1);
        }
    }
    return true;
}

/* Drops whatever's currently sitting in the RX buffer before a request, so a
 * stray byte from the previous exchange can't get mistaken for the start of
 * the next one. Bounded in both time and watchdog feeds: with RX floating
 * (nothing wired up) line noise can refill the FIFO faster than a plain
 * `while (available()) read();` can drain it, spinning forever without ever
 * reaching a wdt_feed() call — exactly what starved this task long enough to
 * trip the watchdog. 200ms is far more than any real stray-byte backlog needs;
 * genuine noise just keeps coming regardless of how long this waits. */
#define KLINE_DRAIN_MAX_MS 200

static void kline_drain_stray() {
    uint32_t t0 = millis(), last_feed = t0;
    while (KLINE_SERIAL.available()) {
        KLINE_SERIAL.read();
        uint32_t now = millis();
        if (now - t0 > KLINE_DRAIN_MAX_MS) break;
        if (now - last_feed >= 100) { wdt_feed(); last_feed = now; }
    }
}

/*
 * Send bytes and drain exactly the same number of echo bytes from RX.
 * flush() blocks until TX is complete, so echo bytes are already in the
 * FIFO when we start draining — they get consumed before the ECU has time
 * to respond.  Timeout 50ms is generous for echo but shorter than ECU P2.
 */
static bool kline_drain_tx_echo(size_t tx_bytes) {
    uint8_t echo[16];
    if (tx_bytes > sizeof(echo)) tx_bytes = sizeof(echo);
    if (!kline_recv(echo, tx_bytes, 50)) {
        Serial.printf("[kline] echo timeout (%u bytes)\n", (unsigned)tx_bytes);
        return false;
    }
    Serial.printf("[kline] echo(%u): ", (unsigned)tx_bytes);
    for (size_t i = 0; i < tx_bytes; i++) Serial.printf("%02X ", echo[i]);
    Serial.println();
    return true;
}

/*
 * Receive one response frame: 0x02 [total_len] [payload...] [checksum]
 * total_len covers every byte in the frame including 0x02 and total_len itself.
 * Returns true and fills buf (must be ≥ 64 bytes) with the full frame.
 */
static bool kline_recv_frame(uint8_t *buf, uint8_t *out_len, uint32_t timeout_ms) {
    /* resync: skip stray bytes until we see the 0x02 start marker */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    unsigned skipped = 0;
    while (true) {
        if (xTaskGetTickCount() > deadline) {
            Serial.printf("[kline] frame timeout waiting for 0x02 (skipped %u)\n", skipped);
            return false;
        }
        if (!kline_recv(buf, 1, timeout_ms)) {
            Serial.printf("[kline] frame timeout waiting for 0x02 (skipped %u)\n", skipped);
            return false;
        }
        if (buf[0] == 0x02) break;
        Serial.printf("[kline] skip stray %02X\n", buf[0]);
        skipped++;
    }

    if (!kline_recv(buf + 1, 1, timeout_ms)) {
        Serial.println("[kline] frame timeout on len byte");
        return false;
    }

    uint8_t total = buf[1];
    if (total < 4 || total > 63) {
        Serial.printf("[kline] frame bad len: %02X\n", total);
        return false;
    }

    if (!kline_recv(buf + 2, total - 2, timeout_ms)) {
        Serial.printf("[kline] frame timeout on body (want %u bytes)\n", (unsigned)(total - 2));
        return false;
    }

    if (buf[total - 1] != kline_checksum(buf, total - 1)) {
        Serial.printf("[kline] frame cs err: got %02X expected %02X\n",
                      buf[total - 1], kline_checksum(buf, total - 1));
        return false;
    }

    /* dump the full accepted frame so we can see the real table payload */
    Serial.printf("[kline] frame(%u):", (unsigned)total);
    for (uint8_t i = 0; i < total; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();

    *out_len = total;
    return true;
}

/* ── session start / re-arm ────────────────────────────────────────────────
 * Send the start-diagnostic-session command (72 05 00 F0 99) and verify the
 * ECU ack (02 04 00 FA). Assumes the UART is already up. Used both as the last
 * step of the slow init and as a lightweight reconnect when polling stalls —
 * re-arming the session this way skips the ~300ms line-toggle of a full init. */
static bool kline_start_session() {
    kline_drain_stray();

    const uint8_t req[] = {0x72, 0x05, 0x00, 0xF0, 0x99};
    Serial.printf("[kline] tx: 72 05 00 F0 99\n");
    KLINE_SERIAL.write(req, sizeof(req));
    KLINE_SERIAL.flush();
    if (!kline_drain_tx_echo(sizeof(req))) return false;

    const uint8_t exp[] = {0x02, 0x04, 0x00, 0xFA};
    uint8_t resp[4];
    if (!kline_recv(resp, sizeof(resp), 1000)) {
        Serial.println("[kline] timeout on session ack");
        return false;
    }
    Serial.printf("[kline] session ack: %02X %02X %02X %02X  (expect 02 04 00 FA)\n",
                  resp[0], resp[1], resp[2], resp[3]);
    if (resp[3] != kline_checksum(resp, 3)) {
        Serial.printf("[kline] cs err on session ack: got %02X expect %02X\n",
                      resp[3], kline_checksum(resp, 3));
        return false;
    }
    return memcmp(resp, exp, 4) == 0;
}

/* ── slow init ───────────────────────────────────────────────────────────── */

static bool kline_slow_init() {

    KLINE_SERIAL.begin(KLINE_BAUD, SERIAL_8N1, KLINE_RX_PIN, KLINE_TX_PIN);
    
    const uint8_t close_session_kline[]={0x72, 0x05, 0x00, 0xF1, 0x98};
    Serial.printf("[kline] tx: 72 05 00 F1 98\n");
    
    KLINE_SERIAL.write(close_session_kline, sizeof(close_session_kline));
    KLINE_SERIAL.flush();
    
    delay(100);
    KLINE_SERIAL.end();

    pinMode(KLINE_TX_PIN, OUTPUT);

    digitalWrite(KLINE_TX_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(70));

    digitalWrite(KLINE_TX_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(140));

    KLINE_SERIAL.begin(KLINE_BAUD, SERIAL_8N1, KLINE_RX_PIN, KLINE_TX_PIN);
    KLINE_SERIAL.onReceiveError([](hardwareSerial_error_t e) {
        // err 1 (FIFO_OVF): dump whatever is sitting in the RX buffer so we can
        // see what the ECU/line is actually putting on the wire. Throttled by
        // byte-count rather than suppressed, since the bytes themselves are the
        // whole point of looking. Other errors stay throttled to one line/500ms.
        if (e == 1) {
            static uint32_t last_dump = 0;
            uint32_t now = millis();
            if (now - last_dump < 200) return;
            last_dump = now;
            int n = KLINE_SERIAL.available();
            Serial.printf("[kline] UART err 1 (FIFO_OVF) avail=%d: ", n);
            int cap = n > 64 ? 64 : n;
            for (int i = 0; i < cap; i++) {
                int b = KLINE_SERIAL.read();
                if (b < 0) break;
                Serial.printf("%02X ", (uint8_t)b);
            }
            if (n > cap) Serial.printf("...(+%d more)", n - cap);
            Serial.println();
            return;
        }
        static uint32_t last_log = 0;
        uint32_t now = millis();
        if (now - last_log < 500) return;
        last_log = now;
        Serial.printf("[kline] UART err %d\n", (int)e);  // 2=BREAK 4=FRAME 5=PARITY 1=FIFO_OVF 3=BUF_FULL
    });
    vTaskDelay(pdMS_TO_TICKS(5));
    kline_drain_stray();

    /* step 4: send FE 04 72 8C */
    const uint8_t req1[] = {0xFE, 0x04, 0x72, 0x8C};
    Serial.printf("[kline] tx: FE 04 72 8C\n");
    KLINE_SERIAL.write(req1, sizeof(req1));
    KLINE_SERIAL.flush();
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!kline_drain_tx_echo(sizeof(req1))) return false;

    /* step 5: expect 0E 04 72 7C */
    const uint8_t exp1[] = {0x0E, 0x04, 0x72, 0x7C};
    uint8_t resp1[4];
    if (!kline_recv(resp1, sizeof(resp1), 1000)) {
        Serial.println("[kline] timeout resp1");
        return false;
    }
    Serial.printf("[kline] resp1: %02X %02X %02X %02X  (expect 0E 04 72 7C)\n",
                  resp1[0], resp1[1], resp1[2], resp1[3]);
    if (resp1[3] != kline_checksum(resp1, 3)) {
        Serial.printf("[kline] cs err resp1: got %02X expect %02X\n",
                      resp1[3], kline_checksum(resp1, 3));
        return false;
    }
    if (memcmp(resp1, exp1, 4) != 0) return false;

    /* step 6/7: start the diagnostic session (72 05 00 F0 99 → 02 04 00 FA) */
    return kline_start_session();
}

/* ── block read (data tables + DTC share one request shape) ──────────────── */

/*
 * Request a block from the ECU.
 * Frame: 72 05 [cmd] [id] [cs]        cmd 0x71 = data table, 0x73 = DTC
 * Response: 02 [len] [cmd] [id] [data...] [cs]
 * Returns number of data bytes copied into data_buf, or 0 on failure/empty.
 */
static uint8_t kline_read_block(uint8_t cmd, uint8_t id, uint8_t *data_buf,
                                uint8_t data_buf_size, uint32_t timeout_ms,
                                uint8_t *out_frame_len = nullptr) {
    if (out_frame_len) *out_frame_len = 0;

    uint8_t req[5] = {0x72, 0x05, cmd, id, 0x00};
    req[4] = kline_checksum(req, 4);

    kline_drain_stray();
    KLINE_SERIAL.write(req, sizeof(req));
    KLINE_SERIAL.flush();
    if (!kline_drain_tx_echo(sizeof(req))) return 0;

    uint8_t frame[64];
    uint8_t frame_len = 0;
    if (!kline_recv_frame(frame, &frame_len, timeout_ms)) return 0;

    /* frame layout: [0x02][len][cmd][id][data...][checksum] */
    if (frame_len < 5) return 0;
    if (frame[2] != cmd || frame[3] != id) return 0;

    /* frame[1] is the length byte (total frame size); expose it so callers can
     * gate on it (an empty response is len 5: 02 len cmd id cs).  Only set for
     * a frame that answers this request, so 0 always means "no valid reply". */
    if (out_frame_len) *out_frame_len = frame[1];

    uint8_t data_len = frame_len - 5; /* strip 02, len, cmd, id, checksum */
    if (data_len > data_buf_size) data_len = data_buf_size;
    memcpy(data_buf, frame + 4, data_len);
    return data_len;
}

/* read a data table (command 0x71) */
static uint8_t kline_read_table(uint8_t table_id, uint8_t *data_buf, uint8_t data_buf_size,
                                uint32_t timeout_ms = 1000, uint8_t *out_frame_len = nullptr) {
    return kline_read_block(0x71, table_id, data_buf, data_buf_size, timeout_ms, out_frame_len);
}

/* Retry policy for reads on an established session: a missed / garbled /
 * mismatched frame (e.g. bytes lost on a noisy bus) is re-requested up to
 * KLINE_READ_RETRIES more times before the caller treats it as a failure. */
#define KLINE_READ_RETRIES   3
#define KLINE_RETRY_GAP_MS   20   /* let a half-received reply finish before re-asking */

/* kline_read_table() with retries. Success = a frame with payload (len > 5)
 * whose cmd/id matched (data > 0). On a failure the last frame_len is returned
 * so the caller's stall handling still sees it. */
static uint8_t kline_read_table_retry(uint8_t table_id, uint8_t *data_buf, uint8_t data_buf_size,
                                      uint32_t timeout_ms, uint8_t *out_frame_len) {
    uint8_t frame_len = 0, n = 0;
    for (int attempt = 0; attempt <= KLINE_READ_RETRIES; attempt++) {
        if (attempt > 0) {
            Serial.printf("[kline] table %02X read failed, retry %d/%d\n",
                          table_id, attempt, KLINE_READ_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(KLINE_RETRY_GAP_MS));
        }
        frame_len = 0;
        n = kline_read_table(table_id, data_buf, data_buf_size, timeout_ms, &frame_len);
        if (frame_len > 5 && n > 0) break;
    }
    if (out_frame_len) *out_frame_len = frame_len;
    return n;
}

/* DTC commands: 0x73 = stored (in memory), 0x74 = current (present),
 * 0x60 = clear one memory bank (72 05 60 [bank] cs -> 02 05 60 [bank] cs) */
#define KLINE_DTC_STORED   0x73
#define KLINE_DTC_CURRENT  0x74
#define KLINE_DTC_CLEAR    0x60

/* read a DTC table with the given DTC command (0x73 stored / 0x74 current) */
static uint8_t kline_read_dtc(uint8_t dtc_cmd, uint8_t dtc_table, uint8_t *data_buf,
                              uint8_t data_buf_size, uint32_t timeout_ms = 1000) {
    return kline_read_block(dtc_cmd, dtc_table, data_buf, data_buf_size, timeout_ms);
}

/* ── table scanner ───────────────────────────────────────────────────────── */

/* Sweep every table id (0x00..0xFF) once after init and print which ones return
 * data, then stop.  Enabled from the build flag -DKLINE_SCAN=1 (the "kline_scan"
 * PlatformIO env); defaults off so the normal build polls the locked tables. */
#ifndef KLINE_SCAN
#define KLINE_SCAN 0
#endif

/* Read every *known* table once after init (10/11/13/17/20/21/D1), decode +
 * print each, then stop.  Enable with -DKLINE_SCAN_ALL=1. */
#ifndef KLINE_SCAN_ALL
#define KLINE_SCAN_ALL 0
#endif

/* ── scan tunables ───────────────────────────────────────────────────────── */
#define KLINE_SCAN_FIRST     0x00   /* first table id to probe                  */
#define KLINE_SCAN_LAST      0xFF   /* last table id to probe (inclusive)       */
#define KLINE_SCAN_TIMEOUT   600    /* ms to wait for each table's response     */
#define KLINE_SCAN_GAP       20     /* ms gap between requests                  */

/*
 * Probe table ids KLINE_SCAN_FIRST..KLINE_SCAN_LAST once.  For each id we log:
 *   - "TABLE XX -> N:" + bytes   (ECU returned a payload — the ones we want)
 *   - nothing on empty/no-response (the per-frame timeout line still shows why)
 * A larger KLINE_SCAN_TIMEOUT gives slow tables more room to answer at the cost
 * of a longer sweep when many ids are silent.
 */
static void kline_scan_tables() {
    Serial.println("[kline] === table scan start ===");
    uint8_t data[64];
    unsigned hits = 0;
    for (uint16_t id = KLINE_SCAN_FIRST; id <= KLINE_SCAN_LAST; id++) {
        uint8_t n = kline_read_table((uint8_t)id, data, sizeof(data), KLINE_SCAN_TIMEOUT);
        if (n > 0) {
            hits++;
            Serial.printf("[kline] TABLE %02X -> %u:", (unsigned)id, n);
            for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", data[i]);
            Serial.println();
        }
        vTaskDelay(pdMS_TO_TICKS(KLINE_SCAN_GAP));
    }
    Serial.printf("[kline] === table scan done (%u tables with data) ===\n", hits);
}

/* ── DTC scanner ─────────────────────────────────────────────────────────── */

/* Read the DTC tables once after init, print them, then stop (one-shot
 * diagnostic).  Enable with -DKLINE_DTC_SCAN=1.  The normal build already reads
 * DTCs in the background while polling — see kline_dtc_step(). */
#ifndef KLINE_DTC_SCAN
#define KLINE_DTC_SCAN 0
#endif

#define KLINE_DTC_FIRST      0x01   /* first DTC table (this ECU has 01..03)    */
#define KLINE_DTC_LAST       0x03   /* last DTC table (inclusive)               */
#define KLINE_DTC_TIMEOUT    600    /* ms to wait for each DTC table's response */

/*
 * DTC payload D9..D15: D9 is skipped, then up to three (code, sub) pairs, e.g.
 *   02 0C 73 01 | 01 07 02 08 01 0C 01 | 5E  ->  007-2, 008-1, 012-1
 * A pair with code 00 is an empty slot.  Appends new codes to list (no dups).
 */
static void kline_dtc_collect(const uint8_t *d, uint8_t n,
                              live_dtc_t *list, uint8_t *count) {
    for (uint8_t i = 1; i + 1 < n; i += 2) {
        uint8_t code = d[i], sub = d[i + 1];
        if (code == 0) continue;
        bool dup = false;
        for (uint8_t k = 0; k < *count; k++)
            if (list[k].code == code && list[k].sub == sub) { dup = true; break; }
        if (!dup && *count < LIVE_DTC_MAX) list[(*count)++] = { code, sub };
    }
}

/*
 * Read DTC tables KLINE_DTC_FIRST..KLINE_DTC_LAST for one DTC command.
 * Request: 72 05 [cmd] [table] [cs]   Response: 02 [len] [cmd] [table] [data...] [cs]
 * Logs the raw payload for each table so codes can be decoded by hand.
 */
static void kline_scan_dtc_cmd(uint8_t dtc_cmd, const char *label) {
    Serial.printf("[kline] --- DTC scan %s (cmd %02X) ---\n", label, dtc_cmd);
    uint8_t data[64];
    for (uint16_t t = KLINE_DTC_FIRST; t <= KLINE_DTC_LAST; t++) {
        uint8_t n = kline_read_dtc(dtc_cmd, (uint8_t)t, data, sizeof(data), KLINE_DTC_TIMEOUT);
        if (n > 0) {
            Serial.printf("[kline] DTC %s %02X -> %u:", label, (unsigned)t, n);
            for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", data[i]);
            Serial.println();
            live_dtc_t codes[LIVE_DTC_MAX];
            uint8_t count = 0;
            kline_dtc_collect(data, n, codes, &count);
            for (uint8_t i = 0; i < count; i++) {
                const char *txt = dtc_describe(codes[i].code, codes[i].sub);
                Serial.printf("        %03u-%u  %s\n", codes[i].code, codes[i].sub,
                              txt ? txt : "(unknown code)");
            }
        } else {
            Serial.printf("[kline] DTC %s %02X -> (empty / no codes)\n", label, (unsigned)t);
        }
        vTaskDelay(pdMS_TO_TICKS(KLINE_SCAN_GAP));
    }
}

static void kline_scan_dtc() {
    Serial.println("[kline] === DTC scan start ===");
    kline_scan_dtc_cmd(KLINE_DTC_STORED,  "stored");
    kline_scan_dtc_cmd(KLINE_DTC_CURRENT, "current");
    Serial.println("[kline] === DTC scan done ===");
}

/* ── on-demand DTC read + clear (normal build) ───────────────────────────────
 * DTCs are read only while the UI shows a diagnostic screen (live_dtc_active()).
 * A full sweep is 6 requests (0x74 then 0x73, tables 01..03).  To keep the
 * gauges smooth it is spread out: kline_dtc_step() sends ONE request per poll
 * cycle, and a new sweep starts KLINE_DTC_EVERY_MS after the last one ended.
 * The list is published only when all 6 replies arrived, so a dropped frame
 * never makes codes blink off the screen. */
#define KLINE_DTC_EVERY_MS        5000   /* gap between full sweeps            */
#define KLINE_DTC_POLL_TIMEOUT_MS 300    /* per-request timeout while polling  */
#define KLINE_DTC_STEPS           6

typedef struct {
    uint8_t         step;      /* next request, 0..KLINE_DTC_STEPS-1         */
    bool            ok;        /* every request of this sweep got a reply    */
    uint32_t        next_ms;   /* earliest millis() for the next sweep start */
    live_dtc_list_t work;      /* list being assembled                       */
} kline_dtc_sweep_t;

static void kline_dtc_log(const live_dtc_list_t *l) {
    Serial.printf("[kline] DTC current=%u stored=%u\n", l->n_current, l->n_stored);
    for (uint8_t i = 0; i < l->n_current; i++)
        Serial.printf("        NOW %03u-%u\n", l->current[i].code, l->current[i].sub);
    for (uint8_t i = 0; i < l->n_stored; i++)
        Serial.printf("        MEM %03u-%u\n", l->stored[i].code, l->stored[i].sub);
}

static void kline_dtc_step(kline_dtc_sweep_t *s) {
    if (s->step == 0) {
        if ((int32_t)(millis() - s->next_ms) < 0) return;   /* not due yet */
        memset(&s->work, 0, sizeof(s->work));
        s->ok = true;
    }

    const bool    current = s->step < 3;
    const uint8_t cmd     = current ? KLINE_DTC_CURRENT : KLINE_DTC_STORED;
    const uint8_t table   = KLINE_DTC_FIRST + s->step % 3;

    uint8_t data[64];
    uint8_t frame_len = 0;
    uint8_t n = kline_read_block(cmd, table, data, sizeof(data),
                                 KLINE_DTC_POLL_TIMEOUT_MS, &frame_len);
    if (frame_len == 0)
        s->ok = false;                                       /* no / bad reply */
    else if (current)
        kline_dtc_collect(data, n, s->work.current, &s->work.n_current);
    else
        kline_dtc_collect(data, n, s->work.stored, &s->work.n_stored);

    if (++s->step < KLINE_DTC_STEPS) return;

    s->step    = 0;
    s->next_ms = millis() + KLINE_DTC_EVERY_MS;
    if (s->ok) {
        live_dtc_publish(&s->work);
        kline_dtc_log(&s->work);
    } else {
        Serial.println("[kline] DTC sweep incomplete, keeping last list");
    }
}

/* Erase the stored codes: send a single 72 05 60 01 28 frame, wait 3 s, then
 * re-read the stored DTC tables.  Success = ECU acked AND the memory list is
 * empty on the verify sweep.  Any remaining stored code -> FAIL. */
static bool kline_clear_dtc() {
    uint8_t data[8];
    uint8_t frame_len = 0;
    kline_read_block(KLINE_DTC_CLEAR, 0x01, data, sizeof(data), 1000, &frame_len);
    bool acked = frame_len != 0;
    Serial.printf("[kline] DTC clear 60 01 -> %s\n", acked ? "ack" : "no ack");
    if (!acked) return false;

    vTaskDelay(pdMS_TO_TICKS(3000));

    uint8_t buf[64];
    live_dtc_t stored[LIVE_DTC_MAX];
    uint8_t n_stored = 0;
    for (uint8_t t = KLINE_DTC_FIRST; t <= KLINE_DTC_LAST; t++) {
        uint8_t fl = 0;
        uint8_t n = kline_read_block(KLINE_DTC_STORED, t, buf, sizeof(buf),
                                     KLINE_DTC_TIMEOUT, &fl);
        if (fl == 0) {
            Serial.printf("[kline] DTC verify stored %02X -> no reply, FAIL\n", t);
            return false;
        }
        kline_dtc_collect(buf, n, stored, &n_stored);
    }
    Serial.printf("[kline] DTC verify stored count=%u\n", n_stored);
    return n_stored == 0;
}

/* ── table locking ───────────────────────────────────────────────────────── */

/*
 * Probe a group of table ids in order and lock onto the first one whose
 * response frame length byte (frame[1]) is > 5 — i.e. it carries real payload
 * (an empty response is frame[1]==5: 02 len cmd id cs).  Returns the locked
 * table id, or 0 if no table in the group returned data.
 */
static uint8_t kline_lock_from_group(const char *label,
                                     const uint8_t *ids, size_t count) {
    uint8_t data[64];
    for (size_t i = 0; i < count; i++) {
        uint8_t id = ids[i];
        uint8_t frame_len = 0;
        uint8_t n = kline_read_table(id, data, sizeof(data), 1000, &frame_len);
        Serial.printf("[kline] probe %s table %02X -> frame[1]=%u (data=%u)\n",
                      label, id, frame_len, n);
        if (frame_len > 5) {
            Serial.printf("[kline] locked %s on table %02X\n", label, id);
            return id;
        }
        vTaskDelay(pdMS_TO_TICKS(KLINE_SCAN_GAP));
    }
    return 0;
}

/* ── decoders ────────────────────────────────────────────────────────────── *
 * Response payload starts at D9, so in the buffers below data[k] == D(k+9):
 *   D9=data[0] D10=data[1] D11=data[2] D12=data[3] D13=data[4] D14=data[5]
 *   D15=data[6] D16=data[7] D17=data[8] D18=data[9] D19=data[10] D20=data[11]
 *   D21=data[12] D22=data[13] D23=data[14] D24=data[15] D25=data[16] D26=data[17]
 */

/* Tables 10/11/13 share this layout (RPM..SPEED, with IAT + MAP). */
static void kline_decode_engine_std(const uint8_t *d, uint8_t n) {
    if (n < 18) return;
    kline_data.rpm      = (uint16_t)((d[0] << 8) | d[1]);          /* D9,D10          */
    kline_data.tps_v    = d[2] * 5.0f / 256.0f;                    /* D11             */
    kline_data.tps_deg  = d[3] * 0.5f;                             /* D12             */
    kline_data.ect_v    = d[4] * 5.0f / 256.0f;                    /* D13             */
    kline_data.ect_c    = (int16_t)d[5] - 40;                      /* D14             */
    kline_data.iat_v    = d[6] * 5.0f / 256.0f;                    /* D15             */
    kline_data.iat_c    = (int16_t)d[7] - 40;                      /* D16             */
    kline_data.map_v    = d[8] * 5.0f / 256.0f;                    /* D17             */
    kline_data.map_kpa  = d[9];                                    /* D18             */
    kline_data.batt_v   = d[12] * 0.1f;                            /* D21             */
    kline_data.mode     = d[13];                                   /* D22             */
    kline_data.inj_ms   = ((d[14] << 8) | d[15]) / 200.0f;         /* D23,D24         */
    kline_data.igt_btdc = -64.0f + d[16] * 0.5f;                   /* D25             */
    kline_data.speed    = d[17];                                   /* D26             */
}

/* Table 17 layout (RPM..SPEED, with EOT + FAST IDLE instead of IAT/MAP). */
static void kline_decode_engine_17(const uint8_t *d, uint8_t n) {
    if (n < 16) return;
    kline_data.rpm       = (uint16_t)((d[0] << 8) | d[1]);         /* D9,D10          */
    kline_data.tps_v     = d[2] * 5.0f / 256.0f;                   /* D11             */
    kline_data.tps_deg   = d[3] * 0.5f;                            /* D12             */
    kline_data.ect_v     = d[4] * 5.0f / 256.0f;                   /* D13             */
    kline_data.ect_c     = (int16_t)d[5] - 40;                     /* D14             */
    kline_data.eot_v     = d[8] * 5.0f / 256.0f;                   /* D17             */
    kline_data.eot_c     = (int16_t)d[9] - 40;                     /* D18             */
    kline_data.batt_v    = d[10] * 0.1f;                           /* D19             */
    kline_data.inj_ms    = ((d[11] << 8) | d[12]) / 200.0f;        /* D20,D21         */
    kline_data.igt_btdc  = -64.0f + d[13] * 0.5f;                  /* D22             */
    kline_data.fast_idle = (d[14] & 0x80) != 0;                    /* D23 bit7        */
    kline_data.mode      = d[14];                                  /* D23             */
    kline_data.speed     = d[15];                                  /* D24             */
}

/* Tables 20/21 — O2 sensor. */
static void kline_decode_o2(const uint8_t *d, uint8_t n) {
    if (n < 3) return;
    kline_data.o2_v        = d[0] * 5.0f / 256.0f;                 /* D9              */
    kline_data.stft        = d[1] / 128.0f;                        /* D10             */
    kline_data.o2_heater_v = d[2] * 5.0f / 256.0f;                 /* D11             */
}

/* Map decoded kline_data into the shared live_data and publish it (the struct
 * the dashboard reads). */
static void kline_publish_live() {
    live_data_t d = {};
    d.rpm     = kline_data.rpm;
    d.tps     = kline_data.tps_deg;
    d.batt_v  = kline_data.batt_v;
    d.ect_c   = (float)kline_data.ect_c;
    d.iat_c   = (float)kline_data.iat_c;
    d.map_kpa = (uint16_t)kline_data.map_kpa;
    d.af      = o2v_to_afr(kline_data.o2_v);   /* narrowband O2 volt -> AFR */
    d.ign_deg = kline_data.igt_btdc;
    d.inj     = kline_data.inj_ms;
    /* extended decode */
    d.eot_c       = (float)kline_data.eot_c;
    d.speed_kmh   = kline_data.speed;
    d.mode        = kline_data.mode;
    d.fast_idle   = kline_data.fast_idle;
    d.o2_v        = kline_data.o2_v;
    d.stft        = kline_data.stft;
    d.o2_heater_v = kline_data.o2_heater_v;
    d.tps_v       = kline_data.tps_v;
    d.ect_v       = kline_data.ect_v;
    d.iat_v       = kline_data.iat_v;
    d.map_v       = kline_data.map_v;
    d.eot_v       = kline_data.eot_v;
    live_publish(&d);
}

/* ── full known-table scan (one-shot diagnostic) ───────────────────────────── */

/*
 * Read every table documented on the reference sheet once, decode it, and print
 * both the raw payload and the decoded values.  D1 is a status table (switches /
 * fuel pump / cooling fan), decoded inline here since it has no struct fields.
 */
static void kline_scan_all() {
    static const uint8_t KLINE_ALL_TABLES[] =
        {0x10, 0x11, 0x13, 0x17, 0x20, 0x21, 0xD1};

    Serial.println("[kline] === scan all known tables ===");
    uint8_t data[64];

    for (size_t i = 0; i < sizeof(KLINE_ALL_TABLES); i++) {
        uint8_t id = KLINE_ALL_TABLES[i];
        uint8_t frame_len = 0;
        uint8_t n = kline_read_table(id, data, sizeof(data), KLINE_SCAN_TIMEOUT, &frame_len);

        if (frame_len <= 5) {
            Serial.printf("[kline] TABLE %02X -> frame[1]=%u (ECM not found / empty)\n",
                          id, frame_len);
            vTaskDelay(pdMS_TO_TICKS(KLINE_SCAN_GAP));
            continue;
        }

        Serial.printf("[kline] TABLE %02X (%u):", id, n);
        for (uint8_t j = 0; j < n; j++) Serial.printf(" %02X", data[j]);
        Serial.println();

        switch (id) {
        case 0x10: case 0x11: case 0x13:
            kline_decode_engine_std(data, n);
            Serial.printf("        RPM=%u ECT=%d°C TPS=%.1f° IAT=%d°C MAP=%.0fkPa "
                          "BATT=%.1fV INJ=%.2fms IGT=%.1f°BTDC SPD=%ukm/h\n",
                          kline_data.rpm, kline_data.ect_c, kline_data.tps_deg,
                          kline_data.iat_c, kline_data.map_kpa, kline_data.batt_v,
                          kline_data.inj_ms, kline_data.igt_btdc, kline_data.speed);
            break;
        case 0x17:
            kline_decode_engine_17(data, n);
            Serial.printf("        RPM=%u ECT=%d°C TPS=%.1f° EOT=%d°C BATT=%.1fV "
                          "INJ=%.2fms IGT=%.1f°BTDC SPD=%ukm/h IDLE=%d\n",
                          kline_data.rpm, kline_data.ect_c, kline_data.tps_deg,
                          kline_data.eot_c, kline_data.batt_v, kline_data.inj_ms,
                          kline_data.igt_btdc, kline_data.speed, kline_data.fast_idle);
            break;
        case 0x20: case 0x21:
            kline_decode_o2(data, n);
            Serial.printf("        O2=%.2fV STFT=%.2f%% O2HTR=%.2fV\n",
                          kline_data.o2_v, kline_data.stft, kline_data.o2_heater_v);
            break;
        case 0xD1:
            /* D9=data[0] statuses, D13=data[4] fuel pump, D14=data[5] fan */
            if (n >= 6) {
                const char *sw;
                switch (data[0]) {
                    case 0x00: sw = "OnGear";  break;
                    case 0x01: sw = "Netral";  break;
                    case 0x02: sw = "SSS ON";  break;
                    case 0x03: sw = "SSW ON";  break;
                    default:   sw = "?";       break;
                }
                bool pump = (data[4] == 0x01 || data[4] == 0x05);
                bool fan  = (data[5] == 0x01);
                Serial.printf("        SW=%s(%02X) FUELPUMP=%s FAN=%s\n",
                              sw, data[0], pump ? "ON" : "OFF", fan ? "ON" : "OFF");
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(KLINE_SCAN_GAP));
    }
    Serial.println("[kline] === scan all done ===");
}

/* ── main task ───────────────────────────────────────────────────────────── */

static void kline_task(void *arg) {
    (void)arg;
    wdt_add_self();

    while (true) {
        Serial.println("[kline] slow init...");
        if (!kline_slow_init()) {
            kline_data.valid = false;
            live_invalidate();
            live_dtc_invalidate();
            /* no link — a pending clear can't be sent, so fail it now rather
             * than leave the UI waiting */
            if (live_dtc_take_clear_request()) live_dtc_set_clear_state(DTC_CLEAR_FAIL);
            Serial.println("[kline] init failed, retry in 500ms");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        Serial.println("[kline] init OK");

#if KLINE_SCAN || KLINE_SCAN_ALL || KLINE_DTC_SCAN
#if KLINE_SCAN
        kline_scan_tables();
#endif
#if KLINE_SCAN_ALL
        kline_scan_all();
#endif
#if KLINE_DTC_SCAN
        kline_scan_dtc();
#endif
        /* scan is a one-shot diagnostic — idle here so it doesn't repeat */
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
#endif

        /*
         * Two independent table groups, locked the same way (see
         * kline_lock_from_group): pick the first table in each group whose
         * response frame[1] > 5.  Group A = engine data (0x10/0x11/0x13/0x17),
         * group B = the second set (0x20/0x21).
         *
         * Group A is required: if it can't lock, reinit.  Group B is optional
         * and probed only once — if it doesn't lock on the first try we just
         * skip it for good (poll A only); no retry, no reinit.
         */
        static const uint8_t KLINE_GROUP_A[] = {0x10, 0x11, 0x13, 0x17};
        static const uint8_t KLINE_GROUP_B[] = {0x20, 0x21};

        uint8_t table_a = kline_lock_from_group("A", KLINE_GROUP_A, sizeof(KLINE_GROUP_A));
        if (table_a == 0) {
            Serial.println("[kline] group A lock failed, reinit");
            kline_data.valid = false;
            live_invalidate();
            live_dtc_invalidate();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint8_t table_b = kline_lock_from_group("B", KLINE_GROUP_B, sizeof(KLINE_GROUP_B));
        if (table_b == 0)
            Serial.println("[kline] group B not found, skipping (poll A only)");
        kline_data.table_a = table_a;
        kline_data.table_b = table_b;

        /* poll both locked tables forever (reinit if either stops responding) */
        uint32_t poll_i    = 0;
        uint8_t  soft_fail = 0;   /* consecutive stalls recovered via session re-arm */
        uint16_t b_miss    = 0;   /* consecutive O2 (group B) misses — retried, not dropped */
        kline_dtc_sweep_t dtc    = {};
        bool              dtc_on = false;   /* UI wants DTCs (last loop's view) */
        while (true) {
            uint8_t data[64];
            uint8_t frame_len = 0;

            /* ── group A: engine data ────────────────────────────────────── */
            /* missed / garbled / mismatched frames are retried 3x before this
             * counts as a stall (see kline_read_table_retry) */
            uint8_t n = kline_read_table_retry(table_a, data, sizeof(data), 1000, &frame_len);
            if (frame_len <= 5 || n == 0) {
                /* Try a lightweight session re-arm (72 05 00 F0 99) first — the
                 * bus often just needs the session poked, not a full slow init.
                 * Only fall back to full reinit if that fails or keeps stalling. */
                if (soft_fail < 3 && kline_start_session()) {
                    soft_fail++;
                    Serial.printf("[kline] table %02X (A) stalled, session re-armed (%u/3)\n",
                                  table_a, soft_fail);
                    continue;
                }
                Serial.printf("[kline] table %02X (A) stopped responding, full reinit\n", table_a);
                kline_data.valid = false;
                live_invalidate();
                live_dtc_invalidate();
                break;
            }
            soft_fail = 0;   /* good read → reset the soft-recover counter */

#if KLINE_VERBOSE
            Serial.printf("[kline] table %02X (A,%u):", table_a, n);
            for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", data[i]);
            Serial.println();
#endif

            /* tables 10/11/13 share one layout; 17 has EOT + fast-idle instead */
            if (table_a == 0x17) kline_decode_engine_17(data, n);
            else                 kline_decode_engine_std(data, n);

#if KLINE_VERBOSE
            if (table_a == 0x17) {
                Serial.printf("[kline] RPM=%u ECT=%d°C TPS=%.1f° EOT=%d°C BATT=%.1fV "
                              "INJ=%.2fms IGT=%.1f°BTDC SPD=%ukm/h IDLE=%d\n",
                              kline_data.rpm, kline_data.ect_c, kline_data.tps_deg,
                              kline_data.eot_c, kline_data.batt_v, kline_data.inj_ms,
                              kline_data.igt_btdc, kline_data.speed, kline_data.fast_idle);
            } else {
                Serial.printf("[kline] RPM=%u ECT=%d°C TPS=%.1f° IAT=%d°C MAP=%.0fkPa "
                              "BATT=%.1fV INJ=%.2fms IGT=%.1f°BTDC SPD=%ukm/h\n",
                              kline_data.rpm, kline_data.ect_c, kline_data.tps_deg,
                              kline_data.iat_c, kline_data.map_kpa, kline_data.batt_v,
                              kline_data.inj_ms, kline_data.igt_btdc, kline_data.speed);
            }
#endif

            /* ── group B: O2 sensor ───────────────────────────────────────
             * Optional (never triggers a reinit) and read only once per
             * KLINE_O2_EVERY engine polls so the engine gauges keep the bus
             * to themselves the rest of the time. */
            if (table_b != 0 && (poll_i % KLINE_O2_EVERY) == 0) {
                frame_len = 0;
                n = kline_read_table_retry(table_b, data, sizeof(data),
                                           KLINE_O2_TIMEOUT_MS, &frame_len);
                if (frame_len <= 5 || n == 0) {
                    /* B missed this cycle. It locked at init so the sensor is
                     * really there — keep it in rotation and just retry next
                     * O2 cycle (the short timeout above keeps A responsive).
                     * Log only the first miss and the recovery, not every one. */
                    if (b_miss == 0)
                        Serial.printf("[kline] table %02X (B) missed, retrying\n", table_b);
                    b_miss++;
                } else {
                    if (b_miss) {
                        Serial.printf("[kline] table %02X (B) recovered after %u misses\n",
                                      table_b, b_miss);
                        b_miss = 0;
                    }
#if KLINE_VERBOSE
                    Serial.printf("[kline] table %02X (B,%u):", table_b, n);
                    for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", data[i]);
                    Serial.println();
#endif

                    kline_decode_o2(data, n);  /* tables 20 and 21 share this layout */
#if KLINE_VERBOSE
                    Serial.printf("[kline] O2=%.2fV STFT=%.2f%% O2HTR=%.2fV\n",
                                  kline_data.o2_v, kline_data.stft, kline_data.o2_heater_v);
#endif
                }
            }

            /* mark valid and publish to the shared live_data (bumps live_seq) */
            kline_data.valid = true;
            kline_publish_live();

            /* ── DTC: a clear request from the UI wins; otherwise, while a
             * diagnostic screen is open, advance the read by one request ─── */
            bool dtc_want = live_dtc_active();
            if (dtc_want != dtc_on) {
                dtc.step    = 0;          /* opened: read now / closed: drop partial sweep */
                dtc.next_ms = millis();
                dtc_on      = dtc_want;
            }
            if (live_dtc_take_clear_request()) {
                bool ok = kline_clear_dtc();
                live_dtc_set_clear_state(ok ? DTC_CLEAR_OK : DTC_CLEAR_FAIL);
                dtc.step    = 0;          /* re-read right away to show the result */
                dtc.next_ms = millis();
            } else if (dtc_on) {
                kline_dtc_step(&dtc);
            }

            poll_i++;
            vTaskDelay(pdMS_TO_TICKS(KLINE_POLL_GAP_MS));
        }
    }
}

void kline_task_start() {
    xTaskCreatePinnedToCore(kline_task, "kline", 4096, nullptr, 3, nullptr, 1);
}
