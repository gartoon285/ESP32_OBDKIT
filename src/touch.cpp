// =============================================================================
//  touch.cpp  —  XPT2046 touch: 3-point affine calibration + LVGL indev
// =============================================================================
#include "touch.h"

#include <lvgl.h>
#include <Preferences.h>

#include "pins.h"
#include "XPT2046.h"
#include "wdt.h"

// -----------------------------------------------------------------------------
//  Hardware: XPT2046 on its own SPI bus (FSPI) so it never fights the ST7789V
//  display bus (HSPI). Low clock — resistive panels are happy at ~2 MHz.
// -----------------------------------------------------------------------------
static SPIClass touchSPI(FSPI);
static XPT2046  touch(touchSPI,
                      PIN_TOUCH_CLK, PIN_TOUCH_DIN, PIN_TOUCH_DO,
                      PIN_TOUCH_CS,  PIN_TOUCH_IRQ);

static ST7789V* s_tft = nullptr;

// -----------------------------------------------------------------------------
//  Affine map  (raw touch -> screen pixels):
//      sx = A*rx + B*ry + C
//      sy = D*rx + E*ry + F
//  Captures scale, rotation and skew in one go, so it stays correct regardless
//  of panel orientation as long as calibration & use share the same rotation.
// -----------------------------------------------------------------------------
struct Cal { float A, B, C, D, E, F; };
static Cal  s_cal = {0, 0, 0, 0, 0, 0};
static bool s_calibrated = false;

static constexpr char NVS_NS[]  = "touchcal";
static constexpr char NVS_KEY[] = "affine";
static constexpr uint32_t CAL_MAGIC = 0x54434131;  // "TCA1"

struct CalBlob { uint32_t magic; Cal cal; };

// -----------------------------------------------------------------------------
//  Persistence
// -----------------------------------------------------------------------------
static bool cal_load() {
    Preferences p;
    if (!p.begin(NVS_NS, /*readOnly=*/true)) return false;
    CalBlob b{};
    size_t n = p.getBytes(NVS_KEY, &b, sizeof(b));
    p.end();
    if (n != sizeof(b) || b.magic != CAL_MAGIC) return false;
    s_cal = b.cal;
    s_calibrated = true;
    return true;
}

static void cal_save(const Cal& c) {
    Preferences p;
    if (!p.begin(NVS_NS, /*readOnly=*/false)) {
        Serial.println("[touch] NVS open failed — calibration not saved");
        return;
    }
    CalBlob b{CAL_MAGIC, c};
    p.putBytes(NVS_KEY, &b, sizeof(b));
    p.end();
    Serial.println("[touch] calibration saved to NVS");
}

// -----------------------------------------------------------------------------
//  Tiny direct-to-panel drawing (calibration UI runs before the LVGL loop).
// -----------------------------------------------------------------------------
static void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (!s_tft) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (w <= 0 || h <= 0) return;
    if (x + w > s_tft->width())  w = s_tft->width()  - x;
    if (y + h > s_tft->height()) h = s_tft->height() - y;
    if (w <= 0 || h <= 0) return;

    uint16_t be = (color >> 8) | (color << 8);          // panel wants big-endian
    static uint16_t row[320];
    for (int16_t i = 0; i < w && i < 320; i++) row[i] = be;
    s_tft->setAddrWindow(x, y, w, h);
    for (int16_t r = 0; r < h; r++) s_tft->pushPixels(row, w);
}

static void draw_target(int16_t cx, int16_t cy, uint16_t color) {
    const int16_t arm = 12, th = 2;
    fill_rect(cx - arm, cy - th / 2, arm * 2, th, color);   // horizontal
    fill_rect(cx - th / 2, cy - arm, th, arm * 2, color);   // vertical
    fill_rect(cx - 2, cy - 2, 4, 4, color);                 // centre dot
}

// -----------------------------------------------------------------------------
//  Blocking: wait for a clean press, average while held, wait for release.
// -----------------------------------------------------------------------------
static bool sample_tap(uint16_t& rx, uint16_t& ry) {
    const uint32_t STABLE_MS = 60;
    uint32_t t0 = millis();

    // Wait for pen-down (timeout 20 s so a wedged panel can't hang boot).
    // This can legitimately run close to that long waiting on a person to
    // tap — feed the watchdog every iteration so it doesn't get mistaken
    // for a hang (see wdt.h).
    while (!touch.touched()) {
        if (millis() - t0 > 20000) return false;
        wdt_feed();
        delay(5);
    }

    // Average readings while the touch stays stable.
    uint32_t sx = 0, sy = 0; uint16_t n = 0;
    uint32_t hold = millis();
    while (millis() - hold < STABLE_MS) {
        uint16_t x, y, z;
        if (touch.read(x, y, z)) { sx += x; sy += y; n++; }
        wdt_feed();
        delay(4);
    }
    if (n < 4) return false;
    rx = sx / n;
    ry = sy / n;

    // Wait for release so taps don't bleed into the next target.
    while (touch.touched()) { wdt_feed(); delay(5); }
    delay(120);   // debounce
    return true;
}

// -----------------------------------------------------------------------------
//  Solve the affine coefficients from 3 screen points + 3 raw samples.
//  (TI "Calibration in touch-screen systems", SLYT277.)
// -----------------------------------------------------------------------------
static bool solve_affine(const int16_t sx[3], const int16_t sy[3],
                         const uint16_t tx[3], const uint16_t ty[3], Cal& out) {
    float dx1 = tx[0] - tx[2], dx2 = tx[1] - tx[2];
    float dy1 = ty[0] - ty[2], dy2 = ty[1] - ty[2];
    float det = dx1 * dy2 - dx2 * dy1;
    if (fabsf(det) < 1e-3f) return false;   // collinear taps -> unsolvable

    float sdx1 = sx[0] - sx[2], sdx2 = sx[1] - sx[2];
    float sdy1 = sy[0] - sy[2], sdy2 = sy[1] - sy[2];

    out.A = (sdx1 * dy2 - sdx2 * dy1) / det;
    out.B = (dx1 * sdx2 - dx2 * sdx1) / det;
    out.C = sx[2] - out.A * tx[2] - out.B * ty[2];

    out.D = (sdy1 * dy2 - sdy2 * dy1) / det;
    out.E = (dx1 * sdy2 - dx2 * sdy1) / det;
    out.F = sy[2] - out.D * tx[2] - out.E * ty[2];
    return true;
}

// -----------------------------------------------------------------------------
//  Interactive calibration
// -----------------------------------------------------------------------------
bool touch_calibrate(ST7789V* tft) {
    s_tft = tft;
    const int16_t W = tft->width(), H = tft->height();
    const int16_t M = 28;   // target inset from the edges

    const int16_t sx[3] = { M,       (int16_t)(W - M), (int16_t)(W / 2) };
    const int16_t sy[3] = { M,       (int16_t)(H / 2), (int16_t)(H - M) };
    uint16_t tx[3], ty[3];

    Serial.println("[touch] calibration: tap each target");
    for (uint8_t i = 0; i < 3; i++) {
        tft->fillScreen(0x0000);
        draw_target(sx[i], sy[i], 0xF800);          // red cross
        if (!sample_tap(tx[i], ty[i])) {
            Serial.printf("[touch] point %u timed out — aborting\n", i + 1);
            return false;
        }
        draw_target(sx[i], sy[i], 0x07E0);          // green = captured
        Serial.printf("[touch] P%u screen(%d,%d) raw(%u,%u)\n",
                      i + 1, sx[i], sy[i], tx[i], ty[i]);
        delay(250);
    }

    Cal c;
    if (!solve_affine(sx, sy, tx, ty, c)) {
        Serial.println("[touch] solve failed (collinear taps) — try again");
        return false;
    }
    s_cal = c;
    s_calibrated = true;
    cal_save(c);

    Serial.printf("[touch] A=%.4f B=%.4f C=%.1f  D=%.4f E=%.4f F=%.1f\n",
                  c.A, c.B, c.C, c.D, c.E, c.F);
    tft->fillScreen(0x0000);
    return true;
}

// -----------------------------------------------------------------------------
//  LVGL indev read callback
// -----------------------------------------------------------------------------
static int32_t s_last_x = 0, s_last_y = 0;

static void indev_read_cb(lv_indev_t* indev, lv_indev_data_t* data) {
    LV_UNUSED(indev);
    uint16_t rx, ry, rz;
    if (s_calibrated && touch.read(rx, ry, rz)) {
        int32_t x = (int32_t)lroundf(s_cal.A * rx + s_cal.B * ry + s_cal.C);
        int32_t y = (int32_t)lroundf(s_cal.D * rx + s_cal.E * ry + s_cal.F);

        const int32_t W = s_tft ? s_tft->width()  : 240;
        const int32_t H = s_tft ? s_tft->height() : 320;
        if (x < 0) x = 0; else if (x >= W) x = W - 1;
        if (y < 0) y = 0; else if (y >= H) y = H - 1;

        s_last_x = x; s_last_y = y;
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->point.x = s_last_x;   // keep last point on release (LVGL needs it)
        data->point.y = s_last_y;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

bool touch_read_raw(uint16_t& x, uint16_t& y, uint16_t& z) {
    return touch.read(x, y, z);
}

bool touch_is_calibrated() { return s_calibrated; }

// -----------------------------------------------------------------------------
//  Bring-up probe
//    Interpretation:
//      IRQ idle must be 1 (HIGH). If it reads 0, PENIRQ is stuck LOW —
//      typically a floating pin, missing pull-up, or an unpowered IC — and
//      touched() will fire continuously, which looks like "all 3 calibration
//      taps captured from one touch".
//      Raw x/y that stay identical across samples (and often 0 or 4095) mean
//      SPI isn't talking to the IC — check MISO, CS, VCC.
// -----------------------------------------------------------------------------
void touch_diag(uint8_t samples, uint16_t sample_ms) {
    touch.begin();
    Serial.println("[touch-diag] ---- probe ----");
    Serial.printf("[touch-diag] IRQ idle = %d  (expect 1)\n",
                  digitalRead(PIN_TOUCH_IRQ));

    for (uint8_t i = 0; i < samples; i++) {
        uint16_t x = 0, y = 0, z = 0;
        bool t = touch.read(x, y, z);
        Serial.printf("[touch-diag] %02u  touched=%d  x=%4u  y=%4u  z=%4u  IRQ=%d\n",
                      i, t, x, y, z, digitalRead(PIN_TOUCH_IRQ));
        wdt_feed();
        delay(sample_ms);
    }
    Serial.println("[touch-diag] ---- done ----");
}

// -----------------------------------------------------------------------------
//  Public bring-up
// -----------------------------------------------------------------------------
static lv_indev_t* s_indev = nullptr;

void touch_init(ST7789V* tft, bool force) {
    s_tft = tft;
    touch.begin();

    // NOTE: we deliberately do NOT trigger recalibration from the BOOT/IO0 pin.
    // IO0 is a strapping pin — holding it through a reset drops the chip into
    // the ROM download bootloader, so the firmware never runs. To recalibrate,
    // flash/run the `touch_cal` build (it passes force=true) instead.
    if (force || !cal_load()) {
        Serial.println(force ? "[touch] forced recalibration"
                             : "[touch] no stored calibration — calibrating");
        touch_calibrate(tft);
    } else {
        Serial.println("[touch] calibration loaded from NVS");
    }

    s_indev = lv_indev_create();
    lv_indev_set_type(s_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_indev, indev_read_cb);
    Serial.println("[touch] LVGL pointer indev registered");
}

lv_indev_t* touch_get_indev() { return s_indev; }
