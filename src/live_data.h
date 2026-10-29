#pragma once
#include <stdint.h>
#include <stdbool.h>


// #define MOTO_SIM 1
/*
 * live_data — protocol-agnostic live telemetry.
 *
 * This is the single struct the dashboard (moto_dash.cpp) reads.  The K-line
 * task (kline.cpp) maps its rich kline_data_t → live_data with live_publish().
 *
 * Keeping this layer separate means the UI never depends on a particular
 * protocol — adding a source only means adding a mapping that calls
 * live_publish().  kline_data_t stays for the K-line task's own internal use
 * (serial logging, …).
 */
typedef struct {
    bool     valid;
    uint16_t rpm;       /* engine speed            [rpm]                        */
    float    tps;       /* throttle position       [°]                          */
    float    batt_v;    /* battery voltage         [V]                          */
    float    ect_c;     /* coolant temperature     [°C]                         */
    float    iat_c;     /* intake-air temperature  [°C]                         */
    uint16_t map_kpa;   /* manifold pressure       [kPa]                        */
    float    af;        /* air/fuel ratio          (from O2 voltage curve)      */
    float    ign_deg;   /* ignition advance        [°]                          */
    float    inj;       /* injection time          [ms]                         */

    /* ── extended K-line decode (0 when the active protocol/table lacks it) ── */
    float    eot_c;      /* engine-oil temperature  [°C]   (kline table 17)      */
    uint16_t speed_kmh;  /* vehicle speed           [km/h]                       */
    uint8_t  mode;       /* ECU mode byte                                        */
    bool     fast_idle;  /* fast-idle active                                     */
    float    o2_v;       /* O2 sensor voltage       [V]    (kline tables 20/21)  */
    float    stft;       /* short-term fuel trim    [%]                          */
    float    o2_heater_v;/* O2 heater voltage       [V]                          */
    float    tps_v;      /* throttle raw voltage    [V]                          */
    float    ect_v;      /* coolant raw voltage     [V]                          */
    float    iat_v;      /* intake-air raw voltage  [V]                          */
    float    map_v;      /* MAP raw voltage         [V]                          */
    float    eot_v;      /* oil-temp raw voltage    [V]                          */
} live_data_t;

extern volatile live_data_t live_data;

/* Bumped once per fresh sample by the active protocol task, so a reader can
 * tell a new sample from one it has already seen. */
extern volatile uint32_t live_seq;

/* Publish a fresh sample: copies *d into live_data (forcing valid=true) and
 * bumps live_seq. */
void live_publish(const live_data_t *d);

/* Mark the live data stale on link loss.  Does not bump live_seq. */
void live_invalidate(void);

/* ── diagnostic trouble codes ─────────────────────────────────────────────────
 * Published by the protocol task alongside live_data; the dashboard reads it
 * through live_dtc_get() (copied under a lock — the list is too big to snapshot
 * by hand).  A code is "main-sub" as shown in the service manual, e.g. 007-2. */
#define LIVE_DTC_MAX 9   /* 3 tables x 3 codes per DTC command */

typedef struct {
    uint8_t code;   /* main number (decimal on the manual, e.g. 7 -> "007") */
    uint8_t sub;    /* sub number  (e.g. 2 -> "-2")                         */
} live_dtc_t;

typedef enum {
    DTC_CLEAR_IDLE = 0,
    DTC_CLEAR_PENDING,   /* requested by the UI, not yet sent             */
    DTC_CLEAR_OK,        /* ECU acked every memory bank                   */
    DTC_CLEAR_FAIL,      /* no / bad ack (no link, ECU refused, ...)      */
} dtc_clear_state_t;

typedef struct {
    bool       valid;                     /* a full read completed on the current link */
    uint8_t    n_current;                 /* present now        (cmd 0x74) */
    uint8_t    n_stored;                  /* stored in memory   (cmd 0x73) */
    live_dtc_t current[LIVE_DTC_MAX];
    live_dtc_t stored[LIVE_DTC_MAX];
    uint8_t    clear_state;               /* dtc_clear_state_t */
} live_dtc_list_t;

/* Bumped whenever the DTC list or the clear state changes. */
extern volatile uint32_t live_dtc_seq;

/* Protocol side: publish a freshly read list (keeps the current clear_state). */
void live_dtc_publish(const live_dtc_list_t *l);
/* Protocol side: mark the list stale on link loss. */
void live_dtc_invalidate(void);
/* Protocol side: report the outcome of a clear request. */
void live_dtc_set_clear_state(dtc_clear_state_t s);
/* Protocol side: true once per UI clear request (consumes it). */
bool live_dtc_take_clear_request(void);

/* UI side: copy the current list out. */
void live_dtc_get(live_dtc_list_t *out);
/* UI side: ask the protocol task to erase the stored codes. */
void live_dtc_request_clear(void);
/* UI side: DTCs are queried only while a diagnostic screen is showing.  Turning
 * it on drops the old list so the screen shows a fresh read, not a stale one. */
void live_dtc_set_active(bool on);
/* Protocol side: true while the UI wants DTCs queried. */
bool live_dtc_active(void);

/* Short English description for a code (fonts are ASCII-only), or nullptr if
 * the code is not in the table. */
const char *dtc_describe(uint8_t code, uint8_t sub);

/* Narrowband O2 sensor voltage [V] -> AFR, via a piecewise-linear lookup of the
 * sensor's characteristic curve (low V = lean/high AFR, high V = rich/low AFR).
 * Clamps to the table ends (~20 AFR at 0 V, ~10 AFR at 1 V). */
float o2v_to_afr(float o2_voltage);
