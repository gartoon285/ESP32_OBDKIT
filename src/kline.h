#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* ── group A: engine data (tables 10/11/13/17) ───────────────────────── */
    uint16_t rpm;          // RPM               (D9*256)+D10
    float    tps_v;        // throttle voltage  D11*5/256   [V]
    float    tps_deg;      // throttle angle    D12*0.5      [°]
    float    ect_v;        // coolant volt      D13*5/256   [V]
    int16_t  ect_c;        // coolant temp      D14-40      [°C]
    float    iat_v;        // intake-air volt   D15*5/256   [V]   (10/11/13)
    int16_t  iat_c;        // intake-air temp   D16-40      [°C]  (10/11/13)
    float    map_v;        // MAP voltage       D17*5/256   [V]   (10/11/13)
    float    map_kpa;      // MAP pressure      D18         [kPa] (10/11/13)
    float    eot_v;        // oil-temp volt     D17*5/256   [V]   (17)
    int16_t  eot_c;        // oil temp          D18-40      [°C]  (17)
    float    batt_v;       // battery           D21*0.1 (10/11/13) / D19*0.1 (17) [V]
    uint8_t  mode;         // MODE              D22 (10/11/13) / D23 (17)
    float    inj_ms;       // injection time    (Dhi*256+Dlo)/200 [ms]
    float    igt_btdc;     // ignition advance  -64+(Dx*0.5) [°BTDC]
    uint8_t  speed;        // vehicle speed     D26 (10/11/13) / D24 (17) [km/h]
    bool     fast_idle;    // fast idle on      D23 & 0x80 (17)

    /* ── group B: O2 sensor (tables 20/21) ───────────────────────────────── */
    float    o2_v;         // O2 sensor voltage D9*5/256   [V]
    float    stft;         // short-term fuel trim D10/128 [%]
    float    o2_heater_v;  // O2 heater voltage D11*5/256  [V]

    uint8_t  table_a;      // locked table id for group A (0 if none)
    uint8_t  table_b;      // locked table id for group B (0 if none)
    bool     valid;
} kline_data_t;

extern volatile kline_data_t kline_data;

void kline_task_start();
