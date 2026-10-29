#include "live_data.h"
#include <string.h>
#include <freertos/FreeRTOS.h>

volatile live_data_t live_data = {};
volatile uint32_t     live_seq  = 0;

void live_publish(const live_data_t *d) {
    memcpy((void *)&live_data, d, sizeof(live_data_t));
    live_data.valid = true;
    live_seq = live_seq + 1;   /* fresh sample */
}

void live_invalidate(void) {
    live_data.valid = false;
}

/* ── DTC list ──────────────────────────────────────────────────────────────── */

static live_dtc_list_t s_dtc     = {};
static portMUX_TYPE    s_dtc_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool   s_dtc_clear_req = false;
volatile uint32_t      live_dtc_seq    = 0;

void live_dtc_publish(const live_dtc_list_t *l) {
    portENTER_CRITICAL(&s_dtc_mux);
    uint8_t cs = s_dtc.clear_state;
    s_dtc = *l;
    s_dtc.valid = true;
    s_dtc.clear_state = cs;
    live_dtc_seq = live_dtc_seq + 1;
    portEXIT_CRITICAL(&s_dtc_mux);
}

void live_dtc_invalidate(void) {
    portENTER_CRITICAL(&s_dtc_mux);
    if (s_dtc.valid) {
        s_dtc.valid = false;
        live_dtc_seq = live_dtc_seq + 1;
    }
    portEXIT_CRITICAL(&s_dtc_mux);
}

void live_dtc_set_clear_state(dtc_clear_state_t s) {
    portENTER_CRITICAL(&s_dtc_mux);
    s_dtc.clear_state = (uint8_t)s;
    live_dtc_seq = live_dtc_seq + 1;
    portEXIT_CRITICAL(&s_dtc_mux);
}

bool live_dtc_take_clear_request(void) {
    if (!s_dtc_clear_req) return false;
    s_dtc_clear_req = false;
    return true;
}

void live_dtc_get(live_dtc_list_t *out) {
    portENTER_CRITICAL(&s_dtc_mux);
    *out = s_dtc;
    portEXIT_CRITICAL(&s_dtc_mux);
}

void live_dtc_request_clear(void) {
    live_dtc_set_clear_state(DTC_CLEAR_PENDING);
    s_dtc_clear_req = true;
}

static volatile bool s_dtc_active = false;

void live_dtc_set_active(bool on) {
    if (on && !s_dtc_active) live_dtc_invalidate();
    s_dtc_active = on;
}

bool live_dtc_active(void) {
    return s_dtc_active;
}

/* Honda PGM-FI codes from the "Protokol Diagnostik Honda" sheet. Append new
 * rows freely — lookup is a linear scan. */
static const struct { uint8_t code, sub; const char *text; } DTC_TABLE[] = {
    {  1, 1, "MAP sensor circuit low voltage" },
    {  1, 2, "MAP sensor circuit high voltage" },
    {  2, 1, "MAP sensor performance problem" },
    {  6, 1, "Barometric sensor circuit low voltage" },
    {  6, 2, "Barometric sensor circuit high voltage" },
    {  7, 1, "ECT sensor circuit low voltage" },
    {  7, 2, "ECT sensor circuit high voltage" },
    {  8, 1, "TP sensor circuit low voltage" },
    {  8, 2, "TP sensor circuit high voltage" },
    {  9, 1, "IAT sensor circuit low voltage" },
    {  9, 2, "IAT sensor circuit high voltage" },
    { 11, 1, "VS sensor no signal" },
    { 12, 1, "No.1 primary injector circuit" },
    { 13, 1, "No.2 primary injector circuit" },
    { 14, 1, "No.3 primary injector circuit" },
    { 15, 1, "No.4 primary injector circuit" },
    { 16, 1, "No.1 secondary injector circuit" },
    { 17, 1, "No.2 secondary injector circuit" },
    { 18, 1, "CMP sensor no signal" },
    { 19, 1, "CKP sensor no signal" },
    { 21, 1, "O2 sensor bank 1 low voltage" },
    { 21, 2, "O2 sensor bank 1 high voltage" },
    { 22, 1, "O2 sensor bank 2 low voltage" },
    { 22, 2, "O2 sensor bank 2 high voltage" },
};

const char *dtc_describe(uint8_t code, uint8_t sub) {
    for (size_t i = 0; i < sizeof(DTC_TABLE) / sizeof(DTC_TABLE[0]); i++)
        if (DTC_TABLE[i].code == code && DTC_TABLE[i].sub == sub) return DTC_TABLE[i].text;
    return nullptr;
}

/* Volt->AFR characteristic (user-supplied narrowband curve). Voltages are
 * strictly increasing; duplicate-voltage rows from the source table were
 * collapsed to their mean AFR so the curve is a proper 1-D function. */
float o2v_to_afr(float v) {
    static const float V[] = {
        0.00f, 0.01f, 0.02f, 0.03f, 0.05f, 0.06f, 0.11f, 0.18f, 0.30f, 0.40f,
        0.50f, 0.61f, 0.68f, 0.72f, 0.74f, 0.76f, 0.78f, 0.81f, 0.82f, 0.84f,
        0.85f, 0.90f, 0.95f, 1.00f
    };
    static const float A[] = {
        20.00f, 19.44f, 18.70f, 17.90f, 17.20f, 16.50f, 15.80f, 15.20f, 15.00f, 14.80f,
        14.50f, 14.30f, 14.20f, 13.80f, 13.60f, 13.40f, 13.20f, 12.80f, 12.60f, 12.30f,
        12.05f, 11.26f, 10.63f, 10.00f
    };
    const int N = (int)(sizeof(V) / sizeof(V[0]));
    if (v <= V[0])     return A[0];
    if (v >= V[N - 1]) return A[N - 1];
    for (int i = 1; i < N; i++) {
        if (v <= V[i]) {
            float t = (v - V[i - 1]) / (V[i] - V[i - 1]);
            return A[i - 1] + t * (A[i] - A[i - 1]);
        }
    }
    return A[N - 1];
}
