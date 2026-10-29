// =============================================================================
//  wdt.cpp  —  see wdt.h
// =============================================================================
#include "wdt.h"

#include <Arduino.h>
#include <esp_task_wdt.h>

void wdt_init(uint32_t timeout_ms) {
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms      = timeout_ms;
    cfg.idle_core_mask  = 0;      // we subscribe our own tasks explicitly instead
    cfg.trigger_panic   = true;   // reset the MCU on a real hang, don't just log it

    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        // Arduino core already initialized one (board-default config) — take it over.
        err = esp_task_wdt_reconfigure(&cfg);
    }
    if (err != ESP_OK) {
        Serial.printf("[wdt] init failed: %d\n", (int)err);
        return;
    }
    esp_task_wdt_add(nullptr);
    Serial.printf("[wdt] watchdog armed, %u ms timeout\n", (unsigned)timeout_ms);
}

void wdt_add_self() {
    esp_task_wdt_add(nullptr);
}

void wdt_feed() {
    esp_task_wdt_reset();
}
