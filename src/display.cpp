// =============================================================================
//  display.cpp  —  Shared ST7789V panel + LVGL bring-up
// =============================================================================
#include "display.h"

#include <Arduino.h>
#include <lvgl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>

#include "pins.h"
#include "wdt.h"

// 80 MHz: max practical ST7789V clock. Needs short/clean wiring — if colours
// glitch or tear, drop back toward 20000000UL. Bumped from 20 to 40 MHz to cut
// per-flush DMA time (helps overall FPS); watch for glitches/tearing on real
// hardware and back off if the wiring can't take it.
static constexpr uint32_t LCD_FREQ = 40000000UL;

// Set to 1 to cycle solid R/G/B before LVGL starts (isolates driver vs LVGL).
#define LCD_SELFTEST 0

// Dedicated SPI host (SPI3, DMA) — touch uses SPI2 via Arduino FSPI.
ST7789V tft(SPI3_HOST,
            PIN_LCD_SCK, PIN_LCD_MOSI, PIN_LCD_MISO,
            PIN_LCD_CS,  PIN_LCD_DC,   PIN_LCD_RST, PIN_LCD_BL);

// Two LVGL draw buffers (partial mode) in DMA-capable internal RAM: LVGL renders
// into one while the SPI DMA sends the other. 80 scanlines => a full 320x240
// redraw takes 3 flushes. Cost = 2 x (320*80*2) = 100 KiB of internal RAM.
//
// Tried bumping this to 120 (2 flushes instead of 3) alongside the arc tach's
// canvas (moto_dash.cpp): these two DMA buffers are allocated once at boot and
// never freed, so at 120 lines they permanently claim ~150 KiB of heap before
// the canvas (~90 KiB, allocated lazily the first time the arc gauge is built)
// ever gets a chance to — leaving too little free heap for the canvas alloc to
// reliably succeed. Kept at 80 so the canvas always has room; revisit together
// if the canvas shrinks or LV_MEM_SIZE drops further.
static constexpr uint32_t BUF_LINES = 80;
static constexpr uint32_t BUF_BYTES = LCD_W * BUF_LINES * 2;  // RGB565 = 2 B/px

static uint32_t lv_tick_cb(void) { return millis(); }

// LVGL calls this before reusing a buffer / at the end of a refresh; block
// (yielding the CPU) until the in-flight DMA transfer is done.
static void lv_flush_wait_cb(lv_display_t*) { tft.waitIdle(); }

static void lv_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    LV_UNUSED(disp);   // async flush signals completion via lv_flush_wait_cb
    const uint32_t w = area->x2 - area->x1 + 1;
    const uint32_t h = area->y2 - area->y1 + 1;

    // LVGL renders little-endian RGB565; the panel wants big-endian.
    lv_draw_sw_rgb565_swap(px_map, w * h);

    // Kick off the DMA and return; lv_flush_wait_cb() stands in for
    // lv_display_flush_ready().
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushPixelsAsync(px_map, w * h);
}

void display_lvgl_init() {
    tft.begin(LCD_FREQ, LCD_W * BUF_LINES);
    tft.setRotation(1);              // landscape 320x240 (moto dashboard)
    tft.backlight(false);                // stays off; splash_show_and_fade() turns it on
                                          // once the boot logo is actually on screen
    Serial.println("[OBDKIT] tft.begin done");

    #if LCD_SELFTEST
        // If you see RED -> GREEN -> BLUE here, the driver/wiring is fine and the
        // problem (if any) is in the LVGL glue. If it stays WHITE, it's SPI/init/wiring.
        Serial.println("[OBDKIT] self-test: R/G/B");
        tft.fillScreen(0xF800); delay(600);   // red
        tft.fillScreen(0x07E0); delay(600);   // green
        tft.fillScreen(0x001F); delay(600);   // blue
    #endif
    tft.fillScreen(0x0000);

    lv_init();
    lv_tick_set_cb(lv_tick_cb);

    lv_display_t* disp = lv_display_create(LCD_W, LCD_H);
    lv_display_set_flush_cb(disp, lv_flush_cb);
    lv_display_set_flush_wait_cb(disp, lv_flush_wait_cb);

    void* buf1 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void* buf2 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!buf1 || !buf2) Serial.println("[OBDKIT] LVGL DMA buffer alloc FAILED");
    Serial.printf("[OBDKIT] LVGL buffers 2x%u B, free internal heap %u B\n",
                  (unsigned)BUF_BYTES,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    lv_display_set_buffers(disp, buf1, buf2, BUF_BYTES,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
}

// -----------------------------------------------------------------------------
//  Dedicated LVGL task (core 1, prio 2)
//
//  Runs lv_timer_handler() on its own task at prio 2, pinned to core 0 so it
//  stays off the K-line core (core 1, prio 3) and ECU timing is never disturbed.
// -----------------------------------------------------------------------------
static SemaphoreHandle_t s_lv_mutex = nullptr;

void lvgl_lock()   { if (s_lv_mutex) xSemaphoreTakeRecursive(s_lv_mutex, portMAX_DELAY); }
void lvgl_unlock() { if (s_lv_mutex) xSemaphoreGiveRecursive(s_lv_mutex); }

// Stack high-water diagnostic. Set to 0 once the stacks are right-sized.
// Prints, for every task, the SMALLEST free stack it has ever had (the peak
// usage) — so you can cut each xTaskCreate... size down to (peak + margin)
// safely instead of guessing. Run the firmware, exercise it (open the dashboard,
// settings menu), then read the lowest "free" numbers.
#ifndef DIAG_STACKS
#define DIAG_STACKS 1
#endif

#if DIAG_STACKS
static void diag_log_task_stacks() {
    UBaseType_t n = uxTaskGetNumberOfTasks();
    TaskStatus_t *st = (TaskStatus_t *)malloc(n * sizeof(TaskStatus_t));
    if (!st) return;
    n = uxTaskGetSystemState(st, n, nullptr);
    Serial.println("[stack] task          free-min");
    for (UBaseType_t i = 0; i < n; i++)
        Serial.printf("[stack] %-14s %5u B\n",
                      st[i].pcTaskName,
                      (unsigned)(st[i].usStackHighWaterMark * sizeof(StackType_t)));
    Serial.printf("[stack] free internal heap %u B, largest DMA block %u B\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    free(st);
}
#endif

static void lvgl_task(void*) {
    wdt_add_self();
    uint32_t last_feed = 0;
#if DIAG_STACKS
    uint32_t last_diag = 0;
#endif
    for (;;) {
        // esp_task_wdt_reset() takes an internal lock and isn't free — this
        // loop runs every 2-10ms, so feeding it every iteration (100-500x/s)
        // was stealing enough CPU time from rendering to visibly drop FPS.
        // Once a second is still nowhere near the 8s timeout.
        uint32_t now0 = millis();
        if (now0 - last_feed >= 1000) { wdt_feed(); last_feed = now0; }
        lvgl_lock();
        uint32_t next_ms = lv_timer_handler();   // ms until the next due timer
        lvgl_unlock();
#if DIAG_STACKS
        uint32_t now = millis();
        if (now - last_diag > 15000) { last_diag = now; diag_log_task_stacks(); }
#endif
        // Clamp: keep touch/animation responsive but don't busy-spin.
        if (next_ms > 10) next_ms = 10;
        if (next_ms < 2)  next_ms = 2;
        vTaskDelay(pdMS_TO_TICKS(next_ms));
    }
}

void display_lvgl_start_task() {
    if (s_lv_mutex) return;                        // already started
    s_lv_mutex = xSemaphoreCreateRecursiveMutex();
    // Stack 8 KiB: measured peak ~5.5 KiB (sw-rendering the 48px font + settings
    // menu), so 8 KiB keeps ~2.5 KiB headroom. prio 2, core 0 (last args); see
    // note above.
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 8 * 1024, nullptr, 2, nullptr, 0);
}
