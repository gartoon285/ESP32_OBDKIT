// =============================================================================
//  display.h  —  Shared ST7789V panel + LVGL bring-up for ESP32_OBDKIT
// =============================================================================
//  Owns the panel object, the LVGL draw buffer and the flush/tick glue so that
//  every firmware "shape" (normal, touch_cal, ...) brings the screen up the
//  same way. Call display_lvgl_init() once, then build your UI on
//  lv_screen_active().
// =============================================================================
#pragma once

#include "ST7789V.h"

// Landscape 320x240 (panel is physically 240x320; rotated in display_lvgl_init).
static constexpr uint16_t LCD_W = 320;
static constexpr uint16_t LCD_H = 240;

// The shared panel instance (also passed to touch_init()).
extern ST7789V tft;

// Panel reset/init + LVGL core (display, buffers, flush/tick callbacks).
void display_lvgl_init();

// Start the dedicated LVGL task (pinned to core 1, priority 2) that drives
// lv_timer_handler(). Call once after the UI has been built. Replaces calling
// lv_timer_handler() from the Arduino loop().
void display_lvgl_start_task();

// LVGL is not thread-safe. The LVGL task holds this recursive lock while it runs
// lv_timer_handler(); any OTHER task that calls lv_*/touches widgets must wrap
// the calls in lvgl_lock()/lvgl_unlock(). (The touch indev read runs inside
// lv_timer_handler on the LVGL task, so it is already covered.) No-op before the
// task is started.
void lvgl_lock();
void lvgl_unlock();
