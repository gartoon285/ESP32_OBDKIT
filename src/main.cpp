// =============================================================================
//  main.cpp  —  ESP32_OBDKIT V1 normal firmware
//  ST7789V + LVGL 9.2 UI + touch + K-line ECU polling.
//
//  The screen/touch bring-up build lives in main_touchcal.cpp (env:touch_cal);
//  the two mains are mutually exclusive via build_src_filter in platformio.ini.
// =============================================================================
#include <Arduino.h>
#include <lvgl.h>

#include "display.h"
#include "touch.h"
#include "kline.h"
#include "moto_dash.h"
#include "splash.h"
#include "wdt.h"
#include "pins.h"

// Arduino loop task stack. setup() now builds a lot more than it used to —
// both gauge styles' widgets, the hierarchical settings menu (8 pages), and
// the splash screen — so the ~2 KiB high-water mark this was once sized
// against (4 KiB budget) is no longer enough and trips the stack-canary
// panic partway through setup(). 12 KiB gives real headroom; loop() still
// just parks, so this only matters while setup() runs.
SET_LOOP_TASK_STACK_SIZE(12288);

void setup() {
    // Kill backlight ASAP: GPIO17 floats on reset, and AP3019's CTRL reads a
    // floating input as "enable" — so the LED turns on at power-up and shows
    // whatever garbage the panel starts with until tft.begin() runs. Driving
    // LOW here (before Serial, delays, kline task, display init) cuts the
    // visible flash down to the ~20-50 ms the bootloader itself takes.
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, LOW);

    Serial.begin(115200);
    delay(200);
    Serial.println("\n[OBDKIT] boot");

    // Arm the task watchdog before anything that can block for a while (touch
    // calibration, K-line handshakes) gets a chance to wedge. See wdt.h for
    // why one flat timeout is safe here.
    wdt_init();

    kline_task_start();

    display_lvgl_init();

    // Boot splash: TW AutoElec logo, backlight stays off until that frame is
    // actually on the panel, then fades in over 2 s. Runs before the LVGL
    // task starts (nothing else touches LVGL yet, so no locking needed) and
    // leaves the screen clean for moto_dash_create() below.
    splash_show_and_fade(2000);

    // Touch: XPT2046 + LVGL pointer indev. Loads the affine map from NVS; runs
    // the on-screen calibration only if none is stored. To force a recalibration
    // use the `touch_cal` build (IO0/BOOT can't trigger it — it's a strap pin).
    // NOTE: calibration is orientation-specific — after switching the panel to
    // landscape (rotation 1) you must recalibrate once via the `touch_cal` build.
    touch_init(&tft);

    // "Modern Retro" moto dashboard (variant 1b). Built after touch so the
    // long-press-to-open-Settings gesture can tune the pointer indev. Reads
    // telemetry straight from the global live_data via its own refresh timer.
    moto_dash_create();

    // Drive LVGL from a dedicated task instead of loop(), kept off the K-line
    // core so ECU timing stays intact while the UI still refreshes smoothly.
    Serial.println("[OBDKIT] starting LVGL task");
    display_lvgl_start_task();

    // K-line (ISO 9141 / ISO 14230) polling task — TX=IO8, RX=IO9 (see pins.h),
    // pinned to core 1 inside kline_task_start().
}

void loop() {
    // LVGL runs on its own task (display_lvgl_start_task); nothing time-
    // critical lives here, so park the Arduino loop task. This task stays
    // subscribed to the watchdog (see wdt_init() in setup()), so keep feeding it.
    wdt_feed();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
