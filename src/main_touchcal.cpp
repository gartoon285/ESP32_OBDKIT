// =============================================================================
//  main_touchcal.cpp  —  ESP32_OBDKIT screen/touch bring-up firmware
//
//  Display + LVGL + a FORCED touch calibration on every boot, followed by a
//  live accuracy test screen. No K-line, no WiFi — this build exists purely to
//  bring up and verify the panel + resistive touch in isolation.
//
//  Build/upload with `-e touch_cal`. Press RESET (EN only — never hold BOOT/IO0,
//  that's a strap pin -> download mode) to recalibrate.
//
//  Compiled instead of main.cpp via build_src_filter in platformio.ini.
// =============================================================================
#include <Arduino.h>
#include <lvgl.h>

#include "display.h"
#include "touch.h"
#include "wdt.h"

static lv_obj_t* s_coord_label = nullptr;
static lv_obj_t* s_cursor      = nullptr;

static void build_touch_test_ui(void) {
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x05161F), LV_PART_MAIN);

    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, "TOUCH TEST");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t* hint = lv_label_create(scr);
    lv_label_set_text(hint, "Tap anywhere.\nPress RESET to\nrecalibrate.");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x6FB1FF), LV_PART_MAIN);
    lv_obj_set_style_text_align(hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 52);

    s_coord_label = lv_label_create(scr);
    lv_label_set_text(s_coord_label, "x: ---  y: ---");
    lv_obj_set_style_text_color(s_coord_label, lv_color_hex(0x00E676), LV_PART_MAIN);
    lv_obj_align(s_coord_label, LV_ALIGN_BOTTOM_MID, 0, -12);

    // A red dot that tracks the (calibrated) touch point.
    s_cursor = lv_obj_create(scr);
    lv_obj_set_size(s_cursor, 14, 14);
    lv_obj_set_style_radius(s_cursor, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_cursor, lv_color_hex(0xFF1744), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_cursor, 0, LV_PART_MAIN);
    lv_obj_add_flag(s_cursor, LV_OBJ_FLAG_HIDDEN);
}

// Poll the indev each tick: show the dot + coordinates while pressed.
static void test_poll_cb(lv_timer_t* t) {
    LV_UNUSED(t);
    lv_indev_t* indev = touch_get_indev();
    if (!indev) return;

    lv_point_t p;
    lv_indev_get_point(indev, &p);
    if (lv_indev_get_state(indev) == LV_INDEV_STATE_PRESSED) {
        lv_obj_clear_flag(s_cursor, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_cursor, p.x - 7, p.y - 7);
        lv_label_set_text_fmt(s_coord_label, "x: %d  y: %d", (int)p.x, (int)p.y);
    } else {
        lv_obj_add_flag(s_cursor, LV_OBJ_FLAG_HIDDEN);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[CAL] touch-calibration build");

    wdt_init();

    display_lvgl_init();
    build_touch_test_ui();

    // Hardware probe before calibration: catches a stuck IRQ or dead SPI
    // (which otherwise masquerades as "calibration finishes with one tap").
    touch_diag();

    // Always run the interactive calibration in this build, then test it live.
    touch_init(&tft, /*force=*/true);

    lv_timer_create(test_poll_cb, 30, nullptr);
    Serial.println("[CAL] ready — tap the screen to verify accuracy");
}

void loop() {
    wdt_feed();
    lv_timer_handler();
    delay(5);
}
