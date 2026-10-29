// =============================================================================
//  touch.h  —  XPT2046 touch: 3-point affine calibration + LVGL input device
// =============================================================================
//  Flow:
//    touch_init(&tft) once after lv_init()/lv_display_create().
//      - brings up the XPT2046 on its own SPI bus (see pins.h)
//      - loads affine coefficients from NVS; if none are stored it runs the
//        interactive on-screen calibration first
//      - registers an LVGL pointer indev that maps raw touch -> screen pixels
//
//  Recalibrate by calling touch_calibrate(&tft), or by running the `touch_cal`
//  build (which forces it). IO0/BOOT can't be used as a trigger — it's a
//  strapping pin, so holding it through reset enters the download bootloader.
// =============================================================================
#pragma once

#include "ST7789V.h"

struct _lv_indev_t;   // fwd-decl so callers don't need the whole lvgl header

// Full bring-up: XPT2046 + load/auto-calibrate + register LVGL indev.
// force=true always runs the interactive calibration (ignores stored NVS data).
void touch_init(ST7789V* tft, bool force = false);

// The registered LVGL pointer indev (e.g. to attach a cursor). null before init.
_lv_indev_t* touch_get_indev();

// Interactive on-screen 3-point calibration. Draws targets, samples taps,
// solves the affine map, persists it to NVS. Blocking (expects to run in
// setup(), not the LVGL loop). Returns true on success.
bool touch_calibrate(ST7789V* tft);

// Raw 12-bit reading for debugging/bring-up. Returns true when touched.
bool touch_read_raw(uint16_t& x, uint16_t& y, uint16_t& z);

// Bring-up probe: prints IRQ idle state + N raw samples to Serial.
// Call BEFORE touch_init() (it brings the SPI bus up on its own). Use to
// diagnose "calibration completes without touching" — see interpretation
// hints in the implementation.
void touch_diag(uint8_t samples = 20, uint16_t sample_ms = 100);

// True once a valid calibration is loaded/computed.
bool touch_is_calibrated();
