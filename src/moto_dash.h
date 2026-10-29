// =============================================================================
//  moto_dash.h  —  "Modern Retro" motorcycle dashboard UI (LVGL 9.2)
// =============================================================================
//  Variant 1b from the Claude Design "Moto Dashboard" project:
//    - 36-segment stepped bar-graph tachometer with white/yellow/red zones
//    - large LCD-style RPM readout with a dim "ghost" backdrop
//    - three user-configurable data slots (TEMP / TPS / O2 / BATT / IAT / MAP)
//    - shift-light indicator + full-screen warning border (overheat / low batt)
//    - long-press (~0.6 s) Settings menu: RPM limits, temp alert, slot assign,
//      shift light — all persisted to NVS.
//
//  Targets a 320x240 landscape screen (display_lvgl_init() sets rotation 1).
//  Reads telemetry straight from the global `live_data` (see live_data.h) via an
//  internal lv_timer, so the caller only has to build it once.
//
//  Usage (in setup(), AFTER display_lvgl_init() + touch_init()):
//      moto_dash_create();
// =============================================================================
#pragma once

// Build the dashboard on the active LVGL screen, load saved settings from NVS,
// and start the refresh timer. Call once after the display and touch indev are
// up (the long-press gesture needs the pointer indev to exist).
void moto_dash_create(void);
