// =============================================================================
//  lv_conf.h  —  LVGL 9.2 configuration for ESP32_OBDKIT
//  Included because build_flags define -DLV_CONF_INCLUDE_SIMPLE.
//  Only the settings that differ from LVGL's built-in defaults are listed;
//  everything else falls back to lv_conf_internal.h.
// =============================================================================
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*----------------
 * Color / rendering
 *----------------*/
#define LV_COLOR_DEPTH 16          // RGB565, matches ST7789V COLMOD 0x55

/*----------------
 * Memory
 *----------------*/
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_STRING    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN
// Bumped from 64K (see history below), then trimmed back from 160K to 128K:
// the arc tach's 56 rotated dial segments — the thing that had originally
// pushed this pool's steady-state usage close to 64K — are gone now (replaced
// by one canvas widget, drawn into a buffer that lives in the system heap
// outside this pool; see arc_canvas in moto_dash.cpp). 128K keeps the same
// safety margin that fixed the original crash while giving some of the
// headroom back to the bigger LVGL display draw buffers (BUF_LINES).
//
// Original bump rationale: with 64K, opening the settings menu (its dim
// overlay needs scratch space to composite) could have nothing left for a
// draw-mask allocation; LVGL doesn't null-check that failure, so it crashed
// (LoadProhibited) instead of just skipping a frame.
#define LV_MEM_SIZE             (128U * 1024U)  // LVGL internal heap (moto dash + settings menu)

/*----------------
 * OS / tick
 * We feed the tick from millis() via lv_tick_set_cb() in main.cpp.
 *----------------*/
#define LV_USE_OS          LV_OS_NONE
#define LV_DEF_REFR_PERIOD 16          // ~60 FPS refresh
#define LV_DPI_DEF         130

/*----------------
 * Logging (off to save flash; bump if debugging LVGL)
 *----------------*/
#define LV_USE_LOG 0

/*----------------
 * Perf monitor (FPS / CPU% overlay, bottom-right). Set both to 0 for release.
 *----------------*/
#define LV_USE_SYSMON        1
#define LV_USE_PERF_MONITOR  1
#define LV_USE_PERF_MONITOR_POS LV_ALIGN_BOTTOM_RIGHT

/*----------------
 * Fonts
 *----------------*/
// The moto dashboard uses its own converted fonts (Chakra Petch / Share Tech
// Mono) in src/ui_fonts/. Montserrat 14 stays as the LVGL default/fallback and
// 28 is used by the touch_cal bring-up screen.
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_DEFAULT       &lv_font_montserrat_14

/*----------------
 * Widgets / themes (defaults are already on; listed for clarity)
 *----------------*/
#define LV_USE_LABEL   1
#define LV_USE_THEME_DEFAULT 1

#endif /* LV_CONF_H */
