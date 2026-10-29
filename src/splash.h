// =============================================================================
//  splash.h  —  Boot splash: TW AutoElec logo, backlight fades in after it's
//  actually on screen, then the caller builds the real UI.
// =============================================================================
#pragma once

#include <stdint.h>

// Draws the logo on the active LVGL screen, forces that frame out to the
// panel (so nothing is visible until the logo is actually there), then ramps
// the backlight 0 -> full over `fade_ms` (blocking). Leaves the screen empty
// (lv_obj_clean'd) afterwards so the caller can build the real UI on it.
// Call after display_lvgl_init() and before display_lvgl_start_task() (LVGL
// isn't thread-safe, and nothing else should be touching it yet at boot).
void splash_show_and_fade(uint32_t fade_ms = 2000);
void splash_init();
