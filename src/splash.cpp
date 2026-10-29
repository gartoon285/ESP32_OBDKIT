// =============================================================================
//  splash.cpp  —  Boot splash (see splash.h)
// =============================================================================
#include "splash.h"

#include <Arduino.h>
#include <lvgl.h>

#include "display.h"
#include "assets/splash_logo.h"

void splash_show_and_fade(uint32_t fade_ms) {
    lv_obj_t* scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* img = lv_image_create(scr);
    lv_image_set_src(img, &splash_logo);
    lv_obj_center(img);

    // Synchronous redraw: blocks (via the flush-wait callback) until the DMA
    // push of this frame to the panel's GRAM has actually completed, so the
    // backlight never comes on over a stale/garbage frame.
    lv_refr_now(nullptr);

    const int STEPS = 100;
    for (int i = 0; i <= STEPS; i++) {
        tft.backlightBrightness((uint8_t)(i * 255 / STEPS));
        delay(fade_ms / STEPS);
    }

    // The real UI (moto_dash_create()) builds fresh on this same screen.
    lv_obj_clean(scr);
}
