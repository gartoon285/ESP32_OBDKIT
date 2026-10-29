// =============================================================================
//  ui_fonts.h  —  Original design fonts converted to LVGL (4bpp)
// =============================================================================
//  Chakra Petch (Medium / SemiBold)  → UI text, labels, headers
//  Share Tech Mono (Regular)         → LCD-style numeric readouts
//
//  Regenerate with lv_font_conv, e.g.:
//    npx lv_font_conv --font ChakraPetch-Medium.ttf --size 12 --bpp 4 \
//        --format lvgl --range 0x20-0x7F,0xB0 --no-compress --no-kerning \
//        -o src/ui_fonts/chakra_12.c
// =============================================================================
#pragma once

#include <lvgl.h>

LV_FONT_DECLARE(chakra_12);
LV_FONT_DECLARE(chakra_14);
LV_FONT_DECLARE(chakra_16);
LV_FONT_DECLARE(chakrab_20);
LV_FONT_DECLARE(techmono_20);
LV_FONT_DECLARE(techmono_24);
LV_FONT_DECLARE(techmono_48);
