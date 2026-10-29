// =============================================================================
//  moto_dash.cpp  —  "Modern Retro" motorcycle dashboard (LVGL 9.2), variant 1b
//  See moto_dash.h for the feature list. Coordinates follow the 320x240 design
//  1:1 (the original canvas was drawn at 320x240 then shown at 2x in the browser
//  preview; on the real panel we render at native size).
// =============================================================================
#include "moto_dash.h"

#include <Arduino.h>
#include <lvgl.h>
#include <Preferences.h>
#include <string.h>
#include <math.h>

#include "live_data.h"
#include "touch.h"
#include "ui_fonts/ui_fonts.h"

// -----------------------------------------------------------------------------
//  Palette (matches the design tokens)
// -----------------------------------------------------------------------------
#define C_BLACK   0x000000
#define C_WHITE   0xFFFFFF
#define C_GRAY    0x8A8A8A   // secondary text
#define C_DIM     0x262626   // hairline separators
#define C_YELLOW  0xFFC400
#define C_RED     0xFF3B2F   // accent / redline
#define C_GREEN   0x3DDC6A   // O2 "in range"
#define C_GHOST   0x141414   // dim LCD backdrop behind the RPM digits
#define C_TRACK   0x222222   // fill-bar track

// -----------------------------------------------------------------------------
//  Demo / bring-up simulation.  When MOTO_SIM is 1 the dashboard ignores the
//  live ECU feed and animates synthetic telemetry (RPM sweeps the whole range
//  so the zone colours and bars move). Set to 0 for real ECU data.
// -----------------------------------------------------------------------------
#ifndef MOTO_SIM
// #define MOTO_SIM 1
#endif

// -----------------------------------------------------------------------------
//  Metrics the slots can show.  Index order must match NAMES[].
// -----------------------------------------------------------------------------
// One entry per selectable field in live_data_t (rpm is the main gauge).
// Append-only: the stored slot assignments in NVS are these indices, so new
// metrics must be added at the END (before M_COUNT) to keep old configs valid.
enum Metric {
    M_TEMP = 0, M_TPS, M_O2, M_BATT, M_IAT, M_MAP, M_IGN, M_INJ,
    M_EOT, M_SPEED, M_MODE, M_IDLE, M_O2V, M_STFT, M_O2HV,
    M_TPSV, M_ECTV, M_IATV, M_MAPV, M_EOTV,
    M_COUNT
};
static const char* const NAMES[M_COUNT] = {
    "TEMP", "TPS", "O2", "BATT", "IAT", "MAP", "IGN", "INJ",
    "EOT", "SPEED", "MODE", "IDLE", "O2V", "STFT", "O2HV",
    "TPSV", "ECTV", "IATV", "MAPV", "EOTV"
};

// -----------------------------------------------------------------------------
//  Persisted settings
// -----------------------------------------------------------------------------
// Which gauge layout is on screen. ARC = Design 1a (arc tach + side stack),
// BARS = Design 1b (stepped bar graph + LCD digits) — the dashboard's original
// layout. Both read/write the exact same MotoSettings below ("ใช้ค่าตั้งเดียวกับ
// 1a–1c" in the design notes), so switching styles never resets limits/slots.
enum GaugeStyle { STYLE_ARC = 0, STYLE_BARS = 1 };

// Which live_data_t field the TEMP ALERT thresholds (and the TEMP slot) read:
// ECT = coolant, EOT = oil. Lets a rider whose bike reports both pick which
// one drives the overheat warning.
enum TempSrc { TEMPSRC_ECT = 0, TEMPSRC_EOT = 1 };

struct MotoSettings {
    uint16_t rpmMax;
    uint16_t rpmYellow;
    uint16_t rpmRed;
    int16_t  tempYellow;   // deg C
    int16_t  tempRed;      // deg C
    uint8_t  slot[3];      // Metric indices
    bool     shiftEnabled;
    uint16_t shiftRpm;
    bool     shiftBlink;
    uint8_t  gaugeStyle;   // GaugeStyle
    uint8_t  tempSrc;      // TempSrc
};

static MotoSettings cfg = {
    /*rpmMax*/ 12000, /*rpmYellow*/ 8000, /*rpmRed*/ 10000,
    /*tempYellow*/ 100, /*tempRed*/ 130,
    /*slot*/ { M_TEMP, M_TPS, M_O2 },
    /*shiftEnabled*/ true, /*shiftRpm*/ 9500, /*shiftBlink*/ true,
    /*gaugeStyle*/ STYLE_BARS,
    /*tempSrc*/ TEMPSRC_ECT,
};

static constexpr char      NVS_NS[]   = "motodash";
static constexpr char      NVS_KEY[]  = "cfg";
static constexpr uint32_t  CFG_MAGIC  = 0x4D440004;  // "MD" + version (v4: + tempSrc)

// The live_data_t field the TEMP ALERT / TEMP slot currently reads from.
static inline float active_temp(const live_data_t& d) {
    return cfg.tempSrc == TEMPSRC_EOT ? d.eot_c : d.ect_c;
}

struct CfgBlob { uint32_t magic; MotoSettings s; };

static void cfg_load() {
    Preferences p;
    if (!p.begin(NVS_NS, /*readOnly=*/true)) return;
    CfgBlob b{};
    size_t n = p.getBytes(NVS_KEY, &b, sizeof(b));
    p.end();
    if (n == sizeof(b) && b.magic == CFG_MAGIC) cfg = b.s;
}

static void cfg_save() {
    Preferences p;
    if (!p.begin(NVS_NS, /*readOnly=*/false)) return;
    CfgBlob b{ CFG_MAGIC, cfg };
    p.putBytes(NVS_KEY, &b, sizeof(b));
    p.end();
}

// -----------------------------------------------------------------------------
//  UI object handles
// -----------------------------------------------------------------------------
#define N_BARS    36
#define N_BARLAB  11   // room for up to 20000 rpm (0..20 step 2)

static lv_obj_t* scr          = nullptr;
static lv_obj_t* gauge_root   = nullptr;   // holds whichever gauge style is active
static lv_obj_t* slots_root   = nullptr;   // holds the 3 metric slots (row or column)
static lv_obj_t* warn_border  = nullptr;

// ---- Design 1b: stepped bar graph + LCD digits -----------------------------
static lv_obj_t* bar[N_BARS]  = {nullptr};
static lv_obj_t* barlab[N_BARLAB] = {nullptr};
static lv_obj_t* rpm_ghost    = nullptr;
static lv_obj_t* rpm_val      = nullptr;
static lv_obj_t* shift_dot    = nullptr;
static lv_obj_t* shift_lbl    = nullptr;

// ---- Design 1a: arc tach + side stack --------------------------------------
// The 56 dashes are drawn as lines into one canvas instead of 56 separate
// rotated lv_obj's — LVGL's software renderer treats a *rotated* widget as a
// full affine-transform draw (expensive per pixel) instead of a cheap fill,
// and with 56 of them packed edge-to-edge their invalidated areas kept
// merging into one giant redraw, so updating a couple of dashes repainted
// most of the ring. A line is a native (non-rotated) draw primitive — no
// transform — and drawing all 56 into a canvas means LVGL only ever
// composites ONE flat image to the screen, however many dashes changed.
#define N_ARC_SEG  56   // dial dashes, -135..+135deg
#define N_ARC_LAB  11   // tick numbers, 0..20 step 2 (room for up to 20000 rpm, same cap as N_BARLAB)
static lv_obj_t* arc_canvas    = nullptr;
static uint8_t*  arc_canvas_buf = nullptr;   // lives in system heap, not LVGL's pool
static lv_obj_t* arc_lab[N_ARC_LAB] = {nullptr};
static lv_obj_t* arc_rpm_val  = nullptr;

// Per-slot widgets
struct SlotUi {
    lv_obj_t* box;
    lv_obj_t* label;    // metric name
    lv_obj_t* status;   // RICH/LEAN/STOICH or unit hint
    lv_obj_t* value;    // big number
    lv_obj_t* unit;     // small unit
    lv_obj_t* track;    // fill-bar track
    lv_obj_t* fill;     // fill (width % of track) — used by non-O2
    lv_obj_t* mark;     // marker (centre-referenced) — used by O2
    lv_obj_t* mark_mid; // centre tick for O2
    // cached last-rendered state — repaint a slot only when something changes
    uint8_t   cMetric;      // 0xFF = unset (forces a full first paint)
    char      cVal[12];
    char      cStatus[8];
    uint32_t  cColor;
    uint32_t  cStatusColor;
    int       cPos;         // marker x (O2) or fill percent
};
static SlotUi slotUi[3];

static bool menu_open = false;
static bool dtc_open  = false;   // full-screen DTC list is showing

// Latest DTC list (refreshed in dash_update when its sequence number moves).
static live_dtc_list_t dtc_cur = {};

// -----------------------------------------------------------------------------
//  Small style helpers
// -----------------------------------------------------------------------------
static lv_obj_t* bare(lv_obj_t* parent) {
    lv_obj_t* o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    // Base objects are CLICKABLE by default; decorative containers must not be,
    // or they swallow the press and the screen's long-press (open Settings)
    // never fires. The menu's lv_button_* widgets stay clickable on their own.
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

static lv_obj_t* mk_label(lv_obj_t* parent, const char* txt, const lv_font_t* font,
                          uint32_t color, int32_t letter_sp = 0) {
    lv_obj_t* l = lv_label_create(parent);
    lv_obj_remove_style_all(l);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    if (letter_sp) lv_obj_set_style_text_letter_space(l, letter_sp, 0);
    lv_label_set_text(l, txt);
    return l;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// -----------------------------------------------------------------------------
//  Zone colouring
// -----------------------------------------------------------------------------
static uint32_t rpm_zone(float v) {
    if (v >= cfg.rpmRed)    return C_RED;
    if (v >= cfg.rpmYellow) return C_YELLOW;
    return C_WHITE;
}

// =============================================================================
//  Dashboard build
// =============================================================================
// LCD-style backing panel (dark vertical gradient + a thin bezel ring) behind
// a readout — design 1b draws three of these (bar graph, RPM digits, side
// legend); design 1a's hub/face circles use the same gradient idea.
static void mk_panel_bg(lv_obj_t* obj, uint32_t top, uint32_t bottom) {
    lv_obj_set_style_radius(obj, 4, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(top), 0);
    lv_obj_set_style_bg_grad_color(obj, lv_color_hex(bottom), 0);
    lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(obj, 1, 0);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x1c2028), 0);
}

static void build_bars(lv_obj_t* parent) {
    // Backing panel: 8,6  304x94 — sits behind both the bar graph and its
    // scale-label row below it.
    lv_obj_t* panel = bare(parent);
    lv_obj_set_pos(panel, 8, 6);
    lv_obj_set_size(panel, 304, 94);
    mk_panel_bg(panel, 0x07090d, 0x000000);

    // Bar-graph tach: 16,14  288x66, bars bottom-aligned, 2px gap.
    lv_obj_t* box = bare(parent);
    lv_obj_set_pos(box, 16, 14);
    lv_obj_set_size(box, 288, 66);
    lv_obj_set_flex_flow(box, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(box, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_style_pad_column(box, 2, 0);
    for (int i = 0; i < N_BARS; i++) {
        lv_obj_t* b = bare(box);
        lv_obj_set_flex_grow(b, 1);
        int h = (int)lroundf(12 + (float)i / (N_BARS - 1) * 54);
        lv_obj_set_height(b, h);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(b, lv_color_hex(C_WHITE), 0);
        bar[i] = b;
    }

    // Scale labels row: 14,84  292 wide, space-between.
    lv_obj_t* lrow = bare(parent);
    lv_obj_set_pos(lrow, 14, 84);
    lv_obj_set_size(lrow, 292, 14);
    lv_obj_set_flex_flow(lrow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(lrow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    for (int i = 0; i < N_BARLAB; i++)
        barlab[i] = mk_label(lrow, "", &chakra_12, C_GRAY);
}

static void build_rpm(lv_obj_t* parent) {
    // Backing panel: 8,100  202x78 — behind the big LCD RPM readout.
    lv_obj_t* rpm_panel = bare(parent);
    lv_obj_set_pos(rpm_panel, 8, 100);
    lv_obj_set_size(rpm_panel, 202, 78);
    mk_panel_bg(rpm_panel, 0x0a0c10, 0x000000);

    // Big LCD number: 16,100  190x64, right-aligned, ghost backdrop + live value.
    lv_obj_t* box = bare(parent);
    lv_obj_set_pos(box, 16, 100);
    lv_obj_set_size(box, 190, 64);

    rpm_ghost = mk_label(box, "88888", &techmono_48, C_GHOST);
    lv_obj_set_width(rpm_ghost, 190);
    lv_obj_set_style_text_align(rpm_ghost, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(rpm_ghost, LV_ALIGN_TOP_RIGHT, 0, 0);

    rpm_val = mk_label(box, "0", &techmono_48, C_WHITE);
    lv_obj_set_width(rpm_val, 190);
    lv_obj_set_style_text_align(rpm_val, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(rpm_val, LV_ALIGN_TOP_RIGHT, 0, 0);

    // Backing panel: 214,100  98x78 — behind the side legend (RPM/SHIFT).
    lv_obj_t* side_panel = bare(parent);
    lv_obj_set_pos(side_panel, 214, 100);
    lv_obj_set_size(side_panel, 98, 78);
    mk_panel_bg(side_panel, 0x0c0f14, 0x030405);

    // Side legend: 218,108.
    lv_obj_t* side = bare(parent);
    lv_obj_set_pos(side, 218, 108);
    lv_obj_set_size(side, 92, 64);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(side, 4, 0);

    mk_label(side, "RPM", &chakrab_20, C_WHITE, 2);
    mk_label(side, "x1000 SCALE", &chakra_12, C_GRAY, 1);

    lv_obj_t* srow = bare(side);
    lv_obj_set_size(srow, 92, 12);
    lv_obj_set_flex_flow(srow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(srow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(srow, 6, 0);
    lv_obj_set_style_pad_top(srow, 6, 0);
    shift_dot = bare(srow);
    lv_obj_set_size(shift_dot, 8, 8);
    lv_obj_set_style_radius(shift_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(shift_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(shift_dot, lv_color_hex(0x333333), 0);
    lv_obj_set_style_shadow_width(shift_dot, 8, 0);
    lv_obj_set_style_shadow_spread(shift_dot, 1, 0);
    lv_obj_set_style_shadow_color(shift_dot, lv_color_hex(0x333333), 0);
    lv_obj_set_style_shadow_opa(shift_dot, LV_OPA_80, 0);
    shift_lbl = mk_label(srow, "SHIFT", &chakra_12, 0x333333, 1);
}

// `vertical` picks the divider between slots: a left rule when the slots sit
// in a row (1b, along the bottom) or a top rule when they're stacked in a
// column (1a, down the right side) — the slot's own internal layout (name +
// status / big value / fill-bar) is identical either way.
static lv_obj_t* make_slot(int i, lv_obj_t* parent, bool vertical) {
    SlotUi& u = slotUi[i];
    u.cMetric = 0xFF;   // unset → slot_render() does a full first paint
    lv_obj_t* b = bare(parent);
    u.box = b;
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(b, 3, 0);
    if (i > 0) {
        if (vertical) {
            lv_obj_set_style_pad_top(b, 8, 0);
            lv_obj_set_style_border_side(b, LV_BORDER_SIDE_TOP, 0);
        } else {
            lv_obj_set_style_pad_top(b, 6, 0);
            lv_obj_set_style_pad_left(b, 10, 0);
            lv_obj_set_style_border_side(b, LV_BORDER_SIDE_LEFT, 0);
        }
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(C_DIM), 0);
    } else if (!vertical) {
        lv_obj_set_style_pad_top(b, 6, 0);
    }

    // header row: name  +  status
    lv_obj_t* hr = bare(b);
    lv_obj_set_width(hr, lv_pct(100));
    lv_obj_set_height(hr, 12);
    lv_obj_set_flex_flow(hr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    u.label  = mk_label(hr, "", &chakra_12, C_GRAY, 1);
    u.status = mk_label(hr, "", &chakra_12, C_GRAY, 1);

    // value row: big value + unit
    lv_obj_t* vr = bare(b);
    lv_obj_set_width(vr, lv_pct(100));
    lv_obj_set_height(vr, 26);
    lv_obj_set_flex_flow(vr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(vr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(vr, 3, 0);
    u.value = mk_label(vr, "0", &techmono_24, C_WHITE);
    u.unit  = mk_label(vr, "", &chakra_12, C_WHITE);

    // fill-bar track
    u.track = bare(b);
    lv_obj_set_width(u.track, lv_pct(92));
    lv_obj_set_height(u.track, 3);
    lv_obj_set_style_bg_opa(u.track, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(u.track, lv_color_hex(C_TRACK), 0);

    u.fill = bare(u.track);
    lv_obj_set_height(u.fill, 3);
    lv_obj_set_width(u.fill, 0);
    lv_obj_align(u.fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(u.fill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(u.fill, lv_color_hex(C_WHITE), 0);

    // centre tick + marker (O2)
    u.mark_mid = bare(u.track);
    lv_obj_set_size(u.mark_mid, 1, 7);
    lv_obj_align(u.mark_mid, LV_ALIGN_TOP_MID, 0, -2);
    lv_obj_set_style_bg_opa(u.mark_mid, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(u.mark_mid, lv_color_hex(0x555555), 0);

    u.mark = bare(u.track);
    lv_obj_set_size(u.mark, 3, 7);
    lv_obj_align(u.mark, LV_ALIGN_TOP_LEFT, 0, -2);
    lv_obj_set_style_bg_opa(u.mark, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(u.mark, lv_color_hex(C_WHITE), 0);
    return b;
}

static void build_slots_row(lv_obj_t* parent) {
    // Row container: 16,182  288x52, 1px top rule, 3 equal columns (design 1b).
    lv_obj_t* row = bare(parent);
    lv_obj_set_pos(row, 16, 182);
    lv_obj_set_size(row, 288, 52);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(0x6a707a), 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(row, lv_color_hex(0x11141a), 0);
    lv_obj_set_style_bg_grad_color(row, lv_color_hex(C_BLACK), 0);
    lv_obj_set_style_bg_grad_dir(row, LV_GRAD_DIR_VER, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    for (int i = 0; i < 3; i++) {
        lv_obj_t* b = make_slot(i, row, /*vertical=*/false);
        lv_obj_set_flex_grow(b, 1);
        lv_obj_set_height(b, lv_pct(100));
    }
}

static void build_slots_col(lv_obj_t* parent) {
    // Vertical instrument panel: 226,6  88x228 (design 1a's side stack).
    lv_obj_t* bezel = bare(parent);
    lv_obj_set_pos(bezel, 226, 6);
    lv_obj_set_size(bezel, 88, 228);
    lv_obj_set_style_radius(bezel, 4, 0);
    lv_obj_set_style_bg_opa(bezel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bezel, lv_color_hex(0x07090d), 0);
    lv_obj_set_style_border_width(bezel, 1, 0);
    lv_obj_set_style_border_color(bezel, lv_color_hex(0x1c2028), 0);

    lv_obj_t* col = bare(parent);
    lv_obj_set_pos(col, 234, 18);
    lv_obj_set_size(col, 74, 206);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);

    for (int i = 0; i < 3; i++) {
        lv_obj_t* b = make_slot(i, col, /*vertical=*/true);
        lv_obj_set_width(b, lv_pct(100));
        lv_obj_set_flex_grow(b, 1);
    }
}

// Ring geometry shared by build_arc() and its frame-update helpers. Pulled
// 1:1 from the current design canvas (project/Moto Dashboard.dc.html, 1a):
// centre (110,122); metal bezel r=108; dark face r=106; thin accent ring
// r=85; the 56 dashes span r=87 (inner edge) to r=100 (outer edge, so
// height=13); hub r=52.
static constexpr int ARC_CX = 110, ARC_CY = 122;
static constexpr int ARC_R_BEZEL = 108, ARC_R_FACE = 106, ARC_R_RING = 85;
static constexpr int ARC_R_DASH_OUT = 100, ARC_R_HUB = 52;
static constexpr int ARC_SEG_W = 4, ARC_SEG_H = 13;   // ARC_R_DASH_OUT - ARC_SEG_H = 87 (inner edge)
static constexpr float ARC_LABEL_R = 74.0f;
// Canvas bounding the dash ring: outer dash radius + a little margin for the
// line width and anti-aliasing.
static constexpr int ARC_CANVAS_R  = ARC_R_DASH_OUT + 6;
static constexpr int ARC_CANVAS_SZ = ARC_CANVAS_R * 2;

static void build_arc(lv_obj_t* parent) {
    lv_obj_t* bezel = bare(parent);
    lv_obj_set_pos(bezel, ARC_CX - ARC_R_BEZEL, ARC_CY - ARC_R_BEZEL);
    lv_obj_set_size(bezel, ARC_R_BEZEL * 2, ARC_R_BEZEL * 2);
    lv_obj_set_style_radius(bezel, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(bezel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(bezel, lv_color_hex(0x9aa0aa), 0);
    lv_obj_set_style_bg_grad_color(bezel, lv_color_hex(0x4a4f58), 0);
    lv_obj_set_style_bg_grad_dir(bezel, LV_GRAD_DIR_VER, 0);

    lv_obj_t* face = bare(parent);
    lv_obj_set_pos(face, ARC_CX - ARC_R_FACE, ARC_CY - ARC_R_FACE);
    lv_obj_set_size(face, ARC_R_FACE * 2, ARC_R_FACE * 2);
    lv_obj_set_style_radius(face, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(face, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(face, lv_color_hex(0x0d121b), 0);
    lv_obj_set_style_bg_grad_color(face, lv_color_hex(0x030406), 0);
    lv_obj_set_style_bg_grad_dir(face, LV_GRAD_DIR_VER, 0);

    // 56 dashes sweeping -135..+135deg (270deg total, open at the bottom) —
    // the same "segmented tach" idea as design 1b's straight bar graph, just
    // bent around a dial. Drawn as lines into one canvas (see the comment by
    // arc_canvas's declaration for why, not as 56 rotated widgets.
    if (!arc_canvas_buf) {
        // NOT `lv_canvas_buf_size(..., 0)` for "auto stride" — the macro behind
        // it computes `~(stride-1)` to round up to that stride, and with
        // stride=0 that underflows to 0, masking the rounded width to 0 and
        // making the whole call return just a few bytes (LV_DRAW_BUF_ALIGN).
        // lv_canvas_set_buffer() then writes a real ~90KB image into that
        // handful of bytes — a heap overflow that corrupted memory until
        // something unrelated crashed. Compute the real stride first instead.
        uint32_t stride = lv_draw_buf_width_to_stride(ARC_CANVAS_SZ, LV_COLOR_FORMAT_RGB565);
        uint32_t sz = lv_canvas_buf_size(ARC_CANVAS_SZ, ARC_CANVAS_SZ, 16, stride);
        arc_canvas_buf = (uint8_t*)malloc(sz);
        if (!arc_canvas_buf) Serial.println("[moto_dash] arc canvas buffer alloc FAILED");
    }
    arc_canvas = lv_canvas_create(parent);
    lv_obj_set_pos(arc_canvas, ARC_CX - ARC_CANVAS_R, ARC_CY - ARC_CANVAS_R);
    if (arc_canvas_buf)
        lv_canvas_set_buffer(arc_canvas, arc_canvas_buf, ARC_CANVAS_SZ, ARC_CANVAS_SZ, LV_COLOR_FORMAT_RGB565);

    // Thin accent ring just inside the dashes.
    lv_obj_t* ring = bare(parent);
    lv_obj_set_pos(ring, ARC_CX - ARC_R_RING, ARC_CY - ARC_R_RING);
    lv_obj_set_size(ring, ARC_R_RING * 2, ARC_R_RING * 2);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ring, 1, 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0x2a2f38), 0);

    for (int i = 0; i < N_ARC_LAB; i++)
        arc_lab[i] = mk_label(parent, "", &chakra_12, C_GRAY);

    lv_obj_t* hub = bare(parent);
    lv_obj_set_pos(hub, ARC_CX - ARC_R_HUB, ARC_CY - ARC_R_HUB);
    lv_obj_set_size(hub, ARC_R_HUB * 2, ARC_R_HUB * 2);
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(hub, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(hub, lv_color_hex(0x161a20), 0);
    lv_obj_set_style_bg_grad_color(hub, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_grad_dir(hub, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(hub, 2, 0);
    lv_obj_set_style_border_color(hub, lv_color_hex(0x3a3f48), 0);

    // RPM caption/value/scale row: left:0, width:220 in the design — a literal
    // span, not derived from the ring radius (centres on cx=110 by coincidence
    // of being screen-wide, not because width==2*ARC_R_DASH_OUT).
    const int lw = 220;
    const int lx = 0;

    lv_obj_t* rpmlbl = mk_label(parent, "RPM", &chakra_12, C_GRAY, 3);
    lv_obj_set_width(rpmlbl, lw);
    lv_obj_set_style_text_align(rpmlbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(rpmlbl, lx, ARC_CY - 30);

    // techmono_48 (used for 1b's LCD digits) was wide enough to overflow past
    // the hub for a 4-5 digit rpm; techmono_24 fits comfortably, so recentre
    // both lines vertically on the hub around it (ARC_R_HUB) instead of the
    // old 48px-font offsets.
    arc_rpm_val = mk_label(parent, "0", &techmono_24, C_WHITE);
    lv_obj_set_width(arc_rpm_val, lw);
    lv_obj_set_style_text_align(arc_rpm_val, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(arc_rpm_val, lx, ARC_CY - 12);

    lv_obj_t* scalelbl = mk_label(parent, "x1000 r/min", &chakra_12, C_GRAY, 1);
    lv_obj_set_width(scalelbl, lw);
    lv_obj_set_style_text_align(scalelbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(scalelbl, lx, ARC_CY + 78);
}

static void build_warn() {
    warn_border = bare(scr);
    lv_obj_set_pos(warn_border, 0, 0);
    lv_obj_set_size(warn_border, 320, 240);
    lv_obj_set_style_bg_opa(warn_border, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(warn_border, 4, 0);
    lv_obj_set_style_border_color(warn_border, lv_color_hex(C_RED), 0);
    lv_obj_add_flag(warn_border, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(warn_border, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_clear_flag(warn_border, LV_OBJ_FLAG_CLICKABLE);
}

static void dtc_screen_open();

// =============================================================================
//  Settings menu
// =============================================================================
// menu_cur: -1 = root list, 0..4 = GAUGE/RPM/TEMPERATURE/SHIFT LIGHT/DIAGNOSTICS
// (matches the design canvas's `pg` state exactly). GAUGE has its own
// sub-list (gauge_sub: -1 = that list, 0 = STYLE page, 1 = SLOTS page).
// root_page pages the 5 root items 3 at a time (0 or 1, like the design's
// rootPage).
enum {
    ACT_TY_DEC, ACT_TY_INC, ACT_TR_DEC, ACT_TR_INC, ACT_TEMPSRC_ECT, ACT_TEMPSRC_EOT,
    ACT_MAX_DEC, ACT_MAX_INC, ACT_RY_DEC, ACT_RY_INC, ACT_RR_DEC, ACT_RR_INC,
    ACT_GAUGE_ARC, ACT_GAUGE_BARS,
    ACT_S0_PREV, ACT_S0_NEXT, ACT_S1_PREV, ACT_S1_NEXT, ACT_S2_PREV, ACT_S2_NEXT,
    ACT_SHIFT_ON, ACT_SHIFT_OFF, ACT_SH_DEC, ACT_SH_INC, ACT_STYLE_SOLID, ACT_STYLE_BLINK,
    ACT_OPEN_GAUGE, ACT_OPEN_RPM, ACT_OPEN_TEMP, ACT_OPEN_SHIFT, ACT_OPEN_DTC,
    ACT_OPEN_GAUGE_STYLE, ACT_OPEN_GAUGE_SLOTS,
    ACT_ROOT_PREV, ACT_ROOT_NEXT, ACT_BACK, ACT_DONE, ACT_DTC_VIEW,
};

#define ROOT_ITEMS   5
#define ROOT_PER_PG  3

static lv_obj_t* menu_overlay = nullptr;
static lv_obj_t* menu_title   = nullptr;
static int       menu_cur     = -1;
static int       root_page    = 0;
static int       gauge_sub    = -1;

// the 8 pages the nav can land on
static lv_obj_t* pg_root        = nullptr;
static lv_obj_t* pg_gauge_root  = nullptr;
static lv_obj_t* pg_gauge_style = nullptr;
static lv_obj_t* pg_gauge_slots = nullptr;
static lv_obj_t* pg_rpm         = nullptr;
static lv_obj_t* pg_temp        = nullptr;
static lv_obj_t* pg_shift       = nullptr;
static lv_obj_t* pg_dtc         = nullptr;

static lv_obj_t* root_btn[ROOT_ITEMS]  = {nullptr};
static lv_obj_t* gauge_root_btn[2]     = {nullptr};

// value labels we refresh
static lv_obj_t* lbl_ty = nullptr;
static lv_obj_t* lbl_tr = nullptr;
static lv_obj_t* lbl_max = nullptr;
static lv_obj_t* lbl_ry = nullptr;
static lv_obj_t* lbl_rr = nullptr;
static lv_obj_t* lbl_slot[3] = {nullptr};
static lv_obj_t* lbl_shift = nullptr;
static lv_obj_t* btn_gauge_arc = nullptr;
static lv_obj_t* btn_gauge_bars = nullptr;
static lv_obj_t* btn_tempsrc_ect = nullptr;
static lv_obj_t* btn_tempsrc_eot = nullptr;
static lv_obj_t* btn_on = nullptr;
static lv_obj_t* btn_off = nullptr;
static lv_obj_t* btn_solid = nullptr;
static lv_obj_t* btn_blink = nullptr;
static lv_obj_t* btn_root_prev = nullptr;
static lv_obj_t* btn_root_next = nullptr;
static lv_obj_t* btn_back = nullptr;
static lv_obj_t* lbl_dtc_now = nullptr;
static lv_obj_t* lbl_dtc_mem = nullptr;

static void menu_refresh();
static void build_gauge_style();   // defined with the rest of the gauge-build code

static void cycle_slot(int i, int dir) {
    bool used[M_COUNT] = {false};
    for (int j = 0; j < 3; j++) if (j != i) used[cfg.slot[j]] = true;
    int k = cfg.slot[i];
    do { k = (k + dir + M_COUNT) % M_COUNT; } while (used[k]);
    cfg.slot[i] = (uint8_t)k;
}

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void set_rpm(char key, int d) {
    int m = cfg.rpmMax, y = cfg.rpmYellow, r = cfg.rpmRed;
    if (key == 'm') {
        m = clampi(m + d, 4000, 20000);
        r = min(r, m);
        y = min(y, r - 500);
    } else if (key == 'y') {
        y = clampi(y + d, 1000, r - 500);
    } else { // 'r'
        r = clampi(r + d, y + 500, m);
    }
    cfg.rpmMax = (uint16_t)m; cfg.rpmYellow = (uint16_t)y; cfg.rpmRed = (uint16_t)r;
}

static void menu_act_cb(lv_event_t* e) {
    int act = (int)(intptr_t)lv_event_get_user_data(e);
    switch (act) {
        case ACT_TY_DEC: cfg.tempYellow = max(40, min(cfg.tempRed - 5, cfg.tempYellow - 5)); break;
        case ACT_TY_INC: cfg.tempYellow = max(40, min(cfg.tempRed - 5, cfg.tempYellow + 5)); break;
        case ACT_TR_DEC: cfg.tempRed = max(cfg.tempYellow + 5, min(200, cfg.tempRed - 5)); break;
        case ACT_TR_INC: cfg.tempRed = max(cfg.tempYellow + 5, min(200, cfg.tempRed + 5)); break;
        case ACT_TEMPSRC_ECT: cfg.tempSrc = TEMPSRC_ECT; break;
        case ACT_TEMPSRC_EOT: cfg.tempSrc = TEMPSRC_EOT; break;
        case ACT_MAX_DEC: set_rpm('m', -500); break;
        case ACT_MAX_INC: set_rpm('m',  500); break;
        case ACT_RY_DEC:  set_rpm('y', -500); break;
        case ACT_RY_INC:  set_rpm('y',  500); break;
        case ACT_RR_DEC:  set_rpm('r', -500); break;
        case ACT_RR_INC:  set_rpm('r',  500); break;
        case ACT_S0_PREV: cycle_slot(0, -1); break;
        case ACT_S0_NEXT: cycle_slot(0,  1); break;
        case ACT_S1_PREV: cycle_slot(1, -1); break;
        case ACT_S1_NEXT: cycle_slot(1,  1); break;
        case ACT_S2_PREV: cycle_slot(2, -1); break;
        case ACT_S2_NEXT: cycle_slot(2,  1); break;
        case ACT_SHIFT_ON:  cfg.shiftEnabled = true;  break;
        case ACT_SHIFT_OFF: cfg.shiftEnabled = false; break;
        case ACT_SH_DEC: cfg.shiftRpm = (uint16_t)max(1000, min((int)cfg.rpmMax, (int)cfg.shiftRpm) - 250); break;
        case ACT_SH_INC: cfg.shiftRpm = (uint16_t)min((int)cfg.rpmMax, cfg.shiftRpm + 250); break;
        case ACT_STYLE_SOLID: cfg.shiftBlink = false; break;
        case ACT_STYLE_BLINK: cfg.shiftBlink = true;  break;
        case ACT_GAUGE_ARC:
            if (cfg.gaugeStyle != STYLE_ARC)  { cfg.gaugeStyle = STYLE_ARC;  build_gauge_style(); }
            break;
        case ACT_GAUGE_BARS:
            if (cfg.gaugeStyle != STYLE_BARS) { cfg.gaugeStyle = STYLE_BARS; build_gauge_style(); }
            break;
        case ACT_OPEN_GAUGE: menu_cur = 0; gauge_sub = -1; break;
        case ACT_OPEN_RPM:   menu_cur = 1; break;
        case ACT_OPEN_TEMP:  menu_cur = 2; break;
        case ACT_OPEN_SHIFT: menu_cur = 3; break;
        case ACT_OPEN_DTC:   menu_cur = 4; break;
        case ACT_OPEN_GAUGE_STYLE: gauge_sub = 0; break;
        case ACT_OPEN_GAUGE_SLOTS: gauge_sub = 1; break;
        case ACT_ROOT_PREV: root_page = max(0, root_page - 1); break;
        case ACT_ROOT_NEXT: root_page = min((ROOT_ITEMS - 1) / ROOT_PER_PG, root_page + 1); break;
        case ACT_BACK:
            if (menu_cur == 0 && gauge_sub >= 0) gauge_sub = -1;
            else { menu_cur = -1; root_page = 0; }
            break;
        case ACT_DONE:
        case ACT_DTC_VIEW:
            menu_open = false;
            lv_obj_add_flag(menu_overlay, LV_OBJ_FLAG_HIDDEN);
            cfg_save();
            if (act == ACT_DTC_VIEW) dtc_screen_open();
            return;
    }
    menu_refresh();
}

static lv_obj_t* step_btn(lv_obj_t* p, const char* txt, int act) {
    lv_obj_t* b = lv_button_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, 30, 28);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_BLACK), 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x333333), LV_STATE_PRESSED);
    lv_obj_t* l = mk_label(b, txt, &techmono_20, C_WHITE);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, menu_act_cb, LV_EVENT_CLICKED, (void*)(intptr_t)act);
    return b;
}

static lv_obj_t* seg_btn(lv_obj_t* p, const char* txt, int act) {
    lv_obj_t* b = lv_button_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_flex_grow(b, 1);
    lv_obj_set_height(b, 30);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_BLACK), 0);
    lv_obj_t* l = mk_label(b, txt, &chakra_12, C_WHITE, 2);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, menu_act_cb, LV_EVENT_CLICKED, (void*)(intptr_t)act);
    return b;
}

// a full-width row-menu item: label + chevron, bottom divider (root list and
// the GAUGE sub-list)
static lv_obj_t* list_btn(lv_obj_t* p, const char* label, int act) {
    lv_obj_t* b = lv_button_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_width(b, lv_pct(100));
    lv_obj_set_height(b, 36);
    lv_obj_set_style_border_side(b, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x222222), LV_STATE_PRESSED);
    lv_obj_set_style_pad_left(b, 4, 0);
    lv_obj_set_style_pad_right(b, 4, 0);
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(b, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    mk_label(b, label, &chakra_12, C_WHITE, 1);
    mk_label(b, ">", &chakra_14, 0x555555);
    lv_obj_add_event_cb(b, menu_act_cb, LV_EVENT_CLICKED, (void*)(intptr_t)act);
    return b;
}

// the "< BACK" nav button shown whenever the menu is inside a category
static lv_obj_t* back_btn(lv_obj_t* p) {
    lv_obj_t* b = lv_button_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, 64, 32);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_BLACK), 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x333333), LV_STATE_PRESSED);
    lv_obj_t* l = mk_label(b, "< BACK", &chakra_12, C_WHITE, 1);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, menu_act_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ACT_BACK);
    return b;
}

// a "label  [-] value [+]" stepper row; returns the value label
static lv_obj_t* stepper_row(lv_obj_t* page, const char* name, uint32_t dotColor,
                             int actDec, int actInc) {
    lv_obj_t* r = bare(page);
    lv_obj_set_width(r, lv_pct(100));
    lv_obj_set_height(r, 32);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* left = bare(r);
    lv_obj_set_height(left, 32);
    lv_obj_set_flex_flow(left, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(left, 6, 0);
    lv_obj_set_width(left, LV_SIZE_CONTENT);
    if (dotColor) {
        lv_obj_t* d = bare(left);
        lv_obj_set_size(d, 8, 8);
        lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(d, lv_color_hex(dotColor), 0);
    }
    mk_label(left, name, &chakra_14, C_WHITE, 1);

    lv_obj_t* right = bare(r);
    lv_obj_set_height(right, 32);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, 6, 0);
    lv_obj_set_width(right, LV_SIZE_CONTENT);
    step_btn(right, "-", actDec);
    lv_obj_t* v = mk_label(right, "", &techmono_24, dotColor ? dotColor : C_WHITE);
    lv_obj_set_width(v, 80);   // room for 5-digit RPM values in techmono_24
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_CENTER, 0);
    step_btn(right, "+", actInc);
    return v;
}

// a "name  [<] value [>]" cycle row (slots)
static lv_obj_t* cycle_row(lv_obj_t* page, const char* name, int actPrev, int actNext) {
    lv_obj_t* r = bare(page);
    lv_obj_set_width(r, lv_pct(100));
    lv_obj_set_height(r, 32);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    mk_label(r, name, &chakra_14, C_WHITE, 1);

    lv_obj_t* right = bare(r);
    lv_obj_set_height(right, 32);
    lv_obj_set_width(right, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, 6, 0);
    step_btn(right, "<", actPrev);
    lv_obj_t* v = mk_label(right, "", &chakra_16, C_WHITE, 1);
    lv_obj_set_width(v, 58);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_CENTER, 0);
    step_btn(right, ">", actNext);
    return v;
}

// a "name ........ value" read-only row; returns the value label
static lv_obj_t* info_row(lv_obj_t* page, const char* name, uint32_t dotColor) {
    lv_obj_t* r = bare(page);
    lv_obj_set_width(r, lv_pct(100));
    lv_obj_set_height(r, 32);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* left = bare(r);
    lv_obj_set_size(left, LV_SIZE_CONTENT, 32);
    lv_obj_set_flex_flow(left, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(left, 6, 0);
    lv_obj_t* d = bare(left);
    lv_obj_set_size(d, 8, 8);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(d, lv_color_hex(dotColor), 0);
    mk_label(left, name, &chakra_14, C_WHITE, 1);

    lv_obj_t* v = mk_label(r, "-", &techmono_24, dotColor);
    lv_obj_set_width(v, 80);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_CENTER, 0);
    return v;
}

static lv_obj_t* make_page(lv_obj_t* panel) {
    lv_obj_t* pg = bare(panel);
    lv_obj_set_width(pg, lv_pct(100));
    // Fixed height = tallest page (4 rows: 4*32 + 3*8 pad) so the panel window
    // stays the same size on every page (a shorter page just has slack at the
    // bottom instead of shrinking the whole dialog).
    lv_obj_set_height(pg, 152);
    lv_obj_set_flex_flow(pg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(pg, 8, 0);
    return pg;
}

static void build_menu() {
    menu_overlay = bare(scr);
    lv_obj_set_pos(menu_overlay, 0, 0);
    lv_obj_set_size(menu_overlay, 320, 240);
    lv_obj_set_style_bg_opa(menu_overlay, LV_OPA_40, 0);
    lv_obj_set_style_bg_color(menu_overlay, lv_color_hex(C_BLACK), 0);
    lv_obj_add_flag(menu_overlay, LV_OBJ_FLAG_IGNORE_LAYOUT);

    lv_obj_t* panel = bare(menu_overlay);
    lv_obj_set_size(panel, 286, LV_SIZE_CONTENT);   // matches the design's panel width exactly
    lv_obj_align(panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(panel, lv_color_hex(C_BLACK), 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_pad_all(panel, 12, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(panel, 8, 0);

    // header: SETTINGS .... <title>
    lv_obj_t* hdr = bare(panel);
    lv_obj_set_width(hdr, lv_pct(100));
    lv_obj_set_height(hdr, 22);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(hdr, 1, 0);
    lv_obj_set_style_border_color(hdr, lv_color_hex(C_DIM), 0);
    lv_obj_set_style_pad_bottom(hdr, 7, 0);
    mk_label(hdr, "SETTINGS", &chakrab_20, C_WHITE, 3);
    menu_title = mk_label(hdr, "", &chakra_12, C_GRAY, 1);

    // ---- root list: 1-GAUGE / 2-RPM / 3-TEMPERATURE / 4-SHIFT LIGHT / 5-DIAGNOSTICS ----
    pg_root = bare(panel);
    lv_obj_set_width(pg_root, lv_pct(100));
    lv_obj_set_height(pg_root, 152);
    lv_obj_set_flex_flow(pg_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(pg_root, 2, 0);
    {
        static const char* labels[ROOT_ITEMS] = {
            "1 - GAUGE", "2 - RPM", "3 - TEMPERATURE", "4 - SHIFT LIGHT", "5 - DIAGNOSTICS"
        };
        static const int acts[ROOT_ITEMS] = {
            ACT_OPEN_GAUGE, ACT_OPEN_RPM, ACT_OPEN_TEMP, ACT_OPEN_SHIFT, ACT_OPEN_DTC
        };
        for (int i = 0; i < ROOT_ITEMS; i++) root_btn[i] = list_btn(pg_root, labels[i], acts[i]);
    }

    // ---- GAUGE's own sub-list: STYLE / SLOTS ----
    pg_gauge_root = bare(panel);
    lv_obj_set_width(pg_gauge_root, lv_pct(100));
    lv_obj_set_height(pg_gauge_root, 152);
    lv_obj_set_flex_flow(pg_gauge_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(pg_gauge_root, 2, 0);
    gauge_root_btn[0] = list_btn(pg_gauge_root, "STYLE", ACT_OPEN_GAUGE_STYLE);
    gauge_root_btn[1] = list_btn(pg_gauge_root, "SLOTS", ACT_OPEN_GAUGE_SLOTS);

    // ---- GAUGE > STYLE ----
    pg_gauge_style = make_page(panel);
    {
        lv_obj_t* r = bare(pg_gauge_style);
        lv_obj_set_width(r, lv_pct(100)); lv_obj_set_height(r, 32);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        mk_label(r, "STYLE", &chakra_14, C_WHITE, 1);
        lv_obj_t* seg = bare(r);
        lv_obj_set_size(seg, 144, 32);
        lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
        btn_gauge_arc  = seg_btn(seg, "ARC",  ACT_GAUGE_ARC);
        btn_gauge_bars = seg_btn(seg, "BARS", ACT_GAUGE_BARS);
    }

    // ---- GAUGE > SLOTS ----
    pg_gauge_slots = make_page(panel);
    lbl_slot[0] = cycle_row(pg_gauge_slots, "SLOT 1", ACT_S0_PREV, ACT_S0_NEXT);
    lbl_slot[1] = cycle_row(pg_gauge_slots, "SLOT 2", ACT_S1_PREV, ACT_S1_NEXT);
    lbl_slot[2] = cycle_row(pg_gauge_slots, "SLOT 3", ACT_S2_PREV, ACT_S2_NEXT);

    // ---- RPM ----
    pg_rpm = make_page(panel);
    lbl_max = stepper_row(pg_rpm, "MAX",       C_WHITE,  ACT_MAX_DEC, ACT_MAX_INC);
    lbl_ry  = stepper_row(pg_rpm, "YELLOW >=", C_YELLOW, ACT_RY_DEC,  ACT_RY_INC);
    lbl_rr  = stepper_row(pg_rpm, "RED >=",    C_RED,    ACT_RR_DEC,  ACT_RR_INC);

    // ---- TEMPERATURE: sensor source (ECT/EOT) + YELLOW/RED thresholds ----
    pg_temp = make_page(panel);
    {
        lv_obj_t* r = bare(pg_temp);
        lv_obj_set_width(r, lv_pct(100)); lv_obj_set_height(r, 32);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        mk_label(r, "SENSOR", &chakra_14, C_WHITE, 1);
        lv_obj_t* seg = bare(r);
        lv_obj_set_size(seg, 144, 32);
        lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
        btn_tempsrc_ect = seg_btn(seg, "ECT", ACT_TEMPSRC_ECT);
        btn_tempsrc_eot = seg_btn(seg, "EOT", ACT_TEMPSRC_EOT);
    }
    lbl_ty = stepper_row(pg_temp, "YELLOW >=", C_YELLOW, ACT_TY_DEC, ACT_TY_INC);
    lbl_tr = stepper_row(pg_temp, "RED >",     C_RED,    ACT_TR_DEC, ACT_TR_INC);

    // ---- SHIFT LIGHT ----
    pg_shift = make_page(panel);
    {
        lv_obj_t* r = bare(pg_shift);
        lv_obj_set_width(r, lv_pct(100)); lv_obj_set_height(r, 32);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        mk_label(r, "LIGHT", &chakra_14, C_WHITE, 1);
        lv_obj_t* seg = bare(r);
        lv_obj_set_size(seg, 130, 30);
        lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
        btn_on  = seg_btn(seg, "ON",  ACT_SHIFT_ON);
        btn_off = seg_btn(seg, "OFF", ACT_SHIFT_OFF);
    }
    lbl_shift = stepper_row(pg_shift, "AT RPM", C_RED, ACT_SH_DEC, ACT_SH_INC);
    {
        lv_obj_t* r = bare(pg_shift);
        lv_obj_set_width(r, lv_pct(100)); lv_obj_set_height(r, 32);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        mk_label(r, "STYLE", &chakra_14, C_WHITE, 1);
        lv_obj_t* seg = bare(r);
        lv_obj_set_size(seg, 130, 30);
        lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
        btn_solid = seg_btn(seg, "SOLID", ACT_STYLE_SOLID);
        btn_blink = seg_btn(seg, "BLINK", ACT_STYLE_BLINK);
    }

    // ---- DIAGNOSTICS ----
    pg_dtc = make_page(panel);
    lbl_dtc_now = info_row(pg_dtc, "CURRENT", C_RED);
    lbl_dtc_mem = info_row(pg_dtc, "STORED",  C_YELLOW);
    {
        lv_obj_t* r = bare(pg_dtc);
        lv_obj_set_width(r, lv_pct(100)); lv_obj_set_height(r, 32);
        lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
        seg_btn(r, "VIEW CODES", ACT_DTC_VIEW);
    }

    // ---- nav row: [< / < BACK]  DONE  [>] ----
    lv_obj_t* nav = bare(panel);
    lv_obj_set_width(nav, lv_pct(100));
    lv_obj_set_height(nav, 32);
    lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nav, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(nav, 6, 0);
    btn_root_prev = step_btn(nav, "<", ACT_ROOT_PREV);
    btn_back      = back_btn(nav);
    lv_obj_t* done = lv_button_create(nav);
    lv_obj_remove_style_all(done);
    lv_obj_set_flex_grow(done, 1);
    lv_obj_set_height(done, 32);
    lv_obj_set_style_border_width(done, 1, 0);
    lv_obj_set_style_border_color(done, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_opa(done, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(done, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_color(done, lv_color_hex(C_BLACK), LV_STATE_PRESSED);
    lv_obj_t* dl = mk_label(done, "DONE", &chakra_14, C_BLACK, 3);
    lv_obj_center(dl);
    lv_obj_add_event_cb(done, menu_act_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ACT_DONE);
    btn_root_next = step_btn(nav, ">", ACT_ROOT_NEXT);

    lv_obj_add_flag(menu_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void seg_active(lv_obj_t* b, bool active) {
    lv_obj_set_style_bg_color(b, lv_color_hex(active ? C_WHITE : C_BLACK), 0);
    lv_obj_t* l = lv_obj_get_child(b, 0);
    if (l) lv_obj_set_style_text_color(l, lv_color_hex(active ? C_BLACK : C_WHITE), 0);
}

static void menu_refresh() {
    bool at_root = menu_cur == -1;
    bool at_gauge_root = menu_cur == 0 && gauge_sub == -1;
    auto vis = [](lv_obj_t* o, bool show) { (show ? lv_obj_clear_flag : lv_obj_add_flag)(o, LV_OBJ_FLAG_HIDDEN); };

    vis(pg_root,        at_root);
    vis(pg_gauge_root,  at_gauge_root);
    vis(pg_gauge_style, menu_cur == 0 && gauge_sub == 0);
    vis(pg_gauge_slots, menu_cur == 0 && gauge_sub == 1);
    vis(pg_rpm,         menu_cur == 1);
    vis(pg_temp,        menu_cur == 2);
    vis(pg_shift,       menu_cur == 3);
    vis(pg_dtc,         menu_cur == 4);

    // root list: show only the 3 items belonging to root_page
    for (int i = 0; i < ROOT_ITEMS; i++)
        vis(root_btn[i], i >= root_page * ROOT_PER_PG && i < root_page * ROOT_PER_PG + ROOT_PER_PG);

    // nav: root pager vs "< BACK" (never both)
    vis(btn_root_prev, at_root);
    vis(btn_root_next, at_root);
    vis(btn_back, !at_root);
    if (at_root) {
        int pages = (ROOT_ITEMS + ROOT_PER_PG - 1) / ROOT_PER_PG;
        lv_obj_set_style_opa(btn_root_prev, root_page == 0 ? LV_OPA_30 : LV_OPA_COVER, 0);
        lv_obj_set_style_opa(btn_root_next, root_page >= pages - 1 ? LV_OPA_30 : LV_OPA_COVER, 0);
    }

    static const char* cat_titles[5] = { "GAUGE", "RPM", "TEMPERATURE", "SHIFT LIGHT", "DIAGNOSTICS" };
    char tbuf[16];
    if (at_root) {
        int pages = (ROOT_ITEMS + ROOT_PER_PG - 1) / ROOT_PER_PG;
        snprintf(tbuf, sizeof tbuf, "%d / %d", root_page + 1, pages);
        lv_label_set_text(menu_title, tbuf);
    } else if (menu_cur == 0) {
        lv_label_set_text(menu_title, gauge_sub == -1 ? "GAUGE" : gauge_sub == 0 ? "STYLE" : "SLOTS");
    } else {
        lv_label_set_text(menu_title, cat_titles[menu_cur]);
    }

    char buf[12];
    snprintf(buf, sizeof buf, "%dC", cfg.tempYellow); lv_label_set_text(lbl_ty, buf);
    snprintf(buf, sizeof buf, "%dC", cfg.tempRed);    lv_label_set_text(lbl_tr, buf);
    snprintf(buf, sizeof buf, "%u", cfg.rpmMax);      lv_label_set_text(lbl_max, buf);
    snprintf(buf, sizeof buf, "%u", cfg.rpmYellow);   lv_label_set_text(lbl_ry, buf);
    snprintf(buf, sizeof buf, "%u", cfg.rpmRed);      lv_label_set_text(lbl_rr, buf);
    for (int i = 0; i < 3; i++) lv_label_set_text(lbl_slot[i], NAMES[cfg.slot[i]]);
    snprintf(buf, sizeof buf, "%u", min((int)cfg.rpmMax, (int)cfg.shiftRpm));
    lv_label_set_text(lbl_shift, buf);

    if (dtc_cur.valid) {
        snprintf(buf, sizeof buf, "%u", dtc_cur.n_current); lv_label_set_text(lbl_dtc_now, buf);
        snprintf(buf, sizeof buf, "%u", dtc_cur.n_stored);  lv_label_set_text(lbl_dtc_mem, buf);
    } else {
        lv_label_set_text(lbl_dtc_now, "-");
        lv_label_set_text(lbl_dtc_mem, "-");
    }

    seg_active(btn_gauge_arc,  cfg.gaugeStyle == STYLE_ARC);
    seg_active(btn_gauge_bars, cfg.gaugeStyle == STYLE_BARS);
    seg_active(btn_tempsrc_ect, cfg.tempSrc == TEMPSRC_ECT);
    seg_active(btn_tempsrc_eot, cfg.tempSrc == TEMPSRC_EOT);
    seg_active(btn_on,  cfg.shiftEnabled);
    seg_active(btn_off, !cfg.shiftEnabled);
    seg_active(btn_solid, !cfg.shiftBlink);
    seg_active(btn_blink, cfg.shiftBlink);
}

// =============================================================================
//  DTC screen (full-screen list + clear)
// =============================================================================
#define DTC_CONFIRM_MS 3000   // second CLEAR tap must come within this window

static lv_obj_t* dtc_overlay  = nullptr;
static lv_obj_t* dtc_status   = nullptr;
static lv_obj_t* dtc_list     = nullptr;
static lv_obj_t* dtc_clear_bt = nullptr;
static lv_obj_t* dtc_clear_lb = nullptr;
static uint32_t  dtc_armed_at = 0;        // millis() of the first CLEAR tap, 0 = not armed

#if MOTO_SIM
// Demo codes so the screen can be checked without an ECU; CLEAR empties them.
static bool     sim_dtc_cleared = false;
static uint32_t sim_dtc_seq     = 1;
#endif

static uint32_t dtc_seq() {
#if MOTO_SIM
    return sim_dtc_seq;
#else
    return live_dtc_seq;
#endif
}

static void dtc_snapshot(live_dtc_list_t* out) {
#if MOTO_SIM
    *out = live_dtc_list_t{};
    out->valid = true;
    out->clear_state = sim_dtc_cleared ? DTC_CLEAR_OK : DTC_CLEAR_IDLE;
    if (!sim_dtc_cleared) {
        out->n_current = 1; out->current[0] = {7, 2};
        out->n_stored  = 3; out->stored[0]  = {7, 2}; out->stored[1] = {8, 1};
        out->stored[2] = {12, 1};
    }
#else
    live_dtc_get(out);
#endif
}

static void dtc_request_clear() {
#if MOTO_SIM
    sim_dtc_cleared = true;
    sim_dtc_seq++;
#else
    live_dtc_request_clear();
#endif
}

static void dtc_set_clear_label(bool armed) {
    lv_label_set_text(dtc_clear_lb, armed ? "CONFIRM?" : "CLEAR");
    lv_obj_set_style_bg_color(dtc_clear_bt, lv_color_hex(armed ? C_RED : C_BLACK), 0);
    lv_obj_set_style_border_color(dtc_clear_bt, lv_color_hex(armed ? C_RED : C_WHITE), 0);
}

static void dtc_add_row(const live_dtc_t& c, bool now) {
    lv_obj_t* r = bare(dtc_list);
    lv_obj_set_size(r, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(r, 8, 0);
    lv_obj_set_style_pad_bottom(r, 6, 0);
    lv_obj_set_style_border_side(r, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(r, 1, 0);
    lv_obj_set_style_border_color(r, lv_color_hex(C_DIM), 0);

    uint32_t col = now ? C_RED : C_YELLOW;
    char code[8];
    snprintf(code, sizeof code, "%03u-%u", c.code, c.sub);
    lv_obj_t* cl = mk_label(r, code, &techmono_20, col);
    lv_obj_set_width(cl, 66);

    lv_obj_t* tag = mk_label(r, now ? "NOW" : "MEM", &chakra_12, col, 1);
    lv_obj_set_width(tag, 30);
    lv_obj_set_style_pad_top(tag, 4, 0);

    const char* txt = dtc_describe(c.code, c.sub);
    lv_obj_t* dl = mk_label(r, txt ? txt : "Unknown code - see service manual",
                            &chakra_12, txt ? C_WHITE : C_GRAY);
    lv_obj_set_flex_grow(dl, 1);
    lv_obj_set_style_pad_top(dl, 4, 0);
    lv_label_set_long_mode(dl, LV_LABEL_LONG_WRAP);
}

static bool dtc_in(const live_dtc_t* list, uint8_t n, const live_dtc_t& c) {
    for (uint8_t i = 0; i < n; i++)
        if (list[i].code == c.code && list[i].sub == c.sub) return true;
    return false;
}

// Rebuild the status line + list from dtc_cur.
static void dtc_screen_refresh() {
    const live_dtc_list_t& l = dtc_cur;
    char buf[32];
    const char* st;
    uint32_t    sc = C_GRAY;
    switch (l.clear_state) {
        case DTC_CLEAR_PENDING: st = "CLEARING...";  sc = C_YELLOW; break;
        case DTC_CLEAR_FAIL:    st = "CLEAR FAILED"; sc = C_RED;    break;
        default:
            if (!l.valid) { st = live_data.valid ? "READING..." : "NO LINK"; break; }
            snprintf(buf, sizeof buf, "%u NOW  %u MEM", l.n_current, l.n_stored);
            st = buf;
            sc = l.n_current ? C_RED : (l.n_stored ? C_YELLOW : C_GREEN);
            break;
    }
    lv_label_set_text(dtc_status, st);
    lv_obj_set_style_text_color(dtc_status, lv_color_hex(sc), 0);

    lv_obj_clean(dtc_list);
    if (!l.valid) {
        mk_label(dtc_list, "Waiting for ECU...", &chakra_14, C_GRAY, 1);
        return;
    }
    // present codes first, then memory-only ones (a present code is usually
    // stored too — show it once, as NOW)
    for (uint8_t i = 0; i < l.n_current; i++) dtc_add_row(l.current[i], true);
    for (uint8_t i = 0; i < l.n_stored; i++)
        if (!dtc_in(l.current, l.n_current, l.stored[i])) dtc_add_row(l.stored[i], false);
    if (l.n_current == 0 && l.n_stored == 0)
        mk_label(dtc_list, l.clear_state == DTC_CLEAR_OK ? "CLEARED - NO CODES" : "NO CODES",
                 &chakra_14, C_GREEN, 2);
}

static void dtc_act_cb(lv_event_t* e) {
    int act = (int)(intptr_t)lv_event_get_user_data(e);
    if (act == 0) {                      // BACK
        dtc_open = false;
        dtc_armed_at = 0;
        lv_obj_add_flag(dtc_overlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    // CLEAR: first tap arms, second tap (within DTC_CONFIRM_MS) sends
    if (dtc_armed_at == 0) {
        dtc_armed_at = millis() | 1;     // never 0 while armed
        dtc_set_clear_label(true);
    } else {
        dtc_armed_at = 0;
        dtc_set_clear_label(false);
        dtc_request_clear();
    }
}

static lv_obj_t* dtc_btn(lv_obj_t* p, const char* txt, int act, lv_obj_t** out_lbl) {
    lv_obj_t* b = lv_button_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_flex_grow(b, 1);
    lv_obj_set_height(b, 32);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_BLACK), 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x333333), LV_STATE_PRESSED);
    lv_obj_t* l = mk_label(b, txt, &chakra_14, C_WHITE, 3);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, dtc_act_cb, LV_EVENT_CLICKED, (void*)(intptr_t)act);
    if (out_lbl) *out_lbl = l;
    return b;
}

static void build_dtc_screen() {
    dtc_overlay = bare(scr);
    lv_obj_set_pos(dtc_overlay, 0, 0);
    lv_obj_set_size(dtc_overlay, 320, 240);
    lv_obj_set_style_bg_opa(dtc_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(dtc_overlay, lv_color_hex(C_BLACK), 0);
    lv_obj_add_flag(dtc_overlay, LV_OBJ_FLAG_IGNORE_LAYOUT);

    // header: DTC .... status
    lv_obj_t* hdr = bare(dtc_overlay);
    lv_obj_set_pos(hdr, 12, 8);
    lv_obj_set_size(hdr, 296, 30);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(hdr, 1, 0);
    lv_obj_set_style_border_color(hdr, lv_color_hex(C_DIM), 0);
    lv_obj_set_style_pad_bottom(hdr, 7, 0);
    mk_label(hdr, "TROUBLE CODES", &chakrab_20, C_WHITE, 2);
    dtc_status = mk_label(hdr, "", &chakra_12, C_GRAY, 1);

    // scrollable code list (clickable so drags scroll it)
    dtc_list = bare(dtc_overlay);
    lv_obj_set_pos(dtc_list, 12, 44);
    lv_obj_set_size(dtc_list, 296, 146);
    lv_obj_add_flag(dtc_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(dtc_list, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(dtc_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dtc_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_bg_opa(dtc_list, LV_OPA_COVER, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(dtc_list, lv_color_hex(0x555555), LV_PART_SCROLLBAR);
    lv_obj_set_style_width(dtc_list, 3, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(dtc_list, 6, 0);
    lv_obj_set_flex_flow(dtc_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(dtc_list, 6, 0);

    // nav: BACK  CLEAR
    lv_obj_t* nav = bare(dtc_overlay);
    lv_obj_set_pos(nav, 12, 198);
    lv_obj_set_size(nav, 296, 32);
    lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(nav, 8, 0);
    dtc_btn(nav, "BACK", 0, nullptr);
    dtc_clear_bt = dtc_btn(nav, "CLEAR", 1, &dtc_clear_lb);

    lv_obj_add_flag(dtc_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void dtc_screen_open() {
    if (!dtc_overlay) build_dtc_screen();
    dtc_open = true;
    dtc_armed_at = 0;
    dtc_set_clear_label(false);
    dtc_screen_refresh();
    lv_obj_clear_flag(dtc_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(dtc_overlay);
}

// Called every dash frame: tell the K-line task whether to query DTCs (only
// while Settings page DIAGNOSTIC or the TROUBLE CODES screen is showing), pick
// up a new list for the settings counts / open DTC screen, and time out an
// armed CLEAR.
static void dtc_update() {
    static bool want_last = false;
    bool want = dtc_open || (menu_open && menu_cur == 4);
    if (want != want_last) {
        live_dtc_set_active(want);
        want_last = want;
    }

    static uint32_t seen = 0;
    uint32_t seq = dtc_seq();
    if (seq != seen) {
        seen = seq;
        dtc_snapshot(&dtc_cur);
        if (menu_open) menu_refresh();
        if (dtc_open)  dtc_screen_refresh();
    }

    if (dtc_open && dtc_armed_at && millis() - dtc_armed_at > DTC_CONFIRM_MS) {
        dtc_armed_at = 0;
        dtc_set_clear_label(false);
    }
}

// =============================================================================
//  Live refresh
// =============================================================================
static void slot_render(int i, const live_data_t& d) {
    SlotUi& u = slotUi[i];
    Metric m = (Metric)cfg.slot[i];

    char val[12] = "0";
    const char* unit = "";
    const char* status = "";
    uint32_t color = C_WHITE, statusColor = C_GRAY;
    float frac = 0;
    bool marker = false;

    switch (m) {
        case M_TEMP: {
            float tc = active_temp(d);
            snprintf(val, sizeof val, "%.1f", tc);
            unit = "C";
            color = tc > cfg.tempRed ? C_RED : (tc >= cfg.tempYellow ? C_YELLOW : C_WHITE);
            frac = clampf((tc - 40) / 100.0f, 0, 1);
            break;
        }
        case M_TPS:
            snprintf(val, sizeof val, "%.1f", d.tps);
            unit = "%"; frac = clampf(d.tps / 100.0f, 0, 1);
            break;
        case M_O2: {
            snprintf(val, sizeof val, "%.1f", d.af);
            unit = "AFR"; marker = true;
            bool off = (d.af < 13.8f || d.af > 15.6f);
            color = off ? C_RED : C_GREEN;
            status = d.af < 13.8f ? "RICH" : (d.af > 15.6f ? "LEAN" : "STOICH");
            statusColor = color;
            frac = clampf((d.af - 10) / 10.0f, 0, 1);
            break;
        }
        case M_BATT:
            snprintf(val, sizeof val, "%.1f", d.batt_v);
            unit = "V"; color = d.batt_v < 12 ? C_YELLOW : C_WHITE;
            frac = clampf((d.batt_v - 10) / 5.0f, 0, 1);
            break;
        case M_IAT:
            snprintf(val, sizeof val, "%.1f", d.iat_c);
            unit = "C"; frac = clampf(d.iat_c / 80.0f, 0, 1);
            break;
        case M_MAP:
            snprintf(val, sizeof val, "%u", d.map_kpa);
            unit = "kPa"; frac = clampf(d.map_kpa / 105.0f, 0, 1);
            break;
        case M_IGN:
            snprintf(val, sizeof val, "%.1f", d.ign_deg);
            unit = "°"; frac = clampf((d.ign_deg + 10) / 60.0f, 0, 1);  // ~-10..50 deg
            break;
        case M_INJ:
            snprintf(val, sizeof val, "%.1f", d.inj);
            unit = "ms"; frac = clampf(d.inj / 20.0f, 0, 1);  // K-line: injection time
            break;
        case M_EOT:
            snprintf(val, sizeof val, "%.1f", d.eot_c);
            unit = "C"; frac = clampf((d.eot_c - 40) / 100.0f, 0, 1);
            break;
        case M_SPEED:
            snprintf(val, sizeof val, "%u", d.speed_kmh);
            unit = "km/h"; frac = clampf(d.speed_kmh / 200.0f, 0, 1);
            break;
        case M_MODE:
            snprintf(val, sizeof val, "%u", d.mode);
            unit = ""; frac = clampf(d.mode / 255.0f, 0, 1);
            break;
        case M_IDLE:
            snprintf(val, sizeof val, "%s", d.fast_idle ? "ON" : "OFF");
            unit = ""; frac = d.fast_idle ? 1.0f : 0.0f;
            color = d.fast_idle ? C_GREEN : C_WHITE;
            break;
        case M_O2V: {
            snprintf(val, sizeof val, "%.2f", d.o2_v);
            unit = "V"; frac = clampf(d.o2_v / 1.0f, 0, 1);  // narrowband 0-1V
            bool off = (d.af < 13.8f || d.af > 15.6f);       // richness from mapped AFR
            color = off ? C_RED : C_GREEN;
            status = d.af < 13.8f ? "RICH" : (d.af > 15.6f ? "LEAN" : "STOICH");
            statusColor = color;
            break;
        }
        case M_STFT:
            snprintf(val, sizeof val, "%.1f", d.stft);
            unit = "%"; frac = clampf((d.stft + 10) / 20.0f, 0, 1);  // centred on 0
            break;
        case M_O2HV:
            snprintf(val, sizeof val, "%.2f", d.o2_heater_v);
            unit = "V"; frac = clampf(d.o2_heater_v / 5.0f, 0, 1);
            break;
        case M_TPSV:
            snprintf(val, sizeof val, "%.2f", d.tps_v);
            unit = "V"; frac = clampf(d.tps_v / 5.0f, 0, 1);
            break;
        case M_ECTV:
            snprintf(val, sizeof val, "%.2f", d.ect_v);
            unit = "V"; frac = clampf(d.ect_v / 5.0f, 0, 1);
            break;
        case M_IATV:
            snprintf(val, sizeof val, "%.2f", d.iat_v);
            unit = "V"; frac = clampf(d.iat_v / 5.0f, 0, 1);
            break;
        case M_MAPV:
            snprintf(val, sizeof val, "%.2f", d.map_v);
            unit = "V"; frac = clampf(d.map_v / 5.0f, 0, 1);
            break;
        case M_EOTV:
            snprintf(val, sizeof val, "%.2f", d.eot_v);
            unit = "V"; frac = clampf(d.eot_v / 5.0f, 0, 1);
            break;
        default: break;
    }

    // name/unit text + marker-vs-fill visibility only change when the assigned
    // metric changes — do them once, then force the value/colour/pos caches to
    // repaint for the new metric.
    if (u.cMetric != (uint8_t)m) {
        u.cMetric = (uint8_t)m;
        lv_label_set_text(u.label, NAMES[m]);
        lv_label_set_text(u.unit, unit);
        if (marker) {
            lv_obj_add_flag(u.fill, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(u.mark, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(u.mark_mid, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(u.fill, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(u.mark, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(u.mark_mid, LV_OBJ_FLAG_HIDDEN);
        }
        // Sentinels that match no real value, so the blocks below always repaint
        // for the new metric — in particular an empty status must overwrite a
        // stale "RICH"/"LEAN" left by the previous (O2) metric.
        u.cVal[0] = '\x01'; u.cVal[1] = '\0';
        u.cStatus[0] = '\x01'; u.cStatus[1] = '\0';
        u.cColor = 0; u.cStatusColor = 0; u.cPos = INT32_MIN;
    }

    // value number
    if (strncmp(u.cVal, val, sizeof u.cVal) != 0) {
        lv_label_set_text(u.value, val);
        strncpy(u.cVal, val, sizeof u.cVal - 1); u.cVal[sizeof u.cVal - 1] = '\0';
    }
    // value + unit + indicator colour (all follow `color`)
    if (u.cColor != color) {
        lv_obj_set_style_text_color(u.value, lv_color_hex(color), 0);
        lv_obj_set_style_text_color(u.unit,  lv_color_hex(color), 0);
        lv_obj_set_style_bg_color(marker ? u.mark : u.fill, lv_color_hex(color), 0);
        u.cColor = color;
    }
    // status (O2 RICH/LEAN/STOICH)
    if (strncmp(u.cStatus, status, sizeof u.cStatus) != 0) {
        lv_label_set_text(u.status, status);
        strncpy(u.cStatus, status, sizeof u.cStatus - 1); u.cStatus[sizeof u.cStatus - 1] = '\0';
    }
    if (u.cStatusColor != statusColor) {
        lv_obj_set_style_text_color(u.status, lv_color_hex(statusColor), 0);
        u.cStatusColor = statusColor;
    }
    // marker position (O2) / fill width — the part that actually moves each frame
    if (marker) {
        int tw = lv_obj_get_width(u.track);
        int x = (int)lroundf(frac * tw);
        if (x < 0) x = 0; if (x > tw - 3) x = tw - 3;
        if (u.cPos != x) { lv_obj_align(u.mark, LV_ALIGN_TOP_LEFT, x, -2); u.cPos = x; }
    } else {
        int pct = (int)lroundf(frac * 100);
        if (u.cPos != pct) { lv_obj_set_width(u.fill, lv_pct(pct)); u.cPos = pct; }
    }
}

#if MOTO_SIM
// Synthetic telemetry: RPM sweeps 0..rpmMax..0 on a ~6 s triangle so every zone
// (white/yellow/red) and all the bars animate; the slot metrics ride along.
static live_data_t moto_sim_sample() {
    float ph  = (millis() % 6000) / 6000.0f;      // 0..1 over 6 s
    float tri = ph < 0.5f ? ph * 2.0f : (1.0f - ph) * 2.0f;  // 0..1..0
    float ms  = millis() / 500.0f;
    live_data_t d{};
    d.valid   = true;
    d.rpm     = (uint16_t)(tri * cfg.rpmMax);
    d.tps     = tri * 100.0f;
    d.ect_c   = 60.0f + 65.0f * tri;              // crosses the TEMP yellow zone
    d.iat_c   = 30.0f + 12.0f * tri;
    d.map_kpa = (uint16_t)(30 + 70 * tri);
    d.batt_v  = 13.4f + 0.5f * tri;
    d.ign_deg = 10.0f + 30.0f * tri;
    d.inj     = 2.0f + 9.0f * tri;
    d.eot_c       = 60.0f + 55.0f * tri;
    d.speed_kmh   = (uint16_t)(tri * 180.0f);
    d.mode        = 10;
    d.fast_idle   = tri < 0.12f;
    d.o2_v        = 0.45f + 0.40f * sinf(ms);     // swings lean..rich
    d.af          = o2v_to_afr(d.o2_v);           // AFR follows the O2 curve
    d.stft        = 3.0f * sinf(ms * 0.7f);
    d.o2_heater_v = 1.4f + 0.1f * tri;
    d.tps_v       = tri * 5.0f;
    d.ect_v       = 0.5f + 2.0f * tri;
    d.iat_v       = 1.0f + 1.0f * tri;
    d.map_v       = 0.5f + 3.5f * tri;
    d.eot_v       = 0.5f + 2.0f * tri;
    return d;
}
#endif

// -----------------------------------------------------------------------------
//  Per-style render caches. File-scope (not function-local statics) so
//  switch_gauge_style() can reset them to "unset" right after it tears down
//  and rebuilds the widgets they refer to — otherwise a stale cached colour
//  would make the first post-switch frame skip a repaint it actually needs.
// -----------------------------------------------------------------------------
static uint32_t bar_c[N_BARS] = {0};   // 0 = "unset" (never a real bar colour)
static lv_opa_t bar_o[N_BARS] = {0};
static int      lab_max = -1, lab_y = -1, lab_r = -1;
static int      last_rpm = -1;
static uint32_t last_rpm_c = 0;
static uint32_t last_sc = 0;

static int      arc_lab_max = -1, arc_lab_y = -1, arc_lab_r = -1;
static uint32_t arc_last_rpm_c = 0;
static int      arc_canvas_rpm = INT32_MIN, arc_canvas_max = -1;
static int      arc_canvas_y = -1, arc_canvas_r = -1;

static void bars_cache_reset() {
    memset(bar_c, 0, sizeof bar_c);
    memset(bar_o, 0, sizeof bar_o);
    lab_max = lab_y = lab_r = -1;
    last_rpm = -1; last_rpm_c = 0; last_sc = 0;
}

static void arc_cache_reset() {
    arc_lab_max = arc_lab_y = arc_lab_r = -1;
    arc_last_rpm_c = 0;
    arc_canvas_rpm = INT32_MIN; arc_canvas_max = -1;
    arc_canvas_y = -1; arc_canvas_r = -1;
}

// Redraws every dash into arc_canvas in one pass: plain lines (a native,
// non-rotated draw primitive) into a single buffer that LVGL then composites
// as one flat image, instead of 56 individually transformed widgets. Skips
// the work entirely when neither rpm nor the thresholds moved since last call.
static void arc_dial_redraw(int rpm, int max) {
    if (!arc_canvas_buf) return;
    if (rpm == arc_canvas_rpm && max == arc_canvas_max &&
        (int)cfg.rpmYellow == arc_canvas_y && (int)cfg.rpmRed == arc_canvas_r) return;
    arc_canvas_rpm = rpm; arc_canvas_max = max;
    arc_canvas_y = cfg.rpmYellow; arc_canvas_r = cfg.rpmRed;

    lv_layer_t layer;
    lv_canvas_init_layer(arc_canvas, &layer);
    lv_canvas_fill_bg(arc_canvas, lv_color_hex(0x030406), LV_OPA_COVER);   // matches the face's outer tone

    const float f = (float)rpm / max;
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.width = ARC_SEG_W;
    dsc.opa   = LV_OPA_COVER;

    const int inner = ARC_R_DASH_OUT - ARC_SEG_H, outer = ARC_R_DASH_OUT;
    for (int i = 0; i < N_ARC_SEG; i++) {
        float p = (i + 0.5f) / N_ARC_SEG;
        float v = p * max;
        bool  lit = p <= f;
        uint32_t zc  = v >= cfg.rpmRed ? C_RED : (v >= cfg.rpmYellow ? C_YELLOW : C_WHITE);
        // Pre-blended "dim" shades (~the old unlit opacity over the face colour).
        uint32_t col = lit ? zc : (zc == C_RED ? 0x4a1311 : zc == C_YELLOW ? 0x4a3a04 : 0x232323);

        float theta = (-135.0f + 270.0f * p) * (float)M_PI / 180.0f;
        float sx = sinf(theta), cy_ = cosf(theta);
        dsc.p1.x = (lv_value_precise_t)lroundf(ARC_CANVAS_R + inner * sx);
        dsc.p1.y = (lv_value_precise_t)lroundf(ARC_CANVAS_R - inner * cy_);
        dsc.p2.x = (lv_value_precise_t)lroundf(ARC_CANVAS_R + outer * sx);
        dsc.p2.y = (lv_value_precise_t)lroundf(ARC_CANVAS_R - outer * cy_);
        dsc.color = lv_color_hex(col);
        lv_draw_line(&layer, &dsc);
    }
    lv_canvas_finish_layer(arc_canvas, &layer);
}

// Tick numbers (0, 2, 4, ... up to rpmMax/1000) placed around the dial. Only
// their text/colour/position depend on the thresholds, so this runs once at
// build time and again only when a setting actually moves them.
static void arc_labels_apply(int max) {
    int kMax = max / 1000;
    int idx = 0;
    for (int k = 0; k <= kMax && idx < N_ARC_LAB; k += 2, idx++) {
        float theta = (-135.0f + 270.0f * (k * 1000.0f) / max) * (float)M_PI / 180.0f;
        int lx = (int)lroundf(ARC_CX + ARC_LABEL_R * sinf(theta));
        int ly = (int)lroundf(ARC_CY - ARC_LABEL_R * cosf(theta));
        char buf[4]; snprintf(buf, sizeof buf, "%d", k);
        lv_label_set_text(arc_lab[idx], buf);
        uint32_t c = (k * 1000) >= cfg.rpmRed ? C_RED : ((k * 1000) >= cfg.rpmYellow ? C_YELLOW : C_WHITE);
        lv_obj_set_style_text_color(arc_lab[idx], lv_color_hex(c), 0);
        lv_obj_set_pos(arc_lab[idx], lx - 8, ly - 7);   // ~centre a 1-2 digit label
        lv_obj_clear_flag(arc_lab[idx], LV_OBJ_FLAG_HIDDEN);
    }
    for (; idx < N_ARC_LAB; idx++) lv_obj_add_flag(arc_lab[idx], LV_OBJ_FLAG_HIDDEN);
}

static void dash_update(lv_timer_t*) {
    live_data_t d;
#if MOTO_SIM
    d = moto_sim_sample();
#else
    // snapshot the volatile global
    noInterrupts();
    d = *(const live_data_t*)&live_data;
    interrupts();
    if (!d.valid) d = live_data_t{};  // all-zero until a sample arrives
#endif

    const int   max = cfg.rpmMax;
    int rpm = d.rpm; if (rpm > max) rpm = max; if (rpm < 0) rpm = 0;
    const float f = (float)rpm / max;
    const bool  blink = (millis() / 300) % 2 == 0;

    if (cfg.gaugeStyle == STYLE_BARS) {
        // (1) bars — touch a segment only when its colour OR opacity actually
        // changes, so each frame invalidates just the few bars crossing the lit
        // edge (plus any whose zone moved after a settings change) instead of
        // all 36.
        for (int i = 0; i < N_BARS; i++) {
            float p = (i + 0.5f) / N_BARS;
            float v = p * max;
            bool lit = p <= f;
            uint32_t zc = v >= cfg.rpmRed ? C_RED : (v >= cfg.rpmYellow ? C_YELLOW : C_WHITE);
            lv_opa_t opa = lit ? LV_OPA_COVER : (zc == C_WHITE ? (lv_opa_t)31 : (lv_opa_t)71);
            if (bar_c[i] != zc)  { lv_obj_set_style_bg_color(bar[i], lv_color_hex(zc), 0); bar_c[i] = zc; }
            if (bar_o[i] != opa) { lv_obj_set_style_bg_opa(bar[i], opa, 0);                bar_o[i] = opa; }
        }

        // scale labels depend only on the thresholds — rebuild only when they move.
        if (lab_max != max || lab_y != (int)cfg.rpmYellow || lab_r != (int)cfg.rpmRed) {
            lab_max = max; lab_y = cfg.rpmYellow; lab_r = cfg.rpmRed;
            int kMax = max / 1000;
            for (int i = 0, k = 0; i < N_BARLAB; i++, k += 2) {
                if (k <= kMax) {
                    char b[4]; snprintf(b, sizeof b, "%d", k);
                    lv_label_set_text(barlab[i], b);
                    uint32_t c = (k * 1000) >= cfg.rpmRed ? C_RED : ((k * 1000) >= cfg.rpmYellow ? C_YELLOW : C_GRAY);
                    lv_obj_set_style_text_color(barlab[i], lv_color_hex(c), 0);
                    lv_obj_clear_flag(barlab[i], LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_add_flag(barlab[i], LV_OBJ_FLAG_HIDDEN);
                }
            }
        }

        int rpm_disp = rpm;
        if (rpm_disp != last_rpm) {
            char rbuf[8]; snprintf(rbuf, sizeof rbuf, "%d", rpm_disp);
            lv_label_set_text(rpm_val, rbuf);
            last_rpm = rpm_disp;
        }
        uint32_t rc = rpm_zone(rpm);
        if (rc != last_rpm_c) { lv_obj_set_style_text_color(rpm_val, lv_color_hex(rc), 0); last_rpm_c = rc; }

        // shift indicator — only repaint when the state flips (design 1b only;
        // 1a's dial has no shift light).
        int shiftAt = min((int)cfg.shiftRpm, max);
        bool shiftLit = cfg.shiftEnabled && rpm >= shiftAt &&
                        (!cfg.shiftBlink || (millis() / 120) % 2 == 0);
        uint32_t sc = shiftLit ? C_RED : 0x333333;
        if (sc != last_sc) {
            lv_obj_set_style_bg_color(shift_dot, lv_color_hex(sc), 0);
            lv_obj_set_style_shadow_color(shift_dot, lv_color_hex(sc), 0);
            lv_obj_set_style_text_color(shift_lbl, lv_color_hex(sc), 0);
            last_sc = sc;
        }
    } else {
        // Design 1a — segmented arc tach, redrawn into one canvas (see
        // arc_dial_redraw()); no-ops on its own if rpm/thresholds didn't move.
        arc_dial_redraw(rpm, max);

        // Tick numbers depend only on the thresholds — reposition/recolour only
        // when they move.
        if (arc_lab_max != max || arc_lab_y != (int)cfg.rpmYellow || arc_lab_r != (int)cfg.rpmRed) {
            arc_lab_max = max; arc_lab_y = cfg.rpmYellow; arc_lab_r = cfg.rpmRed;
            arc_labels_apply(max);
        }

        int rpm_disp = rpm;
        if (rpm_disp != last_rpm) {
            char rbuf[8]; snprintf(rbuf, sizeof rbuf, "%d", rpm_disp);
            lv_label_set_text(arc_rpm_val, rbuf);
            last_rpm = rpm_disp;
        }
        uint32_t rc = rpm_zone(rpm);
        if (rc != arc_last_rpm_c) { lv_obj_set_style_text_color(arc_rpm_val, lv_color_hex(rc), 0); arc_last_rpm_c = rc; }
    }

    // slots
    for (int i = 0; i < 3; i++) slot_render(i, d);

    // DTC badge / list
    dtc_update();

    // warnings — the border is full-screen, so only (un)hide it when the shown
    // state actually changes (0 = hidden). Avoids a full-screen invalidate/frame.
    bool overheat = d.valid && active_temp(d) > cfg.tempRed;
    uint32_t wc = overheat ? C_RED : 0;
    uint32_t shown = (wc && !menu_open && !dtc_open && blink) ? wc : 0;
    static uint32_t last_shown = 0xFFFFFFFF;
    if (shown != last_shown) {
        if (shown) {
            lv_obj_set_style_border_color(warn_border, lv_color_hex(shown), 0);
            lv_obj_clear_flag(warn_border, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(warn_border, LV_OBJ_FLAG_HIDDEN);
        }
        last_shown = shown;
    }
}

// =============================================================================
//  Gauge style switch (Design 1a arc tach <-> Design 1b bar graph)
// =============================================================================
// Tears down whichever gauge/slot widgets are on screen and rebuilds the
// style named by cfg.gaugeStyle. Cheap enough to call straight from the menu
// (not just at boot) since it's only ever triggered by a deliberate settings
// change, never per-frame.
static void build_gauge_style() {
    if (gauge_root) { lv_obj_delete(gauge_root); gauge_root = nullptr; }
    if (slots_root) { lv_obj_delete(slots_root); slots_root = nullptr; }
    memset(bar, 0, sizeof bar); memset(barlab, 0, sizeof barlab);
    rpm_ghost = rpm_val = shift_dot = shift_lbl = nullptr;
    arc_canvas = nullptr;   // deleted along with gauge_root; arc_canvas_buf itself is kept and reused
    memset(arc_lab, 0, sizeof arc_lab);
    arc_rpm_val = nullptr;
    bars_cache_reset();
    arc_cache_reset();

    gauge_root = bare(scr);
    lv_obj_set_pos(gauge_root, 0, 0);
    lv_obj_set_size(gauge_root, 320, 240);
    slots_root = bare(scr);
    lv_obj_set_pos(slots_root, 0, 0);
    lv_obj_set_size(slots_root, 320, 240);

    if (cfg.gaugeStyle == STYLE_BARS) {
        build_bars(gauge_root);
        build_rpm(gauge_root);
        build_slots_row(slots_root);
    } else {
        build_arc(gauge_root);
        arc_labels_apply(cfg.rpmMax);
        build_slots_col(slots_root);
    }

    // Overlays (if already built) must stay above the freshly (re)created
    // gauge/slots layer.
    if (warn_border)   lv_obj_move_foreground(warn_border);
    if (menu_overlay)  lv_obj_move_foreground(menu_overlay);
    if (dtc_overlay)   lv_obj_move_foreground(dtc_overlay);
}

// =============================================================================
//  Long-press -> open settings
// =============================================================================
static void scr_longpress_cb(lv_event_t*) {
    if (menu_open || dtc_open) return;
    if (!menu_overlay) build_menu();
    menu_open = true;
    menu_cur = -1;
    root_page = 0;
    gauge_sub = -1;
    menu_refresh();
    lv_obj_clear_flag(menu_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(menu_overlay);
}

// =============================================================================
//  Public entry
// =============================================================================
void moto_dash_create(void) {
    cfg_load();

    scr = lv_screen_active();
    lv_obj_remove_style_all(scr);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(scr, lv_color_hex(C_BLACK), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);

    build_gauge_style();
    build_warn();

    // ~0.6 s long-press to open Settings (matches the design gesture).
    lv_indev_t* indev = touch_get_indev();
    if (indev) lv_indev_set_long_press_time(indev, 600);
    lv_obj_add_event_cb(scr, scr_longpress_cb, LV_EVENT_LONG_PRESSED, nullptr);

    // UI refresh period. 16 ms targets ~60 Hz animation (matches LVGL's
    // LV_DEF_REFR_PERIOD). Raise toward 33 ms (30 Hz) if core 1 gets starved —
    // real ECU data only arrives ~10-20 Hz anyway, so 30 Hz looks identical for
    // live use and halves the render load. Only the sim sweep benefits from 60.
    lv_timer_create(dash_update, 33, nullptr);
}
