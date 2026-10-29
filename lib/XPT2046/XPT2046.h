// =============================================================================
//  XPT2046.h  —  Minimal XPT2046 resistive-touch controller driver (SPI)
// =============================================================================
//  Bus   : its own SPIClass (keep it OFF the ST7789V display bus) @ <=2.5 MHz.
//  Output: raw 12-bit ADC readings (0..4095) for X and Y plus a pressure value.
//          Mapping raw->screen pixels is the calibration layer's job (touch.*).
//  Detect: active-low IRQ (pen-down) when wired; falls back to pressure.
// =============================================================================
#pragma once

#include <Arduino.h>
#include <SPI.h>

class XPT2046 {
public:
    XPT2046(SPIClass& spi, int8_t clk, int8_t mosi, int8_t miso,
            int8_t cs, int8_t irq = -1);

    void begin(uint32_t freq = 2000000UL);

    // Reads a debounced, median-filtered sample.
    // Returns true and fills x/y (raw 12-bit) + z (pressure) when touched.
    bool read(uint16_t& x, uint16_t& y, uint16_t& z);

    // Cheap pen-down check (IRQ line if present, else a single pressure read).
    bool touched();

    // Reject samples weaker than this pressure (tune per panel).
    void setPressureThreshold(uint16_t z) { _zThresh = z; }

private:
    uint16_t readChannel(uint8_t cmd);     // one 12-bit conversion
    uint16_t readMedian(uint8_t cmd);      // N reads -> median

    SPIClass& _spi;
    int8_t    _clk, _mosi, _miso, _cs, _irq;
    uint32_t  _freq    = 2000000UL;
    uint16_t  _zThresh = 350;
};
