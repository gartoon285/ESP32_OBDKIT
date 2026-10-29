// =============================================================================
//  XPT2046.cpp  —  Minimal XPT2046 resistive-touch controller driver (SPI)
// =============================================================================
#include "XPT2046.h"

// Control bytes (differential mode, 12-bit, power-down between conversions).
//   S=1, A2A1A0 select channel, MODE=0 (12-bit), SER/DFR=0 (differential).
#define CMD_X   0xD0   // A=101 -> X position
#define CMD_Y   0x90   // A=001 -> Y position
#define CMD_Z1  0xB0   // A=011 -> Z1
#define CMD_Z2  0xC0   // A=100 -> Z2

static constexpr uint8_t  N_SAMPLES = 7;    // odd -> clean median
static constexpr uint32_t ADC_MAX   = 4095;

XPT2046::XPT2046(SPIClass& spi, int8_t clk, int8_t mosi, int8_t miso,
                 int8_t cs, int8_t irq)
    : _spi(spi), _clk(clk), _mosi(mosi), _miso(miso), _cs(cs), _irq(irq) {}

void XPT2046::begin(uint32_t freq) {
    _freq = freq;

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    if (_irq >= 0) pinMode(_irq, INPUT_PULLUP);

    _spi.begin(_clk, _miso, _mosi, -1);   // CS handled manually

    // One throwaway conversion + enter power-down so IRQ is armed.
    digitalWrite(_cs, LOW);
    _spi.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));
    _spi.transfer(CMD_Y);          // leave PD=00 -> IRQ enabled
    _spi.transfer16(0);
    _spi.endTransaction();
    digitalWrite(_cs, HIGH);
}

uint16_t XPT2046::readChannel(uint8_t cmd) {
    _spi.transfer(cmd);
    // 12-bit result is left-aligned in the 16-bit frame -> shift down 3.
    uint16_t v = _spi.transfer16(0x0000);
    return v >> 3;
}

uint16_t XPT2046::readMedian(uint8_t cmd) {
    uint16_t s[N_SAMPLES];
    for (uint8_t i = 0; i < N_SAMPLES; i++) s[i] = readChannel(cmd);

    // insertion sort (N is tiny) -> take the middle element
    for (uint8_t i = 1; i < N_SAMPLES; i++) {
        uint16_t k = s[i];
        int8_t j = i - 1;
        while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
        s[j + 1] = k;
    }
    return s[N_SAMPLES / 2];
}

bool XPT2046::touched() {
    if (_irq >= 0) return digitalRead(_irq) == LOW;

    digitalWrite(_cs, LOW);
    _spi.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));
    uint16_t z1 = readChannel(CMD_Z1);
    uint16_t z2 = readChannel(CMD_Z2);
    _spi.endTransaction();
    digitalWrite(_cs, HIGH);
    int32_t z = (int32_t)z1 - (int32_t)z2 + ADC_MAX;
    return z > _zThresh;
}

bool XPT2046::read(uint16_t& x, uint16_t& y, uint16_t& z) {
    if (_irq >= 0 && digitalRead(_irq) != LOW) return false;

    digitalWrite(_cs, LOW);
    _spi.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));

    uint16_t z1 = readChannel(CMD_Z1);
    uint16_t z2 = readChannel(CMD_Z2);
    int32_t pressure = (int32_t)z1 - (int32_t)z2 + ADC_MAX;

    uint16_t rx = 0, ry = 0;
    if (pressure > _zThresh) {
        // Read X then Y (median-filtered). Order X-before-Y settles the DAC.
        rx = readMedian(CMD_X);
        ry = readMedian(CMD_Y);
    }

    _spi.endTransaction();
    digitalWrite(_cs, HIGH);

    if (pressure <= _zThresh) return false;
    // Guard against the rail readings the panel emits at pen lift-off.
    if (rx == 0 || ry == 0 || rx >= ADC_MAX || ry >= ADC_MAX) return false;

    x = rx; y = ry; z = (uint16_t)pressure;
    return true;
}
