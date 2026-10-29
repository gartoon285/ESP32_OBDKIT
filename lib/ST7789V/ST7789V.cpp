// =============================================================================
//  ST7789V.cpp  —  Minimal ST7789V TFT driver (SPI + DMA via esp_lcd)
// =============================================================================
#include "ST7789V.h"

// --- ST7789 command set (subset) ---
#define ST_SWRESET 0x01
#define ST_SLPOUT  0x11
#define ST_NORON   0x13
#define ST_INVOFF  0x20
#define ST_INVON   0x21
#define ST_DISPON  0x29
#define ST_CASET   0x2A
#define ST_RASET   0x2B
#define ST_RAMWR   0x2C
#define ST_RAMWRC  0x3C   // memory write continue
#define ST_MADCTL  0x36
#define ST_COLMOD  0x3A
// Power / porch / gamma tuning
#define ST_PORCTRL   0xB2
#define ST_GCTRL     0xB7
#define ST_VCOMS     0xBB
#define ST_LCMCTRL   0xC0
#define ST_VDVVRHEN  0xC2
#define ST_VRHS      0xC3
#define ST_VDVS      0xC4
#define ST_FRCTRL2   0xC6
#define ST_PWCTRL1   0xD0
#define ST_PVGAMCTRL 0xE0
#define ST_NVGAMCTRL 0xE1

// MADCTL bits
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08   // set this if red/blue look swapped

ST7789V::ST7789V(spi_host_device_t host, int8_t sck, int8_t mosi, int8_t miso,
                 int8_t cs, int8_t dc, int8_t rst, int8_t bl)
    : _host(host), _sck(sck), _mosi(mosi), _miso(miso),
      _cs(cs), _dc(dc), _rst(rst), _bl(bl) {}

// Runs in the SPI ISR when the last DMA chunk of a tx_color() has gone out.
bool ST7789V::onColorDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*,
                          void* ctx) {
    ST7789V* self = static_cast<ST7789V*>(ctx);
    BaseType_t woken = pdFALSE;
    self->_busy = false;
    xSemaphoreGiveFromISR(self->_done, &woken);
    return woken == pdTRUE;
}

void ST7789V::waitIdle() {
    // Loop: a stale give from an earlier transfer must not end the wait early.
    while (_busy) xSemaphoreTake(_done, portMAX_DELAY);
}

void ST7789V::cmd(uint8_t c, const uint8_t* data, size_t n) {
    waitIdle();   // keep commands ordered after any in-flight pixel DMA
    esp_lcd_panel_io_tx_param(_io, c, data, n);
}

void ST7789V::begin(uint32_t freq, uint32_t maxPixelsPerTransfer) {
    _freq = freq;
    _done = xSemaphoreCreateBinary();

    if (_bl >= 0) {
        pinMode(_bl, OUTPUT);
        digitalWrite(_bl, LOW);   // keep off until the panel is initialised
    }

    spi_bus_config_t bus = {};
    bus.sclk_io_num     = _sck;
    bus.mosi_io_num     = _mosi;
    bus.miso_io_num     = _miso;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = maxPixelsPerTransfer * 2;
    ESP_ERROR_CHECK(spi_bus_initialize(_host, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io = {};
    io.cs_gpio_num         = _cs;
    io.dc_gpio_num         = _dc;
    io.spi_mode            = 0;
    io.pclk_hz             = _freq;
    io.trans_queue_depth   = 10;
    io.on_color_trans_done = onColorDone;
    io.user_ctx            = this;
    io.lcd_cmd_bits        = 8;
    io.lcd_param_bits      = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)_host, &io, &_io));

    // Hardware reset
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, HIGH); delay(10);
        digitalWrite(_rst, LOW);  delay(10);
        digitalWrite(_rst, HIGH); delay(120);
    }

    cmd(ST_SWRESET);
    delay(150);
    cmd(ST_SLPOUT);
    delay(120);

    cmd(ST_COLMOD, 0x55);         // 16-bit / RGB565
    delay(10);

    setRotation(0);              // sets MADCTL + w/h/offsets

    // --- Power / porch / gamma: many ST7789V panels stay WHITE without these ---
    { const uint8_t d[] = {0x0C, 0x0C, 0x00, 0x33, 0x33}; cmd(ST_PORCTRL, d, sizeof(d)); }
    cmd(ST_GCTRL,    0x35);
    cmd(ST_VCOMS,    0x19);
    cmd(ST_LCMCTRL,  0x2C);
    cmd(ST_VDVVRHEN, 0x01);
    cmd(ST_VRHS,     0x12);
    cmd(ST_VDVS,     0x20);
    cmd(ST_FRCTRL2,  0x0F);
    { const uint8_t d[] = {0xA4, 0xA1};             cmd(ST_PWCTRL1, d, sizeof(d)); }
    { const uint8_t d[] = {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23};
      cmd(ST_PVGAMCTRL, d, sizeof(d)); }
    { const uint8_t d[] = {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23};
      cmd(ST_NVGAMCTRL, d, sizeof(d)); }

    cmd(ST_INVOFF);              // this panel shows a negative image with INVON
                                 // (inversion is panel-specific: flip back to
                                 //  ST_INVON if colours come out complementary)
    delay(10);
    cmd(ST_NORON);
    delay(10);
    cmd(ST_DISPON);
    delay(120);

    fillScreen(0x0000);          // clear to black
}

void ST7789V::setRotation(uint8_t r) {
    r &= 3;
    uint8_t madctl;
    switch (r) {
        case 0:  madctl = MADCTL_RGB;                          _w = 240; _h = 320; break;
        case 1:  madctl = MADCTL_MV | MADCTL_MX | MADCTL_RGB;  _w = 320; _h = 240; break;
        case 2:  madctl = MADCTL_MX | MADCTL_MY | MADCTL_RGB;  _w = 240; _h = 320; break;
        default: madctl = MADCTL_MV | MADCTL_MY | MADCTL_RGB;  _w = 320; _h = 240; break;
    }
    // 240x320 panel sits at RAM origin, so offsets stay 0 for all rotations.
    _colstart = 0;
    _rowstart = 0;
    cmd(ST_MADCTL, madctl);
}

void ST7789V::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t x0 = x + _colstart;
    uint16_t x1 = x + w - 1 + _colstart;
    uint16_t y0 = y + _rowstart;
    uint16_t y1 = y + h - 1 + _rowstart;

    const uint8_t cbuf[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
    cmd(ST_CASET, cbuf, 4);
    const uint8_t rbuf[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
    cmd(ST_RASET, rbuf, 4);

    // RAMWR is sent together with the first pixel push (esp_lcd tx_color).
    _continue = false;
}

void ST7789V::pushPixelsAsync(const void* data, uint32_t lenPixels) {
    waitIdle();
    _busy = true;
    // First push after setAddrWindow() starts at the window origin (RAMWR);
    // later ones carry on where the previous left off (RAMWRC).
    const uint8_t c = _continue ? ST_RAMWRC : ST_RAMWR;
    _continue = true;
    if (esp_lcd_panel_io_tx_color(_io, c, data, lenPixels * 2) != ESP_OK) _busy = false;
}

void ST7789V::pushPixels(const void* data, uint32_t lenPixels) {
    pushPixelsAsync(data, lenPixels);
    waitIdle();
}

void ST7789V::fillScreen(uint16_t color) {
    // color arrives native; swap to big-endian for the wire
    uint16_t be = (color >> 8) | (color << 8);

    // The colour is constant, so one block is DMA'd repeatedly (both 240 and
    // 320 are multiples of LINES).
    static constexpr uint32_t LINES = 8;
    static uint16_t block[320 * LINES];   // large enough for either orientation
    const uint32_t n = (uint32_t)_w * LINES;
    for (uint32_t i = 0; i < n; i++) block[i] = be;

    setAddrWindow(0, 0, _w, _h);
    for (uint16_t y = 0; y < _h; y += LINES) pushPixelsAsync(block, n);
    waitIdle();
}

void ST7789V::backlight(bool on) {
    if (_bl >= 0) digitalWrite(_bl, on ? HIGH : LOW);
}

void ST7789V::backlightBrightness(uint8_t level) {
    if (_bl >= 0) analogWrite(_bl, level);   // AP3019 CTRL accepts a PWM dim signal
}
