// =============================================================================
//  ST7789V.h  —  Minimal ST7789V TFT driver (SPI + DMA) for ESP32_OBDKIT V1
// =============================================================================
//  Panel : 240 x 320 IPS, SPI, native offset 0,0
//  Bus   : ESP-IDF esp_lcd panel IO on a dedicated SPI host, DMA-backed.
//          pushPixels() blocks until the transfer is done; pushPixelsAsync()
//          returns immediately and the DMA runs in the background (wait with
//          waitIdle()).
//  Pixels are expected big-endian (RGB565, high byte first). LVGL's
//  lv_draw_sw_rgb565_swap() takes care of that before pushPixels().
// =============================================================================
#pragma once

#include <Arduino.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <freertos/semphr.h>

class ST7789V {
public:
    ST7789V(spi_host_device_t host, int8_t sck, int8_t mosi, int8_t miso,
            int8_t cs, int8_t dc, int8_t rst, int8_t bl);

    // freq in Hz. 40 MHz is safe on most modules; many ST7789V panels
    // handle 80 MHz if the wiring is short.
    // maxPixelsPerTransfer sizes the DMA descriptor chain (largest pushPixels).
    void begin(uint32_t freq = 40000000UL, uint32_t maxPixelsPerTransfer = 320 * 40);

    void setRotation(uint8_t r);                 // 0..3
    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    void pushPixels(const void* data, uint32_t lenPixels);       // blocking
    // Non-blocking: `data` must stay valid and untouched until waitIdle()
    // returns. Successive pushes after one setAddrWindow() continue the window.
    void pushPixelsAsync(const void* data, uint32_t lenPixels);
    void waitIdle();                             // block until DMA finished
    void fillScreen(uint16_t color);             // color is native RGB565

    void backlight(bool on);                     // simple on/off
    void backlightBrightness(uint8_t level);     // 0..255 (PWM via AP3019 CTRL)

    uint16_t width()  const { return _w; }
    uint16_t height() const { return _h; }

private:
    void cmd(uint8_t c, const uint8_t* data = nullptr, size_t n = 0);
    void cmd(uint8_t c, uint8_t data) { cmd(c, &data, 1); }
    static bool onColorDone(esp_lcd_panel_io_handle_t io,
                            esp_lcd_panel_io_event_data_t* e, void* ctx);

    spi_host_device_t _host;
    esp_lcd_panel_io_handle_t _io = nullptr;
    SemaphoreHandle_t _done = nullptr;
    volatile bool _busy = false;
    bool _continue = false;      // next push uses RAMWRC instead of RAMWR

    int8_t   _sck, _mosi, _miso, _cs, _dc, _rst, _bl;
    uint32_t _freq   = 40000000UL;
    uint16_t _w      = 240;
    uint16_t _h      = 320;
    uint16_t _colstart = 0;
    uint16_t _rowstart = 0;
};
