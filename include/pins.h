// =============================================================================
//  pins.h  —  Pin assignments for ESP32_OBDKIT V1
// =============================================================================
//  MCU : ESP32-S3-WROOM-1-N4R2 (U7)
//  Auto-derived from KiCad schematic:  C:\Hobby\ESP32_OBD\V1\V1.kicad_sch
//
//  Peripheral summary
//    - LCD   : SPI TFT (ST7789-class)      U ... driven from SPI bus
//    - Touch : Resistive/SPI (XPT2046-class)
//    - CAN   : TJA1050 transceiver (U5)  -> OBD-II CAN bus (J2)
//    - K-line: 2x TLP291 optocoupler (U1 TX / U2 RX) -> OBD-II K-line (J2)
//    - USB   : Native USB (D+/D-) on the ESP32-S3
// =============================================================================
#pragma once

// -----------------------------------------------------------------------------
//  Display  (SPI TFT)
// -----------------------------------------------------------------------------
#define PIN_LCD_CS    10   // U7 pin18  IO10
#define PIN_LCD_MOSI  13   // U7 pin19  IO11  (SDA / DIN)
#define PIN_LCD_SCK   12   // U7 pin20  IO12  (SCL / CLK)
#define PIN_LCD_MISO  11   // U7 pin21  IO13
#define PIN_LCD_DC    14   // U7 pin22  IO14  (data/command)
#define PIN_LCD_RST   21   // U7 pin23  IO21  (reset)
#define PIN_LCD_BL    17   // U7 pin10  IO17  (backlight -> AP3019 CTRL / U8)

// -----------------------------------------------------------------------------
//  Touch controller  (SPI, XPT2046-class)
// -----------------------------------------------------------------------------
#define PIN_TOUCH_IRQ 38   // U7 pin31  IO38  (pen interrupt)
#define PIN_TOUCH_CS  39   // U7 pin32  IO39  (MTCK)
#define PIN_TOUCH_DIN 40   // U7 pin33  IO40  (MTDO) - MOSI to touch
#define PIN_TOUCH_DO  41   // U7 pin34  IO41  (MTDI) - MISO from touch
#define PIN_TOUCH_CLK 42   // U7 pin35  IO42  (MTMS)

// -----------------------------------------------------------------------------
//  CAN bus  (TJA1050 transceiver -> OBD-II)
// -----------------------------------------------------------------------------
#define PIN_CAN_RX     4   // U7 pin4   IO4  -> TJA1050 RXD (U5 pin4)
#define PIN_CAN_TX     5   // U7 pin5   IO5  -> TJA1050 TXD (U5 pin1)

// -----------------------------------------------------------------------------
//  K-line  (ISO 9141 / ISO 14230, via TLP291 optocouplers)
//    TX1 -> U1 (drives K-line low), RX1 <- U2 (reads K-line)
// -----------------------------------------------------------------------------
#define PIN_KLINE_TX   8   // U7 pin12  IO8  -> TLP291 (U1) input
#define PIN_KLINE_RX   9   // U7 pin17  IO9  <- TLP291 (U2) output

// -----------------------------------------------------------------------------
//  USB  (native USB-Serial/JTAG on ESP32-S3)
// -----------------------------------------------------------------------------
#define PIN_USB_DM    19   // U7 pin13  USB_D-/IO19
#define PIN_USB_DP    20   // U7 pin14  USB_D+/IO20

// -----------------------------------------------------------------------------
//  System / strapping
// -----------------------------------------------------------------------------
#define PIN_BOOT       0   // U7 pin27  IO0   (BOOT strap / user button)
// EN (U7 pin3) = hardware reset, not a GPIO.

// -----------------------------------------------------------------------------
//  Unused / spare GPIO (not connected in V1 schematic)
//    IO1, IO2, IO3, IO6, IO7, IO15, IO16, IO18, IO45, IO46, IO47, IO48,
//    IO43 (TXD0), IO44 (RXD0)
// -----------------------------------------------------------------------------
