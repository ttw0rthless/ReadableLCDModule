/*
 распиновка ST7735:
  SCK   -> D13
  SDA   -> D11/MOSI
  A0/DC/RS -> D8  or any digital
  RESET -> D9  or any digital
  CS    -> D10 or any digital
  GND   -> GND
  VCC   -> 3.3V
*/

#ifndef _ST7735_SIMPLE_H_
#define _ST7735_SIMPLE_H_

#include "Arduino.h"
#include "Print.h"
#include <Adafruit_GFX.h>
#include <avr/pgmspace.h>

#define ST7735_TFTWIDTH 	80
#define ST7735_TFTHEIGHT 	160

// регистры для экранчика
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_FRMCTR1    0xB1
#define ST7735_FRMCTR2    0xB2
#define ST7735_FRMCTR3    0xB3
#define ST7735_INVCTR     0xB4
#define ST7735_DISSET5    0xB6

#define ST7735_PWCTR1     0xC0
#define ST7735_PWCTR2     0xC1
#define ST7735_PWCTR3     0xC2
#define ST7735_PWCTR4     0xC3
#define ST7735_PWCTR5     0xC4
#define ST7735_VMCTR1     0xC5

#define ST7735_PWCTR6     0xFC

#define ST7735_GMCTRP1    0xE0
#define ST7735_GMCTRN1    0xE1



#define ST_CMD_DELAY   0x80

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01

#define ST7735_SLPIN   0x10  // sleep on
#define ST7735_SLPOUT  0x11  // sleep off
#define ST7735_PTLON   0x12  // partial on
#define ST7735_NORON   0x13  // partial off
#define ST7735_INVOFF  0x20  // invert off
#define ST7735_INVON   0x21  // invert on
#define ST7735_DISPOFF 0x28  // display off
#define ST7735_DISPON  0x29  // display on
#define ST7735_IDMOFF  0x38  // idle off
#define ST7735_IDMON   0x39  // idle on

#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_PTLAR    0x30
#define ST7735_VSCRDEF  0x33
#define ST7735_VSCRSADD 0x37

#define ST7735_WRDISBV  0x51
#define ST7735_WRCTRLD  0x53
#define ST7735_WRCACE   0x55
#define ST7735_WRCABCMB 0x5e

#define ST7735_POWSAVE    0xbc
#define ST7735_DLPOFFSAVE 0xbd

// 4 бита madctl
#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00


//Перевод 16-битных цветов на человеческий язык
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

class Simple_ST7735 : public Adafruit_GFX {

 public:
  Simple_ST7735(int8_t DC, int8_t RST, int8_t CS = -1);
  void init();
  void begin() { init(); }
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void fillScreen(uint16_t color=BLACK);
  void clearScreen() { fillScreen(BLACK); }
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void resetDisplay();
 protected:
  uint8_t  _colstart, _rowstart, _xstart, _ystart;

  void displayInit(const uint8_t *addr);
  void writeSPI(uint8_t);
  void writeCmd(uint8_t c);
  void writeData(uint8_t d);

 private:
  int8_t  csPin, dcPin, rstPin;
  uint8_t  csMask, dcMask;
  volatile uint8_t  *csPort, *dcPort;

};

#endif
