#include "ST7735_simple_driver.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

static const uint8_t PROGMEM // Чуток магии, без неё ни один ардуино-код не работает!
Rcmd1[] = {                       
  15,
  ST7735_SWRESET,   ST_CMD_DELAY,
  150,
  ST7735_SLPOUT,    ST_CMD_DELAY,
  255,
  ST7735_FRMCTR1, 3,
  0x01, 0x2C, 0x2D,
  ST7735_FRMCTR2, 3,
  0x01, 0x2C, 0x2D,
  ST7735_FRMCTR3, 6,
  0x01, 0x2C, 0x2D,
  0x01, 0x2C, 0x2D,
  ST7735_INVCTR,  1,
  0x07,
  ST7735_PWCTR1,  3,
  0xA2,
  0x02,
  0x84,
  ST7735_PWCTR2,  1,
  0xC5,
  ST7735_PWCTR3,  2,
  0x0A,
  0x00,
  ST7735_PWCTR4,  2,
  0x8A,
  0x2A,
  ST7735_PWCTR5,  2,
  0x8A, 0xEE,
  ST7735_VMCTR1,  1,
  0x0E,
  ST7735_INVOFF,  0,
  ST7735_MADCTL,  1,
  0xC8,
  ST7735_COLMOD,  1,
  0x05 },
      
Rcmd2red[] = {
  2,
  ST7735_CASET,   4,
  0x00, 0x00,
  0x00, 0x7F,
  ST7735_RASET,   4,
  0x00, 0x00,
  0x00, 0x9F },
      
Rcmd3[] = {
  4,
  ST7735_GMCTRP1, 16,
  0x02, 0x1c, 0x07, 0x12,
  0x37, 0x32, 0x29, 0x2d,
  0x29, 0x25, 0x2B, 0x39,
  0x00, 0x01, 0x03, 0x10,
  ST7735_GMCTRN1, 16,
  0x03, 0x1d, 0x07, 0x06,
  0x2E, 0x2C, 0x29, 0x2D,
  0x2E, 0x2E, 0x37, 0x3F,
  0x00, 0x00, 0x02, 0x10,
  ST7735_NORON,     ST_CMD_DELAY,
  10,
  ST7735_DISPON,    ST_CMD_DELAY,
  100 };         

#define SPI_START
#define SPI_END

// Макрос для быстрых переключений пинов DC/CS
#define DC_DATA    *dcPort |= dcMask
#define DC_COMMAND *dcPort &= ~dcMask
#define CS_IDLE    *csPort |= csMask
#define CS_ACTIVE  *csPort &= ~csMask

// Для экранов у которых пин CS всегда подключен к GND
#ifdef CS_ALWAYS_LOW
#define CS_IDLE
#define CS_ACTIVE
#endif

inline void Simple_ST7735::writeSPI(uint8_t c) 
{
    SPDR = c;
    asm volatile("nop"); // 8 NOPs кажется, достаточно для 16 МГц AVR @ DIV2, чтобы избежать использования цикла while
    asm volatile("nop"); // дада, либа для 16 МГц плат
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
}

// Этой функцией вы обозначаете пины экрана в скетче, атятя тут что то трогать! (даже если очень хочется)
Simple_ST7735::Simple_ST7735(int8_t dc, int8_t rst, int8_t cs) : Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT) 
{
  csPin = cs;
  dcPin = dc;
  rstPin = rst;
}

// инициализация дисплея
void Simple_ST7735::init() 
{ //чуток SPI-шной магии
  pinMode(dcPin, OUTPUT);
#ifndef CS_ALWAYS_LOW
  pinMode(csPin, OUTPUT);
#endif

dcPort = portOutputRegister(digitalPinToPort(dcPin));
dcMask = digitalPinToBitMask(dcPin);
#ifndef CS_ALWAYS_LOW
  csPort = portOutputRegister(digitalPinToPort(csPin));
  csMask = digitalPinToBitMask(csPin);
#endif

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE3);
  CS_ACTIVE;
  if(rstPin != -1) {
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, HIGH);
    delay(50);
    digitalWrite(rstPin, LOW);
    delay(50);
    digitalWrite(rstPin, HIGH);
    delay(50);
  }

  _colstart = 26; //оффсет картинки, вот тут трогать можно (если картинка выводится со смещением от 0,0 пикселя)
  _rowstart = 1; // по умолчанию - _colstart = 0; _rowstart = 0;
  _width  = ST7735_TFTWIDTH; //еще магии
  _height = ST7735_TFTHEIGHT;
  displayInit(Rcmd1);
  displayInit(Rcmd2red);
  displayInit(Rcmd3);
  //--------------------------------------------
  /* Тут тоже можно трогать, в том случае, если дисплей выводит что-то не то, например - зеркальный текст
   *  
   * за развёртку отвечают 4 бита - ST7735_MADCTL_MY,ST7735_MADCTL_MX,ST7735_MADCTL_MV,ST7735_MADCTL_ML
   * 
   * в случае чаго, можно поменять их в команде writeData(ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
   * 
   * !!! можно использовать не 1 бит,а хоть все 4 сразу writeData(ST7735_MADCTL_MY | ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_ML | ST7735_MADCTL_RGB);
   */
  writeCmd(ST7735_MADCTL);
  writeData(ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
  //--------------------------------------------
  _ystart = _colstart;
  _xstart = _rowstart;
  _width  = ST7735_TFTHEIGHT;
  _height = ST7735_TFTWIDTH;
  //Если экран выводит черный цвет как белый - его нужно инвертировать, 
  //ежеле с этой либой экран выводит белый как черный, поменяйте ST7735_INVON на ST7735_INVOFF
  writeCmd(ST7735_INVON);
}

//Еще SPI-шные магические заклинания
void Simple_ST7735::writeCmd(uint8_t c) 
{
  DC_COMMAND;
  CS_ACTIVE;
  SPI_START;

  writeSPI(c);

  CS_IDLE;
  SPI_END;
}
void Simple_ST7735::writeData(uint8_t c) 
{
  DC_DATA;
  CS_ACTIVE;
  SPI_START;
    
  writeSPI(c);

  CS_IDLE;
  SPI_END;
}

//инициализация дисплея (эта функция "загружает" в дисплей большое заклинание из начала файла) лучше тут ничего не тыркать
void Simple_ST7735::displayInit(const uint8_t *addr) 
{
  uint8_t  numCommands, numArgs;
  uint16_t ms;
  numCommands = pgm_read_byte(addr++);
  while(numCommands--) {
    writeCmd(pgm_read_byte(addr++));
    numArgs  = pgm_read_byte(addr++);
    ms       = numArgs & ST_CMD_DELAY;
    numArgs &= ~ST_CMD_DELAY;
    while(numArgs--) writeData(pgm_read_byte(addr++));

    if(ms) {
      ms = pgm_read_byte(addr++);
      if(ms == 255) ms = 500;
      delay(ms);
    }
  }
}

// "адресное окно" для операций с пикселями
void Simple_ST7735::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 
{
  uint16_t xs = x0 + _xstart, xe = x1 + _xstart;
  uint16_t ys = y0 + _ystart, ye = y1 + _ystart;
  
  CS_ACTIVE;
  SPI_START;
  
  DC_COMMAND; writeSPI(ST7735_CASET);
  DC_DATA;
  writeSPI(xs >> 8); writeSPI(xs & 0xFF);
  writeSPI(xe >> 8); writeSPI(xe & 0xFF);

  DC_COMMAND; writeSPI(ST7735_RASET);
  DC_DATA;
  writeSPI(ys >> 8); writeSPI(ys & 0xFF);
  writeSPI(ye >> 8); writeSPI(ye & 0xFF);

  DC_COMMAND; writeSPI(ST7735_RAMWR);
  
  CS_IDLE;
  SPI_END;
}
//рисует пиксель
void Simple_ST7735::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  if(x<0 ||x>=_width || y<0 || y>=_height) return;
  setAddrWindow(x,y,x+1,y+1);

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  writeSPI(color >> 8); writeSPI(color);

  CS_IDLE;
  SPI_END;
}

//макрос для заполнения экрана (облегчает жизнь)
void Simple_ST7735::fillScreen(uint16_t color) 
{
  fillRect(0, 0,  _width, _height, color);
}
//рисует заполненный прямоугольник
void Simple_ST7735::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
  if(x>=_width || y>=_height || w<=0 || h<=0) return;
  if(x+w-1>=_width)  w=_width -x;
  if(y+h-1>=_height) h=_height-y;
  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  uint32_t num = (uint32_t)w*h;
  uint16_t num16 = num>>4;
  while(num16--) {
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
  }
  uint8_t num8 = num & 0xf;
  while(num8--) { writeSPI(hi); writeSPI(lo); }

  CS_IDLE;
  SPI_END;
}
