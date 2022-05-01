const char scr0[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr1[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr2[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr3[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr4[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr5[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr6[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char scr7[] PROGMEM =
"" //В ЭТИХ СТРОЧКАХ МАКСИМУМ 26 СИМВОЛОВ!!!
""
""
""
""
""
""
""
""
""; //В ЭТОЙ СТРОЧКЕ МАКСИМУМ 20 СИМВОЛОВ!!!
const char* const names[] PROGMEM = {
  scr0,scr1,scr2, scr3,scr4,scr5,scr6,scr7,
};

#include <EncButton.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "ST7735_simple_driver.h"
// Пины экрана
#define TFT_CS 3
#define TFT_DC  A3
#define TFT_RST A2
// Пины кнопок
EncButton<EB_TICK, 9> up;
EncButton<EB_TICK, 10> down;

Simple_ST7735 lcd = Simple_ST7735(TFT_DC, TFT_RST, TFT_CS);

int list=0;
bool power;

char target[254 + 1] = "";
char *utf8rus(char *source)
{
  int i,j,k;
  unsigned char n;
  strcpy(target, ""); k = strlen(source); i = j = 0;
  while (i < k) {
    n = source[i]; i++;
    if (n >= 0xC0) {
      switch (n) {
        case 0xD0: {
          n = source[i]; i++;
          if (n == 0x81) { n = 0xA8; break; }
          if (n >= 0x90 && n <= 0xBF) n = n + 0x30;
          break;
        }
        case 0xD1: {
          n = source[i]; i++;
          if (n == 0x91) { n = 0xB8; break; }
          if (n >= 0x80 && n <= 0x8F) n = n + 0x70;
          break;
        }
      }
    }
    target[j] = n;
    if (j >= 234) break; j++; //ПРАВКА! ошибка от arduinec исправлена, буфер не переполняется
  }
  strcat(target, '\0');
  return target;
}
void screen_swipe(){
      lcd.clearScreen();
      lcd.setCursor(0,0);
      printPGM(list);
      lcd.setCursor(140,73);
      lcd.print(list+1);lcd.print("/8");
}
void swipe(bool operand){
  if(operand==0) list++;
  if(operand==1) list--;
  if(list>7) list=7;
  if(list<0) list=0;
  screen_swipe();
}

void printPGM(int idx) {
  PGM_P p = pgm_read_word(names + idx);
  char buf[strlen_P(p)];
  strcpy_P(buf, p);
  lcd.setCursor(0,0);
  lcd.print(utf8rus(buf));
  memset(buf, 0, sizeof(buf));
  memset(target, 0, sizeof(target));
}


void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2,HIGH);
  lcd.init();
  lcd.cp437(true);
  lcd.fillScreen(BLACK);
  lcd.setTextSize(1);
  lcd.setTextColor(WHITE);
  screen_swipe();
}
void loop() {
  up.tick();
  down.tick();
  if (up.click()) swipe(0);
  if (down.click()) swipe(1);
  if(down.held()) {if(power==0) {digitalWrite(2, LOW); power=1;} else {digitalWrite(2, HIGH); power=0;}}
}
