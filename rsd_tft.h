#ifndef RSD_TFT
#define RSD_TFT

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

void setup_tft(Adafruit_ST7789& display, uint8_t text_size) {
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);
  display.init(135, 240);
  display.setRotation(3);
  display.setTextWrap(true);
  display.setTextSize(text_size);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.fillScreen(ST77XX_BLACK);
}

void clear_tft(Adafruit_ST7789& display) {
  display.fillScreen(ST77XX_BLACK);
}

#endif
