#ifndef RSD_SD_CARD
#define RSD_SD_CARD

#include <SdFat.h>
#include <SPI.h>

#define SD_CHIP_SELECT 10
#define SD_FREQUENCY 25  // MHz

SdFat sd;
bool sd_card_ok = false;

bool init_sd_card() {
  if (sd.begin(SD_CHIP_SELECT, SD_SCK_MHZ(SD_FREQUENCY))) {
    sd_card_ok = true;
    #ifdef RSD_DEBUG
    Serial.println("SD INIT OK");
    #endif
  } else {
    sd_card_ok = false;
    #ifdef RSD_DEBUG
    Serial.println("SD INIT FAILED");
    #endif
  }
  return sd_card_ok;
}

bool open_for_appending_or_writing(SdFile& file, const char* file_name) {
  if (!sd_card_ok) {
    #ifdef RSD_DEBUG
    Serial.printf("Attempted to open %s for writing but SD card is unavailable\n", file_name);
    #endif
    return false;
  } else if (sd.exists(file_name)) {
    if (file.open(file_name, O_APPEND | O_WRITE)) {
      #ifdef RSD_DEBUG
      Serial.printf("Opened %s for appending\n", file_name);
      #endif
      return true;
    } else {
      #ifdef RSD_DEBUG
      Serial.printf("Failed to open %s for appending\n", file_name);
      #endif
      return false;
    }
  } else {
    if (file.open(file_name, O_WRITE | O_CREAT)) {
      #ifdef RSD_DEBUG
      Serial.printf("Created and opened %s for writing\n", file_name);
      #endif
      return true;
    } else {
      #ifdef RSD_DEBUG
      Serial.printf("Failed to create and open %s for writing\n", file_name);
      #endif
      return false;
    }
  }
}

#endif