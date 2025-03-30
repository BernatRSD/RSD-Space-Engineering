#ifndef RSD_LORA
#define RSD_LORA

#include <RH_RF95.h>


bool setup_lora(RH_RF95& radio, uint8_t reset_pin, float frequency_mhz,
                uint8_t power_dbm, uint8_t spreading_factor,
                long bandwidth_hz, uint8_t coding_rate) {
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, HIGH);
  delay(100);
  digitalWrite(reset_pin, LOW);
  delay(10);
  digitalWrite(reset_pin, HIGH);
  delay(10);
  bool radio_init_ok = false;
  for (int i = 0; i < 3; i++) {
    if (radio.init()) {
      radio_init_ok = true;
      break;
    }
    delay(100);
  }
  if (radio_init_ok) {
    #ifdef RSD_DEBUG
    Serial.println("LoRa INIT OK");
    #endif
    if (!radio.setFrequency(frequency_mhz)) {
      #ifdef RSD_DEBUG
      Serial.println("LoRa set freq FAILED");
      #endif
    }
    radio.setTxPower(power_dbm, false);
    radio.setSpreadingFactor(spreading_factor);
    radio.setSignalBandwidth(bandwidth_hz);
    radio.setCodingRate4(coding_rate);
  } else {
    #ifdef RSD_DEBUG
    Serial.println("LoRa INIT FAILED");
    #endif
  }
  return radio_init_ok;
}

#endif
