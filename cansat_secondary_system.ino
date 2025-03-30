#ifdef CANSAT_SECONDARY_SYSTEM

#include <optional>
#include <sstream>

#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

#include "rsd_bmp390.h"
#include "rsd_environment.h"
#include "rsd_helper.h"
#include "rsd_lora.h"
#include "rsd_lora_868.h"
#include "rsd_lora_rp2040.h"
#include "rsd_sensors.h"


// #define RSD_DEBUG

BMP390 bmp(0x77);
RH_RF95 rf95(RF95_CS, RF95_INT);
bool radio_ok = false;
queue_t radio_queue;



/****************************
 *  Task Running on Core 1  *
 ****************************/
void task1() {
  uint32_t now;
  uint32_t radio_timer = millis();

  while (true) {
    now = millis();
    if (radio_ok && now > radio_timer) {
      radio_timer += 1000;
      send_radio_message();
    }
  }
}


void setup() {
  #ifdef RSD_DEBUG
  Serial.begin(115200);
  while (!Serial) delay(10);
  #endif

  // Initialize I2C communication.
  Wire.begin();
  Wire.setClock(400000);

  setup_bmp();
  radio_ok = setup_lora(
    rf95, RF95_RST, RF95_FREQ_MHZ,
    RF95_TX_POWER_DBM, RF95_SPREADING_FACTOR,
    RF95_BANDWIDTH_HZ, RF95_CODING_RATE);

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the queue to hold up to 10 integers
  queue_init(&radio_queue, sizeof(SensorMeasurement), 100);

  multicore_launch_core1(task1);
}

void loop() {
  static uint32_t bmp_timer = millis();
  uint32_t now = millis();
  SensorMeasurement measurement;
  auto& tpa = measurement.value.tpa;

  if (now >= bmp_timer) {
    bmp_timer += 80;
    if (tpa.read(bmp, SEA_LEVEL_PRESSURE)) {
      measurement.type = TEMPERATURE_PRESSURE_ALTITUDE;
      measurement.timestamp = now;
      queue_try_add(&radio_queue, &measurement);
      #ifdef RSD_DEBUG
      Serial.printf("%s\n", measurement.as_string().c_str());
      #endif
    }
  }
}

void send_radio_message() {
  uint8_t message[RH_RF95_MAX_MESSAGE_LEN] = {0};
  SensorMeasurement measurement;
  std::vector<SensorMeasurement> measurements;
  static uint16_t transmission_id = 1;

  measurement.type = RADIO_TRANSMISSION;
  measurement.timestamp = millis();
  measurement.value.radio_transmission.id = transmission_id;
  transmission_id++;
  measurements.push_back(measurement);
  while (queue_try_remove(&radio_queue, &measurement)) {
    measurements.push_back(measurement);
  }

  uint16_t size = 0;
  uint16_t sent_measurement_counter = 0;
  for (const auto& m : measurements) {
    if (size > RH_RF95_MAX_MESSAGE_LEN - sizeof(SensorMeasurement))
      break;
    size += m.add_to_radio_message(message+size);
    sent_measurement_counter++;
  }
  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t start = millis();
  rf95.send(message, size);
  delay(10);
  rf95.waitPacketSent();
  uint32_t finish = millis();
  digitalWrite(LED_BUILTIN, LOW);
  #ifdef RSD_DEBUG
  Serial.printf("Message %d: Sending %u/%u measurements in %u bytes took %u ms.\n",
                transmission_id, sent_measurement_counter,
                measurements.size(), size, finish-start);
  #endif
}

void setup_bmp() {
  if (bmp.initialize()) {
    #ifdef RSD_DEBUG
    Serial.printf("BMP390 init success\n");
    #endif
  } else {
    #ifdef RSD_DEBUG
    Serial.printf("BMP390 init failed\n");
    #endif
  }
}

#endif