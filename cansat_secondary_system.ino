/****************************************
 *  Board: Adafruit Feather RP2040 RFM  *
 ****************************************/
#ifdef CANSAT_SECONDARY_SYSTEM

#ifndef ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM
#error "Select Adafruit Feather RP2040 RFM board for the secondary system!"
#endif

#include "rsd_bmp390.h"
#include "rsd_environment.h"
#include "rsd_helper.h"
#include "rsd_lora.h"
#include "rsd_lora_868.h"
#include "rsd_lora_rp2040.h"
#include "rsd_sensors.h"


/**********************
 *  Global variables  *
 **********************/
BMP390 bmp;
RH_RF95 rf95(RF95_CS, RF95_INT);
bool radio_ok = false;
queue_t radio_queue;


/***********
 *  Setup  *
 ***********/
void setup() {
  #ifdef RSD_DEBUG
  Serial.begin(115200);
  for (int i = 0; i < 3; i++)
    if (!Serial) delay(1000);
  if (Serial) Serial.println("Serial INIT OK");
  #endif

  // Initialize I2C communication.
  Wire.begin();
  Wire.setClock(400000);

  bmp.initialize();

  radio_ok = setup_lora(
    rf95, RF95_RST, RF95_FREQ_MHZ,
    RF95_TX_POWER_DBM, RF95_SPREADING_FACTOR,
    RF95_BANDWIDTH_HZ, RF95_CODING_RATE);

  // The built-in red LED will be ON while the radio is transmitting.
  pinMode(LED_BUILTIN, OUTPUT);

  queue_init(&radio_queue, sizeof(SensorMeasurement), 100);
}


/*****************************
 *  Task Running on Core 0:  *
 *  BMP Measurement          *
 *****************************/
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


/*****************************
 *  Task Running on Core 1:  *
 *  Radio transmission       *
 *****************************/
void loop1() {
  static uint32_t radio_timer = millis();
  uint32_t now = millis();

  if (radio_ok && now > radio_timer) {
    radio_timer += 1000;
    send_radio_message();
  }
}


/************************
 *  Send Radio Message  *
 ************************/
void send_radio_message() {
  uint8_t message[RH_RF95_MAX_MESSAGE_LEN] = {0};
  SensorMeasurement measurement;
  std::vector<SensorMeasurement> measurements;
  static uint16_t transmission_id = 1;
  static uint16_t transmission_duration = 0;

  // Send transmission counter and previous transmission
  // duration as the first measurement package.
  measurement.type = RADIO_TRANSMISSION;
  measurement.timestamp = millis();
  measurement.value.radio_transmission.id = transmission_id;
  measurement.value.radio_transmission.previous_transmission_duration = transmission_duration;
  transmission_id++;
  measurements.push_back(measurement);

  // Get all measurements from the queue.
  while (queue_try_remove(&radio_queue, &measurement)) {
    measurements.push_back(measurement);
  }

  // Create the transmission payload.
  uint16_t size = 0;
  uint16_t sent_measurement_counter = 0;
  for (const auto& m : measurements) {
    if (size > RH_RF95_MAX_MESSAGE_LEN - sizeof(SensorMeasurement))
      break;
    size += m.add_to_radio_message(message+size);
    sent_measurement_counter++;
  }

  // Switch built-in red LED on.
  digitalWrite(LED_BUILTIN, HIGH);

  // Radio transmission.
  uint32_t start = millis();
  rf95.send(message, size);
  delay(10);
  rf95.waitPacketSent();
  transmission_duration = static_cast<uint16_t>(millis() - start);
  digitalWrite(LED_BUILTIN, LOW);  // Switch LED off.

  #ifdef RSD_DEBUG
  Serial.printf("Message %d: Sending %u bytes, %u measurements out of %u queued took %u ms.\n",
                transmission_id, size, sent_measurement_counter,
                measurements.size(), transmission_duration);
  #endif
}

#endif