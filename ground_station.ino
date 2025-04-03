/**************************************************
 *  Board: Adafruit Feather ESP32-S3 Reverse TFT  *
 **************************************************/
#ifdef GROUND_STATION

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
#error "Select Adafruit Feather ESP32-S3 Reverse TFT board for the primary system!"
#endif

// Select frequency for ground station.
// #define IS_433_MHZ
#define IS_868_MHZ

#include "rsd_helper.h"
#include "rsd_lora.h"
#ifdef IS_433_MHZ
#include "rsd_lora_433.h"
#else
#include "rsd_lora_868.h"
#endif
#include "rsd_lora_wing.h"
#include "rsd_sd_card.h"
#include "rsd_sensors.h"
#include "rsd_sensor_logging.h"
#include "rsd_tft.h"

/**********************
 *  Global Variables  *
 **********************/
// Display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// LoRa radio
RH_RF95 rf95(RF95_CS, RF95_INT);
bool radio_ok = false;
// Queues
QueueHandle_t sd_card_queue;
QueueHandle_t tft_queue;


void setup() {
#ifdef RSD_DEBUG
  Serial.begin(115200);
  for (int i = 0; i < 3; i++)
    if (!Serial) delay(100);
  if (Serial) Serial.println("Serial INIT OK");
  Serial.printf("RESET REASON: %d\n", esp_reset_reason());
#endif

  sd_card_queue = xQueueCreate(200, sizeof(SensorMeasurement));
  tft_queue = xQueueCreate(200, sizeof(SensorMeasurement));

  setup_tft(tft, 2);
  radio_ok = setup_lora(rf95, RF95_RST, RF95_FREQ_MHZ,
                        RF95_TX_POWER_DBM, RF95_SPREADING_FACTOR,
                        RF95_BANDWIDTH_HZ, RF95_CODING_RATE);
  if (radio_ok)
    tft.println("LoRa INIT OK");
  else
    tft.println("LoRa INIT FAILED");

  // Initialize SD card.
  #ifdef IS_433_MHZ
  const std::vector<SensorType>& sensor_types{all_sensor_types()};
  #else
  const std::vector<SensorType>& sensor_types{TEMPERATURE_PRESSURE_ALTITUDE, RADIO_TRANSMISSION, RADIO_RECEPTION};
  #endif
  if (initialize_log_files(sensor_types)) {
    tft.println("SD card INIT OK");
#ifdef RSD_DEBUG
    Serial.println("SD card INIT OK");
#endif
  } else {
    tft.println("SD card INIT FAILED");
#ifdef RSD_DEBUG
    Serial.println("SD card INIT FAILED");
#endif
  }
  delay(1000);
  clear_tft(tft);
}

void loop() {
  SensorMeasurement measurement;
  std::vector<SensorMeasurement> measurements;

  receive_transmission();
  yield();
  update_tft();
  yield();
  while (xQueueReceive(sd_card_queue, &measurement, pdMS_TO_TICKS(5)) == pdPASS)
    measurements.push_back(measurement);
  write_to_sd_card(measurements);
  yield();
}

bool receive_transmission() {
  if (rf95.waitAvailableTimeout(1500)) {
    uint8_t message[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(message);
    if (rf95.recv(message, &len)) {
      uint32_t now = millis();
#ifdef RSD_DEBUG
      Serial.printf("Received %u bytes long message\n", len);
#endif
      SensorMeasurement measurement;
      size_t size = 0;
      uint8_t number_of_messages = 0;
      while (size < len) {
        size += measurement.read_from_radio_message(message + size);
        xQueueSendToBack(tft_queue, &measurement, pdMS_TO_TICKS(10));
        xQueueSendToBack(sd_card_queue, &measurement, pdMS_TO_TICKS(10));
        number_of_messages++;
#ifdef RSD_DEBUG
        Serial.println(measurement.as_string().c_str());
#endif
      }
      measurement.type = RADIO_RECEPTION;
      measurement.timestamp = now;
      measurement.value.radio_reception = { len, number_of_messages, rf95.lastRssi() };
      xQueueSendToBack(tft_queue, &measurement, pdMS_TO_TICKS(10));
      xQueueSendToBack(sd_card_queue, &measurement, pdMS_TO_TICKS(10));
      return true;
    }
  } else {
#ifdef RSD_DEBUG
    Serial.println("No radio reception");
#endif
  }
  return false;
}

void update_tft() {
  SensorMeasurement measurement;
  static TemperaturePressureAltitude tpa{NAN, NAN, NAN};
  static Wind wind{NAN};
  static Accelerometer accelerometer{NAN};
  static Ens ens{0};
  static Scd scd{0, NAN};
  static Gps gps{0, 0, NAN};
  static Max17 max17{NAN};
  static RadioTransmission transmission{0, 0};
  static RadioReception reception{0, 0, 0};
#ifdef RSD_DEBUG
  unsigned long start = micros();
#endif
  while (xQueueReceive(tft_queue, &measurement, 5) == pdPASS) {
    switch (measurement.type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        tpa = measurement.value.tpa;
        break;
      case WIND_SPEED:
        wind = measurement.value.wind;
        break;
      case ACCELEROMETER:
        accelerometer = measurement.value.accelerometer;
        break;
      case ENS:
        ens = measurement.value.ens;
        break;
      case SCD:
        scd = measurement.value.scd;
        break;
      case GPS:
        gps = measurement.value.gps;
        break;
      case MAX17:
        max17 = measurement.value.max17;
        break;
      case RADIO_TRANSMISSION:
        transmission = measurement.value.radio_transmission;
        break;
      case RADIO_RECEPTION:
        reception = measurement.value.radio_reception;
        break;
    }
  }
  tft.setCursor(0, 0);
  tft.printf("#%5u %3u B %2u data\n", transmission.id, reception.length, reception.measurements);
  tft.printf("%4u ms  %4d dBm\n", transmission.previous_transmission_duration, reception.signal_strength);
  tft.printf("%5.1f C %7.2f hPa\n", tpa.temperature, tpa.pressure);
  tft.printf("BAR%4.0f m ", tpa.altitude);
  #ifdef IS_433_MHZ
  tft.printf("GPS%4.0f m\n", gps.altitude);
  tft.printf("%4.1f g\n", accelerometer.acceleration);
  tft.printf("%8.5f %8.5f\n", gps.longitude / 10000000.0, gps.latitude / 10000000.0);
  tft.printf("%2.1f V\n", max17.voltage);
  #endif

#ifdef RSD_DEBUG
  unsigned long finish = micros();
  Serial.printf("Updating TFT took %.3f ms\n", (finish - start) * 0.001);
#endif
}

#endif