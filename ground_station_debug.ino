#ifdef GROUND_STATION_DEBUG

#include <optional>
#include <sstream>

#include "esp_system.h"
#include "esp_task_wdt.h"
#include <SPI.h>

// Uncomment for printing debug info via Serial.
#define RSD_DEBUG

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

/*****************
 *  Definitions  *
 *****************/



/**********************
 *  Global Variables  *
 **********************/
// Display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// LoRa radio
RH_RF95 rf95(RF95_CS, RF95_INT);
bool radio_ok = false;
// SD card
bool sd_card_ok = false;
// Queues
// QueueHandle_t sd_card_queue;
QueueHandle_t tft_queue;

  // while (xQueueReceive(sd_card_queue, &measurement, pdMS_TO_TICKS(5)) == pdPASS) {
  //   measurements.push_back(measurement);
  // }

/****************************
*  Task Running on Core 0  *
****************************/
void task0(void *parameters) {
  // This task updates the display and writes to SD card.

  uint32_t idle_timer = millis();
  uint32_t tft_timer = millis();
  uint32_t sd_card_timer = millis();
  uint32_t now;
  SensorMeasurement measurement;

  while (true) {
    now = millis();
    std::string now_str = timestamp_to_string(now);
    if (now >= idle_timer) {
      idle_timer += 1000;
      Serial.printf("%s TASK0 IDLE\n", now_str.c_str());
      vTaskDelay(1);  // This lets the system run its own tasks. Otherwise,
                      // the EPS32-S3 will restart after a few seconds.
    } else if (now >= tft_timer) {
      tft_timer += 200;
      update_tft();
    } else if (now >= sd_card_timer) {
      sd_card_timer += 5000;
      // std::vector<SensorMeasurement> measurements;
      // while (xQueueReceive(sd_card_queue, &measurement, pdMS_TO_TICKS(5)) == pdPASS)
      //   measurements.push_back(measurement);
      // write_to_sd_card(measurements);
    }
  }
}

/****************************
*  Task Running on Core 1  *
****************************/
void task1(void *parameters) {
  // This task listens to radio messages.

  uint32_t idle_timer = millis();
  uint32_t now;

  while (true) {
    now = millis();
    std::string now_str = timestamp_to_string(now);
    if (now >= idle_timer) {
      idle_timer += 1000;
      Serial.printf("%s TASK1 IDLE\n", now_str.c_str());
      vTaskDelay(1);  // This lets the system run its own tasks. Otherwise,
                      // the EPS32-S3 will restart after a few seconds.
    }
    receive_transmission();
  }
}


void setup() {
  #ifdef RSD_DEBUG
  Serial.begin(115200);
  for (int i = 0; i < 3; i++)
    if (!Serial) delay(1000);
  if (Serial) Serial.println("Serial INIT OK");
  #endif
  Serial.print("RESET REASON: ");
  Serial.println(esp_reset_reason());

  esp_task_wdt_deinit();

  // sd_card_queue = xQueueCreate(100, sizeof(SensorMeasurement));
  tft_queue = xQueueCreate(100, sizeof(SensorMeasurement));


  setup_tft(tft, 2);
  radio_ok = setup_lora(rf95, RF95_RST, RF95_FREQ_MHZ,
                        RF95_TX_POWER_DBM, RF95_SPREADING_FACTOR,
                        RF95_BANDWIDTH_HZ, RF95_CODING_RATE);
  if (radio_ok)
    tft.println("LoRa INIT OK");
  else
    tft.println("LoRa INIT FAILED");
  // Initialize SD card.
  // if (initialize_log_files()) {
  //   tft.println("SD card INIT OK");
  //   #ifdef RSD_DEBUG
  //   Serial.println("SD card INIT OK");
  //   #endif
  // } else {
  //   tft.println("SD card INIT FAILED");
  //   #ifdef RSD_DEBUG
  //   Serial.println("SD card INIT FAILED");
  //   #endif
  // }
  delay(2000);
  clear_tft(tft);

  xTaskCreatePinnedToCore(task0, "task0", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task1, "task1", 8192, NULL, 1, NULL, 1);

}

void loop() {}

bool receive_transmission() {
  Serial.println("waitAvailableTimeout elott");
  delay(100);
  if (rf95.waitAvailableTimeout(1000)) {
    Serial.println("waitAvailableTimeout utan");
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
        size += measurement.read_from_radio_message(message+size);
        xQueueSendToBack(tft_queue, &measurement, pdMS_TO_TICKS(10));
        // xQueueSendToBack(sd_card_queue, &measurement, pdMS_TO_TICKS(10));
        number_of_messages++;
        #ifdef RSD_DEBUG
        Serial.println(measurement.as_string().c_str());
        #endif
      }
      measurement.type = RADIO_RECEPTION;
      measurement.timestamp = now;
      measurement.value.radio_reception = {len, number_of_messages, rf95.lastRssi()};
      xQueueSendToBack(tft_queue, &measurement, pdMS_TO_TICKS(10));
      // xQueueSendToBack(sd_card_queue, &measurement, pdMS_TO_TICKS(10));
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
  #ifdef RSD_DEBUG
  unsigned long start = micros();
  #endif

  // while (xQueueReceive(tft_queue, &measurement, 0) == pdPASS) {
  //   switch (measurement.type) {
  //     case TEMPERATURE_PRESSURE_ALTITUDE:
  //       tpa = measurement.value.tpa;
  //       break;
  //   }
  // }
  tft.setCursor(0, 0);
  tft.println("Hello World!\n");

  #ifdef RSD_DEBUG
  unsigned long finish = micros();
  Serial.printf("Updating TFT took %d us.\n", finish - start);
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Free stack: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));
  #endif
}


#endif