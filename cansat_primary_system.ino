#ifdef CANSAT_PRIMARY_SYSTEM

// #ifndef ADAFRUIT_FEATHER_ESP32S3_REVERSETFT
// #error "Select Adafruit Feather ESP32-S3 Reverse TFT board for the primary system!"
// #endif

#include "rsd_bmp390.h"
#include "rsd_environment.h"
#include "rsd_helper.h"
#include "rsd_lora.h"
#include "rsd_lora_433.h"
#include "rsd_lora_wing.h"
#include "rsd_sd_card.h"
#include "rsd_sensors.h"
#include "rsd_sensor_logging.h"
#include "rsd_tft.h"


/*****************
*  Definitions  *
*****************/
//GPS
#define GPS_BAUD_RATE 115200
#define GPS_SERIAL Serial1
// Button
#define D0_BUTTON 0


/**********************
*  Global variables  *
**********************/
//tft
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
bool tft_on = true;
//bmp
BMP390 bmp1(0x77);  // Main pressure and temperature sensor.
BMP390 bmp2(0x76);  // Wind measurement sensor.
float Ground_Altitude = 0.0;
// LoRa radio
RH_RF95 rf95(RF95_CS, RF95_INT);
bool radio_ok = false;
uint8_t phase=0;
uint32_t phase1_enter_time = 0;
//queue
QueueHandle_t radio_queue;
QueueHandle_t sd_card_queue;
QueueHandle_t tft_queue;
// Sox accelerometer
Adafruit_LSM6DSO32 sox;
bool accelerometer_ok = false;
//ens160
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
//CO2
SensirionI2cScd4x scd41;
static int16_t error;
bool scd_ok = false;
//GPS
Adafruit_GPS gps(&GPS_SERIAL);
// Battery monitor
Adafruit_MAX17048 battery_monitor;
bool max17_ok = false;



void update_tft() {
  SensorMeasurement measurement;
  static TemperaturePressureAltitude tpa{NAN, NAN, NAN};
  static Wind wind{NAN};
  static Accelerometer accelerometer{NAN};
  static Ens ens{0};
  static Scd scd{0, NAN};
  static Gps gps{0, 0, NAN};
  static Max17 max17{NAN};
  #ifdef RSD_DEBUG
  unsigned long start = micros();
  #endif

  while (xQueueReceive(tft_queue, &measurement, 1) == pdPASS) {
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
    }
  }
  #ifdef RSD_DEBUG
  Serial.printf("*** Reading TFT queue took %.3f ms\n", (micros() - start) * 0.001);
  #endif
  tft.setCursor(0, 0);
  tft.printf("%5.1f C %7.2f hPa\n%5.1f m ", tpa.temperature, tpa.pressure, tpa.altitude);
  tft.printf("%2.1f m/s\n%5u ppb ", wind.speed, ens.tvoc);
  tft.printf("%5.2f g\n phase: %u ", accelerometer.acceleration, phase);
  tft.printf("%5u ppm %2.2f %%  %2.1f V\n%2.5f %2.5f\n", scd.co2_ppm, scd.humidity, max17.voltage, gps.longitude / 10000000.0, gps.latitude / 10000000.0);
  tft.printf("GPS alt: %4.2f m", gps.altitude);
  
  #ifdef RSD_DEBUG
  Serial.printf("*** Updating TFT took %.3f ms\n", (micros() - start) * 0.001);
  #endif
}

void send_radio_message() {
  uint8_t message[RH_RF95_MAX_MESSAGE_LEN] = {0};
  uint16_t size = 0;
  SensorMeasurement measurement;
  std::vector<SensorMeasurement> measurements, meas_bmp, meas_wind, meas_acc, meas_ens, meas_co2, meas_gps, meas_max17;
  static uint16_t transmission_id = 1;
  static uint16_t transmission_duration = 0;
  #ifdef RSD_DEBUG
  unsigned long start = micros();
  #endif

  while (xQueueReceive(radio_queue, &measurement, pdMS_TO_TICKS(1)) == pdPASS) {
    measurements.push_back(measurement);
  }
  #ifdef RSD_DEBUG
  Serial.printf("*** Reading %d measurement(s) from the radio queue took %.3f ms\n", measurements.size(), (micros() - start) * 0.001);
  #endif

  measurement.type = RADIO_TRANSMISSION;
  measurement.timestamp = millis();
  measurement.value.radio_transmission.id = transmission_id++;
  measurement.value.radio_transmission.previous_transmission_duration = transmission_duration;
  measurements.push_back(measurement);
  add_measurement_to_queue(measurement, sd_card_queue);
  // Send radio message counter.
  for (const auto& measurement : measurements) {
    switch (measurement.type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        meas_bmp.push_back(measurement);
        break;
      case WIND_SPEED:
        meas_wind.push_back(measurement);
        break;
      case ACCELEROMETER:
        meas_acc.push_back(measurement);
        break;
      case ENS:
        meas_ens.push_back(measurement);
        break;
      case SCD:
        meas_co2.push_back(measurement);
        break;
      case GPS:
        meas_gps.push_back(measurement);
        break;
      case MAX17:
        meas_max17.push_back(measurement);
        break;
    }
  }
  #ifdef RSD_DEBUG
  Serial.printf("*** Reading and processing radio queue took %.3f ms\n", (micros() - start) * 0.001);
  #endif

  size_t bmp_size = meas_bmp.size();
  size_t acc_size = meas_acc.size();
  size_t wind_size = meas_wind.size();
  size_t gps_size = meas_gps.size();
  size_t ens_size = meas_ens.size();
  if(phase == 0 && bmp_size >= 2 && acc_size >= 1 && 
        (meas_bmp[bmp_size-1].value.tpa.altitude >= meas_bmp[bmp_size-2].value.tpa.altitude + 3.0|| 
        meas_acc[acc_size-1].value.accelerometer.acceleration >= 10.0)) {
    phase++;
    phase1_enter_time = millis();
  }
  if (phase == 1 && millis() - phase1_enter_time >= 210000) { // 3.5 min
    phase++;
  }

  if (phase == 0 || phase == 1) {
    if(meas_bmp.size() >= 3) {
      size += meas_bmp[(bmp_size / 3) - 1].add_to_radio_message(message);
      size += meas_bmp[((bmp_size / 3) * 2) - 1].add_to_radio_message(message);
      size += meas_bmp[bmp_size - 1].add_to_radio_message(message);
    } else if(bmp_size == 2) {
      size += meas_bmp[bmp_size - 2].add_to_radio_message(message);
      size += meas_bmp[bmp_size - 1].add_to_radio_message(message);
    } else if(bmp_size == 1) {
      size += meas_bmp[bmp_size - 1].add_to_radio_message(message);
    }
    if(acc_size >= 2) {
      size += meas_acc[(acc_size / 2) - 1].add_to_radio_message(message);
      size += meas_acc[acc_size - 1].add_to_radio_message(message);
    } else if(meas_acc.size()==1) {
      size += meas_acc[acc_size - 1].add_to_radio_message(message);
    }
    if(wind_size>=1) {
      size += meas_wind[wind_size-1].add_to_radio_message(message);
    }
    if(gps_size>=1) {
      size += meas_gps[gps_size - 1].add_to_radio_message(message);
    }
  } else {
    // Phase 2.
    if(bmp_size >= 2) {
      size += meas_bmp[(bmp_size / 2) - 1].add_to_radio_message(message);
      size += meas_bmp[bmp_size - 1].add_to_radio_message(message);
    } else if(bmp_size == 1) {
      size += meas_bmp[bmp_size - 1].add_to_radio_message(message);
    }
    if(meas_wind.size() >= 2) {
      size += meas_wind[(wind_size / 2) - 1].add_to_radio_message(message);
      size += meas_wind[wind_size - 1].add_to_radio_message(message);
    } else if(wind_size == 1) {
      size += meas_wind[wind_size - 1].add_to_radio_message(message);
    }
    if(meas_co2.size()>=1) {
      size += meas_co2[meas_co2.size() - 1].add_to_radio_message(message);
    }
    if(gps_size>=1) {
      size += meas_gps[gps_size - 1].add_to_radio_message(message);
    }
    if(ens_size >= 2) {
      size += meas_ens[(ens_size / 2) - 1].add_to_radio_message(message);
      size += meas_ens[ens_size - 1].add_to_radio_message(message);
    } else if(ens_size == 1) {
      size += meas_ens[ens_size - 1].add_to_radio_message(message);
    }
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t transmission_start = millis();
  rf95.send(message, size);
  delay(10);
  rf95.waitPacketSent();
  transmission_duration = static_cast<uint16_t>(millis() - transmission_start);
  digitalWrite(LED_BUILTIN, LOW);
  #ifdef RSD_DEBUG
  Serial.printf("*** Sending radio message took %u ms\n", millis() - transmission_start);
  Serial.printf("*** Preparing and sending radio message took %.3f ms\n", (micros() - start) * 0.001);
  #endif
}


/*****************************
 *  Task Running on Core 0:  *
 *  Sensor measurements      *
 *****************************/
void task0(void *parameters) {
  uint32_t start = millis();
  uint32_t idle_timer = start;
  uint32_t bmp1_timer = start;
  uint32_t bmp2_timer = start;
  uint32_t accelerometer_timer = start;
  uint32_t ens_timer = start;
  uint32_t scd_timer = start;
  uint32_t gps_timer = start;
  uint32_t max17_timer = start;
  uint32_t now;
  float bmp1_recent_pressure = 0.0;
  float p_diff = 0.0;
  SensorMeasurement measurement;
  // Shortcuts to various sensor values.
  auto &tpa = measurement.value.tpa;
  auto &wind = measurement.value.wind;
  auto &accelerometer = measurement.value.accelerometer;
  auto &ens = measurement.value.ens;
  auto &scd = measurement.value.scd;
  auto &gpss = measurement.value.gps;
  auto &max17 = measurement.value.max17;

  while (true) {
    now = millis();
    std::string now_str = timestamp_to_string(now);
    if (now >= idle_timer) {
      idle_timer += 1000;
      #ifdef RSD_DEBUG
      unsigned long idle_start = micros();
      #endif
      yield();  // This lets the system run its own tasks. Otherwise,
                // the EPS32-S3 will restart after a few seconds.
      #ifdef RSD_DEBUG
      Serial.printf("%s TASK0 IDLE took %.3f ms\n", now_str.c_str(), (micros() - idle_start) * 0.001);
      #endif
    } else if (now >= bmp1_timer) {
      bmp1_timer += 80;
      measurement.type = TEMPERATURE_PRESSURE_ALTITUDE;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (tpa.read(bmp1, SEA_LEVEL_PRESSURE)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from BMP1 took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        // unsigned long end = micros();
        measurement.timestamp = now;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        bmp1_recent_pressure = tpa.pressure;
        #ifdef RSD_DEBUG
        Serial.printf("%s BMP1 %s\n", now_str.c_str(), tpa.as_string().c_str());
        #endif
        if (now >= 5000 && 0.0 == Ground_Altitude) {
          Ground_Altitude = tpa.altitude;
          #ifdef RSD_DEBUG
          Serial.printf("Ground_altitude: %.2f\n", Ground_Altitude);
          #endif
        }
      }
    } else if (now >= bmp2_timer) {
      bmp2_timer += 200;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (tpa.read(bmp2, SEA_LEVEL_PRESSURE)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from BMP2 took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        measurement.type = WIND_SPEED;
        measurement.timestamp = now;
        tpa.pressure -= p_diff;
        if (tpa.pressure < bmp1_recent_pressure)
          wind.speed = sqrt(2*(bmp1_recent_pressure - tpa.pressure)/AIR_DENSITY);
        else
          wind.speed = 0.0;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        #ifdef RSD_DEBUG
        Serial.printf("%s BMP2 %s\n", now_str.c_str(), tpa.as_string().c_str());
        Serial.printf("%s WIND %s\n", now_str.c_str(), wind.as_string().c_str());
        Serial.printf("Pressure difference: %.2f\n", p_diff);
        #endif
        if (p_diff == 0.0 && now > 5000) {
          p_diff = tpa.pressure - bmp1_recent_pressure;
          #ifdef RSD_DEBUG
          Serial.printf("p_diff: %.2f\n", p_diff);
          #endif
        }
      }
    } else if (now >= accelerometer_timer && accelerometer_ok) {
      accelerometer_timer += 200;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (accelerometer.read(sox)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from accelerometer took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        measurement.type = ACCELEROMETER;
        measurement.timestamp = now;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        #ifdef RSD_DEBUG
        Serial.printf("%s ACCELEROMETER %s\n", now_str.c_str(), accelerometer.as_string().c_str());
        #endif
      }
    } else if (now >= ens_timer) {
      ens_timer += 1044;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (ens.read(ens160)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from ENS took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        measurement.type = ENS;
        measurement.timestamp = now;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        #ifdef RSD_DEBUG
        Serial.printf("%s ENS %s\n", now_str.c_str(), ens.as_string().c_str());
        #endif
      }
    } else if (now >= scd_timer) {
      scd_timer += 5000;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (scd.read(scd41)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from SCD41 took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        measurement.type = SCD;
        measurement.timestamp = now;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        #ifdef RSD_DEBUG
        Serial.printf("%s SDC %s\n", now_str.c_str(), scd.as_string().c_str());
        #endif
      }
    } else if (now >= gps_timer) {
      gps_timer += 200;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (gpss.read(gps)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from GPS took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        measurement.type = GPS;
        measurement.timestamp = now;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        #ifdef RSD_DEBUG
        Serial.printf("%s GPS %s\n", now_str.c_str(), gpss.as_string().c_str());
        #endif
      }
    } else if (now >= max17_timer) {
      max17_timer += 60000;
      #ifdef RSD_DEBUG
      unsigned long sensor_start = micros();
      #endif
      if (max17.read(battery_monitor)) {
        #ifdef RSD_DEBUG
        Serial.printf("*** Reading data from MAX17 took %.3f ms\n", (micros() - sensor_start) * 0.001);
        #endif
        measurement.type = MAX17;
        measurement.timestamp = now;
        add_measurement_to_queues(measurement, {radio_queue, sd_card_queue, tft_queue});
        #ifdef RSD_DEBUG
        Serial.printf("%s BATTERY %s\n", now_str.c_str(), max17.as_string().c_str());
        #endif
      }
    }
  }
}

/*****************************
 *  Task Running on Core 1:  *
 *  Radio, SD card, TFT      *
 *****************************/
void task1(void *parameters) {
  uint32_t start = millis();
  uint32_t radio_timer = start;
  uint32_t sd_card_timer = start;
  uint32_t tft_timer = start;
  uint32_t idle_timer = start;
  uint32_t now;
  SensorMeasurement measurement;

  while (true) {
    now = millis();
    if (now >= idle_timer) {
      idle_timer += 1000;
      #ifdef RSD_DEBUG
      unsigned long idle_start = micros();
      #endif
      yield();  // This lets the system run its own tasks. Otherwise,
                // the EPS32-S3 will restart after a few seconds.
      #ifdef RSD_DEBUG
      Serial.printf("%s TASK1 IDLE took %.3f ms\n", timestamp_to_string(now).c_str(), (micros() - idle_start) * 0.001);
      #endif
    } else if (now >= radio_timer && radio_ok) {
      #ifdef RSD_DEBUG
      Serial.printf("phase: %u\n", phase);
      #endif
      if (phase == 0) { 
        radio_timer += 1000;
      } else if (phase == 1) {
        radio_timer += 500;
      } else {
        radio_timer += 5000;
      }
      send_radio_message();
    } else if (now >= sd_card_timer) {
      sd_card_timer += 2000;
      std::vector<SensorMeasurement> measurements;
      while (xQueueReceive(sd_card_queue, &measurement, pdMS_TO_TICKS(5)) == pdPASS)
        measurements.push_back(measurement);
      write_to_sd_card(measurements);
    } else if (now >= tft_timer) {
      tft_timer += 1000;
      update_tft();
    }
  }
}

// void IRAM_ATTR D0_button_release_interrupt() {
//   D0_button_released = true;
// }

void setup() {
  radio_queue = xQueueCreate(1000, sizeof(SensorMeasurement));
  sd_card_queue = xQueueCreate(1000, sizeof(SensorMeasurement));
  tft_queue = xQueueCreate(1000, sizeof(SensorMeasurement));

  #ifdef RSD_DEBUG
  Serial.begin(115200);
  // while (!Serial);
  #endif

  // Initialize I2C communication.
  Wire.begin();
  Wire.setClock(100000);

  // Preparing the D0 button.
  // pinMode(D0_BUTTON, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(D0_BUTTON), D0_button_release_interrupt, RISING);

  setup_tft(tft, 2);
  setup_sox();
  radio_ok = setup_lora(rf95, RF95_RST, RF95_FREQ_MHZ,
                        RF95_TX_POWER_DBM, RF95_SPREADING_FACTOR,
                        RF95_BANDWIDTH_HZ, RF95_CODING_RATE);
  if (radio_ok)
    tft.println("LoRa INIT OK");
  else
    tft.println("LoRa INIT FAILED");
  setup_ens();
  setup_scd();
  setup_gps();
  setup_max17();

  if (bmp1.initialize()) {
    tft.println("BMP390 (1) INIT OK");
    #ifdef RSD_DEBUG
    Serial.println("BMP390 (1) INIT OK");
    #endif
  } else {
    tft.println("BMP390 (1) INIT FAILED");
    #ifdef RSD_DEBUG
    Serial.println("BMP390 (1) INIT FAILED");
    #endif
  }
  if (bmp2.initialize()) {
    tft.println("BMP390 (2) INIT OK");
    #ifdef RSD_DEBUG
    Serial.println("BMP390 (2) INIT OK");
    #endif
  } else {
    tft.println("BMP390 (2) INIT FAILED");
    #ifdef RSD_DEBUG
    Serial.println("BMP390 (2) INIT FAILED");
    #endif
  }

  // Initialize SD card.
  if (initialize_log_files(all_sensor_types())) {
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

  // Initialize LED to indicate radio transmission.
  pinMode(LED_BUILTIN, OUTPUT);

  delay(2000);
  clear_tft(tft);

  xTaskCreatePinnedToCore(task0, "task0", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task1, "task1", 8192, NULL, 1, NULL, 1);
}

void setup_sox() {
  if (sox.begin_I2C()) {
    accelerometer_ok = true;
    sox.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    #ifdef RSD_DEBUG
    Serial.println("ACCELEROMETER INIT OK");
    #endif
  } else {
    #ifdef RSD_DEBUG
    Serial.println("ACCELEROMETER INIT FAILED");
    #endif
  }
}

void setup_ens() {
  ens160.begin();
  if (ens160.available()) {
    ens160.setMode(ENS160_OPMODE_STD);
    #ifdef RSD_DEBUG
    Serial.println("ENS INIT OK");
    #endif
  } else {
    #ifdef RSD_DEBUG
    Serial.println("ENS INIT FAILED");
    #endif
  }
}

void setup_scd() {
  scd41.begin(Wire, SCD41_I2C_ADDR_62);
  delay(50);

  if (scd41.wakeUp() == NO_ERROR &&
      scd41.stopPeriodicMeasurement() == NO_ERROR &&
      scd41.reinit() == NO_ERROR &&
      scd41.startPeriodicMeasurement() == NO_ERROR) {
    scd_ok = true;
    #ifdef RSD_DEBUG
    Serial.println("SCD INIT OK");
    #endif
  } else {
    scd_ok = false;
    #ifdef RSD_DEBUG
    Serial.println("SCD INIT FAILED");
    #endif
  }
}

void setup_gps() {
  gps.begin(9600);
  if (GPS_BAUD_RATE == 57600 || GPS_BAUD_RATE == 115200) {
    if (GPS_BAUD_RATE == 57600)
      gps.sendCommand(PMTK_SET_BAUD_57600);
    else
      gps.sendCommand(PMTK_SET_BAUD_115200);
    delay(100);
    GPS_SERIAL.end();
    GPS_SERIAL.begin(GPS_BAUD_RATE);
    gps.begin(GPS_BAUD_RATE);
  } else {
    gps.sendCommand(PMTK_SET_BAUD_9600);
    if (GPS_BAUD_RATE != 9600) {
      Serial.printf("Invalid GPS baud\nrate: %d\n9600 is used.\n", GPS_BAUD_RATE);
    }
  }
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
}

void setup_max17() {
  if (battery_monitor.begin()) {
    max17_ok = true;
    #ifdef RSD_DEBUG
    Serial.println("MAX17 INIT OK");
    #endif
  } else {
    max17_ok = false;
    #ifdef RSD_DEBUG
    Serial.println("MAX17 INIT FAILED");
    #endif
  }
}

void loop () {}

#endif