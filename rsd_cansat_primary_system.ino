#include <optional>
#include <sstream>

#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ST7789.h>
#include "bmp3.h"
#include <RH_RF95.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "ScioSense_ENS160.h"


/*****************
 *  Definitions  *
 *****************/
#define SD_CS 10
#define TFT_TEXT_SIZE 2
#define SEA_LEVEL_PRESSURE 1010.0
// LoRa radio
#define RF95_INT 12
#define RF95_CS 5
#define RF95_RST 11
#define RF95_FREQ_MHZ 433.0
#define RF95_TX_POWER_DBM 14


/*******************
 *  BMP390 Sensor  *
 *******************/
class BMP390 {
 public:
  BMP390(uint8_t i2c_address): i2c_address(i2c_address) {}
  bool initialize() {
    device.intf = BMP3_I2C_INTF;
    device.intf_ptr = new Adafruit_I2CDevice(i2c_address, &Wire);
    device.read = i2c_read;
    device.write = i2c_write;
    device.delay_us = delay_microseconds;
    if (bmp3_init(&device) != BMP3_OK)
      return false;
    device.settings.press_en = BMP3_ENABLE;
    device.settings.temp_en = BMP3_ENABLE;
    device.settings.odr_filter.press_os = BMP3_OVERSAMPLING_32X;
    device.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    device.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;
    device.settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
    uint32_t settings = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
      BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
      BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;
    if (bmp3_set_sensor_settings(settings, &device) != BMP3_OK)
      return false;
    device.settings.op_mode = BMP3_MODE_NORMAL;
    if (bmp3_set_op_mode(&device) != BMP3_OK)
      return false;
    return true;
  }
  // Set the temperature in °C and pressure in hPa and return true
  // if the device is working and new data is available. Otherwise
  // false is returned and temperature and pressure are set to NaN.
  bool read(float& temperature, float& pressure) {
    struct bmp3_data data;
    if (bmp3_get_status(&device) != BMP3_OK || !device.status.intr.drdy ||
        bmp3_get_sensor_data(BMP3_ALL, &data, &device) != BMP3_OK) {
      temperature = pressure = std::numeric_limits<float>::quiet_NaN();
      return false;
    }
    temperature = static_cast<float>(data.temperature);
    pressure = static_cast<float>(data.pressure * 0.01);
    return true;
  }
  static float calculate_altitude(float pressure, float sea_level_pressure) {
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf 
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
  }
 private:
  struct bmp3_dev device;
  const uint8_t i2c_address;
  static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                         uint32_t len, void *intf_ptr) {
    auto* i2c_dev = reinterpret_cast<Adafruit_I2CDevice*>(intf_ptr);
    if (i2c_dev && !i2c_dev->write_then_read(&reg_addr, 1, reg_data, len))
      return 1;
    return 0;
  }
  static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                          uint32_t len, void *intf_ptr) {
    auto* i2c_dev = reinterpret_cast<Adafruit_I2CDevice*>(intf_ptr);
    if (i2c_dev && !i2c_dev->write((uint8_t *)reg_data, len, true,
                                   &reg_addr, 1))
      return 1;
    return 0;
  }
  static void delay_microseconds(uint32_t us, void *intf_ptr) {
    delayMicroseconds(us);
  }
};


/**********************
 *  Global variables  *
 **********************/
//tft
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
//bmp
BMP390 bmp1(0x77);  // Main pressure and temperature sensor.
BMP390 bmp2(0x76);  // Wind measurement sensor.
float AIR_DENSITY = 1.29; //kg/m3
//SD
bool sd_card_ok = false;
File tpa_log = File();
File wind_log = File();
File accelerometer_log = File();
File ens_log = File();
// LoRa radio
RH_RF95 rf95(RF95_CS, RF95_INT);
bool radio_ok = false;
uint8_t phase=0;
//queue
QueueHandle_t radio_queue;
QueueHandle_t sd_card_queue;
QueueHandle_t tft_queue;
//sox
Adafruit_LSM6DSOX sox;
bool accelerometer_ok = false;
//ens160
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);


/**************************
 *  Forward declarations  *
 **************************/
std::string float_to_string(float x);
struct SensorInfo;

// Template function to pack any type into bytes.
template <typename T>
size_t pack(T value, byte *bytes) {
  byte *valueAsBytes = (byte *)&value;
  size_t size = sizeof(T);
  for (size_t i = 0; i < size; i++) {
    bytes[i] = valueAsBytes[i];
  }
  return size;
}


/***************************
 *  Sensor Types and Data  *
 ***************************/
typedef enum {
  UNDEFINED,
  TEMPERATURE_PRESSURE_ALTITUDE,
  WIND_SPEED,
  ACCELEROMETER,
  ENS
} SensorType;

struct SensorInfo {
  char* id;  // Short identifier, without space or special characters.
  std::vector<const char*> fields;  // Name and unit of the sensor's fields.
};

struct TemperaturePressureAltitude {
  float temperature;
  float pressure;
  float altitude;

  static inline const SensorInfo info{"tpa", {"Temperature [°C]", "Pressure [hPa]", "Altitude [m]"}};
  bool read(BMP390& bmp) {
    if (bmp.read(temperature, pressure)) {
      altitude = bmp.calculate_altitude(pressure, SEA_LEVEL_PRESSURE);
      return true;
    }
    return false;
  }
  std::string as_string() const {
    auto t = float_to_string(temperature);
    auto p = float_to_string(pressure);
    auto a = float_to_string(altitude);
    return t + "\t" + p + "\t" + a;
  }
  size_t add_to_radio_message(uint8_t *message) {
    size_t size = 0;
    size += pack(temperature, message+size);
    size += pack(pressure, message+size);
    size += pack(altitude, message+size);
    return size;
  }
};

struct Wind {
  float speed;

  static inline const SensorInfo info{"wind", {"Wind speed [m/s]"}};
  std::string as_string() const {
    return float_to_string(speed);
  }
  size_t add_to_radio_message(uint8_t *message) {
    return pack(speed, message);
  }
};

struct Accelerometer {
  float acceleration;

  static inline const SensorInfo info{"accel", {"Acceleration [g]"}};
  bool read(Adafruit_LSM6DSOX& sox_sensor) {
    sensors_event_t accel, gyro, temp;
    if (sox_sensor.getEvent(&accel, &gyro, &temp)) {
      sensors_vec_t& a = accel.acceleration;
      acceleration = sqrt(a.x*a.x+a.y*a.y+a.z*a.z) / 4.905;
      return true;
    }
    return false;
  }
  std::string as_string() const {
    return float_to_string(acceleration);
  }
  size_t add_to_radio_message(uint8_t *message) {
    return pack(acceleration, message);
  }
};
struct Ens {
    uint16_t tvoc;

    static inline const SensorInfo info{"tvoc", {"Tvoc [ppb]"}};
    bool read(ScioSense_ENS160& ens_sensor) {
      if (ens160.available()) {
        ens160.measure(true);
        ens160.measureRaw(true);
        tvoc = ens160.getTVOC();
        Serial.printf("TVOC: %u\n", tvoc);
        return true;
      }
      return false;
    }
    std::string as_string() const {
      return float_to_string(tvoc);
    }
    size_t add_to_radio_message(uint8_t *message) {
      return pack(tvoc, message);
    }
};

union SensorValue {
  TemperaturePressureAltitude tpa;
  Wind wind;
  Accelerometer accelerometer;
  Ens ens;
};

struct SensorMeasurement {
  uint32_t timestamp;  // Milliseconds since boot/midnight.
  SensorType type;
  SensorValue value;

  const SensorInfo& info() {
    static SensorInfo undefined{"undef", {}};
    switch (type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        return TemperaturePressureAltitude::info;
      case WIND_SPEED:
        return Wind::info;
      case ACCELEROMETER:
        return Accelerometer::info;
      case ENS:
        return Ens::info;
      default:
        return undefined;
    }
  }

  BaseType_t add_to_queue(QueueHandle_t& queue) {
    return xQueueSendToBack(queue, this, pdMS_TO_TICKS(100));
  }

  BaseType_t add_to_all_queues() {
    BaseType_t result = pdPASS;
    if (add_to_queue(radio_queue) != pdPASS) result = pdFAIL;
    if (add_to_queue(sd_card_queue) != pdPASS) result = pdFAIL;
    if (add_to_queue(tft_queue) != pdPASS) result = pdFAIL;
    return result;
  }

  size_t add_to_radio_message(uint8_t *message) {
    size_t size = 0;
    size += pack(static_cast<uint8_t>(type), message+size);
    size += pack(timestamp, message+size);
    switch (type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        size += value.tpa.add_to_radio_message(message+size);
        break;
      case WIND_SPEED:
        size += value.wind.add_to_radio_message(message+size);
        break;
      case ACCELEROMETER:
        size += value.accelerometer.add_to_radio_message(message+size);
        break;
      case ENS:
        size += value.ens.add_to_radio_message(message+size);
        break;
    }
    return size;
  }
};

/**********************
 *  Helper functions  *
 **********************/
std::string float_to_string(float x, uint8_t precision=2) {
  char buf[16];
  dtostrf(x, 1, precision, buf);
  return std::string(buf);
}

std::string float_to_string(float x) {
  return float_to_string(x, 2);
}

// Timestamp is the number of milliseconds since midnight or boot.
std::string timestamp_to_string(uint32_t timestamp) {
  uint8_t hours = (timestamp / 3600000) % 24;
  uint8_t minutes = (timestamp / 60000) % 60;
  uint8_t seconds = (timestamp / 1000) % 60;
  uint16_t milliseconds = timestamp % 1000;
  char buffer[13];
  sprintf(buffer, "%02u:%02u:%02u.%03u", hours, minutes, seconds, milliseconds);
  return std::string(buffer);
}

std::optional<std::string> create_log_file_name(const std::string& prefix) {
  if (!sd_card_ok)
    return std::nullopt;

  char file_name[16];
  for (int i = 0; i < 1000; i++) {
    sprintf(file_name, "/%s_%03d.txt", prefix.substr(0, 6).c_str(), i);
    if (!SD.exists(file_name))
      return std::string(file_name);
  }
  return std::nullopt;
}

File create_log_file(const SensorInfo& sensor_info) {
  if (sd_card_ok) {
    Serial.println("SD card OK");
    auto file_name = create_log_file_name(std::string(sensor_info.id));
    if (file_name) {
      Serial.printf("Sensor log file: %s\n", file_name->c_str());
      auto log_file = SD.open(file_name->c_str(), FILE_WRITE);
      if (log_file)
        return log_file;
    }
  }
  return File();
}

void write_log_file_header(File& log_file, const SensorInfo& sensor_info) {
  if (log_file) {
    std::ostringstream oss;
    oss << "Timestamp";
    for (auto field_name : sensor_info.fields) {
      oss << "\t" << field_name;
    }
    log_file.printf("%s\n", oss.str().c_str());
    log_file.flush();
  }
}

File initialize_log_file(const SensorInfo& sensor_info) {
  File log_file = create_log_file(sensor_info);
  if (log_file)
    write_log_file_header(log_file, sensor_info);
  return log_file;
}

void write_to_sd_card() {
  SensorMeasurement measurement;
  std::vector<SensorMeasurement> measurements;
  unsigned long start = micros();
  bool seen_tpa = false;
  bool seen_wind = false;
  bool seen_accelerometer = false;
  bool seen_ens = false;

  while (xQueueReceive(sd_card_queue, &measurement, pdMS_TO_TICKS(5)) == pdPASS) {
    measurements.push_back(measurement);
  }
  for (auto& m : measurements) {
    std::string timestamp = timestamp_to_string(m.timestamp);
    switch (m.type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        if (tpa_log) {
          tpa_log.printf("%s\t%s\n", timestamp.c_str(), m.value.tpa.as_string().c_str());
          seen_tpa = true;
        }
        break;
      case WIND_SPEED:
        if (wind_log) {
          wind_log.printf("%s\t%s\n", timestamp.c_str(), m.value.wind.as_string().c_str());
          seen_wind = true;
        }
        break;
      case ACCELEROMETER:
        if (accelerometer_log) {
          accelerometer_log.printf("%s\t%s\n", timestamp.c_str(), m.value.accelerometer.as_string().c_str());
          seen_accelerometer = true;
        }
      case ENS:
        if (ens_log) {
          ens_log.printf("%s\t%s\n", timestamp.c_str(), m.value.ens.as_string().c_str());
        }
    }
  }
  if (seen_tpa) {
    tpa_log.flush();
  }
  if (seen_wind) {
    wind_log.flush();
  }
  if (seen_accelerometer) {
    accelerometer_log.flush();
  }
  if (seen_ens) {
    ens_log.flush();
  }
  unsigned long finish = micros();
  Serial.printf("Writing %d measurement(s) to SD card took %d us.\n", measurements.size(), finish - start);
}

void update_tft() {
  SensorMeasurement measurement;
  static TemperaturePressureAltitude tpa{NAN, NAN, NAN};
  static Wind wind{NAN};
  static Accelerometer accelerometer{NAN};
  static Ens ens{0};
  unsigned long start = micros();
  static char* blank_line = "                    ";

  while (xQueueReceive(tft_queue, &measurement, 0) == pdPASS) {
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
        Serial.println("EEEEEEEEEEEEEEENNNNNNNNNNNNNNNNNSSSSSSSSSSSSSSSSSSS");
        break;
    }
  }
  tft.setCursor(0, 0);
  tft.printf("%5.1f C %7.2f hPa\n%5.1f m    \n", tpa.temperature, tpa.pressure, tpa.altitude);
  tft.printf("%5.1f m/s          \n", wind.speed);
  tft.printf("%5.2f g          \n", accelerometer.acceleration);
  tft.printf("%5u ppb          \n", ens.tvoc);
  
  unsigned long finish = micros();
  Serial.printf("Updating TFT took %d us.\n", finish - start);
}

void send_radio_message() {
  uint8_t message[RH_RF95_MAX_MESSAGE_LEN] = {0};
  uint16_t size = 0;
  //uint8_t message_length=0;
  SensorMeasurement measurement;
  std::vector<SensorMeasurement> measurements,measurements_bmp,measurements_wind,measurements_accelerometer,measurements_ens,measurements_co2,measurements_gps;

  while (xQueueReceive(radio_queue, &measurement, pdMS_TO_TICKS(5)) == pdPASS) {
    measurements.push_back(measurement);
  }
  // For now send only the last temperature/pressure/altitude measurement.
  for (auto& m : measurements) {
    switch (m.type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        measurements_bmp.push_back(m);
        break;
      case WIND_SPEED:
        measurements_wind.push_back(m);
        break;
      case ACCELEROMETER:
        measurements_accelerometer.push_back(m);
        break;
/*      case ENS:
        measurements_ens.push_back(m);
        break;
      case CO2:
        measurements_co2.push_back(m);
        break;
      case GPS:
        measurements_gps.push_back(m);
        break;*/
    }
  }
  if (phase == 0 || phase == 1) {
    if(measurements_bmp.size() >= 3) {
      message[size] = 3; size++;//03 tízes helyiérték:0 - 0-ás számú szenzor adata
      size += measurements_bmp[(measurements_bmp.size() / 3) - 1].add_to_radio_message(message);
      size += measurements_bmp[((measurements_bmp.size() / 3) * 2) - 1].add_to_radio_message(message);
      size += measurements_bmp[measurements_bmp.size() - 1].add_to_radio_message(message);
    } else if(measurements_bmp.size() == 2) {
      message[size] = 2; size++;
      size += measurements_bmp[(measurements_bmp.size() / 2) - 1].add_to_radio_message(message);
      size += measurements_bmp[measurements_bmp.size() - 1].add_to_radio_message(message);
    } else if(measurements_bmp.size() == 1) {
      message[size] = 1;  size++;
      size += measurements_bmp[measurements_bmp.size() - 1].add_to_radio_message(message);
    }
    if(measurements_accelerometer.size()>=2) {
      message[size]=22;  size++;
      size += measurements_accelerometer[(measurements_accelerometer.size() / 2) - 1].add_to_radio_message(message);
      size += measurements_accelerometer[measurements_accelerometer.size() - 1].add_to_radio_message(message);
    } else if(measurements_accelerometer.size()==1) {
      message[size]=22;  size++;
      size += measurements_accelerometer[measurements_accelerometer.size() - 1].add_to_radio_message(message);
    }
    if(measurements_wind.size()>=1) {
      message[size]=10; size++;
      size += measurements_wind[measurements_wind.size()-1].add_to_radio_message(message);
    }
  }
  //size += measurement.add_to_radio_message(message);
  rf95.send(message, size);
  delay(10);
  rf95.waitPacketSent();

}

/****************************
 *  Task Running on Core 0  *
 ****************************/
void task0(void *parameters) {
  uint32_t start = millis();
  uint32_t idle_timer = start;
  uint32_t bmp1_timer = start;
  uint32_t bmp2_timer = start;
  uint32_t accelerometer_timer = start;
  uint32_t ens_timer = start;
  uint32_t now;
  float bmp1_recent_pressure = 0.0;
  SensorMeasurement measurement;
  // Shortcuts to various sensor values.
  auto &tpa = measurement.value.tpa;
  auto &wind = measurement.value.wind;
  auto &accelerometer = measurement.value.accelerometer;
  auto &ens = measurement.value.ens;
 
  while (true) {
    now = millis();
    std::string now_str = timestamp_to_string(now);
    if (now >= idle_timer) {
      idle_timer += 1000;
      Serial.printf("%s TASK0 IDLE\n", now_str.c_str());
      yield();  // This lets the system run its own tasks. Otherwise,
                // the EPS32-S3 will restart after a few seconds.
    } else if (now >= bmp1_timer) {
      bmp1_timer += 80;
      measurement.type = TEMPERATURE_PRESSURE_ALTITUDE;
      // unsigned long start = micros();
      if (tpa.read(bmp1)) {
        // unsigned long end = micros();
        measurement.timestamp = now;
        measurement.add_to_all_queues();
        bmp1_recent_pressure = tpa.pressure;
        Serial.printf("%s BMP1 %s\n", now_str.c_str(), tpa.as_string().c_str());
      }
    } else if (now >= bmp2_timer) {
      bmp2_timer += 300;
      if (tpa.read(bmp2)) {
        measurement.type = WIND_SPEED;
        measurement.timestamp = now;
        wind.speed = sqrt(2*(bmp1_recent_pressure - tpa.pressure)/AIR_DENSITY);
        measurement.add_to_all_queues();
        Serial.printf("%s WIND %s\n", now_str.c_str(), wind.as_string().c_str());
      }
    } else if (now >= accelerometer_timer && accelerometer_ok) {
      accelerometer_timer += 200;
      if (accelerometer.read(sox)) {
        measurement.type = ACCELEROMETER;
        measurement.timestamp = now;
        measurement.add_to_all_queues();
        Serial.printf("%s ACCELEROMETER %s\n", now_str.c_str(), accelerometer.as_string().c_str());
      }
    } else if (now >= ens_timer) {
      ens_timer += 1044;
      if (ens.read(ens160)) {
        measurement.type = ENS;
        measurement.timestamp = now;
        measurement.add_to_all_queues();
        Serial.printf("%s ENS %s\n", now_str.c_str(), ens.as_string().c_str());
      }
    }
  }
}

/****************************
 *  Task Running on Core 1  *
 ****************************/
void task1(void *parameters) {
  uint32_t start = millis();
  uint32_t radio_timer = start;
  uint32_t sd_card_timer = start;
  uint32_t tft_timer = start;
  uint32_t idle_timer = start;
  uint32_t now;
  SensorMeasurement measurement;
  auto &tpa = measurement.value.tpa;
  auto &wind = measurement.value.wind;
  auto &accelerometer = measurement.value.accelerometer;

  while (true) {
    now = millis();
    if (now >= idle_timer) {
      idle_timer += 1000;
      yield();  // This lets the system run its own tasks. Otherwise,
                // the EPS32-S3 will restart after a few seconds.
    } else if (now >= sd_card_timer) {
      sd_card_timer += 2000;
      write_to_sd_card();
    } else if (now >= radio_timer && radio_ok) {
      if (phase == 0) { 
        radio_timer += 1000;
      } else if (phase == 1) {
        radio_timer=now;
      } else {
        radio_timer += 5000;
      }
      send_radio_message();
    } else if (now >= tft_timer) {
      tft_timer += 333;
      update_tft();
    }
  }
}

void setup() {
    radio_queue = xQueueCreate(1000, sizeof(SensorMeasurement));
    sd_card_queue = xQueueCreate(1000, sizeof(SensorMeasurement));
    tft_queue = xQueueCreate(1000, sizeof(SensorMeasurement));

    Serial.begin(115200);
    // while (!Serial);

    // Initialize SD card.
    if (sd_card_ok = SD.begin(SD_CS, SPI, 50000000)) {
      tpa_log = initialize_log_file(TemperaturePressureAltitude::info);
      wind_log = initialize_log_file(Wind::info);
      accelerometer_log = initialize_log_file(Accelerometer::info);
      ens_log = initialize_log_file(Ens::info);
    } else {
      Serial.println("SD card FAILED");
    }

    // Initialize I2C communication.
    Wire.begin();
    Wire.setClock(100000);

    setup_tft();
    setup_sox();
    setup_lora();
    setup_ens();

    if (bmp1.initialize())
      Serial.printf("BMP390 (1) init success\n");
    else
      Serial.printf("BMP390 (1) init failed\n");
    if (bmp2.initialize())
      Serial.printf("BMP390 (2) init success\n");
    else
      Serial.printf("BMP390 (2) init failed\n");
  
    xTaskCreatePinnedToCore(task0, "task0", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task1, "task1", 8192, NULL, 1, NULL, 1);
}

void setup_tft() {
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);
  tft.init(135, 240);
  tft.setRotation(3);
  tft.setTextWrap(true);
  tft.setTextSize(TFT_TEXT_SIZE);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.fillScreen(ST77XX_BLACK);
}

void setup_sox() {
  if (sox.begin_I2C()) {
    accelerometer_ok = true;
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  } else {
    Serial.println("NO ACCELEROMETER");
  }
}

void setup_ens() {
  ens160.begin();
  Serial.print("ENS SETUP...   ");
  Serial.println(ens160.available() ? "done." : "failed!");
  ens160.setMode(ENS160_OPMODE_STD);
}
void setup_lora() {
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, HIGH);
  delay(100);
  digitalWrite(RF95_RST, LOW);
  delay(10);
  digitalWrite(RF95_RST, HIGH);
  delay(10);
  for (int i = 0; i < 3; i++) {
    if (rf95.init()) {
      radio_ok = true;
      break;
    }
    delay(100);
  }
  if (radio_ok) {
    Serial.println("LoRa radio OK");
    if (!rf95.setFrequency(RF95_FREQ_MHZ)) {
      Serial.println("LoRa setting frequency FAILED");
    }
    rf95.setTxPower(RF95_TX_POWER_DBM, false);
  } else {
    Serial.println("LoRa radio FAILED");
  }
}

void loop () {}
