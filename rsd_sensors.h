#ifndef RSD_SENSORS
#define RSD_SENSORS

#include <Adafruit_GPS.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_MAX1704X.h>
#include <ScioSense_ENS160.h>
#include <SensirionI2cScd4x.h>

#include "rsd_bmp390.h"
#include "rsd_helper.h"

/***************************
*  Sensor Types and Data  *
***************************/
typedef enum {
  UNDEFINED_SENSOR,  // Always keep it in the first position!
  TEMPERATURE_PRESSURE_ALTITUDE,  // Always keep it in the second position!
  WIND_SPEED,
  ACCELEROMETER,
  ENS,
  SCD,
  GPS,
  MAX17,
  RADIO_TRANSMISSION,
  RADIO_RECEPTION,
  CALCULATED_ACCEL,
  SENSOR_TYPE_COUNT  // Always keep it in the last position!
} SensorType;

inline SensorType operator++(SensorType& t) {
  t = static_cast<SensorType>(static_cast<int>(t) + 1);
  return t;
}

const std::vector<SensorType>& all_sensor_types() {
  static std::vector<SensorType> all_types;
  static bool initialized = false;

  if (!initialized) {
    for (SensorType t = TEMPERATURE_PRESSURE_ALTITUDE; t < SENSOR_TYPE_COUNT; ++t)
      all_types.push_back(t);
    initialized = true;
  }
  return all_types;
}

struct SensorInfo {
  const char* id;  // Short identifier, without space or special characters.
  const std::vector<const char*> fields;  // Name and unit of the sensor's fields.
};

struct TemperaturePressureAltitude {
  float temperature;
  float pressure;
  float altitude;

  static inline const SensorInfo info{"tpa", {"Temperature [°C]", "Pressure [hPa]", "Altitude [m]"}};
  bool read(BMP390& bmp, float sea_level_pressure) {
    if (bmp.read(temperature, pressure)) {
      altitude = bmp.calculate_altitude(pressure, sea_level_pressure);
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
};

struct Wind {
  float speed;

  static inline const SensorInfo info{"wind", {"Wind speed [m/s]"}};
  std::string as_string() const {
    return float_to_string(speed);
  }
};

struct Accelerometer {
  float acceleration;

  static inline const SensorInfo info{"accel", {"Acceleration [g]"}};
  bool read(Adafruit_LSM6DSO32& sox_sensor) {
    sensors_event_t accel, gyro, temp;
    if (sox_sensor.getEvent(&accel, &gyro, &temp)) {
      sensors_vec_t& a = accel.acceleration;
      acceleration = sqrt(a.x*a.x+a.y*a.y+a.z*a.z) / 9.807;
      return true;
    }
    return false;
  }
  std::string as_string() const {
    return float_to_string(acceleration);
  }
};

struct Ens {
  uint16_t tvoc;

  static inline const SensorInfo info{"tvoc", {"Tvoc [ppb]"}};
  bool read(ScioSense_ENS160& ens_sensor) {
    if (ens_sensor.available()) {
      ens_sensor.measure(true);
      ens_sensor.measureRaw(true);
      tvoc = ens_sensor.getTVOC();
      return true;
    }
    return false;
  }
  std::string as_string() const {
    return std::to_string(tvoc);
  }
};

struct Scd {
  uint16_t co2_ppm; //ppm
  float humidity; //[RH]

  static inline const SensorInfo info{"co2", {"CO2 [ppm]", "Humidity [%]"}};
  bool read(SensirionI2cScd4x& sensor) {
    bool dataReady = false;
    float temperature = 0.0;
    if (NO_ERROR == sensor.getDataReadyStatus(dataReady) and dataReady) {
      sensor.readMeasurement(co2_ppm, temperature, humidity);
      return true;
    }
    return false;
  }
  std::string as_string() const {
    return std::to_string(co2_ppm) + "\t" + float_to_string(humidity);
  }
};

struct Gps {
  int32_t latitude;
  int32_t longitude;
  float altitude;

  static inline const SensorInfo info{"gps", {"Latitude [degrees]", "Longitude [degrees]"}};
  bool read(Adafruit_GPS& gps_sensor) {
    while(gps_sensor.read());
    if (gps_sensor.newNMEAreceived())
      gps_sensor.parse(gps_sensor.lastNMEA());
    if (gps_sensor.fix) {
      latitude = gps_sensor.latitude_fixed;
      longitude = gps_sensor.longitude_fixed;
      altitude = gps_sensor.altitude;
      return true;    
    } else {
      return false;
    }
  }
  std::string as_string() const {
    auto lo = std::to_string(longitude);
    auto la = std::to_string(latitude);
    auto al = float_to_string(altitude);
    return lo + "\t" + la + "\t" + al;
  }
};

struct Max17 {
  float voltage;

  static inline const SensorInfo info{"battery", {"Voltage [V]"}};
  bool read(Adafruit_MAX17048& battery_monitor) {
    float vol = battery_monitor.cellVoltage();
    if (!isnan(vol) and vol != 0.0) {
      voltage = vol;
      return true;
    } else {
      return false;
    }
  }
  std::string as_string() const {
    return float_to_string(voltage);
  }
};

struct RadioTransmission {
  uint16_t id;
  uint16_t previous_transmission_duration;

  static inline const SensorInfo info{"transmission", {"Radio transmission id", "Previous transmission duration [ms]"}};
  std::string as_string() const {
    const auto i = std::to_string(id);
    const auto d = std::to_string(previous_transmission_duration);
    return i + "\t" + d;
  }
};

struct RadioReception {
  uint8_t length;
  uint8_t measurements;
  int16_t signal_strength;

  static inline const SensorInfo info{"reception", {"Payload length [bytes]", "Number of measurements", "Signal Strength [dBm]"}};
  std::string as_string() const {
    const auto l = std::to_string(length);
    const auto m = std::to_string(measurements);
    const auto s = std::to_string(signal_strength);
    return l + "\t" + m + "\t" + s;
  }
};

 struct CalculatedAccel {
   float acceleration;
   float velocity;
 
   static inline const SensorInfo info{"calcaccel", {"Acceleration [g]", "Velocity [m/s]"}};
   std::string as_string() const {
     const auto a = std::to_string(acceleration);
     const auto v = std::to_string(velocity);
     return a + "\t" + v;
   }
 };

const SensorInfo& sensor_info(const SensorType type) {
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
    case SCD:
      return Scd::info;
    case GPS:
      return Gps::info;
    case MAX17:
      return Max17::info;
    case RADIO_TRANSMISSION:
      return RadioTransmission::info;
    case RADIO_RECEPTION:
      return RadioReception::info;
    default:
      return undefined;
  }
}

union SensorValue {
  TemperaturePressureAltitude tpa;
  Wind wind;
  Accelerometer accelerometer;
  Ens ens;
  Scd scd;
  Gps gps;
  Max17 max17;
  RadioTransmission radio_transmission;
  RadioReception radio_reception;
  CalculatedAccel calculated_accel;
};

struct SensorMeasurement {
  uint32_t timestamp;  // Milliseconds since boot/midnight.
  SensorType type;
  SensorValue value;

  const SensorInfo& info() {
    return sensor_info(type);
  }

  std::string as_string() const {
    std::string s = timestamp_to_string(timestamp) + "\t";
    switch (type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        return s + value.tpa.as_string();
      case WIND_SPEED:
        return s + value.wind.as_string();
      case ACCELEROMETER:
        return s + value.accelerometer.as_string();
      case ENS:
        return s + value.ens.as_string();
      case SCD:
        return s + value.scd.as_string();
      case GPS:
        return s + value.gps.as_string();
      case MAX17:
        return s + value.max17.as_string();
      case RADIO_TRANSMISSION:
        return s + value.radio_transmission.as_string();
      case RADIO_RECEPTION:
        return s + value.radio_reception.as_string();
      default:
        return s + "???";
    }
  }

  size_t add_to_radio_message(uint8_t *message) const {
    size_t size = 0;
    size += pack(static_cast<uint8_t>(type), message+size);
    size += pack(timestamp, message+size);
    switch (type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        size += pack(value.tpa.temperature, message+size);
        size += pack(value.tpa.pressure, message+size);
        size += pack(value.tpa.altitude, message+size);
        break;
      case WIND_SPEED:
        size += pack(value.wind.speed, message+size);
        break;
      case ACCELEROMETER:
        size += pack(value.accelerometer.acceleration, message+size);
        break;
      case ENS:
        size += pack(value.ens.tvoc, message+size);
        break;
      case SCD:
        size += pack(value.scd.co2_ppm, message+size);
        size += pack(value.scd.humidity, message+size);
        break;
      case GPS:
        size += pack(value.gps.longitude, message+size);
        size += pack(value.gps.latitude, message+size);
        size += pack(value.gps.altitude, message+size);
        break;
      case MAX17:
        size += pack(value.max17.voltage, message+size);
        break;
      case RADIO_TRANSMISSION:
        size += pack(value.radio_transmission.id, message+size);
        size += pack(value.radio_transmission.previous_transmission_duration, message+size);
        break;
      case RADIO_RECEPTION:
        size += pack(value.radio_reception.length, message+size);
        size += pack(value.radio_reception.measurements, message+size);
        size += pack(value.radio_reception.signal_strength, message+size);
        break;
      case CALCULATED_ACCEL:
         size += pack(value.calculated_accel.acceleration, message+size);
         size += pack(value.calculated_accel.velocity, message+size);
         break;
    }
    return size;
  }

  size_t read_from_radio_message(uint8_t *message) {
    size_t size = 0;
    uint8_t type_uint8;
    size += unpack(type_uint8, message+size);
    type = static_cast<SensorType>(type_uint8);
    size += unpack(timestamp, message+size);
    switch (type) {
      case TEMPERATURE_PRESSURE_ALTITUDE:
        size += unpack(value.tpa.temperature, message+size);
        size += unpack(value.tpa.pressure, message+size);
        size += unpack(value.tpa.altitude, message+size);
        break;
      case WIND_SPEED:
        size += unpack(value.wind.speed, message+size);
        break;
      case ACCELEROMETER:
        size += unpack(value.accelerometer.acceleration, message+size);
        break;
      case ENS:
        size += unpack(value.ens.tvoc, message+size);
        break;
      case SCD:
        size += unpack(value.scd.co2_ppm, message+size);
        size += unpack(value.scd.humidity, message+size);
        break;
      case GPS:
        size += unpack(value.gps.longitude, message+size);
        size += unpack(value.gps.latitude, message+size);
        size += unpack(value.gps.altitude, message+size);
        break;
      case MAX17:
        size += unpack(value.max17.voltage, message+size);
        break;
      case RADIO_TRANSMISSION:
        size += unpack(value.radio_transmission.id, message+size);
        size += unpack(value.radio_transmission.previous_transmission_duration, message+size);
        break;
      case RADIO_RECEPTION:
        size += unpack(value.radio_reception.length, message+size);
        size += unpack(value.radio_reception.measurements, message+size);
        size += unpack(value.radio_reception.signal_strength, message+size);
        break;
    }
    return size;
  }
};


#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT
BaseType_t add_measurement_to_queue(const SensorMeasurement& measurement,
                                    QueueHandle_t& queue) {
  return xQueueSendToBack(queue, &measurement, pdMS_TO_TICKS(100));
}

BaseType_t add_measurement_to_queues(const SensorMeasurement& measurement,
                                    std::vector<QueueHandle_t> queues) {
  BaseType_t result = pdPASS;
  for (auto& queue : queues)
    if (add_measurement_to_queue(measurement, queue) != pdPASS)
     result = pdFAIL;
  return result;
}
#endif

#endif
