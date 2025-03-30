#ifndef RSD_SENSOR_LOGGING
#define RSD_SENSOR_LOGGING

#include <sstream>

#include "rsd_sd_card.h"
#include "rsd_sensors.h"

SdFile sensor_logs[SENSOR_TYPE_COUNT];

bool write_log_file_header(SdFile& log_file, const SensorInfo& sensor_info) {
  if (log_file) {
    log_file.println("---------------");
    std::ostringstream oss;
    oss << "Timestamp";
    for (auto field_name : sensor_info.fields) {
      oss << "\t" << field_name;
    }
    log_file.printf("%s\n", oss.str().c_str());
    return true;
  }
  return false;
}

bool initialize_log_file(SdFile& log_file, const SensorInfo& sensor_info) {
  std::string file_name{sensor_info.id};
  file_name += ".txt";
  if (open_for_appending_or_writing(log_file, file_name.c_str())) {
    if (write_log_file_header(log_file, sensor_info)) {
      #ifdef RSD_DEBUG
      Serial.printf("Added header to %s\n", file_name.c_str());
      #endif
      return true;
    }
  }
  #ifdef RSD_DEBUG
  Serial.printf("Failed to add header to %s\n", file_name.c_str());
  #endif
  return false;
}

bool initialize_log_files(const std::vector<SensorType>& sensor_types) {
  if (sd_card_ok || init_sd_card()) {
    bool success = true;
    for (const auto sensor_type : sensor_types)
      if (!initialize_log_file(sensor_logs[sensor_type], sensor_info(sensor_type)))
        success = false;
    return success;
  }
  return false;
}

void write_to_sd_card(std::vector<SensorMeasurement>& measurements) {
  SensorMeasurement measurement;
  std::vector<bool> seen(SENSOR_TYPE_COUNT, false);
  #ifdef RSD_DEBUG
  unsigned long start = micros();
  #endif

  for (const auto& measurement : measurements) {
    const auto sensor_type = measurement.type;
    if (sensor_logs[sensor_type]) {
      sensor_logs[sensor_type].println(measurement.as_string().c_str());
      seen[sensor_type] = true;
    }
  }
  for (const auto& sensor_type : all_sensor_types())
    if (seen[sensor_type])
      sensor_logs[sensor_type].FatFile::flush();
  
  #ifdef RSD_DEBUG
  unsigned long finish = micros();
  Serial.printf("Writing %d measurement(s) to SD card took %.3f ms\n", measurements.size(), (finish - start) * 0.001);
  #endif
}

#endif