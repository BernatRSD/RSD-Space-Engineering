#ifndef RSD_HELPER
#define RSD_HELPER

#include <string>

#define NO_ERROR 0

std::string float_to_string(float x, uint8_t precision) {
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

// Template function to unpack bytes into any type.
template <typename T>
size_t unpack(T &value, byte *bytes) {
  byte *valueAsBytes = (byte *)&value;
  size_t size = sizeof(T);
  for (size_t i = 0; i < size; i++) {
    valueAsBytes[i] = bytes[i];
  }
  return size;
}

#endif
