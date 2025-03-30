#ifndef RSD_BMP390
#define RSD_BMP390

#include <Adafruit_I2CDevice.h>
#include "bmp3.h"

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

#endif
