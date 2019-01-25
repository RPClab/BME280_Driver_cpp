/*
  Linux userspace test code, simple and mose code directy from the doco.
  compile like this: gcc linux_userspace.c ../bme280.c -I ../ -o bme280
  tested: Raspberry Pi.
  Use like: ./bme280 /dev/i2c-0
*/
#include "bme280.hpp"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
  printf("temp %0.2f, p %0.2f, hum %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
  printf("temp %ld, p %ld, hum %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

int8_t stream_sensor_data_forced_mode(bme280& bm)
{
  int8_t rslt;
  uint8_t settings_sel;
  struct bme280_data comp_data;

  /* Recommended mode of operation: Indoor navigation */
  bm.m_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  bm.m_dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  bm.m_dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  bm.m_dev.settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bm.bme280_set_sensor_settings(settings_sel, &bm.m_dev);

  printf("Temperature, Pressure, Humidity\r\n");
  /* Continuously stream sensor data */
  while (1) {
    rslt = bm.bme280_set_sensor_mode(BME280_FORCED_MODE, &bm.m_dev);
    /* Wait for the measurement to complete and print data @25Hz */
    bm.delay_ms(40);
    rslt = bm.bme280_get_sensor_data(BME280_ALL, &comp_data, &bm.m_dev);
    print_sensor_data(&comp_data);
  }
  return rslt;
}

int main(int argc, char* argv[])
{
  I2C i2c(argv[1],"0x76");
  i2c.connect();
  bme280 bm;
  bm.m_IO=&i2c;
  int8_t rslt = BME280_OK;

  rslt = bm.bme280_init(&bm.m_dev);
  stream_sensor_data_forced_mode(bm);
}
