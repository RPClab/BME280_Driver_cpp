/*
  Linux userspace test code, simple and mose code directy from the doco.
  compile like this: gcc linux_userspace.c ../bme280.c -I ../ -o bme280
  tested: Raspberry Pi.
  Use like: ./bme280 /dev/i2c-0
*/
#include "bme280.hpp"
#include "I2C.hpp"

void print_sensor_data(data& dat)
{
#ifdef BME280_FLOAT_ENABLE
  printf("temp %0.2f, p %0.2f, hum %0.2f\r\n",dat.getTemperature(), dat.getPressure(), dat.getHumidity());
#else
  printf("temp %ld, p %ld, hum %ld\r\n",dat.getTemperature(),dat.getPressure(),dat.getHumidity());
#endif
}

int8_t stream_sensor_data_forced_mode(bme280& bm)
{
  int8_t rslt;
  uint8_t settings_sel;
  settings_sel = OSR_PRESS_SEL | OSR_TEMP_SEL | OSR_HUM_SEL | FILTER_SEL;
  rslt = bm.set_sensor_settings(settings_sel);
  printf("Temperature, Pressure, Humidity\r\n");
  /* Continuously stream sensor data */
rslt = bm.set_sensor_mode(FORCED_MODE);
    /* Wait for the measurement to complete and print data @25Hz */
bm.delay_ms(40);
    rslt = bm.get_sensor_data(ALL);
    print_sensor_data(bm.getData());
  return rslt;
}

int main(int argc, char* argv[])
{
  I2C i2c(argv[1],"0x76");
  I2C i2c2(argv[1],"0x77");
  settings setting;
  setting.setOversamplingPressure("16X");
  setting.setOversamplingHumidity("1X");
  setting.setOversamplingTemperature("2X");
  setting.setFilterCoefficient("16");
  i2c.connect();
  i2c2.connect();
  bme280 bm(i2c,setting);
  bme280 bm2(i2c2,setting);
  int8_t rslt = OK;
  rslt = bm.init();
  rslt = bm2.init();
  stream_sensor_data_forced_mode(bm);
  stream_sensor_data_forced_mode(bm2);
}
