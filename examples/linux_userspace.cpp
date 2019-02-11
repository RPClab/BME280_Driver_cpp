#include "bme280.hpp"
#include "I2C.hpp"
#include <vector>

void print_sensor_data(const data& dat)
{
#ifdef BME280_FLOAT_ENABLE
  printf("temp %0.2f, p %0.2f, hum %0.2f\r\n",dat.getTemperature(), dat.getPressure(), dat.getHumidity());
#else
  printf("temp %ld, p %ld, hum %ld\r\n",dat.getTemperature(),dat.getPressure(),dat.getHumidity());
#endif
}

int main(int argc, char* argv[])
{
  settings setting;
  setting.setOversamplingPressure("16X");
  setting.setOversamplingHumidity("1X");
  setting.setOversamplingTemperature("2X");
  setting.setFilterCoefficient("16");
  std::size_t NbrSensors{2};
  std::vector<I2C> i2cs;
  std::vector<bme280> bme280s;
  std::vector<std::string> Port{"0x76","0x77"};
  for(std::size_t sensor=0;sensor!=NbrSensors;++sensor)
  {
      i2cs.emplace_back(argv[1],Port[sensor]);
      i2cs[sensor].connect();
      bme280s.emplace_back(i2cs[sensor],setting);
      bme280s[sensor].init();
      print_sensor_data(bme280s[sensor].getDataForcedMode());
  }
 }
