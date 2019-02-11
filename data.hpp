#ifndef DATA_H_
#define DATA_H_
#include <cstdint>
#include "calib.hpp"
 /*!
 * @brief bme280 sensor class which comprises of temperature, pressure and
 * humidity data
 */
class data 
{
public:
    void reset()
    {
        resetUncompensated();
        resetCompensated();
    }
    calib_data& returnCalibData()
    {
        return m_calib_data;
    }
    void resetUncompensated()
    {
        m_uncomp_pressure={0};
        m_uncomp_temperature={0};
        m_uncomp_humidity={0};
    }
    void resetCompensated()
    {
        m_pressure={0};
        m_temperature={0};
        m_humidity={0};
    }
        #ifdef BME280_FLOAT_ENABLE
    double getTemperature() const
    {
        return m_temperature;
    }
    double getPressure() const
    {
        return m_pressure;
    }
    double getHumidity() const 
    {
        return m_humidity;
    }
    #else
    uint32_t getTemperature() const 
    {
        return m_temperature;
    }
    uint32_t getPressure() const 
    {
        return m_pressure;
    }
    uint32_t getHumidity() const 
    {
        return m_humidity;
    }
    #endif
    void setUncompPressure(const uint8_t& p1,const uint8_t& p2,const uint8_t& p3)
    {
        uint32_t data_xlsb{(uint32_t)p3 >> 4};
        uint32_t data_lsb{(uint32_t)p2 << 4};
        uint32_t data_msb{(uint32_t)p1 << 12};
        m_uncomp_pressure=data_msb | data_lsb | data_xlsb;
    }
    void setUncompTemperature(const uint8_t& t1,const uint8_t& t2,const uint8_t& t3)
    {
        uint32_t data_xlsb{(uint32_t)t3 >> 4};
        uint32_t data_lsb{(uint32_t)t2 << 4};
        uint32_t data_msb{(uint32_t)t1 << 12};
        m_uncomp_temperature=data_msb | data_lsb | data_xlsb;
    }
    void setUncompHumidity(const uint8_t& h1,const uint8_t& h2)
    {
        uint32_t data_lsb{(uint32_t)h1 << 8};
        uint32_t data_msb{(uint32_t)h2};
        m_uncomp_humidity=data_msb | data_lsb;
    }
    /*!
    * @brief This internal API is used to compensate the raw pressure data and
    * return the compensated pressure data in double data type.
    * @return Compensated pressure data.
    * @retval Compensated pressure data in double.
    */
    void compensate_pressure();

    /*!
    * @brief This internal API is used to compensate the raw humidity data and
    * return the compensated humidity data in double data type.
    * @return Compensated humidity data.
    * @retval Compensated humidity data in double.
    */
    void compensate_humidity();

    /*!
    * @brief This internal API is used to compensate the raw temperature data and
    * return the compensated temperature data in double data type.
    * @return Compensated temperature data.
    * @retval Compensated temperature data in double.
    */
    void compensate_temperature();
private:
    calib_data m_calib_data;
    #ifdef BME280_FLOAT_ENABLE
	/*! Compensated pressure */
	double m_pressure;
	/*! Compensated temperature */
	double m_temperature;
	/*! Compensated humidity */
	double m_humidity;
    #else
    /*! Compensated pressure */
	uint32_t m_pressure;
	/*! Compensated temperature */
	int32_t m_temperature;
	/*! Compensated humidity */
	uint32_t m_humidity;
    #endif /* BME280_USE_FLOATING_POINT */
    /*! un-compensated pressure */
	uint32_t m_uncomp_pressure;
	/*! un-compensated temperature */
	uint32_t m_uncomp_temperature;
	/*! un-compensated humidity */
	uint32_t m_uncomp_humidity;
};
#endif
