#ifndef CALIB_H_
#define CALIB_H_
#include <cstdint>
/*!
 * @brief Calibration data
 */
class calib_data {
 /**
 * @ Trim Variables
 */
/**@{*/
public:
    void setCalibParametersTemperature(const uint16_t& t1,const int16_t& t2,const int16_t& t3)
    {
        m_T1=t1;
        m_T2=t2;
        m_T3=t3;
    }
    void setCalibParametersPressure(const uint16_t& p1,const int16_t& p2,const int16_t& p3,const int16_t& p4,const int16_t& p5,const int16_t& p6,const int16_t& p7,const int16_t& p8,const int16_t& p9)
    {
        m_P1=p1;
        m_P2=p2;
        m_P3=p3;
        m_P4=p4;
        m_P5=p5;
        m_P6=p6;
        m_P7=p7;
        m_P8=p8;
        m_P9=p9;
    }
    void setCalibParametersHumidity(const uint8_t& h1,const int16_t& h2,const uint8_t& h3,const int16_t& h4,const int16_t& h5,const int8_t& h6)
    {
        m_H1=h1;
        m_H2=h2;
        m_H3=h3;
        m_H4=h4;
        m_H5=h5;
        m_H6=h6;
    }
    void setCalibParametersTimeFine(const int32_t& time)
    {
        m_t_fine = time;
    }
    void printCalibParameters();
	uint16_t m_T1;
	int16_t m_T2;
	int16_t m_T3;
	uint16_t m_P1;
	int16_t m_P2;
	int16_t m_P3;
	int16_t m_P4;
	int16_t m_P5;
	int16_t m_P6;
	int16_t m_P7;
	int16_t m_P8;
	int16_t m_P9;
	uint8_t  m_H1;
	int16_t m_H2;
	uint8_t  m_H3;
	int16_t m_H4;
	int16_t m_H5;
	int8_t  m_H6;
	int32_t m_t_fine;
/**@}*/
};
#endif
