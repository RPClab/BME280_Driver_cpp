#include "data.hpp"
#ifdef BME280_FLOAT_ENABLE
/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
void data::compensate_temperature()
{
	double var1{0};
	double var2{0};
	double temperature_min{-40};
	double temperature_max{85};
	var1 = ((double)m_uncomp_temperature) / 16384.0 - ((double)m_calib_data.m_T1) / 1024.0;
	var1 = var1 * ((double)m_calib_data.m_T2);
	var2 = (((double)m_uncomp_temperature) / 131072.0 - ((double)m_calib_data.m_T1) / 8192.0);
	var2 = (var2 * var2) * ((double)m_calib_data.m_T3);
	m_calib_data.m_t_fine = (int32_t)(var1 + var2);
	m_temperature = (var1 + var2) / 5120.0;
	if (m_temperature < temperature_min) m_temperature = temperature_min;
	else if (m_temperature > temperature_max) m_temperature = temperature_max;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
void data::compensate_pressure()
{
	double var1{0};
	double var2{0};
	double var3{0};
	double pressure_min{30000.0};
	double pressure_max{110000.0};
	var1 = ((double)m_calib_data.m_t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)m_calib_data.m_P6) / 32768.0;
	var2 = var2 + var1 * ((double)m_calib_data.m_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)m_calib_data.m_P4) * 65536.0);
	var3 = ((double)m_calib_data.m_P3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)m_calib_data.m_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)m_calib_data.m_P1);
	/* avoid exception caused by division by zero */
	if (var1) 
    {
		m_pressure = 1048576.0 - (double) m_uncomp_pressure;
		m_pressure = (m_pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)m_calib_data.m_P9) * m_pressure * m_pressure / 2147483648.0;
		var2 = pressure * ((double)m_calib_data.m_P8) / 32768.0;
		m_pressure = m_pressure + (var1 + var2 + ((double)m_calib_data.m_P7)) / 16.0;
		if (m_pressure < pressure_min) m_pressure = pressure_min;
		else if (m_pressure > pressure_max) m_pressure = pressure_max;
	} 
	else 
    { /* Invalid case */
		m_pressure = pressure_min;
	}
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
void data::compensate_humidity()
{
	double humidity_min{0.0};
	double humidity_max{100.0};
	double var1{0};
	double var2{0};
	double var3{0};
	double var4{0};
	double var5{0};
	double var6{0};
	var1 = ((double)m_calib_data.m_t_fine) - 76800.0;
	var2 = (((double)m_calib_data.m_H4) * 64.0 + (((double)m_calib_data.m_H5) / 16384.0) * var1);
	var3 = m_uncomp_humidity - var2;
	var4 = ((double)m_calib_data.m_H2) / 65536.0;
	var5 = (1.0 + (((double)m_calib_data.m_H3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)m_calib_data.m_H6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	m_humidity = var6 * (1.0 - ((double)m_calib_data.m_H1) * var6 / 524288.0);
	if (m_humidity > humidity_max) m_humidity = humidity_max;
	else if (m_humidity < humidity_min) m_humidity = humidity_min;
}

#else
/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
void data::compensate_temperature()
{
	int32_t var1{0};
	int32_t var2{0};
	int32_t temperature_min{-4000};
	int32_t temperature_max{8500};
	var1 = (int32_t)((m_uncomp_temperature / 8) - ((int32_t)m_calib_data.m_T1 * 2));
	var1 = (var1 * ((int32_t)m_calib_data.m_T2)) / 2048;
	var2 = (int32_t)((m_uncomp_temperature / 16) - ((int32_t)m_calib_data.m_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)m_calib_data.m_T3)) / 16384;
	m_calib_data.m_t_fine = var1 + var2;
	m_temperature = (m_calib_data.m_t_fine * 5 + 128) / 256;
	if (m_temperature < temperature_min) m_temperature = temperature_min;
	else if (m_temperature > temperature_max) m_temperature = temperature_max;
}
#ifdef BME280_64BIT_ENABLE
/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type with higher
 * accuracy.
 */
void data::compensate_pressure()
{
	int64_t var1{0};
	int64_t var2{0};
	int64_t var3{0};
	int64_t var4{0};
	uint32_t pressure_min{3000000};
	uint32_t pressure_max{11000000};
	var1 = ((int64_t)m_calib_data.m_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)m_calib_data.m_P6;
	var2 = var2 + ((var1 * (int64_t)m_calib_data.m_P5) * 131072);
	var2 = var2 + (((int64_t)m_calib_data.m_P4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)m_calib_data.m_P3) / 256) + ((var1 * ((int64_t)m_calib_data.m_P2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * ((int64_t)m_calib_data.m_P1) / 8589934592;
	/* To avoid divide by zero exception */
	if (var1 != 0) 
    {
		var4 = 1048576 - m_uncomp_pressure;
		var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
		var1 = (((int64_t)m_calib_data.m_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
		var2 = (((int64_t)m_calib_data.m_P8) * var4) / 524288;
		var4 = ((var4 + var1 + var2) / 256) + (((int64_t)m_calib_data.m_P7) * 16);
		m_pressure = (uint32_t)(((var4 / 2) * 100) / 128);
		if (m_pressure < pressure_min) m_pressure = pressure_min;
		else if (m_pressure > pressure_max) m_pressure = pressure_max;
	} 
	else 
    {
		m_pressure = pressure_min;
	}
}
#else
/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
void data::compensate_pressure()
{
	int32_t var1{0};
	int32_t var2{0};
	int32_t var3{0};
	int32_t var4{0};
	uint32_t var5{0};
	uint32_t pressure_min{30000};
	uint32_t pressure_max{110000};
	var1 = (((int32_t)m_calib_data.m_t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)m_calib_data.m_P6);
	var2 = var2 + ((var1 * ((int32_t)m_calib_data.m_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)m_calib_data.m_P4) * 65536);
	var3 = (m_calib_data.m_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)m_calib_data.m_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)m_calib_data.m_P1)) / 32768;
	 /* avoid exception caused by division by zero */
	if (var1) 
    {
		var5 = (uint32_t)((uint32_t)1048576) - m_uncomp_pressure;
		m_pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (m_pressure < 0x80000000) m_pressure = (m_pressure << 1) / ((uint32_t)var1);
		else m_pressure = (m_pressure / (uint32_t)var1) * 2;
		var1 = (((int32_t)m_calib_data.m_P9) * ((int32_t)(((m_pressure / 8) * (m_pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(m_pressure / 4)) * ((int32_t)m_calib_data.m_P8)) / 8192;
		m_pressure = (uint32_t)((int32_t)m_pressure + ((var1 + var2 + m_calib_data.m_P7) / 16));
		if (m_pressure < pressure_min) m_pressure = pressure_min;
		else if (m_pressure > pressure_max) m_pressure = pressure_max;
	} 
	else 
    {
		m_pressure = pressure_min;
	}
}
#endif

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
void data::compensate_humidity()
{
	int32_t var1{0};
	int32_t var2{0};
	int32_t var3{0};
	int32_t var4{0};
	int32_t var5{0};
	uint32_t humidity_max{102400};
	var1 = m_calib_data.m_t_fine - ((int32_t)76800);
	var2 = (int32_t)(m_uncomp_humidity * 16384);
	var3 = (int32_t)(((int32_t)m_calib_data.m_H4) * 1048576);
	var4 = ((int32_t)m_calib_data.m_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)m_calib_data.m_H6)) / 1024;
	var3 = (var1 * ((int32_t)m_calib_data.m_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)m_calib_data.m_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)m_calib_data.m_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	m_humidity = (uint32_t)(var5 / 4096);
	if (m_humidity > humidity_max)m_humidity = humidity_max;
}
#endif
