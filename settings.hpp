#ifndef SETTINGS_H_
#define SETTINGS_H_
#include <map>
#include <iostream>
class settings
{
public:
    void setOversamplingPressure(const std::string& over)
    {
        std::map<std::string,uint8_t>::iterator found=Oversampling.find(over);
        if(found!=Oversampling.end()) osr_p=found->second;
        else
        {
            std::cout<<"Unknowm Oversampling option"<<std::endl;
        }
    }
    void setOversamplingTemperature(const std::string& over)
    {
        std::map<std::string,uint8_t>::iterator found=Oversampling.find(over);
        if(found!=Oversampling.end()) osr_t=found->second;
        else
        {
            std::cout<<"Unknowm Oversampling option"<<std::endl;
        }
    }
    void setOversamplingHumidity(const std::string& over)
    {
        std::map<std::string,uint8_t>::iterator found=Oversampling.find(over);
        if(found!=Oversampling.end()) osr_h=found->second;
        else
        {
            std::cout<<"Unknowm Oversampling option"<<std::endl;
        }
    }
    void setStandbyTime(const std::string& time)
    {
        std::map<std::string,uint8_t>::iterator found=StandbyTime.find(time);
        if(found!=Oversampling.end()) standby_time=found->second;
        else
        {
            std::cout<<"Unknowm StandbyTimne option"<<std::endl;
        }
    }
    void setFilterCoefficient(const std::string& coeff)
    {
        std::map<std::string,uint8_t>::iterator found=FilterCoefficient.find(coeff);
        if(found!=Oversampling.end()) standby_time=found->second;
        else
        {
            std::cout<<"Unknowm FilterCoefficient option"<<std::endl;
        }
    }
    uint8_t getOversamplingPressure()
    {
        return osr_p;
    }
    uint8_t getOversamplingTemperature()
    {
        return osr_t;
    }
    uint8_t getOversamplingHumidity()
    {
        return osr_h;
    }
    uint8_t getFilterCoefficient()
    {
        return filter;
    }
    uint8_t getStandbyTime()
    {
        return standby_time;
    }
    
    void setOversamplingPressure(uint8_t val)
    {
        osr_p=val;
    }
    void setOversamplingTemperature(uint8_t val)
    {
        osr_t=val;
    }
    void setOversamplingHumidity(uint8_t val)
    {
        osr_h=val;
    }
    void setFilterCoefficient(uint8_t val)
    {
        filter=val;
    }
    void setStandbyTime(uint8_t val)
    {
        standby_time=val;
    }
private:
    /*! pressure oversampling */
	uint8_t osr_p;
	/*! temperature oversampling */
	uint8_t osr_t;
	/*! humidity oversampling */
	uint8_t osr_h;
	/*! filter coefficient */
	uint8_t filter;
	/*! standby time */
	uint8_t standby_time;
    /**\name Oversampling macros **/
    std::map<std::string,uint8_t> Oversampling{{"NO_OVERSAMPLING",0x00},{"1X",0x01},{"2X",0x02},{"4X",0x03},{"8X",0x04},{"16X",0x05}};
    std::map<std::string,uint8_t> StandbyTime{{"1ms",0x00},{"62.5ms",0x01},{"125ms",0x02},{"250ms",0x03},{"500ms",0x04},{"1000ms",0x05},{"10ms",0x06},{"20ms",0x07}};
    std::map<std::string,uint8_t> FilterCoefficient{{"OFF",0x00},{"2",0x01},{"4",0x02},{"8",0x03},{"16",0x04}};
};
#endif
