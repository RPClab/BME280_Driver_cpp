#ifndef IO_H_
#define IO_H_
#include <cstdint>
#include <string>
class IO
{
public:
    virtual int8_t read(uint8_t reg_addr,uint8_t *data, uint16_t len)=0;
    virtual int8_t write(uint8_t reg_addr,uint8_t *data, uint16_t len)=0;
    virtual ~IO(){};
    std::string& getInterfaceName()
    {
        return m_interfaceName;
    }
protected:
    std::string m_interfaceName{""};
};
#endif
