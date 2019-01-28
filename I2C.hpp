#ifndef I2C_H_
#define I2C_H_
#include "IO.hpp"
#include "sys/ioctl.h"
#include "linux/i2c-dev.h"
#include "cstring"
#include "fcntl.h"
#include "unistd.h"

class I2C : public IO
{
public:
    I2C(const std::string& path,const std::string& adress):m_path(path)
    {
        m_adress=static_cast<uint8_t>(std::stoi(adress,0,16));
        m_interfaceName="I2C";
    };
    virtual int8_t read(uint8_t reg_addr,uint8_t *data, uint16_t len)
    {
        ::write(m_fd, &reg_addr,1);
        ::read(m_fd, data, len);
        return 0; 
    }
    virtual int8_t write(uint8_t reg_addr,uint8_t *data, uint16_t len)
    {
        int8_t *buf;
        buf =static_cast<int8_t*>(malloc(len +1));
        buf[0] = reg_addr;
        memcpy(buf +1, data, len);
        ::write(m_fd, buf, len +1);
        free(buf);
        return 0;
    }
    int8_t connect()
    {
        if((m_fd = open(m_path.c_str(),O_RDWR)) < 0) 
        {
            printf("Failed to open the i2c bus %s",m_path.c_str());
            exit(1);
        }
        if(ioctl(m_fd,I2C_SLAVE,m_adress) < 0) 
        {
            printf("Failed to acquire bus access and/or talk to slave.\n");
            exit(1);
        }
    }
private:
    int m_fd;
    std::string m_path{""};
    uint8_t m_adress;
};
#endif
