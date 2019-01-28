#include "calib.hpp" 
#include <iostream>

void calib_data::printCalibParameters()
{
    std::cout<<"T paramaters :"<<std::endl;
    std::cout<<"[T1] : "<<m_T1<<" ; "<<"[T2] : "<<m_T2<<" ; "<<"[T3] : "<<m_T3<<" ; "<<std::endl;
    std::cout<<"P paramaters :"<<std::endl;
    std::cout<<"[P1] : "<<m_P1<<" ; "<<"[P2] : "<<m_P2<<" ; "<<"[P3] : "<<m_P3<<"[P4] : "<<m_P4<<" ; "<<"[P5] : "<<m_P5<<" ; "<<"[P6] : "<<m_P6<<"[P7] : "<<m_P7<<" ; "<<"[P8] : "<<m_P8<<" ; "<<"[P9] : "<<m_P9<<" ; "<<std::endl;
    std::cout<<"H paramaters :"<<std::endl;
    std::cout<<"[H1] : "<<(uint8_t)m_H1<<" ; "<<"[H2] : "<<m_H2<<" ; "<<"[H3] : "<<(uint8_t) m_H3<<"[H4] : "<<m_H4<<" ; "<<"[H5] : "<<m_H5<<" ; "<<"[H6] : "<<(int8_t)m_H6<<std::endl;
}
