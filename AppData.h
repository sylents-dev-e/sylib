#include <mbed.h>
#include "datatypes.h"

class AppData {

    public:

        AppData() {
            m_pos = 0;
            m_rpm = 3000;
        };        

    public:

        long            m_pos = 0;
        unsigned int    m_rpm = 3000;
        motor_values_t  m_motorValues;

};