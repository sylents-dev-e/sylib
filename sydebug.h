#include "BufferedSerial.h"
#include <mbed.h>


#define VESC_START_BYTE     0x2
#define VESC_END_BYTE       0x3

class Debug {

    public:

        Debug(BufferedSerial *pserial);

        void debug(char c);
        void debug(int d);
        void debug(const char * dbg);
        

    private:

        BufferedSerial*     m_pSerial;
        FILE*               m_pFile;

};