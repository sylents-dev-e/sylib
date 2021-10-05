#include "InterruptIn.h"
#include "PinNames.h"
#include "mbed.h"

class PwmIn {

    public:

        PwmIn(PinName gpio_in);

        void reinit();

        long GetTime()  {   return m_timeInUs;  };

    private:

        void risingEdge();
        void fallingEdge();

    private:

        InterruptIn     m_pwmInput;
        Timer           m_timer;
        volatile long   m_timeInUs;
        long            m_timeBegin;
        long            m_timeEnd;

};