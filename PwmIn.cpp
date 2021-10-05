#include "PwmIn.h"


PwmIn::PwmIn(PinName gpio_in) :
m_pwmInput(gpio_in)
{
    // attach callbacks to the rising and falling edges
    m_pwmInput.rise(callback(this, &PwmIn::risingEdge));    
    m_pwmInput.fall(callback(this, &PwmIn::fallingEdge));  

    reinit();
}

void PwmIn::reinit()
{
    m_timer.stop();
    m_timer.reset();    
}

void PwmIn::risingEdge()
{
    m_timer.reset();
    m_timer.start();
    m_timeBegin = m_timer.elapsed_time().count();
}

void PwmIn::fallingEdge()
{
    m_timeEnd = m_timer.elapsed_time().count();
    m_timer.stop();
    m_timeInUs = m_timeEnd - m_timeBegin;
}