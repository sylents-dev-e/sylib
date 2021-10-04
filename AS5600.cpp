#include <mbed.h>
#include "AS5600.h"

AS5600::AS5600(I2C* pi2c)
{
    i2c = pi2c;
    revolutions = 0;
    offset = 0;
    first = 1;
}

long AS5600::getPosition()
{
    output = _getRegisters2(_RAWANGLEAddressMSB, _RAWANGLEAddressLSB);
    if (first)
    {
        last_output = output;
        first = 0;
    }
    if ((output - last_output) < -_AS5600_CPR_HALF)
    {
        revolutions++;
    }
    else if ((output - last_output) > _AS5600_CPR_HALF)
    {
        revolutions--;
    }
    last_output = output;
    return (output - offset) + _AS5600_CPR * revolutions;
}

int AS5600::getAngle()
{
    return _getRegisters2(_ANGLEAddressMSB, _ANGLEAddressLSB);
}

int AS5600::getStatus()
{
    return _getRegister(_STATUSAddress) & 0b00111000;
}

void AS5600::setZero()
{
    revolutions = 0;
    offset = _getRegisters2(_RAWANGLEAddressMSB, _RAWANGLEAddressLSB);
}

int AS5600::_getRegister(uint8_t register1)
{
    char regaddr[1] = {register1};
    char readdata[1];
    i2c->write(_AS5600_ADDR, regaddr, 1, true);
    i2c->read(_AS5600_ADDR, readdata, 1);

    _msb = readdata[0];

    return _msb;
}

long AS5600::_getRegisters2(uint8_t registerMSB, uint8_t registerLSB)
{
    _lsb = 0;
    _msb = 0;

    char regaddr[1] = {registerMSB};
    char readdata[1];
    i2c->write(_AS5600_ADDR, regaddr, 1, true);
    i2c->read(_AS5600_ADDR, readdata, 1);
    _msb = readdata[0];
    // wait_ms(10);

    regaddr[0] = registerLSB;
    i2c->write(_AS5600_ADDR, regaddr, 1, true);
    i2c->read(_AS5600_ADDR, readdata, 1);
    _lsb = readdata[0];

    return (_lsb) + (_msb & 0x0F) * 256;
}