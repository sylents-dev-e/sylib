/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BufferedSerial.h"
#include "UnbufferedSerial.h"
#include "mbed.h"

#include "defines.h"
#include "datatypes.h"
#include "crc.h"
#include "VescControl.h"
#include "AS5600.h"
#include "Debug.h"

// Blinking rate in milliseconds
#define BLINKING_RATE     1000ms
#define MAX_RPM           16000

I2C                 i2c(I2C1_SDA , I2C1_SCL);
DigitalOut          led(LED1);
AS5600              as5600(&i2c);
BufferedSerial      uartvesc(UART3_TX, UART3_RX, 115200);
BufferedSerial      uartDebug(UART1_TX, UART1_RX, 115200);
//Debug               dbg(&uartDebug);
VescControl         vesc(&uartvesc);

/*
namespace mbed
{
	FileHandle *mbed_override_console(int fd)
	{
	    static BufferedSerial console(PA_2, PA_3, MBED_CONF_PLATFORM_STDIO_BAUD_RATE);
		return &console;
	}
}*/

// set baud rate of serial port to 115200
/*
static BufferedSerial uartDebug(UART1_TX, UART1_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd) {
    return &uartDebug;
}
*/

int main()
{      
    int angle = 0;
    long pos = 0;
    unsigned int rpm = 3000;

    uartvesc.set_blocking(false);
    uartDebug.set_blocking(false);

    printf("STARTING..");

    as5600.setZero();

    printf("ready!");

    short readStatus = 0;

    char buf[78] = {0};
    char *pBuffer = (char*) vesc.GetRxBuff();

    while (1) 
    {
        ThisThread::sleep_for(300ms);

        pos = as5600.getPosition();

        //printf("\n%ld", pos);

        if( pos < 0 )
        {
            rpm = 0;
        }
        else
        {
            rpm = 500 + (pos & MAX_RPM);
        }
        
        led = !led;

        if( readStatus++ < 10 )
        {
            vesc.SetRpm(rpm);
        }
        else
        {
            readStatus = 0;
            vesc.GetAppConfig();
            //vesc.GetMotorConfig();
            //vesc.GetMotorValues();

            ThisThread::sleep_for(50ms); 

            if( vesc.Read() )
            {
                uartDebug.write(pBuffer, PACKET_LENGTH_MOTOR_CONFIG);
            }
        }

    }
}

