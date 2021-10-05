/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BufferedSerial.h"
#include "DigitalOut.h"
#include "Mutex.h"
#include "ThisThread.h"
#include "UnbufferedSerial.h"
#include "mbed.h"

#include "defines.h"
#include "datatypes.h"
#include "crc.h"
#include "VescControl.h"
#include "AS5600.h"
#include "Debug.h"
#include "AppData.h"

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

AppData             appData;
Mutex               appDataMutex;

/*
namespace mbed
{
	FileHandle *mbed_override_console(int fd)
	{
	    static DirectSerial console(UART1_TX, UART1_RX, MBED_CONF_PLATFORM_STDIO_BAUD_RATE);
		return &console;
	}
}*/

Thread thread_100ms;
Thread thread_200ms;
Thread thread_500ms;
Thread thread_1000ms;

void task_100ms()
{
    while(true)
    {
        appDataMutex.lock();
        appData.m_pos = as5600.getPosition();
        appData.m_rpm = (appData.m_pos < 0 ) ? 0 : (500 + (appData.m_pos & MAX_RPM)); 
        appDataMutex.unlock();

        ThisThread::sleep_for(100ms);
    }
}

void task_200ms()
{
    while(true)
    {
        led = !led;
        appDataMutex.lock();        
        vesc.SetRpm(appData.m_rpm);
        appDataMutex.unlock();

        ThisThread::sleep_for(200ms);
    }
}

void task_500ms()
{
    while(true)
    {
        ThisThread::sleep_for(500ms);
    }
}

void task_1000ms()
{
    while(true)
    {
        appDataMutex.lock();
        appData.m_motorValues = vesc.GetMotorValues();
        printf("\nVin: %d, RPM: %d", (int)appData.m_motorValues.v_in, (int)appData.m_motorValues.rpm);      
        appDataMutex.unlock();

        ThisThread::sleep_for(1000ms);
    }
}



void init()
{
    printf("STARTING..");
    as5600.setZero();
    printf("ready!");
}

int main()
{      
    init();

    thread_100ms.start(task_100ms);
    thread_200ms.start(task_200ms);
    thread_500ms.start(task_500ms);
    thread_1000ms.start(task_1000ms);   
}

