#include "BufferedSerial.h"
#include <mbed.h>
#include "datatypes.h"

#define HEADER_LENGTH_SMALL_PACKET        5
#define HEADER_LENGTH_BIG_PACKET          6

#define PLOAD_INDEX_SMALL_PACKET          2
#define PLOAD_INDEX_BIG_PACKET            3

#define CRCH_TAIL_OFFSET                  3
#define CRCL_TAIL_OFFSET                  2

#define PACKET_LENGTH_MOTOR_VALUES       78
#define PACKET_LENGTH_MOTOR_CONFIG      464
#define PACKET_LENGTH_APP_CONFIG        425

#define PLOAD_LENGTH_MOTOR_VALUES       PACKET_LENGTH_MOTOR_VALUES-HEADER_LENGTH_SMALL_PACKET
#define PLOAD_LENGTH_MOTOR_CONFIG       PACKET_LENGTH_MOTOR_CONFIG-HEADER_LENGTH_BIG_PACKET
#define PLOAD_LENGTH_APP_CONFIG         PACKET_LENGTH_APP_CONFIG-HEADER_LENGTH_BIG_PACKET




class VescControl {


    public:
        VescControl(BufferedSerial *pserial);

        // Setters
        int SetRpm(unsigned int rpm);
        int SetCurrent(float current);
        int SetCurrentBrake(float current);
        int SetDutyCycle(float dutyCycle);
        int SetMotorConfig(motor_config_t* pMotorConfig);
        int SetAppConfig(app_config_t* pAppConfig);
        
        // Getters
        motor_values_t GetMotorValues();
        void GetFirmwareVersion();
        void GetMotorConfig(bool bReadDefaultConfig = false);
        void GetAppConfig(bool bReadDefaultConfig = false);

        void RebootController();
        void SendAlive();

        bool Read();

        const char* GetRxBuff() {   return m_rxData.rxBuff;    };


    private:

        int     send_command(unsigned int payload_length);

        void    read_motor_values(const char* payload, size_t length);
        void    read_motor_config(const char* payload, size_t length, bool bIsDefaultConfig = false);
        void    read_app_config(const char* payload, size_t length, bool bIsDefaultConfig = false);
    
        unsigned short      m_crc;
        char                m_txBuff[512];      
        char                m_payloadBuff[512];  
        vesc_rx_t           m_rxData;
        BufferedSerial*     m_pSerial;

        CMD_PACKET_ID       m_cmdIdRequested;

        motor_values_t      m_motorValues;
        motor_config_t      m_motorConfig;
        motor_config_t      m_motorConfigDefault;
        app_config_t        m_appConfig;
        app_config_t        m_appConfigDefault;
};