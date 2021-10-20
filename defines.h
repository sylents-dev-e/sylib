


#ifdef TARGET_NUCLEO_F466RE
// I2C Definitions
//    AS5600      as5600(PB_9, PB_8);
#define I2C1_SDA PB_9
#define I2C1_SCL PB_8

// UART Definitions
//uartvesc(PA_9, PA_10); // TX; RX
#define UART1_TX PA_9
#define UART1_RX PA_10

#define UART2_TX PA_2
#define UART2_RX PA_3

#define UART3_TX PC_10
#define UART3_RX PC_11

#define PWM_IN  PC_9
#endif






#ifdef TARGET_NUCLEO_L432KC
#define I2C1_SDA PA_10
#define I2C1_SCL PA_9

// UART Definitions
#define UART1_TX PB_6
#define UART1_RX PB_7

#define UART2_TX PA_2  //PA_2
#define UART2_RX PA_15   //PA_3

#define PWM_IN  PA_7

#endif

// VESC related definitions
#define VESC_START_BYTE_SMALL     0x2
#define VESC_START_BYTE_BIG       0x3
#define VESC_END_BYTE             0x3
#define VESC_MAX_LENGTH           512
