/**
 * ORIENTATION OF THE STM32F30X IS BOTTEM RIGHT VBAT AND VDD PINS TOP RIGHT PA3
 */

#define BEEPE PC15 
//#define MPU PC13
#define SOFTSERIAL1_RX_PIN      PB4 // PWM 5
#define SOFTSERIAL1_TX_PIN      PB5 // PWM 6
#define SOFTSERIAL2_RX_PIN      PB0 // PWM 7
#define SOFTSERIAL2_TX_PIN      PB1 // PWM 8
#define ESCSERIAL_TIMER_TX_PIN  PA0  
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10
#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15
#define UART3_TX_PIN            PB10  // PB10 (AF7)
#define UART3_RX_PIN            PB11  // PB11 (AF7)
#define USE_SPI_DEVICE_2 	      // PB12,13,14,15 on AF5
#define FLASH_CS_PIN            PB12
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5
#define RSSI_ADC_PIN            PB2

/**
* 	I2C1_SCL PB6 
* 	I2C1_SDA PB7

    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_PWM | TIM_USE_PPM, 0), // RC_CH1 - PA0  - *TIM2_CH1
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_PWM,               0), // RC_CH2 - PA1  - *TIM2_CH2, TIM15_CH1N
    DEF_TIM(TIM2,  CH4, PB11, TIM_USE_PWM,               0), // RC_CH3 - PB11 - *TIM2_CH4, UART3_RX (AF7)
    DEF_TIM(TIM2,  CH3, PB10, TIM_USE_PWM,               0), // RC_CH4 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_PWM,               0), // RC_CH5 - PB4  - *TIM3_CH1
    DEF_TIM(TIM3,  CH2, PB5,  TIM_USE_PWM,               0), // RC_CH6 - PB5  - *TIM3_CH2
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_PWM,               0), // RC_CH7 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_PWM,               0), // RC_CH8 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    DEF_TIM(TIM16, CH1, PA6,  TIM_USE_MOTOR,             0), // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    DEF_TIM(TIM17, CH1, PA7,  TIM_USE_MOTOR,             0), // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    DEF_TIM(TIM4,  CH1, PA11, TIM_USE_MOTOR,             0), // PWM3 - PA11
    DEF_TIM(TIM4,  CH2, PA12, TIM_USE_MOTOR,             0), // PWM4 - PA12
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_MOTOR,             0), // PWM5 - PB8
    DEF_TIM(TIM4,  CH4, PB9,  TIM_USE_MOTOR,             0), // PWM6 - PB9
    DEF_TIM(TIM15, CH1, PA2,  TIM_USE_MOTOR,             0), // PWM7 - PA2
    DEF_TIM(TIM15, CH2, PA3,  TIM_USE_MOTOR,             0), // PWM8 - PA3
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_LED,               0), // GPIO_TIMER / LED_STRIP
*/


