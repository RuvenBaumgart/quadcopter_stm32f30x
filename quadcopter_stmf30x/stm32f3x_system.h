#define STM32F30X
#define SYSCLK_FREQ_72MHz
#define AIRCR_VECTKEY_MASK ((uint32_t) 0x05FA0000)


void checkForBootLoaderRequest(void);
void systemReset(void);
void systemResetToBootloader(void);
void stm32f3x_init(void);
void init_timer_pwm(void);
void init_timer_ppm(void);
void initBoardLed(void);
