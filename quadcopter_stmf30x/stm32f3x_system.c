#include <stm32f30x.h>
#include "stm32f3x_system.h"

void init_timer_pwm(){
	/**
	 * Clock for GPIO A is initialized in the usart.c
	 * Clock for GPIO B is initialized in initBoardLed
	 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);      

  /**
   * GPIOA Pin 6 and 7 for the PWM1 and2
   */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);

  /**
   * GPIOA Pin 11 and 12 for PWM 3 and 4
   */
   
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_10);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_10);
  /**
   * GIPOB Pin 8 and 9 for the Gimbal
   */
        
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);
	
  GPIO_InitTypeDef GPIOA_pwm_init;
  GPIOA_pwm_init.GPIO_Mode = GPIO_Mode_AF;
  GPIOA_pwm_init.GPIO_OType = GPIO_OType_PP;
  GPIOA_pwm_init.GPIO_Pin = (GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12 );
  GPIOA_pwm_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIOA_pwm_init.GPIO_Speed = GPIO_Speed_Level_3;	
  GPIO_Init(GPIOA, &GPIOA_pwm_init);


  GPIO_InitTypeDef GPIOB_pwm_init;
  GPIOB_pwm_init.GPIO_Mode = GPIO_Mode_AF;
  GPIOB_pwm_init.GPIO_OType = GPIO_OType_PP;
  GPIOB_pwm_init.GPIO_Pin = (GPIO_Pin_8 | GPIO_Pin_9);
  GPIOB_pwm_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIOB_pwm_init.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOB, &GPIOB_pwm_init);
   
	/**
	 * GPIOA 6 & 7 are using Timer 3 Channel 1 and 2
	 */	

	TIM_DeInit(TIM3);
	TIM_CounterModeConfig(TIM3, TIM_CounterMode_Up); //CR1 DIR
	TIM3->CR1 |=(0x01 << 7); //Auto reload preload enable bit set --> TIM3 ARR Register is buffered
	TIM3->CCMR1 |= (0x06 << 4 ) | (0x01 << 3) | (0x06 << 12) | (0x01 << 11);
	TIM3->CCER |= (0x01 ) | (0x01 << 4); 
	TIM3->PSC = 8; //72 / 8 - 1 for oneshot125
    	TIM3->ARR = 2000; // Set the refresh rate to 1/2000 500Hz
	TIM3->CCR1 = 985;
	TIM3->CCR2 = 985;
	//TIM3->EGR |= 0x01;	
	TIM_Cmd(TIM3, ENABLE); //CEN Bit gets written

	/**
	 * GPIOA 11 and 12 are using Timer 4 Channel 1 and 2
 	 */
	TIM_DeInit(TIM4);
	TIM_CounterModeConfig(TIM4, TIM_CounterMode_Up); //CR1 DIR
	TIM4->CR1 |=(0x01 << 7); //Auto reload preload enable bit set --> TIM3 ARR Register is buffered
	TIM4->CCMR1 |= (0x06 << 4 ) | (0x01 << 3) | (0x06 << 12) | (0x01 << 11);
	TIM4->CCMR2 |=  (0x06 << 4 ) | (0x01 << 3) | (0x06 << 12) | (0x01 << 11);
	TIM4->CCER |= (0x01 ) | (0x01 << 4) | (0x01 << 8) | (0x01 << 12); 
	TIM4->PSC = 8;
    	TIM4->ARR = 2000; // Set the refresh rate to 1/2000 500Hz
	TIM4->CCR1 = 985;
	TIM4->CCR2 = 985;
        TIM_Cmd(TIM4, ENABLE); //CEN Bit gets written

        TIM_DeInit(TIM16);
        TIM16->CR1 |= (0x01 << 7);
        TIM16->CCMR1 |= (0x06 << 4 ) | (0x01 << 3);
        TIM16->CCER |= (0x01);
        TIM16->PSC = 71; 
        TIM16->ARR = 3999;
        TIM16->CCR1 = 2000;
        TIM16->BDTR |= TIM_BDTR_MOE;
        TIM16->EGR |= 0x01;
        TIM_Cmd(TIM16, ENABLE); 
 
        TIM_DeInit(TIM17);
        TIM17->CR1 |= (0x01 << 7);
  	TIM17->CCMR1 |= (0x06 << 4 ) | (0x01 << 3);
        TIM17->CCER |= (0x01);
        TIM17->PSC = 71; 
        TIM17->ARR = 3999;
        TIM17->CCR1 = 2000;
        TIM17->BDTR |= TIM_BDTR_MOE;
        TIM17->EGR |= 0x01;
        TIM_Cmd(TIM17, ENABLE); 
 

  NVIC_InitTypeDef nvic_tim7;
  nvic_tim7.NVIC_IRQChannel = TIM7_IRQn;
  nvic_tim7.NVIC_IRQChannelCmd = ENABLE;
  nvic_tim7.NVIC_IRQChannelPreemptionPriority = 0x05;
  nvic_tim7.NVIC_IRQChannelSubPriority = 0x05;
  NVIC_Init(&nvic_tim7);

  TIM_DeInit(TIM7);
  TIM7->CR1 |= (0x01 << 7) ;
 // TIM7->DIER |= (0x01 << 0);
  TIM7->PSC = 71; 
  TIM7->ARR = 0xFFFF;
  TIM7->SR = 0x00; 
  TIM_Cmd(TIM7, ENABLE); 
};

void init_timer_ppm(){	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1); //TIM2_CH1_ETR
	
	GPIO_InitTypeDef GPIOB_ppm_init;
	GPIOB_ppm_init.GPIO_Mode = GPIO_Mode_AF;
	GPIOB_ppm_init.GPIO_OType = GPIO_OType_PP;
	GPIOB_ppm_init.GPIO_Pin = GPIO_Pin_11;
	GPIOB_ppm_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOB_ppm_init.GPIO_Speed = GPIO_Speed_Level_3;	
	GPIO_Init(GPIOB, &GPIOB_ppm_init);


	//MISC of the standard library
	NVIC_InitTypeDef nvic_tim2;
	nvic_tim2.NVIC_IRQChannel = TIM2_IRQn;
	nvic_tim2.NVIC_IRQChannelCmd = ENABLE;
	nvic_tim2.NVIC_IRQChannelPreemptionPriority = 0x05;
	nvic_tim2.NVIC_IRQChannelSubPriority = 0x05;
	NVIC_Init(&nvic_tim2);

	TIM_DeInit(TIM2);
	TIM_CounterModeConfig(TIM2, TIM_CounterMode_Up);	
	TIM2->CCMR2 |= (0x01 << 8); //Setting CC4S to 01 to conect the CCR4 is linked to TI1;
	TIM2->CCMR2 |= (0x1 << 12); // setting the filter to 8 consecutive clock cycles with stable signal  
	TIM2->CCER |= (0x01 << 12); // Seeting the Capture enable CC4E
    	TIM2->CCER |= (0x00 << 13 ); // Detect the falling edge CC4P

	//TIM2->DIER |= (0x01 << 1);
	TIM_PrescalerConfig(TIM2, 71, TIM_PSCReloadMode_Update); // Prescaler will be equal to fCK_PSC / (PSC[15:0] + 1
    	TIM_SetAutoreload(TIM2, 0xFFFF);//TIM2->ARR = 0xFFFF;
    	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE); // Eable the Input Compare Interrupt.
	TIM_Cmd(TIM2, ENABLE); //TIM2->CR1 |= 0x01; Enable the TIMER 1 
    
	/**TI2_Config(TIM2, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x00); 
	 * TIMx_CCR1 must be linked to the TI1 input, so write the CC1S bits to 01 in the TIMx_CCMR1 register. 
	 * As soon as CC1S becomes different from 00, the channel is configured in input and the TIMx_CCR1 
	 * register becomes read-only. Set the CCER register to CC1P or CC1E
	 */
    
};

void stm32f3x_init(void){
	SysTick_Config(SystemCoreClock / 1000);
	// Enable FPU
 	SCB->CPACR = (0x3 << (10 * 2)) | (0x3 << (11 * 2));
	//SystemInit();

	/*!< At this stage the microcontroller clock setting is already configured,
	this is done through SystemInit() function which is called from startup
       	file (startup_stm32f30x_xx.s) before to branch to application main.
       	To reconfigure the default setting of SystemInit() function, refer to
       	system_stm32f30x.c file
	*/
};

void checkForBootLoaderRequest(void)
{
    void(*bootJump)(void);

    if (*((uint32_t *)0x20009FFC) == 0xDEADBEEF) {

        *((uint32_t *)0x20009FFC) = 0x0;

        __enable_irq();
        __set_MSP(*((uint32_t *)0x1FFFD800));

        bootJump = (void(*)(void))(*((uint32_t *) 0x1FFFD804));
        bootJump();
        while (1);
    }
};

void systemReset(void)
{
    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

void systemResetToBootloader(void)
{
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC

    *((uint32_t *)0x20009FFC) = 0xDEADBEEF; // 40KB SRAM STM32F30X

    systemReset();
};

void initBoardLed(void){
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitTypeDef gpio_options;
  gpio_options.GPIO_Mode = GPIO_Mode_OUT;
  gpio_options.GPIO_OType = GPIO_OType_PP;
  gpio_options.GPIO_Pin = GPIO_Pin_3;
  gpio_options.GPIO_Speed = GPIO_Speed_Level_3;	
  GPIO_Init(GPIOB, &gpio_options);
};

void init_Buzzer(void){
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitTypeDef gpioc_init_beeper;
  gpioc_init_beeper.GPIO_Mode = GPIO_Mode_OUT;
  gpioc_init_beeper.GPIO_OType = GPIO_OType_PP;
  gpioc_init_beeper.GPIO_Pin = GPIO_Pin_15;
  gpioc_init_beeper.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOC, &gpioc_init_beeper);	
};

