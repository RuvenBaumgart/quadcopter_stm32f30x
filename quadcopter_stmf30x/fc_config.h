#include <stdlib.h>
#include <string.h>
#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <stdio.h>
#include "i2c.h"
#include "serialOutStandardText.h"
#include "usart.h"

extern volatile uint8_t usart1TransferComplete;	
extern uint32_t esc1, esc2, esc3, esc4;

void escCalibration(void);
void printIntro(void);
void i2cScanner(void);
void checkReceivedSignalAndPerformAction(void);

void queueFloatToBuf(float num);
extern void queueTextToBuf(char* text);
extern void queueIntToBuf(int32_t num);
extern void sendBufferOverUart1(void);
extern void flashBoardLed(uint16_t flashTime);
extern void delay_millis(uint32_t dlyticks);
extern void getReceiverInputValues(int32_t *receiverInputValues);
extern void delay_millis(uint32_t dlyticks);
extern uint32_t millis();
extern void checkLoopTimingAndWait(uint16_t);
extern void writePWMValuesToESC(); 
void calibrate_esc(void);
