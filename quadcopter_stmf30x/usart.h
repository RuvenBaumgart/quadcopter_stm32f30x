#include <stdlib.h>
#include <stdio.h>
#include <stm32f30x.h>

static uint8_t uart1RxBuffer[128] = {0x00};


typedef struct dmaUart1Rx_s{
	uint16_t rxBufferHead;
	uint16_t rxBufferTail;
	uint16_t rxDmaPos;
	uint16_t rxBufferSize;
	DMA_Channel_TypeDef *dmaChannel;
} dmaUart1Rx_t;
static dmaUart1Rx_t dmaUart1Rx;

uint16_t uart1RxTotalBytesWaiting(void);
uint8_t uart1RxRead(void);

/**
* Using a vtable as a interface to the static functions in usart.c
*/

struct serialPortVTable_s{
	uint16_t (*serialRxTotalBytesWaiting)(void);
	uint8_t (*serialRead)(void);
};
struct serialPortVTable_s uart1VTable;

void usart1_init(void);
void sendChar(uint8_t c);
void sendText(char *t);
void sendInt(int32_t num);
void sendFloat(float float_num);


void initDmaUsart1Tx(void); //Tx on Channel 4
void initDmaUsart1Rx(void); //Rx on Channel 5

void float_to_buffer(float fnum, uint8_t* buffer, uint16_t* pos);
void int_to_buffer(int32_t num, uint8_t* buffer, uint16_t* pos);
void text_to_buffer(char* t, uint8_t* buffer, uint16_t* pos);
void init_dmaUart1Rx_t(void);


