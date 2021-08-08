
#include "usart.h"
#include <stm32f30x_conf.h> //All important functions are within this header file
#include <stm32f30x.h>

#define DMA_USART
dmaUart1Rx_t *dmaUart1Rx_p = &dmaUart1Rx;	
uint16_t buffposition = 0;
void usart1_init(){
	//Enable the CONFIGURATION of the Clocks we need to use for the different periphals
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable the clock for the usart
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //Enable the clock for the corresponding pins for the usart

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7); //Connect the corresponding pin to the usart
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

		
	//######### GPIO FOR USART ####################
	//######### GPIO Tx and Rx###########################
	GPIO_InitTypeDef GPIO_USART1_Tx_Struct;
	GPIO_USART1_Tx_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_USART1_Tx_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_USART1_Tx_Struct.GPIO_Pin = GPIO_Pin_9; 
	GPIO_USART1_Tx_Struct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_USART1_Tx_Struct.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOA, &GPIO_USART1_Tx_Struct);


	GPIO_InitTypeDef GPIO_USART1_Rx_Struct;
	GPIO_USART1_Rx_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_USART1_Rx_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_USART1_Rx_Struct.GPIO_Pin = GPIO_Pin_10; 
	GPIO_USART1_Rx_Struct.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(GPIOA, &GPIO_USART1_Rx_Struct);

	//################## USART ####################### 	
	USART_InitTypeDef USART1_InitStruct;
	USART_DeInit(USART1); //DeInit the USART 
	
	//Configuratoin for the USART Initalization
	USART1_InitStruct.USART_BaudRate = 115200;
	USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART1_InitStruct.USART_Parity = USART_Parity_No;
	USART1_InitStruct.USART_StopBits = USART_StopBits_1;
	USART1_InitStruct.USART_WordLength = USART_WordLength_8b;
		
	
	//Configuration for the USART_Clock
	USART_ClockInitTypeDef USART1_ClockInitStruct;
	USART1_ClockInitStruct.USART_Clock = USART_Clock_Disable;
	USART1_ClockInitStruct.USART_CPHA = USART_CPHA_1Edge;
	USART1_ClockInitStruct.USART_CPOL = USART_CPOL_High;
	USART1_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;
	
	USART_ClockInit(USART1, &USART1_ClockInitStruct);
	USART_Init(USART1, &USART1_InitStruct);
	
	USART_Cmd(USART1, ENABLE);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	

	//USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	
	/**
	NVIC_InitTypeDef usart1Rx;
	
	usart1Rx.NVIC_IRQChannel = USART1_IRQn;
	usart1Rx.NVIC_IRQChannelCmd = ENABLE;
	usart1Rx.NVIC_IRQChannelPreemptionPriority = 0x05;
	usart1Rx.NVIC_IRQChannelSubPriority = 0x05;

	NVIC_Init(&usart1Rx);
	*/

};

//Init DMA Controller for USART1Tx
void initDmaUsart1Tx(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
	DMA_InitTypeDef dma_init_usart1; // we need a structure for the standard library
	DMA_DeInit(DMA1_Channel4);
	//uint8_t dmastring[] = 0;
	dma_init_usart1.DMA_BufferSize = 0;
	dma_init_usart1.DMA_DIR = DMA_DIR_PeripheralDST; //Reading from the memory to the peripheral
	dma_init_usart1.DMA_M2M = DMA_M2M_Disable; //Not used
	dma_init_usart1.DMA_MemoryBaseAddr = 0; //Base address of the array or what else 
	dma_init_usart1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //Reading or writing on byte 
	dma_init_usart1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init_usart1.DMA_Mode = DMA_Mode_Normal; //DMA_Mode_Circular
	dma_init_usart1.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR; //Address to which the dma controller should write 
	dma_init_usart1.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	dma_init_usart1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init_usart1.DMA_Priority = DMA_Priority_High;

	DMA_Init(DMA1_Channel4, &dma_init_usart1);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE); //Enable the Transfer Complete interrupt.
	

	//MISC of the standard library
	NVIC_InitTypeDef dma_channel1;
	
	dma_channel1.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	dma_channel1.NVIC_IRQChannelCmd = ENABLE;
	dma_channel1.NVIC_IRQChannelPreemptionPriority = 0x05;
	dma_channel1.NVIC_IRQChannelSubPriority = 0x05;

	NVIC_Init(&dma_channel1);
};

void initDmaUsart1Rx(void){
	DMA_InitTypeDef dmaInitUsart1Rx;
	DMA_DeInit(DMA1_Channel5);	
	dmaInitUsart1Rx.DMA_BufferSize = sizeof(uart1RxBuffer); 

	/**
	* If i read the CNDTR (return the remaining number of data units)
	* each 1ms then the buffer needs to be 120 big at a baud rate of 115200.
	*/
	dmaInitUsart1Rx.DMA_DIR = DMA_DIR_PeripheralSRC;
	dmaInitUsart1Rx.DMA_M2M = DMA_M2M_Disable;
	dmaInitUsart1Rx.DMA_MemoryBaseAddr =(uint32_t)&uart1RxBuffer;
	dmaInitUsart1Rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dmaInitUsart1Rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dmaInitUsart1Rx.DMA_Mode = DMA_Mode_Circular;
	dmaInitUsart1Rx.DMA_PeripheralBaseAddr = (uint32_t)&USART1->RDR;
	dmaInitUsart1Rx.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	dmaInitUsart1Rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dmaInitUsart1Rx.DMA_Priority = DMA_Priority_Low;

	DMA_Init(DMA1_Channel5, &dmaInitUsart1Rx);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	//DMA_ITConfig(DMA1_Channel5, DMA_IT_TC | DMA_IT_HT, ENABLE);
	/*
	NVIC_InitTypeDef dma1Channel5;
	dma1Channel5.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	dma1Channel5.NVIC_IRQChannelCmd = ENABLE;
	dma1Channel5.NVIC_IRQChannelPreemptionPriority = 0x05;
	dma1Channel5.NVIC_IRQChannelSubPriority = 0x05;
	NVIC_Init(&dma1Channel5);
	*/
};


#ifndef DMA_USART	
void sendChar(uint8_t c){
//   	if (!(USART_GetFlagStatus(USART1,USART_FLAG_BUSY) == 1)){ //Check if the USART is BUSY or not
		USART_SendData(USART1, c);
		while((USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET )){ //Prüfe im Register ob senden abgeschlossen	
		};
//	};
 };

void sendText(char *t){
    while (*t != 0){
        sendChar(*t);
        t++;
    };
};

void sendInt(int32_t num) {
    char str[10]; // 10 chars max for INT32_MAX
    int i = 0;
    if (num < 0) {
        sendChar('-');
        num *= -1;
    };
    	do 
        	str[i++] = num % 10 + '0'; 
    	while ((num /= 10) > 0);
	while (i) sendChar(str[--i]);
};

void sendFloat(float float_num){
	/*
	uint8_t *ptr, i;
	ptr = (uint8_t*)&float_num;
	for (i = 0; i < sizeof(float);i++) sendChar(*(ptr + i));
	*/

	int32_t tmp_num,tmp_num2;	
	tmp_num = float_num;
	sendInt(tmp_num);
	sendChar('.');
	float_num = float_num - tmp_num;
	float_num = float_num < 0 ? float_num * -1000 : float_num * 1000;
	char str[3];
	int i = 0;
	tmp_num2 = float_num;
	do { 
		str[i++] = tmp_num2 % 10 + '0';
		tmp_num2 /= 10; 
    	} while(i < 3);
	while (i) sendChar(str[--i]);
}
#endif

void int_to_buffer(int32_t num, uint8_t* buffer, uint16_t* pos){
	char str[20];
	itoa(num, str, 10);
	uint8_t i = 0;
	int8_t c = 0;
	while((c = str[i++]) != '\0') *(buffer + (*pos)++) = c;
	*(buffer + (*pos)++) = 0x20; // Space in ASCII
};

void float_to_buffer(float fnum, uint8_t* buffer, uint16_t* pos){
	int32_t tmp, tmp2;
	uint8_t str[3];

	tmp = fnum;
	//int_to_buffer(tmp, buffer, *pos);
	*(buffer + (*pos)++) = 0x2E; // Point in ASCII
	fnum = fnum - tmp;
	fnum = fnum < 0 ? fnum *-1000 : fnum * 1000;
	tmp2 = fnum;
	int i = 0;
	do{
		str[i++] = tmp2 % 10 + '0';
		tmp2 /= 10;
	} while (i < 3);
	while (i) *(buffer + (*pos)++) = str[--i];
 	*(buffer + (*pos)++) = 0x20; // Space in ASCII
};

void text_to_buffer(char* t, uint8_t* buffer, uint16_t* pos){
	while(*t != 0){
		*(buffer + (*pos)++) = *t++;	
	};
};

void init_dmaUart1Rx_t(void){
	dmaUart1Rx_p->dmaChannel = DMA1_Channel5;		
	dmaUart1Rx_p->rxBufferSize = sizeof(uart1RxBuffer);
	dmaUart1Rx_p->rxBufferHead = 0;
	dmaUart1Rx_p->rxBufferTail = 0;	
	dmaUart1Rx_p->rxDmaPos = sizeof(uart1RxBuffer);
};


struct serialPortVTable_s uart1VTable = {
	.serialRxTotalBytesWaiting = uart1RxTotalBytesWaiting,
	.serialRead = uart1RxRead, 
};

uint16_t uart1RxTotalBytesWaiting(void){
	uint16_t rxDMAHead = dmaUart1Rx_p->dmaChannel->CNDTR;
	uint16_t bytesWaiting;

	if(rxDMAHead <= dmaUart1Rx_p->rxDmaPos){
		bytesWaiting = dmaUart1Rx_p->rxDmaPos - rxDMAHead;
	} else {
		bytesWaiting = dmaUart1Rx_p->rxBufferSize + dmaUart1Rx_p->rxDmaPos - rxDMAHead;
	};
 return bytesWaiting;
}; 

uint8_t uart1RxRead(void){
	uint8_t ch;
	ch = uart1RxBuffer[dmaUart1Rx_p->rxBufferSize - dmaUart1Rx_p->rxDmaPos];
	if(--dmaUart1Rx_p->rxDmaPos == 0){
		dmaUart1Rx_p->rxDmaPos = dmaUart1Rx_p->rxBufferSize;
	};
return ch;
};
