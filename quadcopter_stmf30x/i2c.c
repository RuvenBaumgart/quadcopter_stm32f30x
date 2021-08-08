#include "i2c.h"

void init_i2c1(void){

	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	//Setting the alternate function to the corresponding i2c pins
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);

	//Setting up the GPIO init structure
	GPIO_InitTypeDef GPIO_I2C1_Init;
	
	GPIO_I2C1_Init.GPIO_Mode = GPIO_Mode_AF;
	GPIO_I2C1_Init.GPIO_OType = GPIO_OType_OD;
	GPIO_I2C1_Init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_I2C1_Init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_I2C1_Init.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_I2C1_Init);	

	//Setting up the i2c bus
	I2C_DeInit(I2C1);	

	I2C_InitTypeDef I2C1_Init;
	I2C1_Init.I2C_Ack = I2C_Ack_Enable;
	I2C1_Init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C1_Init.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C1_Init.I2C_DigitalFilter = 0x00;
	I2C1_Init.I2C_Mode = I2C_Mode_I2C;
	I2C1_Init.I2C_OwnAddress1 = 0x00;
	//I2C1_Init.I2C_Timing = 0xC062121F; //1100 0000 0110 0010 0001 0010 0001 1111
	I2C1_Init.I2C_Timing = 0x00E0257A; // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.

	I2C_Init(I2C1, &I2C1_Init);
	I2C_Cmd(I2C1, ENABLE);
        //I2C_DMACmd(I2C1, I2C_DMAReq_Tx, ENABLE)
        // I2C_DMACmd(I2C1, I2C_DMAReq_Rx, ENABLE);
        I2C_ITConfig(I2C1, I2C_IT_TCI | I2C_IT_RXI, ENABLE);
};


void initI2C1DmaTx(){
//DMA1 Channel6 
};

void initI2C1DmaRx(){
  DMA_InitTypeDef i2cdmatx;
  DMA_DeInit(DMA1_Channel7);   
  i2cdmatx.DMA_DIR = DMA_DIR_PeripheralSRC;
  i2cdmatx.DMA_M2M = DMA_M2M_Disable;
  i2cdmatx.DMA_MemoryBaseAddr = (uint32_t)&gyroRawData;
  i2cdmatx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  i2cdmatx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  i2cdmatx.DMA_Mode = DMA_Mode_Normal;
  i2cdmatx.DMA_BufferSize = 14;
  i2cdmatx.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->RXDR;
  i2cdmatx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  i2cdmatx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  i2cdmatx.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel7, &i2cdmatx);
  DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE); //Enable the Transfer Complete interrupt.
  DMA_Cmd(DMA1_Channel7, ENABLE);

  NVIC_InitTypeDef dma1Channel7;
  dma1Channel7.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  dma1Channel7.NVIC_IRQChannelCmd = ENABLE;
  dma1Channel7.NVIC_IRQChannelPreemptionPriority = 0x05;
  dma1Channel7.NVIC_IRQChannelSubPriority = 0x05;
  NVIC_Init(&dma1Channel7);	 
};

void I2C_DmaReadGyroValues(uint8_t DevAddress, uint8_t RegAddress){
  DMA_SetCurrDataCounter(DMA1_Channel7, 14);
  DMA_Cmd(DMA1_Channel7, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET);
  I2C_TransferHandling(I2C1, DevAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  //Wait for the acknowledgement
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
  //Send Registeraddress we want to start reading from
  I2C_SendData(I2C1, RegAddress);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET);
  //Ask for the 14 Bytes of Data 
  I2C_TransferHandling(I2C1, DevAddress, 14, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  
};

void I2C_writeByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t regAddress, uint8_t data){
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET); // Check if the i2c bus is bussy
	I2C_TransferHandling(I2Cx, address, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET);
	//After sending the start condition to the slave address we have waited for the acknowlegedment of the start condition and are then sending the register address we want to sent our data to
	I2C_SendData(I2Cx, regAddress);
	//Wait until the TCR flag is set
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET);
	//We have now send the start condition to the slave, set the direction == Write, and send the regaddress we want to write to. Now we need to write the data
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(I2Cx, address, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	/* Wait until the TXIS flag is set */
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET);
	I2C_SendData(I2Cx, data);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET);	
	I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);
};

uint8_t I2C_readByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t regAddress){
	uint8_t data;
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET);
	I2C_TransferHandling(I2Cx, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET);
	//Send Registeraddress we want to write to
	I2C_SendData(I2Cx, regAddress);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET);
	I2C_TransferHandling(I2Cx, address, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET);
	data = I2C_ReceiveData(I2Cx);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET);
	I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);
	return data;
};

uint8_t I2C_getStatus (I2C_TypeDef* I2Cx, uint8_t address){
	I2C_TransferHandling(I2Cx, address, 0, I2C_AutoEnd_Mode, I2C_No_StartStop);
	I2C_ClearFlag(I2Cx, (I2C_ICR_NACKCF | I2C_ICR_STOPCF));
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET);
	if(I2C_GetFlagStatus(I2Cx, I2C_ISR_NACKF) != RESET){
		I2C_ClearFlag(I2Cx, (I2C_ICR_NACKCF | I2C_ICR_STOPCF));
		return 0;
	} else {
		I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);
		return 1;
	};

};


void I2C_readBytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t regAddress, uint8_t nBytes, uint8_t* destination){
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET);
	I2C_TransferHandling(I2Cx, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	//Wait for the Acknowledgement
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET);
	//Send Registeraddress we want to start reading from
	I2C_SendData(I2Cx, regAddress);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET);
	I2C_TransferHandling(I2Cx, address, nBytes, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET);
	while (nBytes) {
        	while (I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET);
        	*destination++ = I2C_ReceiveData(I2Cx);
        	nBytes--;
    	};
	while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET);
	I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);
};
