#include <stdio.h>
#include <stdlib.h>
#include <stm32f30x_conf.h>
#include <stm32f30x.h>


uint8_t gyroRawData[14];
void initI2C1DmaTx(void);
void initI2C1DmaRx(void);
void init_i2c1(void);
void I2C_writeByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t regAddress, uint8_t data);
uint8_t I2C_readByte(I2C_TypeDef* I2Cx, uint8_t address, uint8_t regAddress);
uint8_t I2C_getStatus(I2C_TypeDef* I2Cx, uint8_t address);
void I2C_readBytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t regAddress, uint8_t nBytes, uint8_t* destination);
void I2C_DmaReadGyroValues(uint8_t DevAddress, uint8_t RegAddress);

