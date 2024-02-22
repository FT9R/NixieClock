#pragma once

#include <avr/io.h>

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t byte);
void I2C_SendByteByADDR(uint8_t byte, uint8_t addr);
uint8_t I2C_ReadByte(void);
uint8_t I2C_ReadLastByte(void);