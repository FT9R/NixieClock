#pragma once

#include <avr/io.h>

void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_SendByte(uint8_t byte);
void TWI_SendByteByADDR(uint8_t byte, uint8_t addr);
uint8_t TWI_ReadByte(void);
uint8_t TWI_ReadLastByte(void);