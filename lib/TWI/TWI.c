#include "TWI.h"

void TWI_Init(void)
{
    TWBR = 0x48; // fSCL = 100 kHz
}

void TWI_Start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

void TWI_Stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void TWI_SendByte(uint8_t byte)
{
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

void TWI_SendByteByADDR(uint8_t byte, uint8_t addr)
{
    TWI_Start();
    TWI_SendByte(addr);
    TWI_SendByte(byte);
    TWI_Stop();
}

uint8_t TWI_ReadByte(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))) {}

    return TWDR;
}

uint8_t TWI_ReadLastByte(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}

    return TWDR;
}