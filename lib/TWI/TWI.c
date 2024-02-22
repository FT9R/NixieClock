#include "TWI.h"

void I2C_Init(void)
{
    TWBR = 0x48; // fSCL = 100 kHz
}

void I2C_Start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

void I2C_Stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void I2C_SendByte(uint8_t byte)
{
    TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

void I2C_SendByteByADDR(uint8_t byte, uint8_t addr)
{
    I2C_Start();
    I2C_SendByte(addr);
    I2C_SendByte(byte);
    I2C_Stop();
}

uint8_t I2C_ReadByte(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))) {}

    return TWDR;
}

uint8_t I2C_ReadLastByte(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}

    return TWDR;
}