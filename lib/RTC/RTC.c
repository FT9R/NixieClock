#include "RTC.h"

uint8_t RTC_BCDtoDEC(uint8_t c)
{
    uint8_t ch = ((c >> 4) * 10 + (0b00001111 & c));

    return ch;
}

uint8_t RTC_DECtoBCD(uint8_t c)
{
    uint8_t ch = ((c / 10) << 4) | (c % 10);

    return ch;
}