#include "RTC.h"

uint8_t RTC_BCDtoDEC(uint8_t bcd)
{
    return ((bcd >> 4) * 10 + (0x0F & bcd));
}

uint8_t RTC_DECtoBCD(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}