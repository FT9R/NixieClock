#pragma once

#include <stdint.h>

#define RTC_SEC_MASK  0x7F
#define RTC_MIN_MASK  0x7F
#define RTC_HOUR_MASK 0x3F

uint8_t RTC_BCDtoDEC(uint8_t bcd);
uint8_t RTC_DECtoBCD(uint8_t dec);