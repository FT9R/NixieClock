#pragma once

#include <stdint.h>

uint8_t RTC_BCDtoDEC(uint8_t bcd);
uint8_t RTC_DECtoBCD(uint8_t dec);