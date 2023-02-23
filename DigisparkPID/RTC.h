#ifndef RTC_H_
#define RTC_H_

#include "main.h"

unsigned char RTC_BCDtoDEC(unsigned char c);
unsigned char RTC_DECtoBCD(unsigned char c);

#endif /* RTC_H_ */