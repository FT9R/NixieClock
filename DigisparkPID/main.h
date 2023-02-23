#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 16000000UL
#define Vref 1.1
#define ADC_samles 5
#define Vout_task 180
#define R1_value 210000
#define R2_value 1000
#define Connected 1
#define Disconnected 0
// #define	First_Start

#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"
#include "RTC.h"
#include "twi.h"
#include "pid.h"

#endif /* MAIN_H_ */