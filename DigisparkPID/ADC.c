#include "ADC.h"

void ADC_Init(void)
{
	ADCSRA |= (1 << ADEN) // ADC EN

			  | (1 << ADATE)								// AUTO TRIG EN
			  | (1 << ADIE)									// ADC CONVERSION INTERRUPT EN
			  | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // 128 PRESCALE

	ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3)); // ADC0 INPUT
	ADMUX &= ~(1 << REFS0);											   // 1.1V REFs

	ADCSRA |= (1 << ADSC); // ADC START
}