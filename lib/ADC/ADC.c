#include "ADC.h"

void ADC_Init(void)
{
    ADCSRA |= ((1 << ADEN) // ADC Enable
               | (1 << ADATE) // ADC Auto Trigger Enable
               | (1 << ADIE) // ADC Interrupt Enable
               | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); // 128x prescaler

    ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3)); // ADC0 input
    ADMUX &= ~(1 << REFS0); // Internal 1.1V voltage reference

    ADCSRA |= (1 << ADSC); // ADC Start Conversion
}