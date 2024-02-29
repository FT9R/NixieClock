#include "init.h"

void IO_Init(void)
{
    /* PORTA */
    MODIFY_REG(DDRA, 0xFF, (1 << 2) | (1 << 3)); // LDP and RDP
    CLEAR_REG(PORTA);
    CLEAR_BIT(DDRA, 1 << 0); // TimeRes
    SET_BIT(PORTA, 1 << 0); // Pull up

    /* PORTB */
    SET_BIT(DDRB, 1 << 2); // Boost PWM
    CLEAR_REG(PORTB);

    /* PORTD */
    MODIFY_REG(DDRD, 0xFF, 0xFF);
    CLEAR_REG(PORTD);
}

void ADC_Init(void)
{
    MODIFY_REG(ADMUX, 0x0F, 0x00); // ADC0
    CLEAR_BIT(ADMUX, 1 << ADLAR); // Right adjust
    CLEAR_BIT(ADMUX, 1 << REFS0); // Internal 1.1V voltage reference
    MODIFY_REG(ADCSRA, 0x07, (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); // 128x prescaler
    SET_BIT(ADCSRA, 1 << ADIE); // ADC Interrupt Enable
    SET_BIT(ADCSRA, 1 << ADATE); // ADC Auto Trigger Enable
    SET_BIT(ADCSRA, 1 << ADEN); // ADC Enable
    SET_BIT(ADCSRA, 1 << ADSC); // ADC Start Conversion
}

void TIMx_Init(uint8_t timx)
{
    switch (timx)
    {
    case 0: // F = 1445Hz; T = 692us
        SET_BIT(TCCR0A, (1 << CS00) | (1 << CS01)); // clk/64
        SET_BIT(TCCR0A, 1 << CTC0); // CTC EN
        SET_BIT(TIMSK0, 1 << OCIE0A); // Output Compare Match A Interrupt Enable
        OCR0A = 173 - 1;
        break;

    case 1: // F = 31.25kHz; T = 32us
        SET_BIT(TCCR1A, 1 << WGM11); // Fast PWM, 9-bit
        SET_BIT(TCCR1A, 1 << COM1B1); // Clear OC1A/OC1B on compare match
        SET_BIT(TCCR1B, 1 << CS10); // clk/1
        SET_BIT(TCCR1B, 1 << WGM12); // Fast PWM, 9-bit
        break;
    }
}