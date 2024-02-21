#include "main.h"

void TIM0_Init(void) // F = 1445Hz; T = 692us
{
    SET_BIT(TCCR0A, (1 << CS00) | (1 << CS01)); // clk/64
    SET_BIT(TCCR0A, 1 << CTC0); // CTC EN
    SET_BIT(TIMSK0, 1 << OCIE0A); // Output Compare Match A Interrupt Enable
    OCR0A = 173 - 1;
}

void TIM1_Init(void) // F = 31.25kHz; T = 32us
{
    SET_BIT(TCCR1A, 1 << WGM11); // Fast PWM, 9-bit
    SET_BIT(TCCR1A, 1 << COM1B1); // Clear OC1A/OC1B on compare match
    SET_BIT(TCCR1B, 1 << CS10); // clk/1
    SET_BIT(TCCR1B, 1 << WGM12); // Fast PWM, 9-bit
}

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
    MODIFY_REG(DDRD, 0xFF, 0xFE);
    CLEAR_REG(PORTD);
}

void CathodeSwitch(unsigned char cathode)
{
    if ((cathode >= 0) && (cathode <= 9))
        MODIFY_REG(PORTD, CATHODE_MASK, cathode << CATHODE_OFFSET);
}

void AnodeSwitch(uint8_t anode)
{
    if ((anode >= 1) && (anode <= 6))
        MODIFY_REG(PORTD, ANODE_MASK, anode << ANODE_OFFSET);
}

void Display_DeadTime(void)
{
    MODIFY_REG(PORTD, CATHODE_MASK | ANODE_MASK, 12u << CATHODE_OFFSET);
}

void TimeToDigit(uint8_t number)
{
    indication.digit1 = number / 10;
    indication.digit2 = number % 10;
}

void TemperatureToDigit(uint16_t temperature)
{
    indication.digit1 = temperature / 10000;
    indication.digit2 = (temperature % 10000) / 1000;
    indication.digit3 = (temperature % 1000) / 100;
    indication.digit4 = (temperature % 100) / 10;
}

void SoftStart(void)
{
    if ((!indication.pause) && (!indication.isTurnedOff) && (voltage.pid.setPoint <= VOUT_TASK))
        voltage.pid.setPoint += 0.01;
}

void SoftTurnoff(void)
{
    if ((indication.isTurnedOff) && (voltage.pid.setPoint > 0.0))
        voltage.pid.setPoint -= 0.005;
}

ISR(TIMER0_COMPA_vect)
{
    if (!indication.pause)
    {
        if (indication.dispMode == DISPLAY_TIME)
        {
            switch (indication.counter)
            {
            case 1:
                TimeToDigit(time.hour);
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(indication.digit1);
                break;
            case 2:
                Display_DeadTime();
                break;
            case 3:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(indication.digit2);
                break;
            case 4:
                Display_DeadTime();
                break;
            case 5:
                TimeToDigit(time.min);
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(indication.digit1);
                break;
            case 6:
                Display_DeadTime();
                break;
            case 7:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(indication.digit2);
                break;
            case 8:
                Display_DeadTime();
                break;
            case 9:
                TimeToDigit(time.sec);
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(indication.digit1);
                break;
            case 10:
                Display_DeadTime();
                break;
            case 11:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(indication.digit2);
                break;
            case 12:
                Display_DeadTime();
                break;
            }
        }

        if (indication.dispMode == DISPLAY_TEMPERATURE)
        {
            switch (indication.counter)
            {
            case 1:
                TemperatureToDigit(temperature.value);
                AnodeSwitch(3u);
                CathodeSwitch(indication.digit1);
                break;
            case 2:
                Display_DeadTime();
                break;
            case 3:
                break;
            case 4:
                AnodeSwitch(4u);
                CathodeSwitch(indication.digit2);
                SET_BIT(PORTA, 1 << 3); // RDP turn on
                break;
            case 5:
                Display_DeadTime();
                break;
            case 6:
                break;
            case 7:
                AnodeSwitch(5u);
                CathodeSwitch(indication.digit3);
                CLEAR_BIT(PORTA, 1 << 3); // RDP turn off
                break;
            case 8:
                Display_DeadTime();
                break;
            case 9:
                break;
            case 10:
                AnodeSwitch(6u);
                CathodeSwitch(indication.digit4);
                break;
            case 11:
                Display_DeadTime();
                break;
            case 12:
                break;
            }
        }

        if (indication.dispMode == DISPLAY_CAD)
        {
            switch (indication.counter)
            {
            case 1:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(cad.digit1);
                break;
            case 2:
                Display_DeadTime();
                break;
            case 3:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(cad.digit2);
                break;
            case 4:
                Display_DeadTime();
                break;
            case 5:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(cad.digit3);
                break;
            case 6:
                Display_DeadTime();
                break;
            case 7:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(cad.digit4);
                break;
            case 8:
                Display_DeadTime();
                break;
            case 9:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(cad.digit5);
                break;
            case 10:
                Display_DeadTime();
                break;
            case 11:
                AnodeSwitch((indication.counter + 1) / 2);
                CathodeSwitch(cad.digit6);
                break;
            case 12:
                Display_DeadTime();
                break;
            }
        }
        if (++indication.counter > 12)
            indication.counter = 1;
        ++cad.counter;
        voltage.pid.run = true;
    }
}

ISR(ADC_vect)
{
    if (voltage.adc.counter < ADC_SAMPLES) // Get ADC_sum by x samples
    {
        voltage.adc.sum += ADC;
        ++voltage.adc.counter;
    }
    else
    {
        voltage.adc.mean = voltage.adc.sum / ADC_SAMPLES;
        voltage.adc.value = (voltage.adc.mean * VREF) / 0x3FF;
        voltage.adc.valueScaled = voltage.adc.value * ((R1_VAL + R2_VAL) / R2_VAL);
        voltage.adc.sum = 0;
        voltage.adc.counter = 0;
    }
    if (voltage.adc.valueScaled > 200.0)
        OCR1B = 0; // Output voltage limitation
}

int main(void)
{
    voltage.pid.pidData.Kp = K_P;
    voltage.pid.pidData.Ki = K_I;
    voltage.pid.pidData.Kd = K_D;
    arm_pid_init_f32(&voltage.pid.pidData, 1u);
    IO_Init();
    TIM0_Init();
    TIM1_Init();
    ADC_Init();
    I2C_Init();
    sei();
    _delay_ms(1000);

    /* Read reference temperature */
    I2C_StartCondition();
    I2C_SendByte(0b10010000); // Device address + write bit
    I2C_SendByte(0x00); // Pointer
    I2C_StartCondition(); // Restart
    I2C_SendByte(0b10010001); // Device address + read bit
    temperature.msb = I2C_ReadByte();
    temperature.lsb = I2C_ReadLastByte();
    I2C_StopCondition();
    temperature.valueRef = 1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125);

#ifdef FIRST_START
    /* CLKOUT turnoff */
    I2C_StartCondition();
    I2C_SendByte(0xA2); // Device address + write bit
    I2C_SendByte(0x0D); // Pointer
    I2C_SendByte(0b00000011); // CLKOUT output is set high-impedance
    I2C_StopCondition();

    /* Write time */
    I2C_StartCondition();
    I2C_SendByte(0xA2); // Device address + write bit
    I2C_SendByte(0x02); // Pointer
    I2C_SendByte(RTC_DECtoBCD(00)); // Sec
    I2C_SendByte(RTC_DECtoBCD(30)); // Min
    I2C_SendByte(RTC_DECtoBCD(15)); // Hour
    I2C_StopCondition();
#endif /* FIRST_START */

    while (1)
    {
        SoftStart();
        // voltage.pid.setPoint = VOUT_TASK; // Manual control

        /* Get time */
        I2C_StartCondition();
        I2C_SendByte(0xA2); // Device address + write bit
        I2C_SendByte(0x02); // Pointer
        I2C_StartCondition(); // Restart
        I2C_SendByte(0xA3); // Device address + read bit
        time.sec = RTC_BCDtoDEC((I2C_ReadByte()) & 0b01111111);
        time.min = RTC_BCDtoDEC((I2C_ReadByte()) & 0b01111111);
        time.hour = RTC_BCDtoDEC((I2C_ReadLastByte()) & 0b00111111);
        I2C_StopCondition();

        // /* Time Correction */
        // if ((time.hour == 19) && (time.min == 59) && (time.sec == 52))
        // {
        //     I2C_StartCondition();
        //     I2C_SendByte(0xA2); // Device address + write bit
        //     I2C_SendByte(0x02); // Pointer
        //     I2C_SendByte(RTC_DECtoBCD(time.sec + 7)); // Sec
        //     I2C_StopCondition();
        // }

        /* TimeRes */
        static uint8_t buttonCounter;
        if (!READ_BIT(PINA, 1 << PINA0))
        {
            if (++buttonCounter == 0xFF)
            {
                buttonCounter = 0;
                I2C_StartCondition();
                I2C_SendByte(0xA2); // Device address + write bit
                I2C_SendByte(0x02); // Pointer
                I2C_SendByte(RTC_DECtoBCD(00)); // Sec
                I2C_SendByte(RTC_DECtoBCD(00)); // Min
                I2C_SendByte(RTC_DECtoBCD(20)); // Hour
                I2C_StopCondition();
            }
        }
        else
            buttonCounter = 0;

        /* Daily turnoff */
        if (time.hour < 6)
        {
            indication.isTurnedOff = true;
            SoftTurnoff();
            if (voltage.pid.setPoint < 100.0)
            {
                indication.pause = true, Display_DeadTime();
                if (indication.pwmOutputStatus == CONNECTED) // PWM turnoff
                {
                    OCR1B = 0;
                    CLEAR_BIT(TCCR1A, 1 << COM1B1); // OC1A/OC1B disconnected
                    indication.pwmOutputStatus = DISCONNECTED;
                }
                if (time.sec % 2) // Turnoff status LED blink
                {
                    SET_BIT(PORTD, 1 << 0);
                    _delay_us(100);
                    CLEAR_BIT(PORTD, 1 << 0);
                    _delay_us(200);
                }
            }
        }
        else
        {
            indication.pause = false, indication.isTurnedOff = false;
            if (indication.pwmOutputStatus == DISCONNECTED) // PWM turn on
            {
                OCR1B = 0;
                SET_BIT(TCCR1A, 1 << COM1B1); // Clear OC1A/OC1B on compare match
                indication.pwmOutputStatus = CONNECTED;
            }
        }

        if (!indication.pause)
        {
            /* Run PID calculations once every PID timer timeout */
            if (voltage.pid.run)
            {
                voltage.pid.processValue =
                    arm_pid_f32(&voltage.pid.pidData, voltage.pid.setPoint - voltage.adc.valueScaled);

                /* PID output limiter */
                if (voltage.pid.processValue < BOOST_DUTY_MIN)
                    voltage.pid.processValue = BOOST_DUTY_MIN;
                else if (voltage.pid.processValue > BOOST_DUTY_MAX)
                    voltage.pid.processValue = BOOST_DUTY_MAX;

                OCR1B = voltage.pid.processValue * 0x1FF;
                voltage.pid.run = false;
            }

            /* Display temperature */
            if ((time.sec > 9) && (time.sec < 15))
            {
                if (indication.dispMode != DISPLAY_TEMPERATURE)
                {
                    indication.dispMode = DISPLAY_TEMPERATURE;

                    /* Read temperature */
                    if (temperature.isCompensated)
                    {
                        I2C_StartCondition();
                        I2C_SendByte(0b10010000); // Device address + write bit
                        I2C_SendByte(0x00); // Pointer
                        I2C_StartCondition(); // Restart
                        I2C_SendByte(0b10010001); // Device address + read bit
                        temperature.msb = I2C_ReadByte();
                        temperature.lsb = I2C_ReadLastByte();
                        I2C_StopCondition();
                        temperature.value =
                            (1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125)) +
                            temperature.compensationFactor;
                    }
                    else
                        temperature.value = temperature.valueRef;
                }
            }
            else if (indication.dispMode == DISPLAY_TEMPERATURE)
                indication.dispMode = DISPLAY_TIME, CLEAR_BIT(PORTA, 1 << 3);

            /* Temperature compensation */
            if (!temperature.isCompensated)
            {
                if ((time.sec % 2) && (temperature.isCompensationAllowed == true))
                    ++temperature.compensationCounter, temperature.isCompensationAllowed = false;
                if (!(time.sec % 2) && (temperature.isCompensationAllowed == false))
                    ++temperature.compensationCounter, temperature.isCompensationAllowed = true;
                if (temperature.compensationCounter == 3600)
                {
                    I2C_StartCondition();
                    I2C_SendByte(0b10010000); // Device address + write bit
                    I2C_SendByte(0x00); // Pointer
                    I2C_StartCondition(); // Restart
                    I2C_SendByte(0b10010001); // Device address + read bit
                    temperature.msb = I2C_ReadByte();
                    temperature.lsb = I2C_ReadLastByte();
                    I2C_StopCondition();
                    temperature.value = 1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125);
                    temperature.compensationFactor = temperature.valueRef - temperature.value;
                    temperature.isCompensated = true; // Temperature value ready after 3600 sec (60 min)
                }
            }

            /* Cathodes anti-degradation (CAD) */
            if ((time.sec > 34) && (time.sec < 40))
            {
                if (indication.dispMode != DISPLAY_CAD)
                    indication.dispMode = DISPLAY_CAD, cad.counter = 0;
                cad.digit1 = (cad.counter / (1445 * 5 / 20)) % 10;
                cad.digit2 = cad.digit1;
                cad.digit3 = cad.digit1;
                cad.digit4 = cad.digit1;
                cad.digit5 = cad.digit1;
                cad.digit6 = cad.digit1;
            }
            else if ((time.sec == 40) && (!cad.update))
            {
                cad.digit1 = 0;
                cad.digit2 = 0;
                cad.digit3 = 0;
                cad.digit4 = 0;
                cad.digit5 = 0;
                cad.digit6 = 0;
                cad.counter = 0;
                cad.updateStage = 1;
                cad.update = true;
            }
            if (cad.update)
            {
                switch (cad.updateStage)
                {
                case 1:
                    cad.digit1 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.hour / 10) == cad.digit1)
                    {
                        ++cad.updateStage;
                        cad.counter = 0;
                    }
                    break;

                case 2:
                    cad.digit2 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.hour % 10) == cad.digit2)
                    {
                        ++cad.updateStage;
                        cad.counter = 0;
                    }
                    break;

                case 3:
                    cad.digit3 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.min / 10) == cad.digit3)
                    {
                        ++cad.updateStage;
                        cad.counter = 0;
                    }
                    break;

                case 4:
                    cad.digit4 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.min % 10) == cad.digit4)
                    {
                        ++cad.updateStage;
                        cad.counter = 0;
                    }
                    break;

                case 5:
                    cad.digit5 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.sec / 10) == cad.digit5)
                    {
                        ++cad.updateStage;
                        cad.counter = 0;
                    }
                    break;

                case 6:
                    cad.digit6 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.sec % 10) == cad.digit6)
                    {
                        cad.update = false;
                        indication.dispMode = DISPLAY_TIME;
                    }
                    break;
                }
            }
        }
    }
}