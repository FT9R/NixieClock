#include "main.h"

ISR(TIMER0_COMPA_vect)
{
    if (indication.pause)
        return;

    switch (indication.dispMode)
    {
    case DISPLAY_TIME:
    case DISPLAY_CAD:
        switch (indication.counter)
        {
        case 1:
            AnodeSwitch((indication.counter + 1) / 2);
            CathodeSwitch(indication.digit1);
            break;
        case 3:
            AnodeSwitch((indication.counter + 1) / 2);
            CathodeSwitch(indication.digit2);
            break;
        case 5:
            AnodeSwitch((indication.counter + 1) / 2);
            CathodeSwitch(indication.digit3);
            break;
        case 7:
            AnodeSwitch((indication.counter + 1) / 2);
            CathodeSwitch(indication.digit4);
            break;
        case 9:
            AnodeSwitch((indication.counter + 1) / 2);
            CathodeSwitch(indication.digit5);
            break;
        case 11:
            AnodeSwitch((indication.counter + 1) / 2);
            CathodeSwitch(indication.digit6);
            break;
        case 2:
        case 4:
        case 6:
        case 8:
        case 10:
        case 12:
            Display_DeadTime();
            break;
        }
        break;

    case DISPLAY_TEMPERATURE:
        switch (indication.counter)
        {
        case 1:
            AnodeSwitch(3u);
            CathodeSwitch(indication.digit3);
            break;
        case 4:
            AnodeSwitch(4u);
            CathodeSwitch(indication.digit4);
            SET_BIT(PORTA, 1 << 3); // RDP turn on
            break;
        case 7:
            AnodeSwitch(5u);
            CathodeSwitch(indication.digit5);
            CLEAR_BIT(PORTA, 1 << 3); // RDP turn off
            break;
        case 10:
            AnodeSwitch(6u);
            CathodeSwitch(indication.digit6);
            break;
        case 2:
        case 3:
        case 5:
        case 6:
        case 8:
        case 9:
        case 11:
        case 12:
            Display_DeadTime();
            break;
        }
        break;
    }

    if (++indication.counter > 12)
        indication.counter = 1;
    ++cad.counter;
    voltage.pid.run = true;
}

ISR(ADC_vect)
{
    if (++voltage.adc.counter <= ADC_SAMPLES)
        voltage.adc.sum += ADC;
    else
    {
        voltage.adc.mean = voltage.adc.sum / ADC_SAMPLES;
        voltage.adc.value = voltage.adc.mean * VREF / 1024;
        voltage.adc.valueScaled = voltage.adc.value * ((R1_VAL + R2_VAL) / R2_VAL);
        voltage.adc.valueScaled *= VOUT_REGL_SLOPE;
        voltage.adc.valueScaled += VOUT_REGL_INTERCEPT;
        voltage.adc.sum = 0;
        voltage.adc.counter = 0;
    }
}

int main(void)
{
    voltage.pid.pidData.Kp = K_P;
    voltage.pid.pidData.Ki = K_I;
    voltage.pid.pidData.Kd = K_D;
    arm_pid_init_f32(&voltage.pid.pidData, 1u);
    IO_Init();
    TIMx_Init(0u);
    TIMx_Init(1u);
    ADC_Init();
    TWI_Init();
    USER_LED_ON;
    _delay_ms(2000);
    USER_LED_OFF;
    sei();

    /* PCF8563 CLKOUT turnoff */
    TWI_Start();
    TWI_SendByte(0xA2); // Device address + write bit
    TWI_SendByte(0x0D); // Pointer
    TWI_SendByte(0x00); // CLKOUT output is set high-impedance
    TWI_Stop();

    /* Read reference temperature */
    TWI_Start();
    TWI_SendByte(0x90); // Device address + write bit
    TWI_SendByte(0x00); // Pointer
    TWI_Start(); // Restart
    TWI_SendByte(0x91); // Device address + read bit
    temperature.msb = TWI_ReadByte();
    temperature.lsb = TWI_ReadLastByte();
    TWI_Stop();
    temperature.compensation.reference = 1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125);
    temperature.value = temperature.compensation.reference;

    // /* Set time */
    // TWI_Start();
    // TWI_SendByte(0xA2); // Device address + write bit
    // TWI_SendByte(0x02); // Pointer
    // TWI_SendByte(RTC_DECtoBCD(00)); // Sec
    // TWI_SendByte(RTC_DECtoBCD(01)); // Min
    // TWI_SendByte(RTC_DECtoBCD(15)); // Hour
    // TWI_Stop();

    while (1)
    {
        SoftStart(&indication, &voltage);
        // voltage.pid.setPoint = VOUT_TASK; // Manual control

        /* Get time */
        TWI_Start();
        TWI_SendByte(0xA2); // Device address + write bit
        TWI_SendByte(0x02); // Pointer
        TWI_Start(); // Restart
        TWI_SendByte(0xA3); // Device address + read bit
        time.sec = RTC_BCDtoDEC((TWI_ReadByte()) & RTC_SEC_MASK);
        time.min = RTC_BCDtoDEC((TWI_ReadByte()) & RTC_MIN_MASK);
        time.hour = RTC_BCDtoDEC((TWI_ReadLastByte()) & RTC_HOUR_MASK);
        TWI_Stop();

        /* TimeRes */
        static uint8_t buttonCounter;
        if (!READ_BIT(PINA, 1 << PINA0))
        {
            if (++buttonCounter == UINT8_MAX)
            {
                buttonCounter = 0;
                TWI_Start();
                TWI_SendByte(0xA2); // Device address + write bit
                TWI_SendByte(0x02); // Pointer
                TWI_SendByte(RTC_DECtoBCD(00)); // Sec
                TWI_SendByte(RTC_DECtoBCD(00)); // Min
                TWI_SendByte(RTC_DECtoBCD(20)); // Hour
                TWI_Stop();
            }
        }
        else
            buttonCounter = 0;

        // /* Time adjust */
        // if ((!time.adjusted) && (time.hour == 19) && (time.min == 59) && (time.sec == 30))
        // {
        //     TWI_Start();
        //     TWI_SendByte(0xA2); // Device address + write bit
        //     TWI_SendByte(0x02); // Pointer
        //     TWI_SendByte(RTC_DECtoBCD(time.sec + 7)); // Sec
        //     TWI_Stop();
        //     time.adjusted = true;
        // }
        // if ((time.adjusted) && (time.hour == 12))
        //     time.adjusted = false;

        /* Daily turnoff */
        if (time.hour < 6)
        {
            indication.isTurnedOff = true;
            SoftTurnoff(&indication, &voltage);
            if (voltage.pid.setPoint < 100.0)
            {
                if (indication.pwmOutputStatus == CONNECTED) // PWM turnoff
                {
                    OCR1B = 0;
                    CLEAR_BIT(TCCR1A, 1 << COM1B1); // OC1A/OC1B disconnected
                    indication.pwmOutputStatus = DISCONNECTED;
                    indication.pause = true;
                    temperature.compensation.counter = 0;
                    Display_DeadTime();
                }
                (time.sec % 2) ? USER_LED_ON : USER_LED_OFF;
            }
        }
        else
        {
            indication.pause = false, indication.isTurnedOff = false;
            if (indication.pwmOutputStatus == DISCONNECTED) // PWM turn on
            {
                OCR1B = 0;
                SET_BIT(TCCR1A, 1 << COM1B1); // Clear OC1A/OC1B on compare match
                USER_LED_OFF;
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

            /* Prepare time */
            if (indication.dispMode == DISPLAY_TIME)
            {
                indication.digit1 = time.hour / 10;
                indication.digit2 = time.hour % 10;
                indication.digit3 = time.min / 10;
                indication.digit4 = time.min % 10;
                indication.digit5 = time.sec / 10;
                indication.digit6 = time.sec % 10;
            }

            /* Temperature compensation */
            if (!temperature.compensation.ready)
            {
                if ((time.sec % 2) && (temperature.compensation.allowIncrement == true))
                    ++temperature.compensation.counter, temperature.compensation.allowIncrement = false;
                if (!(time.sec % 2) && (temperature.compensation.allowIncrement == false))
                    ++temperature.compensation.counter, temperature.compensation.allowIncrement = true;
                if (temperature.compensation.counter == 3600)
                {
                    TWI_Start();
                    TWI_SendByte(0x90); // Device address + write bit
                    TWI_SendByte(0x00); // Pointer
                    TWI_Start(); // Restart
                    TWI_SendByte(0x91); // Device address + read bit
                    temperature.msb = TWI_ReadByte();
                    temperature.lsb = TWI_ReadLastByte();
                    TWI_Stop();
                    temperature.value = 1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125);
                    temperature.compensation.factor = temperature.compensation.reference - temperature.value;
                    temperature.compensation.ready = true; // Temperature is compensated after 3600 sec (60 min)
                }
            }

            /* Prepare temperature */
            if ((time.sec > 4) && (time.sec < 10))
            {
                if (indication.dispMode != DISPLAY_TEMPERATURE)
                {
                    /* Read temperature */
                    if (temperature.compensation.ready)
                    {
                        TWI_Start();
                        TWI_SendByte(0x90); // Device address + write bit
                        TWI_SendByte(0x00); // Pointer
                        TWI_Start(); // Restart
                        TWI_SendByte(0x91); // Device address + read bit
                        temperature.msb = TWI_ReadByte();
                        temperature.lsb = TWI_ReadLastByte();
                        TWI_Stop();
                        temperature.value =
                            (1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125)) +
                            temperature.compensation.factor;
                    }

                    indication.digit3 = temperature.value / 10000;
                    indication.digit4 = (temperature.value / 1000) % 10;
                    indication.digit5 = (temperature.value / 100) % 10;
                    indication.digit6 = (temperature.value / 10) % 10;
                    indication.dispMode = DISPLAY_TEMPERATURE;
                }
            }
            else if (indication.dispMode == DISPLAY_TEMPERATURE)
                CLEAR_BIT(PORTA, 1 << 3), indication.dispMode = DISPLAY_CAD;

            /* Cathodes anti-degradation (CAD) */
            if ((time.sec > 34) && (time.sec < 40))
            {
                if (indication.dispMode != DISPLAY_CAD)
                    indication.dispMode = DISPLAY_CAD, cad.counter = 0;
                indication.digit1 = (cad.counter / (1445 * 5 / 20)) % 10;
                indication.digit2 = indication.digit1;
                indication.digit3 = indication.digit1;
                indication.digit4 = indication.digit1;
                indication.digit5 = indication.digit1;
                indication.digit6 = indication.digit1;
            }
            else if ((indication.dispMode == DISPLAY_CAD) && (!cad.update))
            {
                indication.digit1 = 0;
                indication.digit2 = 0;
                indication.digit3 = 0;
                indication.digit4 = 0;
                indication.digit5 = 0;
                indication.digit6 = 0;
                cad.counter = 0;
                cad.updateStage = 1;
                cad.update = true;
            }
            if ((indication.dispMode == DISPLAY_CAD) && (cad.update))
            {
                switch (cad.updateStage)
                {
                case 1:
                    indication.digit1 = (cad.counter / (1445 * 1 / 10)) % 10;
                    if ((time.hour / 10) == indication.digit1)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 2:
                    indication.digit2 = (cad.counter / (1445 * 1 / 10)) % 10;
                    if ((time.hour % 10) == indication.digit2)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 3:
                    indication.digit3 = (cad.counter / (1445 * 1 / 10)) % 10;
                    if ((time.min / 10) == indication.digit3)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 4:
                    indication.digit4 = (cad.counter / (1445 * 1 / 10)) % 10;
                    if ((time.min % 10) == indication.digit4)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 5:
                    indication.digit5 = (cad.counter / (1445 * 1 / 10)) % 10;
                    if ((time.sec / 10) == indication.digit5)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 6:
                    indication.digit6 = (cad.counter / (1445 * 1 / 10)) % 10;
                    if ((time.sec % 10) == indication.digit6)
                        cad.update = false, indication.dispMode = DISPLAY_TIME;
                    break;
                }
            }
        }
    }
}