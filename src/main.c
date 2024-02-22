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
    if (++voltage.adc.counter <= ADC_SAMPLES) // Get ADC_sum by x samples
        voltage.adc.sum += ADC;
    else
    {
        voltage.adc.mean = voltage.adc.sum / ADC_SAMPLES;
        voltage.adc.value = voltage.adc.mean * VREF / 1024;
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
    TIMx_Init(0u);
    TIMx_Init(1u);
    ADC_Init();
    I2C_Init();
    sei();
    _delay_ms(1000);

    /* PCF8563 CLKOUT turnoff */
    I2C_Start();
    I2C_SendByte(0xA2); // Device address + write bit
    I2C_SendByte(0x0D); // Pointer
    I2C_SendByte(0x00); // CLKOUT output is set high-impedance
    I2C_Stop();

    /* Read reference temperature */
    I2C_Start();
    I2C_SendByte(0x90); // Device address + write bit
    I2C_SendByte(0x00); // Pointer
    I2C_Start(); // Restart
    I2C_SendByte(0x91); // Device address + read bit
    temperature.msb = I2C_ReadByte();
    temperature.lsb = I2C_ReadLastByte();
    I2C_Stop();
    temperature.valueRef = 1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125);
    temperature.value = temperature.valueRef;

    // /* Write time */
    // I2C_Start();
    // I2C_SendByte(0xA2); // Device address + write bit
    // I2C_SendByte(0x02); // Pointer
    // I2C_SendByte(RTC_DECtoBCD(00)); // Sec
    // I2C_SendByte(RTC_DECtoBCD(50)); // Min
    // I2C_SendByte(RTC_DECtoBCD(10)); // Hour
    // I2C_Stop();

    while (1)
    {
        SoftStart(&indication, &voltage);
        // voltage.pid.setPoint = VOUT_TASK; // Manual control

        /* Get time */
        I2C_Start();
        I2C_SendByte(0xA2); // Device address + write bit
        I2C_SendByte(0x02); // Pointer
        I2C_Start(); // Restart
        I2C_SendByte(0xA3); // Device address + read bit
        time.sec = RTC_BCDtoDEC((I2C_ReadByte()) & 0x7F);
        time.min = RTC_BCDtoDEC((I2C_ReadByte()) & 0x7F);
        time.hour = RTC_BCDtoDEC((I2C_ReadLastByte()) & 0x3F);
        I2C_Stop();

        /* TimeRes */
        static uint8_t buttonCounter;
        if (!READ_BIT(PINA, 1 << PINA0))
        {
            if (++buttonCounter == 0xFF)
            {
                buttonCounter = 0;
                I2C_Start();
                I2C_SendByte(0xA2); // Device address + write bit
                I2C_SendByte(0x02); // Pointer
                I2C_SendByte(RTC_DECtoBCD(00)); // Sec
                I2C_SendByte(RTC_DECtoBCD(00)); // Min
                I2C_SendByte(RTC_DECtoBCD(20)); // Hour
                I2C_Stop();
            }
        }
        else
            buttonCounter = 0;

        // /* Time adjust */
        // if ((!time.adjusted) && (time.hour == 19) && (time.min == 59) && (time.sec == 30))
        // {
        //     I2C_Start();
        //     I2C_SendByte(0xA2); // Device address + write bit
        //     I2C_SendByte(0x02); // Pointer
        //     I2C_SendByte(RTC_DECtoBCD(time.sec + 7)); // Sec
        //     I2C_Stop();
        //     time.adjusted = true;
        // }
        // if ((time.adjusted) && (time.hour == 12))
        //     time.adjusted = false;

        /* Temperature compensation */
        if (!temperature.isCompensated)
        {
            if ((time.sec % 2) && (temperature.isCompensationAllowed == true))
                ++temperature.compensationCounter, temperature.isCompensationAllowed = false;
            if (!(time.sec % 2) && (temperature.isCompensationAllowed == false))
                ++temperature.compensationCounter, temperature.isCompensationAllowed = true;
            if (temperature.compensationCounter == 3600)
            {
                I2C_Start();
                I2C_SendByte(0x90); // Device address + write bit
                I2C_SendByte(0x00); // Pointer
                I2C_Start(); // Restart
                I2C_SendByte(0x91); // Device address + read bit
                temperature.msb = I2C_ReadByte();
                temperature.lsb = I2C_ReadLastByte();
                I2C_Stop();
                temperature.value = 1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125);
                temperature.compensationFactor = temperature.valueRef - temperature.value;
                temperature.isCompensated = true; // Temperature is compensated after 3600 sec (60 min)
            }
        }

        /* Daily turnoff */
        if (time.hour < 6)
        {
            indication.isTurnedOff = true;
            SoftTurnoff(&indication, &voltage);
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
                    SET_BIT(PORTD, 1 << 0);
                else
                    CLEAR_BIT(PORTD, 1 << 0);
            }
        }
        else
        {
            indication.pause = false, indication.isTurnedOff = false;
            if (indication.pwmOutputStatus == DISCONNECTED) // PWM turn on
            {
                OCR1B = 0;
                SET_BIT(TCCR1A, 1 << COM1B1); // Clear OC1A/OC1B on compare match
                CLEAR_BIT(PORTD, 1 << 0);
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

            /* Prepare temperature */
            if ((time.sec > 9) && (time.sec < 15))
            {
                if (indication.dispMode != DISPLAY_TEMPERATURE)
                {
                    /* Read temperature */
                    if (temperature.isCompensated)
                    {
                        I2C_Start();
                        I2C_SendByte(0x90); // Device address + write bit
                        I2C_SendByte(0x00); // Pointer
                        I2C_Start(); // Restart
                        I2C_SendByte(0x91); // Device address + read bit
                        temperature.msb = I2C_ReadByte();
                        temperature.lsb = I2C_ReadLastByte();
                        I2C_Stop();
                        temperature.value =
                            (1000 * (((((temperature.msb << 8) | temperature.lsb) >> 5) & 0x7FF) * 0.125)) +
                            temperature.compensationFactor;
                    }

                    indication.digit3 = temperature.value / 10000;
                    indication.digit4 = (temperature.value / 1000) % 10;
                    indication.digit5 = (temperature.value / 100) % 10;
                    indication.digit6 = (temperature.value / 10) % 10;
                    indication.dispMode = DISPLAY_TEMPERATURE;
                }
            }
            else if (indication.dispMode == DISPLAY_TEMPERATURE)
                indication.dispMode = DISPLAY_TIME, CLEAR_BIT(PORTA, 1 << 3);

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
                    indication.digit1 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.hour / 10) == indication.digit1)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 2:
                    indication.digit2 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.hour % 10) == indication.digit2)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 3:
                    indication.digit3 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.min / 10) == indication.digit3)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 4:
                    indication.digit4 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.min % 10) == indication.digit4)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 5:
                    indication.digit5 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.sec / 10) == indication.digit5)
                        ++cad.updateStage, cad.counter = 0;
                    break;

                case 6:
                    indication.digit6 = (cad.counter / (1445 * 2 / 10)) % 10;
                    if ((time.sec % 10) == indication.digit6)
                        cad.update = false, indication.dispMode = DISPLAY_TIME;
                    break;
                }
            }
        }
    }
}