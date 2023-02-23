#include "main.h"

//--------------------------------------------
float Vsense = 0.0, Vout = 0.0, Duty = 0.0;
float referenceValue = 0.0, refSend = 0.0, measurementValue = 0.0, inputValue = 0.0;
uint8_t OC1B_status = Connected;
uint8_t sec = 0, min = 0, hour = 0;
uint8_t secCAD = 0, minCAD = 0, hourCAD = 0;
uint8_t dispMode = 1, digit1 = 0, digit2 = 0, digit3 = 0, digit4 = 0;
uint8_t tempsec = 0, tempmin = 0, temphour = 0;
uint8_t MSByte, LSByte, turnOff_stat = 0, count = 1, pause = 0, butcount = 0;
uint16_t Temperature = 0, Temperature_Ref = 0, Temp_Ready = 0, Temp_Comp = 0, Temp_Comp_Cnt = 0, Temp_Comp_Cnt_Perm = 0;
uint16_t generalCount = 0;
uint32_t ADC_sum = 0;
float ADC_mean = 0.0;
//--------------------------------------------

/*! \brief P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
//! \xrefitem todo "Todo" "Todo list"
#define K_P 5.0
//! \xrefitem todo "Todo" "Todo list"
#define K_I 0.05
//! \xrefitem todo "Todo" "Todo list"
#define K_D 0.0

/*! \brief Flags for status information
 */
struct GLOBAL_FLAGS
{
	//! True when PID control loop should run one time
	uint8_t pidTimer : 1;
	uint8_t dummy : 7;
} gFlags = {0, 0};

//! Parameters for regulator
struct PID_DATA pidData;

/*! \brief Sampling Time Interval
 *
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255
 */
//! \xrefitem todo "Todo" "Todo list"
#define TIME_INTERVAL 2 // 624uS * 2 = 1248uS

/*! \brief Initializations
 */
void timer0_Init(void) // f = 1.6 kHz; T = 624 us
{
	TCCR0A |= (1 << CTC0);	 // CTC EN
	TIMSK0 |= (1 << OCIE0A); // match A interrupt EN
	OCR0A = 39;				 // number to compare
	TCCR0A |= (1 << CS02);	 // 256 prescale
}
//--------------------------------------------
void timer1_Init(void)
{
	TCCR1A |= (1 << WGM11);	 // Fast PWM, 9-bit
	TCCR1B |= (1 << WGM12);	 // Fast PWM, 9-bit
	TCCR1A |= (1 << COM1B1); // clear OC1B on compare
	TCCR1B |= (1 << CS10);	 // 1 prescale
}
//--------------------------------------------
void port_Init(void)
{
	/*PORT A*/
	DDRA = (1 << 2) | (1 << 3); // LDP and RDP
	PORTA &= ~(1 << 2) | (1 << 3);
	DDRA &= ~(1 << 0); // TimeRes
	PORTA |= (1 << 0); // Pull up
	/*PORT B*/
	DDRB = (1 << 2); // PWM
	PORTB = 0x00;
	/*PORT D*/
	DDRD = 0xFF;
	PORTD = 0x00;
}
//--------------------------------------------
void segment(unsigned char seg)
{
	switch (seg)
	{
	case 0:
		PORTD &= 0b11100000;
		PORTD |= 0b00000000;
		break;
	case 1:
		PORTD &= 0b11100000;
		PORTD |= 0b00000010;
		break;
	case 2:
		PORTD &= 0b11100000;
		PORTD |= 0b00000100;
		break;
	case 3:
		PORTD &= 0b11100000;
		PORTD |= 0b00000110;
		break;
	case 4:
		PORTD &= 0b11100000;
		PORTD |= 0b00001000;
		break;
	case 5:
		PORTD &= 0b11100000;
		PORTD |= 0b00001010;
		break;
	case 6:
		PORTD &= 0b11100000;
		PORTD |= 0b00001100;
		break;
	case 7:
		PORTD &= 0b11100000;
		PORTD |= 0b00001110;
		break;
	case 8:
		PORTD &= 0b11100000;
		PORTD |= 0b00010000;
		break;
	case 9:
		PORTD &= 0b11100000;
		PORTD |= 0b00010010;
		break;
	case 10:
		PORTD = 0b00011000;
		break; // Extraordinary state
	}
}
//--------------------------------------------
void num(unsigned char num_send)
{
	digit1 = num_send / 10;
	digit2 = num_send % 10;
}
//--------------------------------------------
void numTemp(uint16_t Temp_send)
{
	digit1 = Temp_send / 10000;
	digit2 = (Temp_send % 10000) / 1000;
	digit3 = (Temp_send % 1000) / 100;
	digit4 = (Temp_send % 100) / 10;
}
//--------------------------------------------
void SoftStart(void)
{
	if ((pause == 0) && (turnOff_stat == 0) && (refSend <= Vout_task))
		refSend += 0.02;
}
//--------------------------------------------
void SoftTurnoff(void)
{
	if ((turnOff_stat == 1) && (refSend > 0.0))
		refSend -= 0.005;
}

ISR(TIMER0_COMPA_vect) // Interrupt Service Routine
{
	if (pause == 0)
	{
		if (dispMode == 1)
		{
			switch (count)
			{
			case 1:
				PORTD = 0b00111000;
				num(hour);
				segment(digit1);
				break; // hour1 out
			case 2:
				segment(10);
				break; // DeadTime
			case 3:
				PORTD = 0b01011000;
				segment(digit2);
				break; // hour2 out
			case 4:
				segment(10);
				break; // DeadTime
			case 5:
				PORTD = 0b01111000;
				num(min);
				segment(digit1);
				break; // min1 out
			case 6:
				segment(10);
				break; // DeadTime
			case 7:
				PORTD = 0b10011000;
				segment(digit2);
				break; // min2 out
			case 8:
				segment(10);
				break; // DeadTime
			case 9:
				PORTD = 0b10111000;
				num(sec);
				segment(digit1);
				break; // sec1 out
			case 10:
				segment(10);
				break; // DeadTime
			case 11:
				PORTD = 0b11011000;
				segment(digit2);
				break; // sec2 out
			case 12:
				segment(10);
				break; // DeadTime
			}
			count++;
			if (count > 13)
				count = 1;
		}

		if (dispMode == 2)
		{
			switch (count)
			{
			case 1:
				PORTD = 0b01111000;
				numTemp(Temperature);
				segment(digit1);
				break; // temperature out
			case 2:
				segment(10);
				break;
			case 3:
				segment(10);
				break; // 2xDeadTime
			case 4:
				PORTD = 0b10011000;
				segment(digit2);
				PORTA |= (1 << 3);
				break; // RDP turn on
			case 5:
				segment(10);
				break;
			case 6:
				segment(10);
				break; // 2xDeadTime
			case 7:
				PORTD = 0b10111000;
				segment(digit3);
				PORTA &= ~(1 << 3);
				break; // RDP turn off
			case 8:
				segment(10);
				break;
			case 9:
				segment(10);
				break; // 2xDeadTime
			case 10:
				PORTD = 0b11011000;
				segment(digit4);
				break;
			case 11:
				segment(10);
				break;
			case 12:
				segment(10);
				break; // 2xDeadTime
			}
			count++;
			if (count > 13)
				count = 1;
		}

		if (dispMode == 3)
		{
			switch (count)
			{
			case 1:
				num(hourCAD);
				PORTD = 0b00111000;
				segment(digit2);
				break; // hour CAD
			case 2:
				segment(10);
				break; // DeadTime
			case 3:
				PORTD = 0b01011000;
				segment(digit2);
				break;
			case 4:
				segment(10);
				break; // DeadTime
			case 5:
				num(minCAD);
				PORTD = 0b01111000;
				segment(digit2);
				break; // min CAD
			case 6:
				segment(10);
				break; // DeadTime
			case 7:
				PORTD = 0b10011000;
				segment(digit2);
				break;
			case 8:
				segment(10);
				break; // DeadTime
			case 9:
				num(secCAD);
				PORTD = 0b10111000;
				segment(digit2);
				break; // sec CAD
			case 10:
				segment(10);
				break; // DeadTime
			case 11:
				PORTD = 0b11011000;
				segment(digit2);
				break;
			case 12:
				segment(10);
				break; // DeadTime
			}
			count++;
			if (count > 13)
				count = 1;
		}
		generalCount++;
		//------ Sampling Time Interval ------//
		static unsigned char i = 0;
		if (i < TIME_INTERVAL)
			i++;
		else
		{
			gFlags.pidTimer = TRUE;
			i = 0;
		}
	}
}
//--------------------------------------------
ISR(ADC_vect)
{
	/*filtering (cause the overflow)*/
	static uint8_t ADC_cnt = 0;
	/*V1 source*/
	if (ADC_cnt < ADC_samles) // get ADC_sum by x samples
	{
		ADC_sum += ADC;
		ADC_cnt++;
	}
	else
	{
		ADC_cnt = 0;
		ADC_mean = ADC_sum / ADC_samles;
		ADC_sum = 0;
		Vsense = (ADC_mean * Vref) / 1023;
		Vout = Vsense * ((R1_value + R2_value) / R2_value);
	}
	if (Vout > 200.0)
		OCR1B = 0; // Vout Limitation

	// 	/*Without filtering*/
	// 	Vsense = (ADC * Vref) / 1023;
	// 	Vout = Vsense * ((R1_value + R2_value) / R2_value);
	// 	if (Vout > 200) OCR1B = 0;	// Vout Limitation
}
/******************************************************************************************************
												Main
******************************************************************************************************/
int main(void)
{
	port_Init();
	timer0_Init();
	timer1_Init();
	ADC_Init();
	pid_Init(K_P, K_I, K_D, &pidData);
	I2C_Init();
	sei();
	_delay_ms(1000);
	// Read reference temperature
	I2C_StartCondition();
	I2C_SendByte(0b10010000); // device address + write bit
	I2C_SendByte(0x00);		  // pointer
	I2C_StartCondition();	  // restart
	I2C_SendByte(0b10010001); // device address + read bit
	MSByte = I2C_ReadByte();
	LSByte = I2C_ReadLastByte();
	I2C_StopCondition();
	Temperature_Ref = 1000 * (((((MSByte << 8) | LSByte) >> 5) & 0x7FF) * 0.125);
#ifdef First_Start
	// CLKOUT turnoff
	I2C_StartCondition();
	I2C_SendByte(0xA2);		  // device address + write bit
	I2C_SendByte(0x0D);		  // pointer
	I2C_SendByte(0b00000011); // CLKOUT output is set high-impedance
	I2C_StopCondition();

	// write time
	I2C_StartCondition();
	I2C_SendByte(0xA2);				// device address + write bit
	I2C_SendByte(0x02);				// pointer
	I2C_SendByte(RTC_DECtoBCD(00)); // sec
	I2C_SendByte(RTC_DECtoBCD(30)); // min
	I2C_SendByte(RTC_DECtoBCD(15)); // hour
	I2C_StopCondition();
#endif

	while (1)
	{
		SoftStart();
		// refSend = Vout_task;	// manual control

		// read time
		I2C_StartCondition();
		I2C_SendByte(0xA2);	  // device address + write bit
		I2C_SendByte(0x02);	  // pointer
		I2C_StartCondition(); // restart
		I2C_SendByte(0xA3);	  // device address + read bit
		tempsec = ((I2C_ReadByte()) & 0b01111111);
		tempmin = ((I2C_ReadByte()) & 0b01111111);
		temphour = ((I2C_ReadLastByte()) & 0b00111111);
		I2C_StopCondition();
		sec = RTC_BCDtoDEC(tempsec);
		min = RTC_BCDtoDEC(tempmin);
		hour = RTC_BCDtoDEC(temphour);

		/*//TimeCorrection
		if((hour == 19) && (min == 59) && (sec == 52))
		{
		I2C_StartCondition();
		I2C_SendByte(0xA2);							//device address + write bit
		I2C_SendByte(0x02);							//pointer
		I2C_SendByte(RTC_DECtoBCD(sec + 7));		//sec
		I2C_StopCondition();
		}*/

		// TimeRes
		if (!(PINA & (1 << PINA0)))
		{
			if (butcount < 0xFF)
				butcount++;
			else
			{
				I2C_StartCondition();
				I2C_SendByte(0xA2);				// device address + write bit
				I2C_SendByte(0x02);				// pointer
				I2C_SendByte(RTC_DECtoBCD(00)); // sec
				I2C_SendByte(RTC_DECtoBCD(00)); // min
				I2C_SendByte(RTC_DECtoBCD(20)); // hour
				I2C_StopCondition();
				butcount = 0;
			}
		}

		// Daily turnoff
		if (hour < 6)
		{
			turnOff_stat = 1;
			SoftTurnoff();
			if (refSend < 100.0)
			{
				pause = 1, segment(10);
				if (OC1B_status == Connected) // PWM turnoff
				{
					OCR1B = 0;
					TCCR1A &= ~(1 << COM1B1);
					OC1B_status = Disconnected;
				}
				if (sec % 2) // turnoff status LED blink
				{
					PORTD |= (1 << 0), _delay_us(100), PORTD &= ~(1 << 0), _delay_us(200);
				}
			}
		}
		else
		{
			pause = 0, turnOff_stat = 0;
			if (OC1B_status == Disconnected) // PWM turn on
			{
				OCR1B = 0;
				TCCR1A |= (1 << COM1B1); // clear OC1B on compare
				OC1B_status = Connected;
			}
		}

		/***************************************************************************************************/
		if (pause == 0)
		{
			// Run PID calculations once every PID timer timeout
			if (gFlags.pidTimer)
			{
				measurementValue = Vout;
				inputValue = pid_Controller(refSend, measurementValue, &pidData);
				Duty = inputValue * 0.368; // define max duty 70%:(70)/MAX_INT
				OCR1B = (Duty / 100) * 512;
				gFlags.pidTimer = FALSE;
			}
			//--------------- Temperature sensor processing ---------------//
			// read temperature
			if (sec == 9)
			{
				if (Temp_Ready)
				{
					I2C_StartCondition();
					I2C_SendByte(0b10010000); // device address + write bit
					I2C_SendByte(0x00);		  // pointer
					I2C_StartCondition();	  // restart
					I2C_SendByte(0b10010001); // device address + read bit
					MSByte = I2C_ReadByte();
					LSByte = I2C_ReadLastByte();
					I2C_StopCondition();
					Temperature = 1000 * ((((((MSByte << 8) | LSByte) >> 5) & 0x7FF) * 0.125) - Temp_Comp);
				}
				else
					Temperature = Temperature_Ref;
			}
			// display temperature
			if ((sec > 9) && (sec < 15))
				dispMode = 2;
			if (sec == 15)
				dispMode = 1, PORTA &= ~(1 << 3);
			// temperature compensation counter (freq = 1Hz)
			if (!Temp_Ready)
			{
				if ((sec % 2) && (Temp_Comp_Cnt_Perm == 1))
					Temp_Comp_Cnt++, Temp_Comp_Cnt_Perm = 0;
				if (!(sec % 2) && (Temp_Comp_Cnt_Perm == 0))
					Temp_Comp_Cnt++, Temp_Comp_Cnt_Perm = 1;
				if (Temp_Comp_Cnt == 3600)
				{
					I2C_StartCondition();
					I2C_SendByte(0b10010000); // device address + write bit
					I2C_SendByte(0x00);		  // pointer
					I2C_StartCondition();	  // restart
					I2C_SendByte(0b10010001); // device address + read bit
					MSByte = I2C_ReadByte();
					LSByte = I2C_ReadLastByte();
					I2C_StopCondition();
					Temperature = 1000 * (((((MSByte << 8) | LSByte) >> 5) & 0x7FF) * 0.125);
					Temp_Comp = (Temperature - Temperature_Ref) / 1000;
					Temp_Ready = 1; // Temp_Ready after 3600 sec (60 min)
				}
			}

			// cathodes anti-degradation (CAD)
			if (sec == 29)
				generalCount = 0; // Slow switch
			if ((sec > 29) && (sec < 35))
			{
				dispMode = 3;
				secCAD = generalCount / 392;
				minCAD = generalCount / 392;
				hourCAD = generalCount / 392;
			}
			else if (sec == 35)
				dispMode = 1;
			//--------------------------------------------
			if (sec == 54)
				generalCount = 0; // Fast switch
			if ((sec > 54) && (sec < 60))
			{
				dispMode = 3;
				secCAD = generalCount / 196;
				minCAD = generalCount / 98;
				hourCAD = generalCount / 49;
			}
			else if (sec == 0)
				dispMode = 1;
		}
	}
}

/*! \mainpage
 * \section Intro Introduction
 * This documents data structures, functions, variables, defines, enums, and
 * typedefs in the software for application note AVR221.
 *
 * \section CI Compilation Info
 * This software was written for the IAR Embedded Workbench 4.11A.
 *
 * To make project:
 * <ol>
 * <li> Add the file main.c and pid.c to project.
 * <li> Under processor configuration, select desired Atmel AVR device.
 * <li> Enable bit definitions in I/O include files
 * <li> High optimization on speed is recommended for best performance
 * </ol>
 *
 * \section DI Device Info
 * The included source code is written for all Atmel AVR devices.
 *
 * \section TDL ToDo List
 * \todo Put in own code in:
 * \ref Get_Reference(void), \ref Get_Measurement(void) and \ref Set_Input(int16_t inputValue)
 *
 * \todo Modify the \ref K_P (P), \ref K_I (I) and \ref K_D (D) gain to adapt to your application
 * \todo Specify the sampling interval time \ref TIME_INTERVAL
 */