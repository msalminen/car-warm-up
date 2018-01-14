/*
 * CarWarmUpControl.c
 *
 * Created: 03.07.2013 14:35:24

    Software for controlling car pre-warmer
    Copyright (C) 2018  Marko Salminen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#include "lcd.h"
#include "CarWarmUpControl.h"

// IO initializations
void io_init()
{
	DDRA |= (1 << CAR1_RELAY);			// Set output direction on CAR1_RELAY
	DDRA |= (1 << CAR1_BUTTON_LED);		// Set output direction on CAR1_BUTTON_LED
	DDRB &= ~(1 << CAR1_BUTTON);		// Set input direction on CAR1_BUTTON
	DDRB &= ~(1 << SELECT_BUTTON);		// Set input direction on SELECT_BUTTON
	DDRB &= ~(1 << SETUP_BUTTON);		// Set input direction on SETUP_BUTTON

	PORTB |= (1 << CAR1_BUTTON);		// Set 1 on CAR1_BUTTON pin (enable pull-up)
	PORTB |= (1 << SELECT_BUTTON);		// Set 1 on SELECT_BUTTON pin (enable pull-up)
	PORTB |= (1 << SETUP_BUTTON);		// Set 1 on SETUP_BUTTON pin (enable pull-up)

	_delay_ms(10);						// wait 10ms before reading the buttons

	// Check the initial state for CAR1_BUTTON
	if (bit_is_clear(BUTTON_PINS, CAR1_BUTTON)) {
		PORTA &= ~(1 << CAR1_BUTTON_LED);	// Turn CAR1_BUTTON_LED ON
		PORTA |= (1 << CAR1_RELAY);			// Turn CAR1_RELAY ON ( drive FET to HIGH )
		button_state.car1 = ON;
		button_state.car1_old = button_state.car1;
	}	
	else {
		PORTA |= (1 << CAR1_BUTTON_LED);	// Turn CAR1_BUTTON_LED OFF
		PORTA &= ~(1 << CAR1_RELAY);		// Turn CAR1_RELAY OFF ( drive FET to LOW )
		button_state.car1 = OFF;
		button_state.car1_old = button_state.car1;
	}
}

// Interrupt initializations
void int_init()
{
	GIMSK |= (1 << PCIE0);					// Enable pin change interrupts on pins PB[0...3]
	
	TIMSK |= (1 << OCIE1A);					// Enable comparator interrupt for RT clock
	TCCR1B |= (1 << CS10)|(1 << CS12)|(1 << CS13);
	OCR1A = 195;							// Overflow ~0.1sec ( 1 step @  9,4 ms )
	
	ACSRA |= (1 << ACD);					// Disable analog comparator
	ADCSRA |= (1 << ADIE);					// Enable A/D conversion interrupt
	ADCSRA |= (1 << ADPS2)|(1 << ADPS1);	// Settings CK/64 prescaler for A/D clock ( CPU 8MHz @ 125kHz )
	ADCSRA |= (1 << ADEN);					// Enable A/D conversion ( will be executed in IDLE sleep mode )
	DIDR0 |= (1 << ADC0D);					// Disable digital input for ADC0 pin ( used for ADC )

	OSCCAL = 0x80;							// OSCCAL = 0x87 was original value ( 1 step @ 6,6 ms )
	sei();									// Enable global interrupts
}

void display_init()
{
	lcd_init(LCD_DISP_ON);

	display_clock();
	display_temperature();
	display_timer();
}

// Interrupt handling for pin change interrupt
ISR(PCINT_vect)
{
	if (!int_flag.io_pin) {			// No new button event before previous event handled
		int_flag.io_pin = true;
		button_state.car1 = OFF;
		button_state.select = OFF;
		button_state.setup = OFF;
	}
}

// Interrupt handling for timer0 interrupt
ISR(TIMER1_COMPA_vect)
{
	_delay_us(506); //503 -> 507 -> 505 -> 506 ( edistää ) -> 507 ( jätättää )
	if ( button_state.select == ON && !int_flag.selection_tmr) {
		if ( --selection_counter == 0 )
			int_flag.selection_tmr = true;
	}

	if ( --rt_clock_counter == 0 ) {
			int_flag.rt_clock_tmr = true;
			rt_clock_counter = timer0_scaler;
			_delay_us(110); //20 -> 50 -> 100 -> 120 -> 110
	}

	TCNT1 = 0;				// Start counting again from zero
}

// Interrupt handling fro A/D conversion
ISR(ADC_vect)
{
	if ( !int_flag.ad_conversion ) {
		int_flag.ad_conversion = true;
		adc_value = 0;
		adc_value = ADCL;					// ADCL must be read first!!
		adc_value |= (ADCH<<8);
		ADCSRA &= ~(1 << ADEN);				// disable A/D conversion
	}		
}

int main()
{
	uint8_t select_count = 0;				// counter for the cursor position
	bool edit_mode = false;					// enables editing mode after long press of the selection button
	bool clock_updated = false;				// resets seconds after time updated

	button_state.select = OFF;
	button_state.setup = OFF;

	selection_counter = timer0_scaler * selection_time_sec;
	rt_clock_counter = timer0_scaler;

	int_flag.rt_clock_tmr = false;
	int_flag.selection_tmr = false;
	int_flag.ad_conversion = false;
	int_flag.io_pin = false;

	io_init();								// initialize IO ports
	int_init();								// initialize timer,IO and A/D interrupts
	read_eeprom();
	display_init();							// initialize display

	// resume heating after reset if applicable
	if ( heating && button_state.car1 == OFF ) {
		PORTA &= ~(1 << CAR1_BUTTON_LED);	// Turn CAR1_BUTTON_LED ON
		PORTA |= (1 << CAR1_RELAY);			// Turn CAR1_RELAY ON ( drive FET to HIGH )
	}
	else if ( !heating && button_state.car1 == ON ) {
		heating = 1;
		update_eeprom();
	}		

	while(1)
	{
		// **********************************
		//
		// Handle timer interrupt flags here
		//
		// **********************************

		if ( int_flag.rt_clock_tmr == true ) {

			clock_seconds++;

			if ( !edit_mode ) {
				if ( clock_seconds >= 60 ) {
					clock_minutes += (uint8_t)(clock_seconds / 60);
					clock_seconds -= 60 * ((uint8_t)(clock_seconds / 60));
					ADCSRA |= (1 << ADEN);					// enable A/D conversion in every minute
				}

				if ( clock_minutes >= 60 ) {
					clock_hours += (uint8_t)(clock_minutes / 60);
					clock_minutes -= 60 * ((uint8_t)(clock_minutes / 60));
					if ( clock_hours >= 24 )
						clock_hours = 0;
					}

				// Check if heating start or end time passed
				if (heating) {
					if ( clock_hours >= timer_hours && clock_minutes >= timer_minutes && button_state.car1 == OFF) {
						PORTA |= (1 << CAR1_BUTTON_LED);	// Set 1 on CAR1_BUTTON_LED pin (LED turn OFF)
						PORTA &= ~(1 << CAR1_RELAY);		// Set 0 on CAR1_RELAY pin (RELAY turn OFF)
						heating = 0;
						update_eeprom();
					}
				}
				else if ( heating_hours > 0 || heating_minutes > 0) {
					// Calculate starting time
					int starting_hour = timer_hours - heating_hours;
					int starting_minute = timer_minutes - heating_minutes;
					
					if (starting_hour < 0)
						starting_hour += 24;
					if (starting_minute < 0) {
						starting_minute += 60;
						starting_hour--;
					}

					if ( (clock_hours == starting_hour && clock_minutes == starting_minute) ||
						 (clock_hours > starting_hour && clock_hours < timer_hours) || (clock_hours == timer_hours && clock_minutes < timer_minutes) ) {
						PORTA &= ~(1 << CAR1_BUTTON_LED);	// Turn CAR1_BUTTON_LED ON
						PORTA |= (1 << CAR1_RELAY);			// Turn CAR1_RELAY ON ( drive FET to HIGH )
						heating = 1;
						update_eeprom();
					}
				}

				display_clock();
			}

			int_flag.rt_clock_tmr = false;
		}

		if ( int_flag.selection_tmr == true && button_state.select == ON ) {
			edit_mode ^= true;
			if ( !edit_mode ) {
				if ( clock_updated ) {
					clock_seconds = 0;
					rt_clock_counter = timer0_scaler;
					TCNT1 = 0;
					clock_updated = false;
				}					
				lcd_command(LCD_DISP_ON);
				display_clock();
				display_timer();
				update_eeprom();
			}				
			else {
				select_count = 0;
				lcd_gotoxy(2,0);
				lcd_putc(':');
				lcd_home();
				lcd_command(LCD_DISP_ON_CURSOR_BLINK);
			}

			int_flag.selection_tmr = false;
			button_state.select = OFF;
		}

		// ******************************************
		//
		// Handle A/D conversion interrupt flags here
		//
		// ******************************************
		
		if ( int_flag.ad_conversion == true ) {
			convert_temperature();
			// check temperature table heating times
			if (!heating) {
				for (uint8_t i = 0;i < TEMP_TABLE_LENGTH; i++ ) {
					if (temp_table[i] <= (int)temperature) {
						heating_hours = (uint16_t)(heat_time_table[i] / 60);
						heating_minutes = heat_time_table[i];
						heating_minutes -= heating_hours * 60;
						i = TEMP_TABLE_LENGTH;
					}
				}
			}
			if ( !edit_mode )
				display_temperature();
			int_flag.ad_conversion = false;
		}

		// *******************************
		//
		// Handle IO interrupt flags here
		//
		// *******************************

		if ( int_flag.io_pin == true ) {
			_delay_ms(10);							// wait 10ms before reading the buttons

			if (bit_is_clear(BUTTON_PINS, CAR1_BUTTON) )
				button_state.car1 = ON;

			if ( bit_is_clear(BUTTON_PINS, SELECT_BUTTON) ) {
				button_state.select = ON;
				selection_counter = timer0_scaler * selection_time_sec;
			}
			else if ( bit_is_clear(BUTTON_PINS, SETUP_BUTTON) ) {
				button_state.setup = ON;
			}
			
			if ( button_state.car1 != button_state.car1_old ) {
				button_state.car1_old = button_state.car1;

				if ( button_state.car1 == ON ) {
					PORTA &= ~(1 << CAR1_BUTTON_LED);	// Set 0 on CAR1_BUTTON_LED pin (LED turn ON)
					PORTA |= (1 << CAR1_RELAY);			// Set 1 on CAR1_RELAY pin (RELAY turn ON)
					heating = 1;
					update_eeprom();
				}
				else {
					PORTA |= (1 << CAR1_BUTTON_LED);	// Set 1 on CAR1_BUTTON_LED pin (LED turn OFF)
					PORTA &= ~(1 << CAR1_RELAY);		// Set 0 on CAR1_RELAY pin (RELAY turn OFF)
					heating = 0;
					update_eeprom();
				}

			}
			else if ( button_state.setup == ON && edit_mode == true ) {
				button_state.setup = OFF;
				switch ( select_count ) {
					case CLOCK_HOURS_0:		update_hours(&clock_hours, str_clock_hours, 1);
											break;

					case CLOCK_HOURS_1:		update_hours(&clock_hours, str_clock_hours, 0);
											break;

					case CLOCK_MINUTES_0:	update_minutes(&clock_minutes, str_clock_minutes, 1);
											clock_updated = true;
											break;

					case CLOCK_MINUTES_1:	update_minutes(&clock_minutes, str_clock_minutes, 0);
											clock_updated = true;
											break;

					case TIMER1_HOURS_0:	update_hours(&timer_hours, str_timer_hours, 1);
											break;

					case TIMER1_HOURS_1:	update_hours(&timer_hours, str_timer_hours, 0);
											break;

					case TIMER1_MINUTES_0:	update_minutes(&timer_minutes, str_timer_minutes, 1);
											break;

					case TIMER1_MINUTES_1:	update_minutes(&timer_minutes, str_timer_minutes, 0);
											break;
				}
			}
			else if ( button_state.select == ON && edit_mode == true ) {
				select_count++;
				switch ( select_count ) {
					// Clock hours
					case CLOCK_HOURS_1:		lcd_gotoxy(select_count,0);
											break;
					// Clock minutes
					case CLOCK_MINUTES_0: 
					case CLOCK_MINUTES_1:	lcd_gotoxy(select_count+1,0);
											break;
					// Timer hours
					case TIMER1_HOURS_0:
					case TIMER1_HOURS_1:	lcd_gotoxy(select_count-4,1);
											break;
					// Timer minutes
					case TIMER1_MINUTES_0:
					case TIMER1_MINUTES_1:	lcd_gotoxy(select_count-3,1);
											break;
					// Back to beginning
					default:
							lcd_home();
							select_count = 0;
				}
				
			}
			//io_pin_handled = true;
			int_flag.io_pin = false;
		}

		// check if A/D conversion is requested
		if ( bit_is_clear(ADCSRA, ADEN) )
			set_sleep_mode(SLEEP_MODE_IDLE);			// set IDLE sleep mode
		else
			set_sleep_mode(SLEEP_MODE_ADC);				// set A/D conversion mode
		
			cli();										// disable interrupts
		    sleep_enable();								// enable sleep
			sei();										// enable interrupts just before sleep
			sleep_cpu();								// enter to sleep
			sleep_disable();							// disable sleep after wake-up
	}

	return 0;
}

// Stores important values to EEPROM for possible reset
void read_eeprom()
{
	clock_hours = eeprom_read_byte(&NV_clock_hours);
	clock_minutes = eeprom_read_byte(&NV_clock_minutes);
	timer_hours = eeprom_read_byte(&NV_timer_hours);
	timer_minutes = eeprom_read_byte(&NV_timer_minutes);
	heating = eeprom_read_byte(&NV_heating);
}

// Reads values from EEPROM
void update_eeprom()
{
	eeprom_update_byte(&NV_clock_hours, clock_hours);
	eeprom_update_byte(&NV_clock_minutes, clock_minutes);
	eeprom_update_byte(&NV_timer_hours, timer_hours);
	eeprom_update_byte(&NV_timer_minutes, timer_minutes);
	eeprom_update_byte(&NV_heating, heating);
}

void display_clock()
{
	static bool blink = false;

	itoa(clock_minutes, str_clock_minutes, 10);
	itoa(clock_hours, str_clock_hours, 10);

	if ( clock_minutes < 10 ) {
		str_clock_minutes[1] = str_clock_minutes[0];
		str_clock_minutes[0] = '0';
	}
	
	if ( clock_hours < 10 ) {
		str_clock_hours[1] = str_clock_hours[0];
		str_clock_hours[0] = '0';
	}

	lcd_home();
	lcd_puts(str_clock_hours);
	lcd_putc(' ');
	lcd_puts(str_clock_minutes);
	blink ^= true;
	if (blink) { lcd_gotoxy(2,0); lcd_putc(':');}
}

void display_timer()
{
	itoa(timer_minutes, str_timer_minutes, 10);
	itoa(timer_hours, str_timer_hours, 10);
	
	if ( timer_minutes < 10 ) {
		str_timer_minutes[1] = str_timer_minutes[0];
		str_timer_minutes[0] = '0';
	}
	
	if ( timer_hours < 10 ) {
		str_timer_hours[1] = str_timer_hours[0];
		str_timer_hours[0] = '0';
	}

	lcd_gotoxy(0,1);
	lcd_puts(str_timer_hours);
	lcd_putc(':');
	lcd_puts(str_timer_minutes);
}

void display_temperature()
{
	char str_temperature[5] = "0000\0";

	itoa((int)temperature,str_temperature, 10);
	lcd_gotoxy(9,0);
	if ( (int)temperature < 10 && (int)temperature > -10 )
		lcd_putc(' ');
	if ( (int)temperature >= 0 )
		lcd_putc('+');
	lcd_puts(str_temperature);
	lcd_putc('.');
	temperature = fabs(temperature);
	temperature -= (int)temperature;
	itoa((int)(temperature*10),str_temperature, 10);
	lcd_puts(str_temperature);
	lcd_putc(0xB2);
	lcd_putc('C');
}

void convert_temperature()
{
	temperature = 0.0;
	temperature = adc_value * 4.8828125;	// resolution = 5/1024 * 1000mV
	temperature -= 1350;					// 1.125V ~ 0*C ( 1.350 calibrated value )
	temperature /= 22.5;					// Temp sensor ouputs 22.5mV / *C
}

void update_hours(uint8_t *hours, char *str_hours, bool tens)
{
	if (sizeof(str_hours) / sizeof(char) < 2)
		return;

	switch (tens) {
		case 0: if (*hours == 23) { *hours = 20; str_hours[1] = '0'; }
				else if ( str_hours[1] == '9' ) { hours -= 9; str_hours[1] = '0'; }
				else { *hours += 1; str_hours[1] += 1; }
				lcd_putc(str_hours[1]);
				lcd_command(LCD_MOVE_CURSOR_LEFT);
				break;

		case 1:	if (*hours > 13) { *hours -= 10; str_hours[0] = '0'; }
				else { *hours += 10; str_hours[0] += 1; }
				lcd_putc(str_hours[0]);
				lcd_command(LCD_MOVE_CURSOR_LEFT);
				break;
	}
}

void update_minutes(uint8_t *minutes, char *str_minutes, bool tens)
{
	if (sizeof(str_minutes) / sizeof(char) < 2)
	return;

	switch (tens) {
		case 0:	if (*minutes == 59) { *minutes = 50; str_minutes[1] = '0'; }
				else if ( str_minutes[1] == '9' ) { *minutes -= 9; str_minutes[1] = '0'; }
				else { *minutes += 1; str_minutes[1]++; }
				lcd_putc(str_minutes[1]);
				lcd_command(LCD_MOVE_CURSOR_LEFT);
				break;

		case 1:	if (*minutes >= 50) { *minutes -= 50; str_minutes[0] = '0'; }
				else { *minutes += 10; str_minutes[0]++; }
				lcd_putc(str_minutes[0]);
				lcd_command(LCD_MOVE_CURSOR_LEFT);
				break;
	}
}

/* sending byte example
 void sputchar( uint8_t c )
 {
	 c = ~c;
	 STX_PORT &= ~(1<<STX_BIT);            // start bit
	 for( uint8_t i = 10; i; i-- ){        // 10 bits
		_delay_us( 1e6 / BAUD );            // bit duration
	 if( c & 1 )
		STX_PORT &= ~(1<<STX_BIT);        // data bit 0
	 else
		STX_PORT |= 1<<STX_BIT;           // data bit 1 or stop bit
	 c >>= 1;
     }
}

*/

// EOF
