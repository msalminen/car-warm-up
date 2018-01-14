/*
 * CarWarmUpControl.h
 *
 * Created: 03.07.2013 14:35:24
 *  Author: salmarko
 
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


#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_


// IO definitions
#define CAR1_RELAY		PA3		// Define car1 relay pin
#define CAR1_BUTTON_LED PA1		// Define car1 warm-up control button led pin
#define CAR1_BUTTON		PB0		// Define car1 warm-up control button pin
#define SELECT_BUTTON	PB1		// Define selection button pin
#define SETUP_BUTTON	PB2		// Define setup button pin
#define BUTTON_PINS		PINB	// Define buttons pins for reading button states

// Button states
enum {ON, OFF};

// Edit fields
enum {CLOCK_HOURS_0, CLOCK_HOURS_1, CLOCK_MINUTES_0, CLOCK_MINUTES_1, TIMER1_HOURS_0, TIMER1_HOURS_1, TIMER1_MINUTES_0, TIMER1_MINUTES_1};

// Globals
const uint8_t selection_time_sec = 2;	// Selection timeout
const uint16_t timer0_scaler = 10;		// timer0 scaling counter for 1s
uint8_t heating = 0;
float temperature = 0.0;
volatile uint16_t selection_counter;	// counter for selection timeout
volatile uint16_t rt_clock_counter;		// counter for rt clock;
volatile uint16_t adc_value = 0;		// container for A/D conversion result
volatile uint8_t io_pin_handled = false;

uint8_t clock_hours = 0;
uint8_t clock_minutes = 0;
uint16_t clock_seconds = 0;
char str_clock_hours[3] = "00\0";
char str_clock_minutes[3] = "00\0";
uint8_t timer_hours = 0;
uint8_t timer_minutes = 0;
char str_timer_hours[3] = "00\0";
char str_timer_minutes[3] = "00\0";

uint16_t heating_hours = 0;
uint16_t heating_minutes = 0;

#define TEMP_TABLE_LENGTH	6
int temp_table[TEMP_TABLE_LENGTH] = {6,0,-5,-10,-20,-50};
uint16_t heat_time_table[TEMP_TABLE_LENGTH] = {0, 30, 90, 120, 360, 360};

uint8_t EEMEM NV_clock_hours = 00;
uint8_t EEMEM NV_clock_minutes = 00;
uint8_t EEMEM NV_timer_hours = 07;
uint8_t EEMEM NV_timer_minutes = 30;
uint8_t EEMEM NV_heating = 0;

volatile struct
{
	bool selection_tmr: 1;				// Selection time is used for checking select button state after selection timeout
	bool rt_clock_tmr: 1;				// Real time clock ready to update
	bool ad_conversion: 1;				// A/D conversion ready to read
	bool io_pin: 1;
} int_flag;

volatile struct
{
	uint8_t car1: 1;
	uint8_t car1_old: 1;
	uint8_t select: 1;
	uint8_t setup: 1;
} button_state;

// Function declarations
void read_eeprom();
void update_eeprom();

void display_clock();
void display_timer();
void display_temperature();
void convert_temperature();
void update_hours(uint8_t *, char *, bool);
void update_minutes(uint8_t *, char *, bool);

#endif /* DEFINITIONS_H_ */

// EOF
