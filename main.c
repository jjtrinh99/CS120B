#include <avr/io.h>
#include <util/delay.h>
#include <bit.h>
#include <avr/eeprom.h>
#include <keypad.h>
#include "io.c"
#include "nokia5110.h"
#include <avr/interrupt.h>

volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.
// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks
void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B = 0x0B;// bit3 = 0: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A = 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register
	TIMSK1 = 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1=0;

	_avr_timer_cntcurr = _avr_timer_M;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds

	//Enable global interrupts
	SREG |= 0x80; // 0x80: 1000000
}

void TimerOff() {
	TCCR1B = 0x00; // bit3bit1bit0=000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; // Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { // results in a more efficient compare
		TimerISR(); // Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void ADC_Init(){
	ADCSRA |= (1<< ADEN) | (1 << ADSC) | (1 << ADATE);
}

void transmit_data(unsigned char data) {
	unsigned char i;
	for (i = 0; i < 8 ; ++i) {
		PORTC = 0x08; // set srclk to 1 to allow data to actually come
		PORTC |= ((data >> i) & 0x01); //set ser to the data being sent in
		PORTC |= 0x04; //make srclk 1 to move the input into the shift itself
	}
	PORTC |= 0x02; //make rclk 1 to copy from shift to storage in prep of outputting it
	PORTC = 0x00; //clear it all to output
}

enum State{Wait, Cwalk, SWalk, NoSWalk,Cars,NoCars, Error}button;
//NoSWalks7
#define A0  ((~PINA) & 0x01) //swalk
#define A1  ((~PINA) & 0x02) //no swalk
#define A2  ((~PINA) & 0x04) //car
#define A3  ((~PINA) & 0x08) //nocar
#define A4  ((~PINA) & 0x10) //cwalk
#define A6  ((~PINA) & 0x40) //ir sensor, moved it from a5 to a6 to avoid bugging

unsigned char cnt = 0x00;

void Tick(){
	switch(button){//transitions
		case Wait:
		if((A0 && (cnt > 5))){
			button = SWalk;
			cnt = 0;
		}
		else if(A2 && (cnt > 5)){
			button = Cars;
			cnt = 0;
		}
		else if(A4 && (cnt > 5)){
			button = Cwalk;
			cnt = 0;
		}
		else{
			button = Wait;
		}
		break;
		case Cwalk:
		if((cnt>10) && A2){
			//save the state to eeprom
			button = Error;
			cnt = 0;
		}
		else if((cnt>10) && A1){
			button = NoSWalk;
			cnt = 0;
		}
		else if((cnt>5) && A0){
			button = SWalk;
			cnt = 0;
		}
		else{
			button = Cwalk;
		}
		break;
		case SWalk:
		if((cnt>5) && A1){
			button = NoSWalk;
			cnt = 0;
		}
		else if((cnt>5) && A2){
			button = Error;
			cnt = 0;
		}
		else if(A4 && (cnt>5)){
			button = Cwalk;
	
		}
		else{
			button = SWalk;
		}
		break;
		case NoSWalk:
		if((cnt>5) && A0){
			button = SWalk;
			cnt = 0;
		}
		else if((cnt>5) && A2){
			button = Cars;
			cnt = 0;
		}
		else if(A4 && (cnt>5)){
			button = Cwalk;
			
		}
		else{
			button = NoSWalk;
		}
		break;
		case Cars:
		if((cnt>5) && A0){
			//save the state to eeprom
			button = Error;
			cnt = 0;
		}
		else if((cnt>5) && A3){
			button = NoCars;
			cnt = 0;
		}
		else{
			button = Cars;
		}
		break;
		case NoCars:
		if((cnt>5) && A2){
			button = Cars;
			cnt = 0;
		}
		else if((cnt>5) && A0){
			button = SWalk;
			cnt = 0;
		}
		else if((cnt>5) &&  A4){
			button = Cwalk;
			cnt = 0;
		}
		else{
			button = NoCars;
		}
		break;
		case Error:
		//if A5, go back to previous state. otherwise, stay in error.
		if(cnt<=15){
			button = Error;
		}
		else{
			button = Wait;
			cnt = 0;
		}
	}
	switch(button){ //A C T I O N S
		case Wait:
			++cnt;
			//PORTC = 0xFF;
			transmit_data(0xFF);
			nokia_lcd_clear();
			nokia_lcd_write_string("u u",1);
			nokia_lcd_set_cursor(0,10);
			nokia_lcd_write_string(" w",1);
			nokia_lcd_render();
			break;
		case SWalk:
			++cnt;
			if(A6){
				transmit_data(0x00);
				nokia_lcd_clear();
				nokia_lcd_write_string("Sidewalk!",1);
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("Day!",1);
				nokia_lcd_render();
				//PORTC = 0x00; //0
			}
			else{
				transmit_data(0x07);
				nokia_lcd_clear();
				nokia_lcd_write_string("Sidewalk!",1);
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("Night!",1);
				nokia_lcd_render();
				//PORTC = 0x07;
			}
			break;
		case NoSWalk:
			++cnt;
			if(A6){
				transmit_data(0x00);
				nokia_lcd_clear();
				nokia_lcd_write_string("No Sidewalk!",1);
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("Day!",1);
				nokia_lcd_render();
				//PORTC = 0x06; //1
			}
			else{
				transmit_data(0x06);
				nokia_lcd_clear();
				nokia_lcd_write_string("No Sidewalk!",1);
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("Night!",1);
				nokia_lcd_render();
				//PORTC = 0x00;
			}
			break;
		case Cars:
			++cnt;
			nokia_lcd_clear();
			nokia_lcd_write_string("Cars!",1);
			nokia_lcd_render();
			transmit_data(0xF0);
			//PORTC = 0xF0; //2
			break;
		case NoCars:
			++cnt;
			if(!A6){
				transmit_data(0x0F);
				//PORTC = 0x0F; //bit position 3
				nokia_lcd_clear();
				nokia_lcd_write_string("No Cars, Go!",1);
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("Night!",1);
				nokia_lcd_render();
			}
			else{
				transmit_data(0x00);
				nokia_lcd_clear();
				nokia_lcd_write_string("No Cars, Go!",1);
				nokia_lcd_set_cursor(0,10);
				nokia_lcd_write_string("Day!",1);
				nokia_lcd_render();
				//PORTC = 0x00;
			}
			break;
		case Error:
			++cnt;
			nokia_lcd_clear();
			nokia_lcd_write_string("Error! EMT!",1);
			nokia_lcd_render();
			transmit_data(0x99);
			//PORTC = 0x99; ; //bit position 5
			break;
		case Cwalk:
			++cnt;
			nokia_lcd_clear();
			nokia_lcd_write_string("Cross!!",1);
			nokia_lcd_render();
			transmit_data(0x09);
			//PORTC = 0x09; //16, bit position 4
			break;
	}
}

void main()
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRC = 0xFF; PORTC = 0x00;
	
	TimerSet(10);
	TimerOn();

	button = Wait;
	
	nokia_lcd_init();
	nokia_lcd_clear();
	nokia_lcd_write_string("!@#$%^... wat",1);
	nokia_lcd_render();
	
	while(1) {
		Tick();
		while (!TimerFlag);	// Wait 1 sec
		TimerFlag = 0;	
	}
	
}
