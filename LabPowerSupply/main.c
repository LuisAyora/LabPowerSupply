/*
 * LabPowerSupply.c
 *
 * Created: 27/11/2019 7:29:22 PM
 * Author : Luis A Ayora
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ADC.h"

ISR (INT0_vect) {
	// static uint8_t LED = 0x00;
	PORTB ^= (1 << PB0);
}

ISR (TIMER0_COMPA_vect) {
	static uint8_t overflowCounter = 0;

	if (overflowCounter == 1) {
		PORTB ^= (1 << PB1);
		overflowCounter = 0;
	} else {
		overflowCounter++;
	}
}

void INT0_EnableOnFallingEdge(void) {
	// Configure PD2 as an input pin:
	DDRD &= ~(1 << PD2);

	// Falling edge setup:
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);

	// Enable INT0 interrupt:
	EIMSK |= (1 << INT0);
}

void OutputLEDConfigure(void) {
	// Configure PB0 as output:
	DDRB |= ((1 << PB0) | (1 << PB1));
}

void TIMER0_EnableOnCTC(void) {
	// Configure Control Register A for normal port operation:
	TCCR0A &= ~((1 << COM0A0) | (1 << COM0A1));

	// Set up CTC mode:
	TCCR0A |= (1 << WGM01);
	TCCR0A &= ~(1 << WGM00);
	TCCR0B &= ~(1 << WGM02);

	// Initialise timer and select the compare match:
	TCNT0 = 0x00;
	OCR0A = 0xF4;
	
	// Enable Output Compare interrupts:
	// TIMSK0 |= (1 << OCIE0A);

	// Set clock pre-scaler to CLK/1024:
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);
}

int main(void) {
	OutputLEDConfigure();
	INT0_EnableOnFallingEdge();
	TIMER0_EnableOnCTC();
	ADC_Init();
	sei();

    while (1) 
    {
		
    }
}