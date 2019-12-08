/*
 * TIMER0A.c
 *
 * Created: 8/12/2019 6:22:08 AM
 *  Author: laayo
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "TIMER0A.h"

void TIMER0A_EnableOnCTC(void) {
	// Configure Control Register A for normal port operation:
	TCCR0A &= ~((1 << COM0A0) | (1 << COM0A1));

	// Set up CTC mode:
	TCCR0A |= (1 << WGM01);
	TCCR0A &= ~(1 << WGM00);
	TCCR0B &= ~(1 << WGM02);

	// Initialise timer and select the compare match:
	TCNT0 = 0x00;
	OCR0A = 0x31;
	
	// Enable Output Compare interrupts:
	// TIMSK0 |= (1 << OCIE0A);

	// Set clock pre-scaler to CLK/8:
	TCCR0B |= ((1 << CS01) | (1 << CS00));
	TCCR0B &= ~(1 << CS02);
}

ISR (TIMER0_COMPA_vect) {
	PORTB ^= (1 << PB1);
}


