/*
 * other_code.c
 *
 * Created: 8/12/2019 6:32:51 AM
 *  Author: laayo
 */ 

 #include <avr/io.h>
 #include <avr/interrupt.h>

 ISR (INT0_vect) {
	 // static uint8_t LED = 0x00;
	 PORTB ^= (1 << PB0);
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