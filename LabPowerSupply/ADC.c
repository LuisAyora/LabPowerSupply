/******************************************************************************
 *
 *	@file: ADC.h
 *  @brief: ADC driver for the ATmega328P. This implementation assumes a clock
			speed of 1 MHz.
 *  @compiler: AVR-GCC
 *  @version: 0.1
 *  @author: Luis A Ayora
 *  @date: 29-11-2019
 *  @todo: - Implement functions in .c file
 *		   - Test ADC conversion on board
 *
 *****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ADC.h"

void ADC_Init(void) {
	// Set the voltage reference to AVcc
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);

	// Set the clock source to 125 kHz, Prescaler of 128
	// Enable Auto Trigger, Interrupts and the ADC
	ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) 
			  | (1 << ADATE) | (1 << ADIE) | (1 << ADEN));

	// Set the automatic trigger for TIMER/COUNTER0 compare match
	ADCSRB |= ((1 << ADTS1) | (1 << ADTS0));
	ADCSRB &= ~(1 << ADTS2);

	// Set the channel to ADCIN0
	ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
}

void ADC_In(void) {
	
}

ISR (ADC_vect) {
	TIFR0 = (1 << OCF0A);
}
