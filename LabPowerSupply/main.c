/*
 * LabPowerSupply.c
 *
 * Created: 27/11/2019 7:29:22 PM
 * Author : Luis A Ayora
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

int main(void) {
	DDRB = (1 << DDB0);
    /* Replace with your application code */
    while (1) 
    {
		PORTB |= (1 << PB0);
		_delay_ms(500);
		PORTB &= ~(1 << PB0);
		_delay_ms(500);
    }
}