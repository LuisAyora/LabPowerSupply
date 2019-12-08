/*
 * LabPowerSupply.c
 *
 * Created: 27/11/2019 7:29:22 PM
 * Author : Luis A Ayora
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "TIMER0A.h"
#include "ADC.h"
#include "Nokia5110.h"

int main(void) {
	//TIMER0A_EnableOnCTC();
	//ADC_Init();
	Nokia5110_Init();

	sei();

    while (1) 
    {
		Nokia5110_SetCursor(0, 0);
		Nokia5110_OutString("Hi Sarah!");
		Nokia5110_SetCursor(0, 1);
		Nokia5110_OutUnsignedDecimal(1);
		Nokia5110_SetCursor(0, 2);
		Nokia5110_OutUnsignedDecimal(69);
		Nokia5110_SetCursor(0, 3);
		Nokia5110_OutUnsignedDecimal(420);
		Nokia5110_SetCursor(0, 4);
		Nokia5110_OutUnsignedDecimal(1234);
		Nokia5110_SetCursor(0, 5);
		Nokia5110_OutUnsignedDecimal(69420);
    }
}