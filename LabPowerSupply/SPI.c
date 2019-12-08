/******************************************************************************
 *
 *	@file: SPI.h
 *  @brief: SPI driver for the ATmega328P
 *  @compiler: AVR-GCC
 *  @version: 0.1
 *  @author: Luis A Ayora
 *  @date: 06-12-2019
 *  @todo: Implement functionality for SS and for multiple slave
 *
 * This is an SPI driver for the ATmega 48A/PA/88A/PA/168A/PA/328/P
 *****************************************************************************/
 
#include <avr/io.h>
#include "SPI.h"

#define SPI2X_HIGH 1
#define SPI2X_LOW 0
#define SPR1_HIGH 1
#define SPR1_LOW 0
#define SPR0_HIGH 1
#define SPR0_LOW 0

// SPI pins. Change for your particular application:
#define DDR_SPI  DDRB
#define SPI_PORT PORTB
#define DD_MOSI  PB3
#define DD_MISO  PB4
#define DD_SCK   PB5
#define DD_SS    PB2

// Function declarations:
void SPI_SetClockRate(uint8_t SPI2X_Level, uint8_t SPR1_Level, uint8_t SPR0_Level);

// Function implementations:
void SPI_Init(uint8_t Mode_Select, uint8_t Interrupt_Enable, uint8_t Data_Order,
			  uint8_t Clock_Polarity, uint8_t Clock_Phase, uint8_t Clock_Rate) {
	// Select mode
	if (Mode_Select == MASTER) {
		// MOSI, SS' and SCK as outputs, all the rest as input:
		DDR_SPI |= ((1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS));
		DDR_SPI &= ~(1 << DD_MISO);
		SPCR = ((1 << MSTR) | (1 << SPE));
	} else if(Mode_Select == SLAVE) {
		// MISO as output, all others as input:
		DDR_SPI |= (1 << DD_MISO);
		DDR_SPI &= ~((1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS));
		SPCR |= (1 << SPE);
		SPCR &= ~(1 << MSTR);
	}
	// Select whether interrupts are enabled or 
	if (Interrupt_Enable == INTERRUPTS_ENABLED) {
		SPCR |= (1 << SPIE);
	} else if(Interrupt_Enable == INTERRUPTS_DISABLED) {
		SPCR &= ~(1 << SPIE);
	}
	// Select Data Order
	if (Data_Order == LSB_FIRST) {
		SPCR |= (1 << DORD);
	} else if(Data_Order == MSB_FIRST) {
		SPCR &= ~(1 << DORD);
	}
	// Select Clock Polarity and Clock Phase Functionality
	if (Clock_Polarity == SCK_HIGH_ON_IDLE) {
		SPCR |= (1 << CPOL);
	} else if(Clock_Polarity == SCK_LOW_ON_IDLE) {
		SPCR &= ~(1 << CPOL);
	}

	if (Clock_Phase == SAMPLE_ON_TRAILING_EDGE) {
		SPCR |= (1 << CPHA);
	} else if(Clock_Phase == SAMPLE_ON_LEADING_EDGE) {
		SPCR &= ~(1 << CPHA);
	}
	// Select Clock Rate
	switch (Clock_Rate) {
		case F_OSC_2:	SPI_SetClockRate(SPI2X_HIGH, SPR1_LOW, SPR0_LOW);
						break;
		case F_OSC_4:	SPI_SetClockRate(SPI2X_LOW, SPR1_LOW, SPR0_LOW);
						break;
		case F_OSC_8:	SPI_SetClockRate(SPI2X_HIGH, SPR1_LOW, SPR0_HIGH);
						break;
		case F_OSC_16:	SPI_SetClockRate(SPI2X_LOW, SPR1_LOW, SPR0_HIGH);
						break;
		case F_OSC_32:	SPI_SetClockRate(SPI2X_HIGH, SPR1_HIGH, SPR0_LOW);
						break;
		case F_OSC_64:	SPI_SetClockRate(SPI2X_LOW, SPR1_HIGH, SPR0_LOW);
						break;
		case F_OSC_128: SPI_SetClockRate(SPI2X_LOW, SPR1_HIGH, SPR0_HIGH);
						break;
	}
}

void SPI_SetClockRate(uint8_t SPI2X_Level, uint8_t SPR1_Level, uint8_t SPR0_Level) {
	if (SPI2X_Level == SPI2X_HIGH) {
		SPSR |= (1 << SPI2X);
	} else if (SPI2X_Level == SPI2X_LOW) {
		SPSR &= ~(1 << SPI2X);
	}

	if (SPR1_Level == SPR1_HIGH) {
		SPCR |= (1 << SPR1);
		} else if (SPR1_Level == SPR1_LOW) {
		SPCR &= ~(1 << SPR1);
	}

	if (SPR0_Level == SPR0_HIGH) {
		SPCR |= (1 << SPR0);
		} else if (SPR0_Level == SPR0_LOW) {
		SPCR &= ~(1 << SPR0);
	}
}

void SPI_MasterTransmit(char cData) {
	// Start transmission:
	SPDR = cData;
	
	// Wait for transmission to complete:
	while (!(SPSR & (1 << SPIF)))
		;
}

char SPI_SlaveReceive(void) {
	// Wait for reception to complete:
	while (!(SPSR & (1 << SPIF)))
		;

	// Return contents of Data Register:
	return SPDR;
}