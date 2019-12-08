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

#define MASTER 1
#define SLAVE  0
#define INTERRUPTS_ENABLED  1
#define INTERRUPTS_DISABLED 0
#define LSB_FIRST 1
#define MSB_FIRST 0
#define SCK_HIGH_ON_IDLE 1
#define SCK_LOW_ON_IDLE	 0
#define SAMPLE_ON_TRAILING_EDGE 1
#define SAMPLE_ON_LEADING_EDGE  0
#define F_OSC_2   1
#define F_OSC_4   2
#define F_OSC_8   3
#define F_OSC_16  4
#define F_OSC_32  5
#define F_OSC_64  6
#define F_OSC_128 7

#ifndef SPI_H_
#define SPI_H_

/*  SPI_Init
 *	@brief: Initialises SPI communications
 *	@param: Mode_Select
 *			MASTER or SLAVE mode
 *	@param: Interrupt_Enable
 *			INTERRUPTS_ENABLED or INTERRUPTS_DISABLED
 *	@param: Data_Order
 *			LSB_FIRST or MSB_FIRST
 *	@param: Clock_Polarity
 *			SCK_HIGH_ON_IDLE or SCK_LOW_ON_IDLE
 *	@param: Clock_Phase
 *			SAMPLE_ON_TRAILING_EDGE or SAMPLE_ON_LEADING_EDGE
 *	@param: Clock_Rate
 *			Select clock scaling by 2, 4, 8, 16, 32, 64 or 128
 */
void SPI_Init(uint8_t Mode_Select, uint8_t Interrupt_Enable, uint8_t Data_Order,
			  uint8_t Clock_Polarity, uint8_t Clock_Phase, uint8_t Clock_Rate);

/*
 *	@brief: Transmits a data packet as a Master
 *	@param: cData
 *			The data packet to be sent.
 */
void SPI_MasterTransmit(char cData);

/*
*	@brief: Receives a data packet as a slave
*/
char SPI_SlaveReceive(void);

#endif /* SPI_H_ */