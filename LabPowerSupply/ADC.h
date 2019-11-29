/******************************************************************************
 *
 *	@file: ADC.h
 *  @brief: ADC driver for the ATmega328P
 *  @compiler: AVR-GCC
 *  @version: 0.1
 *  @author: Luis A Ayora
 *  @date: 29-11-2019
 *  @todo: - Implement functions in .c file
 *		   - Test ADC conversion on board
 *
 *****************************************************************************/


#ifndef ADC_H_
#define ADC_H_

/*
 *	@brief: Initialises the ADC 
 */
void ADC_Init(void);

/*
 *	@brief: Reads the value of the ADC.
 */
void ADC_In(void);


#endif /* ADC_H_ */