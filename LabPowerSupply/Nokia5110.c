/******************************************************************************
 *
 *	@file: Nokia5110.c
 *  @brief: Implementation of the Nokia 5110 driver for the ATmega8 family
 *  @compiler: AVR-GCC
 *  @version: 0.1
 *  @author: Luis A Ayora
 *  @date: 06-12-2019
 *  @todo: 
 *
 * This is a Nokia 5110 LCD driver for the ATmega 48A/PA/88A/PA/168A/PA/328/P.
 * Communications to the LCD are done using the SPI interface to send data to
 * a display of 48x84 pixels.
 *
 * Some functions have been cloned or ported from the following sources:
 *	- Daniel Valvano - Nokia 5110 library for the Cortex-M4 architecture,
 *	  implemented on a TM4C123 TI microcontroller
 *	  Link: http://users.ece.utexas.edu/~valvano/arm/
 *	- Joseph Fortune - Nokia 5110 LCD library for the AVR. Contains some 
 *	  functionality to draw graphics on the display.
 *	  Link: https://github.com/josephfortune/NokiaLCD
 *****************************************************************************/
 
/* Adafruit Nokia 5110 LCD Pins and connections:
 * (Change your pin connections as desired)
 * --------------------------------------------
 * Pin 1: GND - Ground (0V)		-
 * Pin 2: VCC - Power  (3.3V)	- 
 * Pin 3: CLK - SPI Clock		- 
 * Pin 4: DIN - SPI Data in		- 
 * Pin 5: D/C - Data/Command	- 
 * Pin 6: CSE - Chip Select		- Active Low
 * Pin 7: RST - Reset			- Active Low
 * Pin 8: LED - Backlight (NC)	- 
 * --------------------------------------------
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "Nokia5110.h"
#include "SPI.h"

// Pin definitions (Change as required):
#define LCD_DDR		DDRB
#define LCD_PORT	PORTB
#define LCD_RST		PB0
#define LCD_DC		PB1
#define LCD_CS		PB2
#define LCD_DIN		PB3 
#define LCD_SCK		PB5

// Command Byte definitions:
#define DB7 7
#define DB6 6
#define DB5 5
#define DB4 4
#define DB3 3
#define DB2 2
#define DB1 1
#define DB0 0

// Other definitions:
#define CONTRAST	 0xB0 // Play around with this value for your specific LCD
#define LCD_MAX_COLS 12
#define LCD_MAX_ROWS 6
#define LCD_WIDTH	 84
#define LCD_HEIGHT	 48

enum WriteType {
	COMMAND,
	DATA
};

enum PowerMode {
	CHIP_ACTIVE,
	CHIP_POWER_DOWN
};

enum AddressType{
	HORIZONTAL_ADDRESSING,
	VERTICAL_ADDRESSING
};

enum InstructionSetType {
	BASIC_INSTRUCTION_SET,
	EXTENDED_INSTRUCTION_SET
};

enum DisplayConfiguration {
	DISPLAY_BLANK,
	NORMAL_MODE,
	ALL_DISPLAY_SEGMENTS_ON,
	INVERSE_VIDEO_MODE
};

enum TemperatureCoefficient {
	TEMPERATURE_COEFF_0,
	TEMPERATURE_COEFF_1,
	TEMPERATURE_COEFF_2,
	TEMPERATURE_COEFF_3
};

// Function prototypes:
/*	LCD_ConfigurePort
 *	@brief: Configures port pins to connect to the LCD
 *	
 *	Refer to the pin definitions and datasheet for descriptions of pin values.
 *	For the Nokia 5110 LCD, the RST, D_C and CSE need to be digital outputs on
 *	the microcontroller and inputs into the LCD.
 */
void LCD_ConfigurePort(void);

/*	LCD_SendFrame
 *	@brief: Sends a data frame to the LCD
 *	@param: frameType
 *			Sends whether the frame is data or a command
 *	@param: dataFrame
 *			The data frame being sent to the LCD
 *	
 *	Utility function to send data frames to the LCD as either data or commands.
 *	Refer to the datasheet for the different commands available for the LCD.
 */
void LCD_SendFrame(enum WriteType frameType, uint8_t dataFrame);

/*	LCD_Initialisation
 *	@brief: Initialises the LCD hardware
 *	
 *	The LCD is initialised immediately after power on by applying pulse on the
 *	reset line. This will reset all the internal registers of the LCD within a
 *	specified time. The reset pulse must pulled low within 100 ms.
 *  
 *  After reset, the LCD has the following state:
 *		- Power-down mode (PD = 1)
 *		- Horizontal addressing (V = 0)
 *		- Normal instruction set (H = 0)
 *		- Display blank bit (E = D = 0)
 *		- Address counter X6-0 = 0; Y2-0 = 0
 *		- Temperature control mode (TC1 = TC0 = 0)
 *		- Bias system (BS2-0 = 0)
 *		- Vlcd is 0 and HV generator is switched off (Vop6-Vop0 = 0)
 *		- RAM contents are undefined
 */
void LCD_Initialisation();

/*	LCD_FunctionSet
 *	@brief: Sets the power mode, addressing type and instruction set of the LCD
 *	@param: PowerMode:
 *			Chip active or Power-down mode
 *			AddressType:
 *			Horizontal or Vertical addressing
 *			InstructionSetType
 *			Basic or extended instruction set
 *	
 *	The Nokia 5110 is initialised by following the procedure outlined in the
 *	PCD8544 driver datasheet. Maximum baud clock for the LCD is 4 MHz.
 */
void LCD_FunctionSet(enum PowerMode pm, enum AddressType ad, 
					 enum InstructionSetType it);

/*	LCD_DisplayControl
 *	@brief: Selects the display mode of the LCD
 *	@param: DisplayConfiguration:
 *			Options for blank display, all segments on, normal and inverse mode
 *	
 *	Refer to datasheet for specific display modes.
 */
void LCD_DisplayControl(enum DisplayConfiguration dc);

/*	LCD_SetYAddressOfRAM
 *	@brief: Sets the Y address of RAM
 *	@param: Bank
 *			Selects the Y bank of RAM (0 <= Y <= 5)
 *	
 *	Refer to datasheet for more information of RAM locations.
 */
void LCD_SetYAddressOfRAM(uint8_t bank);

/*	LCD_SetXAddressOfRAM
 *	@brief: Sets the X address of RAM
 *	@param: Bank
 *			Selects the X bank of RAM (0 <= X <= 83)
 *	
 *	Refer to datasheet for more information of RAM locations.
 */
void LCD_SetXAddressOfRAM(uint8_t bank);

/*	LCD_TemperatureControl
 *	@brief: Sets the temperature control coefficient of the LCD
 *	@param: TemperatureCoefficient
 *			Runs coefficients 1 through 4
 *	
 *	Refer to datasheet for more information of RAM locations.
 */
void LCD_TemperatureControl(enum TemperatureCoefficient tc);

/*	LCD_BiasValue
 *	@brief: Sets the bias voltage levels of the LCD
 *	@param: TemperatureCoefficient
 *			Runs coefficients 1 through 4
 *	
 *	Refer to datasheet for more information of RAM locations. Recommended MUX
 *	rate is 48 for this LCD screen, so implementation restricts user's ability
 *	to configure this setting.
 */
void LCD_BiasValue(void);

/*	LCD_SetVopValue
 *	@brief: Sets the operating voltage of the LCD
 *	@param: contrast
 *			The contrast value of the LCD
 *	
 *	Contrast values dependent on each LCD for optimum performance and may 
 *	require some tweaking for you particular model. Recommended settings are:
 *		- Adafruit blue LCD (0xB0)
 *		- Sparkfun red LCD (0xB1)
 *		- Sparkfun blue LCD (0xB8)
 */
void LCD_SetVopValue(uint8_t contrast);

// Function implementations:
void Nokia5110_Init() {
	// Initialise SPI hardware and interface port:
	SPI_Init(MASTER, INTERRUPTS_DISABLED, MSB_FIRST, SCK_HIGH_ON_IDLE,
			 SAMPLE_ON_TRAILING_EDGE, F_OSC_32);
	LCD_ConfigurePort();

	// Initialisation routine of the LCD:
	LCD_Initialisation();
	LCD_FunctionSet(CHIP_ACTIVE, HORIZONTAL_ADDRESSING, 
					EXTENDED_INSTRUCTION_SET);
	LCD_BiasValue();
	LCD_TemperatureControl(TEMPERATURE_COEFF_0);
	LCD_SetVopValue(CONTRAST);
	LCD_FunctionSet(CHIP_ACTIVE, HORIZONTAL_ADDRESSING, 
					BASIC_INSTRUCTION_SET);
	LCD_DisplayControl(NORMAL_MODE);
}

void Nokia5110_OutChar(unsigned char charData) {
	LCD_SendFrame(DATA, 0x00); // Blank vertical line
	for (int i = 0; i < 5; i++) {
		LCD_SendFrame(DATA, ASCII[charData - 0x20][i]);
	}
	LCD_SendFrame(DATA, 0x00); // Blank vertical line
}

void Nokia5110_OutString(char *stringPtr) {
	while (*stringPtr) {
		Nokia5110_OutChar((unsigned char)*stringPtr);
		stringPtr++;
	}
}

void Nokia5110_OutUnsignedDecimal(uint32_t decData) {
	if (decData < 10) {
		Nokia5110_OutString("    ");
		Nokia5110_OutChar(decData + '0');
	} else if (decData < 100) {
		Nokia5110_OutString("   ");
		Nokia5110_OutChar(decData/10 + '0');
		Nokia5110_OutChar(decData%10 + '0');
	} else if (decData < 1000) {
		Nokia5110_OutString("  ");
		Nokia5110_OutChar(decData/100 + '0');
		decData %= 100;
		Nokia5110_OutChar(decData/10 + '0');
		Nokia5110_OutChar(decData%10 + '0');
	} else if (decData < 10000) {
		Nokia5110_OutChar(' ');
		Nokia5110_OutChar(decData/1000 + '0');
		decData %= 1000;
		Nokia5110_OutChar(decData/100 + '0');
		decData %= 100;
		Nokia5110_OutChar(decData/10 + '0');
		Nokia5110_OutChar(decData%10 + '0');
	} else {
		Nokia5110_OutChar(decData/10000 + '0');
		decData %= 10000;
		Nokia5110_OutChar(decData/1000 + '0');
		decData %= 1000;
		Nokia5110_OutChar(decData/100 + '0');
		decData %= 100;
		Nokia5110_OutChar(decData/10 + '0');
		Nokia5110_OutChar(decData%10 + '0');
	}
}

void Nokia5110_SetCursor(uint8_t newX, uint8_t newY) {
	if ((newX >= LCD_MAX_COLS) || (newY >= LCD_MAX_ROWS)) {
		return;	// Do nothing
	}
	LCD_SetXAddressOfRAM(newX * 7);
	LCD_SetYAddressOfRAM(newY);
}

void Nokia5110_ClearScreen(void) {
	for (int i = 0; i < (LCD_WIDTH * LCD_HEIGHT/8); i++) {
		LCD_SendFrame(DATA, 0x00);
	}
	Nokia5110_SetCursor(0, 0);
}

void Nokia5110_DrawFullImage(const char *imagePtr) {

}

// Hardware functions:
void LCD_ConfigurePort(void) {
	LCD_DDR |= ((1 << LCD_DC) | (1 << LCD_RST) | (1 << LCD_CS));
	LCD_PORT |= (1 << LCD_CS);
}

void LCD_SendFrame(enum WriteType frameType, uint8_t dataFrame) {
	if (frameType == DATA) {
		LCD_PORT |= (1 << LCD_DC);
	} else if (frameType == COMMAND) {
		LCD_PORT &= ~(1 << LCD_DC);
	}

	LCD_PORT &= ~(1 << LCD_CS);
	SPI_MasterTransmit(dataFrame);
	LCD_PORT |= (1 << LCD_CS);
}

void LCD_Initialisation(void) {
	LCD_PORT &= ~(1 << LCD_RST);
	// May need to include a delay of a couple of clock pulses
	_delay_ms(100);
	LCD_PORT |= (1 << LCD_RST);
}

void LCD_FunctionSet(enum PowerMode pm, enum AddressType ad, 
					 enum InstructionSetType it) {
	uint8_t functionSetFrame = 0x00;

	if (pm == CHIP_ACTIVE) {
		functionSetFrame &= ~(1 << DB2);
	} else if (pm == CHIP_POWER_DOWN) {
		functionSetFrame |= (1 << DB2);
	}

	if (ad == HORIZONTAL_ADDRESSING) {
		functionSetFrame &= ~(1 << DB1);
	} else if (ad == VERTICAL_ADDRESSING){
		functionSetFrame |= (1 << DB1);
	}

	if (it == BASIC_INSTRUCTION_SET) {
		functionSetFrame &= ~(1 << DB0);
	} else if (it == EXTENDED_INSTRUCTION_SET){
		functionSetFrame |= (1 << DB0);
	}

	functionSetFrame |= (1 << DB5);

	LCD_SendFrame(COMMAND, functionSetFrame);
};

void LCD_DisplayControl(enum DisplayConfiguration dc) {
	uint8_t displayControlFrame = 0x00;

	switch (dc) {
		case DISPLAY_BLANK:
			displayControlFrame &= ~((1 << DB2) | (1 << DB0));
			break;
		case NORMAL_MODE:
			displayControlFrame |= (1 << DB2);
			displayControlFrame &= ~(1 << DB0);
			break;
		case ALL_DISPLAY_SEGMENTS_ON:
			displayControlFrame |= (1 << DB0);
			displayControlFrame &= ~(1 << DB2);
			break;
		case INVERSE_VIDEO_MODE:
			displayControlFrame |= ((1 << DB2) | (1 << DB0));
			break;
	}

	displayControlFrame |= (1 << DB3);

	LCD_SendFrame(COMMAND, displayControlFrame);
};

void LCD_SetYAddressOfRAM(uint8_t bank) {
	if (bank > 5) return;

	uint8_t yAddress = bank;
	yAddress |= (1 << DB6);

	LCD_SendFrame(COMMAND, yAddress);
}

void LCD_SetXAddressOfRAM(uint8_t bank) {
	if (bank > 83) return;

	uint8_t xAddress = bank;
	xAddress |= (1 << DB7);

	LCD_SendFrame(COMMAND, xAddress);
}

void LCD_TemperatureControl(enum TemperatureCoefficient tc) {
	uint8_t temperatureControlFrame = 0x00;

	switch (tc) {
		case TEMPERATURE_COEFF_0:
			temperatureControlFrame &= ~((1 << DB1) | (1 << DB0));
			break;
		case TEMPERATURE_COEFF_1:
			temperatureControlFrame |= (1 << DB0);
			temperatureControlFrame &= ~(1 << DB1);
			break;
		case TEMPERATURE_COEFF_2:
			temperatureControlFrame |= (1 << DB1);
			temperatureControlFrame &= ~(1 << DB0);
			break;
		case TEMPERATURE_COEFF_3:
			temperatureControlFrame |= ((1 << DB1) | (1 << DB0));
			break;
	}

	temperatureControlFrame |= (1 << DB2);

	LCD_SendFrame(COMMAND, temperatureControlFrame);
};

void LCD_BiasValue(void) {
	LCD_SendFrame(COMMAND, 0x13); // LCD bias mode 1:48. Can try 0x14 too
}

void LCD_SetVopValue(uint8_t contrast) {
	LCD_SendFrame(COMMAND, contrast);
}
