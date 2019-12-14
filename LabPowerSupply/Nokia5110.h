/******************************************************************************
 *
 *	@file: Nokia5110.h
 *  @brief: Nokia 5110 LCD driver for the ATmega328P
 *  @compiler: AVR-GCC
 *  @version: 0.1
 *  @author: Luis A Ayora
 *  @date: 06-12-2019
 *  @todo: Implement functions on c file
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
 
/* Adafruit Nokia 5110 LCD Pins:
 * ----------------------------
 * Pin 1: GND - Ground (0V)
 * Pin 2: VCC - Power  (3.3V)
 * Pin 3: CLK - SPI Clock 
 * Pin 4: DIN - SPI Data in
 * Pin 5: D/C - Data/Command
 * Pin 6: CSE - Chip Select
 * Pin 7: RST - Reset 
 * Pin 8: LED - Backlight (NC)
 * ----------------------------
 */

// Defines the back light contrast of the screen
// Modify this value for your particular LCD
// #define CONTRAST 0xB1

static const char ASCII[][5] = {
	 {0x00, 0x00, 0x00, 0x00, 0x00} // 20
	,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
	,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
	,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
	,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
	,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
	,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
	,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
	,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
	,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
	,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
	,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
	,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
	,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
	,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
	,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
	,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
	,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
	,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
	,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
	,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
	,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
	,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
	,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
	,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
	,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
	,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
	,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
	,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
	,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
	,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
	,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
	,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
	,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
	,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
	,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
	,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
	,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
	,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
	,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
	,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
	,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
	,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
	,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
	,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
	,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
	,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
	,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
	,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
	,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
	,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
	,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
	,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
	,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
	,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
	,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
	,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
	,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
	,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
	,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
	,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c '\'
	,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
	,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
	,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
	,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
	,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
	,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
	,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
	,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
	,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
	,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
	,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
	,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
	,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
	,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
	,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
	,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
	,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
	,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
	,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
	,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
	,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
	,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
	,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
	,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
	,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
	,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
	,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
	,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
	,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
	,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
	,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
	,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
	,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
	,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
	//  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
	,{0x1f, 0x24, 0x7c, 0x24, 0x1f} // 7f UT sign
};

#ifndef NOKIA5110_H_
#define NOKIA5110_H_

/*	Nokia5110_Init
 *	@brief: Initialises the Nokia 5110 LCD
 *	
 *	The Nokia 5110 is initialised by following the procedure outlined in the
 *	PCD8544 driver datasheet. Maximum baud clock for the LCD is 4 MHz.
 */
void Nokia5110_Init(void);

/*	Nokia5110_OutChar
 *	@brief: Prints a character to the LCD
 *	@param:	data
 *			Character data to be printed
 *	
 *	Character is printed at the current matrix position and the cursor will be
 *	updated to the position at the end of the print procedure. Allows for one
 *	blank column of pixels on both sides to improve readability.
 *	Assumes that the LCD is in default horizontal addressing mode (V = 0).
 */
void Nokia5110_OutChar(unsigned char charData);

/*	Nokia5110_OutString
 *	@brief: Prints a string of characters to the LCD
 *	@param: charData
 *			Points to the first character of the NULL-terminated string
 *	
 *	The string will automatically wrap, padding spaces will be required as a 
 *	result.
 *	Assumes that the LCD is in default horizontal addressing mode (V = 0).
 */
void Nokia5110_OutString(char *stringPtr);

/*	Nokia5110_OutUnsignedDecimal
 *	@brief: Output a 16-bit unsigned decimal
 *	@param: decData
 *			Unsigned 16-bit decimal to be sent
 *	
 *	The number will be formatted with a fixed size of five right-justified 
 *	digits of output.
 */
void Nokia5110_OutUnsignedDecimal(uint32_t decData);

/*	Nokia5110_SetCursor
 *	@brief: Position the cursor at the desired position on the LCD matrix
 *	@param: newX
 *			Desired X position
 *	@param: newY
 *			Desired Y position
 *	
 *	X position starts at leftmost column of the LCD from 0 to MAX_WIDTH-1.
 *	Y position starts at top row of the LCD from 0 to MAX_HEIGHT-1.
 */
void Nokia5110_SetCursor(uint8_t newX, uint8_t newY);

/*	Nokia5110_ClearScreen
 *	@brief: Clears the LCD to a blank screen
 *	
 *	Clearing is achieved by writing zeros to the entire matrix and setting the
 *	cursor to the (0, 0) position (top-left corner).
 */
void Nokia5110_ClearScreen(void);

/*	Nokia5110_DrawFullImage
 *	@brief: Populate the whole screen by writing a 48x84 bitmap image
 *	@param: imagePtr
 *			Points to the first element of the bitmap
 *	
 *	Assumes that the LCD is in default horizontal addressing mode (V = 0).
 */
void Nokia5110_DrawFullImage(const char *imagePtr);

#endif /* NOKIA5110_H_ */