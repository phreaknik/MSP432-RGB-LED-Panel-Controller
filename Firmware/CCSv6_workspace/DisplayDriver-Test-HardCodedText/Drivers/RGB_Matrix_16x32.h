/*
 * RGB_Matrix_16x32.h
 *
 *  Created on: Apr 16, 2015
 *      Author: John
 */

#ifndef RGB_MATRIX_16X32_H_
#define RGB_MATRIX_16X32_H_

// Includes
//**********************************
#include <stdint.h>

// Pin Definitions
//**********************************
#define CtrlDIR		P2DIR
#define CtrlPort	P2OUT
#define DataDIR_R0R1		P1DIR
#define DataDIR_G0			P2DIR
#define DataDIR_B0B1		P3DIR
#define DataDIR_G1			P6DIR
#define DataPort_R0R1		P1OUT
#define DataPort_G0			P2OUT
#define DataPort_B0B1		P3OUT
#define DataPort_G1			P6OUT

#define R0			BIT3
#define G0			BIT2
#define B0			BIT2
#define R1			BIT7
#define G1			BIT5
#define B1			BIT7

#define A			BIT0
#define B			BIT3
#define C			BIT4
#define D			BIT5
#define CLK			BIT1
#define STB			BIT6
#define OE			BIT7

// Declarations
//**********************************
typedef struct displayRGB16x32
{
	uint8_t redBrightness;
	uint8_t greenBrightness;
	uint8_t blueBrightness;

	uint32_t redRow[16];
	uint32_t greenRow[16];
	uint32_t blueRow[16];
} displayRGB16x32;

extern void initDisplayDriver();						// Configure pins to drive display
extern void refreshDisplay(displayRGB16x32 *display);	// Load image to display
extern void delayCycles(uint32_t delay);

#endif /* RGB_MATRIX_16X32_H_ */
