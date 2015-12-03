/*
 * RGB_Matrix_16x32.c
 *
 *  Created on: Apr 16, 2015
 *      Author: phreaknik
 */

#include "msp.h"
#include "../Drivers/RGB_Matrix_16x32.h"


extern void initDisplayDriver()	// Configure pins to drive display
{
    CtrlDIR |= A + B + C + D + CLK + STB + OE;	// Set control pins as outputs
    DataDIR_R0R1 |= R0 + R1;				// Set data pins as outputs
    DataDIR_G0 |= G0;
    DataDIR_G1 |= G1;
    DataDIR_B0B1 |= B0 + B1;

    CtrlPort = 0x00;						// Clear outputs
    DataPort_R0R1 = 0x00;					// Clear outputs
    DataPort_G1 = 0x00;						// Clear outputs
    DataPort_G0 = 0x00;						// Clear outputs
	DataPort_B0B1 = 0x00;					// Clear outputs
}

void refreshDisplay(displayRGB16x32 *display) // Load image to display
{
	uint32_t delay = 10;				// Define clock pulse delay
	//CtrlPort &= ~(A+B+C+D+OE+CLK+STB);	// Clear control pins
	//CtrlPort |= OE;

	// Transmit image to display
	uint8_t i;
	int8_t j;
	for(i=0;i<8;i++)	// Transmit each row
	{
		// Set row address
		CtrlPort = (i & 1) ? (CtrlPort | A) : (CtrlPort & ~A);
		CtrlPort = (i & 2) ? (CtrlPort | B) : (CtrlPort & ~B);
		CtrlPort = (i & 4) ? (CtrlPort | C) : (CtrlPort & ~C);
		CtrlPort = (i & 8) ? (CtrlPort | D) : (CtrlPort & ~D);

		uint32_t bitmask = BIT(31);
		for(j=31;j>=0;j--)	// Transmit each pixel in each row
		{
			if(display->redRow[i] & bitmask) DataPort_R0R1 |= R0;
			else DataPort_R0R1 &= ~R0;
			if(display->redRow[i+8] & bitmask) DataPort_R0R1 |= R1;
			else DataPort_R0R1 &= ~R1;

			if(display->greenRow[i] & bitmask) DataPort_G0 |= G0;
			else DataPort_G0 &= ~G0;
			if(display->greenRow[i+8] & bitmask) DataPort_G1 |= G1;
			else DataPort_G1 &= ~G1;

			if(display->blueRow[i] & bitmask) DataPort_B0B1 |= B0;
			else DataPort_B0B1 &= ~B0;
			if(display->blueRow[i+8] & bitmask) DataPort_B0B1 |= B1;
			else DataPort_B0B1 &= ~B1;

			// Clock pulse
			delayCycles(delay);
			CtrlPort &= ~CLK;
			delayCycles(delay);
			CtrlPort |= CLK;

			// Update bitmask
			bitmask = bitmask >> 1;
		}
		// Latch and flash row
		CtrlPort |= STB;		// Latch data
		delayCycles(delay);
		CtrlPort &= ~STB;		// Clear latch
		delayCycles(delay);
		CtrlPort &= ~OE;		// Enable LED output
		delayCycles(500);
		CtrlPort |= OE;			// Disable LED output
	}
}

void delayCycles(uint32_t delay)
{
	uint32_t i;
	for(i=0; i<delay; i++);
}


