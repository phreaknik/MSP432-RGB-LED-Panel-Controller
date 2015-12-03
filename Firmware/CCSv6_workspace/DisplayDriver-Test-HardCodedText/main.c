#include "msp.h"
#include <stdint.h>
#include "Drivers/RGB_Matrix_16x32.h"

// Main
//**********************************
int main(void) {
	// Initialize MCU
    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT

    CSKEY = 0x695A;                        	// Unlock CS module for register access
    CSCTL0 = 0;                            	// Reset tuning parameters
    CSCTL0 = DCORSEL_5;           			// Set DCO to 48MHz nominal
    /* Select MCLK = DCO, no divider */
    CSCTL1 = CSCTL1 & ~(SELM_M | DIVM_M) | SELM_3;
    CSKEY = 0;                             	// Lock CS module from unintended accesses

    // Initialize display driver
    initDisplayDriver();						// Configure pins to drive display
    displayRGB16x32 display;				// Create instance of the display

    // Just initialize the array to something for now
    display.redRow[0] = 	0b00000000000000000000000000000000;
    display.redRow[1] = 	0b00000000000000000000000000000000;
    display.redRow[2] = 	0b00000000000000000000000000000000;
    display.redRow[3] = 	0b00000000000000000000000000000000;
    display.redRow[4] = 	0b00000000000000000000000000000000;
    display.redRow[5] = 	0b00000000000000000000000000000000;
    display.redRow[6] = 	0b00000000000000000000000000000000;
    display.redRow[7] = 	0b00000000000000000000000000000000;
    display.redRow[8] = 	0b00000000000000000000000000000000;
    display.redRow[9] = 	0b00000000000000000000000000000000;
    display.redRow[10] = 	0b00000000000000000000000000000000;
    display.redRow[11] = 	0b11111111111111111111111111111111;
    display.redRow[12] = 	0b00000000000000000000000000000000;
    display.redRow[13] = 	0b00000111111111111111111111100000;
    display.redRow[14] = 	0b00000000000000000000000000000000;
    display.redRow[15] = 	0b00000000000000000000000000000000;

    display.greenRow[0] = 	0b00000000000000000000000000000000;
    display.greenRow[1] = 	0b01111111011111110011111110100010;
    display.greenRow[2] = 	0b00001000010000001000010000100010;
    display.greenRow[3] = 	0b00001000010000001000010000100010;
    display.greenRow[4] = 	0b00001000010000001000010000100010;
    display.greenRow[5] = 	0b00001000010011110000010000100010;
    display.greenRow[6] = 	0b01001000010000001000010000100010;
    display.greenRow[7] = 	0b01001000010000001000010000100010;
    display.greenRow[8] = 	0b01001000010000001000010000010100;
    display.greenRow[9] = 	0b00110000011111110011111110001000;
    display.greenRow[10] = 	0b00000000000000000000000000000000;
    display.greenRow[11] = 	0b00000000000000000000000000000000;
    display.greenRow[12] = 	0b00000000000000000000000000000000;
    display.greenRow[13] = 	0b00000000000000000000000000000000;
    display.greenRow[14] = 	0b00000000000000000000000000000000;
    display.greenRow[15] = 	0b00000000000000000000000000000000;

    display.blueRow[0] = 	0b00000000000000000000000000000000;
    display.blueRow[1] = 	0b01111111011111110011111110100010;
    display.blueRow[2] = 	0b00001000010000001000010000100010;
    display.blueRow[3] = 	0b00001000010000001000010000100010;
    display.blueRow[4] = 	0b00001000010000001000010000100010;
    display.blueRow[5] = 	0b00001000010011110000010000100010;
    display.blueRow[6] = 	0b01001000010000001000010000100010;
    display.blueRow[7] = 	0b01001000010000001000010000100010;
    display.blueRow[8] = 	0b01001000010000001000010000010100;
    display.blueRow[9] = 	0b00110000011111110011111110001000;
    display.blueRow[10] = 	0b00000000000000000000000000000000;
    display.blueRow[11] = 	0b11111111111111111111111111111111;
    display.blueRow[12] = 	0b00000000000000000000000000000000;
    display.blueRow[13] = 	0b00000111111111111111111111100000;
    display.blueRow[14] = 	0b00000000000000000000000000000000;
    display.blueRow[15] = 	0b00000000000000000000000000000000;

    while(1)
    {
    	refreshDisplay(&display);

    	delayCycles((uint32_t) 0);
    }
}

