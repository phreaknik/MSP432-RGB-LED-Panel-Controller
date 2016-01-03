#include "msp.h"
#include <stdint.h>

#include "RGB-LED-Panel-Library/RGB_LED_Panel.h"

// Function declarations
void systemInit();

int main(void) {
	// Variables
	DISP__imgBuf screenBuff;
	DISP__PDMcolor drawColor;
	int red, green, blue, x, redStep, greenStep, blueStep, xStep;

	// Initialize MCU
	systemInit();

    // Initialize display driver
	DISP__initDisplayDriver();					// Configure pins to drive display
    DISP__startDisplayDriver();
    DISP__setFrameRate(60);
    DISP__setBrightness(0x4000);

    // Clear screen
    DISP__setColorPDM(&drawColor, 0, 0, 0);
    DISP__fillScreen(&screenBuff, &drawColor);

    x = 0;
    xStep = 1;
    red = 3;
    green = 6;
    blue = 0;
    redStep = 1;
    greenStep = 1;
    blueStep = 1;
    while(1) // Main loop
    {
        // Update state machine
    	DISP__stateMachine();	// Must be called periodically to handle display state machine

    	// Draw next shape
    	x += xStep;
    	red += redStep;
    	green += greenStep;
    	blue += blueStep;

    	DISP__setColorPDM(&drawColor, red, green, blue);
    	DISP__drawRect(&screenBuff, &drawColor, x, 0, 8, 16);

    	// Draw screen
    	DISP__drawScreen(&screenBuff);

    	// Scan position and color
    	if((x >= 24) || (x <= 0)) xStep *= -1;
    	if((red > 8) || (red <= 0)) redStep *= -1;
    	if((green > 8) || (green <= 0)) greenStep *= -1;
    	if((blue > 8) || (blue <= 0)) blueStep *= -1;

        // Go to sleep until next interrupt
        __sleep();
    }
}

// Definitions
void systemInit()
{
	// Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Configure system clock
    CSKEY = 0x695A;                        	// Unlock CS module for register access
    CSCTL0 = 0;                            	// Reset tuning parameters
    CSCTL0 = DCORSEL_5;           			// Set DCO to 48MHz nominal
    /* Select MCLK = DCO, no divider */
    CSCTL1 = CSCTL1 & ~(SELM_M | DIVM_M) | SELM__DCOCLK;
    CSKEY = 0;								// Lock CS module from unintended accesses

    // Wake up on exit from ISR
    SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;
}
