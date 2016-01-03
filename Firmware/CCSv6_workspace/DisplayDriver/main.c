#include "msp.h"
#include <stdint.h>

#include "RGB-LED-Panel-Library/RGB_LED_Panel.h"

// Function declarations
void systemInit();

int main(void) {
	// Variables
	DISP__imgBuf screenBuff;				// Buffer to build image and send to screen
	DISP__PDMcolor bgColor;
	DISP__PDMcolor drawColor;

	// Initialize MCU
	systemInit();

    // Initialize display driver
	DISP__initDisplayDriver();					// Configure pins to drive display
    DISP__startDisplayDriver();
    DISP__setFrameRate(60);
    DISP__setBrightness(0x4000);

	// Set background color
	DISP__setColorPDM(&bgColor, 0, 0, 0);
    DISP__fillScreen(&screenBuff, &bgColor);

    // Draw stuff
	DISP__setColorPDM(&drawColor, 0, 8, 2);
	DISP__drawRect(&screenBuff, &drawColor, 4, 4, 24, 8);
	DISP__setColorPDM(&drawColor, 8, 8, 8);
	DISP__drawRect(&screenBuff, &drawColor, 12, 0, 8, 16);
	DISP__setColorPDM(&drawColor, 0, 0, 8);
	DISP__drawCircle(&screenBuff, &drawColor, 7, 7, 7);
	DISP__setColorPDM(&drawColor, 8, 0, 0);
	DISP__drawLine(&screenBuff, &drawColor, 0, 13, 31, 13);
	DISP__setColorPDM(&drawColor, 8, 0, 8);
	DISP__drawPixel(&screenBuff, &drawColor, 30, 1);
	DISP__setColorPDM(&drawColor, 8, 8, 0);
	DISP__drawChar(&screenBuff, &drawColor, "Test!", 7, 1, 0, 0);

	// Draw screen
	DISP__drawScreen(&screenBuff);

    while(1) // Main loop
    {
        // Update state machine
    	DISP__stateMachine();	// Must be called periodically to handle display state machine

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
