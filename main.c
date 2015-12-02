#include "msp.h"
#include <stdint.h>
#include "DisplayDriver/Display_RGB_16x32.h"

// Function declarations
void systemInit();

int main(void) {
	// Variables
	DISP__imgBuf screenBuff;				// Buffer to build image and send to screen
	DISP__PDMcolor bgColor;
	DISP__PDMcolor textColor;
	int i = 0;

	// Initialize MCU
	systemInit();

    // Initialize display driver
	DISP__initDisplayDriver();					// Configure pins to drive display
    DISP__startDisplayDriver();
    DISP__setFrameRate(60);
    DISP__setBrightness(0x4000);

	// Set background color
	DISP__setColorPDM(&bgColor, 0, 2, 4);
    DISP__fillScreen(&screenBuff, &bgColor);

    // Draw character
	DISP__setColorPDM(&textColor, 4, 0, 0);
	DISP__drawChar(&screenBuff, "A", &textColor);

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
