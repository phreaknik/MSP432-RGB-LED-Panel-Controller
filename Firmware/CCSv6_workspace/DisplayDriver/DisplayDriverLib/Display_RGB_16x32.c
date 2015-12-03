/*
 * Display_RGB_16x32.c
 *
 *  Created on: Apr 16, 2015
 *      Author: phreaknik
 *      Description:
 *      	The Display_RGB_16x32 code serves as a driver for the TI MSP432 to drive the SparkFun/Adafruit 16x32
 *      	RGB LED display. This driver uses timer interrupts and eUSCI modules to offload most of the processing
 *      	requirements from the host CPU. Some MSP432 peripherals will be in use by this driver and will be
 *      	unavailable for other processes. Below is a list of the used peripherals.
 *
 *      	Peripherals Used:
 *      		- eUSCIA0
 *      		- eUSCIA1
 *      		- eUSCIA2
 *      		- eUSCIB0
 *      		- eUSCIB1
 *      		- eUSCIB2
 *      		- TimerA0
 *      		- TimerA1
 */

#include "../DisplayDriverLib/Display_RGB_16x32.h"

#include "msp.h"


// Global variables
DISP__imgBuf DISP__TXBuff;				// TXBuffer to hold screen data to be sent to display
uint8_t row = 0;						// Display row index
int8_t chunk_shift = 24;				// Shift amount for current chunk
int8_t color_page = 0;					// Color page index
DISP__SM_states SM_state = SM_reset;	// State machine initiated to idle
DISP__SM_flag_types SM_flags;			// State machine flags
DISP__SM_flag_types SM_flag_reset;		// Empty struct used to reset flags

void DISP__stateMachine()
{
	switch(SM_state) {
	case SM_idle:
		if(SM_flags.reset)
		{
			SM_state = SM_reset;
			break;
		}
		if(SM_flags.send_frame)
		{
			SM_state = SM_sending_frame;
			DISP__sendFrame();
		}
		break;

	case SM_start:
		break;

	case SM_init:
		break;

	case SM_reset:
		if(SM_flags.reset)
		{
			DISP__stopDisplayDriver();		// Stop frame timer
			DISP__resetDisplayDriver();		// Clear TX buffers
			SM_flags = SM_flag_reset;	// Reset all SM_flags
		}
		if(SM_flags.send_frame)
		{
			SM_state = SM_sending_frame;
			DISP__sendFrame();
		}
		break;

	case SM_sending_frame:

	    // Set overrun flag if the last frame is still sending
	    if(SM_flags.send_frame)
		{
	    	SM_flags.frame_overrun = 1;
		}
	    else SM_flags.frame_overrun = 0;	// Clear flag if not

		if(SM_flags.reset)
		{
			SM_state = SM_reset;
			break;
		}
		if(SM_flags.frame_sent)
		{
			SM_flags.frame_sent = 0;
			SM_state = SM_idle;
		}
		break;

	default:
		while(1);	// Catch unknown state
	}
}

void DISP__initDisplayDriver()							// Configure GPIOs, timers and eUSCI modules
{
	// Make sure these are all positive to prevent div/0 in code
	assert(DISP__NUM_ROWS);
	assert(DISP__NUM_COLUMNS);
	assert(DISP__COLOR_DEPTH);

	// *** CLOCK CONFIG *** //
    // Enable and configure SMCLK & ACLK
    PJSEL0 |= BIT0 | BIT1;                    		// set LFXT pin as second function
    CSKEY = 0x695A;                        			// Unlock CS module for register access
    CSCTL1 |= SELS__DCOCLK | DIVS_1;				// Set SMCLK to DCO/2 = 24 MHz
    CSCTL2 |= LFXT_EN;                         		// LFXT on

    // Loop until XT1, XT2 & DCO fault flag is cleared
    do
    {
    	// Clear XT2,XT1,DCO fault flags
    	CSCLRIFG |= CLR_DCORIFG | CLR_HFXTIFG | CLR_LFXTIFG;
    	SYSCTL_NMI_CTLSTAT &= ~ SYSCTL_NMI_CTLSTAT_CS_SRC;
    } while (SYSCTL_NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_FLG);	// Test oscillator fault flag

    CSCTL1 |= SELA__LFXTCLK|DIVA_0;					// Select ACLK as LFXTCLK
    CSKEY = 0;										// Lock CS module from unintended accesses
    // *** END CLOCK CONFIG *** //

    // Configure Timer_A0
    NVIC_ISER0 = 1 << ((INT_TA0_0 - 16) & 31);		// Enable timer interrupt in NVIC
    TA0CCTL0 &= ~CCIFG;						  		// Clear interrupt flag
    TA0CCTL0 = CCIE;                          		// TACCR0 interrupt enabled
    TA0CTL = TASSEL__SMCLK | TACLR | ID_0;     		// ACLK, clear timer and divide ACLK by 1
    DISP__setBrightness(0x4000);			  		// Set timer period (AKA brightness)

    // Configure Timer_A1
    NVIC_ISER0 = 1 << ((INT_TA1_0 - 16) & 31);		// Enable timer interrupt in NVIC
    NVIC_IPR2 |= 0x00200000;						// DEMOTE TO LEVEL 1 PRIORITY
    TA1CCTL0 &= ~CCIFG;						  		// Clear interrupt flag
    TA1CCTL0 = CCIE;                          		// TACCR0 interrupt enabled
    TA1CTL = TASSEL__ACLK | TACLR | ID_0;     		// ACLK, clear timer and divide ACLK by 1
    DISP__setFrameRate(60);					  		// Set timer period for 60 fps


	// Configure control pins
	CtrlDIR |= (A+B+C+D+OE+CLK+STB);
	CtrlPort &= ~(A+B+C+D+OE+CLK+STB);				// Clear control pins
	CtrlPort |= OE;

	// Put all eUSCI state machines in reset
    UCA0CTLW0 |= UCSWRST;                     		// **Put state machine in reset**
    UCA1CTLW0 |= UCSWRST;                     		// **Put state machine in reset**
    UCA2CTLW0 |= UCSWRST;                     		// **Put state machine in reset**
    UCB0CTLW0 |= UCSWRST;                     		// **Put state machine in reset**
    UCB1CTLW0 |= UCSWRST;                     		// **Put state machine in reset**
    UCB2CTLW0 |= UCSWRST;                     		// **Put state machine in reset**

    // Configure eUSCIA0 SPI Interface as master
	P1DIR &= ~(BIT1|BIT3);
    P1SEL0 |= BIT1|BIT3;

    NVIC_ISER0 = 1 << ((INT_EUSCIA0 - 16) & 31); 	// Enable eUSCIA0 interrupt in NVIC module

    UCA0CTLW0 = UCMST|UCSYNC|UCCKPL|
    			UCMSB|UCSSEL__SMCLK;				// 3-pin, 8-bit SPI master, clock polarity high, MSB
    UCA0BR0 = 0x01;			                    	// fBitClock = fBRCLK/(UCBRx+1).
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x0000;                         	// No modulation

    // Configure eUSCIA1 SPI Interface as slave
    P2DIR &= ~(BIT1|BIT2);
    P2SEL0 |= BIT1|BIT2;             		  		// set pin as CLK
    UCA1CTLW0 = UCSYNC|UCCKPH|UCMSB;   	      		// 3-pin, 8-bit SPI slave, Clock polarity high, MSB
    UCA1MCTLW = 0;                            		// No modulation

    // Configure eUSCIA2 SPI Interface as slave
    P3DIR &= ~(BIT1|BIT2);
    P3SEL0 |= BIT1|BIT2;             		  		// set pin as CLK
    UCA2CTLW0 = UCSYNC|UCCKPH|UCMSB;         		// 3-pin, 8-bit SPI slave, Clock polarity high, MSB
    UCA2MCTLW = 0;                            		// No modulation

    // Configure eUSCIB0 SPI Interface as slave
    P1DIR &= ~(BIT5|BIT7);
    P1SEL0 |= BIT5|BIT7;             		  		// set pin as CLK
    UCB0CTLW0 = UCSYNC|UCCKPL|UCCKPH|UCMSB;         // 3-pin, 8-bit SPI slave, Clock polarity high, MSB

    // Configure eUSCIB1 SPI Interface as slave
    P6DIR &= ~(BIT3|BIT5);
    P6SEL0 |= BIT3|BIT5;             		  		// set pin as CLK
    UCB1CTLW0 = UCSYNC|UCCKPL|UCCKPH|UCMSB;         // 3-pin, 8-bit SPI slave, Clock polarity high, MSB

    // Configure eUSCIB2 SPI Interface as slave
    P3DIR &= ~(BIT5|BIT7);
    P3SEL0 |= BIT5|BIT7;             		  		// set pin as CLK
    UCB2CTLW0 = UCSYNC|UCCKPL|UCCKPH|UCMSB;         // 3-pin, 8-bit SPI slave, Clock polarity high, MSB

    // Enable all SPI state machines
    UCA0CTLW0 &= ~UCSWRST;                    		// **Initialize USCIA0 state machine**
    UCA1CTLW0 &= ~UCSWRST;                    		// **Initialize USCIA1 state machine**
    UCA2CTLW0 &= ~UCSWRST;                    		// **Initialize USCIA2 state machine**
    UCB0CTLW0 &= ~UCSWRST;                    		// **Initialize USCIB0 state machine**
    UCB1CTLW0 &= ~UCSWRST;                    		// **Initialize USCIB1 state machine**
    UCB2CTLW0 &= ~UCSWRST;                    		// **Initialize USCIB2 state machine**

    DISP__resetDisplayDriver();

    // Enable SPI interrupt. Only USCA1 interrupt is needed, since its ISR will reload all 6 USCI modules
    __enable_interrupt();
}

void DISP__startDisplayDriver()
{
	TA1CTL |= MC__UP;	// Start frame timer
}

void DISP__stopDisplayDriver()
{
	TA1CTL &= ~MC_M;	// Disable frame timer
}

void DISP__pauseDisplayDriver()
{
	UCA0IE &= ~UCTXIE;	// Disable eUSCI interrupts to prevent data transmission
}

void DISP__resetDisplayDriver()
{
	// Reset color page index
	color_page = 0;

	// Clear buffers
	UCB2TXBUF = 0x00;
	UCB1TXBUF = 0x00;
	UCB0TXBUF = 0x00;
	UCA2TXBUF = 0x00;
	UCA1TXBUF = 0x00;
	UCA0TXBUF = 0x00;

	// Wait for first TX to finish, and trigger one more TX to purge all buffers
	while(	!(UCA0IFG&UCTXIFG) );
	UCA0TXBUF = 0x00;
}

void DISP__flashRow()
{
	// Latch and flash row
	CtrlPort |= STB;		// Latch data
	CtrlPort &= ~OE;		// Enable LED output

	TA0CTL |= MC__UP;	// Start timer A0 in up-mode
}

void DISP__setBrightness(uint16_t brightness)
{
	assert(DISP__COLOR_DEPTH);	// Prevent div/0
	uint16_t max_brightness = 0x4000 / DISP__COLOR_DEPTH;

	// Set flash timer period
	brightness = brightness / DISP__COLOR_DEPTH;		// Scale LED pulse width down to match color depth
	if(brightness < 1) brightness = 1;	// Minimum brightness
	else if (brightness > max_brightness) brightness = max_brightness;	// Max brightess

	// Set timer period accordingly
	TA0CCR0 = brightness;
}

int32_t DISP__setFrameRate(int32_t frame_rate)
{
	uint16_t min_period = 0x0001;
	uint16_t period = 0x0000;

	// Make sure frame rate isnt negative
	if ((frame_rate > 0) && (DISP__COLOR_DEPTH > 0))
	{
		period = 0x8000 / frame_rate;	// Calculate timer period from frame rate
	}
	else return -1;	// Return error

	if(period > min_period)	TA1CCR0 = period;
	else TA1CCR0 = min_period;

	return 0;
}

void DISP__calculateRowAddress()
{
	// Recalculate row address
	CtrlPort = (row & 1) ? (CtrlPort | A) : (CtrlPort & ~A);
	CtrlPort = (row & 2) ? (CtrlPort | B) : (CtrlPort & ~B);
	CtrlPort = (row & 4) ? (CtrlPort | C) : (CtrlPort & ~C);
	CtrlPort = (row & 8) ? (CtrlPort | D) : (CtrlPort & ~D);
}

void DISP__sendFrame()
{
	SM_flags.send_frame = 0;		// Clear flag
	UCA0IE |= UCTXIE;       		// Enable USCI_A0 TX interrupt
}

void DISP__eUSCIA0ISR(void)
{
	// Wait for all USCI modules to be ready
	while(	!(UCA0IFG&UCTXIFG) | !(UCA1IFG&UCTXIFG) |
			!(UCA2IFG&UCTXIFG) | !(UCB0IFG&UCTXIFG) |
			!(UCB1IFG&UCTXIFG) | !(UCB2IFG&UCTXIFG) );

	// Handle row & chunk counters and flash LEDs
	if(chunk_shift < 0)
	{
		// Pause eUSCI modules while latching and flashing
	    DISP__pauseDisplayDriver();

		row ++;										// Increment row counter
		chunk_shift = 24;							// Reset chunk_shift

		// Start flash timer
		DISP__flashRow();

	    return;											// Exit interrupt. Timer ISR will relaunch next transfer
	}

	// Send new data
	UCB2TXBUF = (uint8_t) ((DISP__TXBuff.blueRow[row+8][color_page] 	>> chunk_shift) & 0x000000FF);		// Transmit next byte
	UCB1TXBUF = (uint8_t) ((DISP__TXBuff.greenRow[row+8][color_page] 	>> chunk_shift) & 0x000000FF);		// Transmit next byte
	UCB0TXBUF = (uint8_t) ((DISP__TXBuff.redRow[row+8][color_page] 	>> chunk_shift) & 0x000000FF);		// Transmit next byte
	UCA2TXBUF = (uint8_t) ((DISP__TXBuff.blueRow[row][color_page]	 	>> chunk_shift) & 0x000000FF);		// Transmit next byte
	UCA1TXBUF = (uint8_t) ((DISP__TXBuff.greenRow[row][color_page] 	>> chunk_shift) & 0x000000FF);		// Transmit next byte
	UCA0TXBUF = (uint8_t) ((DISP__TXBuff.redRow[row][color_page] 		>> chunk_shift) & 0x000000FF);		// Transmit next byte

	// Decrement chunk_shift by 8 columns
	chunk_shift -= 8;

	return;
}

void DISP__oneShotFlashTimerISR(void)
{
	// Handle timer interrupt
    TA0CTL &= ~MC_M;		// Stop timer
    TA0CTL |= TACLR;		// Reset timer
    TA0CCTL0  &= ~CCIFG;	// Clear interrupt flag

	// Disable LED output and reset latch
	CtrlPort |= OE;			// Disable LED output
	CtrlPort &= ~STB;		// Clear latch

	// Reset row and stop transmission at end of frame
	if(row > 7)
	{
		row = 0;
		DISP__calculateRowAddress();

		// Stop transmission if we have transmitted all color pages in the frame
		if(color_page >= (DISP__COLOR_DEPTH - 1))
		{
			color_page = 0;						// Reset color page
			SM_flags.frame_sent = 1;
			return;
		}
		else color_page++;
	}

	// Set next row address
	DISP__calculateRowAddress();

	// Restart eUSCI modules
	DISP__sendFrame();

	return;
}

void DISP__frameTimerISR(void)
{
	// Handle timer interrupt
    TA1CCTL0  &= ~CCIFG;	// Clear interrupt flag

	// Set flag to send next frame
    SM_flags.send_frame = 1;

    return;
}


void DISP__drawLine()
{

}

void DISP__drawRect()
{

}

void DISP__drawCircle()
{

}

void DISP__drawChar(DISP__imgBuf *buf, const char alphNum, const DISP__PDMcolor *textColor)
{
	uint32_t letter[DISP__NUM_ROWS];
	letter[0]	=	0b00100000000000000000000000000000;
	letter[1]	=	0b01010000000000000000000000000000;
	letter[2]	=	0b10001000000000000000000000000000;
	letter[3]	=	0b10001000000000000000000000000000;
	letter[4]	=	0b11111000000000000000000000000000;
	letter[5]	=	0b10001000000000000000000000000000;
	letter[6]	=	0b10001000000000000000000000000000;
	letter[7]	=	0b00000000000000000000000000000000;
	letter[8]	=	0b00000000000000000000000000000000;
	letter[9]	=	0b00000000000000000000000000000000;
	letter[10]	=	0b00000000000000000000000000000000;
	letter[11]	=	0b00000000000000000000000000000000;
	letter[12]	=	0b00000000000000000000000000000000;
	letter[13]	=	0b00000000000000000000000000000000;
	letter[14]	=	0b00000000000000000000000000000000;
	letter[15]	=	0b00000000000000000000000000000000;

	int R, P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		for(R = 0; R < DISP__NUM_ROWS; R++)
		{
			if(textColor->red & BIT(P)) buf->redRow[R][P] |= letter[R];
			else buf->redRow[R][P] &= ~letter[R];

			if(textColor->green & BIT(P)) buf->greenRow[R][P] |= letter[R];
			else buf->greenRow[R][P] &= ~letter[R];

			if(textColor->blue & BIT(P)) buf->blueRow[R][P] |= letter[R];
			else buf->blueRow[R][P] &= ~letter[R];
		}
	}

}

void DISP__drawScreen(const DISP__imgBuf *buf)
{
	DISP__TXBuff = *buf;	//FIXME This should be done with DMA
}

void DISP__fillScreen(DISP__imgBuf *buf, const DISP__PDMcolor *PDMColor)
{
	int R, P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		for(R = 0; R < DISP__NUM_ROWS; R++)
		{
			buf->redRow[R][P] = 		(PDMColor->red & BIT(P)) 	? 	0xFFFFFFFF : 0x00000000;
			buf->greenRow[R][P] = 		(PDMColor->green & BIT(P)) 	? 	0xFFFFFFFF : 0x00000000;
			buf->blueRow[R][P] = 		(PDMColor->blue & BIT(P)) 	?	0xFFFFFFFF : 0x00000000;
		}
	}
}

void DISP__setColorPDM(DISP__PDMcolor *PDMColor, const int32_t red, const int32_t green, const int32_t blue)
{
	// First, clear colors
	PDMColor->red = 0;
	PDMColor->green = 0;
	PDMColor->blue = 0;

	// If all colors are zero (pixel is blank), then return.
	// This prevents wasting CPU cycles on blank pixels
	if(!(red || green || blue)) return;

	int32_t i = 0,
			redError = 0,
			greenError = 0,
			blueError = 0;

	// Perform PDM modulation
	for(i=0; i < DISP__COLOR_DEPTH; i++)
	{
		redError += red;
		greenError += green;
		blueError += blue;

		if(redError >= DISP__COLOR_DEPTH)
		{
			PDMColor->red |= BIT(i);
			redError -= DISP__COLOR_DEPTH;
		}

		if(greenError >= DISP__COLOR_DEPTH)
		{
			PDMColor->green |= BIT(i);
			greenError -= DISP__COLOR_DEPTH;
		}

		if(blueError >= DISP__COLOR_DEPTH)
		{
			PDMColor->blue |= BIT(i);
			blueError -= DISP__COLOR_DEPTH;
		}
	}
}
