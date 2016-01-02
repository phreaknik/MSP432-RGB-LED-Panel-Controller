/*
 * RGB_LED_Panel.c
 *
 *  Created on: Apr 16, 2015
 *      Author: phreaknik
 *      Description:
 *      	The RGB_LED_Panel code serves as a driver for the TI MSP432 to drive the SparkFun/Adafruit 16x32
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

#include "../RGB-LED-Panel-Library/RGB_LED_Panel.h"
#include "../RGB-LED-Panel-Library/char_map.h"


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


void DISP__drawPixel(DISP__imgBuf *buf, const DISP__PDMcolor *color, int X, int Y)
{
	// Make sure pixel is on screen
	if(X < 0) return;
	if(X >= DISP__NUM_COLUMNS) return;
	if(Y < 0) return;
	if(Y >= DISP__NUM_ROWS) return;

	// Create bar with pixel on X coordinate
	uint32_t bar = BIT(DISP__NUM_COLUMNS - X - 1);

	// Draw to screenBuf
	int P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		if(color->red & BIT(P)) buf->redRow[Y][P] |= bar;
		else buf->redRow[Y][P] &= ~bar;

		if(color->green & BIT(P)) buf->greenRow[Y][P] |= bar;
		else buf->greenRow[Y][P] &= ~bar;

		if(color->blue & BIT(P)) buf->blueRow[Y][P] |= bar;
		else buf->blueRow[Y][P] &= ~bar;
	}
}

void DISP__drawLine(DISP__imgBuf *buf, const DISP__PDMcolor *color, int X0, int Y0, int X1, int Y1)
{
	// Don't draw anything if line is completely off screen
	if(Y0 >= DISP__NUM_ROWS && Y1 >= DISP__NUM_ROWS) return;			// Line is off screen, so nothing to draw
	if(X0 >= DISP__NUM_COLUMNS && X1 >= DISP__NUM_COLUMNS) return;			// Line is off screen, so nothing to draw
	if(Y0 < 0 && Y1 < 0) return;			// Line is off screen, so nothing to draw
	if(X0 < 0 && X1 < 0) return;			// Line is off screen, so nothing to draw

	// Bresenham's Line Algorithm (https://en.wikipedia.org/wiki/Bresenham's_line_algorithm)
	int steep = fabs(Y1 - Y0) > fabs(X1 - X0);
	if(steep)
	{
		// Swap X0 and Y0
		int temp = X0;
		X0 = Y0;
		Y0 = temp;

		// Swap X1 and Y1
		temp = X1;
		X1 = Y1;
		Y1 = temp;
	}

	if(X0 > X1)
	{
		// Swap X0 and X1
		int temp = X0;
		X0 = X1;
		X1 = temp;

		// Swap Y0 and Y1
		temp = Y1;
		Y1 = Y0;
		Y0 = temp;
	}

	int dx, dy, err, ystep;
	dx = X1 - X0;
	dy = fabs(Y1 - Y0);
	err = dx/2;
	ystep = (Y0 < Y1) ? 1 : -1;

	// Draw to screenBuf
	if(steep)
	{
		int R, P, oldR;
		uint32_t bar = 0;
		oldR = X0;
		for(; X0 <= X1; X0++)
		{
			bar |= BIT(DISP__NUM_COLUMNS - Y0 - 1);
			err -= dy;
			if((err < 0) || (X0 == X1))
			{
				for(R = oldR; R <= X0; R++)
				{
					if((R >= 0) && (R < DISP__NUM_ROWS))
					{
						for(P = 0; P < DISP__COLOR_DEPTH; P++)
						{
							if(color->red & BIT(P)) buf->redRow[R][P] |= bar;
							else buf->redRow[R][P] &= ~bar;

							if(color->green & BIT(P)) buf->greenRow[R][P] |= bar;
							else buf->greenRow[R][P] &= ~bar;

							if(color->blue & BIT(P)) buf->blueRow[R][P] |= bar;
							else buf->blueRow[R][P] &= ~bar;
						}
					}
				}
				Y0 += ystep;
				err += dx;
				bar = 0;
				oldR = X0;
			}
		}
	}
	else
	{
		int R, P;
		uint32_t bar = 0;
		for(; X0 <= X1; X0++)
		{
			bar |= BIT(DISP__NUM_COLUMNS - X0 - 1);
			err -= dy;
			if((err < 0) || (X0 == X1))
			{
				R = Y0;
				if((R >= 0) && (R < DISP__NUM_ROWS))
				{
					for(P = 0; P < DISP__COLOR_DEPTH; P++)
					{
						if(color->red & BIT(P)) buf->redRow[R][P] |= bar;
						else buf->redRow[R][P] &= ~bar;

						if(color->green & BIT(P)) buf->greenRow[R][P] |= bar;
						else buf->greenRow[R][P] &= ~bar;

						if(color->blue & BIT(P)) buf->blueRow[R][P] |= bar;
						else buf->blueRow[R][P] &= ~bar;
					}
				}
				Y0 += ystep;
				err += dx;
				bar = 0;
			}
		}
	}

}

void DISP__drawRect(DISP__imgBuf *buf, const DISP__PDMcolor *color, int X, int Y, int width, int height)
{
	// Limit dimensions
	if(Y > (DISP__NUM_ROWS - 1) || (Y + height) < 0) return;			// Rectangle is off screen, so nothing to draw
	if(X > (DISP__NUM_COLUMNS - 1) || (X + width) < 0) return;			// Rectangle is off screen, so nothing to draw
	if(Y < 0)
	{
		height += Y;			// Reduce height to simulate drawing off screen
		Y = 0;					// Limit Y
	}
	if(X < 0)
	{
		width += X;				// Reduce width to simulate drawing off screen
		X = 0;					// Limit X
	}
	int Y_lim = Y + height - 1;
	if(Y_lim >= DISP__NUM_ROWS) Y_lim = DISP__NUM_ROWS - 1;	// Limit height
	int X_lim = DISP__NUM_COLUMNS - X - width;
	if(X_lim < 0) X_lim = 0;	// Limit width

	// Create bar to build rectangle from
	uint32_t top, sides;
	top = 0xFFFFFFFF << (DISP__NUM_COLUMNS - width - 1);
	top = top >> X;
	sides = BIT(DISP__NUM_COLUMNS - X - 1) + BIT(DISP__NUM_COLUMNS - X - width - 1);

	// Build rectangle from bar
	int R, P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		// Draw top & bottom
		if(color->red & BIT(P))
		{
			buf->redRow[Y][P] |= top;
			buf->redRow[Y_lim][P] |= top;
		}
		else
		{
			buf->redRow[Y][P] &= ~top;
			buf->redRow[Y_lim][P] &= ~top;
		}

		if(color->green & BIT(P))
		{
			buf->greenRow[Y][P] |= top;
			buf->greenRow[Y_lim][P] |= top;
		}
		else
		{
			buf->greenRow[Y][P] &= ~top;
			buf->greenRow[Y_lim][P] &= ~top;
		}

		if(color->blue & BIT(P))
		{
			buf->blueRow[Y][P] |= top;
			buf->blueRow[Y_lim][P] |= top;
		}
		else
		{
			buf->blueRow[Y][P] &= ~top;
			buf->blueRow[Y_lim][P] &= ~top;
		}

		// Draw sides
		for(R = Y + 1; R < Y_lim; R++)
		{
			if(color->red & BIT(P)) buf->redRow[R][P] |= sides;
			else buf->redRow[R][P] &= ~sides;

			if(color->green & BIT(P)) buf->greenRow[R][P] |= sides;
			else buf->greenRow[R][P] &= ~sides;

			if(color->blue & BIT(P)) buf->blueRow[R][P] |= sides;
			else buf->blueRow[R][P] &= ~sides;
		}
	}
}

void DISP__fillRect(DISP__imgBuf *buf, const DISP__PDMcolor *color, int X, int Y, int width, int height)
{
	// Limit dimensions
	if(Y > (DISP__NUM_ROWS - 1) || (Y + height) < 0) return;			// Rectangle is off screen, so nothing to draw
	if(X > (DISP__NUM_COLUMNS - 1) || (X + width) < 0) return;			// Rectangle is off screen, so nothing to draw
	if(Y < 0)
	{
		height += Y;			// Reduce height to simulate drawing off screen
		Y = 0;					// Limit Y
	}
	if(X < 0)
	{
		width += X;				// Reduce width to simulate drawing off screen
		X = 0;					// Limit X
	}
	int Y_lim = Y + height - 1;
	if(Y_lim >= DISP__NUM_ROWS) Y_lim = DISP__NUM_ROWS - 1;	// Limit height
	int X_lim = DISP__NUM_COLUMNS - X - width;
	if(X_lim < 0) X_lim = 0;	// Limit width

	// Create bar to build rectangle from
	uint32_t bar = 0;
	bar = 0xFFFFFFFF << (DISP__NUM_COLUMNS - width - 1);
	bar = bar >> X;

	// Build rectangle from bar
	int R, P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		for(R = Y; R <= Y_lim; R++)
		{
			if(color->red & BIT(P)) buf->redRow[R][P] |= bar;
			else buf->redRow[R][P] &= ~bar;

			if(color->green & BIT(P)) buf->greenRow[R][P] |= bar;
			else buf->greenRow[R][P] &= ~bar;

			if(color->blue & BIT(P)) buf->blueRow[R][P] |= bar;
			else buf->blueRow[R][P] &= ~bar;
		}
	}
}

void DISP__drawCircle(DISP__imgBuf *buf, const DISP__PDMcolor *color, int X, int Y, int radius)
{
	// Local variables
	uint32_t barx, bary;
	int f, ddF_x, ddF_y, x, y, P;

	// Take absolute value of radius
	if (radius < 0) radius = 0 - radius;	// Efficient absolute value

	// Initialize variables
	f = 1 - radius;
	ddF_x = 1;
	ddF_y = -2 * radius;
	x = 0;
	y = radius;

	// Limit dimensions
	if((Y - radius > (DISP__NUM_ROWS - 1)) || (Y + radius < 0)) return;			// Circle is completely off screen, so nothing to draw
	if((X - radius > (DISP__NUM_COLUMNS - 1)) || (X + radius < 0)) return;			// Circle is completely off screen, so nothing to draw

	// Build circle from bar
	while(x<y)
	{
		if(f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		// Create bars to draw
		bary = 0xFFFFFFFF & ~(0xFFFFFFFF >> X + x + 1);
		bary &= ~(0xFFFFFFFF << (DISP__NUM_COLUMNS - X + x - 1));
		barx = 0xFFFFFFFF & ~(0xFFFFFFFF >> X + y + 1);
		barx &= ~(0xFFFFFFFF << (DISP__NUM_COLUMNS - X + y - 1));

		// Set indices
		int Ypy = Y + y;
		int Ymy = Y - y + 1;
		int Ypx = Y + x;
		int Ymx = Y - x + 1;
		// Now draw bars
		for(P = 0; P < DISP__COLOR_DEPTH; P++)
		{
			if(color->red & BIT(P))
			{
				if( Ypy >= 0 && Ypy < DISP__NUM_ROWS)	buf->redRow[Ypy][P] |= bary;
				if( Ymy >= 0 && Ymy < DISP__NUM_ROWS)	buf->redRow[Ymy][P] |= bary;
				if( Ypx >= 0 && Ypx < DISP__NUM_ROWS)	buf->redRow[Ypx][P] |= barx;
				if( Ymx >= 0 && Ymx < DISP__NUM_ROWS)	buf->redRow[Ymx][P] |= barx;
			}
			else
			{
				if( Ypy >= 0 && Ypy < DISP__NUM_ROWS)	buf->redRow[Ypy][P] &= ~bary;
				if( Ymy >= 0 && Ymy < DISP__NUM_ROWS)	buf->redRow[Ymy][P] &= ~bary;
				if( Ypx >= 0 && Ypx < DISP__NUM_ROWS)	buf->redRow[Ypx][P] &= ~barx;
				if( Ymx >= 0 && Ymx < DISP__NUM_ROWS)	buf->redRow[Ymx][P] &= ~barx;
			}

			if(color->green & BIT(P))
			{
				if( Ypy >= 0 && Ypy < DISP__NUM_ROWS)	buf->greenRow[Ypy][P] |= bary;
				if( Ymy >= 0 && Ymy < DISP__NUM_ROWS)	buf->greenRow[Ymy][P] |= bary;
				if( Ypx >= 0 && Ypx < DISP__NUM_ROWS)	buf->greenRow[Ypx][P] |= barx;
				if( Ymx >= 0 && Ymx < DISP__NUM_ROWS)	buf->greenRow[Ymx][P] |= barx;
			}
			else
			{
				if( Ypy >= 0 && Ypy < DISP__NUM_ROWS)	buf->greenRow[Ypy][P] &= ~bary;
				if( Ymy >= 0 && Ymy < DISP__NUM_ROWS)	buf->greenRow[Ymy][P] &= ~bary;
				if( Ypx >= 0 && Ypx < DISP__NUM_ROWS)	buf->greenRow[Ypx][P] &= ~barx;
				if( Ymx >= 0 && Ymx < DISP__NUM_ROWS)	buf->greenRow[Ymx][P] &= ~barx;
			}

			if(color->blue & BIT(P))
			{
				if( Ypy >= 0 && Ypy < DISP__NUM_ROWS)	buf->blueRow[Ypy][P] |= bary;
				if( Ymy >= 0 && Ymy < DISP__NUM_ROWS)	buf->blueRow[Ymy][P] |= bary;
				if( Ypx >= 0 && Ypx < DISP__NUM_ROWS)	buf->blueRow[Ypx][P] |= barx;
				if( Ymx >= 0 && Ymx < DISP__NUM_ROWS)	buf->blueRow[Ymx][P] |= barx;
			}
			else
			{
				if( Ypy >= 0 && Ypy < DISP__NUM_ROWS)	buf->blueRow[Ypy][P] &= ~bary;
				if( Ymy >= 0 && Ymy < DISP__NUM_ROWS)	buf->blueRow[Ymy][P] &= ~bary;
				if( Ypx >= 0 && Ypx < DISP__NUM_ROWS)	buf->blueRow[Ypx][P] &= ~barx;
				if( Ymx >= 0 && Ymx < DISP__NUM_ROWS)	buf->blueRow[Ymx][P] &= ~barx;
			}
		}
	}
}

void DISP__drawChar(DISP__imgBuf *buf, const DISP__PDMcolor *textColor, char alphNum, int X, int Y)
{
	// Limit X & Y range
	if(X >= DISP__NUM_COLUMNS) 	X = DISP__NUM_COLUMNS - 1;
	if(X < 0)	X = 0;
	if(Y >= DISP__NUM_ROWS) 	Y = DISP__NUM_ROWS - 1;
	if(Y < 0)	Y = 0;
	// Shift ASCII value to match charMap, since charMap starts at the 32nd ASCII value
	alphNum -= 32;

	int R, P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		for(R = 0; R < 7; R++)
		{
			if(textColor->red & BIT(P)) buf->redRow[Y + R][P] |= (unsigned) (charMap[alphNum][R] >> X);
			else buf->redRow[Y + R][P] &= ~((unsigned) (charMap[alphNum][R] >> X));

			if(textColor->green & BIT(P)) buf->greenRow[Y + R][P] |= (unsigned) (charMap[alphNum][R] >> X);
			else buf->greenRow[Y + R][P] &= ~((unsigned) (charMap[alphNum][R] >> X));

			if(textColor->blue & BIT(P)) buf->blueRow[Y + R][P] |= (unsigned) (charMap[alphNum][R] >> X);
			else buf->blueRow[Y + R][P] &= ~((unsigned) (charMap[alphNum][R] >> X));
		}
	}

}

void DISP__drawScreen(const DISP__imgBuf *buf)
{
	DISP__TXBuff = *buf;	//FIXME This should be done with DMA
}

void DISP__fillScreen(DISP__imgBuf *buf, const DISP__PDMcolor *color)
{
	int R, P;
	for(P = 0; P < DISP__COLOR_DEPTH; P++)
	{
		for(R = 0; R < DISP__NUM_ROWS; R++)
		{
			buf->redRow[R][P] = 		(color->red & BIT(P)) 	? 	0xFFFFFFFF : 0x00000000;
			buf->greenRow[R][P] = 		(color->green & BIT(P)) ? 	0xFFFFFFFF : 0x00000000;
			buf->blueRow[R][P] = 		(color->blue & BIT(P)) 	?	0xFFFFFFFF : 0x00000000;
		}
	}
}

void DISP__setColorPDM(DISP__PDMcolor *PDMColor, int red, int green, int blue)
{
	// First, clear colors
	PDMColor->red = 0;
	PDMColor->green = 0;
	PDMColor->blue = 0;

	// If all colors are zero (pixel is blank), then return.
	// This prevents wasting CPU cycles on blank pixels
	if(!(red || green || blue)) return;

	// Limit color values
	if(red >= DISP__COLOR_DEPTH) red = DISP__COLOR_DEPTH;
	if(red < 0) red = 0;
	if(green >= DISP__COLOR_DEPTH) green = DISP__COLOR_DEPTH;
	if(green < 0) green = 0;
	if(blue >= DISP__COLOR_DEPTH) blue = DISP__COLOR_DEPTH;
	if(blue < 0) blue = 0;

	int i = 0,
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
