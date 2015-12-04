/*
 * RGB_LED_Panel.h
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

#ifndef DISPLAY_RGB_16X32_H_
#define DISPLAY_RGB_16X32_H_

// Includes
#include <stdint.h>
#include <math.h>
#include <assert.h>

// Display parameters
#define DISP__NUM_ROWS			16
#define	DISP__NUM_COLUMNS		32
#define DISP__COLOR_DEPTH		8		// Number of PDM Pulses per color

// Pin Definitions
#define CtrlDIR					P2DIR
#define CtrlPort				P2OUT

#define A						BIT0
#define B						BIT3
#define C						BIT4
#define D						BIT5
#define CLK						BIT1
#define STB						BIT6
#define OE						BIT7

// Declarations
typedef struct DISP__imgBuf
{
	uint32_t redRow[DISP__NUM_ROWS][DISP__COLOR_DEPTH];
	uint32_t greenRow[DISP__NUM_ROWS][DISP__COLOR_DEPTH];
	uint32_t blueRow[DISP__NUM_ROWS][DISP__COLOR_DEPTH];
} DISP__imgBuf;

typedef struct DISP__PDMcolor
{
	uint32_t red;
	uint32_t green;
	uint32_t blue;
} DISP__PDMcolor;

typedef enum DISP__SM_states
{
	SM_start,
	SM_init,
	SM_pause,
	SM_stop,
	SM_reset,
	SM_idle,
	SM_sending_frame
} DISP__SM_states;

typedef struct DISP__SM_flag_types
{
	uint8_t send_frame;
	uint8_t frame_sent;
	uint8_t reset;
	uint8_t frame_overrun;
} DISP__SM_flag_types;

extern void DISP__stateMachine();
extern void DISP__initDisplayDriver();
extern void DISP__startDisplayDriver();
extern void DISP__stopDisplayDriver();
extern void DISP__pauseDisplayDriver();
extern void DISP__resetDisplayDriver();
extern void DISP__flashRow();
extern void DISP__setBrightness(uint16_t brightness);
extern int32_t DISP__setFrameRate(int32_t frame_rate);
extern void DISP__recalculateRowAddress();
extern void DISP__sendFrame();
extern void DISP__eUSCIA0IsrHandler(void);					// SPI interrupt service routine
extern void DISP__oneShotFlashTimerISR(void);					// One-shot timer to pulse LED row
extern void DISP__frameTimerISR(void);						// Continuous timer to transmit each frame

extern void DISP__drawLine();
extern void DISP__drawRect(DISP__imgBuf *buf, const DISP__PDMcolor *color, int X, int Y, int height, int width);
extern void DISP__drawCircle();
extern void DISP__drawChar(DISP__imgBuf *buf, const DISP__PDMcolor *PDMColor, const char alphNum);
extern void DISP__drawScreen();
extern void DISP__fillScreen(DISP__imgBuf *buf, const DISP__PDMcolor *PDMColor);
extern void DISP__setColorPDM(DISP__PDMcolor *PDMcolor, const int32_t red, const int32_t green, const int32_t blue);

#endif /* DISPLAY_RGB_16X32_H_ */
