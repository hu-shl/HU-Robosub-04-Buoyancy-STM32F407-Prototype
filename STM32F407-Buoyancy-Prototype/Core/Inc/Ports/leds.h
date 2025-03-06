/*
Author: 	W Pielage & E Helmond
Date:		2-10-2013
Revision:	2

    leds.c:
           LED-driver for SpARM-board v1

    pin-info:
           PB1 - RCK: Storage register clock
           PB8  - SCK: Shift register clock
           PA15 - SER: Data input

To use the LEDs use the following line to set the pins:
	LED_init();
After that, you can put an 8 bit value to the LEDs
*/

/****************Libraries******************************/
/* Libraries needed for LEDs are (These are in main.h):
 * #include "stm32f4xx.h"
 * #include "stm32f4xx_gpio.h"
 */

/****************Defines********************************/

/****************Function Prototypes********************/
void 	LED_init	(void);
void 	LED_put		(unsigned short led_byte);
