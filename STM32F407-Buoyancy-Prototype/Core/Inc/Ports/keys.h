/*
Author: 	J.F. van der Bent
			W Pielage & E Helmond

      Jannick Bloemendal 
Date:		6-06-2025
Revision:	6

    keys.c:
          Keyboard-driver for SpARM-board v3?

    pin-info:
           PD1  - R2
           PD2  - R3
           PD3  - R4
           PD6  - K1
           PD7  - K2
           PD8  - K3
           PD9  - K4
           PD11 - R1

To use the keyboard first initialize the ports:
	KEYS_init();
After that, you can use polling to read a key with:
	KEYS_read();

	07-07-2014 toevoeging defines key values
*/

/****************Libraries******************************/
/* Libraries needed for Keys are (These are in main.h):
 * #include "stm32f4xx.h"
 * #include "stm32f4xx_gpio.h"
 * #include "stm32f4xx_syscfg.h"
 */

/****************Defines********************************/
#define	KEY_ROW			(KEY_R1_Pin | KEY_R2_Pin | KEY_R3_Pin | KEY_R4_Pin)
// #define PKEY_ROW		GPIOD
#define	KEY_COL			(KEY_K1_Pin | KEY_K2_Pin | KEY_K3_Pin | KEY_K4_Pin)
// #define	PKEY_COL		GPIOD

/****************Function Prototypes********************/
unsigned int 	KEYS_kolom	(void);
unsigned int 	KEYS_read	(void);
// void 			KEYS_init	(void); is in main.c
void 			KEYS_initISR(int);

// Key map
#define key_1 1
#define key_2	2
#define key_3	3
#define key_4	5
#define key_5	6
#define key_6	7
#define key_7	9
#define key_8	10
#define key_9	11
#define key_0	14
#define key_s	13	// Star
#define key_h	15	// Hash
#define key_a	4
#define key_b	8
#define key_c	12
#define key_d	16
