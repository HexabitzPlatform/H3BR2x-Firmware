/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H3BR2.h
 Description: Header file for H3BR2 module.
 Peripherals: Defines USART1-6 for ports (P1-P6), GPIO for 7-segment display.
 Features: Declares functions for 7-segment display (integers, floats, hex, single digits),
           UART port mappings, and module-specific configurations (GPIO, DMA, EEPROM).
*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef H3BR2_H
#define H3BR2_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H3BR2_MemoryMap.h"
#include "H3BR2_uart.h"
#include "H3BR2_gpio.h"
#include "H3BR2_dma.h"
#include "H3BR2_inputs.h"
#include "H3BR2_eeprom.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H3BR2

/* Port-related Definitions */
#define	NUM_OF_PORTS	6
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart6
#define UART_P4 &huart3
#define UART_P5 &huart1
#define UART_P6 &huart5

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define	USART6_AF			GPIO_AF3_USART6

/* Pins For Seven Segment */
#define CC1_PIN             GPIO_PIN_13
#define CC1_GPIO_PORT       GPIOC
#define CC2_PIN             GPIO_PIN_14
#define CC2_GPIO_PORT       GPIOC
#define E_PIN               GPIO_PIN_1
#define E_GPIO_PORT         GPIOB
#define C_PIN               GPIO_PIN_2
#define C_GPIO_PORT         GPIOB
#define F_PIN               GPIO_PIN_8
#define F_GPIO_PORT         GPIOA
#define G_PIN               GPIO_PIN_12
#define G_GPIO_PORT         GPIOA
#define D_PIN               GPIO_PIN_1
#define D_GPIO_PORT         GPIOD
#define A_PIN               GPIO_PIN_5
#define A_GPIO_PORT         GPIOB
#define DP_PIN              GPIO_PIN_8
#define DP_GPIO_PORT        GPIOB
#define B_PIN               GPIO_PIN_9
#define B_GPIO_PORT         GPIOB

/* Indicator LED */
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_11

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS	1

/* Module-specific Type Definition *****************************************/
/* Module_Status Type Definition */
typedef enum {
	H3BR2_OK =0,
	H3BR2_ERR_UNKNOWNMESSAGE,
	H3BR2_ERR_WRONGPARAMS,
	H3BR2_NUMBER_IS_OUT_OF_RANGE,
	H3BR2_ERROR =255
} Module_Status;

/* Numbers / Letters Representations Type Definition */
typedef enum{
	/* Numbers Representation*/
	ZERO_NUMBER = 0XC0, ONE_NUMBER = 0XF9, TWO_NUMBER = 0XA4, THREE_NUMBER = 0XB0, FOUR_NUMBER = 0X19,
	FIVE_NUMBER = 0X12, SIX_NUMBER = 0X82, SEVEN_NUMBER = 0XF8, EIGHT_NUMBER = 0X80 ,NINE_NUMBER = 0X90,

	/* Small Letter Representation*/
	a_LETTER = 0xA0, b_LETTER = 0x83, c_LETTER = 0xA7, d_LETTER = 0xA1, e_LETTER = 0x86, f_LETTER = 0x8E,
	g_LETTER = 0x90, h_LETTER = 0x8B, i_LETTER = 0xEF, j_LETTER = 0xE1, k_LETTER = 0x8A, l_LETTER = 0xC7,
	m_LETTER = 0xC8, n_LETTER = 0xAB, o_LETTER = 0xAB, p_LETTER = 0x8C, q_LETTER = 0x67, r_LETTER = 0x50,
	s_LETTER = 0x93, t_LETTER = 0x87, u_LETTER = 0xE3, v_LETTER = 0xC1, w_LETTER = 0x81, x_LETTER = 0x89,
	y_LETTER = 0x91, z_LETTER = 0xE4,

	/* Capital Letter Representation*/
	A_LETTER =0X88, B_LETTER = 0X83, C_LETTER = 0XC6, D_LETTER = 0XA1, E_LETTER = 0X86, F_LETTER = 0X8E,
	G_LETTER =0XC2, H_LETTER = 0X8B, I_LETTER = 0XEF, J_LETTER = 0XE1, K_LETTER = 0X8A, L_LETTER = 0XC7,
	M_LETTER =0XC8, N_LETTER = 0XAB, O_LETTER = 0XA3, P_LETTER = 0X8C, Q_LETTER = 0X98, R_LETTER = 0XAF,
	S_LETTER =0X93, T_LETTER = 0X87, U_LETTER = 0XE3, V_LETTER = 0XC1, W_LETTER = 0X81, X_LETTER = 0X89,
	Y_LETTER =0X91, Z_LETTER = 0XE4,

	EMPTY = 0XFF,

	SYMBOL_MINUS = 0X40

} SegmentCodes;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SevenDisplayOff(void);
Module_Status SevenDisplayNumber(uint8_t Number);
Module_Status SevenDisplayNumberF(float NumberF);
Module_Status SevenDisplayNumberHexa( uint8_t Hexadecimal);
Module_Status SevenDisplayOneDigit(uint8_t Number, uint8_t StartSevSeg);
Module_Status SevenDisplayOneDigitHexa(uint8_t Number, uint8_t StartSevSeg);

#endif /* H3BR2_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
