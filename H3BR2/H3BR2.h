/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H3BR2.h
 Description   : Header file for module H3BR2.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H3BR2_H
#define H3BR2_H

/* Includes ------------------------------------------------------------------*/
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
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
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
#define cc1_Pin             GPIO_PIN_13
#define cc1_GPIO_Port       GPIOC
#define cc2_Pin             GPIO_PIN_14
#define cc2_GPIO_Port       GPIOC
#define E_Pin               GPIO_PIN_1
#define E_GPIO_Port         GPIOB
#define C_Pin               GPIO_PIN_2
#define C_GPIO_Port         GPIOB
#define F_Pin               GPIO_PIN_8
#define F_GPIO_Port         GPIOA
#define G_Pin               GPIO_PIN_12
#define G_GPIO_Port         GPIOA
#define D_Pin               GPIO_PIN_1
#define D_GPIO_Port         GPIOD
#define A_Pin               GPIO_PIN_5
#define A_GPIO_Port         GPIOB
#define DP_Pin              GPIO_PIN_8
#define DP_GPIO_Port        GPIOB
#define B_Pin               GPIO_PIN_9
#define B_GPIO_Port         GPIOB

/* Indicator LED */
#define _IND_LED_PORT			GPIOA
#define _IND_LED_PIN			GPIO_PIN_11

/* Module-specific Macro Definitions ***************************************/

#define NUM_MODULE_PARAMS						1


/* Module_Status Type Definition */
typedef enum {
	H3BR2_OK =0,
	H3BR2_ERR_UnknownMessage,
	H3BR2_ERR_WrongParams,
	H3BR2_NUMBER_IS_OUT_OF_RANGE,
	H3BR2_ERROR =255
} Module_Status;
// Encode  numbers
typedef enum{
	zero_number=0xc0, one_number=0xf9, two_number=0xa4, three_number=0xb0, four_number=0x19, five_number=0x12, six_number=0x82, seven_number=0xf8, eight_number=0x80 ,nine_number=0x90,

	a_letter=0xA0, b_letter=0x83, c_letter=0xA7, d_letter=0xA1, e_letter=0x86, f_letter=0x8E, g_letter=0x90, h_letter=0x8B, i_letter=0xEF, j_letter=0xE1,k_letter=0x8A,

	l_letter=0xC7  , m_letter=0xC8,n_letter=0xAB, o_letter=0xAB, p_letter=0x8C, q_letter=0x67, r_letter=0x50, s_letter=0x93, t_letter=0x87, u_letter=0xE3,

	v_letter=0xC1, w_letter=0x81 ,x_letter=0x89,y_letter=0x91, z_letter=0xE4,

	A_letter=0x88, B_letter=0x83 , C_letter=0xC6 , D_letter=0xA1, E_letter=0x86 , F_letter=0x8E , G_letter=0xC2 , H_letter=0x8B, I_letter=0xEF , J_letter=0xE1 ,K_letter=0x8A,

	L_letter=0xC7  , M_letter=0xC8,N_letter=0xAB , O_letter=0xA3 , P_letter=0x8C ,  Q_letter=0x98 , R_letter=0xAF,  S_letter=0x93  ,T_letter=0x87, U_letter=0xE3,

	V_letter=0xC1 ,W_letter=0x81 , X_letter=0x89 ,Y_letter=0x91 , Z_letter=0xE4  ,

	Empty = 0xff,

	Symbol_minus=0x40

} Segment_Codes;

extern uint8_t  test;
extern int32_t Number;



extern Segment_Codes Digit[2]; //Digit[0]: LSD, Digit[1]: MSD
extern   char letter[2];
// declaration of private functions
extern Segment_Codes get_number_code(uint8_t digit);
extern Segment_Codes get_letter_code(char letter);
extern Segment_Codes clear_all_digits(void);


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
extern void ExecuteMonitor(void);

/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
extern Module_Status SevenDisplayNumberF(float NumberF);
extern Module_Status SevenDisplayNumber(uint8_t Number);
extern Module_Status SevenDisplayNumberHexa( uint8_t Hexadecimal);
extern Module_Status SevenDisplayOff(void);
extern Module_Status SevenDisplayOneDigit(uint8_t Number, uint8_t StartSevSeg);
extern Module_Status SevenDisplayOneDigitHexa(uint8_t Number, uint8_t StartSevSeg);

/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */
extern const CLI_Command_Definition_t CLI_SevenDisplayNumberCommandDefinition;
extern const CLI_Command_Definition_t CLI_SevenDisplayNumberFCommandDefinition;
extern const CLI_Command_Definition_t CLI_SevenDisplayNumberHexaCommandDefinition;
extern const CLI_Command_Definition_t CLI_SevenDisplayOffCommandDefinition;
extern const CLI_Command_Definition_t CLI_SevenDisplayOnDigitCommandDefinition;
extern const CLI_Command_Definition_t CLI_SevenDisplayOnDigitHexaCommandDefinition;


/* -----------------------------------------------------------------------*/

#endif /* H3BR2_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
