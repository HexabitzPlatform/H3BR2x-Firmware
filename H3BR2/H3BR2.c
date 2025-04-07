/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H3BR2.c
 Description   : Source code for module H3BR2.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H3BR2_inputs.h"
#include <ctype.h>
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

Segment_Codes Digit[2] = {Empty,Empty}; //Digit[0]: LSD, Digit[1]: MSD
 char letter[2];
 int32_t Number;
 uint8_t StartSevSeg;

/* Module exported parameters ------------------------------------------------*/
 module_param_t modParam[NUM_MODULE_PARAMS] ={0};

/* Private variables ---------------------------------------------------------*/
uint8_t index_7_seg = 0;
TimerHandle_t xTimerSwitch = NULL;
uint8_t flag_float=0;
/* Private function prototypes -----------------------------------------------*/
void SwitchTimerCallback(TimerHandle_t xTimerSwitch);
void ExecuteMonitor(void);
void FLASH_Page_Eras(uint32_t Addr );

/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_SevenDisplayNumberCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SevenDisplayNumberFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SevenDisplayOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SevenDisplayNumberHexaCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SevenDisplayOneDigitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_SevenDisplayOneDigitHexaCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/*-----------------------------------------------------------*/
/* CLI command structure : SevenDisplayNumber */
const CLI_Command_Definition_t CLI_SevenDisplayNumberCommandDefinition =
{
	( const int8_t * ) "sevendisplaynumber", /* The command string to type. */
	( const int8_t * ) "sevendisplaynumber:\r\n Parameters required to execute a SevenDisplayNumber: Number  \r\n\r\n",
	CLI_SevenDisplayNumberCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
//*************************************************************
/* CLI command structure : SevenDisplayNumberF */
const CLI_Command_Definition_t CLI_SevenDisplayNumberFCommandDefinition =
{
	( const int8_t * ) "sevendisplaynumberf", /* The command string to type. */
	( const int8_t * ) "sevendisplaynumberf:\r\n Parameters required to execute a SevenDisplayNumberF: NumberF  \r\n\r\n",
	CLI_SevenDisplayNumberFCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
//*************************************************************
/* CLI command structure : SevenDisplayNumberh */
const CLI_Command_Definition_t CLI_SevenDisplayNumberHexaCommandDefinition =
{
	( const int8_t * )"sevendisplaynumberhexa", /* The command string to type. */
	( const int8_t * ) "sevendisplaynumberhexa:\r\n Parameters required to execute a SevenDisplayNumber: Number  \r\n\r\n",
	CLI_SevenDisplayNumberHexaCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
//******************************************************************
/* CLI command structure : SevenDisplayOff */
const CLI_Command_Definition_t CLI_SevenDisplayOffCommandDefinition =
{
	( const int8_t * ) "sevendisplayoff", /* The command string to type. */
	( const int8_t * ) "sevendisplayoff:\r\nParameters required to execute a SevenDisplayOff \r\n\r\n",
	CLI_SevenDisplayOffCommand, /* The function to run. */
	0 /* zero parameters are expected. */
};

//***********************************************************************

/* CLI command structure : SevenDisplayoneNumbercommand */
const CLI_Command_Definition_t CLI_SevenDisplayOneDigitCommandDefinition =
{
		( const int8_t * ) "sevendisplayonedigit", /* The command string to type. */
			( const int8_t * ) "sevendisplayonedigit:\r\n Parameters required to execute a SevenDisplayNumber: Number , StartSevSeg \r\n\r\n",
			CLI_SevenDisplayOneDigitCommand, /* The function to run. */
			2 /* two parameters are expected. */
};
//***********************************************************************
/* CLI command structure : SevenDisplayoneNumberhcommand */
const CLI_Command_Definition_t CLI_SevenDisplayOneDigitHexaCommandDefinition =
{
		( const int8_t * ) "sevendisplayonedigithexa", /* The command string to type. */
			( const int8_t * ) "sevendisplayonedigithexa:\r\n Parameters required to execute a SevenDisplayNumber: Number , StartSevSeg \r\n\r\n",
			CLI_SevenDisplayOneDigitHexaCommand, /* The function to run. */
			2 /* two parameters are expected. */
};
/* -----------------------------------------------------------------------
 |						 Private Functions	 														|
 -----------------------------------------------------------------------
 */
/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex,float *value){
	Module_Status status =BOS_OK;

	switch(paramIndex){

		/* Invalid parameter index */
		default:
			status =BOS_ERR_WrongParam;
			break;
	}

	return status;
}

/*-----------------------------------------------------------*/
Segment_Codes get_number_code(uint8_t digit)
{
	Module_Status status = H3BR2_OK;

	Segment_Codes code;
	switch(digit)
	{
	case 0:
		code = zero_number;
		break;
	case 1:
		code = one_number;
		break;
	case 2:
		code = two_number;
		break;
	case 3:
		code = three_number;
		break;
	case 4:
		code = four_number;
		break;
	case 5:
		code = five_number;
		break;
	case 6:
		code = six_number;
		break;
	case 7:
		code = seven_number;
		break;
	case 8:
		code = eight_number;
		break;
	case 9:
		code = nine_number;
		break;
	case 10:
			code = A_letter;
			break;
	case 11:
				code = B_letter;
				break;
	case 12:
				code = C_letter;
				break;
	case 13:
				code = D_letter;
				break;
	case 14:
				code = E_letter;
				break;
	case 15:
				code = F_letter;
				break;


	default:
		code = Empty;
		break;

	}
	return code;
}

/*-----------------------------------------------------------*/
Segment_Codes get_letter_code(char letter){
	Module_Status status = H3BR2_OK;

	Segment_Codes letter_code;
	switch(letter){

	case 'A':
		letter_code=A_letter;
		break;
	case 'B':
		letter_code=B_letter;
		break;
	case'C':
		letter_code=C_letter;
		break;
	case'D':
		letter_code=D_letter;
		break;
	case'E':
		letter_code=E_letter;
		break;
	case'F':
		letter_code=F_letter;
		break;
	case'G':
		letter_code=G_letter;
		break;
	case'H':
		letter_code=H_letter;
		break;
	case'I':
		letter_code=I_letter;
		break;
	case'J':
		letter_code=J_letter;
		break;
	case'K':
		letter_code=K_letter;
		break;
	case'L':
		letter_code=L_letter;
		break;
	case'M':
		letter_code=M_letter;
		break;
	case'N':
		letter_code=N_letter;
		break;
	case'O':
		letter_code=O_letter;
		break;
	case'P':
		letter_code=P_letter;
		break;
	case'Q':
		letter_code=Q_letter;
		break;
	case'R':
		letter_code=R_letter;
		break;
	case'S':
		letter_code=S_letter;
		break;
	case'T':
		letter_code=T_letter;
		break;
	case'U':
		letter_code=U_letter;
		break;
	case'V':
		letter_code=V_letter;
		break;
	case'W':
		letter_code=W_letter;
		break;
	case'X':
		letter_code=X_letter;
		break;
	case'Y':
		letter_code=Y_letter;
		break;
	case'Z':
		letter_code=Z_letter;
		break;

	case 'a':
		letter_code=a_letter;
		break;
	case 'b':
		letter_code=b_letter;
		break;
	case'c':
		letter_code=c_letter;
		break;
	case'd':
		letter_code=d_letter;
		break;
	case'e':
		letter_code=e_letter;
		break;
	case'f':
		letter_code=f_letter;
		break;
	case'g':
		letter_code=g_letter;
		break;
	case'h':
		letter_code=h_letter;
		break;
	case'i':
		letter_code=i_letter;
		break;
	case'j':
		letter_code=j_letter;
		break;
	case'k':
		letter_code=k_letter;
		break;
	case'l':
		letter_code=l_letter;
		break;
	case'm':
		letter_code=m_letter;
		break;
	case'n':
		letter_code=n_letter;
		break;
	case'o':
		letter_code=o_letter;
		break;
	case'p':
		letter_code=p_letter;
		break;
	case'q':
		letter_code=q_letter;
		break;
	case'r':
		letter_code=r_letter;
		break;
	case's':
		letter_code=s_letter;
		break;
	case't':
		letter_code=t_letter;
		break;
	case'u':
		letter_code=u_letter;
		break;
	case'v':
		letter_code=v_letter;
		break;
	case'w':
		letter_code=w_letter;
		break;
	case'x':
		letter_code=x_letter;
		break;
	case'y':
		letter_code=y_letter;
		break;
	case'z':
		letter_code=z_letter;
		break;

	default: break;

	}
	return letter_code;
}
/*-----------------------------------------------------------*/
uint32_t get_hexadigit_code(char Number){
	Module_Status status = H3BR2_OK;

	uint32_t hexadigit_code;
	switch(Number){

	case 'A':
		hexadigit_code=10;
		break;
	case 'B':
		hexadigit_code=11;
		break;
	case'C':
		hexadigit_code=12;
		break;
	case'D':
		hexadigit_code=13;
		break;
	case'E':
		hexadigit_code=14;
		break;
	case'F':
		hexadigit_code=15;
		break;


	case 'a':
		hexadigit_code=10;
		break;
	case 'b':
		hexadigit_code=11;
		break;
	case'c':
		hexadigit_code=12;
		break;
	case'd':
		hexadigit_code=13;
		break;
	case'e':
		hexadigit_code=14;
		break;
	case'f':
		hexadigit_code=15;
		break;
	case'0':
		hexadigit_code=0;
		break;
	case'1':
			hexadigit_code=1;
			break;
	case'2':
			hexadigit_code=2;
			break;
	case'3':
			hexadigit_code=3;
			break;
	case'4':
			hexadigit_code=4;
			break;
	case'5':
			hexadigit_code=5;
			break;
	case'6':
			hexadigit_code=6;
			break;
	case'7':
			hexadigit_code=7;
			break;
	case'8':
			hexadigit_code=8;
			break;
	case'9':
			hexadigit_code=9;
			break;


	default: break;

	}
	return hexadigit_code;
}
/*-----------------------------------------------------------*/





Segment_Codes clear_all_digits(void){
	Module_Status status = H3BR2_OK;
	flag_float=0;
	for(int i=0;i<2;i++) Digit[i] = Empty;
}
//************************************************************

/**
* @brief  System Clock Configuration
*         This function configures the system clock as follows:
*            - System Clock source            = PLL (HSE)
*            - SYSCLK(Hz)                     = 64000000
*            - HCLK(Hz)                       = 64000000
*            - AHB Prescaler                  = 1
*            - APB1 Prescaler                 = 1
*            - HSE Frequency(Hz)              = 8000000
*            - PLLM                           = 1
*            - PLLN                           = 16
*            - PLLP                           = 2
*            - Flash Latency(WS)              = 2
*            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
* @param  None
* @retval None
*/
void SystemClock_Config(void){
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /** Configure the main internal regulator output voltage */
   HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

   /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
   RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
   RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
   RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
   RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
   RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
   RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
   RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /** Initializes the CPU, AHB and APB buses clocks */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}
/*-----------------------------------------------------------*/

/* --- Save Command Topology in Flash RO --- */

uint8_t SaveTopologyToRO(void)
{
	HAL_StatusTypeDef flashStatus =HAL_OK;
	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
    uint16_t temp =0;

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MaxNumOfPorts; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Save Command Snippets in Flash RO --- */

uint8_t SaveSnippetsToRO(void)
{
	HAL_StatusTypeDef FlashStatus =HAL_OK;
    uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
    /* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index = 0; index < numOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(snippets[index].cond.conditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[index],sizeof(snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j = 0; j < ((strlen(snippets[index].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[index].cmd + j*4 ));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Clear array topology in SRAM and Flash RO --- */

uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);


	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);
	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/* --- H3BR2 module initialization.
 */

void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	//Circulating DMA Channels ON All Module
	 for(int i=1;i<=NumOfPorts;i++)
		{
		  if(GetUart(i)==&huart1)
				   { index_dma[i-1]=&(DMA1_Channel1->CNDTR); }
		  else if(GetUart(i)==&huart2)
				   { index_dma[i-1]=&(DMA1_Channel2->CNDTR); }
		  else if(GetUart(i)==&huart3)
				   { index_dma[i-1]=&(DMA1_Channel3->CNDTR); }
		  else if(GetUart(i)==&huart4)
				   { index_dma[i-1]=&(DMA1_Channel4->CNDTR); }
		  else if(GetUart(i)==&huart5)
				   { index_dma[i-1]=&(DMA1_Channel5->CNDTR); }
		  else if(GetUart(i)==&huart6)
				   { index_dma[i-1]=&(DMA1_Channel6->CNDTR); }
		}
	//Init a timer for 7-seg:
   	xTimerSwitch =xTimerCreate("SwitchTimer",pdMS_TO_TICKS(10),pdTRUE,(void* )1,SwitchTimerCallback);
	xTimerStart(xTimerSwitch, 0 );
	//seven segment GPIO Init:
	 MX_GPIO_Init();
	/* Create module special task (if needed) */
}
/* --- Switch timer callback ---*/
void SwitchTimerCallback(TimerHandle_t xTimerSwitch) {

	HAL_GPIO_WritePin(cc1_GPIO_Port, cc1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(cc2_GPIO_Port, cc2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, 1);
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, 	Digit[index_7_seg] & 0b00000001);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, 	Digit[index_7_seg] & 0b00000010);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, 	Digit[index_7_seg] & 0b00000100);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, 	Digit[index_7_seg] & 0b00001000);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, 	Digit[index_7_seg] & 0b00010000);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, 	Digit[index_7_seg] & 0b00100000);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, 	Digit[index_7_seg] & 0b01000000);

			if(index_7_seg==1 && flag_float==1)
			HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, 0);
            else if(index_7_seg==0 && flag_float==1)
    			{HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, 1);
    			}

			switch(index_7_seg)
			{
				case 0:
					HAL_GPIO_WritePin(cc2_GPIO_Port, cc2_Pin, GPIO_PIN_SET);

					break;

				case 1:
					HAL_GPIO_WritePin(cc1_GPIO_Port, cc1_Pin, GPIO_PIN_SET);
					break;
				default:
					break;

			}

			index_7_seg++;
			if(index_7_seg > 1) index_7_seg = 0;


				}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/* --- H3BR2 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H3BR2_OK;
	uint32_t Number=0;
	uint8_t StartSevSeg=0;

	uint32_t Number_int;
	float NumberF=0;
	switch(code){
	case CODE_H3BR2_SevenDisplayNumber:
		  Number=(uint32_t )cMessage[port - 1][shift];
		  SevenDisplayNumber(Number);
		  break;

	case CODE_H3BR2_SevenDisplayNumberF:

  Number_int=((uint32_t) cMessage[port - 1][shift] + (uint32_t) (cMessage[port - 1][1+shift] <<8) + (uint32_t) (cMessage[port - 1][2+shift]<<16) + (uint32_t) (cMessage[port - 1][3+shift] <<24));
  NumberF = *((float*)&Number_int);
  SevenDisplayNumberF(NumberF);
			  break;

	case CODE_H3BR2_SevenDisplayNumberHexa:
			  Number=(uint32_t )cMessage[port - 1][shift];
			  SevenDisplayNumberHexa(Number);
		  break;
	case CODE_H3BR2_SevenDisplayOneDigit:
		  Number=(uint32_t )cMessage[port - 1][shift];
		  StartSevSeg=(uint32_t )cMessage[port - 1][shift+1];
		  SevenDisplayOneDigit(Number, StartSevSeg);
		  break;
	case CODE_H3BR2_SevenDisplayOneDigitHexa:
			  Number=(uint32_t )cMessage[port - 1][shift];
			  StartSevSeg=(uint32_t )cMessage[port - 1][shift+1];
			  SevenDisplayOneDigitHexa(Number, StartSevSeg);
			  break;
	case CODE_H3BR2_SevenDisplayOff:
			  SevenDisplayOff();
			  break;

		default:
			result =H3BR2_ERR_UnknownMessage;
			break;
	}

	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P4;
	else if(huart->Instance == USART1)
		return P5;
	else if(huart->Instance == USART5)
		return P6;
	else if(huart->Instance == USART6)
		return P3;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){
	 FreeRTOS_CLIRegisterCommand(&CLI_SevenDisplayNumberCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_SevenDisplayNumberFCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_SevenDisplayOffCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_SevenDisplayNumberHexaCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_SevenDisplayOneDigitCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_SevenDisplayOneDigitHexaCommandDefinition);

}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
//void Module_Special_Task(void *argument){
//
//	/* Infinite loop */
//	uint8_t cases; // Test variable.
//	for(;;){
//		/*  */
//		switch(cases){
//
//
//			default:
//				osDelay(10);
//				break;
//		}
//
//		taskYIELD();
//	}
//
//}


/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */

/*-----------------------------------------------------------*/
Module_Status SevenDisplayNumberF(float NumberF)
{
      Module_Status status = H3BR2_OK;
    clear_all_digits();   //Seven segment display off

    uint8_t Number_int;
    flag_float=1;
    if(NumberF <0 || NumberF >9.9)
        {
            status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
            return status;
        }

    Number_int=(uint8_t)(NumberF*10);

    for(int i = 0; i < 2;i++)
    {
       Digit[i] = get_number_code(Number_int % 10);
        Number_int /= 10;
    }
    HAL_Delay(5);

    return status;
}



/*----------------------------------------------------------*/
Module_Status SevenDisplayNumber(uint8_t Number)
{

	Module_Status status = H3BR2_OK;
	clear_all_digits();   //Seven segment display off


	if(Number < 0 || Number > 99 )
	{
		status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
		return status;
	}

	for(int i = 0; i < 2;i++)
		{

			Digit[i] = get_number_code(Number % 10);
			Number /= 10;
		}
	HAL_Delay(5);
	return status ;

}
/*-----------------------------------------------------------*/
Module_Status SevenDisplayNumberHexa( uint8_t Hexadecimal)
{
   	Module_Status status = H3BR2_OK;
	clear_all_digits();   //Seven segment display off

if(Hexadecimal <0 || Hexadecimal >255)
	{
		status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
		return status;
	}


uint32_t low= Hexadecimal & 0x0f;
uint32_t high = (Hexadecimal & 0xf0) >> 4;

	Digit[0] = get_number_code(low);
	Digit[1] = get_number_code(high);
	HAL_Delay(5);
return status ;

}
/*-----------------------------------------------------------*/
Module_Status SevenDisplayOneDigit(uint8_t Number, uint8_t StartSevSeg)
{
	Module_Status status = H3BR2_OK;
	clear_all_digits();   //Seven segment display off


	if( !(StartSevSeg >= 0 && StartSevSeg <= 1) )
	{
		status = H3BR2_ERR_WrongParams;
		return status;
	}
 if(Number > 9 )
	       	{
	       		status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
	       		return status;
	       	}
	  if(StartSevSeg==0)
   {


   			Digit[0] = get_number_code(Number);
   			Digit[1] =Empty;
   }
   else if(StartSevSeg==1)
     {

     			Digit[0] = Empty;
        		Digit[1] =get_number_code(Number);
     }



	  HAL_Delay(5);
	return status;

}
/* ----------------------------------------------------------------------------*/
Module_Status SevenDisplayOneDigitHexa(uint8_t Hexadecimal, uint8_t StartSevSeg)
{
	Module_Status status = H3BR2_OK;
	clear_all_digits();   //Seven segment display off

	uint32_t max_value;
	uint32_t low , high;
	 low= Hexadecimal & 0x0f;

	if( !(StartSevSeg >= 0 && StartSevSeg <= 1) )
	{
		status = H3BR2_ERR_WrongParams;
		return status;
	}
	if(Number > 9 )
		       	{
		       		status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
		       		return status;
		       	}
		  if(StartSevSeg==0)
	   {


	   			Digit[0] = get_number_code(low);
	   			Digit[1] =Empty;
	   }
	   else if(StartSevSeg==1)
	     {

	     			Digit[0] = Empty;
	        		Digit[1] =get_number_code(low);
	     }

		  HAL_Delay(5);
	return status;

}
/*-----------------------------------------------------------*/
Module_Status SevenDisplayOff(void){

	Module_Status status = H3BR2_OK;
	clear_all_digits();   //Seven segment display off
	return status;

}
/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */

portBASE_TYPE CLI_SevenDisplayNumberCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;

	static uint8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;


	static const int8_t *pcOKMessage=(int8_t* )"SevenSegmentDisplay is on:\r\n%d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Number is out of range!\n\r";



	(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	Number =(uint32_t )atol((char* )pcParameterString1);


	status=SevenDisplayNumber(Number);

	if(status == H3BR2_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,Number);

	}

	else if(status == H3BR2_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	else if(status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongRangeMessage);


	return pdFALSE;
}
/* ----------------------------------------------------------------------------*/
portBASE_TYPE CLI_SevenDisplayNumberFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;
	 float NumberF=0;
	static uint8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;


	static const int8_t *pcOKMessage=(int8_t* )"SevenSegmentDisplay is on:\r\n%0.1f  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"NumberF is out of range!\n\r";



	(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	NumberF =(float )atof((char* )pcParameterString1);


	status=SevenDisplayNumberF(NumberF);

	if(status == H3BR2_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,NumberF);

	}

	else if(status == H3BR2_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	else if(status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongRangeMessage);


	return pdFALSE;
}

/* ----------------------------------------------------------------------------*/
portBASE_TYPE CLI_SevenDisplayNumberHexaCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;
	char* Sentence=NULL;
	uint32_t hexacode;
	uint32_t hexadigit[2];
		static uint8_t *pcParameterString1;
		portBASE_TYPE xParameterStringLength1 =0;

		static const char *pcOKMessage=(int8_t* )"SevenSegmentDisplay is on:\r\n%c%c \n\r";
		static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
		static const int8_t *pcWrongRangeMessage =(int8_t* )"Number is out of range!\n\r";

		(void )xWriteBufferLen;
			configASSERT(pcWriteBuffer);

		pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
		 Sentence =(char* )pcParameterString1;
for(int i=0 ; i<2;i++)
{
	hexadigit[i]=get_hexadigit_code(Sentence[i]);
}
hexacode=hexadigit[0]*16 +hexadigit[1] ;

		status=SevenDisplayNumberHexa(hexacode);

		if(status == H3BR2_OK)
		{
			sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,Sentence[0],Sentence[1]);

		}

		else if(status == H3BR2_ERR_WrongParams)
			strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

		else if(status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
			strcpy((char* )pcWriteBuffer,(char* )pcWrongRangeMessage);
		return pdFALSE;
}
/*-----------------------------------------------------------*/

portBASE_TYPE CLI_SevenDisplayOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;

	static const int8_t *pcOKMessage=(int8_t* )"SevenSegmentDisplay is off \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

	 	status=SevenDisplayOff();

	 if(status == H3BR2_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	 }

	 else if(status == H3BR2_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_SevenDisplayOneDigitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;
	uint32_t Number;
		uint8_t StartSevSeg;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	static const int8_t *pcOKMessage=(int8_t* )"SevenSegmentDisplay is on:\r\n%d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Number is out of range!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	Number =(uint32_t )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	StartSevSeg =(uint8_t )atol((char* )pcParameterString2);

	status=SevenDisplayOneDigit(Number,StartSevSeg);

	if(status == H3BR2_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,Number);

	}

	else if(status == H3BR2_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	else if(status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongRangeMessage);


	return pdFALSE;
}

/*-----------------------------------------------------------*/
portBASE_TYPE CLI_SevenDisplayOneDigitHexaCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;
	char* letter=NULL;
		uint32_t hexadigit;
	uint8_t StartSevSeg;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	static const char *pcOKMessage=(int8_t* )"SevenSegmentDisplay is on:\r\n%c \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Number is out of range!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	 letter =(char* )pcParameterString1;
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	StartSevSeg =(uint8_t )atol((char* )pcParameterString2);
	hexadigit=get_hexadigit_code(letter[0]);


	status=SevenDisplayOneDigitHexa(hexadigit,StartSevSeg);

	if(status == H3BR2_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,letter[0]);

	}

	else if(status == H3BR2_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	else if(status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongRangeMessage);


	return pdFALSE;
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
