/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H3BR2.c
 Description: Main source code for H3BR2 module, controlling a 2-digit 7-segment display.
 Peripherals: Configures USART1-6 for module ports (P1-P6), uses software timer for 7-segment multiplexing.
 Features: Displays integers (0-99), floating-point numbers (0.0-9.9), hexadecimal values (0x00-0xFF),
           and single digits (decimal or hex) on 7-segment. Supports CLI commands, remote bootloader updates,
           and power modes (stop/standby).
*/

/* Includes ****************************************************************/
#include "BOS.h"
#include "H3BR2_inputs.h"
#include <ctype.h>

/* Exported Typedef ********************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

TimerHandle_t xTimerSwitch = NULL;
SegmentCodes Digit[2] = {EMPTY,EMPTY};

/* Private variables *******************************************************/
uint8_t StartSevSeg;
uint8_t SevenSegIndex = 0;
uint8_t FloatFlag = 0;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={0};

/* Private function prototypes *********************************************/
void SwitchTimerCallback(TimerHandle_t xTimerSwitch);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
uint8_t ClearROtopology(void);
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift);

/* Local function prototypes ***********************************************/
SegmentCodes GetNumberCode(uint8_t digit);
SegmentCodes ClearAllDigits(void);

/* Create CLI commands *****************************************************/
portBASE_TYPE CLI_DisplayNumberCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_DisplayOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_DisplayNumberHexaCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* CLI command structure ***************************************************/
/* CLI command structure : DisplayNumber */
const CLI_Command_Definition_t CLI_DisplayNumberCommandDefinition = {
	(const int8_t*) "display_num", /* The command string to type. */
	(const int8_t*) "display_num:\r\n Parameters required to execute a SevenDisplayNumber: Number  \r\n\r\n",
	CLI_DisplayNumberCommand, /* The function to run. */
	1 /* one parameter is expected. */
};

/***************************************************************************/
/* CLI command structure : DisplayNumberhexa */
const CLI_Command_Definition_t CLI_DisplayNumberHexaCommandDefinition = {
	(const int8_t*) "display_num_hexa", /* The command string to type. */
	(const int8_t*) "display_num_hexa:\r\n Parameters required to execute a SevenDisplayNumber: Number  \r\n\r\n",
	CLI_DisplayNumberHexaCommand, /* The function to run. */
    1 /* one parameter is expected. */
};

/***************************************************************************/
/* CLI command structure : DisplayOff */
const CLI_Command_Definition_t CLI_DisplayOffCommandDefinition = {
	(const int8_t*) "display_off", /* The command string to type. */
	(const int8_t*) "display_off:\r\nParameters required to execute a SevenDisplayOff \r\n\r\n",
	CLI_DisplayOffCommand, /* The function to run. */
    0 /* zero parameters are expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
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
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
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
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
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
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
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
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

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
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
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
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
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
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
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

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
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

/***************************************************************************/
/* H3BR2 module initialization */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	/* seven segment GPIO Init */
	SevenSegGPIOInit();

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* Init a timer for 7-seg */
	xTimerSwitch = xTimerCreate("SwitchTimer", pdMS_TO_TICKS(10), pdTRUE, (void*) 1, SwitchTimerCallback);
	xTimerStart(xTimerSwitch, 0);
}

/***************************************************************************/
/* H3BR2 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src,
		uint8_t dst, uint8_t shift){
	Module_Status result = H3BR2_OK;
	uint8_t decimal;
	uint32_t Number_int;
	float NumberF = 0;
	switch (code) {
	case CODE_H3BR2_DISPLAY_NUMBER:
		Number_int = ((uint32_t) cMessage[port - 1][shift]
				+ (uint32_t) (cMessage[port - 1][1 + shift] << 8)
				+ (uint32_t) (cMessage[port - 1][2 + shift] << 16)
				+ (uint32_t) (cMessage[port - 1][3 + shift] << 24));
		NumberF = *((float*) &Number_int);
		DisplayNumber(NumberF);
		break;

	case CODE_H3BR2_DISPLAY_NUMBER_HEXA:
		decimal = (uint32_t) cMessage[port - 1][shift];
		DisplayNumberHexa(decimal);
		break;

	case CODE_H3BR2_DISPLAY_OFF:
		DisplayOff();
		break;

	default:
		result = H3BR2_ERR_UNKNOWNMESSAGE;
		break;
	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P4;
	else if (huart->Instance == USART1)
		return P5;
	else if (huart->Instance == USART5)
		return P6;
	else if (huart->Instance == USART6)
		return P3;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&CLI_DisplayNumberCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_DisplayOffCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_DisplayNumberHexaCommandDefinition);
}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
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

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/*Switch timer callback */
void SwitchTimerCallback(TimerHandle_t xTimerSwitch) {

	HAL_GPIO_WritePin(CC1_GPIO_PORT, CC1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CC2_GPIO_PORT, CC2_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DP_GPIO_PORT, DP_PIN, 1);

	HAL_GPIO_WritePin(A_GPIO_PORT, A_PIN, Digit[SevenSegIndex] & 0b00000001);
	HAL_GPIO_WritePin(B_GPIO_PORT, B_PIN, Digit[SevenSegIndex] & 0b00000010);
	HAL_GPIO_WritePin(C_GPIO_PORT, C_PIN, Digit[SevenSegIndex] & 0b00000100);
	HAL_GPIO_WritePin(D_GPIO_PORT, D_PIN, Digit[SevenSegIndex] & 0b00001000);
	HAL_GPIO_WritePin(E_GPIO_PORT, E_PIN, Digit[SevenSegIndex] & 0b00010000);
	HAL_GPIO_WritePin(F_GPIO_PORT, F_PIN, Digit[SevenSegIndex] & 0b00100000);
	HAL_GPIO_WritePin(G_GPIO_PORT, G_PIN, Digit[SevenSegIndex] & 0b01000000);

	if (SevenSegIndex == 1 && FloatFlag == 1)
		HAL_GPIO_WritePin(DP_GPIO_PORT, DP_PIN, 0);
	else if (SevenSegIndex == 0 && FloatFlag == 1) {
		HAL_GPIO_WritePin(DP_GPIO_PORT, DP_PIN, 1);
	}

	switch (SevenSegIndex) {
	case 0:
		HAL_GPIO_WritePin(CC2_GPIO_PORT, CC2_PIN, GPIO_PIN_SET);
		break;

	case 1:
		HAL_GPIO_WritePin(CC1_GPIO_PORT, CC1_PIN, GPIO_PIN_SET);
		break;

	default:
		break;
	}

	SevenSegIndex++;
	if (SevenSegIndex > 1)
		SevenSegIndex = 0;

}

/***************************************************************************/
/*
 * GetNumberCode: Function to retrieve the display code for a given digit
 * digit: The numerical digit (0-9) to get the corresponding display code for
 */
SegmentCodes GetNumberCode(uint8_t digit) {
	Module_Status status = H3BR2_OK;

	SegmentCodes code;
	switch (digit) {
	case 0:
		code = ZERO_NUMBER;
		break;

	case 1:
		code = ONE_NUMBER;
		break;

	case 2:
		code = TWO_NUMBER;
		break;

	case 3:
		code = THREE_NUMBER;
		break;

	case 4:
		code = FOUR_NUMBER;
		break;

	case 5:
		code = FIVE_NUMBER;
		break;

	case 6:
		code = SIX_NUMBER;
		break;

	case 7:
		code = SEVEN_NUMBER;
		break;

	case 8:
		code = EIGHT_NUMBER;
		break;

	case 9:
		code = NINE_NUMBER;
		break;
	case 10:
		code = A_LETTER;
		break;
	case 11:
		code = B_LETTER;
		break;
	case 12:
		code = C_LETTER;
		break;
	case 13:
		code = D_LETTER;
		break;
	case 14:
		code = E_LETTER;
		break;
	case 15:
		code = F_LETTER;
		break;

	default:
		code = EMPTY;
		break;

	}
	return code;
}

/***************************************************************************/
/*
 * ClearAllDigits: Function to clear all digits on the Seven Segment display
 * In case of parameters {}
 */
SegmentCodes ClearAllDigits(void) {
	Module_Status status = H3BR2_OK;
	FloatFlag = 0;
	for (int i = 0; i < 2; i++)
		Digit[i] = EMPTY;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/*
 * DisplayNumber: Function to display a number on the Seven Segment display
 * Number: The decimal number to be displayed (float)
 * Display an integer value on the display, the range is from (0 to 99).
 * Display a float value on the display, the range is from (0 to 9.9).
 */
Module_Status DisplayNumber(float Number) {
	Module_Status status = H3BR2_OK;
	uint8_t Number_int=0;

	ClearAllDigits();

	if (fmod(Number, 1.0) == 0.0)
	{
		status=H3BR2_INT_NUMBER;
	}
	else
	{
		status =H3BR2_FLOAT_NUMBER ;
	}


	if ((Number >= 0 && Number <= 9)&&status==H3BR2_INT_NUMBER) {
		status =H3BR2_INT_ONEDIGIT ;
	}
	else if ((Number >= 10 && Number <= 99)&&status==H3BR2_INT_NUMBER) {
		status =H3BR2_INT_TWODIGIT ;
	}
	else if ((Number >= 0 && Number <= 9.9)&&status ==H3BR2_FLOAT_NUMBER) {
		status =H3BR2_FLOAT_NUMBER ;
	}
	else
	{
		status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
	}


	switch (status) {

		case H3BR2_INT_ONEDIGIT:
			FloatFlag = 0;
			Number_int=Number;
			Digit[0] = GetNumberCode(Number_int % 10);
			Digit[1] = EMPTY;
			status = H3BR2_OK;
			return status;
			break;

		case H3BR2_INT_TWODIGIT:
			FloatFlag = 0;
			Number_int=Number;
			Digit[0] = GetNumberCode(Number_int % 10);
			Number_int /= 10;
			Digit[1] = GetNumberCode(Number_int % 10);
			status = H3BR2_OK;
			return status;
			break;

		case H3BR2_FLOAT_NUMBER:
			FloatFlag = 1;
			Number_int = (uint8_t) (Number * 10);
			Digit[0] = GetNumberCode(Number_int % 10);
			Number_int /= 10;
			Digit[1] = GetNumberCode(Number_int % 10);
			status = H3BR2_OK;
			return status;
			break;

		case H3BR2_NUMBER_IS_OUT_OF_RANGE:
			FloatFlag = 0;
			Digit[0] =0XAF;
			Digit[1] =0X86;
			return status;
			break;

	}

}
/***************************************************************************/
/*
 * DisplayNumberHexa: Function to display a hexa value on the displa
 * decimal:the range is from (0 ~255)
 */
Module_Status DisplayNumberHexa(uint8_t decimal) {
	Module_Status status = H3BR2_OK;
	uint8_t low = 0, high = 0;
	char hex_str[4]={0};
	ClearAllDigits();

	if (decimal < 0 || decimal > 255) {
		status = H3BR2_NUMBER_IS_OUT_OF_RANGE;
		return status;
	}
	sprintf(hex_str, "0x%02X", decimal);

	 low = decimal & 0x0F;
	 high = (decimal & 0xF0) >> 4;


	Digit[0] = GetNumberCode(low);
	Digit[1] = GetNumberCode(high);

	HAL_Delay(5);
	return status;

}

/***************************************************************************/
/*
 * DisplayOff: Function to turn off the Seven Segment display
 */
Module_Status DisplayOff(void) {

	Module_Status status = H3BR2_OK;
	ClearAllDigits();   //Seven segment display off
	return status;

}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
portBASE_TYPE CLI_DisplayNumberCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H3BR2_OK;
	float Number=0;
	uint8_t Res=0;
	static uint8_t *pcParameterString1;

	portBASE_TYPE xParameterStringLength1 =0;
	static const int8_t *formatString[50];
	static const int8_t *pcOKMessage=(int8_t* )"SevenSegmentDisplay is on:\r\n %%0.%df \n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Number is out of range!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 = (int8_t* ) FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	Number = (float) atof((char*) pcParameterString1);

	status =DisplayNumber(Number);

	if (fmod(Number, 1.0) == 0.0)
		{
		Res=0;
		}
		else
		{
			Res=1;
		}

	if (status == H3BR2_OK) {
		sprintf((char* )formatString,(char* )pcOKMessage, Res);
		sprintf((char* )pcWriteBuffer,(char* )formatString,Number);
	}

	else if (status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
		strcpy((char*) pcWriteBuffer, (char*) pcWrongRangeMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_DisplayNumberHexaCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H3BR2_OK;
	uint8_t Number=0;
	char hex_str[4]={0};

	static uint8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	static const char *pcOKMessage = (int8_t*) "SevenSegmentDisplay is on:\r\n0x%c%c \n\r";
	static const int8_t *pcWrongRangeMessage = (int8_t*) "Number is out of range!\n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1);
	Number = (uint8_t) atoi((char*) pcParameterString1);

	status = DisplayNumberHexa(Number);

	sprintf(hex_str, "0x%02X", Number);

	if (status == H3BR2_OK) {
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage,hex_str[2] ,hex_str[3]);
	}

	else if (status == H3BR2_NUMBER_IS_OUT_OF_RANGE)
		strcpy((char*) pcWriteBuffer, (char*) pcWrongRangeMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_DisplayOffCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H3BR2_OK;

	static const int8_t *pcOKMessage = (int8_t*) "SevenSegmentDisplay is off \n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status = DisplayOff();

	if (status == H3BR2_OK) {
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage);

	}
	return pdFALSE;

}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
