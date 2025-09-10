/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main,c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// variables que gobiernan el encoder
uint8_t data                = 0;
uint8_t casoRGB             = 0; //variable auxiliar para entrar a cada caso de posibles colores del led RGB

//variables usadas para el display
uint16_t numero             = 0;   // Este es el numero general que se vera reflejado en el display completo y el que aumenta o disminuye girando el encoder
uint8_t baseT               = 0;   // Variable auxiliar para entrar en cada caso de prender alguno de los 4 displays abriendo la base de sus transistores correspondientes

// Variables para la descomposicion de "numero" en 4 numeros para cada 7 segmentos
uint8_t unidades            = 0;
uint8_t decenas             = 0;
uint8_t	centenas            = 0;
uint8_t miles               = 0;

fsm_states_t fsm_taximetro = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void refreshDisplay(uint8_t numero); //Refresco de los 4 digitos mostrados en el display que corresponden cada uno a un 7 segmentos
void displayNumber(uint8_t n);       //Casos que pintan en un 7 segmentos un numero del 0 al 9
void getDigit(uint16_t numero);      //Descomposicion de "numero" en 4 digitos : unidad, decena, centena, mil.
void colorRGB (uint8_t y);           //Casos del LedRGB
e_PosibleStates state_machine_action(uint8_t event);    //Maquina de estados
void initProgram(void);              //Función que inicializa la maquina de estados en el estado IDLE (espera)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	initProgram();
	HAL_TIM_Base_Start_IT(&htim2); // Activa el timer y la interrupción del mismo
	HAL_TIM_Base_Start_IT(&htim3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		state_machine_action(fsm_taximetro.state);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 250-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16000-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 5-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, D3_Pin|D1_Pin|D2_Pin|BLUE_Pin
			|GREEN_Pin|userLedD_Pin|userLedC_Pin|userLedE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(userLed_GPIO_Port, userLed_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|userLedF_Pin|userLedB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, userLedA_Pin|D4_Pin|RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(userLedG_GPIO_Port, userLedG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : D3_Pin */
	GPIO_InitStruct.Pin = D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : userLed_Pin */
	GPIO_InitStruct.Pin = userLed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(userLed_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : D1_Pin D2_Pin */
	GPIO_InitStruct.Pin = D1_Pin|D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : DT_Pin */
	GPIO_InitStruct.Pin = DT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : userClk_Pin SW_Pin */
	GPIO_InitStruct.Pin = userClk_Pin|SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : userLedA_Pin RED_Pin */
	GPIO_InitStruct.Pin = userLedA_Pin|RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : BLUE_Pin GREEN_Pin userLedD_Pin userLedC_Pin */
	GPIO_InitStruct.Pin = BLUE_Pin|GREEN_Pin|userLedD_Pin|userLedC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : userLedF_Pin userLedB_Pin */
	GPIO_InitStruct.Pin = userLedF_Pin|userLedB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : userLedE_Pin */
	GPIO_InitStruct.Pin = userLedE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(userLedE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : userLedG_Pin */
	GPIO_InitStruct.Pin = userLedG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(userLedG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : D4_Pin */
	GPIO_InitStruct.Pin = D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
e_PosibleStates state_machine_action(uint8_t event){

	switch(fsm_taximetro.state){

	case IDLE:{
		__NOP();
		break;
	}


	case ENCODER:{
		GPIO_PinState data = HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin);
		if (data == 0){ //Revisamos si fue un flanco de subida o de bajada

			if (numero == 4095){   //Si el numero es 4095 debemos devolvernos al 0
				numero=0;
			}

			else{       // de lo contrario sumamos 1 a la variable "numero"
				numero ++;
			}
		}

		else{             //en caso contrario vamos a restar 1 a la variable "numero"
			if (numero ==0){         //si "numero" es 0 debemos llegar a 4095 que es el limte para una variable de 12 Bits
				numero=4095;
			}

			else{
				numero --;
			}

		}

		fsm_taximetro.state = IDLE;
		break;
	}

	case SWITCH:{
		casoRGB ++;    //pasamos al siguiente caso de Led RGB
		if (casoRGB >7){  //Reiniciamos casoRGB cuando llega al ultimo para así volver a empezar por el primer caso
			casoRGB = 0;

		}
		colorRGB(casoRGB);
		fsm_taximetro.state = IDLE;
		break;
	}

	case REFRESH:{
		baseT ++;         //Entramos al siguiente digito activando la base del transistor que sigue, entrando en los casos de la funcion "refreshDisplay"
		if (baseT >3){    //Reiniciamos "baseT" cuando llega al ultimo caso y asi volvemos a empezar por el primer digito (D4)
			baseT = 0;
		}
		getDigit(numero);        //obtenemos los 4 digitos necesarios (unidad,decena,centena,mil)  a partir de "numero"
		refreshDisplay(baseT);
		fsm_taximetro.state = IDLE;
		break;
	}


	}
	return fsm_taximetro.state;

}

//Descomposicion de "numero" en 4 digitos : unidad, decena, centena, mil.
void getDigit(uint16_t numero){

	unidades = (((numero %1000) %100) %10);
	decenas = ((numero % 1000) %100) / 10;
	centenas = (numero % 1000) /100;
	miles = numero /1000;

}


//Casos que pintan en un 7 segmentos un numero del 0 al 9
void displayNumber(uint8_t n){
	HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

	switch(n){
	case 0:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_SET);

		break;
	}
	case 1:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_SET);

		break;
	}
	case 2:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	case 3:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	case 4:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	case 5:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	case 6:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	case 7:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_SET);
		break;
	}
	case 8:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	case 9:{
		HAL_GPIO_WritePin(GPIOB, userLedA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, userLedB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, userLedD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, userLedE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, userLedF_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, userLedG_Pin, GPIO_PIN_RESET);

		break;
	}
	}
}


//Refresco de los 4 digitos mostrados en el display que corresponden cada uno a un 7 segmentos. En esta función primero se apagan todos los digitos y luego se prende en orden del primero al ultimo activando o desactivando sus transistores correspondientes
void refreshDisplay (uint8_t n){


	switch(n){
	case 0:{
		HAL_GPIO_WritePin(GPIOC, D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
		displayNumber(unidades);
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_RESET);

		break;
	}
	case 1:{
		HAL_GPIO_WritePin(GPIOC, D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
		displayNumber(decenas);
		HAL_GPIO_WritePin(GPIOC, D3_Pin, GPIO_PIN_RESET);
		break;
	}
	case 2:{
		HAL_GPIO_WritePin(GPIOC, D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
		displayNumber(centenas);
		HAL_GPIO_WritePin(GPIOC, D2_Pin, GPIO_PIN_RESET);
		break;
	}
	case 3:{
		HAL_GPIO_WritePin(GPIOC, D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, D3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
		displayNumber(miles);
		HAL_GPIO_WritePin(GPIOC, D1_Pin, GPIO_PIN_RESET);
		break;
	}
	}


}


//Casos del Led RGB
void colorRGB (uint8_t casoRGB){
	HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);

	switch (casoRGB){

	case 0:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);
		break;
	}
	case 1:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);
		break;
	}
	case 2:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);
		break;
	}
	case 3:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);
		break;
	}
	case 4:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_RESET);
		break;
	}
	case 5:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);
		break;
	}
	case 6:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);
		break;
	}
	case 7:{
		HAL_GPIO_WritePin(GPIOC, GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, BLUE_Pin, GPIO_PIN_SET);
		break;
	}
	}

}
void initProgram (void){            //Inicia la maquina de estados en el estado IDLE

	fsm_taximetro.state = IDLE;
}
//Callbacks

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){ // cada 5ms
		fsm_taximetro.state = REFRESH;// Código cuando se desborda TIM2
	}
	else if(htim->Instance == TIM2){ // cada 250 ms
		HAL_GPIO_TogglePin(userLed_GPIO_Port, userLed_Pin);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == userClk_Pin){
		fsm_taximetro.state = ENCODER;// interrupción externa con la señal de reloj
	}
	if(GPIO_Pin == SW_Pin)
	{
		fsm_taximetro.state = SWITCH;
	}
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
