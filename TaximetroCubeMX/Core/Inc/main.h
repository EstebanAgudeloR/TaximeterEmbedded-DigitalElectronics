/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum    //Casos de la maquina de estados
{
	IDLE,
	REFRESH,
	ENCODER,
	SWITCH
} e_PosibleStates;

typedef struct
{
	e_PosibleStates state;

}fsm_states_t;

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D3_Pin GPIO_PIN_13
#define D3_GPIO_Port GPIOC
#define userLed_Pin GPIO_PIN_1
#define userLed_GPIO_Port GPIOH
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOC
#define DT_Pin GPIO_PIN_1
#define DT_GPIO_Port GPIOB
#define userClk_Pin GPIO_PIN_2
#define userClk_GPIO_Port GPIOB
#define userClk_EXTI_IRQn EXTI2_IRQn
#define userLedA_Pin GPIO_PIN_12
#define userLedA_GPIO_Port GPIOB
#define SW_Pin GPIO_PIN_15
#define SW_GPIO_Port GPIOB
#define SW_EXTI_IRQn EXTI15_10_IRQn
#define D2_Pin GPIO_PIN_6
#define D2_GPIO_Port GPIOC
#define BLUE_Pin GPIO_PIN_8
#define BLUE_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_9
#define GREEN_GPIO_Port GPIOC
#define userLedF_Pin GPIO_PIN_11
#define userLedF_GPIO_Port GPIOA
#define userLedB_Pin GPIO_PIN_12
#define userLedB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define userLedD_Pin GPIO_PIN_10
#define userLedD_GPIO_Port GPIOC
#define userLedC_Pin GPIO_PIN_11
#define userLedC_GPIO_Port GPIOC
#define userLedE_Pin GPIO_PIN_12
#define userLedE_GPIO_Port GPIOC
#define userLedG_Pin GPIO_PIN_2
#define userLedG_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_8
#define RED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
