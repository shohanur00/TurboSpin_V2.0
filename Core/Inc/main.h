/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart3;

extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern TIM_HandleTypeDef htim17;  // <-- Declare here for global access
extern volatile uint16_t adc1_buffer[2]; // ADC1
extern volatile uint16_t adc2_buffer[4]; // ADC2
extern volatile int32_t c_asense_adc;
extern volatile int32_t c_bsense_adc;
extern volatile int32_t c_csense_adc;
extern uint16_t voltage_a;
extern uint16_t voltage_b;
extern uint16_t voltage_c;
extern int16_t current_a;
extern int16_t current_b;
extern int16_t current_c;
extern int16_t temperature;
extern int16_t bus_voltage;
extern int16_t total_current;

extern volatile uint8_t updateFlag;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define Temp_S_Pin GPIO_PIN_1
#define Temp_S_GPIO_Port GPIOA
#define C_ASENSE_Pin GPIO_PIN_2
#define C_ASENSE_GPIO_Port GPIOA
#define C_BSENSE_Pin GPIO_PIN_3
#define C_BSENSE_GPIO_Port GPIOA
#define C_CSENSE_Pin GPIO_PIN_4
#define C_CSENSE_GPIO_Port GPIOA
#define Current_S_Pin GPIO_PIN_5
#define Current_S_GPIO_Port GPIOA
#define PVDD_S_Pin GPIO_PIN_6
#define PVDD_S_GPIO_Port GPIOA
#define ASENSE_Pin GPIO_PIN_7
#define ASENSE_GPIO_Port GPIOA
#define BSENSE_Pin GPIO_PIN_0
#define BSENSE_GPIO_Port GPIOB
#define CSENSE_Pin GPIO_PIN_1
#define CSENSE_GPIO_Port GPIOB
#define Filter_SW_Pin GPIO_PIN_2
#define Filter_SW_GPIO_Port GPIOB
#define NFAULT_Pin GPIO_PIN_10
#define NFAULT_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_12
#define EN_GATE_GPIO_Port GPIOB
#define INL_A_Pin GPIO_PIN_13
#define INL_A_GPIO_Port GPIOB
#define INL_B_Pin GPIO_PIN_14
#define INL_B_GPIO_Port GPIOB
#define INL_C_Pin GPIO_PIN_15
#define INL_C_GPIO_Port GPIOB
#define INH_A_Pin GPIO_PIN_8
#define INH_A_GPIO_Port GPIOA
#define INH_B_Pin GPIO_PIN_9
#define INH_B_GPIO_Port GPIOA
#define INH_C_Pin GPIO_PIN_10
#define INH_C_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
#define DSHOT_Pin GPIO_PIN_6
#define DSHOT_GPIO_Port GPIOB
#define NOCTW_Pin GPIO_PIN_7
#define NOCTW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
