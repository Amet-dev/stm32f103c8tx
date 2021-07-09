/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Modbus.h"
#include "semphr.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define AI1_Pin GPIO_PIN_0
#define AI1_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_1
#define AI2_GPIO_Port GPIOA
#define AI3_Pin GPIO_PIN_2
#define AI3_GPIO_Port GPIOA
#define AI4_Pin GPIO_PIN_3
#define AI4_GPIO_Port GPIOA
#define AI5_Pin GPIO_PIN_4
#define AI5_GPIO_Port GPIOA
#define DI1_Pin GPIO_PIN_10
#define DI1_GPIO_Port GPIOB
#define DI2_Pin GPIO_PIN_11
#define DI2_GPIO_Port GPIOB
#define DI3_Pin GPIO_PIN_12
#define DI3_GPIO_Port GPIOB
#define DI4_Pin GPIO_PIN_13
#define DI4_GPIO_Port GPIOB
#define DI5_Pin GPIO_PIN_14
#define DI5_GPIO_Port GPIOB
#define DI6_Pin GPIO_PIN_15
#define DI6_GPIO_Port GPIOB
#define DI7_Pin GPIO_PIN_8
#define DI7_GPIO_Port GPIOA
#define DO7_Pin GPIO_PIN_15
#define DO7_GPIO_Port GPIOA
#define DO1_Pin GPIO_PIN_3
#define DO1_GPIO_Port GPIOB
#define DO2_Pin GPIO_PIN_4
#define DO2_GPIO_Port GPIOB
#define DO3_Pin GPIO_PIN_6
#define DO3_GPIO_Port GPIOB
#define PWM_DO4_Pin GPIO_PIN_7
#define PWM_DO4_GPIO_Port GPIOB
#define PWM_DO5_Pin GPIO_PIN_8
#define PWM_DO5_GPIO_Port GPIOB
#define DO6_Pin GPIO_PIN_9
#define DO6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
