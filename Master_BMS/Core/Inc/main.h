/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define TEMP2_Pin GPIO_PIN_2
#define TEMP2_GPIO_Port GPIOC
#define TEMP3_Pin GPIO_PIN_3
#define TEMP3_GPIO_Port GPIOC
#define TEMP1_Pin GPIO_PIN_0
#define TEMP1_GPIO_Port GPIOA
#define SPI1_NSS1_Pin GPIO_PIN_4
#define SPI1_NSS1_GPIO_Port GPIOA
#define SPI1_NSS2_Pin GPIO_PIN_5
#define SPI1_NSS2_GPIO_Port GPIOA
#define TEMP4_Pin GPIO_PIN_4
#define TEMP4_GPIO_Port GPIOC
#define TEMP5_Pin GPIO_PIN_5
#define TEMP5_GPIO_Port GPIOC
#define CHG_Pin GPIO_PIN_0
#define CHG_GPIO_Port GPIOB
#define DSG_Pin GPIO_PIN_1
#define DSG_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_6
#define A0_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_7
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_8
#define A2_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_9
#define A3_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
