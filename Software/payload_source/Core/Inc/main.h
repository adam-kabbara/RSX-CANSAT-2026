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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_nucleo.h"
#include <stdio.h>

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define G_CAM_OUT_Pin GPIO_PIN_0
#define G_CAM_OUT_GPIO_Port GPIOF
#define PG_CAM_OUT_Pin GPIO_PIN_1
#define PG_CAM_OUT_GPIO_Port GPIOF
#define SERVO_WING_DIR_Pin GPIO_PIN_0
#define SERVO_WING_DIR_GPIO_Port GPIOA
#define SERVO_WING_PWM_Pin GPIO_PIN_1
#define SERVO_WING_PWM_GPIO_Port GPIOA
#define SPI_CS_GPIO_OUT_Pin GPIO_PIN_4
#define SPI_CS_GPIO_OUT_GPIO_Port GPIOA
#define SERVO_EGG_REL_Pin GPIO_PIN_0
#define SERVO_EGG_REL_GPIO_Port GPIOB
#define PG_CAM_IN_Pin GPIO_PIN_8
#define PG_CAM_IN_GPIO_Port GPIOA
#define XBEE_TX_Pin GPIO_PIN_9
#define XBEE_TX_GPIO_Port GPIOA
#define XBEE_RX_Pin GPIO_PIN_10
#define XBEE_RX_GPIO_Port GPIOA
#define SERVO_CC_REL_Pin GPIO_PIN_11
#define SERVO_CC_REL_GPIO_Port GPIOA
#define SERVO_NC_REL_Pin GPIO_PIN_12
#define SERVO_NC_REL_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_PIN_15
#define I2C_SCL_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define SERVO_ELEVATOR_Pin GPIO_PIN_4
#define SERVO_ELEVATOR_GPIO_Port GPIOB
#define SERVO_AILERON_Pin GPIO_PIN_5
#define SERVO_AILERON_GPIO_Port GPIOB
#define G_CAM_IN_Pin GPIO_PIN_6
#define G_CAM_IN_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
