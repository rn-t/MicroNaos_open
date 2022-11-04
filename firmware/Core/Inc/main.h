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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f405xx.h"
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
#define DAC_Pin GPIO_PIN_4
#define DAC_GPIO_Port GPIOA
#define IR_LED1_Pin GPIO_PIN_5
#define IR_LED1_GPIO_Port GPIOA
#define IR_LED2_Pin GPIO_PIN_6
#define IR_LED2_GPIO_Port GPIOA
#define BATT_CHECK_Pin GPIO_PIN_7
#define BATT_CHECK_GPIO_Port GPIOA
#define MOTOR1_CW_Pin GPIO_PIN_4
#define MOTOR1_CW_GPIO_Port GPIOC
#define MOTOR_EN_Pin GPIO_PIN_5
#define MOTOR_EN_GPIO_Port GPIOC
#define MOTOR1_PWM_Pin GPIO_PIN_0
#define MOTOR1_PWM_GPIO_Port GPIOB
#define MOTOR_RESET_Pin GPIO_PIN_1
#define MOTOR_RESET_GPIO_Port GPIOB
#define MOTOR_DMODE2_Pin GPIO_PIN_2
#define MOTOR_DMODE2_GPIO_Port GPIOB
#define MOTOR_DMODE1_Pin GPIO_PIN_10
#define MOTOR_DMODE1_GPIO_Port GPIOB
#define MOTOR_DMODE0_Pin GPIO_PIN_11
#define MOTOR_DMODE0_GPIO_Port GPIOB
#define MOTOR1_ERR_Pin GPIO_PIN_14
#define MOTOR1_ERR_GPIO_Port GPIOB
#define MOTOR2_ERR_Pin GPIO_PIN_15
#define MOTOR2_ERR_GPIO_Port GPIOB
#define MOTOR2_PWM_Pin GPIO_PIN_6
#define MOTOR2_PWM_GPIO_Port GPIOC
#define MOTOR2_CW_Pin GPIO_PIN_7
#define MOTOR2_CW_GPIO_Port GPIOC
#define GYRO_IN_Pin GPIO_PIN_15
#define GYRO_IN_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_2
#define SPI_CS_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_3
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_6
#define SW_1_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_7
#define SW_2_GPIO_Port GPIOB
#define SPEAKER_Pin GPIO_PIN_8
#define SPEAKER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
