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
#include "stm32f4xx_hal.h"

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
/* Adafruit Feather STM32F405 Express, pins from Adafruit's schematic and
   CircuitPython's board file (feather_stm32f405_express/pins.c). */
#define LED_Pin GPIO_PIN_1              /* D13, red LED: on while PPS-locked */
#define LED_GPIO_Port GPIOC
#define W5500_RST_Pin GPIO_PIN_2        /* D12 <- Ethernet FeatherWing RESET pad */
#define W5500_RST_GPIO_Port GPIOC
#define W5500_INT_Pin GPIO_PIN_3        /* D11 <- Ethernet FeatherWing IRQ pad */
#define W5500_INT_GPIO_Port GPIOC
#define W5500_INT_EXTI_IRQn EXTI3_IRQn
#define GPS_PPS_Pin GPIO_PIN_5          /* A1, TIM2_CH1 <- GPS FeatherWing 1PPS pad */
#define GPS_PPS_GPIO_Port GPIOA
#define RTC_SQW_Pin GPIO_PIN_6          /* A2, TIM3_CH1 <- DS3231 FeatherWing SQW pad */
#define RTC_SQW_GPIO_Port GPIOA
#define BTN_A_Pin GPIO_PIN_8            /* D9, OLED FeatherWing button A */
#define BTN_A_GPIO_Port GPIOB
#define W5500_CS_Pin GPIO_PIN_9         /* D10, Ethernet FeatherWing CS (SJ4) */
#define W5500_CS_GPIO_Port GPIOB
#define BTN_B_Pin GPIO_PIN_6            /* D6, OLED FeatherWing button B */
#define BTN_B_GPIO_Port GPIOC
#define BTN_C_Pin GPIO_PIN_7            /* D5, OLED FeatherWing button C */
#define BTN_C_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
