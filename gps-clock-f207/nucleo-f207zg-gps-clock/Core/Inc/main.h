/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.cpp file.
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
#include "stm32f2xx_hal.h"

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
/* NUCLEO-F207ZG with an X-NUCLEO-GNSS2A1 (Teseo-VIC3DA) on its Arduino
   connector. The Nucleo's pins (LEDs, ST-LINK virtual COM port, Arduino
   D0..D15) are from memory of ST's UM1974, not checked here; the shield's
   jumpers from ST's UM3020 and X-CUBE-GNSS1 readme. The Ethernet pins are
   in LWIP/Target/ethernetif.c (ST's example). */
#define LD1_Pin GPIO_PIN_0              /* green: locked to the PPS */
#define LD1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7              /* blue: the network has an address */
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14             /* red: a fault (Error_Handler, no MAC) */
#define LD3_GPIO_Port GPIOB
#define GNSS_PPS_Pin GPIO_PIN_9         /* D6, TIM1_CH1 <- shield PPS (J29 1-2) */
#define GNSS_PPS_GPIO_Port GPIOE
#define GNSS_RST_Pin GPIO_PIN_13        /* D7 -> shield reset, low = reset (J25 1-2) */
#define GNSS_RST_GPIO_Port GPIOF
#define GNSS_RX_Pin GPIO_PIN_9          /* D0, USART6_RX <- shield TX (J28) */
#define GNSS_RX_GPIO_Port GPIOG
#define GNSS_TX_Pin GPIO_PIN_14         /* D1, USART6_TX -> shield RX (J27) */
#define GNSS_TX_GPIO_Port GPIOG
#define VCP_TX_Pin GPIO_PIN_8           /* USART3 -> ST-LINK virtual COM port */
#define VCP_TX_GPIO_Port GPIOD
#define VCP_RX_Pin GPIO_PIN_9
#define VCP_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
