/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : GPS clock example for the STM32L432KC
  *
  *  DS1307 RTC + SSD1306 128x64 OLED on I2C1, MTK3333 GPS on USART1.
  *  The time is read from the DS1307 and shown on the SSD1306. Once the
  *  GPS has a valid fix the DS1307 is set from GPS (UTC) time.
  *
  *  Wiring (NUCLEO-L432KC):
  *    I2C1_SCL  PA9  (D1)  -> DS1307 SCL, SSD1306 SCL
  *    I2C1_SDA  PA10 (D0)  -> DS1307 SDA, SSD1306 SDA
  *    USART1_TX PB6  (D5)  -> GPS RX
  *    USART1_RX PB7  (D4)  <- GPS TX
  *    USART2_TX PA2  (A7)  -> ST-LINK virtual COM port (debug, 115200)
  *    USART2_RX PA15       <- ST-LINK virtual COM port
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "RTClib.h"
#include "SSD1306.h"
#include "MTK3333.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The DS1307 is kept in UTC. Set this to show local (standard) time on the display.
#define UTC_OFFSET_MINUTES      0

// Re-sync the DS1307 from GPS this often once it has been set
#define RTC_RESYNC_INTERVAL_MS  (60UL * 60UL * 1000UL)

// "GPS no data" is shown if no RMC sentence arrives for this long
#define GPS_TIMEOUT_MS          3000UL

#define I2C_PROBE_TIMEOUT_MS    10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for i2c1_mutex */
osMutexId_t i2c1_mutexHandle;
const osMutexAttr_t i2c1_mutex_attributes = {
  .name = "i2c1_mutex"
};
/* Definitions for gps_data_mutex */
osMutexId_t gps_data_mutexHandle;
const osMutexAttr_t gps_data_mutex_attributes = {
  .name = "gps_data_mutex"
};
/* USER CODE BEGIN PV */
RTC_DS1307 rtc(&hi2c1);
SSD1306 oled(&hi2c1, SSD1306_I2C_ADDR, 128, 64);
MTK3333 gps(&huart1);

// Shared between gpsTask (writer) and displayTask (reader), guarded by gps_data_mutex
static GPS_Data gpsSnapshot;
static uint32_t gpsLastRmcTick = 0;     // 0 = never received
static bool rtcSynced = false;
static DateTime rtcLastSync;            // UTC time of the last GPS -> RTC sync

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDisplayTask(void *argument);
void StartGpsTask(void *argument);

/* USER CODE BEGIN PFP */
static void debugPrint(const char *str);
static void drawScreen(bool rtcOk, bool rtcRunning, const DateTime &utc, const GPS_Data &g,
                       bool gpsAlive, bool synced, const DateTime &lastSync);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *const dayNames[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

static void debugPrint(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
}

/* HAL UART callbacks are C functions: they must have C linkage to replace the weak HAL versions */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  gps.rxCpltCallback(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  gps.errorCallback(huart);
}

// "37.77490N 122.41940W" from micro-degrees, without printf float support
static void formatPosition(char *out, size_t len, int32_t latE6, int32_t lonE6)
{
  long lat = labs(latE6), lon = labs(lonE6);
  snprintf(out, len, "%ld.%05ld%c %ld.%05ld%c",
           lat / 1000000L, (lat % 1000000L) / 10, latE6 < 0 ? 'S' : 'N',
           lon / 1000000L, (lon % 1000000L) / 10, lonE6 < 0 ? 'W' : 'E');
}

static void drawScreen(bool rtcOk, bool rtcRunning, const DateTime &utc, const GPS_Data &g,
                       bool gpsAlive, bool synced, const DateTime &lastSync)
{
  char line[32];

  oled.clear();

  // Line 1: GPS status
  if (!gpsAlive)
    snprintf(line, sizeof(line), "GPS: no data");
  else if (g.fix)
    snprintf(line, sizeof(line), "GPS: fix  %u sats", g.satellites);
  else
    snprintf(line, sizeof(line), "GPS: no fix %u sats", g.satellites);
  oled.drawString(0, 0, line);
  oled.drawHLine(0, 9, oled.width());

  if (rtcOk) {
    DateTime local = utc + TimeSpan((int32_t) UTC_OFFSET_MINUTES * 60);

    // Time, double size
    snprintf(line, sizeof(line), "%02d:%02d:%02d", local.hour(), local.minute(), local.second());
    oled.drawStringCentered(14, line, 2);

    // Date
    snprintf(line, sizeof(line), "%s %04d-%02d-%02d %s", dayNames[local.dayOfTheWeek()],
             local.year(), local.month(), local.day(), UTC_OFFSET_MINUTES == 0 ? "UTC" : "");
    oled.drawStringCentered(32, line);
  } else {
    oled.drawStringCentered(14, "--:--:--", 2);
    oled.drawStringCentered(32, "DS1307 not found");
  }

  // Position
  if (gpsAlive && g.fix) {
    formatPosition(line, sizeof(line), g.latitudeE6, g.longitudeE6);
    oled.drawStringCentered(44, line);
  }

  // RTC sync status
  if (synced)
    snprintf(line, sizeof(line), "RTC synced %02d:%02dZ", lastSync.hour(), lastSync.minute());
  else if (rtcOk && !rtcRunning)
    snprintf(line, sizeof(line), "RTC stopped, wait GPS");
  else
    snprintf(line, sizeof(line), "RTC not synced");
  oled.drawString(0, 56, line);
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  debugPrint("\r\nSTM32L432KC GPS clock: DS1307 + SSD1306 + MTK3333\r\n");

  rtc.begin(DS1307_ADDR);
  if (!oled.begin())
    debugPrint("SSD1306 not found at 0x3C\r\n");

  gps.begin();
  gps.setNmeaOutput(true, true);   // RMC + GGA only
  gps.setUpdateRate(1000);         // 1 Hz
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c1_mutex */
  i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes);

  /* creation of gps_data_mutex */
  gps_data_mutexHandle = osMutexNew(&gps_data_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function (GPS)
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function (debug, ST-LINK VCP)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDisplayTask */
/**
  * @brief  Reads the DS1307 and redraws the SSD1306 every time the second changes.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  char msg[96];
  int lastSecond = -1;
  bool lastRtcOk = true;

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(i2c1_mutexHandle, osWaitForever);
    bool rtcOk = HAL_I2C_IsDeviceReady(&hi2c1, DS1307_ADDR << 1, 2, I2C_PROBE_TIMEOUT_MS) == HAL_OK;
    bool rtcRunning = rtcOk && rtc.isrunning();
    DateTime now = rtcOk ? rtc.now() : DateTime(SECONDS_FROM_1970_TO_2000);
    osMutexRelease(i2c1_mutexHandle);

    if (now.second() != lastSecond || rtcOk != lastRtcOk) {
      lastSecond = now.second();
      lastRtcOk = rtcOk;

      GPS_Data g;
      bool synced;
      DateTime lastSync;
      osMutexAcquire(gps_data_mutexHandle, osWaitForever);
      g = gpsSnapshot;
      bool gpsAlive = gpsLastRmcTick != 0 && (osKernelGetTickCount() - gpsLastRmcTick) < GPS_TIMEOUT_MS;
      synced = rtcSynced;
      lastSync = rtcLastSync;
      osMutexRelease(gps_data_mutexHandle);

      drawScreen(rtcOk, rtcRunning, now, g, gpsAlive, synced, lastSync);

      osMutexAcquire(i2c1_mutexHandle, osWaitForever);
      oled.display();
      osMutexRelease(i2c1_mutexHandle);

      snprintf(msg, sizeof(msg), "RTC %04d-%02d-%02d %02d:%02d:%02d UTC | GPS %s sats=%u%s\r\n",
               now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(),
               !gpsAlive ? "no data" : (g.fix ? "fix" : "no fix"), g.satellites,
               rtcOk ? "" : " | DS1307 not found");
      debugPrint(msg);
    }
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
  * @brief  Parses NMEA from the GPS and sets the DS1307 once the GPS has a fix.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  GPS_Data g;
  uint32_t lastSyncTick = 0;
  bool synced = false;

  /* Infinite loop */
  for(;;)
  {
    gps.update();

    if (gps.newData()) {
      gps.getData(&g);

      osMutexAcquire(gps_data_mutexHandle, osWaitForever);
      gpsSnapshot = g;
      gpsLastRmcTick = osKernelGetTickCount();
      if (gpsLastRmcTick == 0)
        gpsLastRmcTick = 1;
      osMutexRelease(gps_data_mutexHandle);

      // Set the DS1307 from GPS (UTC) on the first valid fix, then once per interval
      if (g.dateValid && (!synced || (osKernelGetTickCount() - lastSyncTick) >= RTC_RESYNC_INTERVAL_MS)) {
        DateTime gpsTime(g.year, g.month, g.day, g.hour, g.minute, g.second);

        osMutexAcquire(i2c1_mutexHandle, osWaitForever);
        bool rtcOk = HAL_I2C_IsDeviceReady(&hi2c1, DS1307_ADDR << 1, 2, I2C_PROBE_TIMEOUT_MS) == HAL_OK;
        if (rtcOk)
          rtc.adjust(gpsTime);   // also clears the clock-halt bit
        osMutexRelease(i2c1_mutexHandle);

        if (rtcOk) {
          synced = true;
          lastSyncTick = osKernelGetTickCount();
          osMutexAcquire(gps_data_mutexHandle, osWaitForever);
          rtcSynced = true;
          rtcLastSync = gpsTime;
          osMutexRelease(gps_data_mutexHandle);
        }
      }
    }
    osDelay(20);
  }
  /* USER CODE END StartGpsTask */
}

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
#ifdef USE_FULL_ASSERT
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
