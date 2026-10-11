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
  *  The SSD1306 and the GPS use the non-blocking drivers from
  *  Claude-iTransport (see iTransport/SOURCE.txt):
  *    xssd1306<Stm32HalI2CTransport>   iDisplay/ssd1306
  *    xmtk3339<Stm32HalUartTransport>  isensor/mtk3339 (the MT3333 speaks the
  *                                     same NMEA/PMTK as the MT3339)
  *  The DS1307 still uses RTC-lib's blocking HAL calls, so it takes
  *  i2c1_mutex itself, the same mutex the SSD1306's transport holds for
  *  each of its transfers.
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
#include "Stm32HalI2CTransport.h"
#include "Stm32HalUartTransport.h"
#include "xssd1306.h"
#include "xmtk3339.h"
#include "Font5x7.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef SSD1306::xssd1306<Stm32HalI2CTransport, 64> OledDisplay;
typedef xmtk3339<Stm32HalUartTransport> GpsReceiver;

// What displayTask needs from the GPS, copied out by gpsTask
typedef struct {
  bool     alive;          // the GPS is sending good NMEA
  bool     fix;
  uint8_t  satellites;     // used in the fix (GGA)
  int32_t  latitudeE7;     // 1e-7 degrees
  int32_t  longitudeE7;
  mtk3339_config_status_t config;
} GpsStatus;
/* USER CODE END PTD *//* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The DS1307 is kept in UTC. Set this to show local (standard) time on the display.
#define UTC_OFFSET_MINUTES      0

// Re-sync the DS1307 from GPS this often once it has been set
#define RTC_RESYNC_INTERVAL_MS  (60UL * 60UL * 1000UL)

#define I2C_PROBE_TIMEOUT_MS    10
// Bounded, never osWaitForever: displayTask also owns the SSD1306, whose
// transport holds this mutex while a transfer is in flight
#define I2C_MUTEX_TIMEOUT_MS    100
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

// Shared between gpsTask (writer) and displayTask (reader), guarded by gps_data_mutex
static GpsStatus gpsStatus;
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
static void drawScreen(idisplay::MonoCanvas &c, bool rtcOk, bool rtcRunning, const DateTime &utc,
                       const GpsStatus &g, bool synced, const DateTime &lastSync);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *const dayNames[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

// The HAL UART and I2C interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32UartItCallbacks.cpp, Stm32I2CItCallbacks.cpp): don't define them here.

static void debugPrint(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
}

static const char *configName(mtk3339_config_status_t s)
{
  switch (s) {
  case mtk3339_config_status_t::Done:     return "done";
  case mtk3339_config_status_t::Rejected: return "rejected";
  case mtk3339_config_status_t::NoAnswer: return "no answer";
  default:                                return "pending";
  }
}

// "37.77490N 122.41940W" from 1e-7 degrees, without printf float support
static void formatPosition(char *out, size_t len, int32_t latE7, int32_t lonE7)
{
  long lat = labs(latE7), lon = labs(lonE7);
  snprintf(out, len, "%ld.%05ld%c %ld.%05ld%c",
           lat / 10000000L, (lat % 10000000L) / 100, latE7 < 0 ? 'S' : 'N',
           lon / 10000000L, (lon % 10000000L) / 100, lonE7 < 0 ? 'W' : 'E');
}

// MonoCanvas draws text in 6x8 cells; this draws it `scale` times larger
static void drawTextScaled(idisplay::MonoCanvas &c, int16_t x, int16_t y, const char *s, uint8_t scale)
{
  for (; *s; ++s, x += (int16_t)(idisplay::MonoCanvas::kCellWidth * scale)) {
    const uint8_t *glyph = idisplay::fontGlyph(*s);
    for (uint8_t col = 0; col < idisplay::kFontWidth; ++col)
      for (uint8_t row = 0; row < idisplay::kFontHeight; ++row)
        if (glyph[col] & (1u << row))
          c.fillRect(x + col * scale, y + row * scale, scale, scale, true);
  }
}

static int16_t centeredX(const idisplay::MonoCanvas &c, const char *s, uint8_t scale)
{
  // the blank column after the last character is not counted
  int16_t w = (int16_t)((strlen(s) * idisplay::MonoCanvas::kCellWidth - 1) * scale);
  return w >= (int16_t) c.width() ? 0 : (int16_t)((c.width() - w) / 2);
}

static void drawCentered(idisplay::MonoCanvas &c, int16_t y, const char *s, uint8_t scale = 1)
{
  if (scale == 1)
    c.drawText(centeredX(c, s, 1), y, s, false);
  else
    drawTextScaled(c, centeredX(c, s, scale), y, s, scale);
}

static void drawScreen(idisplay::MonoCanvas &c, bool rtcOk, bool rtcRunning, const DateTime &utc,
                       const GpsStatus &g, bool synced, const DateTime &lastSync)
{
  char line[32];

  c.clear();

  // Line 1: GPS status
  if (!g.alive)
    snprintf(line, sizeof(line), "GPS: no data");
  else if (g.fix)
    snprintf(line, sizeof(line), "GPS: fix  %u sats", g.satellites);
  else
    snprintf(line, sizeof(line), "GPS: no fix %u sats", g.satellites);
  c.drawText(0, 0, line, false);
  c.drawHLine(0, 9, c.width(), true);

  if (rtcOk) {
    DateTime local = utc + TimeSpan((int32_t) UTC_OFFSET_MINUTES * 60);

    // Time, double size
    snprintf(line, sizeof(line), "%02d:%02d:%02d", local.hour(), local.minute(), local.second());
    drawCentered(c, 14, line, 2);

    // Date
    snprintf(line, sizeof(line), "%s %04d-%02d-%02d%s", dayNames[local.dayOfTheWeek()],
             local.year(), local.month(), local.day(), UTC_OFFSET_MINUTES == 0 ? " UTC" : "");
    drawCentered(c, 32, line);
  } else {
    drawCentered(c, 14, "--:--:--", 2);
    drawCentered(c, 32, "DS1307 not found");
  }

  // Position
  if (g.alive && g.fix) {
    formatPosition(line, sizeof(line), g.latitudeE7, g.longitudeE7);
    drawCentered(c, 44, line);
  }

  // RTC sync status
  if (synced)
    snprintf(line, sizeof(line), "RTC synced %02d:%02dZ", lastSync.hour(), lastSync.minute());
  else if (rtcOk && !rtcRunning)
    snprintf(line, sizeof(line), "RTC stopped, wait GPS");
  else
    snprintf(line, sizeof(line), "RTC not synced");
  c.drawText(0, 56, line, false);
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
  // The SSD1306 and GPS drivers are created in their tasks, once the
  // scheduler is running and i2c1_mutex exists.
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
  // Built here, not as a global: the transport needs i2c1_mutex, which
  // only exists once main() has created it. Static, to keep the 1 KB
  // frame buffer off this task's stack.
  static OledDisplay oled(SSD1306::ssd1306_default_param(64), &hi2c1, SSD1306_I2C_ADDR, i2c1_mutexHandle);

  char msg[112];
  int lastSecond = -1;
  bool lastRtcOk = true;
  bool redrawPending = false;
  bool rtcOk = false, rtcRunning = false;
  DateTime now(SECONDS_FROM_1970_TO_2000);
  GpsStatus g;
  bool synced = false;
  DateTime lastSync;

  /* Infinite loop */
  for(;;)
  {
    // DS1307: blocking RTC-lib calls, so take the bus for them
    if (osMutexAcquire(i2c1_mutexHandle, I2C_MUTEX_TIMEOUT_MS) == osOK) {
      rtcOk = HAL_I2C_IsDeviceReady(&hi2c1, DS1307_ADDR << 1, 2, I2C_PROBE_TIMEOUT_MS) == HAL_OK;
      rtcRunning = rtcOk && rtc.isrunning();
      now = rtcOk ? rtc.now() : DateTime(SECONDS_FROM_1970_TO_2000);
      osMutexRelease(i2c1_mutexHandle);
    }

    if (now.second() != lastSecond || rtcOk != lastRtcOk) {
      lastSecond = now.second();
      lastRtcOk = rtcOk;
      redrawPending = true;

      osMutexAcquire(gps_data_mutexHandle, osWaitForever);
      g = gpsStatus;
      synced = rtcSynced;
      lastSync = rtcLastSync;
      osMutexRelease(gps_data_mutexHandle);

      snprintf(msg, sizeof(msg), "RTC %04d-%02d-%02d %02d:%02d:%02d UTC | GPS %s sats=%u cfg=%s%s%s\r\n",
               now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(),
               !g.alive ? "no data" : (g.fix ? "fix" : "no fix"), g.satellites, configName(g.config),
               rtcOk ? "" : " | DS1307 not found", oled.failed() ? " | SSD1306 not responding" : "");
      debugPrint(msg);
    }

    // Draw only between flushes (iDisplayDevice rule). The driver sends
    // just the pages that changed.
    if (redrawPending && oled.idle()) {
      redrawPending = false;
      drawScreen(oled, rtcOk, rtcRunning, now, g, synced, lastSync);
      oled.requestFlush();
    }

    // Run the display's state machine until the flush is out (it sleeps
    // in osThreadFlagsWait while each transfer is in flight). In the
    // error state one call waits out part of the back-off and returns.
    do {
      oled.main(osKernelGetTickCount());
    } while (!oled.idle() && !oled.failed());

    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
  * @brief  Runs the GPS driver and sets the DS1307 once the GPS has a fix.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  mtk3339_param_t param(true);              // configure the module
  param.updateMs = 1000;                    // 1 Hz
  param.sentences = mtk3339_sentence_bit(pmtk::Nmea::RMC) | mtk3339_sentence_bit(pmtk::Nmea::GGA);

  // Built here so reception starts after MX_USART1_UART_Init(); main()
  // must be called from this one thread (it is the one wake() wakes).
  static GpsReceiver gps(param, &huart1);
  static GnssData d;                        // ~400 bytes: kept off the stack

  bool haveData = false;
  uint32_t lastSyncTick = 0;
  bool synced = false;

  /* Infinite loop */
  for(;;)
  {
    // Sleeps inside until a line arrives (woken from the UART interrupt) or 100 ms
    gps.main(osKernelGetTickCount());

    bool fresh = gps.newData();
    if (fresh)
      haveData = gps.getData(&d);
    if (gps.state() == mtk3339_error_state)
      haveData = false;                     // silent for 3 s: nothing is valid

    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    gpsStatus.alive = haveData;
    gpsStatus.fix = haveData && d.positionValid;
    gpsStatus.satellites = haveData ? d.satellitesUsed : 0;
    gpsStatus.latitudeE7 = d.latitudeE7;
    gpsStatus.longitudeE7 = d.longitudeE7;
    gpsStatus.config = gps.configStatus();
    osMutexRelease(gps_data_mutexHandle);

    // Set the DS1307 from GPS (UTC) on the first valid fix, then once per interval
    if (fresh && haveData && d.positionValid && d.dateValid && d.timeValid
        && (!synced || (osKernelGetTickCount() - lastSyncTick) >= RTC_RESYNC_INTERVAL_MS)) {
      DateTime gpsTime(d.year, d.month, d.day, d.hour, d.minute, d.second);

      bool rtcOk = false;
      if (osMutexAcquire(i2c1_mutexHandle, I2C_MUTEX_TIMEOUT_MS) == osOK) {
        rtcOk = HAL_I2C_IsDeviceReady(&hi2c1, DS1307_ADDR << 1, 2, I2C_PROBE_TIMEOUT_MS) == HAL_OK;
        if (rtcOk)
          rtc.adjust(gpsTime);   // also clears the clock-halt bit
        osMutexRelease(i2c1_mutexHandle);
      }

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
