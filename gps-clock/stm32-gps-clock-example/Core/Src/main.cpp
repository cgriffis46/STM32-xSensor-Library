/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : BME280 bring-up on the NUCLEO-L432KC
  *
  *  A live test of one sensor: a BME280 (temperature, pressure, humidity)
  *  on I2C1, read by Claude-iTransport's non-blocking driver
  *  (xbme280<Stm32HalI2CTransport>, isensor/bme280; see
  *  iTransport/SOURCE.txt) in forced mode, about five readings a second.
  *  Once a second the latest reading goes to the ST-LINK's virtual COM
  *  port (USART2, 115200 8N1), with a flag on any value outside the
  *  BME280's range.
  *
  *  At start-up the I2C bus is scanned and the chip ID read at 0x76 and
  *  0x77 (the address depends on the board's SDO pin): 0x60 is a BME280,
  *  0x58 a BMP280 (no humidity, and many boards sold as BME280 carry one),
  *  0x61 a BME680. The scan repeats every 2 s until a BME280 answers.
  *
  *  A debug log (Claude-iTransport's DebugLog.h, at ITRANSPORT_DEBUG 2, set
  *  in the project's defines) goes out on USART1 TX, D5 (PB6), at 921600
  *  8N1: the driver's state changes and I2C faults ("bme280", "bus"), and
  *  this application's events ("app"). Read it with a 3.3 V USB-serial
  *  adapter or the Saleae's async serial analyser.
  *
  *  The GPS clock this project started as (GPS, OLED, W5500, NTP, PPS) is
  *  in the repository's history at commit eb4b352.
  *
  *  Wiring (NUCLEO-L432KC):
  *    I2C1_SCL  PA9  (D1)  -> BME280 SCL (SCK)
  *    I2C1_SDA  PA10 (D0)  -> BME280 SDA (SDI)
  *    3V3, GND             -> BME280 VCC/VIN, GND
  *    USART2    PA2/PA15   <> ST-LINK virtual COM port (the USB cable)
  *    USART1_TX PB6  (D5)  -> debug log, 921600 8N1 (USART1_RX PB7, D4, unused)
  *    LD3       PB3        green LED: toggles on each reading
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
#include <stdarg.h>
#include <math.h>
#include <atomic>
#include "Stm32HalI2CTransport.h"
#include "Stm32HalUartTransport.h"
#include "Stm32DebugPort.h"
#include "DebugLog.h"
#include "xBME280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef BME280::xbme280<Stm32HalI2CTransport> Bme280Sensor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The BME280's ID register (0xD0) and what it holds for the parts that
// look alike (Bosch's datasheets, as the driver and general knowledge have them)
#define CHIP_ID_REG             0xD0
#define CHIP_ID_BME280          0x60
#define CHIP_ID_BMP280          0x58
#define CHIP_ID_BME680          0x61

#define SCAN_RETRY_MS           2000UL     // no BME280 found: scan again this often
#define READING_STALE_MS        3000UL     // no reading for this long: say so

// The debug log's UART (USART1): fast, since level 2 logs every state change
#define DEBUG_BAUD              921600

#define CONSOLE_RING            2048
#define CONSOLE_LINE            48
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for consoleTask */
osThreadId_t consoleTaskHandle;
const osThreadAttr_t consoleTask_attributes = {
  .name = "consoleTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for i2c1_mutex */
osMutexId_t i2c1_mutexHandle;
const osMutexAttr_t i2c1_mutex_attributes = {
  .name = "i2c1_mutex"
};
/* Definitions for console_mutex */
osMutexId_t console_mutexHandle;
const osMutexAttr_t console_mutex_attributes = {
  .name = "console_mutex"
};
/* USER CODE BEGIN PV */
// The console: a ring the tasks write (under console_mutex) and
// consoleTask hands to USART2
static char g_console[CONSOLE_RING];
static uint32_t g_conHead = 0, g_conTail = 0;
static uint32_t g_conDropped = 0;

// A command line typed on the console, handed from the UART interrupt to
// sensorTask: g_cmdReady set once it is complete, cleared when taken
static char g_cmd[CONSOLE_LINE];
static std::atomic<bool> g_cmdReady{false};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartSensorTask(void *argument);
void StartConsoleTask(void *argument);

/* USER CODE BEGIN PFP */
static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// The HAL UART and I2C interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32UartItCallbacks.cpp, Stm32I2CItCallbacks.cpp): don't define them here.

// printf to the console. Tasks only; a line that doesn't fit is dropped
// and counted.
static void consolePrint(const char *fmt, ...)
{
  char line[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n <= 0)
    return;
  if (n >= (int) sizeof(line))
    n = sizeof(line) - 1;
  if (console_mutexHandle == nullptr || osMutexAcquire(console_mutexHandle, 20) != osOK)
    return;
  if (g_conHead - g_conTail + (uint32_t) n > CONSOLE_RING) {
    ++g_conDropped;
  } else {
    for (int i = 0; i < n; ++i)
      g_console[(g_conHead + i) % CONSOLE_RING] = line[i];
    g_conHead += n;
  }
  osMutexRelease(console_mutexHandle);
}

// Keys typed on the console, a line at a time (Enter ends it; Backspace
// works). Interrupt context. A line typed while the last is still waiting
// is dropped. The terminal should echo locally: nothing is sent back.
struct ConsoleKeys : iTransportRxSink {
  char buf[CONSOLE_LINE];
  uint8_t len = 0;
  void onByteReceived(uint8_t b) override {
    if (b == '\r' || b == '\n') {
      if (len != 0 && !g_cmdReady.load(std::memory_order_acquire)) {
        memcpy(g_cmd, buf, len);
        g_cmd[len] = 0;
        g_cmdReady.store(true, std::memory_order_release);
      }
      len = 0;
    } else if (b == 0x08 || b == 0x7F) {
      if (len != 0)
        --len;
    } else if (b >= 0x20 && len < CONSOLE_LINE - 1) {
      buf[len++] = (char) b;
    }
  }
};
static ConsoleKeys consoleKeys;

static void printHelp(void)
{
  consolePrint("\r\nNUCLEO-L432KC BME280 bring-up: I2C1 SCL=D1 (PA9) SDA=D0 (PA10), 100 kHz\r\n"
               "commands (Enter to finish; turn on your terminal's local echo):\r\n"
               "  s   scan the I2C bus and read the chip ID at 0x76 and 0x77\r\n"
               "  h   this help\r\n\r\n");
}

// "-12.34" from a double with `decimals` places, without printf float
// support (newlib nano); "nan" for NAN
static void formatFixed(char *out, size_t len, double v, int decimals)
{
  if (isnan(v)) {
    snprintf(out, len, "nan");
    return;
  }
  long scale = 1;
  for (int i = 0; i < decimals; ++i)
    scale *= 10;
  const bool neg = v < 0;
  const long long n = (long long)((neg ? -v : v) * scale + 0.5);
  snprintf(out, len, "%s%ld.%0*ld", neg ? "-" : "", (long)(n / scale), decimals, (long)(n % scale));
}

static const char *chipName(uint8_t id)
{
  switch (id) {
  case CHIP_ID_BME280: return "BME280";
  case CHIP_ID_BMP280: return "BMP280 (no humidity)";
  case CHIP_ID_BME680: return "BME680";
  default:             return "unknown";
  }
}

// Every address 0x08..0x77 that acknowledges, and the chip ID at 0x76 and
// 0x77. Blocking HAL calls under i2c1_mutex, about 15 ms. The mutex is
// not recursive and a driver holds it from issuing a transfer until it
// sees it land, so call this only from a thread with no transfer
// outstanding (sensorTask waits for a state of its driver's with none). Returns the
// address of a BME280, or 0 (and 0 if the bus stayed busy).
static uint8_t scanBus(void)
{
  uint8_t found = 0;
  char list[160];
  size_t n = 0;
  list[0] = 0;
  if (osMutexAcquire(i2c1_mutexHandle, 200) != osOK) {
    consolePrint("I2C bus busy: no scan\r\n");
    return 0;
  }
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 2) == HAL_OK && n + 6 < sizeof(list))
      n += (size_t) snprintf(list + n, sizeof(list) - n, " 0x%02X", a);
  }
  uint8_t ids[2] = {0, 0};
  bool idOk[2] = {false, false};
  for (int i = 0; i < 2; ++i) {
    const uint8_t a = (uint8_t)(BME280::bme280_i2c_addr_1 + i);
    idOk[i] = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(a << 1), CHIP_ID_REG, I2C_MEMADD_SIZE_8BIT, &ids[i], 1, 10) == HAL_OK;
  }
  osMutexRelease(i2c1_mutexHandle);

  consolePrint("I2C scan:%s\r\n", n ? list : " nothing answered (check SDA D0, SCL D1, 3V3, GND, pull-ups)");
  for (int i = 0; i < 2; ++i) {
    const uint8_t a = (uint8_t)(BME280::bme280_i2c_addr_1 + i);
    if (idOk[i]) {
      consolePrint("  0x%02X: chip ID 0x%02X = %s\r\n", a, ids[i], chipName(ids[i]));
      DBG_EVENT("app", "chip-id", (int32_t) a, (int32_t) ids[i]);
      if (ids[i] == CHIP_ID_BME280 && found == 0)
        found = a;
    }
  }
  return found;
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
  // The debug log on USART1: lines queue in a ring from here on, and
  // consoleTask's dbg::poll() sends them
  static Stm32HalUartTransport debugUart(&huart1);
  dbg::begin(&debugUart, Stm32DebugPort::port());
  DBG_EVENT("app", "start", (int32_t) HAL_GetREVID(), (int32_t) HAL_GetDEVID());
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c1_mutex */
  i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes);

  /* creation of console_mutex */
  console_mutexHandle = osMutexNew(&console_mutex_attributes);

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
  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of consoleTask */
  consoleTaskHandle = osThreadNew(StartConsoleTask, NULL, &consoleTask_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * MSI 4 MHz -> PLL x40 /2 -> 80 MHz; the LSE crystal (X2) trims the MSI.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x10909CEC;   // 100 kHz (standard mode) at 80 MHz; the BME280 also takes 400 kHz
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
  * @brief USART1 Initialization Function (the debug log, D5/D4)
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
  huart1.Init.BaudRate = DEBUG_BAUD;
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
  * @brief USART2 Initialization Function (the ST-LINK virtual COM port)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Finds the BME280 (bus scan, chip ID), then runs its driver,
  *         takes the console's commands and prints the latest reading
  *         once a second.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  printHelp();

  // Find it first: 0x76 or 0x77, by its chip ID
  uint8_t addr = 0;
  while ((addr = scanBus()) == 0) {
    consolePrint("no BME280 (chip ID 0x60) at 0x76 or 0x77; scanning again in %lu s\r\n",
                 (unsigned long)(SCAN_RETRY_MS / 1000));
    osDelay(SCAN_RETRY_MS);
  }
  consolePrint("BME280 at 0x%02X: forced mode, pressure x4, temperature x1, humidity x1, no filter\r\n", addr);

  // Forced mode: a measurement whenever the driver asks for one (about
  // every 0.2 s: its 100 ms pause, then up to 100 ms of conversion)
  BME280::bme280_param_t param;
  param._osrs_p_t = BME280::bme280_osrs_p_4x;
  param._osrs_t_t = BME280::bme280_osrs_t_1x;
  param._osrs_h_t = BME280::bme280_osrs_h_1x;
  param._mode_t = BME280::bme280_forced_mode;
  param._sb_t = BME280::bme280_sb_1000;      // normal mode only; not used here
  param._filter_t = BME280::bme280_filter_off;
  // Built here: the transport needs i2c1_mutex and the address just found
  static Bme280Sensor sensor(param, &hi2c1, addr, i2c1_mutexHandle);
  sensor.setDebugTag("bme280");             // its state changes and faults in the debug log

  double t = NAN, p = NAN, h = NAN;
  uint32_t readings = 0;
  uint32_t lastReadingTick = 0;
  uint32_t lastPrintTick = osKernelGetTickCount();
  double tMin = NAN, tMax = NAN;
  bool scanPending = false;

  for(;;)
  {
    sensor.main(osKernelGetTickCount());    // sleeps inside, up to 100 ms
    const uint32_t tick = osKernelGetTickCount();

    if (sensor.newData()) {
      sensor.GetTemperature(&t);
      sensor.GetPressure(&p);
      sensor.GetHumidity(&h);
      ++readings;
      lastReadingTick = tick;
      if (!isnan(t)) {
        if (isnan(tMin) || t < tMin) tMin = t;
        if (isnan(tMax) || t > tMax) tMax = t;
      }
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      DBG_EVENT("app", "reading", (int32_t) lround(t * 100), (int32_t) lround(p), (int32_t) lround(h * 100));
    }

    // A command typed on the console
    if (g_cmdReady.load(std::memory_order_acquire)) {
      char cmd[CONSOLE_LINE];
      strcpy(cmd, g_cmd);
      g_cmdReady.store(false, std::memory_order_release);
      consolePrint("> %s\r\n", cmd);
      DBG_EVENT("app", "cmd", (int32_t) cmd[0]);
      if (cmd[0] == 's' && cmd[1] == 0)
        scanPending = true;                 // done below, between the driver's transfers
      else if ((cmd[0] == 'h' || cmd[0] == '?') && cmd[1] == 0)
        printHelp();
      else
        consolePrint("unknown command; h for help\r\n");
    }

    // The scan: only in a state where the driver has no transfer
    // outstanding (it holds i2c1_mutex from issuing one until it sees it
    // land, and the mutex isn't recursive). It spends most of its time in these.
    if (scanPending) {
      const BME280::bme280_state_t st = sensor.state();
      if (st == BME280::bme280_sleeping || st == BME280::bme280_measuring || st == BME280::bme280_error
          || st == BME280::bme280_reset_settle || st == BME280::bme280_im_update_pause) {
        scanPending = false;
        (void) scanBus();
      }
    }

    // Once a second: the latest reading, flagged where it is outside the
    // BME280's range (-40..85 C, 300..1100 hPa; humidity pinned at 0 or
    // 100 % is the compensation's clamp, so suspect too)
    if (tick - lastPrintTick >= 1000) {
      lastPrintTick = tick;
      const bool stale = readings == 0 || tick - lastReadingTick > READING_STALE_MS;
      if (stale) {
        consolePrint("BME280 at 0x%02X: no reading%s (driver state %d%s)\r\n", addr,
                     readings == 0 ? " yet" : " for 3 s", (int) sensor.state(),
                     sensor.state() == BME280::bme280_error ? ", error: not answering, or a wrong chip ID" : "");
      } else {
        char ts[16], ps[16], hs[16], lo[16], hi[16];
        formatFixed(ts, sizeof(ts), t, 2);
        formatFixed(ps, sizeof(ps), p / 100.0, 2);
        formatFixed(hs, sizeof(hs), h, 1);
        formatFixed(lo, sizeof(lo), tMin, 2);
        formatFixed(hi, sizeof(hi), tMax, 2);
        const bool tBad = !(t >= -40.0 && t <= 85.0);
        const bool pBad = !(p >= 30000.0 && p <= 110000.0);
        const bool hBad = !(h > 0.0 && h < 100.0);
        consolePrint("T %s C%s  P %s hPa%s  H %s %%RH%s | %lu readings, T %s..%s C\r\n", ts, tBad ? " (?)" : "", ps,
                     pBad ? " (?)" : "", hs, hBad ? " (?)" : "", (unsigned long) readings, lo, hi);
      }
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartConsoleTask */
/**
  * @brief  Hands the console ring to USART2 (the ST-LINK's virtual COM
  *         port), a piece at a time, and listens for keys.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartConsoleTask */
void StartConsoleTask(void *argument)
{
  /* USER CODE BEGIN StartConsoleTask */
  static Stm32HalUartTransport port(&huart2);
  static uint8_t chunk[128];               // sent from here by interrupt: static
  uint32_t lastDropped = 0;
  port.setRxSink(consoleKeys);

  for(;;)
  {
    osMutexAcquire(console_mutexHandle, osWaitForever);
    const uint32_t avail = g_conHead - g_conTail;
    const uint32_t len = avail < sizeof(chunk) ? avail : sizeof(chunk);
    for (uint32_t i = 0; i < len; ++i)
      chunk[i] = (uint8_t) g_console[(g_conTail + i) % CONSOLE_RING];
    const uint32_t dropped = g_conDropped;
    osMutexRelease(console_mutexHandle);

    dbg::poll();                           // the debug log, to USART1
    if (len == 0) {
      osDelay(5);
      continue;
    }
    while (!port.write(chunk, len))        // the last piece still going out
      osDelay(1);
    // write() only started it: wait until it is out before chunk is reused
    while (huart2.gState != HAL_UART_STATE_READY)
      osDelay(1);
    osMutexAcquire(console_mutexHandle, osWaitForever);
    g_conTail += len;
    osMutexRelease(console_mutexHandle);
    if (dropped != lastDropped) {
      lastDropped = dropped;
      consolePrint("(console: %lu line(s) dropped so far)\r\n", (unsigned long) dropped);
    }
  }
  /* USER CODE END StartConsoleTask */
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
