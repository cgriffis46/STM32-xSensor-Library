/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : DS1307 bring-up on the NUCLEO-L432KC
  *
  *  The first live test of the GPS clock's parts: the DS1307 on its own.
  *  The driver (xds1307<Stm32HalI2CTransport>, Claude-iTransport's
  *  isensor/ds1307; see iTransport/SOURCE.txt) reads the DS1307 every
  *  50 ms, and once a second a status line goes to the ST-LINK's virtual
  *  COM port (USART2, 115200 8N1): the time, whether the oscillator runs,
  *  and what the SQW/OUT pin is doing. Commands typed there set the time
  *  and look at the battery-backed RAM.
  *
  *  The DS1307's SQW/OUT gives 1 Hz. Wired to A1 (PA1, TIM2_CH2) its edges
  *  are latched by TIM2 at 1 MHz: the line shows its period (against the
  *  Nucleo's LSE crystal, which trims the 80 MHz clock), and which edge
  *  the DS1307's seconds tick on, worked out from when each read saw the
  *  seconds change (Core/Inc/rtc_edge.h, the F405 example's; coarse, to the
  *  50 ms between reads, and narrowing as more ticks are seen). A Saleae on
  *  SQW and the I2C lines gives the precise answer
  *  (Claude-iTransport's tools/saleae/saleae_log.py --rtc-edge).
  *
  *  A debug log (Claude-iTransport's DebugLog.h, at ITRANSPORT_DEBUG 2, set
  *  in the project's defines) goes out on USART1 TX, D5 (PB6), at 921600
  *  8N1: the driver's state changes and I2C faults ("ds1307", "bus"), and
  *  this application's events ("app"). Read it with a 3.3 V USB-serial
  *  adapter or the Saleae's async serial analyser; Claude-iTransport's
  *  tools/saleae/saleae_log.py turns a capture into a timeline.
  *
  *  The full GPS clock (GPS, OLED, W5500, NTP, PPS) that this project was
  *  is in the repository's history, at commit eb4b352.
  *
  *  Wiring (NUCLEO-L432KC):
  *    I2C1_SCL  PA9  (D1)  -> DS1307 SCL
  *    I2C1_SDA  PA10 (D0)  -> DS1307 SDA
  *    TIM2_CH2  PA1  (A1)  <- DS1307 SQW/OUT (optional; 10k pull-up to 3.3 V)
  *    5V, GND              -> DS1307 VCC (5 V), GND
  *    USART2    PA2/PA15   <> ST-LINK virtual COM port (the USB cable)
  *    USART1_TX PB6  (D5)  -> debug log, 921600 8N1 (USART1_RX PB7, D4, unused)
  *    LD3       PB3        green LED: toggles on each DS1307 second
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
#include <atomic>
#include "Stm32HalI2CTransport.h"
#include "Stm32HalUartTransport.h"
#include "Stm32DebugPort.h"
#include "DebugLog.h"
#include "xds1307.h"
#include "rtc_edge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef DS1307::xds1307<Stm32HalI2CTransport> Ds1307Clock;
typedef DS1307::ds1307_time_t ClockTime;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The DS1307 is read this often: the seconds' tick is bracketed to about
// twice this (two loop turns) against the SQW edges
#define DS1307_READ_PERIOD_MS   50UL
// DS1307 SQW/OUT: 1 Hz (control register 0x10)
#define DS1307_SQW_MODE         DS1307::ds1307_sqw_t::Hz1

// TIM2 counts at 1 MHz (80 MHz / 80) and latches both SQW edges on PA1
#define SQW_TIMER_PRESCALER     79
#define SQW_TIMEOUT_MS          2500UL     // no edge for this long: no SQW

// Battery-backed RAM: a start counter at 0x08 ("DS", then 32 bits, little
// endian), to show the RAM keeps what was written across a power cycle
#define RAM_MAGIC0              'D'
#define RAM_MAGIC1              'S'

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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for rtcTask */
osThreadId_t rtcTaskHandle;
const osThreadAttr_t rtcTask_attributes = {
  .name = "rtcTask",
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
// SQW edges as TIM2 latched them, with their direction: the last eight,
// written by the capture interrupt
static volatile uint32_t g_sqwAt[8];
static volatile uint8_t g_sqwRising[8];
static std::atomic<uint32_t> g_sqwCount{0};

// The console: a ring the tasks write (under console_mutex) and
// consoleTask hands to USART2
static char g_console[CONSOLE_RING];
static uint32_t g_conHead = 0, g_conTail = 0;
static uint32_t g_conDropped = 0;

// A command line typed on the console, handed from the UART interrupt to
// rtcTask: g_cmdReady set once it is complete, cleared when taken
static char g_cmd[CONSOLE_LINE];
static std::atomic<bool> g_cmdReady{false};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartRtcTask(void *argument);
void StartConsoleTask(void *argument);

/* USER CODE BEGIN PFP */
static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *const dayNames[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

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
  consolePrint("\r\nNUCLEO-L432KC DS1307 bring-up: I2C1 SCL=D1 (PA9) SDA=D0 (PA10), SQW=A1 (PA1)\r\n"
               "commands (Enter to finish; turn on your terminal's local echo):\r\n"
               "  t YYYY-MM-DD HH:MM:SS   set the DS1307 (24 hour, kept as UTC); starts a stopped oscillator\r\n"
               "  b                       set it to this firmware's build time (the build PC's local time)\r\n"
               "  r                       read and show the 56 bytes of battery-backed RAM\r\n"
               "  p                       start the SQW edge measurement again\r\n"
               "  h                       this help\r\n\r\n");
}

// "2026-10-18 12:34:56", or false
static bool parseTime(const char *s, ClockTime *t)
{
  unsigned y, mo, d, h, mi, se;
  char tail;
  if (sscanf(s, "%u-%u-%u %u:%u:%u%c", &y, &mo, &d, &h, &mi, &se, &tail) != 6)
    return false;
  t->year = (uint16_t) y; t->month = (uint8_t) mo; t->day = (uint8_t) d;
  t->hour = (uint8_t) h; t->minute = (uint8_t) mi; t->second = (uint8_t) se;
  t->weekday = 0;
  return DS1307::ds1307_valid(*t);
}

// __DATE__ ("Oct 18 2026") and __TIME__ ("12:34:56")
static bool buildTime(ClockTime *t)
{
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = { __DATE__[0], __DATE__[1], __DATE__[2], 0 };
  const char *m = strstr(months, mon);
  if (m == nullptr)
    return false;
  t->month = (uint8_t)((m - months) / 3 + 1);
  t->day = (uint8_t) atoi(__DATE__ + 4);
  t->year = (uint16_t) atoi(__DATE__ + 7);
  t->hour = (uint8_t) atoi(__TIME__);
  t->minute = (uint8_t) atoi(__TIME__ + 3);
  t->second = (uint8_t) atoi(__TIME__ + 6);
  t->weekday = 0;
  return DS1307::ds1307_valid(*t);
}

// "-0.100", from microseconds, as ms with three decimals
static void formatMs(char *out, size_t len, int32_t us)
{
  const long a = labs((long) us);
  snprintf(out, len, "%c%ld.%03ld", us < 0 ? '-' : '+', a / 1000, a % 1000);
}

static const char *stateName(DS1307::ds1307_state_t s)
{
  switch (s) {
  case DS1307::ds1307_init_state:
  case DS1307::ds1307_control_state:
  case DS1307::ds1307_wait_control_state: return "starting";
  case DS1307::ds1307_err_state:          return "NOT ANSWERING";
  default:                                return "ok";
  }
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // The debug log on USART1: lines queue in a ring from here on, and
  // consoleTask's dbg::poll() sends them
  static Stm32HalUartTransport debugUart(&huart1);
  dbg::begin(&debugUart, Stm32DebugPort::port());
  DBG_EVENT("app", "start", (int32_t) HAL_GetREVID(), (int32_t) HAL_GetDEVID());
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);   // SQW, both edges; TIM2 free-running at 1 MHz
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
  /* creation of rtcTask */
  rtcTaskHandle = osThreadNew(StartRtcTask, NULL, &rtcTask_attributes);

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
  hi2c1.Init.Timing = 0x10909CEC;   // 100 kHz (standard mode: the DS1307's limit) at 80 MHz
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
  * @brief TIM2 Initialization Function: a free-running 32-bit 1 MHz counter,
  *        channel 2 capturing both edges of the DS1307's SQW on PA1
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = SQW_TIMER_PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;   // a few samples: ignores glitches shorter than ~0.1 us
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
// An SQW edge, latched by TIM2 channel 2 in hardware (exact whatever the
// interrupt's latency); its direction read from the pin, which stays put
// for half a second
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    const uint32_t at = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    const bool high = HAL_GPIO_ReadPin(DS1307_SQW_GPIO_Port, DS1307_SQW_Pin) == GPIO_PIN_SET;
    const uint32_t n = g_sqwCount.load(std::memory_order_relaxed);
    g_sqwAt[n & 7] = at;
    g_sqwRising[n & 7] = high ? 1 : 0;
    g_sqwCount.store(n + 1, std::memory_order_release);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRtcTask */
/**
  * @brief  Runs the DS1307's driver, takes the console's commands, follows
  *         the SQW edges, and prints a status line on each new second.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRtcTask */
void StartRtcTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Built here: the transport needs i2c1_mutex, which main() creates
  DS1307::ds1307_param_t param;
  param.period_ms = DS1307_READ_PERIOD_MS;
  param.sqw = DS1307_SQW_MODE;
  static Ds1307Clock rtc(param, &hi2c1, DS1307_ADDR, i2c1_mutexHandle);
  rtc.setDebugTag("ds1307");               // its state changes and faults in the debug log

  printHelp();

  rtcedge::Analyzer an;
  uint32_t usedSqw = 0;
  uint32_t lastSqwTick = 0;
  bool sqwEver = false;
  // the SQW's period: falling edge to falling edge, and averaged since the first
  uint32_t lastFall = 0, firstFall = 0, falls = 0;
  int32_t lastPeriodUs = 0;
  uint32_t lastRise = 0;
  int32_t highUs = 0;                      // rising to the next falling edge

  ClockTime t = {};
  bool haveTime = false;
  int lastSecond = -1;
  uint32_t lastStatusTick = 0;
  bool lastRunning = true;
  DS1307::ds1307_set_status_t lastSet = DS1307::ds1307_set_status_t::None;

  // The start counter in RAM: read it once the DS1307 answers, then write it back plus one
  enum class Ram : uint8_t { Idle, Reading, Writing, Done, Dumping } ram = Ram::Idle;
  bool ramCounted = false;
  uint8_t ramBytes[DS1307::ds1307_ram_size];
  static uint8_t ramOut[6];

  uint32_t mainBefore = TIM2->CNT;         // TIM2 at the start of the last main() call

  for(;;)
  {
    // A read lands in the main() call after the one that issued it, so its
    // START lies between the start of the call before and the end of this one
    const uint32_t from = mainBefore;
    mainBefore = TIM2->CNT;
    rtc.main(osKernelGetTickCount());      // sleeps inside, up to the read period
    const uint32_t to = TIM2->CNT;
    const uint32_t tick = osKernelGetTickCount();

    // SQW edges since last time, in order, before the read that may follow them
    const uint32_t n = g_sqwCount.load(std::memory_order_acquire);
    if (n - usedSqw > 8)
      usedSqw = n - 8;                     // fell behind: the ring holds the last eight
    for (; usedSqw != n; ++usedSqw) {
      const uint32_t at = g_sqwAt[usedSqw & 7];
      const bool rising = g_sqwRising[usedSqw & 7] != 0;
      an.edge(at, rising);
      sqwEver = true;
      lastSqwTick = tick;
      if (rising) {
        lastRise = at;
      } else {
        if (falls != 0)
          lastPeriodUs = (int32_t)(at - lastFall);
        if (falls == 0)
          firstFall = at;
        if (lastRise != 0 && at - lastRise < 1000000u)
          highUs = (int32_t)(at - lastRise);
        lastFall = at;
        ++falls;
        DBG_EVENT("app", "sqw-fall", (int32_t) at, lastPeriodUs);
      }
    }
    const bool sqwNow = sqwEver && (tick - lastSqwTick) < SQW_TIMEOUT_MS;

    // A command typed on the console
    if (g_cmdReady.load(std::memory_order_acquire)) {
      char cmd[CONSOLE_LINE];
      strcpy(cmd, g_cmd);
      g_cmdReady.store(false, std::memory_order_release);
      consolePrint("> %s\r\n", cmd);
      DBG_EVENT("app", "cmd", (int32_t) cmd[0]);
      ClockTime set;
      if (cmd[0] == 't' && cmd[1] == ' ') {
        if (parseTime(cmd + 2, &set) && rtc.setTime(set))
          consolePrint("setting the DS1307 to %04u-%02u-%02u %02u:%02u:%02u\r\n", set.year, set.month, set.day,
                       set.hour, set.minute, set.second);
        else
          consolePrint("not a date and time: t YYYY-MM-DD HH:MM:SS (2000..2099, 24 hour)\r\n");
      } else if (cmd[0] == 'b' && cmd[1] == 0) {
        if (buildTime(&set) && rtc.setTime(set))
          consolePrint("setting the DS1307 to the build time, %04u-%02u-%02u %02u:%02u:%02u\r\n", set.year,
                       set.month, set.day, set.hour, set.minute, set.second);
        else
          consolePrint("the build time (%s %s) didn't parse\r\n", __DATE__, __TIME__);
      } else if (cmd[0] == 'r' && cmd[1] == 0) {
        if (ram == Ram::Idle || ram == Ram::Done) {
          rtc.readRam();
          ram = Ram::Dumping;
        } else {
          consolePrint("the RAM is busy; try again\r\n");
        }
      } else if (cmd[0] == 'p' && cmd[1] == 0) {
        an.restart();
        consolePrint("SQW edge measurement started again\r\n");
      } else if ((cmd[0] == 'h' || cmd[0] == '?') && cmd[1] == 0) {
        printHelp();
      } else {
        consolePrint("unknown command; h for help\r\n");
      }
    }

    // What became of a time set
    if (rtc.setStatus() != lastSet) {
      lastSet = rtc.setStatus();
      if (lastSet == DS1307::ds1307_set_status_t::Done)
        consolePrint("DS1307 set\r\n");
      else if (lastSet == DS1307::ds1307_set_status_t::Failed)
        consolePrint("DS1307 set FAILED (the I2C write didn't go through)\r\n");
      DBG_EVENT("app", "set", (int32_t) lastSet);
    }

    // A read landed: the time, and the seconds' tick against the SQW edges
    if (rtc.newData() && rtc.getTime(&t)) {
      haveTime = true;
      uint32_t tf = 0, tt = 0;
      if (an.read(from, to, t.second, &tf, &tt))   // brackets the tick when the seconds moved on
        DBG_EVENT("app", "tick", (int32_t) t.second, (int32_t) tf, (int32_t) tt);
      if (ram == Ram::Idle && !ramCounted) {
        rtc.readRam();
        ram = Ram::Reading;
      }
    }

    // The RAM: the start counter, or a dump
    if ((ram == Ram::Reading || ram == Ram::Dumping) && rtc.ramReadStatus() == DS1307::ds1307_set_status_t::Done
        && rtc.getRam(ramBytes)) {
      if (ram == Ram::Dumping) {
        for (uint8_t row = 0; row < DS1307::ds1307_ram_size; row += 8) {
          consolePrint("RAM 0x%02X: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 0x08 + row, ramBytes[row],
                       ramBytes[row + 1], ramBytes[row + 2], ramBytes[row + 3], ramBytes[row + 4],
                       ramBytes[row + 5], ramBytes[row + 6], ramBytes[row + 7]);
        }
        ram = Ram::Done;
      } else {
        uint32_t count = 0;
        const bool known = ramBytes[0] == RAM_MAGIC0 && ramBytes[1] == RAM_MAGIC1;
        if (known)
          count = (uint32_t) ramBytes[2] | ((uint32_t) ramBytes[3] << 8) | ((uint32_t) ramBytes[4] << 16)
                  | ((uint32_t) ramBytes[5] << 24);
        ++count;
        ramOut[0] = RAM_MAGIC0;
        ramOut[1] = RAM_MAGIC1;
        ramOut[2] = (uint8_t) count;
        ramOut[3] = (uint8_t)(count >> 8);
        ramOut[4] = (uint8_t)(count >> 16);
        ramOut[5] = (uint8_t)(count >> 24);
        if (rtc.writeRam(0, ramOut, sizeof(ramOut))) {
          ram = Ram::Writing;
          DBG_EVENT("app", "ram-count", (int32_t) count, known ? 1 : 0);
          consolePrint(known ? "RAM: start %lu (the count was kept: the RAM holds across resets and power cycles)\r\n"
                             : "RAM: no start count yet (new chip, or the battery was out): starting at %lu\r\n",
                       (unsigned long) count);
        } else {
          ram = Ram::Done;
        }
      }
    } else if ((ram == Ram::Reading || ram == Ram::Dumping)
               && rtc.ramReadStatus() == DS1307::ds1307_set_status_t::Failed) {
      consolePrint("RAM read FAILED\r\n");
      ram = Ram::Done;
      ramCounted = true;
    }
    if (ram == Ram::Writing && rtc.ramWriteStatus() != DS1307::ds1307_set_status_t::Pending) {
      if (rtc.ramWriteStatus() == DS1307::ds1307_set_status_t::Failed)
        consolePrint("RAM write FAILED\r\n");
      ram = Ram::Done;
      ramCounted = true;
    }

    // The status line: on each new second, or once a second without a time
    const bool newSecond = haveTime && (int) t.second != lastSecond;
    if (newSecond || (!haveTime && tick - lastStatusTick >= 1000)) {
      lastSecond = haveTime ? t.second : -1;
      lastStatusTick = tick;
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      char sqw[80], edge[96], a[16], b[16];
      if (!sqwNow) {
        snprintf(sqw, sizeof(sqw), "SQW none on A1");
      } else if (falls < 2) {
        snprintf(sqw, sizeof(sqw), "SQW seen");
      } else {
        // ppm against TIM2 (the LSE): averaged over every period since the first
        const int64_t span = (int64_t)(uint32_t)(lastFall - firstFall);
        const int64_t ppb = (span - (int64_t)(falls - 1) * 1000000) * 1000 / (int64_t)(falls - 1);
        const long ap = labs((long) ppb);
        snprintf(sqw, sizeof(sqw), "SQW %ldus high=%ldus avg %c%ld.%03ldppm/%lus", (long) lastPeriodUs,
                 (long) highUs, ppb < 0 ? '-' : '+', ap / 1000, ap % 1000, (unsigned long)(falls - 1));
      }
      const rtcedge::Polarity v = an.verdict();
      if (!sqwNow || an.ticks() == 0) {
        edge[0] = 0;
      } else if (v == rtcedge::Polarity::Unknown) {
        snprintf(edge, sizeof(edge), " | tick: undecided (%u)", an.ticks());
      } else {
        const rtcedge::Window &w = v == rtcedge::Polarity::Falling ? an.falling() : an.rising();
        formatMs(a, sizeof(a), w.loUs);
        formatMs(b, sizeof(b), w.hiUs);
        snprintf(edge, sizeof(edge), " | tick %s..%sms after %s edge (%u)", a, b,
                 v == rtcedge::Polarity::Falling ? "falling" : "rising", w.n);
      }
      if (haveTime)
        consolePrint("%s %04u-%02u-%02u %02u:%02u:%02u | DS1307 %s%s | %s%s\r\n", dayNames[t.weekday % 7], t.year,
                     t.month, t.day, t.hour, t.minute, t.second, stateName(rtc.state()),
                     rtc.running() ? "" : ", STOPPED (CH set): set it with t or b", sqw, edge);
      else
        consolePrint("DS1307 %s, no time yet (I2C address 0x68: SDA D0, SCL D1, pull-ups, 5 V) | %s\r\n",
                     stateName(rtc.state()), sqw);
      if (haveTime && rtc.running() != lastRunning) {
        lastRunning = rtc.running();
        if (!lastRunning)
          consolePrint("the DS1307's oscillator is stopped (a new chip, or the battery ran out): it keeps no time"
                       " until it is set\r\n");
      }
    }
    if (rtc.state() == DS1307::ds1307_err_state)
      haveTime = false;
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
