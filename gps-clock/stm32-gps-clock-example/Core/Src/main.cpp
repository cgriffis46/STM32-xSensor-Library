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
  *  A W5500 on SPI1 gives the board an address (DHCP), shown on the
  *  display, and network time (SNTP), used to set the DS1307 when the GPS
  *  has no fix.
  *
  *  All four use the non-blocking drivers from Claude-iTransport
  *  (see iTransport/SOURCE.txt):
  *    xds1307<Stm32HalI2CTransport>    isensor/ds1307
  *    xssd1306<Stm32HalI2CTransport>   iDisplay/ssd1306
  *    xmtk3339<Stm32HalUartTransport>  isensor/mtk3339 (the MT3333 speaks the
  *                                     same NMEA/PMTK as the MT3339)
  *    w5500<Stm32HalSpiBlockTransport> iNetTransport/w5500, behind xEthernet
  *
  *  Wiring (NUCLEO-L432KC):
  *    I2C1_SCL  PA9  (D1)  -> DS1307 SCL, SSD1306 SCL
  *    I2C1_SDA  PA10 (D0)  -> DS1307 SDA, SSD1306 SDA
  *    USART1_TX PB6  (D5)  -> GPS RX
  *    USART1_RX PB7  (D4)  <- GPS TX
  *    USART2_TX PA2  (A7)  -> ST-LINK virtual COM port (debug, 115200)
  *    USART2_RX PA15       <- ST-LINK virtual COM port
  *    SPI1_SCK  PB3  (D13) -> W5500 SCLK (also drives LD3)
  *    SPI1_MISO PB4  (D12) <- W5500 MISO
  *    SPI1_MOSI PB5  (D11) -> W5500 MOSI
  *    GPIO      PA4  (A3)  -> W5500 SCSn
  *    GPIO      PA3  (A2)  -> W5500 RSTn
  *    EXTI1     PA1  (A1)  <- W5500 INTn
  *  SPI1 is on PB3/PB4/PB5, not PA5/PA6/PA7: on the NUCLEO-L432KC, SB16/SB18
  *  join PA5/PA6 to PB7/PB6, the GPS UART.
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
#include "Stm32HalI2CTransport.h"
#include "Stm32HalUartTransport.h"
#include "xds1307.h"
#include "xssd1306.h"
#include "xmtk3339.h"
#include "Stm32HalSpiBlockTransport.h"
#include "W5500.h"
#include "xEthernet.h"
#include <atomic>
#include "Font5x7.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef DS1307::xds1307<Stm32HalI2CTransport> RtcClock;
typedef SSD1306::xssd1306<Stm32HalI2CTransport, 64> OledDisplay;
typedef xmtk3339<Stm32HalUartTransport> GpsReceiver;
typedef DS1307::ds1307_time_t ClockTime;
typedef W5500::w5500<Stm32HalSpiBlockTransport> EthChip;

// What displayTask needs from the GPS, copied out by gpsTask
typedef struct {
  bool     alive;          // the GPS is sending good NMEA
  bool     fix;
  uint8_t  satellites;     // used in the fix (GGA)
  int32_t  latitudeE7;     // 1e-7 degrees
  int32_t  longitudeE7;
  mtk3339_config_status_t config;
  // UTC from the last RMC with a fix, date and time; timeSeq counts them
  ClockTime utc;
  uint32_t  timeTick;      // osKernelGetTickCount() when it was parsed
  uint32_t  timeSeq;
} GpsStatus;
/* USER CODE END PTD *//* USER CODE END PTD *//* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The DS1307 is kept in UTC. Set this to show local (standard) time on the display.
#define UTC_OFFSET_MINUTES      0

// Re-sync the DS1307 from GPS this often once it has been set
#define RTC_RESYNC_INTERVAL_MS  (60UL * 60UL * 1000UL)

// A GPS time older than this is not written to the DS1307
#define GPS_TIME_MAX_AGE_MS     500UL

// How often the DS1307 is read: the display shows a new second at most this late
#define RTC_READ_PERIOD_MS      100UL

// Network time is written to the DS1307 only this close after a second
// starts, since writing the seconds register restarts the chip's 1 Hz divider
#define NTP_SET_WINDOW_MS       150UL

// Ethernet address: DHCP, or this static one
#define ETH_DHCP                1
#define ETH_STATIC_IP           192, 168, 1, 50
#define ETH_STATIC_MASK         255, 255, 255, 0
#define ETH_STATIC_GW           192, 168, 1, 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

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
/* Definitions for ethTask */
osThreadId_t ethTaskHandle;
const osThreadAttr_t ethTask_attributes = {
  .name = "ethTask",
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
/* Definitions for spi1_mutex */
osMutexId_t spi1_mutexHandle;
const osMutexAttr_t spi1_mutex_attributes = {
  .name = "spi1_mutex"
};
/* USER CODE BEGIN PV */
// Written by gpsTask, read by displayTask, guarded by gps_data_mutex
static GpsStatus gpsStatus;

// Set by ethTask once the interface has begun; read by displayTask and
// the INT pin's EXTI callback. xNetInterface's status calls are thread-safe.
static std::atomic<xEthernet*> g_eth{nullptr};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDisplayTask(void *argument);
void StartGpsTask(void *argument);
void StartEthTask(void *argument);

/* USER CODE BEGIN PFP */
static void debugPrint(const char *str);
enum class SyncSource : uint8_t { None, Gps, Ntp };
static void drawScreen(idisplay::MonoCanvas &c, bool rtcOk, bool rtcRunning, const ClockTime &utc,
                       const GpsStatus &g, const char *ethLine, SyncSource synced, const ClockTime &lastSync);
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

// The Ethernet line: the address, or how far it has got
static void formatEth(char *out, size_t len)
{
  xEthernet *eth = g_eth.load();
  char ip[16];
  if (eth == nullptr)
    snprintf(out, len, "ETH: starting");
  else if (!eth->ready())
    snprintf(out, len, "ETH: no W5500");
  else if (!eth->linkUp())
    snprintf(out, len, "ETH: no link");
  else if (!eth->hasAddress())
    snprintf(out, len, "ETH: waiting for DHCP");
  else
    snprintf(out, len, "IP %s", eth->address().ip.format(ip));
}

static void drawScreen(idisplay::MonoCanvas &c, bool rtcOk, bool rtcRunning, const ClockTime &utc,
                       const GpsStatus &g, const char *ethLine, SyncSource synced, const ClockTime &lastSync)
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
    ClockTime local;
    DS1307::ds1307_from_unix(DS1307::ds1307_to_unix(utc) + (int32_t) UTC_OFFSET_MINUTES * 60, &local);

    // Time, double size
    snprintf(line, sizeof(line), "%02u:%02u:%02u", local.hour, local.minute, local.second);
    drawCentered(c, 12, line, 2);

    // Date
    snprintf(line, sizeof(line), "%s %04u-%02u-%02u%s", dayNames[local.weekday],
             local.year, local.month, local.day, UTC_OFFSET_MINUTES == 0 ? " UTC" : "");
    drawCentered(c, 29, line);
  } else {
    drawCentered(c, 12, "--:--:--", 2);
    drawCentered(c, 29, "DS1307 not found");
  }

  // Position
  if (g.alive && g.fix) {
    formatPosition(line, sizeof(line), g.latitudeE7, g.longitudeE7);
    drawCentered(c, 38, line);
  }

  // Ethernet
  c.drawText(0, 47, ethLine, false);

  // RTC sync status
  if (synced != SyncSource::None)
    snprintf(line, sizeof(line), "RTC synced %02u:%02uZ %s", lastSync.hour, lastSync.minute,
             synced == SyncSource::Gps ? "GPS" : "NTP");
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  debugPrint("\r\nSTM32L432KC GPS clock: DS1307 + SSD1306 + MTK3333\r\n");

  // The drivers are created in their tasks, once the scheduler is
  // running and i2c1_mutex exists.
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c1_mutex */
  i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes);

  /* creation of gps_data_mutex */
  gps_data_mutexHandle = osMutexNew(&gps_data_mutex_attributes);

  /* creation of spi1_mutex */
  spi1_mutexHandle = osMutexNew(&spi1_mutex_attributes);

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

  /* creation of ethTask */
  ethTaskHandle = osThreadNew(StartEthTask, NULL, &ethTask_attributes);

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
  * @brief SPI1 Initialization Function (W5500)
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;            // W5500: mode 0
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 10 MHz: SCK also drives LD3
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, W5500_RST_Pin|W5500_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : W5500_INT_Pin */
  GPIO_InitStruct.Pin = W5500_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(W5500_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_RST_Pin W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_RST_Pin|W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// The W5500's INTn (PA1): wakes the network driver thread at once
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  xEthernet *eth = g_eth.load();
  if (GPIO_Pin == W5500_INT_Pin && eth != nullptr)
    eth->interruptFromIsr();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDisplayTask */
/**
  * @brief  Owns the DS1307 and the SSD1306: reads the clock, sets it from
  *         GPS time, and redraws the display when the second changes.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Built here, not as globals: the transports need i2c1_mutex, which only
  // exists once main() has created it. Static, to keep them (the SSD1306's
  // 1 KB frame buffer above all) off this task's stack. Neither driver is
  // locked, so this one thread calls everything on them.
  DS1307::ds1307_param_t rtcParam;
  rtcParam.period_ms = RTC_READ_PERIOD_MS;
  static RtcClock rtc(rtcParam, &hi2c1, DS1307_ADDR, i2c1_mutexHandle);
  static OledDisplay oled(SSD1306::ssd1306_default_param(64), &hi2c1, SSD1306_I2C_ADDR, i2c1_mutexHandle);

  char msg[160];
  int lastSecond = -1;
  bool lastRtcOk = true;
  bool redrawPending = false;
  bool rtcOk = false;
  ClockTime now = {};
  GpsStatus g = {};

  uint32_t usedTimeSeq = 0;      // the GPS time last written, by its timeSeq
  bool syncing = false;          // a setTime() is waiting to be written
  ClockTime syncTime = {};
  SyncSource syncingFrom = SyncSource::None;
  SyncSource synced = SyncSource::None;
  uint32_t lastSyncTick = 0;
  ClockTime lastSync = {};
  char ethLine[24] = "";
  char lastEthLine[24] = "";

  /* Infinite loop */
  for(;;)
  {
    // Reads the DS1307 every RTC_READ_PERIOD_MS; sleeps (osDelay) until then
    rtc.main(osKernelGetTickCount());

    if (rtc.newData())
      rtcOk = rtc.getTime(&now);
    if (rtc.state() == DS1307::ds1307_err_state)
      rtcOk = false;

    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    g = gpsStatus;
    osMutexRelease(gps_data_mutexHandle);

    // Set the DS1307 (UTC) when it has never been set, or once per interval:
    // from the GPS when it has a fix, otherwise from network time (SNTP)
    const uint32_t tick = osKernelGetTickCount();
    const bool syncDue = synced == SyncSource::None || (tick - lastSyncTick) >= RTC_RESYNC_INTERVAL_MS;
    xEthernet *eth = g_eth.load();
    // A clock set from the network is set again from the GPS as soon as it has a fix
    const bool gpsBetter = synced == SyncSource::Ntp;
    if (!syncing && (syncDue || gpsBetter)) {
      if (g.timeSeq != usedTimeSeq && (tick - g.timeTick) < GPS_TIME_MAX_AGE_MS) {
        usedTimeSeq = g.timeSeq;
        if (rtc.setTime(g.utc)) {
          syncing = true;
          syncingFrom = SyncSource::Gps;
          syncTime = g.utc;
        }
      } else if (syncDue && !g.fix && eth != nullptr && eth->timeValid()) {
        const uint64_t unixMs = eth->unixTimeMs();
        if (unixMs % 1000 < NTP_SET_WINDOW_MS) {
          ClockTime t;
          DS1307::ds1307_from_unix((uint32_t)(unixMs / 1000), &t);
          if (rtc.setTime(t)) {
            syncing = true;
            syncingFrom = SyncSource::Ntp;
            syncTime = t;
          }
        }
      }
    }
    if (syncing && rtc.setStatus() == DS1307::ds1307_set_status_t::Done) {
      syncing = false;
      synced = syncingFrom;
      lastSyncTick = tick;
      lastSync = syncTime;
    } else if (syncing && rtc.setStatus() == DS1307::ds1307_set_status_t::Failed) {
      syncing = false;           // tried again with the next time
    }

    formatEth(ethLine, sizeof(ethLine));
    if (strcmp(ethLine, lastEthLine) != 0) {
      strcpy(lastEthLine, ethLine);
      redrawPending = true;
    }

    if ((rtcOk && now.second != lastSecond) || rtcOk != lastRtcOk) {
      lastSecond = rtcOk ? now.second : -1;
      lastRtcOk = rtcOk;
      redrawPending = true;

      snprintf(msg, sizeof(msg), "RTC %04u-%02u-%02u %02u:%02u:%02u UTC | GPS %s sats=%u cfg=%s | %s%s%s\r\n",
               now.year, now.month, now.day, now.hour, now.minute, now.second,
               !g.alive ? "no data" : (g.fix ? "fix" : "no fix"), g.satellites, configName(g.config), ethLine,
               rtcOk ? "" : " | DS1307 not responding", oled.failed() ? " | SSD1306 not responding" : "");
      debugPrint(msg);
    }

    // Draw only between flushes (iDisplayDevice rule). The driver sends
    // just the pages that changed.
    if (redrawPending && oled.idle()) {
      redrawPending = false;
      drawScreen(oled, rtcOk, rtc.running(), now, g, ethLine, synced, lastSync);
      oled.requestFlush();
    }

    // Run the display's state machine until the flush is out (it sleeps
    // in osThreadFlagsWait while each transfer is in flight). In the
    // error state one call waits out part of the back-off and returns.
    do {
      oled.main(osKernelGetTickCount());
    } while (!oled.idle() && !oled.failed());
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
  * @brief  Runs the GPS driver and publishes its fix and UTC time.
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
  uint32_t timeSeq = 0;

  /* Infinite loop */
  for(;;)
  {
    // Sleeps inside until a line arrives (woken from the UART interrupt) or 100 ms
    gps.main(osKernelGetTickCount());

    const bool fresh = gps.newData();
    if (fresh)
      haveData = gps.getData(&d);
    if (gps.state() == mtk3339_error_state)
      haveData = false;                     // silent for 3 s: nothing is valid

    ClockTime utc = {};
    utc.year = d.year; utc.month = d.month; utc.day = d.day;
    utc.hour = d.hour; utc.minute = d.minute; utc.second = d.second;
    const bool timeGood = fresh && haveData && d.positionValid && d.dateValid && d.timeValid
                          && DS1307::ds1307_valid(utc);

    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    gpsStatus.alive = haveData;
    gpsStatus.fix = haveData && d.positionValid;
    gpsStatus.satellites = haveData ? d.satellitesUsed : 0;
    gpsStatus.latitudeE7 = d.latitudeE7;
    gpsStatus.longitudeE7 = d.longitudeE7;
    gpsStatus.config = gps.configStatus();
    if (timeGood) {
      gpsStatus.utc = utc;
      gpsStatus.timeTick = osKernelGetTickCount();
      gpsStatus.timeSeq = ++timeSeq;
    }
    osMutexRelease(gps_data_mutexHandle);
  }
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartEthTask */
/**
  * @brief  The W5500's driver thread: starts the interface, then runs it.
  *         DHCP, DNS and SNTP run inside the driver, on the W5500's socket 7.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEthTask */
void StartEthTask(void *argument)
{
  /* USER CODE BEGIN StartEthTask */
  const uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

  // Two sockets for the application (0, 1) and the service socket (7,
  // DHCP/DNS/SNTP); the W5500's other buffers are left unused.
  W5500::w5500_param_t param;
  for (int i = 0; i < W5500::w5500_sockets; ++i)
    param.rxBufKb[i] = param.txBufKb[i] = (i < 2 || i == W5500::w5500_service_socket) ? 2 : 0;
  param.pollMs = 100;            // INTn is wired: polling is only a backstop
  param.seed = uid;

  xNetInterface::Config cfg;
  cfg.maxSockets = 2;            // stream buffers from the FreeRTOS heap: 2 x 2 x 1 KB

  // Built here, after spi1_mutex exists. Static: the driver keeps a 1 KB buffer.
  static EthChip chip(param, &hspi1, W5500_CS_GPIO_Port, W5500_CS_Pin, spi1_mutexHandle);
  static xEthernet eth(chip, cfg);

  // Hardware reset: RSTn low for 2 ms, then about 60 ms for the W5500's PLL
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  osDelay(2);
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  osDelay(60);

  // A locally administered MAC from the MCU's unique ID (02:08:DC = WIZnet's OUI, local bit set)
  NetConfig net;
  net.mac = MacAddress(0x02, 0x08, 0xDC, (uint8_t)(uid >> 16), (uint8_t)(uid >> 8), (uint8_t)uid);
  net.dhcp = ETH_DHCP;
  if (!net.dhcp) {
    net.ip = IpAddress(ETH_STATIC_IP);
    net.subnet = IpAddress(ETH_STATIC_MASK);
    net.gateway = IpAddress(ETH_STATIC_GW);
  }

  if (!eth.begin(net)) {
    debugPrint("ETH: begin() failed: FreeRTOS heap too small\r\n");
    for(;;)
      osDelay(osWaitForever);
  }
  g_eth.store(&eth);             // the EXTI callback and displayTask may use it from here

  /* Infinite loop */
  eth.run();                     // never returns
  /* USER CODE END StartEthTask */
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
