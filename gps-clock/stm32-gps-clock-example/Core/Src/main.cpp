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
  *  The time from the GPS is applied to the STM32's own RTC (to a fraction
  *  of a second) and from there to the DS1307. The STM32 RTC is what the
  *  display shows and what the NTP server serves: a W5500 on SPI1 gives
  *  the board an address (DHCP) and answers NTP requests on UDP port 123
  *  from the STM32 RTC. Without a GPS fix, network time (SNTP) sets the
  *  RTCs instead; after a power cycle the DS1307 seeds the STM32 RTC.
  *
  *  With the GPS's PPS output wired to PA0, TIM2 latches each second's edge
  *  in hardware and the STM32 RTC is disciplined to it: phase by sub-second
  *  shifts, frequency by the RTC's smooth calibration (Core/Inc/pps_math.h).
  *  NMEA timing (to about 100-500 ms) is then only the fallback.
  *
  *  With the DS1307's SQW/OUT (1 Hz) wired to PA1, TIM2 latches its edges
  *  too: the DS1307's offset and drift are measured against the PPS and
  *  kept in its RAM, and after a power cycle the STM32 RTC is seeded from
  *  its edge with them (Core/Inc/ds1307_ref.h).
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
  *    EXTI8     PA8  (D9)  <- W5500 INTn
  *    TIM2_CH1  PA0  (A0)  <- GPS PPS (optional)
  *    TIM2_CH2  PA1  (A1)  <- DS1307 SQW/OUT (optional; 10k pull-up to 3.3 V)
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
#include "NtpServer.h"
#include "stm32_rtc.h"
#include "pps_math.h"
#include "ds1307_ref.h"
#include <atomic>
#include "Font5x7.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef DS1307::xds1307<Stm32HalI2CTransport> Ds1307Clock;
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
  uint32_t  timeTimer;     // TIM2's count (1 MHz) then: to pair it with its PPS edge
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

// The DS1307 is read once a second by readNow() (at a chosen place in its
// second when its SQW is wired); the driver's own period is the backstop
#define DS1307_READ_PERIOD_MS   5000UL

// The DS1307 is written from the STM32 RTC only this close after a second
// starts, since writing its seconds register restarts its 1 Hz divider
#define DS1307_SET_WINDOW_MS    100UL

// The STM32 RTC: LSE 32768 Hz / (PREDIV_A + 1) / (PREDIV_S + 1) = 1 Hz,
// with PREDIV_S + 1 = 8192 sub-second steps (122 us, NTP precision -13).
// PREDIV_A 3 is the smallest that keeps smooth calibration's CALP usable
// (CALP must stay 0 when PREDIV_A < 3: RM0394's smooth calibration section
// as quoted on ST's community forum, not read in the manual here).
#define RTC_ASYNCH_PREDIV       3
#define RTC_SYNCH_PREDIV        8191
#define NTP_PRECISION_LOG2      (-13)

// How late an NMEA RMC sentence is relative to the second it names. The GPS
// time is applied with this much added. Measure it (against the module's PPS
// output, or a good NTP server) and set it; 0 leaves the RTC late by it.
#define GPS_NMEA_DELAY_MS       0
// What the NTP server reports as the error of the time it was set to
#define GPS_ROOT_DISPERSION_US  100000UL   // NMEA timing, without PPS
#define NET_ROOT_DISPERSION_US  50000UL
#define NET_STRATUM             3          // assumes an upstream stratum 2 server (pool.ntp.org mostly is)
// After this long without a GPS or network set the NTP server says "not synchronised"
#define NTP_HOLDOVER_S          (24UL * 3600UL)

// PPS: TIM2 counts at 1 MHz (80 MHz / 80) and latches the PPS edge on PA0
#define PPS_TIMER_PRESCALER     79
#define PPS_ROOT_DISPERSION_US  1000UL     // locked: the RTC's 122 us step and the loop's 250 us band
#define PPS_TIMEOUT_MS          3000UL     // no PPS paired with an RMC for this long: back to NMEA timing

// DS1307 SQW/OUT on PA1 (TIM2_CH2): the DS1307 as a reference clock
#define SQW_TIMEOUT_MS          2500UL     // no edge for this long: the SQW isn't there
#define SQW_WAIT_MS             8000UL     // a cold start waits this long for the SQW check, then seeds without it
#define DS_REWRITE_US           200000LL   // with SQW, the DS1307 is rewritten only when this far off UTC
#define DS_SAVE_EVERY_S         600UL      // the measurement is saved to the DS1307's RAM this often
#define DS_ROOT_DISPERSION_US   2000UL     // a cold start seeded from a measured DS1307

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

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 1024 * 4,
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
/* Definitions for rtc_mutex */
osMutexId_t rtc_mutexHandle;
const osMutexAttr_t rtc_mutex_attributes = {
  .name = "rtc_mutex"
};
/* USER CODE BEGIN PV */
// Written by gpsTask, read by displayTask, guarded by gps_data_mutex
static GpsStatus gpsStatus;

// Set by ethTask once the interface has begun; read by displayTask and
// the INT pin's EXTI callback. xNetInterface's status calls are thread-safe.
static std::atomic<xEthernet*> g_eth{nullptr};

// The NTP server's clock: the STM32 RTC. Read in the W5500's driver thread
// when a request arrives; a short wait for rtc_mutex at most.
struct StmRtcNtpClock : NtpServer::Clock {
  bool ntpNow(uint64_t *t) override {
    uint32_t sec, frac;
    if (!StmRtc_Read(&sec, &frac, 2)) return false;
    *t = NtpServer::fromUnix(sec, frac);
    return true;
  }
};
static StmRtcNtpClock ntpClock;

// PPS edges, as TIM2 latched them: the last four, written by the capture
// interrupt, read by the clock task (a second apart, so no slot is reused
// while it is being read).
static volatile uint32_t g_ppsRing[4];
static std::atomic<uint32_t> g_ppsCount{0};
static NtpServer ntpServer(ntpClock, NTP_PRECISION_LOG2, 20);   // LSE: 20 ppm

// The DS1307's SQW falling edges, likewise (TIM2 channel 2), and how many.
static volatile uint32_t g_sqwRing[4];
static std::atomic<uint32_t> g_sqwCount{0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDisplayTask(void *argument);
void StartGpsTask(void *argument);
void StartEthTask(void *argument);

/* USER CODE BEGIN PFP */
static void debugPrint(const char *str);
static void drawScreen(idisplay::MonoCanvas &c, bool clockSet, const ClockTime &utc, const GpsStatus &g,
                       const char *ethLine, RtcSource source, const ClockTime &lastSet, bool ds1307Ok,
                       bool ppsLocked, int32_t calPpb, bool dsSqw);
static bool findPps(uint32_t rmcAt, uint32_t *ppsAt);
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

// The PPS edge an RMC (parsed at TIM2 count rmcAt) belongs to: the one
// before it, within a second, which itself came a second after the one
// before (a lone or glitched edge isn't trusted).
static bool findPps(uint32_t rmcAt, uint32_t *ppsAt)
{
  const uint32_t n = g_ppsCount.load(std::memory_order_acquire);
  for (uint32_t i = 1; i < 4 && i < n; ++i) {
    const uint32_t at = g_ppsRing[(n - i) & 3];
    if (ppsmath::rmcFollowsPps(at, rmcAt)) {
      if (!ppsmath::ppsIntervalOk(g_ppsRing[(n - i - 1) & 3], at))
        return false;
      *ppsAt = at;
      return true;
    }
  }
  return false;
}

// The latest DS1307 SQW edge before timer count `before` (within a second
// and a bit), a second after the one before it, and its number (the
// capture interrupt's count).
static bool findSqw(uint32_t before, uint32_t *at, uint32_t *num)
{
  const uint32_t n = g_sqwCount.load(std::memory_order_acquire);
  for (uint32_t i = 1; i < 4 && i < n; ++i) {
    const uint32_t e = g_sqwRing[(n - i) & 3];
    if (before - e < 1100000u) {
      if (!dsref::edgeIntervalOk(g_sqwRing[(n - i - 1) & 3], e))
        return false;
      *at = e;
      *num = n - i;
      return true;
    }
  }
  return false;
}

// "+12.3" from ppb, without printf float support
static void formatPpm(char *out, size_t len, int32_t ppb)
{
  const long a = labs((long)ppb);
  snprintf(out, len, "%c%ld.%ld", ppb < 0 ? '-' : '+', a / 1000, (a % 1000) / 100);
}

static void drawScreen(idisplay::MonoCanvas &c, bool clockSet, const ClockTime &utc, const GpsStatus &g,
                       const char *ethLine, RtcSource source, const ClockTime &lastSet, bool ds1307Ok,
                       bool ppsLocked, int32_t calPpb, bool dsSqw)
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

  if (clockSet) {
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
    drawCentered(c, 29, "waiting for time");
  }

  // Position
  if (g.alive && g.fix) {
    formatPosition(line, sizeof(line), g.latitudeE7, g.longitudeE7);
    drawCentered(c, 38, line);
  }

  // Ethernet
  c.drawText(0, 47, ethLine, false);

  // Where the clock's time came from
  switch (source) {
  case RtcSource::GpsPps:
    if (ppsLocked) {
      char ppm[12];
      formatPpm(ppm, sizeof(ppm), calPpb);
      snprintf(line, sizeof(line), "PPS lock %sppm", ppm);
      break;
    }
    // fall through: PPS lost, the RTC holds the time it was locked to
  case RtcSource::Gps:
  case RtcSource::Network:
    snprintf(line, sizeof(line), "RTC synced %02u:%02uZ %s", lastSet.hour, lastSet.minute,
             source == RtcSource::GpsPps ? "PPS" : source == RtcSource::Gps ? "GPS" : "NTP");
    break;
  case RtcSource::Ds1307:
    snprintf(line, sizeof(line), dsSqw ? "RTC from DS1307 SQW" : "RTC from DS1307");
    break;
  default:
    snprintf(line, sizeof(line), ds1307Ok ? "RTC not set" : "RTC not set, no DS1307");
    break;
  }
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
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // PPS capture: TIM2 free-running at 1 MHz
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);   // DS1307 SQW (1 Hz) falling edges, same timer
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

  /* creation of rtc_mutex */
  rtc_mutexHandle = osMutexNew(&rtc_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  StmRtc_Begin(&hrtc, rtc_mutexHandle);
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  hrtc.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  // The calendar survives a reset (the RTC runs on as long as VBAT has
  // power): keep it. Year 00 leaves INITS clear, so the default below
  // reads as "not set" (StmRtc_IsSet()) until GPS, network or DS1307 time.
  if (__HAL_RTC_IS_CALENDAR_INITIALIZED(&hrtc))
  {
    // HAL_RTC_Init() leaves a running calendar alone, prescalers too. An
    // RTC set by firmware with other prescalers gets these now: the date
    // and time stay, the second restarts (PPS or GPS puts the phase right).
    const uint32_t prer = ((uint32_t)RTC_ASYNCH_PREDIV << RTC_PRER_PREDIV_A_Pos) | RTC_SYNCH_PREDIV;
    if (hrtc.Instance->PRER != prer)
    {
      __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
      if (RTC_EnterInitMode(&hrtc) == HAL_OK)
      {
        hrtc.Instance->PRER = RTC_SYNCH_PREDIV;
        hrtc.Instance->PRER |= (uint32_t)RTC_ASYNCH_PREDIV << RTC_PRER_PREDIV_A_Pos;
        (void)RTC_ExitInitMode(&hrtc);
      }
      __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
      hrtc.State = HAL_RTC_STATE_READY;
    }
    return;
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function: a free-running 32-bit 1 MHz counter,
  *        channel 1 capturing the GPS's PPS rising edge on PA0, channel 2
  *        the DS1307's SQW falling edge on PA1
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
  htim2.Init.Prescaler = PPS_TIMER_PRESCALER;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;   // a few samples: ignores glitches shorter than ~0.1 us
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// The PPS edge, latched by TIM2 channel 1 in hardware: the count is exact
// whatever the interrupt's latency
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    const uint32_t n = g_ppsCount.load(std::memory_order_relaxed);
    g_ppsRing[n & 3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    g_ppsCount.store(n + 1, std::memory_order_release);
  }
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    const uint32_t n = g_sqwCount.load(std::memory_order_relaxed);
    g_sqwRing[n & 3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    g_sqwCount.store(n + 1, std::memory_order_release);
  }
}

// The W5500's INTn (PA8): wakes the network driver thread at once
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  xEthernet *eth = g_eth.load();
  if (GPIO_Pin == W5500_INT_Pin && eth != nullptr)
    eth->interruptFromIsr();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDisplayTask */
/**
  * @brief  The clock task. Owns the DS1307 and the SSD1306 (neither driver
  *         is locked) and sets the STM32 RTC: from GPS time when there is
  *         a fix, else network time, else (after a power cycle) the DS1307.
  *         The DS1307 is then set from the STM32 RTC. Tells the NTP server
  *         where its time came from. Redraws when the second changes.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Built here, not as globals: the transports need i2c1_mutex, which only
  // exists once main() has created it. Static, to keep them (the SSD1306's
  // 1 KB frame buffer above all) off this task's stack.
  DS1307::ds1307_param_t dsParam;
  dsParam.period_ms = DS1307_READ_PERIOD_MS;
  static Ds1307Clock ds1307(dsParam, &hi2c1, DS1307_ADDR, i2c1_mutexHandle);
  static OledDisplay oled(SSD1306::ssd1306_default_param(64), &hi2c1, SSD1306_I2C_ADDR, i2c1_mutexHandle);

  char msg[360];
  int lastSecond = -1;
  bool lastClockSet = false;
  bool redrawPending = true;
  GpsStatus g = {};
  ClockTime ds = {};             // the DS1307's last reading
  bool dsOk = false;

  // Where the STM32 RTC's time came from, and when (UTC). A reset keeps
  // both: the RTC runs on, and the backup registers hold the rest.
  RtcSource source = RtcSource::None;
  uint32_t lastSetUnix = 0;
  uint32_t lastSetTick = 0;
  bool ntpSynced = false;
  if (StmRtc_LastSet(&source, &lastSetUnix)) {
    uint32_t nowSec, frac;
    const bool fresh = StmRtc_Read(&nowSec, &frac, 10) && nowSec - lastSetUnix < NTP_HOLDOVER_S;
    if (fresh && (source == RtcSource::Gps || source == RtcSource::GpsPps)) {
      ntpServer.setReference(1, NtpServer::refIdAscii(source == RtcSource::GpsPps ? "PPS" : "GPS"),
                             NtpServer::fromUnix(lastSetUnix, 0), 0, GPS_ROOT_DISPERSION_US);
      ntpSynced = true;
    } else if (fresh && source == RtcSource::Network) {
      ntpServer.setReference(NET_STRATUM, 0, NtpServer::fromUnix(lastSetUnix, 0), 0, NET_ROOT_DISPERSION_US);
      ntpSynced = true;
    }
    // Due again RTC_RESYNC_INTERVAL_MS after the set, counted on this boot's tick
    const uint32_t ageMs = fresh ? (nowSec - lastSetUnix) * 1000u : RTC_RESYNC_INTERVAL_MS;
    lastSetTick = osKernelGetTickCount() - (ageMs < RTC_RESYNC_INTERVAL_MS ? ageMs : RTC_RESYNC_INTERVAL_MS);
  }

  uint32_t usedTimeSeq = 0;      // the GPS time last applied, by its timeSeq

  // PPS discipline
  ppsmath::Discipline disc(StmRtc_CalibrationPpb());   // RTC_CALR survives a reset
  uint32_t usedPpsSeq = 0;       // the GPS time last paired with a PPS edge
  bool havePps = false;
  uint32_t lastPpsTick = 0;
  int64_t lastPpsErrUs = 0;
  uint32_t nmeaDelayMs = 0;      // how late the RMC came after its PPS edge: measured
  bool dsCopyPending = false;    // write the DS1307 from the STM32 RTC at the next second
  bool dsWriting = false;

  // The DS1307's SQW as a reference (Core/Inc/ds1307_ref.h)
  dsref::LabelCheck labels;      // is the falling edge where its seconds tick?
  dsref::Meter meter;            // its offset and drift against the PPS
  dsref::Record rec;             // what its RAM holds
  bool haveRec = false;
  bool ramAsked = false, ramRead = false;
  bool recDirty = false, ramSaving = false;
  uint32_t savedSpan = 0;
  bool savedThisRun = false;
  uint32_t lastSqwTick = 0;
  bool sqwEver = false;
  uint32_t readEdgeNum = 0xFFFFFFFFu;  // the edge a read was last asked for
  uint32_t lastDsAskTick = 0;
  bool readTimed = false;        // the read in flight was issued between readFrom and readTo
  uint32_t readFrom = 0, readTo = 0;
  bool haveLabel = false;        // the latest labelled edge
  uint32_t labelAt = 0, label = 0, labelTick = 0;
  int64_t dsOffsetUs = 0;        // the latest measured DS1307 - UTC
  bool dsAgree = true;
  bool seededFromSqw = false;
  uint32_t dsFirstOkTick = 0;
  bool dsSeenOk = false;
  bool ppsActive = false;        // last time round
  uint32_t ppsRefSec = 0, ppsRefAt = 0, ppsRefPer = 1000000;
  bool ppsRef = false;           // the latest PPS edge paired with its second: TIM2 counts to UTC

  // Rewrite the DS1307 from the STM32 RTC? Always without a measured SQW;
  // with one, only when it has wandered DS_REWRITE_US from UTC (each write
  // restarts its divider, and with it the drift measurement).
  auto dsCopyWanted = [&]() -> bool {
    if (!labels.verified() || !meter.have() || !ppsActive)
      return true;
    const int64_t o = meter.offsetUs();
    return o > DS_REWRITE_US || o < -DS_REWRITE_US;
  };
  char ethLine[24] = "";
  char lastEthLine[24] = "";

  /* Infinite loop */
  for(;;)
  {
    // The DS1307: read once a second. With its SQW wired, early or late in
    // its second as the falling-edge check wants (Core/Inc/ds1307_ref.h).
    {
      const uint32_t cnt = TIM2->CNT, t = osKernelGetTickCount();
      uint32_t eAt, eNum;
      const bool edge = findSqw(cnt, &eAt, &eNum);
      if (edge) {
        lastSqwTick = t;
        sqwEver = true;
      }
      if (edge && eNum != readEdgeNum) {
        const uint32_t since = cnt - eAt;
        const dsref::ReadPhase want = labels.verified() ? dsref::ReadPhase::Early : labels.wanted();
        if ((want == dsref::ReadPhase::Early && since >= 50000u && since < 300000u)
            || (want == dsref::ReadPhase::Late && since >= 600000u && since < 850000u)) {
          ds1307.readNow();
          readEdgeNum = eNum;
          lastDsAskTick = t;
        }
      } else if (!edge && t - lastDsAskTick >= 1000u) {
        ds1307.readNow();
        lastDsAskTick = t;
      }
    }
    // (sleeps in osDelay up to 100 ms here). When a read is issued, note
    // the timer either side: its START lies between.
    const DS1307::ds1307_state_t dsBefore = ds1307.state();
    const uint32_t issueFrom = TIM2->CNT;
    ds1307.main(osKernelGetTickCount());
    const uint32_t issueTo = TIM2->CNT;
    if (dsBefore == DS1307::ds1307_read_state && ds1307.state() == DS1307::ds1307_wait_read_state) {
      readTimed = true;
      readFrom = issueFrom;
      readTo = issueTo;
    }
    if (ds1307.newData()) {
      dsOk = ds1307.getTime(&ds);
      uint32_t eAt, eNum;
      if (dsOk && readTimed && ds1307.running() && findSqw(readFrom, &eAt, &eNum)) {
        const dsref::ReadPhase ph = dsref::readPhase(eAt, readFrom, readTo);
        if (ph != dsref::ReadPhase::None) {
          const uint32_t L = DS1307::ds1307_to_unix(ds);   // the DS1307's second from that edge on
          labels.add(ph, (int64_t)L - (int64_t)eNum);
          haveLabel = true;
          labelAt = eAt;
          label = L;
          labelTick = osKernelGetTickCount();
          const int32_t fromPps = (int32_t)(eAt - ppsRefAt);
          if (labels.verified() && ppsActive && ppsRef && fromPps > -2000000 && fromPps < 2000000) {
            // Against the PPS: the edge's UTC, TIM2 scaled by its measured rate
            const int64_t utcUs = (int64_t)ppsRefSec * 1000000 + (int64_t)fromPps * 1000000 / (int64_t)ppsRefPer;
            dsOffsetUs = (int64_t)L * 1000000 - utcUs;
            if (!meter.add(utcUs, dsOffsetUs))
              savedThisRun = false;           // a new line: save its offset soon
            if (!savedThisRun || meter.spanS() >= savedSpan + DS_SAVE_EVERY_S) {
              rec = meter.record(rec);
              haveRec = true;
              recDirty = true;
              savedSpan = meter.spanS();
              savedThisRun = true;
            }
          } else if (labels.verified() && !ppsActive && haveRec && rec.refUnix != 0
                     && (source == RtcSource::GpsPps || source == RtcSource::Ds1307)) {
            // Holdover: does the DS1307 still agree with the STM32 RTC?
            uint32_t sec, frac, at;
            if (StmRtc_ReadWithCounter(&sec, &frac, &TIM2->CNT, &at, 5)) {
              const int64_t rtcUs = (int64_t)sec * 1000000 + (int64_t)(((uint64_t)frac * 1000000u) >> 32)
                                    - (int64_t)(uint32_t)(at - eAt);
              dsOffsetUs = (int64_t)L * 1000000 - rtcUs;
              const int64_t predicted = (int64_t)L * 1000000 - dsref::utcAtEdgeUs(rec, L);
              const bool agree = dsref::agrees(dsOffsetUs, predicted, sec - rec.refUnix);
              if (!agree && dsAgree && ntpSynced) {
                ntpServer.setUnsynchronized();  // one of the two oscillators is wrong: say so
                ntpSynced = false;
              }
              dsAgree = agree;
            }
          }
        }
      }
      readTimed = false;
    }
    if (ds1307.state() == DS1307::ds1307_err_state)
      dsOk = false;
    if (dsOk && !dsSeenOk) {
      dsSeenOk = true;
      dsFirstOkTick = osKernelGetTickCount();
    }
    // Its RAM: read once, then the record written when it changes
    if (dsOk && !ramAsked) {
      ds1307.readRam();
      ramAsked = true;
    }
    if (ramAsked && !ramRead) {
      const DS1307::ds1307_set_status_t rs = ds1307.ramReadStatus();
      if (rs == DS1307::ds1307_set_status_t::Done) {
        uint8_t ram[DS1307::ds1307_ram_size];
        if (ds1307.getRam(ram) && dsref::decode(ram, &rec))
          haveRec = true;
        ramRead = true;
      } else if (rs == DS1307::ds1307_set_status_t::Failed) {
        ramAsked = false;                     // ask again
      }
    }
    if (ramSaving && ds1307.ramWriteStatus() != DS1307::ds1307_set_status_t::Pending) {
      if (ds1307.ramWriteStatus() == DS1307::ds1307_set_status_t::Failed)
        recDirty = true;
      ramSaving = false;
    }
    if (recDirty && !ramSaving && ramRead) {
      uint8_t buf[dsref::kRecordLen];
      dsref::encode(rec, buf);
      if (ds1307.writeRam(0, buf, dsref::kRecordLen)) {
        recDirty = false;
        ramSaving = true;
      }
    }
    const bool sqwActive = sqwEver && (osKernelGetTickCount() - lastSqwTick) < SQW_TIMEOUT_MS;

    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    g = gpsStatus;
    osMutexRelease(gps_data_mutexHandle);

    const uint32_t tick = osKernelGetTickCount();
    xEthernet *eth = g_eth.load();
    const bool due = source == RtcSource::None || source == RtcSource::Ds1307
                     || (tick - lastSetTick) >= RTC_RESYNC_INTERVAL_MS;
    const bool fromGps = source == RtcSource::Gps || source == RtcSource::GpsPps;

    // 0. The PPS edge of the second the latest RMC names, if one was latched:
    //    measure the RTC against it, and correct phase and frequency.
    if (g.timeSeq != usedPpsSeq && (tick - g.timeTick) < GPS_TIME_MAX_AGE_MS) {
      usedPpsSeq = g.timeSeq;
      uint32_t ppsAt;
      if (findPps(g.timeTimer, &ppsAt)) {
        const uint32_t S = DS1307::ds1307_to_unix(g.utc);   // the second the edge started
        nmeaDelayMs = (g.timeTimer - ppsAt) / 1000;
        {
          // For timing the DS1307's edges: this edge's second, and TIM2's
          // counts per second (from the edge before, already checked)
          const uint32_t n = g_ppsCount.load(std::memory_order_acquire);
          for (uint32_t i = 1; i < 4 && i < n; ++i)
            if (g_ppsRing[(n - i) & 3] == ppsAt) {
              ppsRefPer = ppsAt - g_ppsRing[(n - i - 1) & 3];
              break;
            }
          ppsRefSec = S;
          ppsRefAt = ppsAt;
          ppsRef = true;
        }
        int64_t err = ppsmath::Discipline::kSetUs;          // not set yet: set it
        uint32_t sec, frac, at;
        bool measured = !StmRtc_IsSet();
        if (!measured && StmRtc_ReadWithCounter(&sec, &frac, &TIM2->CNT, &at, 5)) {
          err = ppsmath::rtcErrorUs(sec, frac, at, S, ppsAt);
          measured = true;
        }
        lastPpsErrUs = err;
        // RTC busy just now: skip this second rather than take it as unset
        const ppsmath::Discipline::Step st = measured ? disc.update(S, err) : ppsmath::Discipline::Step();
        if (st.set) {
          const uint32_t el = TIM2->CNT - ppsAt;            // time since the edge
          if (StmRtc_Set(S + el / 1000000u, (uint32_t)(((uint64_t)(el % 1000000u) << 32) / 1000000u),
                         RtcSource::GpsPps)) {
            source = RtcSource::GpsPps;
            lastSetUnix = S;
            lastSetTick = tick;
            dsCopyPending = dsCopyWanted();
          }
        }
        if (st.shift)
          StmRtc_ShiftUs(st.shiftUs);
        if (st.calibrate)
          StmRtc_SetCalibrationPpb(st.calPpb);
        havePps = true;
        lastPpsTick = tick;
        if (disc.locked()) {
          ntpServer.setReference(1, NtpServer::refIdAscii("PPS"), NtpServer::fromUnix(S, 0), 0, PPS_ROOT_DISPERSION_US);
          ntpSynced = true;
          if (source != RtcSource::GpsPps || (tick - lastSetTick) >= RTC_RESYNC_INTERVAL_MS) {
            source = RtcSource::GpsPps;
            lastSetUnix = S;
            lastSetTick = tick;
            StmRtc_NoteSource(RtcSource::GpsPps, S);
            dsCopyPending = dsCopyWanted(); // the DS1307 follows, hourly (with SQW: when it wanders)
          }
        }
      }
    }
    ppsActive = havePps && (tick - lastPpsTick) < PPS_TIMEOUT_MS;
    if (havePps && !ppsActive) {
      havePps = false;                      // PPS lost: NMEA timing from here, the loop starts over
      disc.restart();
    }

    // 1. GPS time from NMEA, without PPS: when it is due, or when the clock came from anywhere else
    if (ppsActive) {
      // the PPS keeps the RTC right
    }
    else if ((due || !fromGps) && g.timeSeq != usedTimeSeq && (tick - g.timeTick) < GPS_TIME_MAX_AGE_MS) {
      usedTimeSeq = g.timeSeq;
      // g.utc is the second the RMC named; time has moved on since it was parsed
      const uint32_t ms = GPS_NMEA_DELAY_MS + (tick - g.timeTick);
      const uint32_t unixSec = DS1307::ds1307_to_unix(g.utc) + ms / 1000;
      const uint32_t frac = (uint32_t)(((uint64_t)(ms % 1000) << 32) / 1000);
      if (StmRtc_Set(unixSec, frac, RtcSource::Gps)) {
        ntpServer.setReference(1, NtpServer::refIdAscii("GPS"), NtpServer::fromUnix(unixSec, frac), 0,
                               GPS_ROOT_DISPERSION_US);
        ntpSynced = true;
        source = RtcSource::Gps;
        lastSetUnix = unixSec;
        lastSetTick = tick;
        dsCopyPending = true;
      }
    }
    // 2. Network time (SNTP), when there is no GPS fix and the clock isn't GPS time still in holdover
    else if (due && !g.fix && eth != nullptr && eth->timeValid()
             && !(fromGps && ntpSynced)) {
      const uint64_t unixMs = eth->unixTimeMs();
      const uint32_t unixSec = (uint32_t)(unixMs / 1000);
      const uint32_t frac = (uint32_t)(((unixMs % 1000) << 32) / 1000);
      if (StmRtc_Set(unixSec, frac, RtcSource::Network)) {
        const IpAddress upstream = eth->address().ntp;
        ntpServer.setReference(NET_STRATUM, upstream.isZero() ? 0 : NtpServer::refIdIp(upstream),
                               NtpServer::fromUnix(unixSec, frac), 0, NET_ROOT_DISPERSION_US);
        ntpSynced = true;
        source = RtcSource::Network;
        lastSetUnix = unixSec;
        lastSetTick = tick;
        dsCopyPending = true;
      }
    }
    // 3. After a power cycle: the DS1307's time (whole seconds), until better comes
    //    With the SQW checked: from a labelled edge, corrected by the offset
    //    and drift kept in its RAM. Without: whole seconds, +0.5 s.
    else if (!StmRtc_IsSet() && dsOk && ds1307.running()) {
      const bool waitedOut = (tick - dsFirstOkTick) >= SQW_WAIT_MS;
      const bool fresh = haveLabel && (tick - labelTick) < 2000u;
      if (labels.verified() && fresh && sqwActive && (ramRead || waitedOut)) {
        const bool measured = haveRec && rec.refUnix != 0;
        const int64_t edgeUtcUs = measured ? dsref::utcAtEdgeUs(rec, label) : (int64_t)label * 1000000;
        const int64_t nowUs = edgeUtcUs + (int64_t)(uint32_t)(TIM2->CNT - labelAt);
        const uint32_t sec = (uint32_t)(nowUs / 1000000);
        const uint32_t frac = (uint32_t)(((uint64_t)(nowUs % 1000000) << 32) / 1000000u);
        if (StmRtc_Set(sec, frac, RtcSource::Ds1307)) {
          source = RtcSource::Ds1307;
          seededFromSqw = true;
          if (measured && sec - rec.refUnix < NTP_HOLDOVER_S) {
            // Holding over: the DS1307 carried the PPS's time across the power cycle
            ntpServer.setReference(1, NtpServer::refIdAscii("PPS"), NtpServer::fromUnix(rec.refUnix, 0), 0,
                                   DS_ROOT_DISPERSION_US);
            ntpSynced = true;
            lastSetUnix = rec.refUnix;
          }
        }
      } else if (labels.mismatch() || waitedOut) {
        // No usable SQW (after giving it SQW_WAIT_MS to show itself)
        if (StmRtc_Set(DS1307::ds1307_to_unix(ds), 0x80000000u, RtcSource::Ds1307))   // +0.5 s: its phase is unknown
          source = RtcSource::Ds1307;
      }
    }

    // The STM32 RTC is the clock from here on
    uint32_t nowSec = 0, nowFrac = 0;
    const bool clockSet = StmRtc_Read(&nowSec, &nowFrac, 10);
    ClockTime now = {};
    if (clockSet)
      DS1307::ds1307_from_unix(nowSec, &now);

    // Holdover: too long since GPS or network time
    if (ntpSynced && clockSet && nowSec - lastSetUnix > NTP_HOLDOVER_S) {
      ntpServer.setUnsynchronized();
      ntpSynced = false;
    }

    // The DS1307 follows the STM32 RTC, written early in a second
    if (dsCopyPending && clockSet && !dsWriting && nowFrac < (uint32_t)(((uint64_t)DS1307_SET_WINDOW_MS << 32) / 1000)) {
      if (ds1307.setTime(now)) {
        dsCopyPending = false;
        dsWriting = true;
      }
    }
    if (dsWriting && ds1307.setStatus() != DS1307::ds1307_set_status_t::Pending) {
      if (ds1307.setStatus() == DS1307::ds1307_set_status_t::Failed) {
        dsCopyPending = true;    // try again at the next second
      } else {
        // Its divider restarted: a new phase, measured afresh. Until then
        // the RAM says the offset is unknown (the drift stays).
        meter.restart();
        labels.restart();
        haveLabel = false;
        savedThisRun = false;
        if (haveRec) {
          rec.refUnix = 0;
          rec.offsetUs = 0;
          recDirty = true;
        }
      }
      dsWriting = false;
    }

    formatEth(ethLine, sizeof(ethLine));
    if (strcmp(ethLine, lastEthLine) != 0) {
      strcpy(lastEthLine, ethLine);
      redrawPending = true;
    }

    if ((clockSet && now.second != lastSecond) || clockSet != lastClockSet) {
      lastSecond = clockSet ? now.second : -1;
      lastClockSet = clockSet;
      redrawPending = true;

      const NtpServer::Stats ns = ntpServer.stats();
      char calText[12];
      formatPpm(calText, sizeof(calText), disc.calPpb());
      char dsDrift[12];
      formatPpm(dsDrift, sizeof(dsDrift), rec.driftPpb);
      snprintf(msg, sizeof(msg),
               "RTC %04u-%02u-%02u %02u:%02u:%02u UTC (%s) | DS1307 %s | GPS %s sats=%u cfg=%s | %s | NTP %s req=%lu rep=%lu"
               " | PPS %s err=%ldus cal=%sppm nmea=%lums | SQW %s%s off=%ldus drift=%sppm/%lus%s\r\n",
               now.year, now.month, now.day, now.hour, now.minute, now.second,
               source == RtcSource::GpsPps ? "PPS" : source == RtcSource::Gps ? "GPS"
                 : source == RtcSource::Network ? "network" : source == RtcSource::Ds1307 ? "DS1307" : "not set",
               !dsOk ? "not responding" : (ds1307.running() ? "ok" : "stopped"),
               !g.alive ? "no data" : (g.fix ? "fix" : "no fix"), g.satellites, configName(g.config), ethLine,
               ntpSynced ? "synced" : "unsynced", (unsigned long)ns.requests, (unsigned long)ns.replies,
               !ppsActive ? "none" : (disc.locked() ? "locked" : "locking"), (long)lastPpsErrUs, calText,
               (unsigned long)nmeaDelayMs,
               !sqwActive ? "none" : labels.mismatch() ? "MISMATCH" : labels.verified() ? "ok" : "checking",
               dsAgree ? "" : " DISAGREE", (long)dsOffsetUs, dsDrift, (unsigned long)rec.spanS,
               oled.failed() ? " | SSD1306 not responding" : "");
      debugPrint(msg);
    }

    // Draw only between flushes (iDisplayDevice rule). The driver sends
    // just the pages that changed.
    if (redrawPending && oled.idle()) {
      redrawPending = false;
      ClockTime lastSet;
      DS1307::ds1307_from_unix(lastSetUnix, &lastSet);
      drawScreen(oled, clockSet, now, g, ethLine, source, lastSet, dsOk, ppsActive && disc.locked(), disc.calPpb(),
                 seededFromSqw);
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
      gpsStatus.timeTimer = TIM2->CNT;
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

  // Two sockets for the application (0, 1), the NTP server (6, the UDP
  // responder: UDP port 123) and the service socket (7, DHCP/DNS/SNTP
  // client); the W5500's other buffers are left unused.
  W5500::w5500_param_t param;
  for (int i = 0; i < W5500::w5500_sockets; ++i)
    param.rxBufKb[i] = param.txBufKb[i] = (i < 2 || i >= 6) ? 2 : 0;
  param.responder = &ntpServer;  // answers in this thread, from the STM32 RTC
  param.responderPort = NtpServer::kPort;
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
