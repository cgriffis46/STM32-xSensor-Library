/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : GPS clock and NTP server on a NUCLEO-F207ZG
  *
  *  The board:
  *    NUCLEO-F207ZG (STM32F207ZG, 120 MHz from the ST-LINK's 8 MHz, LAN8742A
  *    Ethernet PHY on RMII, 32.768 kHz LSE)
  *    X-NUCLEO-GNSS2A1 (Teseo-VIC3DA) on the Arduino connector: NMEA on
  *    USART6 (D0/D1) at 115200, its PPS on D6 (PE9, TIM1_CH1)
  *    The ST-LINK's virtual COM port (USART3): the console
  *
  *  The clock is the Ethernet MAC's IEEE 1588 system time (ptp_clock.h),
  *  not the RTC: the F2's RTC has no sub-seconds. It is set from GPS time,
  *  and its rate is steered to the PPS (Core/Inc/ptp_math.h: a PI loop on
  *  the phase, slewing, never stepping once close). The NTP server answers
  *  from it in lwIP's tcpip thread (raw UDP API, port 123). The RTC keeps
  *  whole seconds across a reset, re-aligned to the clock every minute
  *  while it is locked (f2_rtc.h).
  *
  *  The PPS edge is latched by TIM1 (16-bit, 1 MHz), started by TIM2 (32-bit,
  *  1 MHz) so the two count in step, and widened to TIM2's 32 bits. The
  *  capture interrupt reads the system time and TIM2 together, so each
  *  edge comes with the clock's reading a few microseconds after it.
  *
  *  The drivers from Claude-iTransport (iTransport/SOURCE.txt):
  *    xteseo<Stm32HalUartTransport>   isensor/teseo
  *    NtpServer                       iNetTransport/sntp
  *    Stm32HalUartTransport           the console as well
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
#include "lwip/tcpip.h"
#include "lwip/udp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "ethernetif.h"
#include "Stm32HalUartTransport.h"
#include "ds1307.h"          // only for its date helpers (ds1307_to_unix, ...)
#include "xteseo.h"
#include "NtpServer.h"
#include "ptp_clock.h"
#include "ptp_math.h"
#include "f2_rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef xteseo<Stm32HalUartTransport> GpsReceiver;
typedef DS1307::ds1307_time_t ClockTime;

// What clockTask needs from the GPS, copied out by gpsTask
typedef struct {
  bool     alive;          // the GPS is sending good NMEA
  bool     fix;
  uint8_t  satellites;     // used in the fix (GGA)
  teseo_config_status_t config;
  // UTC from the last RMC with a fix, date and time; timeSeq counts them
  ClockTime utc;
  uint32_t  timeTick;      // osKernelGetTickCount() when it was parsed
  uint32_t  timeTimer;     // TIM2's count (1 MHz) then: to pair it with its PPS edge
  uint32_t  timeSeq;
} GpsStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The F2's RTC: LSE 32768 Hz / (127 + 1) / (255 + 1) = 1 Hz
#define RTC_ASYNCH_PREDIV       127
#define RTC_SYNCH_PREDIV        255

// A GPS time older than this is not used
#define GPS_TIME_MAX_AGE_MS     500UL
// How late an NMEA RMC sentence is relative to the second it names (used
// without PPS). Measure it with PPS ("nmea=" on the console) and set it.
#define GPS_NMEA_DELAY_MS       0
// Without PPS, set the clock from NMEA this often (its rate isn't steered)
#define NMEA_RESYNC_INTERVAL_MS (10UL * 60UL * 1000UL)
// After the PPS is lost the clock runs on at the learned rate; NMEA time
// takes over after this long
#define PPS_HOLDOVER_MS         (60UL * 60UL * 1000UL)

// The NTP server's view of its clock. Precision: the system time's step,
// 43 x 2^-31 s (about 20 ns), so -25; replies are timestamped in software,
// in the tcpip thread, which costs tens of microseconds of their accuracy.
#define NTP_PRECISION_LOG2      (-25)
#define NTP_DRIFT_PPM           15         // RFC 5905's frequency tolerance
#define PPS_ROOT_DISPERSION_US  10UL
#define GPS_ROOT_DISPERSION_US  100000UL   // NMEA timing, without PPS
#define RTC_ROOT_DISPERSION_US  3000UL     // after a reset, from the RTC's whole seconds
#define NTP_HOLDOVER_S          (24UL * 3600UL)

// PPS: TIM2 counts at 1 MHz (60 MHz APB1 timer clock / 60); TIM1, started
// by TIM2, counts at 1 MHz too (120 MHz APB2 timer clock / 120) and latches
// the PPS edge on PE9.
#define TIM2_PRESCALER          59
#define TIM1_PRESCALER          119
#define PPS_TIMEOUT_MS          3000UL
// While locked: the RTC re-aligned this often, the frequency saved this often
#define RTC_ALIGN_INTERVAL_S    60UL
#define FREQ_SAVE_INTERVAL_S    3600UL

// The X-NUCLEO-GNSS2A1's Teseo-VIC3DA: 115200 baud (ST's X-CUBE-GNSS1
// readme); reset low 10 ms, then 150 ms to start (ST's driver)
#define GNSS_BAUD               115200
#define GNSS_RESET_LOW_MS       10
#define GNSS_RESET_WAIT_MS      150

// Ethernet address: DHCP, or this static one
#define ETH_DHCP                1
#define ETH_STATIC_IP           192, 168, 1, 50
#define ETH_STATIC_MASK         255, 255, 255, 0
#define ETH_STATIC_GW           192, 168, 1, 1

// The console ring (USART3, 115200); lines are dropped when it is full
#define CONSOLE_BAUD            115200
#define CONSOLE_RING            4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for clockTask */
osThreadId_t clockTaskHandle;
const osThreadAttr_t clockTask_attributes = {
  .name = "clockTask",
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
/* Definitions for netTask */
osThreadId_t netTaskHandle;
const osThreadAttr_t netTask_attributes = {
  .name = "netTask",
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
/* Definitions for gps_data_mutex */
osMutexId_t gps_data_mutexHandle;
const osMutexAttr_t gps_data_mutex_attributes = {
  .name = "gps_data_mutex"
};
/* Definitions for console_mutex */
osMutexId_t console_mutexHandle;
const osMutexAttr_t console_mutex_attributes = {
  .name = "console_mutex"
};
/* USER CODE BEGIN PV */
// Written by gpsTask, read by clockTask, guarded by gps_data_mutex
static GpsStatus gpsStatus;

// The interface, once netTask has added it; and whether the MAC is up
static struct netif g_netif;
static std::atomic<bool> g_netUp{false};

// The clock has been set this boot (from GPS or the RTC): until then the
// system time counts from 1970 and the NTP server says nothing.
static std::atomic<bool> g_clockSet{false};

// The NTP server's clock: the MAC's system time. Read in the tcpip thread
// when a request arrives; lock-free.
struct PtpNtpClock : NtpServer::Clock {
  bool ntpNow(uint64_t *t) override {
    uint32_t sec, frac;
    if (!g_clockSet.load(std::memory_order_acquire) || !PtpClock_Read(&sec, &frac))
      return false;
    *t = NtpServer::fromUnix(sec, frac);
    return true;
  }
};
static PtpNtpClock ntpClock;
static NtpServer ntpServer(ntpClock, NTP_PRECISION_LOG2, NTP_DRIFT_PPM);

// PPS edges, as TIM1 latched them (widened to TIM2's 32 bits), each with
// the system time and TIM2 read together just after: the last four,
// written by the capture interrupt (a second apart, so no slot is reused
// while it is read).
static volatile uint32_t g_ppsAt[4];
static volatile uint32_t g_ppsClockSec[4], g_ppsClockFrac[4], g_ppsReadAt[4];
static volatile uint8_t g_ppsClockOk[4];
static std::atomic<uint32_t> g_ppsCount{0};
static int g_timSync = 0;                       // TIM2 - TIM1 (low 16 bits) at start-up
static bool g_lseOk = false;

// The console: a ring the tasks write (under console_mutex) and
// consoleTask hands to USART3; keys typed there
static char g_console[CONSOLE_RING];
static uint32_t g_conHead = 0, g_conTail = 0;     // head written by writers, tail by consoleTask
static uint32_t g_conDropped = 0;
static std::atomic<bool> g_helpRequest{true};   // greet at start-up
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartClockTask(void *argument);
void StartGpsTask(void *argument);
void StartNetTask(void *argument);
void StartConsoleTask(void *argument);

/* USER CODE BEGIN PFP */
static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static bool findPps(uint32_t rmcAt, uint32_t *slot);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// The HAL UART interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32UartItCallbacks.cpp): don't define them here.

// printf to the console. Tasks only (not interrupts); a line that doesn't
// fit is dropped and counted.
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
  const uint32_t used = g_conHead - g_conTail;
  if (used + (uint32_t) n > CONSOLE_RING) {
    ++g_conDropped;
  } else {
    for (int i = 0; i < n; ++i)
      g_console[(g_conHead + i) % CONSOLE_RING] = line[i];
    g_conHead += n;
  }
  osMutexRelease(console_mutexHandle);
}

// What the terminal types: 'h' prints the help. Interrupt context.
struct ConsoleKeys : iTransportRxSink {
  void onByteReceived(uint8_t b) override {
    if (b == 'h' || b == 'H' || b == '?')
      g_helpRequest.store(true);
  }
};
static ConsoleKeys consoleKeys;

static const char *configName(teseo_config_status_t s)
{
  switch (s) {
  case teseo_config_status_t::Done:       return "done";
  case teseo_config_status_t::Rejected:   return "rejected";
  case teseo_config_status_t::NoAnswer:   return "no answer";
  case teseo_config_status_t::Unverified: return "unverified";
  default:                                return "pending";
  }
}

// "+12.345" ppm from ppb, without printf float support
static void formatPpm(char *out, size_t len, int32_t ppb)
{
  const long a = labs((long)ppb);
  snprintf(out, len, "%c%ld.%03ld", ppb < 0 ? '-' : '+', a / 1000, a % 1000);
}

// The PPS slot an RMC (parsed at TIM2 count rmcAt) belongs to: the edge
// before it, within a second, which itself came a second after the one
// before (a lone or glitched edge isn't trusted).
static bool findPps(uint32_t rmcAt, uint32_t *slot)
{
  const uint32_t n = g_ppsCount.load(std::memory_order_acquire);
  for (uint32_t i = 1; i < 4 && i < n; ++i) {
    const uint32_t at = g_ppsAt[(n - i) & 3];
    if (ptpmath::rmcFollowsPps(at, rmcAt)) {
      if (!ptpmath::ppsIntervalOk(g_ppsAt[(n - i - 1) & 3], at))
        return false;
      *slot = (n - i) & 3;
      return true;
    }
  }
  return false;
}

// lwIP's raw UDP API: an NTP request, in the tcpip thread. The receive
// time stamp is taken inside respond(), now; the reply goes straight out.
static void ntpReceive(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  (void)arg;
  uint8_t buf[NtpServer::kPacket + 20];   // anything past 48 bytes (extensions, MACs) is ignored
  const uint16_t len = pbuf_copy_partial(p, buf, sizeof(buf), 0);
  pbuf_free(p);
  const ip4_addr_t *a = ip_2_ip4(addr);
  const IpAddress from(ip4_addr1(a), ip4_addr2(a), ip4_addr3(a), ip4_addr4(a));
  const size_t n = ntpServer.respond(from, port, buf, len, sizeof(buf), sys_now());
  if (n == 0)
    return;
  struct pbuf *q = pbuf_alloc(PBUF_TRANSPORT, (u16_t)n, PBUF_RAM);
  if (q == nullptr)
    return;
  memcpy(q->payload, buf, n);
  udp_sendto(pcb, q, addr, port);
  pbuf_free(q);
}

// "IP 192.168.1.73", or how far the network has got
static void formatNet(char *out, size_t len)
{
  uint32_t mbps;
  bool fd;
  if (!g_netUp.load())
    snprintf(out, len, "ETH starting");
  else if (!ethernetif_mac_ok())
    snprintf(out, len, "ETH MAC failed");
  else if (!netif_is_link_up(&g_netif))
    snprintf(out, len, "ETH no link");
  else if (ip4_addr_isany_val(*netif_ip4_addr(&g_netif)))
    snprintf(out, len, "ETH waiting for DHCP");
  else {
    ethernetif_link_mode(&mbps, &fd);
    const ip4_addr_t *a = netif_ip4_addr(&g_netif);
    snprintf(out, len, "IP %u.%u.%u.%u (%luM %s)", ip4_addr1(a), ip4_addr2(a), ip4_addr3(a), ip4_addr4(a),
             (unsigned long)mbps, fd ? "FD" : "HD");
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
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // TIM1 waits for TIM2's enable (trigger mode on ITR1 = TIM2's TRGO), so
  // start it first: both then count from 0 at 1 MHz, TIM1 = TIM2 mod 65536.
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);   // PPS
  HAL_TIM_Base_Start(&htim2);                   // starts both counters
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of gps_data_mutex */
  gps_data_mutexHandle = osMutexNew(&gps_data_mutex_attributes);

  /* creation of console_mutex */
  console_mutexHandle = osMutexNew(&console_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  if (g_lseOk)
    F2Rtc_Begin(&hrtc);
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
  /* creation of clockTask */
  clockTaskHandle = osThreadNew(StartClockTask, NULL, &clockTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* creation of netTask */
  netTaskHandle = osThreadNew(StartNetTask, NULL, &netTask_attributes);

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
  * @brief System Clock Configuration: 8 MHz HSE bypass (the ST-LINK's MCO)
  *        -> PLL /8 x240 /2 = 120 MHz, /5 = 48 MHz. APB1 30 MHz (its timers
  *        60 MHz), APB2 60 MHz (its timers 120 MHz). As ST's NUCLEO-F207ZG
  *        examples. The LSE is started in MX_RTC_Init(), where a missing
  *        crystal costs only the RTC.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function: the LSE, 1 Hz. If the LSE doesn't
  *        start, there is no RTC (the clock still runs, from the MAC).
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  RCC_OscInitTypeDef lse = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  lse.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  lse.LSEState = RCC_LSE_ON;
  lse.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&lse) != HAL_OK)
    return;                                  // no LSE: no RTC
  g_lseOk = true;
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  hrtc.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    g_lseOk = false;
    return;
  }

  /* USER CODE BEGIN RTC_Init 2 */
  // The calendar is left alone: it is only set by F2Rtc_StartSecondAt(),
  // at a second of the clock (HAL_RTC_Init() leaves a running one as it is).
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function: 16-bit, 1 MHz, started by TIM2
  *        (slave in trigger mode on ITR1, which is TIM2's TRGO for TIM1:
  *        RM0033's internal trigger table, general knowledge; the console's
  *        greeting shows whether the two are in step). Channel 1 captures
  *        the PPS's rising edge on PE9.
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;   // a few samples: ignores glitches shorter than ~0.1 us
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function: a free-running 32-bit 1 MHz
  *        counter, the timebase for the PPS and the NMEA. Its enable is its
  *        trigger output (TRGO), which starts TIM1.
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM2_PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function (the ST-LINK virtual COM port): 115200 8N1
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = CONSOLE_BAUD;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function (the Teseo-VIC3DA): 115200 8N1
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = GNSS_BAUD;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);   // not in reset

  /*Configure GPIO pins : LD1_Pin LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GNSS_RST_Pin */
  GPIO_InitStruct.Pin = GNSS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_RST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// TIM1 channel 1: a PPS edge. Widened to TIM2's 32 bits (TIM1 counts in
// step with it), and the system time read with TIM2 right after, so the
// clock's error at the edge is known to a microsecond however late this
// interrupt runs.
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    const uint16_t cap = (uint16_t) HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t sec = 0, frac = 0, readAt = TIM2->CNT;
    const bool ok = PtpClock_ReadWithCounter(&sec, &frac, &TIM2->CNT, &readAt);
    const uint32_t n = g_ppsCount.load(std::memory_order_relaxed);
    const uint32_t k = n & 3;
    g_ppsAt[k] = ptpmath::widen(cap, readAt);
    g_ppsClockSec[k] = sec;
    g_ppsClockFrac[k] = frac;
    g_ppsReadAt[k] = readAt;
    g_ppsClockOk[k] = ok ? 1 : 0;
    g_ppsCount.store(n + 1, std::memory_order_release);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartClockTask */
/**
  * @brief  The clock task: sets the MAC's system time (from the RTC at
  *         start-up, then from GPS), steers it to the PPS, tells the NTP
  *         server where its time came from, keeps the RTC aligned, and
  *         prints a status line once a second.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartClockTask */
void StartClockTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  {
    // TIM1 should count in step with TIM2 (the low 16 bits, give or take one)
    __disable_irq();
    const uint32_t t2 = TIM2->CNT;
    const uint16_t t1 = (uint16_t) TIM1->CNT;
    __enable_irq();
    g_timSync = (int)(int16_t)((uint16_t) t2 - t1);
  }

  // The system time starts with the MAC, in netTask (if the MAC doesn't
  // start, neither does it: then the clock is never set, and says so)
  while (!g_netUp.load())
    osDelay(10);

  ClockSource source = ClockSource::None;
  uint32_t lastSetUnix = 0;          // when the time was last set or confirmed from GPS
  uint32_t lastSetTick = 0;
  bool ntpSynced = false;

  // After a reset: the RTC's whole seconds, caught as one begins
  ClockSource bkSource;
  uint32_t bkUnix;
  if (g_lseOk && F2Rtc_IsSet()) {
    uint32_t sec, at;
    if (F2Rtc_WaitForTick(&sec, &TIM2->CNT, &at) && PtpClock_SetAt(sec, 0, &TIM2->CNT, at)) {
      g_clockSet.store(true, std::memory_order_release);
      if (F2Rtc_LastSet(&bkSource, &bkUnix) && sec - bkUnix < NTP_HOLDOVER_S
          && (bkSource == ClockSource::Gps || bkSource == ClockSource::GpsPps)) {
        ntpServer.setReference(1, NtpServer::refIdAscii(bkSource == ClockSource::GpsPps ? "PPS" : "GPS"),
                               NtpServer::fromUnix(bkUnix, 0), 0,
                               bkSource == ClockSource::GpsPps ? RTC_ROOT_DISPERSION_US : GPS_ROOT_DISPERSION_US);
        ntpSynced = true;
        source = bkSource;
        lastSetUnix = bkUnix;
        lastSetTick = osKernelGetTickCount() - (sec - bkUnix) * 1000u;
      }
      consolePrint("clock: from the RTC, %lu (last set from %s %lu s before)\r\n", (unsigned long) sec,
                   ntpSynced ? (bkSource == ClockSource::GpsPps ? "PPS" : "GPS") : "nothing recent",
                   (unsigned long)(ntpSynced ? sec - bkUnix : 0));
    }
  }

  int32_t savedPpb = 0;
  if (!(g_lseOk && F2Rtc_FrequencyPpb(&savedPpb)))
    savedPpb = 0;
  ptpmath::Discipline disc(savedPpb);
  PtpClock_SetFrequencyPpb(disc.holdoverPpb());

  GpsStatus g = {};
  uint32_t usedPpsSeq = 0;           // the GPS time last paired with a PPS edge, by its timeSeq
  uint32_t usedTimeSeq = 0;          // the GPS time last applied without PPS
  bool havePps = false;
  uint32_t lastPpsTick = 0;
  int64_t lastErrUs = 0;
  uint32_t nmeaDelayMs = 0;          // how late the RMC came after its PPS edge: measured
  bool ppsActive = false;
  uint32_t lastAlignUnix = 0;        // the RTC last aligned
  uint32_t lastSaveUnix = 0;         // the frequency last saved
  uint32_t lastSecond = 0xFFFFFFFFu;
  uint32_t lastPrintTick = 0;

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    g = gpsStatus;
    osMutexRelease(gps_data_mutexHandle);

    const uint32_t tick = osKernelGetTickCount();
    const bool fresh = (tick - g.timeTick) < GPS_TIME_MAX_AGE_MS;

    if (g_helpRequest.exchange(false))
      consolePrint("\r\nNUCLEO-F207ZG GPS clock: X-NUCLEO-GNSS2A1 (Teseo-VIC3DA) + Ethernet, NTP on UDP 123\r\n"
                   "clock: the MAC's IEEE 1588 system time, steered to the PPS on D6\r\n"
                   "TIM1 against TIM2 at start-up: %d count(s)%s; RTC (LSE) %s\r\n"
                   "keys: h = this help\r\n",
                   g_timSync, (g_timSync >= -1 && g_timSync <= 1) ? " (in step)" : "  MISMATCH: PPS times are off",
                   g_lseOk ? "running" : "absent (no LSE)");

    // 0. The PPS edge of the second the latest RMC names, if one was latched:
    //    the clock's error at it steers the clock's rate (or sets it).
    if (g.timeSeq != usedPpsSeq && fresh) {
      usedPpsSeq = g.timeSeq;
      uint32_t k;
      if (findPps(g.timeTimer, &k) && g_ppsClockOk[k]) {
        const uint32_t S = DS1307::ds1307_to_unix(g.utc);   // the second the edge started
        const uint32_t ppsAt = g_ppsAt[k];
        nmeaDelayMs = (g.timeTimer - ppsAt) / 1000;
        int64_t err = ptpmath::Discipline::kSetUs;          // not set yet: set it
        if (g_clockSet.load())
          err = ptpmath::errorUs(g_ppsClockSec[k], g_ppsClockFrac[k], g_ppsReadAt[k], S, ppsAt);
        lastErrUs = err;
        const ptpmath::Discipline::Step st = disc.update(S, err);
        if (st.set && PtpClock_SetAt(S, 0, &TIM2->CNT, ppsAt)) {
          g_clockSet.store(true, std::memory_order_release);
          source = ClockSource::GpsPps;
          lastSetUnix = S;
          lastSetTick = tick;
          lastAlignUnix = 0;                                 // align the RTC soon
        }
        PtpClock_SetFrequencyPpb(st.ppb);
        havePps = true;
        lastPpsTick = tick;
        if (disc.locked()) {
          ntpServer.setReference(1, NtpServer::refIdAscii("PPS"), NtpServer::fromUnix(S, 0), 0, PPS_ROOT_DISPERSION_US);
          ntpSynced = true;
          source = ClockSource::GpsPps;
          lastSetUnix = S;
          lastSetTick = tick;
          // The RTC: started again at the next whole second of the clock, now and then
          if (g_lseOk && S - lastAlignUnix >= RTC_ALIGN_INTERVAL_S) {
            const uint32_t now = TIM2->CNT;
            uint32_t ahead = (now - ppsAt) / 1000000u + 1;
            if (ppsAt + ahead * 1000000u - now < 3000u)
              ++ahead;
            if (F2Rtc_StartSecondAt(S + ahead, &TIM2->CNT, ppsAt + ahead * 1000000u)) {
              F2Rtc_NoteSource(ClockSource::GpsPps, S + ahead);
              lastAlignUnix = S;
            }
          }
          if (g_lseOk && S - lastSaveUnix >= FREQ_SAVE_INTERVAL_S) {
            F2Rtc_SaveFrequencyPpb(disc.frequencyPpb());
            lastSaveUnix = S;
          }
        }
      }
    }
    ppsActive = havePps && (tick - lastPpsTick) < PPS_TIMEOUT_MS;
    if (havePps && !ppsActive) {
      havePps = false;                      // PPS lost: hold the learned rate
      disc.restart();
      PtpClock_SetFrequencyPpb(disc.holdoverPpb());
    }
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, (ppsActive && disc.locked()) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // 1. GPS time from NMEA, without PPS: when it is due, or when the clock came from anywhere else
    const bool fromPps = source == ClockSource::GpsPps && ntpSynced && (tick - lastSetTick) < PPS_HOLDOVER_MS;
    const bool due = !g_clockSet.load() || (tick - lastSetTick) >= NMEA_RESYNC_INTERVAL_MS
                     || source != ClockSource::Gps;
    if (!ppsActive && !fromPps && due && g.timeSeq != usedTimeSeq && fresh) {
      usedTimeSeq = g.timeSeq;
      // g.utc is the second the RMC named, GPS_NMEA_DELAY_MS before TIM2 read g.timeTimer
      const uint32_t S = DS1307::ds1307_to_unix(g.utc);
      const uint32_t frac = (uint32_t)(((uint64_t)GPS_NMEA_DELAY_MS << 32) / 1000);
      if (PtpClock_SetAt(S, frac, &TIM2->CNT, g.timeTimer)) {
        g_clockSet.store(true, std::memory_order_release);
        ntpServer.setReference(1, NtpServer::refIdAscii("GPS"), NtpServer::fromUnix(S, frac), 0,
                               GPS_ROOT_DISPERSION_US);
        ntpSynced = true;
        source = ClockSource::Gps;
        lastSetUnix = S;
        lastSetTick = tick;
        if (g_lseOk)
          F2Rtc_NoteSource(ClockSource::Gps, S);
      }
    }

    uint32_t nowSec = 0, nowFrac = 0;
    const bool clockSet = g_clockSet.load() && PtpClock_Read(&nowSec, &nowFrac);

    // Holdover: too long since GPS time
    if (ntpSynced && clockSet && nowSec - lastSetUnix > NTP_HOLDOVER_S) {
      ntpServer.setUnsynchronized();
      ntpSynced = false;
    }

    // A status line on each new second, or each second of the tick while
    // the clock isn't set
    if (clockSet ? nowSec != lastSecond : (tick - lastPrintTick) >= 1000) {
      lastSecond = nowSec;
      lastPrintTick = tick;
      ClockTime now = {};
      if (clockSet)
        DS1307::ds1307_from_unix(nowSec, &now);
      char net[40], ppm[16];
      formatNet(net, sizeof(net));
      formatPpm(ppm, sizeof(ppm), PtpClock_FrequencyPpb());
      const NtpServer::Stats ns = ntpServer.stats();
      consolePrint("%04u-%02u-%02u %02u:%02u:%02u.%03lu UTC (%s) | GPS %s sats=%u cfg=%s | %s | NTP %s req=%lu rep=%lu"
                   " | PPS %s err=%ldus rate=%sppm nmea=%lums\r\n",
                   now.year, now.month, now.day, now.hour, now.minute, now.second,
                   (unsigned long)(((uint64_t)nowFrac * 1000) >> 32),
                   !clockSet ? "not set" : source == ClockSource::GpsPps ? "PPS" : source == ClockSource::Gps ? "GPS"
                     : "RTC",
                   !g.alive ? "no data" : (g.fix ? "fix" : "no fix"), g.satellites, configName(g.config), net,
                   ntpSynced ? "synced" : "unsynced", (unsigned long)ns.requests, (unsigned long)ns.replies,
                   !ppsActive ? "none" : (disc.locked() ? "locked" : "locking"), (long)lastErrUs, ppm,
                   (unsigned long)nmeaDelayMs);
    }

    osDelay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
  * @brief  Resets the shield's Teseo, then runs its driver and publishes the
  *         fix and UTC time.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_RESET);
  osDelay(GNSS_RESET_LOW_MS);
  HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);
  osDelay(GNSS_RESET_WAIT_MS);

  teseo_param_t param(true);                // configure it: GGA, GSA, RMC, GSV

  // Built here so reception starts after MX_USART6_UART_Init(); main()
  // must be called from this one thread (it is the one wake() wakes).
  static GpsReceiver gps(param, &huart6);
  static GnssData d;                        // ~400 bytes: kept off the stack

  bool haveData = false;
  uint32_t timeSeq = 0;
  bool versionShown = false;
  teseo_config_status_t lastConfig = teseo_config_status_t::NotDone;

  /* Infinite loop */
  for(;;)
  {
    // Sleeps inside until a line arrives (woken from the UART interrupt) or 100 ms
    gps.main(osKernelGetTickCount());

    const bool fresh = gps.newData();
    if (fresh)
      haveData = gps.getData(&d);
    if (gps.state() == teseo_error_state)
      haveData = false;                     // silent for 3 s: nothing is valid

    if (!versionShown && gps.version()[0] != 0) {
      versionShown = true;
      consolePrint("GPS: %s\r\n", gps.version());
    }
    if (gps.configStatus() != lastConfig) {
      lastConfig = gps.configStatus();
      consolePrint("GPS: configuration %s (step %u)\r\n", configName(lastConfig), (unsigned)gps.configStep());
    }

    ClockTime utc = {};
    utc.year = d.year; utc.month = d.month; utc.day = d.day;
    utc.hour = d.hour; utc.minute = d.minute; utc.second = d.second;
    const bool timeGood = fresh && haveData && d.positionValid && d.dateValid && d.timeValid
                          && DS1307::ds1307_valid(utc);

    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    gpsStatus.alive = haveData;
    gpsStatus.fix = haveData && d.positionValid;
    gpsStatus.satellites = haveData ? d.satellitesUsed : 0;
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

/* USER CODE BEGIN Header_StartNetTask */
/**
  * @brief  Starts lwIP and the Ethernet interface (and with the MAC, the
  *         system time), the NTP server on UDP port 123 and DHCP; then
  *         follows the link every 250 ms.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartNetTask */
void StartNetTask(void *argument)
{
  /* USER CODE BEGIN StartNetTask */
  tcpip_init(NULL, NULL);

  // A locally administered MAC address from the MCU's unique ID (02:80:E1 =
  // ST's OUI with the local bit set)
  const uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
  const uint8_t mac[6] = { 0x02, 0x80, 0xE1, (uint8_t)(uid >> 16), (uint8_t)(uid >> 8), (uint8_t)uid };
  ethernetif_set_mac(mac);

  ip4_addr_t ip, mask, gw;
#if ETH_DHCP
  ip4_addr_set_zero(&ip);
  ip4_addr_set_zero(&mask);
  ip4_addr_set_zero(&gw);
#else
  IP4_ADDR(&ip, ETH_STATIC_IP);
  IP4_ADDR(&mask, ETH_STATIC_MASK);
  IP4_ADDR(&gw, ETH_STATIC_GW);
#endif

  LOCK_TCPIP_CORE();
  netif_add(&g_netif, &ip, &mask, &gw, NULL, ethernetif_init, tcpip_input);   // HAL_ETH_Init() in here
  netif_set_default(&g_netif);
  netif_set_up(&g_netif);
  struct udp_pcb *ntp = udp_new();
  if (ntp != nullptr && udp_bind(ntp, IP_ANY_TYPE, NtpServer::kPort) == ERR_OK)
    udp_recv(ntp, ntpReceive, NULL);
#if ETH_DHCP
  dhcp_start(&g_netif);
#endif
  UNLOCK_TCPIP_CORE();

  // The system time: after HAL_ETH_Init(), which resets it
  if (!ethernetif_mac_ok()) {
    consolePrint("ETH: the MAC didn't start (no 50 MHz reference clock from the PHY?)\r\n");
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  }
  if (!PtpClock_Begin(HAL_RCC_GetHCLKFreq(), 0))
    consolePrint("ETH: the IEEE 1588 system time didn't start\r\n");
  g_netUp.store(true);

  uint32_t lastIp = 0;
  bool lastLink = false;

  /* Infinite loop */
  for(;;)
  {
    ethernetif_link_poll(&g_netif);
    const bool link = netif_is_link_up(&g_netif);
    const uint32_t addr = ip4_addr_get_u32(netif_ip4_addr(&g_netif));
    if (link != lastLink || addr != lastIp) {
      char net[40];
      formatNet(net, sizeof(net));
      consolePrint("ETH: %s\r\n", net);
      lastLink = link;
      lastIp = addr;
    }
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, (link && addr != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    osDelay(250);
  }
  /* USER CODE END StartNetTask */
}

/* USER CODE BEGIN Header_StartConsoleTask */
/**
  * @brief  Hands the console ring to USART3 (the ST-LINK's virtual COM
  *         port), a piece at a time, and listens for keys.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartConsoleTask */
void StartConsoleTask(void *argument)
{
  /* USER CODE BEGIN StartConsoleTask */
  static Stm32HalUartTransport port(&huart3);
  static uint8_t chunk[128];                // sent from here by interrupt: static
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

    if (len == 0) {
      osDelay(5);
      continue;
    }
    while (!port.write(chunk, len))         // the last piece still going out
      osDelay(1);
    // write() only started it: wait until it is out before chunk is reused
    while (huart3.gState != HAL_UART_STATE_READY)
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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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
