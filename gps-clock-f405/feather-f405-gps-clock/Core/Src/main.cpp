/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : GPS clock and NTP server on an Adafruit Feather STM32F405
  *
  *  The Feather stack:
  *    Adafruit Feather STM32F405 Express (168 MHz, 12 MHz HSE, 32.768 kHz LSE)
  *    Ultimate GPS FeatherWing (MTK3333) on USART3, its 1PPS on TIM2_CH1
  *    Ethernet FeatherWing (W5500) on SPI2, with DMA
  *    DS3231 Precision RTC FeatherWing on I2C1, its SQW on TIM3_CH1
  *    128x32 OLED FeatherWing (SSD1306) on I2C1, buttons A/B/C
  *    USB (CDC virtual COM port): the console
  *
  *  The STM32 RTC is the clock: set from GPS time, disciplined to the PPS
  *  (phase by sub-second shifts, frequency by smooth calibration;
  *  Core/Inc/pps_math.h), served by an NTP server on the W5500's UDP
  *  responder. Network time (SNTP) when there is no fix.
  *
  *  The DS3231 is, for now, only measured: TIM3 runs in step with TIM2, so
  *  its SQW edges and the PPS are on one 1 MHz timebase, and the probe
  *  (button B, or 'p' on the console) reads its seconds every 20 ms for
  *  12 s to find which SQW edge its seconds tick on (Core/Inc/rtc_edge.h),
  *  while a Saleae records the same thing on the wires (Claude-iTransport's
  *  tools/saleae/saleae_log.py --rtc-edge). A DS3231 driver follows.
  *
  *  The drivers are the non-blocking ones from Claude-iTransport (see
  *  iTransport/SOURCE.txt):
  *    xssd1306<Stm32HalI2CTransport, 32> iDisplay/ssd1306
  *    xmtk3339<Stm32HalUartTransport>    isensor/mtk3339 (the MT3333 speaks
  *                                       the same NMEA/PMTK as the MT3339)
  *    w5500<Stm32HalSpiBlockTransport>   iNetTransport/w5500, behind xEthernet
  *
  *  Wiring: the FeatherWings stack on the Feather's headers; four pads need
  *  wires (none of them reaches a Feather pin on its own; Adafruit's
  *  schematics):
  *    GPS FeatherWing  1PPS   -> A1  (PA5, TIM2_CH1)
  *    DS3231 FeatherWing SQW  -> A2  (PA6, TIM3_CH1; 10k pull-up to 3.3 V)
  *    Ethernet FeatherWing IRQ   -> D11 (PC3, EXTI3)
  *    Ethernet FeatherWing RESET -> D12 (PC2)
  *  Already connected by the headers: GPS TX/RX (PB11/PB10, USART3), I2C
  *  (PB6/PB7), SPI (PB13/14/15, SPI2), W5500 CS on D10 (PB9, SJ4), OLED
  *  buttons A D9 (PB8), B D6 (PC6), C D5 (PC7).
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <atomic>
#include "usbd_cdc_if.h"
#include "Stm32HalI2CTransport.h"
#include "Stm32HalUartTransport.h"
#include "Stm32HalSpiBlockTransport.h"
#include "ds1307.h"          // only for its date helpers (ds1307_to_unix, ...)
#include "xssd1306.h"
#include "xmtk3339.h"
#include "W5500.h"
#include "xEthernet.h"
#include "NtpServer.h"
#include "Font5x7.h"
#include "stm32_rtc.h"
#include "pps_math.h"
#include "rtc_edge.h"
#include "rtc_probe.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef SSD1306::xssd1306<Stm32HalI2CTransport, 32> OledDisplay;
typedef xmtk3339<Stm32HalUartTransport> GpsReceiver;
typedef DS1307::ds1307_time_t ClockTime;
typedef W5500::w5500<Stm32HalSpiBlockTransport> EthChip;
typedef xrtc_probe<Stm32HalI2CTransport> RtcProbe;

// What clockTask needs from the GPS, copied out by gpsTask
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

// What clockTask needs from rtcTask (the DS3231 and its SQW)
typedef struct {
  bool      present;       // answered, control register written
  bool      timeValid;
  ClockTime time;          // its last reading
  bool      sqw;           // edges seen in the last 2.5 s
  bool      sqwVsPps;      // sqwOffsetUs is fresh
  int32_t   sqwOffsetUs;   // last falling edge after the PPS edge before it
  bool      probing;
  uint8_t   probeSecondsLeft;
  rtcedge::Polarity verdict;     // of the last probe
  int32_t   verdictLoUs, verdictHiUs;
} RtcStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The clock is kept in UTC. Set this to show local (standard) time.
#define UTC_OFFSET_MINUTES      0

// Re-sync the RTC from GPS (NMEA) this often once it has been set
#define RTC_RESYNC_INTERVAL_MS  (60UL * 60UL * 1000UL)
// A GPS time older than this is not used
#define GPS_TIME_MAX_AGE_MS     500UL

// The STM32 RTC: LSE 32768 Hz / (PREDIV_A + 1) / (PREDIV_S + 1) = 1 Hz,
// with PREDIV_S + 1 = 8192 sub-second steps (122 us, NTP precision -13).
// PREDIV_A 3 is the smallest that keeps smooth calibration's CALP usable
// (CALP must stay 0 when PREDIV_A < 3: the RTC chapter's smooth calibration
// section as quoted on ST's community forum, not read in the manual here).
#define RTC_ASYNCH_PREDIV       3
#define RTC_SYNCH_PREDIV        8191
#define NTP_PRECISION_LOG2      (-13)

// How late an NMEA RMC sentence is relative to the second it names (used
// without PPS). Measure it with PPS ("nmea=" on the console) and set it.
#define GPS_NMEA_DELAY_MS       0
#define GPS_ROOT_DISPERSION_US  100000UL   // NMEA timing, without PPS
#define NET_ROOT_DISPERSION_US  50000UL
#define NET_STRATUM             3          // assumes an upstream stratum 2 server
#define NTP_HOLDOVER_S          (24UL * 3600UL)

// PPS: TIM2 counts at 1 MHz (84 MHz APB1 timer clock / 84) and latches the
// PPS edge on PA5. TIM3, started by TIM2, counts in step and latches SQW.
#define PPS_TIMER_PRESCALER     83
#define PPS_ROOT_DISPERSION_US  1000UL
#define PPS_TIMEOUT_MS          3000UL

// The DS3231 (I2C 0x68) and the probe
#define RTC_I2C_ADDR            0x68
#define RTC_CONTROL_REG         0x0E       // DS3231 control: 0x00 = oscillator on, 1 Hz on SQW/INT
#define RTC_CONTROL_VALUE       0x00
#define RTC_PROBE_PERIOD_MS     20
#define RTC_PROBE_DURATION_MS   12000UL
#define SQW_TIMEOUT_MS          2500UL

// Ethernet address: DHCP, or this static one
#define ETH_DHCP                1
#define ETH_STATIC_IP           192, 168, 1, 50
#define ETH_STATIC_MASK         255, 255, 255, 0
#define ETH_STATIC_GW           192, 168, 1, 1

// The console ring (USB CDC); lines are dropped when it is full
#define CONSOLE_RING            8192
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

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
/* Definitions for ethTask */
osThreadId_t ethTaskHandle;
const osThreadAttr_t ethTask_attributes = {
  .name = "ethTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rtcTask */
osThreadId_t rtcTaskHandle;
const osThreadAttr_t rtcTask_attributes = {
  .name = "rtcTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for consoleTask */
osThreadId_t consoleTaskHandle;
const osThreadAttr_t consoleTask_attributes = {
  .name = "consoleTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
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
/* Definitions for spi2_mutex */
osMutexId_t spi2_mutexHandle;
const osMutexAttr_t spi2_mutex_attributes = {
  .name = "spi2_mutex"
};
/* Definitions for rtc_mutex */
osMutexId_t rtc_mutexHandle;
const osMutexAttr_t rtc_mutex_attributes = {
  .name = "rtc_mutex"
};
/* Definitions for console_mutex */
osMutexId_t console_mutexHandle;
const osMutexAttr_t console_mutex_attributes = {
  .name = "console_mutex"
};
/* USER CODE BEGIN PV */
// Written by gpsTask, read by clockTask, guarded by gps_data_mutex
static GpsStatus gpsStatus;
// Written by rtcTask, read by clockTask, guarded by gps_data_mutex too
static RtcStatus rtcStatus;

// Set by ethTask once the interface has begun; read by clockTask and
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
static NtpServer ntpServer(ntpClock, NTP_PRECISION_LOG2, 20);   // LSE: 20 ppm

// PPS edges, as TIM2 latched them: the last four, written by the capture
// interrupt (a second apart, so no slot is reused while it is read).
static volatile uint32_t g_ppsRing[4];
static std::atomic<uint32_t> g_ppsCount{0};

// SQW edges, as TIM3 latched them (widened to TIM2's 32 bits), with their
// direction: the last eight.
static volatile uint32_t g_sqwRing[8];
static volatile uint8_t g_sqwRising[8];
static std::atomic<uint32_t> g_sqwCount{0};

// Requests from the buttons and the console
static std::atomic<bool> g_probeRequest{false};
static std::atomic<bool> g_helpRequest{false};
static int g_timSync = 0;                       // TIM2 - TIM3 (low 16 bits) at start-up

// The console: a ring the tasks write (under console_mutex) and
// consoleTask hands to USB
static char g_console[CONSOLE_RING];
static uint32_t g_conHead = 0, g_conTail = 0;     // head written by writers, tail by consoleTask
static uint32_t g_conDropped = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
void StartClockTask(void *argument);
void StartGpsTask(void *argument);
void StartEthTask(void *argument);
void StartRtcTask(void *argument);
void StartConsoleTask(void *argument);

/* USER CODE BEGIN PFP */
static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static bool findPps(uint32_t rmcAt, uint32_t *ppsAt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *const dayNames[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

// The HAL UART, I2C and SPI interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32UartItCallbacks.cpp, ...): don't define them here.

// printf to the console. Tasks only (not interrupts); a line that doesn't
// fit is dropped and counted.
static void consolePrint(const char *fmt, ...)
{
  char line[160];
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

// What the host types: 'p' starts the probe, 'h' prints the help
extern "C" void CDC_RxFromIsr(const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i) {
    if (buf[i] == 'p' || buf[i] == 'P')
      g_probeRequest.store(true);
    else if (buf[i] == 'h' || buf[i] == 'H' || buf[i] == '?')
      g_helpRequest.store(true);
  }
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

// "37.77490N" from 1e-7 degrees, without printf float support
static void formatDegrees(char *out, size_t len, int32_t e7, char pos, char neg)
{
  const long a = labs(e7);
  snprintf(out, len, "%ld.%05ld%c", a / 10000000L, (a % 10000000L) / 100, e7 < 0 ? neg : pos);
}

// "+12.3" from ppb, without printf float support
static void formatPpm(char *out, size_t len, int32_t ppb)
{
  const long a = labs((long)ppb);
  snprintf(out, len, "%c%ld.%ld", ppb < 0 ? '-' : '+', a / 1000, (a % 1000) / 100);
}

// "-1.26" from microseconds, as ms with two decimals
static void formatMs(char *out, size_t len, int32_t us)
{
  const long a = labs((long)us);
  snprintf(out, len, "%c%ld.%02ld", us < 0 ? '-' : '+', a / 1000, (a % 1000) / 10);
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
    snprintf(out, len, "ETH starting");
  else if (!eth->ready())
    snprintf(out, len, "ETH no W5500");
  else if (!eth->linkUp())
    snprintf(out, len, "ETH no link");
  else if (!eth->hasAddress())
    snprintf(out, len, "ETH waiting for DHCP");
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

static const char *verdictName(rtcedge::Polarity p)
{
  switch (p) {
  case rtcedge::Polarity::Falling: return "falling";
  case rtcedge::Polarity::Rising:  return "rising";
  default:                         return "none";
  }
}

// The pages of the 128x32 OLED (button A: next, C: previous)
enum Page : uint8_t { PageClock, PageGps, PageNet, PageRtc, PageCount };

struct Screen {
  bool clockSet;
  ClockTime utc;
  RtcSource source;
  ClockTime lastSet;
  bool ppsActive, ppsLocked;
  int32_t calPpb;
  int64_t ppsErrUs;
  uint32_t nmeaDelayMs;
  bool ntpSynced;
  const char *ethLine;
};

static void drawScreen(idisplay::MonoCanvas &c, Page page, const Screen &s, const GpsStatus &g, const RtcStatus &r)
{
  char line[48], a[16], b[16];
  c.clear();
  switch (page) {
  case PageClock:
    if (s.clockSet) {
      ClockTime local;
      DS1307::ds1307_from_unix(DS1307::ds1307_to_unix(s.utc) + (int32_t) UTC_OFFSET_MINUTES * 60, &local);
      snprintf(line, sizeof(line), "%02u:%02u:%02u", local.hour, local.minute, local.second);
      drawCentered(c, 0, line, 2);
      snprintf(line, sizeof(line), "%s %04u-%02u-%02u%s", dayNames[local.weekday], local.year, local.month,
               local.day, UTC_OFFSET_MINUTES == 0 ? "Z" : "");
      drawCentered(c, 16, line);
    } else {
      drawCentered(c, 0, "--:--:--", 2);
      drawCentered(c, 16, "waiting for time");
    }
    if (s.ppsLocked) {
      formatPpm(a, sizeof(a), s.calPpb);
      snprintf(line, sizeof(line), "PPS lock %sppm", a);
    } else if (s.source == RtcSource::GpsPps || s.source == RtcSource::Gps || s.source == RtcSource::Network) {
      snprintf(line, sizeof(line), "synced %02u:%02uZ %s", s.lastSet.hour, s.lastSet.minute,
               s.source == RtcSource::GpsPps ? "PPS" : s.source == RtcSource::Gps ? "GPS" : "NTP");
    } else {
      snprintf(line, sizeof(line), "RTC not set");
    }
    c.drawText(0, 24, line, false);
    break;

  case PageGps:
    if (!g.alive)
      snprintf(line, sizeof(line), "GPS no data");
    else
      snprintf(line, sizeof(line), "GPS %s %u sats", g.fix ? "fix" : "no fix", g.satellites);
    c.drawText(0, 0, line, false);
    if (g.alive && g.fix) {
      formatDegrees(a, sizeof(a), g.latitudeE7, 'N', 'S');
      formatDegrees(b, sizeof(b), g.longitudeE7, 'E', 'W');
      c.drawText(0, 8, a, false);
      c.drawText(0, 16, b, false);
    } else {
      snprintf(line, sizeof(line), "cfg %s", configName(g.config));
      c.drawText(0, 8, line, false);
    }
    if (s.ppsActive)
      snprintf(line, sizeof(line), "PPS %s %ldus", s.ppsLocked ? "lock" : "seen", (long) s.ppsErrUs);
    else
      snprintf(line, sizeof(line), "PPS none");
    c.drawText(0, 24, line, false);
    break;

  case PageNet: {
    c.drawText(0, 0, s.ethLine, false);
    const NtpServer::Stats ns = ntpServer.stats();
    snprintf(line, sizeof(line), "NTP %s", s.ntpSynced ? "synced" : "unsynced");
    c.drawText(0, 8, line, false);
    snprintf(line, sizeof(line), "req %lu rep %lu", (unsigned long) ns.requests, (unsigned long) ns.replies);
    c.drawText(0, 16, line, false);
    snprintf(line, sizeof(line), "nmea %lums", (unsigned long) s.nmeaDelayMs);
    c.drawText(0, 24, line, false);
    break;
  }

  case PageRtc:
    if (!r.present)
      snprintf(line, sizeof(line), "DS3231 absent");
    else if (r.timeValid)
      snprintf(line, sizeof(line), "DS3231 %02u:%02u:%02u", r.time.hour, r.time.minute, r.time.second);
    else
      snprintf(line, sizeof(line), "DS3231 no time");
    c.drawText(0, 0, line, false);
    if (!r.sqw) {
      snprintf(line, sizeof(line), "SQW none");
    } else if (r.sqwVsPps) {
      formatMs(a, sizeof(a), r.sqwOffsetUs);
      snprintf(line, sizeof(line), "SQW %sms PPS", a);
    } else {
      snprintf(line, sizeof(line), "SQW seen, no PPS");
    }
    c.drawText(0, 8, line, false);
    if (r.probing) {
      snprintf(line, sizeof(line), "probing %us", r.probeSecondsLeft);
    } else if (r.verdict != rtcedge::Polarity::Unknown) {
      formatMs(a, sizeof(a), r.verdictLoUs);
      formatMs(b, sizeof(b), r.verdictHiUs);
      snprintf(line, sizeof(line), "%s %s..%s", r.verdict == rtcedge::Polarity::Falling ? "fall" : "rise", a, b);
    } else {
      snprintf(line, sizeof(line), "B: probe the edge");
    }
    c.drawText(0, 16, line, false);
    c.drawText(0, 24, "A/C page  B probe", false);
    break;

  default:
    break;
  }
}

// The OLED FeatherWing's buttons (to ground, pulled up): a press is two
// low samples in a row after a high one, polled every 10-100 ms.
struct Button {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t history;
  bool pressed() {
    const bool low = HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
    history = (uint8_t)((history << 1) | (low ? 1 : 0));
    return (history & 0x07) == 0x03;   // high, low, low
  }
};
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
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // TIM3 waits for TIM2's enable (trigger mode on ITR1 = TIM2's TRGO), so
  // start it first: both then count from 0 at 1 MHz, TIM3 = TIM2 mod 65536.
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);   // SQW, both edges
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   // PPS; starts both counters
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c1_mutex */
  i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes);

  /* creation of gps_data_mutex */
  gps_data_mutexHandle = osMutexNew(&gps_data_mutex_attributes);

  /* creation of spi2_mutex */
  spi2_mutexHandle = osMutexNew(&spi2_mutex_attributes);

  /* creation of rtc_mutex */
  rtc_mutexHandle = osMutexNew(&rtc_mutex_attributes);

  /* creation of console_mutex */
  console_mutexHandle = osMutexNew(&console_mutex_attributes);

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
  /* creation of clockTask */
  clockTaskHandle = osThreadNew(StartClockTask, NULL, &clockTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* creation of ethTask */
  ethTaskHandle = osThreadNew(StartEthTask, NULL, &ethTask_attributes);

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
  * @brief System Clock Configuration: 12 MHz HSE (the Feather's Y1) ->
  *        PLL /6 x168 /2 = 168 MHz, /7 = 48 MHz for USB. APB1 42 MHz (its
  *        timers 84 MHz), APB2 84 MHz. LSE (X2) on for the RTC.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Backup domain access, for the LSE and the RTC
  */
  HAL_PWR_EnableBkUpAccess();

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function: 400 kHz (the DS3231 and the
  *        SSD1306 are both fast-mode parts)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  // The calendar survives a reset (the RTC runs on while VDD or VBAT has
  // power): keep it. Year 00 leaves INITS clear, so the default below
  // reads as "not set" (StmRtc_IsSet()) until GPS or network time.
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
  * @brief SPI2 Initialization Function (W5500): 42 MHz / 4 = 10.5 MHz, mode 0
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function: a free-running 32-bit 1 MHz counter,
  *        channel 1 capturing the GPS's PPS rising edge on PA5. Its enable
  *        is its trigger output (TRGO), which starts TIM3.
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function: 16-bit, the same 1 MHz, started by
  *        TIM2 (slave in trigger mode on ITR1, which is TIM2's TRGO for TIM3:
  *        RM0090's internal trigger table, general knowledge; checked at
  *        start-up on the console). Channel 1 captures both edges of the
  *        DS3231's SQW on PA6.
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PPS_TIMER_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function (GPS): 9600 8N1
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
  huart3.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin W5500_RST_Pin */
  GPIO_InitStruct.Pin = LED_Pin|W5500_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_INT_Pin */
  GPIO_InitStruct.Pin = W5500_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(W5500_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_B_Pin BTN_C_Pin */
  GPIO_InitStruct.Pin = BTN_B_Pin|BTN_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_A_Pin */
  GPIO_InitStruct.Pin = BTN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_A_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// TIM2 channel 1: a PPS edge. TIM3 channel 1: an SQW edge, widened to
// TIM2's 32 bits (TIM3 counts in step with it), its direction read from
// the pin (it stays put for half a second).
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    const uint32_t n = g_ppsCount.load(std::memory_order_relaxed);
    g_ppsRing[n & 3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    g_ppsCount.store(n + 1, std::memory_order_release);
  }
  else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    const uint16_t cap = (uint16_t) HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    const uint32_t now = TIM2->CNT;
    const bool high = HAL_GPIO_ReadPin(RTC_SQW_GPIO_Port, RTC_SQW_Pin) == GPIO_PIN_SET;
    const uint32_t n = g_sqwCount.load(std::memory_order_relaxed);
    g_sqwRing[n & 7] = rtcedge::widen(cap, now);
    g_sqwRising[n & 7] = high ? 1 : 0;
    g_sqwCount.store(n + 1, std::memory_order_release);
  }
}

// The W5500's INTn (PC3): wakes the network driver thread at once
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  xEthernet *eth = g_eth.load();
  if (GPIO_Pin == W5500_INT_Pin && eth != nullptr)
    eth->interruptFromIsr();
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartClockTask */
/**
  * @brief  The clock task: sets and disciplines the STM32 RTC (PPS, else
  *         NMEA, else network time), tells the NTP server where its time
  *         came from, runs the OLED and its buttons, and prints a status
  *         line once a second.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartClockTask */
void StartClockTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Built here, not as a global: the transport needs i2c1_mutex, which only
  // exists once main() has created it. Static: the frame buffer stays off
  // this task's stack.
  static OledDisplay oled(SSD1306::ssd1306_default_param(32), &hi2c1, SSD1306_I2C_ADDR, i2c1_mutexHandle);

  {
    // TIM3 should count in step with TIM2 (the low 16 bits, give or take one)
    __disable_irq();
    const uint32_t t2 = TIM2->CNT;
    const uint16_t t3 = (uint16_t) TIM3->CNT;
    __enable_irq();
    g_timSync = (int)(int16_t)((uint16_t) t2 - t3);
  }

  int lastSecond = -1;
  bool lastClockSet = false;
  bool redrawPending = true;
  Page page = PageClock;
  Button btnA = { BTN_A_GPIO_Port, BTN_A_Pin, 0 };
  Button btnB = { BTN_B_GPIO_Port, BTN_B_Pin, 0 };
  Button btnC = { BTN_C_GPIO_Port, BTN_C_Pin, 0 };
  GpsStatus g = {};
  RtcStatus r = {};

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
    const uint32_t ageMs = fresh ? (nowSec - lastSetUnix) * 1000u : RTC_RESYNC_INTERVAL_MS;
    lastSetTick = osKernelGetTickCount() - (ageMs < RTC_RESYNC_INTERVAL_MS ? ageMs : RTC_RESYNC_INTERVAL_MS);
  }

  uint32_t usedTimeSeq = 0;      // the GPS time last applied, by its timeSeq

  // PPS discipline
  ppsmath::Discipline disc(StmRtc_CalibrationPpb());   // RTC_CALR survives a reset
  uint32_t usedPpsSeq = 0;
  bool havePps = false;
  uint32_t lastPpsTick = 0;
  int64_t lastPpsErrUs = 0;
  uint32_t nmeaDelayMs = 0;      // how late the RMC came after its PPS edge: measured
  bool ppsActive = false;
  char ethLine[24] = "";
  char lastEthLine[24] = "";

  /* Infinite loop */
  for(;;)
  {
    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    g = gpsStatus;
    r = rtcStatus;
    osMutexRelease(gps_data_mutexHandle);

    const uint32_t tick = osKernelGetTickCount();
    xEthernet *eth = g_eth.load();
    const bool due = source == RtcSource::None || (tick - lastSetTick) >= RTC_RESYNC_INTERVAL_MS;
    const bool fromGps = source == RtcSource::Gps || source == RtcSource::GpsPps;

    // Buttons: A next page, C previous, B the probe
    if (btnA.pressed()) { page = (Page)((page + 1) % PageCount); redrawPending = true; }
    if (btnC.pressed()) { page = (Page)((page + PageCount - 1) % PageCount); redrawPending = true; }
    if (btnB.pressed()) { g_probeRequest.store(true); page = PageRtc; redrawPending = true; }
    if (g_helpRequest.exchange(false))
      consolePrint("\r\nFeather STM32F405 GPS clock: GPS + DS3231 + W5500 + OLED\r\n"
                   "TIM3 against TIM2 at start-up: %d count(s)%s\r\n"
                   "keys: p = probe the DS3231's SQW edge (12 s), h = this help\r\n"
                   "buttons: A/C = page, B = probe\r\n",
                   g_timSync, (g_timSync >= -1 && g_timSync <= 1) ? " (in step)" : "  MISMATCH: SQW times are off");

    // 0. The PPS edge of the second the latest RMC names, if one was latched:
    //    measure the RTC against it, and correct phase and frequency.
    if (g.timeSeq != usedPpsSeq && (tick - g.timeTick) < GPS_TIME_MAX_AGE_MS) {
      usedPpsSeq = g.timeSeq;
      uint32_t ppsAt;
      if (findPps(g.timeTimer, &ppsAt)) {
        const uint32_t S = DS1307::ds1307_to_unix(g.utc);   // the second the edge started
        nmeaDelayMs = (g.timeTimer - ppsAt) / 1000;
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
          }
        }
      }
    }
    ppsActive = havePps && (tick - lastPpsTick) < PPS_TIMEOUT_MS;
    if (havePps && !ppsActive) {
      havePps = false;                      // PPS lost: NMEA timing from here, the loop starts over
      disc.restart();
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (ppsActive && disc.locked()) ? GPIO_PIN_SET : GPIO_PIN_RESET);

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
      char calText[12], sqwText[12];
      formatPpm(calText, sizeof(calText), disc.calPpb());
      formatMs(sqwText, sizeof(sqwText), r.sqwOffsetUs);
      if (!r.probing)   // the probe has the console to itself while it runs
        consolePrint("RTC %04u-%02u-%02u %02u:%02u:%02u UTC (%s) | GPS %s sats=%u cfg=%s | %s | NTP %s req=%lu rep=%lu"
                     " | PPS %s err=%ldus cal=%sppm nmea=%lums | DS3231 %s SQW %s%s%s%s\r\n",
                     now.year, now.month, now.day, now.hour, now.minute, now.second,
                     source == RtcSource::GpsPps ? "PPS" : source == RtcSource::Gps ? "GPS"
                       : source == RtcSource::Network ? "network" : "not set",
                     !g.alive ? "no data" : (g.fix ? "fix" : "no fix"), g.satellites, configName(g.config), ethLine,
                     ntpSynced ? "synced" : "unsynced", (unsigned long)ns.requests, (unsigned long)ns.replies,
                     !ppsActive ? "none" : (disc.locked() ? "locked" : "locking"), (long)lastPpsErrUs, calText,
                     (unsigned long)nmeaDelayMs,
                     r.present ? "ok" : "absent", r.sqw ? "seen" : "none",
                     r.sqwVsPps ? " at PPS" : "", r.sqwVsPps ? sqwText : "", r.sqwVsPps ? "ms" : "");
    }
    if (r.probing)
      redrawPending = true;   // the countdown

    // Draw only between flushes (iDisplayDevice rule). The driver sends
    // just the pages that changed.
    if (redrawPending && oled.idle()) {
      redrawPending = false;
      Screen s;
      s.clockSet = clockSet;
      s.utc = now;
      s.source = source;
      DS1307::ds1307_from_unix(lastSetUnix, &s.lastSet);
      s.ppsActive = ppsActive;
      s.ppsLocked = ppsActive && disc.locked();
      s.calPpb = disc.calPpb();
      s.ppsErrUs = lastPpsErrUs;
      s.nmeaDelayMs = nmeaDelayMs;
      s.ntpSynced = ntpSynced;
      s.ethLine = ethLine;
      drawScreen(oled, page, s, g, r);
      oled.requestFlush();
    }

    // Run the display's state machine until the flush is out (it sleeps
    // in osThreadFlagsWait while each transfer is in flight), or wait a
    // little for the buttons and the next second.
    if (!oled.idle() || oled.failed()) {
      do {
        oled.main(osKernelGetTickCount());
      } while (!oled.idle() && !oled.failed());
    } else {
      osDelay(20);
    }
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

  // Built here so reception starts after MX_USART3_UART_Init(); main()
  // must be called from this one thread (it is the one wake() wakes).
  static GpsReceiver gps(param, &huart3);
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
  *         DHCP, DNS and SNTP run inside the driver, on the W5500's socket 7;
  *         the NTP server answers on socket 6.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEthTask */
void StartEthTask(void *argument)
{
  /* USER CODE BEGIN StartEthTask */
  const uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

  W5500::w5500_param_t param;
  for (int i = 0; i < W5500::w5500_sockets; ++i)
    param.rxBufKb[i] = param.txBufKb[i] = (i < 2 || i >= 6) ? 2 : 0;
  param.responder = &ntpServer;  // answers in this thread, from the STM32 RTC
  param.responderPort = NtpServer::kPort;
  param.pollMs = 100;            // INTn is wired: polling is only a backstop
  param.seed = uid;

  xNetInterface::Config cfg;
  cfg.maxSockets = 2;

  // Built here, after spi2_mutex exists. Static: the driver keeps a 1 KB buffer.
  static EthChip chip(param, &hspi2, W5500_CS_GPIO_Port, W5500_CS_Pin, spi2_mutexHandle);
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
    consolePrint("ETH: begin() failed: FreeRTOS heap too small\r\n");
    for(;;)
      osDelay(osWaitForever);
  }
  g_eth.store(&eth);             // the EXTI callback and clockTask may use it from here

  /* Infinite loop */
  eth.run();                     // never returns
  /* USER CODE END StartEthTask */
}

/* USER CODE BEGIN Header_StartRtcTask */
/**
  * @brief  The DS3231: turns its 1 Hz SQW on, reads its time once a second,
  *         puts its SQW edges against the PPS, and runs the probe: reads
  *         every 20 ms for 12 s, every read and edge printed on the console
  *         ("probe" lines), then where the seconds tick against SQW.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRtcTask */
void StartRtcTask(void *argument)
{
  /* USER CODE BEGIN StartRtcTask */
  rtc_probe_param_t param;
  param.controlReg = RTC_CONTROL_REG;
  param.controlValue = RTC_CONTROL_VALUE;
  param.periodMs = 1000;
  param.probePeriodMs = RTC_PROBE_PERIOD_MS;
  param.counter = &TIM2->CNT;
  static RtcProbe probe(param, &hi2c1, RTC_I2C_ADDR, i2c1_mutexHandle);

  rtcedge::Analyzer an;
  uint32_t usedSqw = 0;          // edges taken from the ring
  uint32_t lastSqwTick = 0;
  bool sqwEver = false;
  bool wasProbing = false;
  uint32_t probeStartTick = 0;
  rtcedge::Polarity verdict = rtcedge::Polarity::Unknown;
  int32_t vLo = 0, vHi = 0;
  int32_t sqwOffsetUs = 0;
  uint32_t sqwOffsetTick = 0;
  bool haveOffset = false;

  for(;;)
  {
    const uint32_t tick = osKernelGetTickCount();
    if (g_probeRequest.exchange(false) && !probe.probing()) {
      an.restart();
      probe.startProbe(tick, RTC_PROBE_DURATION_MS);
      probeStartTick = tick;
      consolePrint("probe start: reads every %u ms for %lu s; I2C 0x%02X, SQW on A2 (TIM3), PPS on A1 (TIM2)\r\n",
                   RTC_PROBE_PERIOD_MS, RTC_PROBE_DURATION_MS / 1000, RTC_I2C_ADDR);
    }

    probe.main(tick);            // sleeps in osDelay between reads
    const bool probing = probe.probing();

    // SQW edges since last time
    const uint32_t n = g_sqwCount.load(std::memory_order_acquire);
    if (n - usedSqw > 8)
      usedSqw = n - 8;           // fell behind: the ring holds the last eight
    for (; usedSqw != n; ++usedSqw) {
      const uint32_t at = g_sqwRing[usedSqw & 7];
      const bool rising = g_sqwRising[usedSqw & 7] != 0;
      an.edge(at, rising);
      lastSqwTick = tick;
      sqwEver = true;
      if (probing)
        consolePrint("probe sqw %s %lu\r\n", rising ? "rise" : "fall", (unsigned long) at);
      if (!rising) {
        // against the PPS edge before it
        const uint32_t pn = g_ppsCount.load(std::memory_order_acquire);
        if (pn != 0) {
          const uint32_t p = g_ppsRing[(pn - 1) & 3];
          const uint32_t d = at - p;
          if (d < 1000000u) {
            sqwOffsetUs = (int32_t) d;
            sqwOffsetTick = tick;
            haveOffset = true;
            if (probing)
              consolePrint("probe pps %lu sqw-fall-after-pps %ldus\r\n", (unsigned long) p, (long) d);
          }
        }
      }
    }

    // A read landed
    rtc_probe_read_t rd;
    bool timeValid = false;
    ClockTime t = {};
    if (probe.takeRead(&rd)) {
      const uint8_t sec = (uint8_t)(((rd.regs[0] >> 4) & 0x07) * 10 + (rd.regs[0] & 0x0F));
      uint32_t tf = 0, tt = 0;
      const bool ticked = an.read(rd.from, rd.to, sec, &tf, &tt);
      if (probing) {
        consolePrint("probe read %lu..%lu sec %02u\r\n", (unsigned long) rd.from, (unsigned long) rd.to, sec);
        if (ticked)
          consolePrint("probe tick between %lu and %lu\r\n", (unsigned long) tf, (unsigned long) tt);
      }
      // BCD time, 24 hour (the DS3231's layout matches the DS1307's here;
      // its century bit, month bit 7, is ignored: 2000..2099)
      t.second = sec;
      t.minute = (uint8_t)(((rd.regs[1] >> 4) & 0x07) * 10 + (rd.regs[1] & 0x0F));
      t.hour = (uint8_t)(((rd.regs[2] >> 4) & 0x03) * 10 + (rd.regs[2] & 0x0F));
      t.day = (uint8_t)(((rd.regs[4] >> 4) & 0x03) * 10 + (rd.regs[4] & 0x0F));
      t.month = (uint8_t)(((rd.regs[5] >> 4) & 0x01) * 10 + (rd.regs[5] & 0x0F));
      t.year = (uint16_t)(2000 + (rd.regs[6] >> 4) * 10 + (rd.regs[6] & 0x0F));
      timeValid = (rd.regs[2] & 0x40) == 0 && DS1307::ds1307_valid(t);   // 12 hour mode isn't read here
      if (timeValid)
        t.weekday = DS1307::ds1307_weekday(t.year, t.month, t.day);
    }

    // The probe just ended: the verdict
    if (wasProbing && !probing) {
      verdict = an.verdict();
      const rtcedge::Window &f = an.falling(), &ri = an.rising();
      const rtcedge::Window &w = verdict == rtcedge::Polarity::Rising ? ri : f;
      vLo = w.loUs;
      vHi = w.hiUs;
      consolePrint("probe falling: %s %ld..%ld us (%u ticks)\r\n", f.consistent() ? "consistent" : "inconsistent",
                   (long) f.loUs, (long) f.hiUs, f.n);
      consolePrint("probe rising: %s %ld..%ld us (%u ticks)\r\n", ri.consistent() ? "consistent" : "inconsistent",
                   (long) ri.loUs, (long) ri.hiUs, ri.n);
      if (verdict != rtcedge::Polarity::Unknown)
        consolePrint("probe result: the seconds follow the %s edge of SQW, %ld..%ld us after it (%u ticks, %lu reads)\r\n",
                     verdictName(verdict), (long) vLo, (long) vHi, an.ticks(), (unsigned long) probe.reads());
      else
        consolePrint("probe result: undecided (%u ticks bracketed; SQW %s)\r\n", an.ticks(),
                     sqwEver ? "seen" : "never seen: check the wire to A2 and its pull-up");
    }
    wasProbing = probing;

    osMutexAcquire(gps_data_mutexHandle, osWaitForever);
    rtcStatus.present = probe.present();
    if (timeValid) {
      rtcStatus.time = t;
      rtcStatus.timeValid = true;
    } else if (!probe.present()) {
      rtcStatus.timeValid = false;
    }
    rtcStatus.sqw = sqwEver && (tick - lastSqwTick) < SQW_TIMEOUT_MS;
    rtcStatus.sqwVsPps = haveOffset && (tick - sqwOffsetTick) < SQW_TIMEOUT_MS;
    rtcStatus.sqwOffsetUs = sqwOffsetUs;
    rtcStatus.probing = probing;
    const uint32_t left = probing ? RTC_PROBE_DURATION_MS - (tick - probeStartTick) : 0;
    rtcStatus.probeSecondsLeft = (uint8_t)((left + 999) / 1000);
    rtcStatus.verdict = verdict;
    rtcStatus.verdictLoUs = vLo;
    rtcStatus.verdictHiUs = vHi;
    osMutexRelease(gps_data_mutexHandle);
  }
  /* USER CODE END StartRtcTask */
}

/* USER CODE BEGIN Header_StartConsoleTask */
/**
  * @brief  Starts USB (the CDC virtual COM port) and hands it the console
  *         ring. With no host on the port, what is waiting is thrown away,
  *         so a terminal opened later starts with current lines.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartConsoleTask */
void StartConsoleTask(void *argument)
{
  /* USER CODE BEGIN StartConsoleTask */
  MX_USB_DEVICE_Init();
  uint8_t chunk[APP_TX_DATA_SIZE];
  uint32_t lastDropped = 0;
  bool hostWasReady = false;

  for(;;)
  {
    osMutexAcquire(console_mutexHandle, osWaitForever);
    const uint32_t avail = g_conHead - g_conTail;
    if (!CDC_Ready_FS()) {
      g_conTail = g_conHead;
      osMutexRelease(console_mutexHandle);
      hostWasReady = false;
      osDelay(100);
      continue;
    }
    if (!hostWasReady) {
      hostWasReady = true;
      g_helpRequest.store(true);   // a terminal just opened the port: greet it
    }
    const uint32_t len = avail < sizeof(chunk) ? avail : sizeof(chunk);
    for (uint32_t i = 0; i < len; ++i)
      chunk[i] = (uint8_t) g_console[(g_conTail + i) % CONSOLE_RING];
    const uint32_t dropped = g_conDropped;
    osMutexRelease(console_mutexHandle);

    if (len == 0) {
      osDelay(5);
      continue;
    }
    uint8_t res;
    while ((res = CDC_Transmit_FS(chunk, (uint16_t) len)) == USBD_BUSY)
      osDelay(1);
    if (res == USBD_OK) {
      osMutexAcquire(console_mutexHandle, osWaitForever);
      g_conTail += len;
      osMutexRelease(console_mutexHandle);
    }
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
