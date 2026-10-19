/**
  ******************************************************************************
  * @file           : app.cpp
  * @brief          : BME680 bring-up on the NUCLEO-L432KC
  *
  *  A live test of a BME680 on I2C1: temperature, pressure, humidity and
  *  gas resistance, with Claude-iTransport's non-blocking driver
  *  (xbme680<Stm32HalI2CTransport>, forced every 3 s with the gas heater
  *  at 320 C for 150 ms, in sensorTask; see iTransport/SOURCE.txt). An
  *  SHT31 (0x44 or 0x45), if one answers, runs in its own task
  *  (xSHT31<Stm32HalI2CTransport>, periodic, one a second) to compare
  *  against. The BME680's station pressure also goes through a
  *  SeaLevelBarometer (isensor/inc): the altimeter setting for the
  *  station's elevation, the sea-level figure a Davis console or
  *  weather.gov shows, with a calibration offset the console can set
  *  against such a reference. Once a second the latest readings go to
  *  the ST-LINK's virtual COM port (USART2, 115200 8N1).
  *
  *  At start-up the I2C bus is scanned and two ID registers read at 0x76
  *  and 0x77: 0x61 in register 0xD0 is a BME680 (or BME688, which the same
  *  driver runs); 0x60 or 0x58 there a BME280 or BMP280, and 0x60 in
  *  register 0x00 a BMP390, reported but not driven. The scan repeats
  *  every 2 s until a BME680 or an SHT31 answers; each one found then
  *  gets its driver.
  *
  *  A debug log (Claude-iTransport's DebugLog.h, at ITRANSPORT_DEBUG 2, set
  *  in the project's defines) goes out on USART1 TX, D5 (PB6), at 921600
  *  8N1: the drivers' state changes and I2C faults ("bme680", "sht31", "bus"), and
  *  this application's events ("app"). Read it with a 3.3 V USB-serial
  *  adapter or the Saleae's async serial analyser.
  *
  *  main.c is CubeMX's: its USER CODE blocks only call app_init(),
  *  app_sensor_task() and app_console_task() (app.h), so regenerating code
  *  in CubeMX keeps them, and this file is never touched.
  *
  *  The GPS clock this project started as (GPS, OLED, W5500, NTP, PPS) is
  *  in the repository's history at commit eb4b352.
  *
  *  Wiring (NUCLEO-L432KC):
  *    I2C1_SCL  PA9  (D1)  -> BME680 SCK (SCL), SHT31 SCL (if fitted)
  *    I2C1_SDA  PA10 (D0)  -> BME680 SDI (SDA), SHT31 SDA
  *    3V3, GND             -> BME680 (and SHT31) VIN, GND
  *    (BME680 SDO: 0x77 high or open on most boards, 0x76 to GND; either is found)
  *    USART2    PA2/PA15   <> ST-LINK virtual COM port (the USB cable)
  *    USART1_TX PB6  (D5)  -> debug log, 921600 8N1 (USART1_RX PB7, D4, unused)
  *    LD3       PB3        green LED: toggles on each BME680 reading
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"
#include "app.h"
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
#include "xSHT31.h"
#include "xbme680.h"
#include "SeaLevelBarometer.h"

// Defined by CubeMX in main.c
extern "C" {
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern osMutexId_t i2c1_mutexHandle;
extern osMutexId_t console_mutexHandle;
}

typedef xSHT31<Stm32HalI2CTransport> Sht31Sensor;
typedef BME680::xbme680<Stm32HalI2CTransport> Bme680Sensor;

// The two addresses of Bosch's chips (their SDO pin)
#define BOSCH_ADDR_LOW          0x76
#define BOSCH_ADDR_HIGH         0x77
// The ID register of the BME680 and Bosch's older chips (0xD0) and what
// it holds for them (Bosch's datasheets and APIs, as the drivers and
// general knowledge have them): the BME680 is driven, the rest reported
#define CHIP_ID_REG             0xD0
#define CHIP_ID_BME280          0x60
#define CHIP_ID_BMP280          0x58
#define CHIP_ID_BME680          0x61
// The BMP390's ID register is 0x00 (Bosch's BMP3 API): 0x60, the same
// value as a BME280's, in another register. Reported only.
#define BMP3_CHIP_ID_REG        0x00
#define CHIP_ID_BMP390          0x60
#define CHIP_ID_BMP388          0x50

#define SHT31_ADDR_A            0x44       // ADDR pin low (most boards)
#define SHT31_ADDR_B            0x45       // ADDR pin high

// The station's elevation above mean sea level, for the altimeter setting.
// Set yours here, or with the console's e command (it isn't kept across a
// reset). 40 m: this board's station (elevationfinder.net).
#define STATION_ELEVATION_M     40.0f
#define M_PER_FT                0.3048f

#define SCAN_RETRY_MS           2000UL     // neither sensor found: scan again this often
#define READING_STALE_MS        3000UL     // SHT31: no reading for this long: say so
#define BME680_PERIOD_MS        3000UL     // a measurement every 3 s: the heater warms the chip less
#define BME680_STALE_MS         (2 * BME680_PERIOD_MS + 1000UL)

// The debug log's UART (USART1) runs at 921600, set in the .ioc (main.c):
// fast, since level 2 logs every state change

#define CONSOLE_RING            2048
#define CONSOLE_LINE            48

// shtTask: started by sensorTask when an SHT31 answers. Not in the .ioc,
// so it is defined here rather than in main.c.
static osThreadId_t shtTaskHandle;
static const osThreadAttr_t shtTask_attributes = {
  .name = "shtTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// The console: a ring the tasks write (under console_mutex) and
// consoleTask hands to USART2
static char g_console[CONSOLE_RING];
static uint32_t g_conHead = 0, g_conTail = 0;
static uint32_t g_conDropped = 0;

// A command line typed on the console, handed from the UART interrupt to
// sensorTask: g_cmdReady set once it is complete, cleared when taken
static char g_cmd[CONSOLE_LINE];
static std::atomic<bool> g_cmdReady{false};

// The SHT31: its address (0 if none) for shtTask, and its latest reading
// for sensorTask (written by shtTask only; each value a 32-bit atomic)
static std::atomic<uint8_t> g_shtAddr{0};
static std::atomic<float> g_shtT{NAN}, g_shtH{NAN};
static std::atomic<uint32_t> g_shtReadings{0}, g_shtTick{0};
static std::atomic<int> g_shtState{0};

// The BME680's latest pressure as a Barometer, for the SeaLevelBarometer
// (the driver has no Barometer interface of its own). NAN until the first
// reading, and while there is no BME680.
struct ReadingBarometer : Barometer {
  float pa = NAN;
  float getPressure() const override { return pa; }
};

static void StartShtTask(void *argument);
static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// The HAL UART and I2C interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32UartItCallbacks.cpp, Stm32I2CItCallbacks.cpp): don't define them here.

// printf to the console. Tasks only; a line that doesn't fit is dropped
// and counted.
static void consolePrint(const char *fmt, ...)
{
  char line[256];
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
  consolePrint("\r\nNUCLEO-L432KC BME680 bring-up: I2C1 SCL=D1 (PA9) SDA=D0 (PA10), 100 kHz\r\n"
               "commands (Enter to finish; turn on your terminal's local echo):\r\n"
               "  s          scan the I2C bus and read the chip IDs at 0x76 and 0x77\r\n"
               "  e 131      the station's elevation in feet (e 40m for metres), for the sea-level reading\r\n"
               "  b 30.54    calibrate: make the sea-level reading this many inHg now (from weather.gov, say)\r\n"
               "  o          clear the calibration offset\r\n"
               "  h          this help\r\n\r\n");
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

// Every address 0x08..0x77 that acknowledges, and two ID registers at 0x76
// and 0x77: 0x00 (the BMP390's) and 0xD0 (the older Bosch chips'). Blocking
// HAL calls under i2c1_mutex, about 15 ms. The mutex is not recursive and
// a driver holds it from issuing a transfer until it sees it land, so call
// this only from a thread with no transfer outstanding (sensorTask waits
// for a state of its driver's with none). Returns the address of a BME680
// (or BME688), or 0 (and 0 if the bus stayed busy); an SHT31's through
// shtAddr (0 if none).
static uint8_t scanBus(uint8_t *shtAddr = nullptr)
{
  uint8_t found = 0;
  uint8_t sht = 0;
  char list[160];
  size_t n = 0;
  list[0] = 0;
  if (osMutexAcquire(i2c1_mutexHandle, 200) != osOK) {
    consolePrint("I2C bus busy: no scan\r\n");
    return 0;
  }
  bool acked[2] = {false, false};
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 2) == HAL_OK) {
      if (n + 6 < sizeof(list))
        n += (size_t) snprintf(list + n, sizeof(list) - n, " 0x%02X", a);
      if ((a == SHT31_ADDR_A || a == SHT31_ADDR_B) && sht == 0)
        sht = a;
      if (a == BOSCH_ADDR_LOW || a == BOSCH_ADDR_HIGH)
        acked[a - BOSCH_ADDR_LOW] = true;
    }
  }
  // Both ID registers at each address that answered
  uint8_t ids3[2] = {0, 0}, ids[2] = {0, 0};
  bool idOk[2] = {false, false};
  for (int i = 0; i < 2; ++i) {
    const uint8_t a = (uint8_t)(BOSCH_ADDR_LOW + i);
    if (!acked[i])
      continue;
    idOk[i] = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(a << 1), BMP3_CHIP_ID_REG, I2C_MEMADD_SIZE_8BIT, &ids3[i], 1, 10) == HAL_OK
              && HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(a << 1), CHIP_ID_REG, I2C_MEMADD_SIZE_8BIT, &ids[i], 1, 10) == HAL_OK;
  }
  osMutexRelease(i2c1_mutexHandle);

  consolePrint("I2C scan:%s\r\n", n ? list : " nothing answered (check SDA D0, SCL D1, 3V3, GND, pull-ups)");
  for (int i = 0; i < 2; ++i) {
    const uint8_t a = (uint8_t)(BOSCH_ADDR_LOW + i);
    if (!idOk[i])
      continue;
    DBG_EVENT("app", "chip-id", (int32_t) a, (int32_t) ids3[i], (int32_t) ids[i]);
    // The BME680's register first: it is the one this example drives
    if (ids[i] == CHIP_ID_BME680) {
      consolePrint("  0x%02X: chip ID 0x%02X (register 0xD0) = BME680 (or BME688)\r\n", a, ids[i]);
      if (found == 0)
        found = a;
    } else if (ids[i] == CHIP_ID_BME280 || ids[i] == CHIP_ID_BMP280) {
      consolePrint("  0x%02X: chip ID 0x%02X (register 0xD0) = %s: not driven by this firmware\r\n", a, ids[i],
                   chipName(ids[i]));
    } else if (ids3[i] == CHIP_ID_BMP390 || ids3[i] == CHIP_ID_BMP388) {
      consolePrint("  0x%02X: chip ID 0x%02X (register 0x00) = %s: not driven by this firmware\r\n", a, ids3[i],
                   ids3[i] == CHIP_ID_BMP390 ? "BMP390" : "BMP388");
    } else {
      // Two chips at one address read as the AND of their answers
      consolePrint("  0x%02X: chip IDs 0x%02X (0x00), 0x%02X (0xD0): unknown; two chips at this address?\r\n", a,
                   ids3[i], ids[i]);
    }
  }
  if (sht != 0)
    consolePrint("  0x%02X: answers where an SHT31 would be (it has no ID register to check)\r\n", sht);
  if (shtAddr != nullptr)
    *shtAddr = sht;
  return found;
}

// From main(), after the peripherals are set up and before the scheduler
// starts: the debug log on USART1. Lines queue in a ring from here on, and
// consoleTask's dbg::poll() sends them.
void app_init(void)
{
  static Stm32HalUartTransport debugUart(&huart1);
  dbg::begin(&debugUart, Stm32DebugPort::port());
  DBG_EVENT("app", "start", (int32_t) HAL_GetREVID(), (int32_t) HAL_GetDEVID());
}

/**
  * @brief  sensorTask's body (main.c's StartSensorTask calls it): finds the
  *         BME680 and the SHT31 (bus scan, chip ID), runs the BME680's
  *         driver, starts shtTask for an SHT31, takes the console's
  *         commands and prints the latest readings once a second.
  */
void app_sensor_task(void *argument)
{
  (void) argument;
  printHelp();

  // Find them first: the BME680 at 0x76 or 0x77 by its chip ID, an SHT31
  // at 0x44 or 0x45
  uint8_t addr = 0;
  uint8_t shtAddr = 0;
  while ((addr = scanBus(&shtAddr)) == 0 && shtAddr == 0) {
    consolePrint("no BME680 (chip ID 0x61 in register 0xD0) at 0x76 or 0x77, no SHT31 at 0x44 or 0x45;"
                 " scanning again in %lu s\r\n", (unsigned long)(SCAN_RETRY_MS / 1000));
    osDelay(SCAN_RETRY_MS);
  }
  if (shtAddr != 0) {
    g_shtAddr.store(shtAddr);
    shtTaskHandle = osThreadNew(StartShtTask, NULL, &shtTask_attributes);
    consolePrint("SHT31 at 0x%02X: periodic mode, 1 a second, high repeatability\r\n", shtAddr);
  }

  // The BME680: forced every 3 s; temperature x2, pressure x4, humidity
  // x1, IIR filter 3; the gas heater at 320 C for 150 ms (Bosch's forced
  // mode example's settings, the heater value worked out for 25 C).
  // Built here only if it answered: its transport needs i2c1_mutex and
  // the address just found.
  Bme680Sensor *bme = nullptr;
  if (addr != 0) {
    BME680::bme680_param_t param;
    param.bus = BME680::bme680_i2c;
    param.os_temperature = BME680::bme680_os_2x;
    param.os_pressure = BME680::bme680_os_4x;
    param.os_humidity = BME680::bme680_os_1x;
    param.filter = BME680::bme680_filter_3;
    param.heater_c = 320;
    param.heater_ms = 150;
    param.ambient_c = 25;
    param.period_ms = BME680_PERIOD_MS;
    static Bme680Sensor sensor(param, &hi2c1, addr, i2c1_mutexHandle);
    sensor.setDebugTag("bme680");           // its state changes and faults in the debug log
    bme = &sensor;
    consolePrint("BME680 at 0x%02X: forced every %lu s, T x2, P x4, H x1, filter 3, heater 320 C for 150 ms\r\n", addr,
                 (unsigned long)(BME680_PERIOD_MS / 1000));
  } else {
    consolePrint("no BME680 at 0x76 or 0x77: SHT31 only\r\n");
  }
  // Its pressure brought to sea level (the altimeter setting) for the
  // station's elevation; the offset is the console's b command's
  static ReadingBarometer pressure;
  static SeaLevelBarometer slp(pressure, STATION_ELEVATION_M);

  BME680::bme680_reading_t r;
  r.temperature = r.pressure = r.humidity = r.gas_resistance = NAN;
  r.gas_valid = r.heat_stable = false;
  uint32_t readings = 0;
  uint32_t lastReadingTick = 0;
  bool printPending = false;             // a reading not printed yet
  uint32_t lastPrintTick = osKernelGetTickCount();
  float tMin = NAN, tMax = NAN;
  bool scanPending = false;

  for(;;)
  {
    if (bme != nullptr)
      bme->main(osKernelGetTickCount());    // sleeps inside, up to 100 ms
    else
      osDelay(100);
    const uint32_t tick = osKernelGetTickCount();

    if (bme != nullptr && bme->newData()) {
      bme->getReading(&r);
      pressure.pa = r.pressure;
      ++readings;
      lastReadingTick = tick;
      printPending = true;
      if (!isnan(r.temperature)) {
        if (isnan(tMin) || r.temperature < tMin) tMin = r.temperature;
        if (isnan(tMax) || r.temperature > tMax) tMax = r.temperature;
      }
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      DBG_EVENT("app", "reading", (int32_t) lroundf(r.temperature * 100), (int32_t) lroundf(r.pressure),
                (int32_t) lroundf(r.humidity * 100),
                isnan(r.gas_resistance) ? (int32_t) -1 : (int32_t) lroundf(r.gas_resistance),
                (int32_t)((r.gas_valid ? 1 : 0) | (r.heat_stable ? 2 : 0)));
    }

    // A command typed on the console
    if (g_cmdReady.load(std::memory_order_acquire)) {
      char cmd[CONSOLE_LINE];
      strcpy(cmd, g_cmd);
      g_cmdReady.store(false, std::memory_order_release);
      consolePrint("> %s\r\n", cmd);
      DBG_EVENT("app", "cmd", (int32_t) cmd[0]);
      char *end = nullptr;
      if (cmd[0] == 's' && cmd[1] == 0) {
        scanPending = true;                 // done below, between the driver's transfers
      } else if (cmd[0] == 'e' && cmd[1] == ' ') {
        const float v = strtof(cmd + 2, &end);
        const bool metres = end != nullptr && (*end == 'm' || *end == 'M');
        if (end == cmd + 2 || !(v > -500.0f && v < 30000.0f)) {
          consolePrint("e <feet> or e <metres>m, e.g. e 131 or e 40m\r\n");
        } else {
          slp.setElevation(metres ? v : v * M_PER_FT);
          char m[16];
          formatFixed(m, sizeof(m), slp.elevation(), 1);
          consolePrint("elevation %s m (%ld ft); the calibration offset is kept: b again if it was set at another elevation\r\n",
                       m, lroundf(slp.elevation() / M_PER_FT));
        }
      } else if (cmd[0] == 'b' && cmd[1] == ' ') {
        const float inHg = strtof(cmd + 2, &end);
        if (end == cmd + 2 || !(inHg > 25.0f && inHg < 33.0f)) {
          consolePrint("b <inHg>, the sea-level pressure to match now, e.g. b 30.54\r\n");
        } else if (!slp.calibrateTo(baro::inHgToPa(inHg))) {
          consolePrint("no pressure reading yet: try again in a moment\r\n");
        } else {
          char o[16];
          formatFixed(o, sizeof(o), slp.offset() / 100.0, 2);
          consolePrint("calibrated: offset %s hPa added to the station pressure\r\n", o);
          DBG_EVENT("app", "cal", (int32_t) lroundf(slp.offset()), (int32_t) lroundf(slp.elevation()));
        }
      } else if (cmd[0] == 'o' && cmd[1] == 0) {
        slp.setOffset(0.0f);
        consolePrint("calibration offset cleared\r\n");
      } else if ((cmd[0] == 'h' || cmd[0] == '?') && cmd[1] == 0) {
        printHelp();
      } else {
        consolePrint("unknown command; h for help\r\n");
      }
    }

    // The scan: only in a state where the driver has no transfer
    // outstanding (it holds i2c1_mutex from issuing one until it sees it
    // land, and the mutex isn't recursive). It spends most of its time in these.
    if (scanPending) {
      const BME680::bme680_state_t st = bme != nullptr ? bme->state() : BME680::bme680_done_state;
      if (st == BME680::bme680_done_state || st == BME680::bme680_measuring_state
          || st == BME680::bme680_field_poll_state || st == BME680::bme680_reset_settle_state
          || st == BME680::bme680_error_state) {
        scanPending = false;
        (void) scanBus();
      }
    }

    // Once a second: the latest readings, flagged where pinned at a limit
    // of Bosch's compensation (-40..85 C, 0 or 100 %RH)
    if (tick - lastPrintTick >= 1000) {
      lastPrintTick = tick;
      if (bme != nullptr) {
        const bool stale = readings == 0 || tick - lastReadingTick > BME680_STALE_MS;
        if (stale) {
          consolePrint("BME680 at 0x%02X: no reading%s (driver state %d%s)\r\n", addr,
                       readings == 0 ? " yet" : " for 7 s", (int) bme->state(),
                       bme->state() == BME680::bme680_error_state ? ", error: not answering, or a wrong chip ID" : "");
        } else if (printPending) {
          // A new reading (one every 3 s): print it once. A flag, not "less
          // than a second old": the print's second drifts against the
          // reading's, and one reading in ten went unprinted that way.
          printPending = false;
          char ts[16], tf[16], hs[16], ps[16], sl[16], sh[16], el[16], of[16], gs[16], lo[16], hi[16];
          formatFixed(ts, sizeof(ts), r.temperature, 2);
          formatFixed(tf, sizeof(tf), r.temperature * 9.0 / 5.0 + 32.0, 1);
          formatFixed(hs, sizeof(hs), r.humidity, 1);
          formatFixed(ps, sizeof(ps), r.pressure / 100.0, 2);
          const float alt = slp.getPressure();
          formatFixed(sl, sizeof(sl), baro::paToInHg(alt), 2);
          formatFixed(sh, sizeof(sh), alt / 100.0, 2);
          formatFixed(el, sizeof(el), slp.elevation(), 0);
          formatFixed(of, sizeof(of), slp.offset() / 100.0, 2);
          formatFixed(gs, sizeof(gs), r.gas_resistance / 1000.0, 1);
          formatFixed(lo, sizeof(lo), tMin, 2);
          formatFixed(hi, sizeof(hi), tMax, 2);
          const bool tBad = r.temperature <= -40.0f || r.temperature >= 85.0f;
          const bool hBad = r.humidity <= 0.0f || r.humidity >= 100.0f;
          consolePrint("BME680 T %s C (%s F)%s  H %s %%RH%s  P %s hPa = %s inHg / %s hPa at sea level (%s m, cal %s hPa)"
                       "  gas %s kOhm%s | %lu readings, T %s..%s C\r\n", ts, tf, tBad ? " (?)" : "", hs, hBad ? " (?)" : "",
                       ps, sl, sh, el, of, gs,
                       !r.gas_valid ? " (no gas reading)" : !r.heat_stable ? " (heater not stable)" : "",
                       (unsigned long) readings, lo, hi);
          DBG_EVENT("app", "slp", (int32_t) lroundf(alt));
        }
      }
      // The SHT31 beside it
      const uint32_t shtN = g_shtReadings.load();
      if (g_shtAddr.load() != 0) {
        const float st = g_shtT.load(), shh = g_shtH.load();
        if (shtN == 0 || tick - g_shtTick.load() > READING_STALE_MS) {
          consolePrint("SHT31  no reading%s (driver state %d%s)\r\n", shtN == 0 ? " yet" : " for 3 s", g_shtState.load(),
                       g_shtState.load() == sht31_error_state ? ", error: not answering" : "");
        } else {
          char a[16], af[16], b[16], dt[16], dh[16];
          formatFixed(a, sizeof(a), st, 2);
          formatFixed(af, sizeof(af), st * 9.0 / 5.0 + 32.0, 1);
          formatFixed(b, sizeof(b), shh, 1);
          formatFixed(dt, sizeof(dt), r.temperature - st, 2);
          formatFixed(dh, sizeof(dh), r.humidity - shh, 1);
          consolePrint("SHT31  T %s C (%s F)  H %s %%RH | BME680 - SHT31: T %s C, H %s %%RH | %lu readings\r\n", a, af, b,
                       dt, dh, (unsigned long) shtN);
        }
      }
    }
  }
}

/**
  * @brief  Runs the SHT31's driver in periodic mode (one reading a second)
  *         and publishes each reading for sensorTask. Started by
  *         sensorTask once the bus scan has found an SHT31; the driver
  *         gets its own thread since its main() sleeps inside.
  * @param  argument: Not used
  * @retval None
  */
static void StartShtTask(void *argument)
{
  SHT31_param_t param;
  param.periodic_mode = true;
  param.heater_enable = false;
  param.sample_rate = _1mps_high_Res;
  static Sht31Sensor sht(param, &hi2c1, g_shtAddr.load(), i2c1_mutexHandle);
  sht.setDebugTag("sht31");

  for(;;)
  {
    sht.main(osKernelGetTickCount());       // sleeps inside, up to 100 ms
    g_shtState.store((int) sht.state());
    if (sht.newData()) {
      float t = NAN, h = NAN;
      sht.GetTemperature(&t);
      sht.GetHumidity(&h);
      g_shtT.store(t);
      g_shtH.store(h);
      g_shtTick.store(osKernelGetTickCount());
      g_shtReadings.fetch_add(1);
      DBG_EVENT("app", "sht", (int32_t) lroundf(t * 100), (int32_t) lroundf(h * 100));
    }
  }
}

/**
  * @brief  consoleTask's body (main.c's StartConsoleTask calls it): hands
  *         the console ring to USART2 (the ST-LINK's virtual COM port), a
  *         piece at a time, listens for keys, and sends the debug log.
  */
void app_console_task(void *argument)
{
  (void) argument;
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
  
}
