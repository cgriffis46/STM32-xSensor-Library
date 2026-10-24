/**
  ******************************************************************************
  * @file           : app.cpp
  * @brief          : Sensor bring-up on the NUCLEO-L432KC
  *
  *  A live test bench for Claude-iTransport's non-blocking sensor drivers
  *  (see iTransport/SOURCE.txt). Plug sensors onto I2C1 and reset: the bus
  *  is scanned, each chip is told apart by its address and, where it has
  *  one, its ID register, and each one found gets its driver in a thread
  *  of its own (an x* driver sleeps inside main(), and each driver holds
  *  the bus mutex from issuing a transfer until it lands, so two cannot
  *  share a thread). Each reading is printed as it arrives, on the
  *  ST-LINK's virtual COM port (USART2, 115200 8N1), which is a GUI for
  *  PuTTY (iDisplay's xTerminal and xGui): the home screen at the top (the
  *  MPL3115A2's cell, the command line) and the console's last lines below.
  *
  *  I2C1, what is looked for (address: chip, how it is recognised):
  *    0x19      LSM303DLHC accelerometer (answers; it has no ID register)
  *    0x1E      LSM303DLHC magnetometer  (IRA..IRC registers "H43")
  *    0x21      HMC6352 compass          (answers)
  *    0x30      MMC5603/MMC5633          (product ID 0x10 in 0x39)
  *    0x38      AHT20                    (answers)
  *    0x40      Si7021/Si7020/Si7013     (electronic ID byte SNB_3 0x15/0x14/0x0D),
  *              otherwise HTU21D(F)      (the same commands; general knowledge)
  *    0x44/45   SHT31                    (answers)
  *    0x5C/5D   LPS35HW                  (WHO_AM_I 0xB1 in 0x0F)
  *    0x60      MPL3115A2                (WHO_AM_I 0xC4 in 0x0C)
  *    0x68      DS3231                   (registers 0x0E..0x12 as only a DS3231 has
  *                                        them, and its register pointer wrapping
  *                                        from 0x12 to the seconds; read only),
  *              otherwise DS1307         (answers; an IMU would too)
  *    0x76/77   BME280 0x60, BMP280 0x58, BME680 0x61 in 0xD0;
  *              BMP390 0x60, BMP388 0x50 in 0x00
  *  One of each kind is driven (the first found); a second is reported.
  *
  *  USART1 is the debug log's (TX, D5) unless BRINGUP_USART1 says otherwise
  *  (the L432KC has no other free UART): BRINGUP_USART1_PM25 reads a
  *  Plantower PMS5003 (PM25 driver) at 9600 on RX, D4; BRINGUP_USART1_DS18B20
  *  runs a 1-Wire bus for one DS18B20 on D5 (open drain) linked to D4.
  *
  *  Every barometer's station pressure also goes through its own
  *  SeaLevelBarometer (isensor/inc): the altimeter setting for the
  *  station's elevation, the sea-level figure a Davis console or
  *  weather.gov shows, with a calibration offset the console can set.
  *  The MPL3115A2 also measures its own altitude each period (both
  *  modes, one after the other) from the altimeter setting the console's
  *  a command gives it: with weather.gov's, an estimate of the station's
  *  elevation; until then 29.92 inHg (pressure altitude).
  *
  *  A debug log (Claude-iTransport's DebugLog.h, at ITRANSPORT_DEBUG 2, set
  *  in the project's defines) goes out on USART1 TX, D5 (PB6), at 921600
  *  8N1: each driver's state changes and I2C faults (tagged by sensor),
  *  "bus", and this application's events ("app"), each reading included.
  *
  *  main.c is CubeMX's: its USER CODE blocks only call app_init(),
  *  app_sensor_task() and app_console_task() (app.h), so regenerating code
  *  in CubeMX keeps them, and this file is never touched.
  *
  *  Wiring (NUCLEO-L432KC):
  *    I2C1_SCL  PA9  (D1)  -> every sensor's SCL (SCK)
  *    I2C1_SDA  PA10 (D0)  -> every sensor's SDA (SDI)
  *    3V3, GND             -> the sensors' VIN, GND
  *    USART2    PA2/PA15   <> ST-LINK virtual COM port (the USB cable)
  *    USART1_TX PB6  (D5)  -> debug log, 921600 8N1 (or the 1-Wire bus)
  *    USART1_RX PB7  (D4)  <- unused (or a PMS5003's TX, or the 1-Wire bus)
  *    LD3       PB3        green LED: toggles on each reading
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
#include <new>
#include "Stm32HalI2CTransport.h"
#include "Stm32HalUartTransport.h"
#include "Stm32HalOneWireTransport.h"
#include "Stm32DebugPort.h"
#include "DebugLog.h"
#include "SeaLevelBarometer.h"
#include "xGui.h"
#include "xTerminal.h"
#include "xTerminalKeys.h"
#include "xLogLines.h"
#include "xCell.h"
#include "xAHT20.h"
#include "xBME280.h"
#include "xbme680.h"
#include "xbmp280.h"
#include "xbmp390.h"
#include "xDS18B20.h"
#include "xds1307.h"
#include "xds3231.h"
#include "xhmc6352.h"
#include "xHTU21DF.h"
#include "xlps35hw.h"
#include "xlsm303dlhc.h"
#include "xmmc56x3.h"
#include "xmpl3115a2.h"
#include "xPM25.h"
#include "xSHT31.h"
#include "xSi7021.h"

// Defined by CubeMX in main.c
extern "C" {
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern osMutexId_t i2c1_mutexHandle;
extern osMutexId_t console_mutexHandle;
}

typedef Stm32HalI2CTransport I2C;

// What USART1 is for: the debug log, a PMS5003, or a 1-Wire bus. Set it in
// the project's defines (-DBRINGUP_USART1=1, say).
#define BRINGUP_USART1_DEBUG    0
#define BRINGUP_USART1_PM25     1
#define BRINGUP_USART1_DS18B20  2
#ifndef BRINGUP_USART1
#define BRINGUP_USART1          BRINGUP_USART1_DEBUG
#endif

// The station's elevation above mean sea level, for the altimeter setting.
// Set yours here, or with the console's e command (it isn't kept across a
// reset). 40 m: this board's station (elevationfinder.net).
#define STATION_ELEVATION_M     40.0f
#define M_PER_FT                0.3048f

#define SCAN_RETRY_MS           2000UL     // nothing found: scan again this often
#define MAX_SENSORS             16
#define SENSOR_STACK_WORDS      512        // each sensor thread: driver, formatting, consolePrint

#define CONSOLE_LINE            48
// The PuTTY window: the GUI (40 x 7 inside a frame) and below it the
// console's last lines, 79 wide, so the whole fits PuTTY's default 80 x 24
// with its last row free
#define CONSOLE_COLS            40
#define CONSOLE_ROWS            7
#define CONSOLE_LOG_ROWS        14
#define CONSOLE_LOG_COLS        79
#define GUI_STACK_BYTES         1536

// Chip IDs and their registers (Bosch's and the makers' APIs and
// datasheets, as the drivers and general knowledge have them)
#define BOSCH_ID_REG            0xD0       // BME280, BMP280, BME680
#define BMP3_ID_REG             0x00       // BMP390, BMP388
#define ID_BME280               0x60
#define ID_BMP280               0x58
#define ID_BME680               0x61
#define ID_BMP390               0x60
#define ID_BMP388               0x50
#define LPS35HW_ID_REG          0x0F
#define MPL3115A2_ID_REG        0x0C
#define MMC56X3_ID_REG          0x39
#define LSM303_MAG_ID_REG       0x0A       // "H43" in 0x0A..0x0C

// ---- the console ----------------------------------------------------------
// The PuTTY terminal (USART2) is a GUI: the home screen (the MPL3115A2's
// cell, the command line) framed at the top, and below it a pane of the
// console's last lines. consolePrint() adds to those lines (under
// console_mutex, which the terminal also takes to read them) and asks the
// GUI to draw; the terminal is the only thing that writes to USART2.
// The terminal and these lines are in SRAM2 (the linker script's .ram2,
// not cleared at start-up: their constructors and first use set every byte
// they read), as SRAM1 is full.
#define IN_RAM2 __attribute__((section(".ram2")))
static idisplay::xLogLines<CONSOLE_LOG_ROWS, CONSOLE_LOG_COLS> g_logLines IN_RAM2;
static std::atomic<idisplay::xGui *> g_gui{nullptr};   // set once consoleTask has started it

// The log pane's own lock: held only while a finished line is copied in or
// out. Writers also hold console_mutex around a whole append.
static osMutexId_t g_logMutex = nullptr;
static void logLock(void *) { osMutexAcquire(g_logMutex, osWaitForever); }
static void logUnlock(void *) { osMutexRelease(g_logMutex); }

// Asks the GUI to draw again (new data, a new console line, a key). From a
// task or an interrupt; dropped while an event is already waiting, which
// draws everything anyway.
static void guiRefresh(void)
{
  idisplay::xGui *gui = g_gui.load(std::memory_order_acquire);
  if (gui != nullptr)
    gui->post(idisplay::xGuiEvent::refresh(), 0);
}

// A command line typed on the console, handed from the UART interrupt to
// sensorTask: g_cmdReady set once it is complete, cleared when taken
static char g_cmd[CONSOLE_LINE];
static std::atomic<bool> g_cmdReady{false};

static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// The HAL UART and I2C interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32UartItCallbacks.cpp, Stm32I2CItCallbacks.cpp): don't define them here.

// printf to the console's log pane. Tasks only. Lines end at '\n' ('\r' is
// dropped); one longer than the pane is cut there.
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
  g_logLines.append(line, (size_t) n);
  osMutexRelease(console_mutexHandle);
  guiRefresh();
}

// The terminal's keys (interrupt context). Escape sequences (the arrow
// keys), Ctrl-L, and Enter or Backspace on an empty command line go to the
// GUI (xTerminalKeys); printable characters build the command line, which
// the home screen shows on its bottom row, and Enter hands it to
// sensorTask. Nothing is echoed by the terminal: turn PuTTY's local echo
// off. A line finished while the last is still waiting is kept, to send
// again with Enter.
struct ConsoleKeys : iTransportRxSink {
  char buf[CONSOLE_LINE];
  std::atomic<uint8_t> len{0};
  idisplay::xTerminalKeys *keys = nullptr;   // set before the UART's sink is
  uint8_t esc = 0;                           // 1 after ESC, 2 in ESC [ ..., 3 after ESC O
  bool lastCr = false;
  void key(uint8_t b) { if (keys != nullptr) keys->onByteReceived(b); }
  void onByteReceived(uint8_t b) override {
    if (esc != 0) {
      key(b);
      if (esc == 1)
        esc = b == '[' ? 2 : b == 'O' ? 3 : 0;
      else if (esc == 3 || (b >= 0x40 && b <= 0x7E))
        esc = 0;                             // the sequence's last byte
      return;
    }
    if (b == 0x1B) {
      esc = 1;
      key(b);
      return;
    }
    const bool cr = b == '\r';
    if (b == '\n' && lastCr) {              // CR LF: one Enter
      lastCr = false;
      return;
    }
    lastCr = cr;
    const uint8_t n = len.load(std::memory_order_relaxed);
    if (cr || b == '\n') {
      if (n == 0) {
        key('\r');
      } else if (!g_cmdReady.load(std::memory_order_acquire)) {
        memcpy(g_cmd, buf, n);
        g_cmd[n] = 0;
        g_cmdReady.store(true, std::memory_order_release);
        len.store(0, std::memory_order_release);
      }
    } else if (b == 0x08 || b == 0x7F) {
      if (n == 0)
        key(b);
      else
        len.store((uint8_t) (n - 1), std::memory_order_release);
    } else if (b == 0x0C) {
      key(b);                                // Ctrl-L: repaint
    } else if (b >= 0x20 && b < 0x7F && n < CONSOLE_LINE - 1) {
      buf[n] = (char) b;
      len.store((uint8_t) (n + 1), std::memory_order_release);
    } else {
      return;
    }
    guiRefresh();
  }
  // The line so far, for the home screen (the GUI task)
  uint8_t copy(char *out, uint8_t cap) const {
    uint8_t n = len.load(std::memory_order_acquire);
    if (n > cap - 1)
      n = (uint8_t) (cap - 1);
    memcpy(out, buf, n);
    out[n] = 0;
    return n;
  }
};
static ConsoleKeys consoleKeys;

static void printHelp(void)
{
  static const char *const lines[] = {
    "NUCLEO-L432KC sensor bring-up: I2C1 SCL=D1 (PA9) SDA=D0 (PA10), 100 kHz",
    "type a command, Enter to run it (PuTTY's local echo off); Ctrl-L repaints",
    "  s         scan the I2C bus again (reports only)",
    "  e 131     the station's elevation, feet (e 40m: metres)",
    "  b 30.54   calibrate each barometer's sea-level reading to this, inHg",
    "  o         clear the calibration offsets",
    "  a 30.36   the MPL3115A2's altimeter setting, inHg, for its altitude",
    "  t 2026-10-19 14:05:00   set the RTC's time (DS1307 or DS3231, 24 hour)",
    "  g -5      the DS3231's aging offset (-128..127, about 0.1 ppm a step)",
    "  h         this help",
  };
  for (const char *l : lines)
    consolePrint("%s\n", l);
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

// A reading's line, built a piece at a time
struct Line {
  char buf[200];
  size_t n = 0;
  int32_t dbg[6];                          // for the debug log: the values x100 (or as given)
  int nDbg = 0;
  Line() { buf[0] = 0; }
  void text(const char *s) { n += (size_t) snprintf(buf + n, n < sizeof(buf) ? sizeof(buf) - n : 0, "%s", s); if (n >= sizeof(buf)) n = sizeof(buf) - 1; }
  // "name value unit", the value with `decimals` places; logged x10^decimals
  void value(const char *name, double v, int decimals, const char *unit) {
    char f[16];
    formatFixed(f, sizeof(f), v, decimals);
    n += (size_t) snprintf(buf + n, n < sizeof(buf) ? sizeof(buf) - n : 0, "%s%s %s%s", n ? "  " : "", name, f, unit);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    long scale = 1;
    for (int i = 0; i < decimals; ++i) scale *= 10;
    if (nDbg < 6) dbg[nDbg++] = isnan(v) ? INT32_MIN : (int32_t) lround(v * scale);
  }
  void temperature(double c) {             // "T 25.31 C (77.6 F)"
    value("T", c, 2, " C");
    char f[16];
    formatFixed(f, sizeof(f), c * 9.0 / 5.0 + 32.0, 1);
    n += (size_t) snprintf(buf + n, n < sizeof(buf) ? sizeof(buf) - n : 0, " (%s F)", f);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  }
};

// The latest pressure as a Barometer, for a driver that has no Barometer
// interface of its own. NAN until the first reading.
struct ReadingBarometer : Barometer {
  float pa = NAN;
  float getPressure() const override { return pa; }
};

// ---- the sensors ------------------------------------------------------------
// One found on the bus, its driver inside. step() is the driver's main()
// (which sleeps); reading() builds the line for a new reading, or says
// there is none.
class Sensor {
public:
  const char *tag;                         // short name: the console, and the debug log's tag
  uint8_t addr;                            // I2C address, 0 for a UART sensor
  Sensor(const char *t, uint8_t a) : tag(t), addr(a) {}
  virtual ~Sensor() {}
  virtual void step(uint32_t nowMs) = 0;
  virtual bool reading(Line &l) = 0;
  virtual int state() = 0;
  virtual uint32_t periodMs() const = 0;  // usual time between readings
  virtual const Barometer *barometer() { return nullptr; }  // station pressure, Pa
  // Console commands that only some sensors take
  virtual bool setTime(const DS1307::ds1307_time_t &) { return false; }
  virtual bool setAging(int8_t offset) { (void) offset; return false; }
  virtual bool setAltimeterSetting(float pa) { (void) pa; return false; }   // sea-level reference, Pa
  // A barometer's sea-level figure (Pa), after each reading
  virtual void onSeaLevel(float pa) { (void) pa; }
};

class Aht20 : public Sensor {
  xAHT20<I2C> d;
public:
  explicit Aht20(uint8_t a) : Sensor("aht20", a), d(&hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float t, h;
    d.getTempHumidity(&t, &h);
    l.temperature(t);
    l.value("H", h, 1, " %RH");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return aht20_period_ms; }
};

class Bme280 : public Sensor {
  BME280::xbme280<I2C> d;
  static BME280::bme280_param_t param() {
    BME280::bme280_param_t p;
    p._osrs_p_t = BME280::bme280_osrs_p_4x;
    p._osrs_t_t = BME280::bme280_osrs_t_1x;
    p._osrs_h_t = BME280::bme280_osrs_h_1x;
    p._mode_t = BME280::bme280_forced_mode;
    p._sb_t = BME280::bme280_sb_1000;      // normal mode only
    p._filter_t = BME280::bme280_filter_off;
    return p;
  }
public:
  explicit Bme280(uint8_t a) : Sensor("bme280", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    double t, p, h;
    d.GetTemperature(&t);
    d.GetPressure(&p);
    d.GetHumidity(&h);
    l.temperature(t);
    l.value("H", h, 1, " %RH");
    l.value("P", p / 100.0, 2, " hPa");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 250; }   // forced mode: about five a second
  const Barometer *barometer() override { return &d; }
};

class Bme680 : public Sensor {
  BME680::xbme680<I2C> d;
  ReadingBarometer baro;
  static BME680::bme680_param_t param() {
    // Every 3 s: the gas heater (320 C for 150 ms) warms the chip less
    BME680::bme680_param_t p;
    p.bus = BME680::bme680_i2c;
    p.os_temperature = BME680::bme680_os_2x;
    p.os_pressure = BME680::bme680_os_4x;
    p.os_humidity = BME680::bme680_os_1x;
    p.filter = BME680::bme680_filter_3;
    p.heater_c = 320;
    p.heater_ms = 150;
    p.ambient_c = 25;
    p.period_ms = 3000;
    return p;
  }
public:
  explicit Bme680(uint8_t a) : Sensor("bme680", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    BME680::bme680_reading_t r;
    d.getReading(&r);
    baro.pa = r.pressure;
    l.temperature(r.temperature);
    l.value("H", r.humidity, 1, " %RH");
    l.value("P", r.pressure / 100.0, 2, " hPa");
    l.value("gas", r.gas_resistance / 1000.0, 1, " kOhm");
    if (!r.gas_valid) l.text(" (no gas reading)");
    else if (!r.heat_stable) l.text(" (heater not stable)");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 3000; }
  const Barometer *barometer() override { return &baro; }
};

class Bmp280 : public Sensor {
  BMP280::xbmp280<I2C> d;
  ReadingBarometer baro;
  static BMP280::bmp280_param_t param() {
    BMP280::bmp280_param_t p;
    p._osrs_p_t = BMP280::bmp280_osrs_p_4x;
    p._osrs_t_t = BMP280::bmp280_osrs_t_1x;
    p._mode_t = BMP280::bmp280_forced_mode;
    p._t_sb = BMP280::bmp280_sb_1000;      // normal mode only
    p._filter_t = BMP280::bmp280_filter_off;
    return p;
  }
public:
  explicit Bmp280(uint8_t a) : Sensor("bmp280", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    double t, p;
    d.GetTemperature(&t);
    d.GetPressure(&p);
    baro.pa = (float) p;
    l.temperature(t);
    l.value("P", p / 100.0, 2, " hPa");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return BMP280::bmp280_period_ms; }
  const Barometer *barometer() override { return &baro; }
};

class Bmp390 : public Sensor {
  BMP390::xbmp390<I2C> d;
  static BMP390::bmp390_param_t param() {
    BMP390::bmp390_param_t p;
    p.osr_pressure = BMP390::bmp390_osr_8x;
    p.osr_temperature = BMP390::bmp390_osr_1x;
    p.iir = BMP390::bmp390_iir_off;
    p.period_ms = 1000;
    return p;
  }
public:
  explicit Bmp390(uint8_t a, bool bmp388) : Sensor(bmp388 ? "bmp388" : "bmp390", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    double t, p;
    d.GetTemperature(&t);
    d.GetPressure(&p);
    l.temperature(t);
    l.value("P", p / 100.0, 2, " hPa");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
  const Barometer *barometer() override { return &d; }
};

class Hmc6352 : public Sensor {
  HMC6352::xhmc6352<I2C> d;
  static HMC6352::hmc6352_param_t param() {
    HMC6352::hmc6352_param_t p;
    p._op_mode = HMC6352::hmc6352_standby;  // a get data command each period
    p.rate = HMC6352::hmc6352_1hz;          // continuous mode only
    p.per = HMC6352::hmc6352_per_en;
    p.output_mode = HMC6352::hmc6352_heading_mode;
    return p;
  }
public:
  explicit Hmc6352(uint8_t a) : Sensor("hmc6352", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float heading;
    d.getHeading(&heading);
    l.value("heading", heading, 1, " deg");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return HMC6352::hmc6352_period_ms; }
};

class Htu21df : public Sensor {
  xHTU21DF<I2C> d;
public:
  explicit Htu21df(uint8_t a) : Sensor("htu21df", a), d(&hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float t, h;
    d.readTemperature(&t);
    d.readHumidity(&h);
    l.temperature(t);
    l.value("H", h, 1, " %RH");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return htu21df_period_ms; }
};

class Si7021s : public Sensor {
  xSi7021<I2C> d;
  static Si7021_param_t param() {
    Si7021_param_t p;
    p.enable = Si7021_heater_disable;
    p.heater_current = 0;
    p.res = Si7021_RH12_T14;
    return p;
  }
public:
  explicit Si7021s(uint8_t a) : Sensor("si7021", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float t, h;
    d.readTempHumidity(&t, &h);
    l.temperature(t);
    l.value("H", h, 1, " %RH");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return Si7021_period_ms; }
};

class Lps35hw : public Sensor {
  LPS35HW::xlps35hw<I2C> d;
  ReadingBarometer baro;
  static LPS35HW::lps35hw_param_t param() {
    LPS35HW::lps35hw_param_t p;
    memset(&p, 0, sizeof(p));
    p.odr = LPS35HW::lps35hw_odr_0hz;      // one shot each period
    p.en_lpfp = false;
    p.lpfp_cfg = false;
    p.fifo_mode = LPS35HW::fifo_bypass_mode;
    return p;
  }
public:
  explicit Lps35hw(uint8_t a) : Sensor("lps35hw", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float t, p;
    d.tempPressure(&t, &p);                // degrees C, hPa
    baro.pa = p * 100.0f;
    l.temperature(t);
    l.value("P", p, 2, " hPa");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return LPS35HW::lps35hw_period_ms; }
  const Barometer *barometer() override { return &baro; }
};

// The MPL3115A2's cell on the console's GUI, once one is found (the home
// screen draws it)
static std::atomic<idisplay::xCell *> g_mplCell{nullptr};

// A label's value from a float another thread stores (NAN shows "--")
static double loadFloat(const void *v)
{
  return static_cast<const std::atomic<float> *>(v)->load(std::memory_order_relaxed);
}

// Both modes each period: the pressure (a barometer measurement), then the
// chip's own altitude (an altimeter measurement) from the sea-level
// reference the a command gives it; 29.92 inHg (pressure altitude) until then.
// Its readings also go to its cell on the console's GUI: the sensor thread
// stores them here and the GUI task reads them (never the driver, whose
// getters clear its new-data flag).
class Mpl3115a2 : public Sensor {
  mpl3115a2::xmpl3115a2<I2C> d;
  ReadingBarometer baro;
  std::atomic<float> tempC{NAN}, pressHpa{NAN}, seaInHg{NAN}, altM{NAN}, refInHg{NAN};
  idisplay::xLabel labels[5];
  idisplay::xCell cell;
  static mpl3115a2::mpl3115a2_param_t param() {
    mpl3115a2::mpl3115a2_param_t p;
    p.mode = mpl3115a2::mpl3115a2_both_mode;
    p.oversample_ratio = mpl3115a2::MPL3115A2_CTRL_REG1_OS32;
    return p;
  }
public:
  explicit Mpl3115a2(uint8_t a)
    : Sensor("mpl3115a2", a), d(param(), &hi2c1, a, i2c1_mutexHandle),
      labels{idisplay::xLabel::fn("Temperature", loadFloat, &tempC, 2, " \x7F" "C"),
             idisplay::xLabel::fn("Pressure", loadFloat, &pressHpa, 2, " hPa"),
             idisplay::xLabel::fn("Sea level", loadFloat, &seaInHg, 2, " inHg"),
             idisplay::xLabel::fn("Altitude", loadFloat, &altM, 1, " m"),
             idisplay::xLabel::fn("Altimeter setting", loadFloat, &refInHg, 2, " inHg")},
      cell(0, 0, CONSOLE_COLS, 6, labels, 5, "MPL3115A2 (I2C1)") {
    d.setDebugTag(tag);
    g_mplCell.store(&cell, std::memory_order_release);
    guiRefresh();
  }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    const float t = d.getTemp(), p = d.getPressure(), a = d.getAltitude();   // degrees C, hPa, m
    baro.pa = p * 100.0f;
    tempC.store(t, std::memory_order_relaxed);
    pressHpa.store(p, std::memory_order_relaxed);
    altM.store(a, std::memory_order_relaxed);
    refInHg.store(baro::paToInHg(d.seaLevelPressure()), std::memory_order_relaxed);
    l.temperature(t);
    l.value("P", p, 2, " hPa");
    l.value("alt", a, 1, " m");
    char ref[16], ft[40];
    formatFixed(ref, sizeof(ref), baro::paToInHg(d.seaLevelPressure()), 2);
    if (isnan(a))
      snprintf(ft, sizeof(ft), " at %s inHg", ref);
    else
      snprintf(ft, sizeof(ft), " (%ld ft) at %s inHg", lroundf(a / M_PER_FT), ref);
    l.text(ft);
    return true;
  }
  bool setAltimeterSetting(float pa) override { return d.setSeaLevelPressure(pa); }
  void onSeaLevel(float pa) override { seaInHg.store(baro::paToInHg(pa), std::memory_order_relaxed); }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return mpl3115a2::mpl3115a2_period_ms; }
  const Barometer *barometer() override { return &baro; }
};

class Sht31s : public Sensor {
  xSHT31<I2C> d;
  static SHT31_param_t param() {
    SHT31_param_t p;
    p.periodic_mode = true;
    p.heater_enable = false;
    p.sample_rate = _1mps_high_Res;
    return p;
  }
public:
  explicit Sht31s(uint8_t a) : Sensor("sht31", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float t, h;
    d.GetTemperature(&t);
    d.GetHumidity(&h);
    l.temperature(t);
    l.value("H", h, 1, " %RH");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
};

class Ds1307 : public Sensor {
  DS1307::xds1307<I2C> d;
  bool lastRunning = true;
public:
  explicit Ds1307(uint8_t a) : Sensor("ds1307", a), d(DS1307::ds1307_param_t(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    DS1307::ds1307_time_t t;
    static const char *const days[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    if (!d.getTime(&t)) {
      l.text("no valid time (set it with t)");
      return true;
    }
    char s[48];
    snprintf(s, sizeof(s), "%04u-%02u-%02u %02u:%02u:%02u %s", t.year, t.month, t.day, t.hour, t.minute, t.second,
             t.weekday < 7 ? days[t.weekday] : "?");
    l.text(s);
    l.dbg[l.nDbg++] = (int32_t) DS1307::ds1307_to_unix(t);
    if (!d.running()) l.text(" (oscillator stopped: set the time with t to start it)");
    lastRunning = d.running();
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
  bool setTime(const DS1307::ds1307_time_t &t) override { return d.setTime(t); }
};

// The DS3231 reads all its registers once a second: the time, and the
// temperature its crystal compensation uses (a conversion every 64 s).
class Ds3231 : public Sensor {
  DS3231::xds3231<I2C> d;
public:
  explicit Ds3231(uint8_t a) : Sensor("ds3231", a), d(DS3231::ds3231_param_t(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    rtc::rtc_time_t t;
    static const char *const days[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    if (!d.getTime(&t)) {
      l.text("no valid time (set it with t)");
      return true;
    }
    char s[48];
    snprintf(s, sizeof(s), "%04u-%02u-%02u %02u:%02u:%02u %s", t.year, t.month, t.day, t.hour, t.minute, t.second,
             t.weekday < 7 ? days[t.weekday] : "?");
    l.text(s);
    l.dbg[l.nDbg++] = (int32_t) rtc::toUnix(t);
    l.value("T", d.getTemperature(), 2, " C");
    int8_t aging;
    if (d.getAging(&aging) && aging != 0) {
      char g[24];
      snprintf(g, sizeof(g), "  aging %d", aging);
      l.text(g);
    }
    if (!d.timeValid()) l.text("  (OSF: the oscillator stopped; set the time with t)");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
  bool setTime(const DS1307::ds1307_time_t &t) override { return d.setTime(t); }
  bool setAging(int8_t offset) override { d.setAging(offset); return true; }
};

class Mmc56x3 : public Sensor {
  MMC56X3::xmmc56x3<I2C> d;
  static MMC56X3::mmc56x3_param_t param() {
    MMC56X3::mmc56x3_param_t p;
    p.mode = MMC56X3::mmc56x3_one_shot;
    p.bandwidth = MMC56X3::mmc56x3_bw_6_6ms;
    p.odr = 0;                             // continuous mode only
    p.auto_sr = true;
    p.read_temperature = true;
    p.period_ms = 1000;
    return p;
  }
public:
  explicit Mmc56x3(uint8_t a) : Sensor("mmc56x3", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float x, y, z, t;
    d.getField(&x, &y, &z);
    d.getTemperature(&t);
    l.value("X", x, 2, " uT");
    l.value("Y", y, 2, " uT");
    l.value("Z", z, 2, " uT");
    l.value("|B|", sqrt((double) x * x + (double) y * y + (double) z * z), 2, " uT");
    l.temperature(t);
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
};

class Lsm303Accel : public Sensor {
  LSM303DLHC::xlsm303dlhc_accel<I2C> d;
  static LSM303DLHC::lsm303dlhc_accel_param_t param() {
    LSM303DLHC::lsm303dlhc_accel_param_t p;
    p.odr = LSM303DLHC::lsm303dlhc_accel_10hz;
    p.scale = LSM303DLHC::lsm303dlhc_accel_2g;
    p.period_ms = 1000;
    return p;
  }
public:
  explicit Lsm303Accel(uint8_t a) : Sensor("lsm303-a", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float x, y, z;
    d.getAccel(&x, &y, &z);
    l.value("X", x, 2, " m/s2");
    l.value("Y", y, 2, " m/s2");
    l.value("Z", z, 2, " m/s2");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
};

class Lsm303Mag : public Sensor {
  LSM303DLHC::xlsm303dlhc_mag<I2C> d;
  static LSM303DLHC::lsm303dlhc_mag_param_t param() {
    LSM303DLHC::lsm303dlhc_mag_param_t p;
    p.rate = LSM303DLHC::lsm303dlhc_mag_15hz;
    p.gain = LSM303DLHC::lsm303dlhc_mag_1_3g;
    p.period_ms = 1000;
    return p;
  }
public:
  explicit Lsm303Mag(uint8_t a) : Sensor("lsm303-m", a), d(param(), &hi2c1, a, i2c1_mutexHandle) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float x, y, z;
    d.getMag(&x, &y, &z);
    l.value("X", x, 2, " uT");
    l.value("Y", y, 2, " uT");
    l.value("Z", z, 2, " uT");
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
};

#if BRINGUP_USART1 == BRINGUP_USART1_PM25
class Pm25 : public Sensor {
  xPM25<Stm32HalUartTransport> d;
public:
  Pm25() : Sensor("pm25", 0), d(&huart1) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    PM25_AQI_Data a;
    if (!d.getData(&a)) return false;
    char s[160];
    snprintf(s, sizeof(s), "PM1.0 %u  PM2.5 %u  PM10 %u ug/m3 (env.) | particles/0.1 L: >0.3um %u >0.5um %u >1um %u >2.5um %u",
             a.pm10_env, a.pm25_env, a.pm100_env, a.particles_03um, a.particles_05um, a.particles_10um, a.particles_25um);
    l.text(s);
    l.dbg[l.nDbg++] = a.pm10_env;
    l.dbg[l.nDbg++] = a.pm25_env;
    l.dbg[l.nDbg++] = a.pm100_env;
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
};
#endif

#if BRINGUP_USART1 == BRINGUP_USART1_DS18B20
static osMutexId_t g_oneWireMutex;
class Ds18b20 : public Sensor {
  DS18B20::xds18b20<Stm32HalOneWireTransport> d;
public:
  Ds18b20() : Sensor("ds18b20", 0), d(&huart1, g_oneWireMutex) { d.setDebugTag(tag); }
  void step(uint32_t now) override { d.main(now); }
  bool reading(Line &l) override {
    if (!d.newData()) return false;
    float t;
    d.getTemp(&t);
    l.temperature(t);
    return true;
  }
  int state() override { return (int) d.state(); }
  uint32_t periodMs() const override { return 1000; }
};
#endif

// ---- running them ---------------------------------------------------------
// One per sensor started, each with its own thread. The console's
// commands reach the threads through these: a new elevation, and a
// calibration or a clear-offset numbered so each thread does it once.
struct Running {
  Sensor *s = nullptr;
  SeaLevelBarometer *slp = nullptr;        // barometers only
  std::atomic<uint32_t> readings{0}, lastTick{0};
  uint32_t startTick = 0;
  bool reportedStale = false;              // sensorTask's, so a silent sensor is reported once
  uint32_t calSeen = 0, clearSeen = 0;     // the thread's
  uint32_t altSeen = 0, agingSeen = 0;
};
static Running g_running[MAX_SENSORS];
static int g_nRunning = 0;
static std::atomic<float> g_elevationM{STATION_ELEVATION_M};
static std::atomic<float> g_calInHg{NAN};
static std::atomic<uint32_t> g_calGen{0}, g_clearGen{0};
static std::atomic<float> g_altInHg{NAN};  // the a command's altimeter setting
static std::atomic<uint32_t> g_altGen{0};
static std::atomic<int> g_aging{0};        // the g command's aging offset
static std::atomic<uint32_t> g_agingGen{0};
static std::atomic<bool> g_timeReq{false};
static DS1307::ds1307_time_t g_time;      // valid while g_timeReq is set

static void sensorThread(void *arg)
{
  Running *r = static_cast<Running *>(arg);
  Sensor *s = r->s;
  for(;;)
  {
    s->step(osKernelGetTickCount());       // the driver; sleeps inside, up to 100 ms
    osDelay(1);                            // and one tick a cycle, whatever state it is in
    if (r->slp != nullptr) {
      const float e = g_elevationM.load();
      if (e != r->slp->elevation())
        r->slp->setElevation(e);
      const uint32_t cal = g_calGen.load(), clr = g_clearGen.load();
      if (cal != r->calSeen) {
        r->calSeen = cal;
        char o[16];
        if (r->slp->calibrateTo(baro::inHgToPa(g_calInHg.load()))) {
          formatFixed(o, sizeof(o), r->slp->offset() / 100.0, 2);
          consolePrint("%-9s calibrated: offset %s hPa added to its station pressure\r\n", s->tag, o);
          DBG_EVENT(s->tag, "cal", (int32_t) lroundf(r->slp->offset()), (int32_t) lroundf(r->slp->elevation()));
        } else {
          consolePrint("%-9s not calibrated: no pressure reading yet\r\n", s->tag);
        }
      }
      if (clr != r->clearSeen) {
        r->clearSeen = clr;
        r->slp->setOffset(0.0f);
      }
    }
    if (g_timeReq.load(std::memory_order_acquire) && s->setTime(g_time)) {
      g_timeReq.store(false, std::memory_order_release);
      consolePrint("%-9s time set\r\n", s->tag);
    }
    const uint32_t gg = g_agingGen.load();
    if (gg != r->agingSeen) {
      r->agingSeen = gg;
      if (s->setAging((int8_t) g_aging.load()))
        consolePrint("%-9s aging offset %d: written, then a temperature conversion applies it\r\n", s->tag, g_aging.load());
    }
    const uint32_t ag = g_altGen.load();
    if (ag != r->altSeen) {
      r->altSeen = ag;
      if (s->setAltimeterSetting(baro::inHgToPa(g_altInHg.load())))
        consolePrint("%-9s altimeter setting taken: its altitude from the next reading\r\n", s->tag);
    }

    Line l;
    if (!s->reading(l))
      continue;
    if (r->slp != nullptr) {
      const float alt = r->slp->getPressure();
      s->onSeaLevel(alt);
      char sl[16], sh[16], el[16], of[16];
      formatFixed(sl, sizeof(sl), baro::paToInHg(alt), 2);
      formatFixed(sh, sizeof(sh), alt / 100.0, 2);
      formatFixed(el, sizeof(el), r->slp->elevation(), 0);
      formatFixed(of, sizeof(of), r->slp->offset() / 100.0, 2);
      char t[112];
      snprintf(t, sizeof(t), " = %s inHg / %s hPa at sea level (%s m, cal %s hPa)", sl, sh, el, of);
      l.text(t);
      if (l.nDbg < 6) l.dbg[l.nDbg++] = (int32_t) lroundf(alt);
    }
    const uint32_t now = osKernelGetTickCount();
    r->readings.fetch_add(1);
    r->lastTick.store(now);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    consolePrint("%-9s %s | %lu\r\n", s->tag, l.buf, (unsigned long) r->readings.load());
    switch (l.nDbg) {                      // the values as integers (x10^decimals), the sea-level Pa last
    case 0: DBG_EVENT(s->tag, "reading"); break;
    case 1: DBG_EVENT(s->tag, "reading", l.dbg[0]); break;
    case 2: DBG_EVENT(s->tag, "reading", l.dbg[0], l.dbg[1]); break;
    case 3: DBG_EVENT(s->tag, "reading", l.dbg[0], l.dbg[1], l.dbg[2]); break;
    case 4: DBG_EVENT(s->tag, "reading", l.dbg[0], l.dbg[1], l.dbg[2], l.dbg[3]); break;
    case 5: DBG_EVENT(s->tag, "reading", l.dbg[0], l.dbg[1], l.dbg[2], l.dbg[3], l.dbg[4]); break;
    default: DBG_EVENT(s->tag, "reading", l.dbg[0], l.dbg[1], l.dbg[2], l.dbg[3], l.dbg[4], l.dbg[5]); break;
    }
  }
}

// A sea-level wrapper for each barometer, from a fixed pool
static SeaLevelBarometer *seaLevelFor(const Barometer &b)
{
  // SeaLevelBarometer holds a reference, so it is built in place here
  alignas(SeaLevelBarometer) static uint8_t pool[MAX_SENSORS][sizeof(SeaLevelBarometer)];
  static int used = 0;
  if (used >= MAX_SENSORS)
    return nullptr;
  return new (pool[used++]) SeaLevelBarometer(b, g_elevationM.load());
}

// Starts a thread for a sensor built by the caller (a function-local
// static, so each kind is built at most once and never freed)
static void start(Sensor &s)
{
  if (g_nRunning >= MAX_SENSORS)
    return;
  Running &r = g_running[g_nRunning];
  r.s = &s;
  if (s.barometer() != nullptr)
    r.slp = seaLevelFor(*s.barometer());
  r.startTick = osKernelGetTickCount();
  r.calSeen = g_calGen.load();
  r.clearSeen = g_clearGen.load();
  osThreadAttr_t attr;
  memset(&attr, 0, sizeof(attr));
  attr.name = s.tag;
  attr.stack_size = SENSOR_STACK_WORDS * 4;
  attr.priority = osPriorityNormal;
  if (osThreadNew(sensorThread, &r, &attr) == nullptr) {
    consolePrint("%-9s not started: no room for its thread (FreeRTOS heap)\r\n", s.tag);
    DBG_FAULT("app", "no-thread", (int32_t) g_nRunning);
    return;
  }
  ++g_nRunning;
  if (s.addr != 0)
    consolePrint("%-9s at 0x%02X: started\r\n", s.tag, s.addr);
  else
    consolePrint("%-9s on USART1: started\r\n", s.tag);
  DBG_EVENT("app", "sensor", (int32_t) s.addr, (int32_t) g_nRunning);
}

// ---- the bus scan ---------------------------------------------------------
// Blocking HAL calls under i2c1_mutex, a few ms each. Called by sensorTask,
// which has no transfer of its own outstanding; the sensor threads wait
// for the mutex meanwhile (a transfer of theirs in flight only delays it).
static bool readReg(uint8_t addr, uint8_t reg, uint8_t *v, uint16_t n = 1)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, v, n, 10) == HAL_OK;
}

// A DS3231 or a DS1307 at 0x68? Reads only. A DS3231's registers 0x0E..0x12
// show what only it has (ds3231_probe_regs()), and its register pointer
// wraps from 0x12 to the seconds, where a DS1307 has RAM at 0x13: two bytes
// read from 0x12 end with the seconds, read just before or after (they may
// tick between). Seconds of 00 can't tell a zeroed RAM from a DS3231, so
// that is tried again. Anything else is a DS1307, as before. (The wrap is
// general knowledge, not yet checked against the datasheet.)
static bool isDs3231(void)
{
  for (int tries = 0; tries < 3; ++tries) {
    uint8_t r[5], s0, two[2], s1;
    if (!readReg(0x68, 0x0E, r, 5) || !DS3231::ds3231_probe_regs(r)) return false;
    if (!readReg(0x68, 0x00, &s0) || !readReg(0x68, 0x12, two, 2) || !readReg(0x68, 0x00, &s1)) return false;
    if (two[1] != s0 && two[1] != s1) return false;
    if (two[1] != 0x00) return true;
    osDelay(600);                          // :00, ambiguous: again (the third try is past :00)
  }
  return false;
}

// The Si70xx's electronic ID, second part (command 0xFC 0xC9): its first
// byte is SNB_3, the part number (0x0D Si7013, 0x14 Si7020, 0x15 Si7021;
// Silicon Labs' datasheet as general knowledge has it). An HTU21D(F) is
// expected to answer something else or not at all.
static uint8_t si70xxPart(void)
{
  uint8_t cmd[2] = {0xFC, 0xC9}, id[6] = {0};
  if (HAL_I2C_Master_Transmit(&hi2c1, 0x40 << 1, cmd, 2, 10) != HAL_OK) return 0;
  if (HAL_I2C_Master_Receive(&hi2c1, 0x40 << 1, id, 6, 10) != HAL_OK) return 0;
  return id[0];
}

// What one scan found, decided while holding the bus, started after
enum Found : uint8_t {
  F_NONE, F_AHT20, F_BME280, F_BME680, F_BMP280, F_BMP390, F_BMP388, F_HMC6352, F_HTU21DF, F_SI7021,
  F_LPS35HW, F_MPL3115A2, F_SHT31, F_DS1307, F_MMC56X3, F_LSM303_A, F_LSM303_M, F_DS3231
};
struct Hit { Found what; uint8_t addr; };

static int scanBus(Hit *hits, int maxHits)
{
  // Static, not on sensorTask's stack: only sensorTask scans
  static char list[200];
  static char notes[6][96];
  static bool ack[128];
  int nHits = 0;
  size_t n = 0;
  list[0] = 0;
  int nNotes = 0;
  auto add = [&](Found f, uint8_t a) { if (nHits < maxHits) hits[nHits++] = Hit{f, a}; };
  auto note = [&](const char *fmt, uint8_t a, unsigned v1, unsigned v2) {
    if (nNotes < 6) snprintf(notes[nNotes++], sizeof(notes[0]), fmt, a, v1, v2);
  };
  if (osMutexAcquire(i2c1_mutexHandle, 500) != osOK) {
    consolePrint("I2C bus busy: no scan\r\n");
    return 0;
  }
  memset(ack, 0, sizeof(ack));
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    ack[a] = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 2) == HAL_OK;
    if (ack[a] && n + 6 < sizeof(list))
      n += (size_t) snprintf(list + n, sizeof(list) - n, " 0x%02X", a);
  }
  uint8_t v[3] = {0, 0, 0};
  if (ack[0x19]) add(F_LSM303_A, 0x19);
  if (ack[0x1E]) {
    if (readReg(0x1E, LSM303_MAG_ID_REG, v, 3) && v[0] == 'H' && v[1] == '4' && v[2] == '3') add(F_LSM303_M, 0x1E);
    else note("  0x%02X: not an LSM303DLHC magnetometer (ID %02X..)\r\n", 0x1E, v[0], 0);
  }
  if (ack[0x21]) add(F_HMC6352, 0x21);
  if (ack[0x30]) {
    if (readReg(0x30, MMC56X3_ID_REG, v) && v[0] == MMC56X3_PRODUCT_ID) add(F_MMC56X3, 0x30);
    else note("  0x%02X: not an MMC56x3 (product ID 0x%02X)\r\n", 0x30, v[0], 0);
  }
  if (ack[0x38]) add(F_AHT20, 0x38);
  if (ack[0x40]) {
    const uint8_t part = si70xxPart();
    if (part == 0x15 || part == 0x14 || part == 0x0D) add(F_SI7021, 0x40);
    else add(F_HTU21DF, 0x40);
  }
  if (ack[0x44]) add(F_SHT31, 0x44);
  else if (ack[0x45]) add(F_SHT31, 0x45);
  for (uint8_t a = 0x5C; a <= 0x5D; ++a)
    if (ack[a]) {
      if (readReg(a, LPS35HW_ID_REG, v) && v[0] == LPS35HW_WHO_AM_I) add(F_LPS35HW, a);
      else note("  0x%02X: not an LPS35HW (WHO_AM_I 0x%02X)\r\n", a, v[0], 0);
    }
  if (ack[0x60]) {
    if (readReg(0x60, MPL3115A2_ID_REG, v) && v[0] == mpl3115a2_who_am_i) add(F_MPL3115A2, 0x60);
    else note("  0x%02X: not an MPL3115A2 (WHO_AM_I 0x%02X)\r\n", 0x60, v[0], 0);
  }
  if (ack[0x68]) add(isDs3231() ? F_DS3231 : F_DS1307, 0x68);
  for (uint8_t a = 0x76; a <= 0x77; ++a) {
    uint8_t d0 = 0, i0 = 0;
    if (!ack[a] || !readReg(a, BOSCH_ID_REG, &d0) || !readReg(a, BMP3_ID_REG, &i0))
      continue;
    DBG_EVENT("app", "chip-id", (int32_t) a, (int32_t) i0, (int32_t) d0);
    if (d0 == ID_BME280) add(F_BME280, a);
    else if (d0 == ID_BMP280) add(F_BMP280, a);
    else if (d0 == ID_BME680) add(F_BME680, a);
    else if (i0 == ID_BMP390) add(F_BMP390, a);
    else if (i0 == ID_BMP388) add(F_BMP388, a);
    else note("  0x%02X: chip IDs 0x%02X (0xD0), 0x%02X (0x00): unknown; two chips at this address?\r\n", a, d0, i0);
  }
  osMutexRelease(i2c1_mutexHandle);

  consolePrint("I2C scan:%s\r\n", n ? list : " nothing answered (check SDA D0, SCL D1, 3V3, GND, pull-ups)");
  for (int i = 0; i < nNotes; ++i)
    consolePrint("%s", notes[i]);
  static const char *const names[] = {"", "AHT20", "BME280", "BME680", "BMP280", "BMP390", "BMP388", "HMC6352",
                                      "HTU21D(F)", "Si70xx", "LPS35HW", "MPL3115A2", "SHT31", "DS1307 (assumed)",
                                      "MMC56x3", "LSM303DLHC accelerometer (assumed)", "LSM303DLHC magnetometer",
                                      "DS3231"};
  for (int i = 0; i < nHits; ++i)
    consolePrint("  0x%02X: %s\r\n", hits[i].addr, names[hits[i].what]);
  return nHits;
}

// One of each kind: the first found. A function-local static is built the
// first time its line runs, so only what was found is ever constructed.
static void startFound(const Hit &h, bool *started)
{
  if (started[h.what]) {
    consolePrint("  0x%02X: a second one; only the first of each kind is driven\r\n", h.addr);
    return;
  }
  started[h.what] = true;
  switch (h.what) {
  case F_AHT20:     { static Aht20 s(h.addr); start(s); break; }
  case F_BME280:    { static Bme280 s(h.addr); start(s); break; }
  case F_BME680:    { static Bme680 s(h.addr); start(s); break; }
  case F_BMP280:    { static Bmp280 s(h.addr); start(s); break; }
  case F_BMP390:    { static Bmp390 s(h.addr, false); start(s); break; }
  case F_BMP388:    { static Bmp390 s(h.addr, true); start(s); break; }
  case F_HMC6352:   { static Hmc6352 s(h.addr); start(s); break; }
  case F_HTU21DF:   { static Htu21df s(h.addr); start(s); break; }
  case F_SI7021:    { static Si7021s s(h.addr); start(s); break; }
  case F_LPS35HW:   { static Lps35hw s(h.addr); start(s); break; }
  case F_MPL3115A2: { static Mpl3115a2 s(h.addr); start(s); break; }
  case F_SHT31:     { static Sht31s s(h.addr); start(s); break; }
  case F_DS1307:    { static Ds1307 s(h.addr); start(s); break; }
  case F_DS3231:    { static Ds3231 s(h.addr); start(s); break; }
  case F_MMC56X3:   { static Mmc56x3 s(h.addr); start(s); break; }
  case F_LSM303_A:  { static Lsm303Accel s(h.addr); start(s); break; }
  case F_LSM303_M:  { static Lsm303Mag s(h.addr); start(s); break; }
  default: break;
  }
}

// ---- I2C bus clear ----
//
// A reset of the STM32 (after flashing, or the button) doesn't reset the
// sensors. One that was sending a 0 when it came keeps SDA low, waiting for
// a clock that never comes, and the scan then finds nothing. busClear()
// clocks SCL by hand until that sensor lets go, then sends a STOP. Up to 9
// clocks (the standard bus clear, from general knowledge of the I2C
// specification, not checked against it here). Nothing is touched if SDA is
// already high.

static const int BUS_CLEAR_MAX_CLOCKS = 9;
static int g_busClear = 0;                 // clocks sent; -1 SDA still low after them; -2 SCL held low

// At least 5 us (a 100 kHz bus or slower): each pass of the loop takes at
// least 4 cycles.
static void busClearHalfBit(void)
{
  const uint32_t n = SystemCoreClock / 1000000u * 5u / 4u;
  for (volatile uint32_t i = 0; i < n; ++i) {}
}

static const uint32_t BUS_SCL = GPIO_PIN_9, BUS_SDA = GPIO_PIN_10;   // PA9, PA10 (I2C1 in the .ioc)

// With I2C1 de-initialised: the pins as open drain GPIOs, SCL clocked until
// SDA is free, a START and a STOP, then the pins handed back to I2C1 by
// HAL_I2C_Init() (its MSP init sets their function again). Returns the
// clocks it took; -1 SDA still low after BUS_CLEAR_MAX_CLOCKS; -2 SCL held
// low, when clocking can't help.
static int busClearPins(void)
{
  const uint32_t SCL = BUS_SCL, SDA = BUS_SDA;
  // Open drain outputs, released (high), so they can still be read
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, SCL | SDA, GPIO_PIN_SET);
  GPIO_InitTypeDef g;
  memset(&g, 0, sizeof(g));
  g.Pin = SCL | SDA;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);
  busClearHalfBit();

  int result;
  if (HAL_GPIO_ReadPin(GPIOA, SCL) == GPIO_PIN_RESET) {
    result = -2;                           // something holds the clock: clocking can't help
  } else {
    int clocks = 0;
    while (clocks < BUS_CLEAR_MAX_CLOCKS && HAL_GPIO_ReadPin(GPIOA, SDA) == GPIO_PIN_RESET) {
      HAL_GPIO_WritePin(GPIOA, SCL, GPIO_PIN_RESET);
      busClearHalfBit();
      HAL_GPIO_WritePin(GPIOA, SCL, GPIO_PIN_SET);
      busClearHalfBit();
      ++clocks;
    }
    result = HAL_GPIO_ReadPin(GPIOA, SDA) == GPIO_PIN_SET ? clocks : -1;
    if (result >= 0) {
      // A START, then a STOP, with SCL still high: every sensor drops what
      // it was doing. SCL isn't taken low first, or a sensor still sending
      // could put its next bit on SDA.
      HAL_GPIO_WritePin(GPIOA, SDA, GPIO_PIN_RESET);
      busClearHalfBit();
      HAL_GPIO_WritePin(GPIOA, SDA, GPIO_PIN_SET);
      busClearHalfBit();
    }
  }

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
  return result;
}

// At start: only if SDA is low. A free bus is only read.
static int busClear(void)
{
  if (HAL_GPIO_ReadPin(GPIOA, BUS_SDA) == GPIO_PIN_SET)
    return 0;                              // the bus is free: leave I2C1 alone
  HAL_I2C_DeInit(&hi2c1);
  return busClearPins();
}

// ---- I2C1 recovery at run time ----
//
// On the board, a sensor misbehaving in the middle of a transfer has left
// I2C1 holding SCL low with its BUSY flag set, and every later transfer was
// refused at once until a reset. sensorTask checks the bus every
// BUS_CHECK_MS while holding the bus mutex (so no transfer of ours is in
// flight, and an idle bus has SCL and SDA high and BUSY clear). If it isn't
// idle, I2C1 is reset (its RCC reset too) and the bus cleared. The drivers
// see their transfers fail and start again by themselves.
//
// If the mutex can't be had for BUS_STALL_MS, a transfer never ended (no
// completion interrupt came): I2C1's interrupts are stopped and that
// transfer is ended as failed, so its driver lets go of the mutex.
//
// After a recovery that didn't leave the bus idle, the next waits
// BUS_RECOVER_BACKOFF_MS, doubling up to BUS_RECOVER_MAX_MS, so a dead chip
// holding SDA is reported now and then, not every half second.

static const uint32_t BUS_CHECK_MS = 500;
static const uint32_t BUS_STALL_MS = 2000;
static const uint32_t BUS_RECOVER_BACKOFF_MS = 5000;
static const uint32_t BUS_RECOVER_MAX_MS = 80000;

enum {                                     // why a recovery ran: app bus-recover's second value
  BUS_WHY_BUSY = 1,                        // I2C1's BUSY flag set with nothing in flight
  BUS_WHY_SCL = 2,                         // SCL low
  BUS_WHY_SDA = 4,                         // SDA low
  BUS_WHY_STALL = 8                        // a transfer that never ended
};

struct BusWatch {
  uint32_t lastCheck = 0;
  uint32_t lastFree = 0;                   // the last tick the mutex could be had
  uint32_t lastRecover = 0;
  uint32_t backoff = 0;                    // 0: none pending
  uint32_t recoveries = 0;
};
static BusWatch g_bus;

static int busNotIdle(void)
{
  int why = 0;
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) why |= BUS_WHY_BUSY;
  if (HAL_GPIO_ReadPin(GPIOA, BUS_SCL) == GPIO_PIN_RESET) why |= BUS_WHY_SCL;
  if (HAL_GPIO_ReadPin(GPIOA, BUS_SDA) == GPIO_PIN_RESET) why |= BUS_WHY_SDA;
  return why;
}

// With the bus mutex held
static void busRecover(int why)
{
  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  const int r = busClearPins();
  const int after = busNotIdle();
  ++g_bus.recoveries;
  DBG_FAULT("app", "bus-recover", (int32_t) r, (int32_t) why, (int32_t) after);
  consolePrint("I2C1 recovered (#%lu):%s%s%s%s",
               (unsigned long) g_bus.recoveries,
               why & BUS_WHY_STALL ? " a transfer never ended;" : "",
               why & BUS_WHY_BUSY ? " I2C1 busy;" : "",
               why & BUS_WHY_SCL ? " SCL low;" : "",
               why & BUS_WHY_SDA ? " SDA low;" : "");
  if (r >= 0)
    consolePrint(" %d clock(s), bus %s\r\n", r, after ? "still not free" : "free");
  else
    consolePrint(" %s still held low: check the wiring and power\r\n", r == -2 ? "SCL" : "SDA");
  g_bus.backoff = after == 0 ? 0
                : g_bus.backoff == 0 ? BUS_RECOVER_BACKOFF_MS
                : (g_bus.backoff * 2 > BUS_RECOVER_MAX_MS ? BUS_RECOVER_MAX_MS : g_bus.backoff * 2);
}

// From sensorTask's loop: at most every BUS_CHECK_MS
static void busWatch(uint32_t tick)
{
  if (tick - g_bus.lastCheck < BUS_CHECK_MS)
    return;
  g_bus.lastCheck = tick;
  const bool waiting = g_bus.backoff != 0 && tick - g_bus.lastRecover < g_bus.backoff;

  if (osMutexAcquire(i2c1_mutexHandle, 0) == osOK) {
    g_bus.lastFree = tick;
    const int why = busNotIdle();
    if (why == 0) {
      g_bus.backoff = 0;
    } else if (!waiting) {
      busRecover(why);
      g_bus.lastRecover = osKernelGetTickCount();
    }
    osMutexRelease(i2c1_mutexHandle);
    return;
  }
  if (tick - g_bus.lastFree < BUS_STALL_MS || waiting)
    return;

  // A transfer that never ended: end it as failed, with I2C1's interrupts
  // off so the interrupt can't end it at the same time
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  const int why = BUS_WHY_STALL | busNotIdle();
  HAL_I2C_DeInit(&hi2c1);
  I2CTransport::onTransferComplete(&hi2c1, /*failed=*/true);
  // Its driver lets go of the mutex when it next looks
  if (osMutexAcquire(i2c1_mutexHandle, 1000) == osOK) {
    busRecover(why);                       // HAL_I2C_Init() turns the interrupts on again
    osMutexRelease(i2c1_mutexHandle);
  } else {
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)    // try again next time
      Error_Handler();
    DBG_FAULT("app", "bus-stall", (int32_t) why);
  }
  g_bus.lastRecover = osKernelGetTickCount();
  g_bus.lastFree = g_bus.lastRecover;
  if (g_bus.backoff == 0)
    g_bus.backoff = BUS_RECOVER_BACKOFF_MS;
}

// From main(), after the peripherals are set up and before the scheduler
// starts: the I2C bus clear, then USART1 for what BRINGUP_USART1 says. For the debug log, lines
// queue in a ring from here on, and consoleTask's dbg::poll() sends them.
void app_init(void)
{
#if BRINGUP_USART1 == BRINGUP_USART1_DEBUG
  static Stm32HalUartTransport debugUart(&huart1);
  dbg::begin(&debugUart, Stm32DebugPort::port());
  DBG_EVENT("app", "start", (int32_t) HAL_GetREVID(), (int32_t) HAL_GetDEVID());
#elif BRINGUP_USART1 == BRINGUP_USART1_PM25
  // The PMS5003 sends at 9600 (Plantower's datasheet, as the driver has it)
  huart1.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
#elif BRINGUP_USART1 == BRINGUP_USART1_DS18B20
  // 1-Wire: TX (D5) open drain on the line, with RX (D4) linked to it so the
  // UART hears what it sends, and a 4.7k pull-up to 3V3. The transport sets
  // the baud rates itself.
  GPIO_InitTypeDef g;
  memset(&g, 0, sizeof(g));
  g.Pin = DBG_TX_Pin;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(DBG_TX_GPIO_Port, &g);
#endif
  g_logMutex = osMutexNew(nullptr);       // the console's log pane (before any task prints)
  if (g_logMutex != nullptr)
    g_logLines.setLock(logLock, logUnlock, nullptr);
  g_busClear = busClear();
  if (g_busClear != 0)
    DBG_FAULT("app", "bus-clear", (int32_t) g_busClear);
}

// Commands typed on the console
static void command(const char *cmd, bool *scanPending)
{
  consolePrint("> %s\r\n", cmd);
  DBG_EVENT("app", "cmd", (int32_t) cmd[0]);
  char *end = nullptr;
  if (cmd[0] == 's' && cmd[1] == 0) {
    *scanPending = true;
  } else if (cmd[0] == 'e' && cmd[1] == ' ') {
    const float v = strtof(cmd + 2, &end);
    const bool metres = end != nullptr && (*end == 'm' || *end == 'M');
    if (end == cmd + 2 || !(v > -500.0f && v < 30000.0f)) {
      consolePrint("e <feet> or e <metres>m, e.g. e 131 or e 40m\r\n");
    } else {
      g_elevationM.store(metres ? v : v * M_PER_FT);
      char m[16];
      formatFixed(m, sizeof(m), g_elevationM.load(), 1);
      consolePrint("elevation %s m (%ld ft); calibration offsets are kept: b again if they were set at another elevation\r\n",
                   m, lroundf(g_elevationM.load() / M_PER_FT));
    }
  } else if (cmd[0] == 'b' && cmd[1] == ' ') {
    const float inHg = strtof(cmd + 2, &end);
    if (end == cmd + 2 || !(inHg > 25.0f && inHg < 33.0f)) {
      consolePrint("b <inHg>, the sea-level pressure to match now, e.g. b 30.54\r\n");
    } else {
      int baros = 0;
      for (int i = 0; i < g_nRunning; ++i)
        if (g_running[i].slp != nullptr) ++baros;
      if (baros == 0) {
        consolePrint("no barometer running\r\n");
      } else {
        g_calInHg.store(inHg);
        g_calGen.fetch_add(1);            // each barometer's thread calibrates and says so
      }
    }
  } else if (cmd[0] == 'a' && cmd[1] == ' ') {
    const float inHg = strtof(cmd + 2, &end);
    bool any = false;
    for (int i = 0; i < g_nRunning; ++i)
      if (strcmp(g_running[i].s->tag, "mpl3115a2") == 0) any = true;
    if (end == cmd + 2 || !(inHg > 25.0f && inHg < 33.0f)) {
      consolePrint("a <inHg>, the altimeter setting (e.g. weather.gov's), e.g. a 30.36\r\n");
    } else if (!any) {
      consolePrint("no MPL3115A2 running\r\n");
    } else {
      g_altInHg.store(inHg);
      g_altGen.fetch_add(1);              // the MPL3115A2's thread takes it and says so
    }
  } else if (cmd[0] == 'g' && cmd[1] == ' ') {
    const long v = strtol(cmd + 2, &end, 10);
    bool any = false;
    for (int i = 0; i < g_nRunning; ++i)
      if (strcmp(g_running[i].s->tag, "ds3231") == 0) any = true;
    if (end == cmd + 2 || v < -128 || v > 127) {
      consolePrint("g <offset>, the DS3231's aging offset -128..127, e.g. g -5\r\n");
    } else if (!any) {
      consolePrint("no DS3231 running\r\n");
    } else {
      g_aging.store((int) v);
      g_agingGen.fetch_add(1);            // the DS3231's thread takes it and says so
    }
  } else if (cmd[0] == 'o' && cmd[1] == 0) {
    g_clearGen.fetch_add(1);
    consolePrint("calibration offsets cleared\r\n");
  } else if (cmd[0] == 't' && cmd[1] == ' ') {
    unsigned y, mo, d, h, mi, s;
    DS1307::ds1307_time_t t;
    memset(&t, 0, sizeof(t));
    if (sscanf(cmd + 2, "%u-%u-%u %u:%u:%u", &y, &mo, &d, &h, &mi, &s) != 6) {
      consolePrint("t YYYY-MM-DD HH:MM:SS, e.g. t 2026-10-19 14:05:00\r\n");
    } else {
      t.year = (uint16_t) y; t.month = (uint8_t) mo; t.day = (uint8_t) d;
      t.hour = (uint8_t) h; t.minute = (uint8_t) mi; t.second = (uint8_t) s;
      t.weekday = DS1307::ds1307_weekday(t.year, t.month, t.day);
      if (!DS1307::ds1307_valid(t)) {
        consolePrint("not a valid date and time in 2000..2099\r\n");
      } else if (g_timeReq.load()) {
        consolePrint("the last time set hasn't been taken yet (is there a DS1307 or DS3231?)\r\n");
      } else {
        g_time = t;
        g_timeReq.store(true, std::memory_order_release);   // the RTC's thread takes it
      }
    }
  } else if ((cmd[0] == 'h' || cmd[0] == '?') && cmd[1] == 0) {
    printHelp();
  } else {
    consolePrint("unknown command; h for help\r\n");
  }
}

/**
  * @brief  sensorTask's body (main.c's StartSensorTask calls it): scans the
  *         bus until something answers, starts a thread for each sensor
  *         found, then takes the console's commands and reports sensors
  *         that have gone quiet.
  */
void app_sensor_task(void *argument)
{
  (void) argument;
  printHelp();
  if (g_busClear > 0)
    consolePrint("I2C1: SDA was held low at start; %d clock(s) freed it\r\n", g_busClear);
  else if (g_busClear == -1)
    consolePrint("I2C1: SDA still held low after %d clocks: check the wiring and power\r\n", BUS_CLEAR_MAX_CLOCKS);
  else if (g_busClear == -2)
    consolePrint("I2C1: SCL held low at start: check the wiring and power\r\n");
  bool started[F_LSM303_M + 1];
  memset(started, 0, sizeof(started));
  g_bus.lastCheck = g_bus.lastFree = osKernelGetTickCount();

#if BRINGUP_USART1 == BRINGUP_USART1_PM25
  { static Pm25 s; start(s); }
#elif BRINGUP_USART1 == BRINGUP_USART1_DS18B20
  g_oneWireMutex = osMutexNew(nullptr);
  { static Ds18b20 s; start(s); }
#endif

  Hit hits[16];
  int nHits;
  while ((nHits = scanBus(hits, 16)) == 0 && g_nRunning == 0) {
    consolePrint("no sensor recognised on I2C1; scanning again in %lu s\r\n", (unsigned long)(SCAN_RETRY_MS / 1000));
    osDelay(SCAN_RETRY_MS);
    busWatch(osKernelGetTickCount());      // a stuck bus would hide every sensor
  }
  for (int i = 0; i < nHits; ++i)
    startFound(hits[i], started);
  consolePrint("%d sensor%s running\r\n\r\n", g_nRunning, g_nRunning == 1 ? "" : "s");

  bool scanPending = false;
  uint32_t lastCheck = osKernelGetTickCount();
  for(;;)
  {
    osDelay(50);
    if (g_cmdReady.load(std::memory_order_acquire)) {
      char cmd[CONSOLE_LINE];
      strcpy(cmd, g_cmd);
      g_cmdReady.store(false, std::memory_order_release);
      command(cmd, &scanPending);
    }
    if (scanPending) {
      scanPending = false;
      (void) scanBus(hits, 16);            // reports only: sensors start at the first scan
    }
    busWatch(osKernelGetTickCount());
    // Once a second: a sensor with no reading for two periods and 2 s more
    // is reported once (and again after it has come back and gone again)
    const uint32_t tick = osKernelGetTickCount();
    if (tick - lastCheck >= 1000) {
      lastCheck = tick;
      for (int i = 0; i < g_nRunning; ++i) {
        Running &r = g_running[i];
        const uint32_t n = r.readings.load();
        const uint32_t since = tick - (n == 0 ? r.startTick : r.lastTick.load());
        const bool stale = since > 2 * r.s->periodMs() + 2000;
        if (stale && !r.reportedStale) {
          r.reportedStale = true;
          consolePrint("%-9s no reading %s (driver state %d)\r\n", r.s->tag, n == 0 ? "yet" : "lately", r.s->state());
          DBG_EVENT("app", "silent", (int32_t) i, (int32_t) r.s->state());
        } else if (!stale) {
          r.reportedStale = false;
        }
      }
    }
  }
}

// ---- the console's GUI ------------------------------------------------------
// The home screen: the MPL3115A2's cell once one is found, and the command
// line being typed on the bottom row. Drawn by the GUI task.
class HomeScreen : public idisplay::xScreen {
public:
  void onKey(idisplay::xKey key, idisplay::xKeyAction action, idisplay::xNavigator &nav) override {
    (void) action; (void) nav;
    idisplay::xCell *c = g_mplCell.load(std::memory_order_acquire);
    if (c != nullptr && key == idisplay::xKey::Up)
      c->scroll(-1);
    else if (c != nullptr && key == idisplay::xKey::Down)
      c->scroll(1);
  }
  void render(idisplay::iTextSurface &s) override {
    idisplay::xCell *c = g_mplCell.load(std::memory_order_acquire);
    if (c != nullptr) {
      c->render(s);
    } else {
      s.setInverse(true);
      s.printAt(0, 0, "NUCLEO-L432KC sensor bring-up");
      s.setInverse(false);
      s.printAt(0, 2, "no MPL3115A2 found (yet)");
      s.printAt(0, 3, "the other sensors: below");
    }
    // "> a 30.3_", its end when longer than the row
    char line[CONSOLE_LINE];
    const uint8_t n = consoleKeys.copy(line, sizeof(line));
    const uint8_t room = CONSOLE_COLS - 3;
    s.printAt(0, CONSOLE_ROWS - 1, "> ");
    s.print(line + (n > room ? n - room : 0));
    s.print("_");
  }
};
static HomeScreen g_home;

static void guiSleep(uint32_t ms) { osDelay(ms); }

/**
  * @brief  consoleTask's body (main.c's StartConsoleTask calls it): puts
  *         the GUI on USART2 (the ST-LINK's virtual COM port), in a task
  *         of its own, takes the keys, and sends the debug log.
  */
void app_console_task(void *argument)
{
  (void) argument;
  static Stm32HalUartTransport port(&huart2);
  idisplay::xTerminalConfig cfg;
  cfg.sleep = guiSleep;                    // while the UART is busy
  cfg.log = &g_logLines;
  static idisplay::xTerminal<CONSOLE_COLS, CONSOLE_ROWS, CONSOLE_LOG_ROWS, CONSOLE_LOG_COLS> term IN_RAM2 (port, cfg);
  static idisplay::xGui gui(term, g_home);
  static idisplay::xTerminalKeys keys(
      [](idisplay::xKey k, idisplay::xKeyAction a, void *g) {
        static_cast<idisplay::xGui *>(g)->post(idisplay::xGuiEvent::keyEvent(k, a), 0);
      }, &gui,
      [](void *t) {                          // Ctrl-L
        static_cast<idisplay::xTerminalBase *>(t)->redraw();
        guiRefresh();
      }, &term);
  consoleKeys.keys = &keys;
  port.setRxSink(consoleKeys);
  if (gui.start("gui", GUI_STACK_BYTES, osPriorityBelowNormal))
    g_gui.store(&gui, std::memory_order_release);
  else
    DBG_FAULT("app", "gui-start");         // no memory: nothing on the console

  for(;;)
  {
    dbg::poll();                           // the debug log, to USART1 (nothing if it isn't the log's)
    osDelay(5);
  }
}
