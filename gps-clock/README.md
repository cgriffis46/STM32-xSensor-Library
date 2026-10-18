# STM32L432KC BMP390 + SHT31 Bring-up

STM32CubeIDE / FreeRTOS project for the NUCLEO-L432KC, for live testing:

* a BMP390 (pressure, temperature) on I2C;
* an SHT31 (temperature, humidity) beside it, so the two temperatures can be
  compared;
* the BMP390's pressure brought to sea level for your elevation. That is the
  "barometer" figure a Davis console or weather.gov shows.

It runs Claude-iTransport's non-blocking drivers, `xbmp390<Stm32HalI2CTransport>`
and `xSHT31<Stm32HalI2CTransport>`, and its `SeaLevelBarometer`. The commits are
in `iTransport/SOURCE.txt`. A console runs on the ST-LINK's USB port, and a debug
log goes out on a second UART.

The project folder is still called `stm32-gps-clock-example`: it started as the
L432 GPS clock (GPS, OLED, W5500, NTP server, PPS), which is in this
repository's history at commit `eb4b352`. Earlier bring-ups are in its history
too:

* the DS1307 at `619ac6c`;
* the BME280 at `2ea91b7`;
* the BME280 with a BMP390 and SHT31 at `57c8bb2`.

The BME280 driver's files are still vendored here, but unused.

## Wiring (NUCLEO-L432KC)

| BMP390 board | Nucleo pin | MCU pin | |
|--------------|------------|---------|---|
| SCK (SCL) | **D1** | PA9, I2C1_SCL | |
| SDI (SDA) | **D0** | PA10, I2C1_SDA | |
| VIN / VCC | **3V3** | | |
| GND | **GND** | | |
| SDO | (leave it) | | sets the address: low 0x76, high 0x77. The firmware finds either |
| CS | (leave it) | | high, or open on boards that pull it up, selects I2C |

| SHT31 board | Nucleo pin | |
|-------------|------------|---|
| SCL | **D1** | the same bus as the BMP390 |
| SDA | **D0** | |
| VIN / VCC | **3V3** | |
| GND | **GND** | |
| ADR / ADDR | (leave it) | low 0x44 (most boards), high 0x45: either is found |

| Debug log | Nucleo pin | MCU pin | |
|-----------|------------|---------|---|
| TX out | **D5** | PB6, USART1_TX | to a 3.3 V USB-serial adapter's **RX**, or a Saleae channel; 921600 8N1 |
| GND | **GND** | | the adapter's GND |
| (RX in) | D4 | PB7, USART1_RX | not used; leave it open (it is pulled up) |

* **I2C pull-ups.** Breakout boards carry pull-ups on SDA and SCL. A bare chip
  needs 4.7k from each to 3V3.
* **One Bosch chip per address.** The BMP390, BME280 and BME680 each have only
  two addresses, 0x76 and 0x77. Two chips at the same address answer
  together, and a read gives the AND of their replies. The 2026-10-18 capture
  with three of them connected showed chip ID 0x00 at 0x77 and nothing at
  0x76, most likely for this reason.
* **The Nucleo pins.** D0/D1 = PA10/PA9 and D4/D5 = PB7/PB6 are from memory of
  ST's UM1956, not checked here. The console is USART2 (PA2/PA15) on the
  ST-LINK's virtual COM port, so D0/D1 are free.

## Using it

1. Flash it (STM32CubeIDE, or STM32CubeProgrammer with the `.elf`).
2. Open the ST-LINK's virtual COM port at 115200 8N1, with local echo on.
3. At start-up the board scans the I2C bus. At 0x76 and 0x77 it reads two ID
   registers: 0x00 (the BMP390's) and 0xD0 (the older Bosch chips'). The
   output looks like this:

       I2C scan: 0x44 0x77
         0x77: chip ID 0x60 (register 0x00) = BMP390
         0x44: answers where an SHT31 would be (it has no ID register to check)
       SHT31 at 0x44: periodic mode, 1 a second, high repeatability
       BMP390 at 0x77: forced, 1 a second, pressure x8, temperature x1, no filter

   * **0x60 in register 0x00** is a BMP390, and **0x50** is a BMP388. The
     same driver runs both.
   * **0x60, 0x58 or 0x61 in register 0xD0** is a BME280, BMP280 or BME680.
     It is reported, but not driven.
   * **Anything else** is reported as unknown. Two chips at one address look
     like this.
   * **Nothing at all:** check SDA/SCL, 3V3 and GND.

   The scan repeats every 2 s until a BMP390 or an SHT31 answers. Whichever is
   found then gets its driver, so connect both before power-up.
4. Then, once a second, the latest readings:

       BMP390 T 25.61 C (78.1 F)  P 1030.48 hPa = 30.57 inHg / 1035.06 hPa at sea level (40 m, cal 0.00 hPa) | 12 readings, T 25.60..25.62 C
       SHT31  T 25.80 C (78.4 F)  H 34.1 %RH | BMP390 - SHT31: T -0.19 C | 12 readings

   These numbers are only an example.

   * `(?)` after a value means it is pinned at a limit of Bosch's
     compensation (-40 or 85 C, 300 or 1250 hPa). Such a reading is suspect.
   * The count and the min..max show the readings keep coming, one a second.
   * `no reading ... driver state <n>` means the driver isn't getting through.
     The debug log says why. The state numbers are:
     * `bmp390_state_t` in `isensor/bmp390/inc/bmp390.h`, where 28 is the
       error state;
     * `sht31_state_t` in `SHT31.h`, where 40 is the error state.
5. LD3 (green) toggles on each BMP390 reading.

### Sea-level pressure

The BMP390 measures the *station* pressure, the air pressure where it sits.
Weather reports and the Davis console show it brought to sea level. The
`SeaLevelBarometer` (Claude-iTransport's `isensor/inc/SeaLevelBarometer.h`) does
the same. It gives the **altimeter setting**: NOAA's formula from the station
pressure and the elevation, which is what weather.gov's "Barometer" is.

1. **Set your elevation.** `STATION_ELEVATION_M` in `main.cpp` is **40 m**
   (131 ft, this board's station, from elevationfinder.net). Change it there,
   or for the session with `e 131` (feet) or `e 40m` (metres). `e` isn't kept
   across a reset.
2. **Calibrate if you like.** When the elevation is right, type the current
   reference reading, e.g. `b 30.54` from weather.gov. The difference becomes
   an offset added to the station pressure. That offset is the sensor's own
   error, so it stays right as the weather changes. `o` clears it.

**What to expect.**

* **Temperature.** Room temperature, perhaps a little high: the chip and the
  board warm it. The SHT31 line shows the difference between the two.
* **Pressure.** Your local station pressure. It is lower than the sea-level
  figure weather reports give, by about 12 hPa per 100 m of altitude.
* **Humidity (SHT31).** Should follow a breath on the sensor up, and back
  down within a few seconds.

### Commands

| Command | |
|---------|---|
| `s` | scan the I2C bus again, and read the chip IDs at 0x76 and 0x77 |
| `e 131` / `e 40m` | the station's elevation, in feet or metres |
| `b 30.54` | calibrate the sea-level reading to this many inHg now |
| `o` | clear the calibration offset |
| `h` | the help |

### The debug log (D5)

Open the USB-serial adapter at **921600 8N1**, or put the Saleae's async serial
analyser on D5. Each line is `<seq> <ms> <tag> <what> [<value>...]`
(Claude-iTransport's `DebugLog.h`, at `ITRANSPORT_DEBUG` 2, which is set in the
project's C and C++ defines). The lines are:

* `app start <rev> <dev>`: the chip's revision and device IDs.
* `app chip-id <address> <id 0x00> <id 0xD0>`: what each scan read at an
  address that answered. Three values; the earlier firmware logged two.
* `bmp390 st <from> <to>`: the BMP390 driver's state changes, as
  `bmp390_state_t` numbers. `bmp390 fail`, `bus xfer-fail` and `bus-err` are
  I2C faults.
* `app reading <T x100> <P Pa>`: each BMP390 reading, as integers. For
  example, `app reading 2561 103048` is 25.61 C, 1030.48 hPa.
* `sht31 st <from> <to>`: the SHT31 driver's states. Each start begins with
  a break (state 38), then the soft reset: the chip keeps its power, and its
  periodic mode, while the board is reset or reflashed, and in periodic mode
  it refuses the reset. On 2026-10-18 the driver without the break failed in
  state 2 every 100 ms (`sht31 bus-err 2`, `sht31 fail 2`).
* `app sht <T x100> <H x100>`: each SHT31 reading.
* `app slp <Pa>`: the sea-level reading, once a second.
* `app cal <offset Pa> <elevation m>`: a `b` calibration.
* `app cmd <char>`: a console command.

A gap in `seq` means lines were dropped (the 1 KB ring was full). Set
`ITRANSPORT_DEBUG` to 3 to see every I2C transfer as well, or to 0 to turn the
log off.

## Regenerating code in CubeMX

`Core/Src/main.c` is CubeMX's own file. The application is in
`Core/Src/app.cpp`, and `main.c` only calls it from three USER CODE blocks
(declared in `Core/Inc/app.h`):

| main.c | USER CODE block | call |
|--------|-----------------|------|
| `main()`, after the `MX_..._Init()` calls | `2` | `app_init();` |
| `StartSensorTask()` | `5` | `app_sensor_task(argument);` |
| `StartConsoleTask()` | `StartConsoleTask` | `app_console_task(argument);` |

The `#include "app.h"` sits in the `Includes` block. CubeMX keeps USER CODE
blocks when it regenerates `main.c`, so the calls survive regeneration, and
`main.c` never needs renaming to `.cpp`. Only `app.cpp` is C++.

A regeneration from before this layout wrote a fresh `main.c` with empty task
loops (`osDelay(1)`). Such a build never touches the I2C bus or the debug
UART. If a regeneration ever loses the calls, put the three lines back in
those blocks.

## Inside

* **sensorTask** (`app_sensor_task()`):
  * scans for the BMP390 and the SHT31, then builds the BMP390's driver at
    the address it found;
  * runs `xbmp390::main()`, which sleeps between steps: chip ID, soft reset,
    error register, calibration, oversampling and filter, then a forced
    measurement once a second, polling for data-ready;
  * takes the console's commands, and prints the readings once a second.
  * A scan asked for while the driver has a transfer outstanding waits for it
    to land. The driver holds the bus mutex across a transfer, and the mutex
    isn't recursive.
* **shtTask:** started by sensorTask when the first scan finds an SHT31. It
  runs `xSHT31::main()` in periodic mode, and publishes each reading through
  atomics. The chip measures once a second, and the driver fetches each
  result a second after the last. A fetch that comes before the chip has a
  new result is refused; it is tried again 100 ms later. Each refusal
  lengthens the fetch period by 1 %, so a chip whose clock runs slow is
  followed, and each refusal shows as one `bus xfer-fail` line in the debug
  log.

  The driver before this fetched every 2 s, so the capture of 2026-10-18 shows
  an SHT31 reading every 2000 ms.
* **consoleTask** (`app_console_task()`):
  * hands the console ring to USART2 through `Stm32HalUartTransport`, which
    also takes the keys;
  * calls `dbg::poll()`, which sends the debug log to USART1.
* **The system clock** is 80 MHz (MSI + PLL, trimmed by the LSE crystal X2).
  Without the LSE, `SystemClock_Config()` stops in `Error_Handler()`.
* **I2C1** runs at 100 kHz. Both chips also take 400 kHz.

## Not verified

* **On the board:**
  * The BME280 bring-up ran on a NUCLEO-L432KC on 2026-10-18, and a Saleae
    capture checked it.
  * A later capture showed the scan, the SHT31 at 0x44, and the BMP390's
    address 0x77 answering. That firmware was the earlier one, and three
    Bosch chips were most likely sharing 0x77 at the time.
  * This firmware, the BMP390 driver and the SHT31 driver haven't run on the
    board yet.
* **Build:** built with arm-none-eabi-gcc 13 using the CubeIDE flags, without
  warnings: 68 KB flash at -Os (102 KB at -O0), 27 KB RAM. sensorTask's worst
  case stack use is about 1.6 KB of its 2 KB.
* **Host tests:** they pass in Claude-iTransport at `e0c5a1a`:
  * `bmp390_test` and `SHT31_test`, on simulated chips;
  * `bmp390_ref_test`, which runs Bosch's BMP3 API on the same simulated
    BMP390 (the driver agrees with it);
  * `sea_level_barometer_test`.
* **Chip IDs:** the BMP390's (0x60 in register 0x00, 0x50 for the BMP388) are
  from Bosch's BMP3 Sensor API. The older chips' IDs are from Bosch's
  datasheets, as the drivers and general knowledge have them, not re-read
  here.
* **CubeMX:** `main.c` was written here in CubeMX's layout; CubeMX hasn't been
  run on it here.
* **The `.ioc`** was edited by hand to match:
  * I2C1, USART1 (921600, PB6/PB7), and USART2 with its interrupt;
  * LD3;
  * sensorTask, consoleTask and their mutexes.

  shtTask isn't in it: sensorTask starts it at run time with `osThreadNew()`.
  Open the `.ioc` in CubeMX and check it before generating code over the
  project.
