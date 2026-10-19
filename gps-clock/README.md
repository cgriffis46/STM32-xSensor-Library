# STM32L432KC BME680 Bring-up

STM32CubeIDE / FreeRTOS project for the NUCLEO-L432KC, for live testing:

* a BME680 (temperature, pressure, humidity, gas resistance) on I2C;
* its pressure brought to sea level for your elevation, the "barometer" figure a
  Davis console or weather.gov shows;
* optionally an SHT31 (temperature, humidity) beside it, to compare against.

It runs Claude-iTransport's non-blocking drivers, `xbme680<Stm32HalI2CTransport>`
and `xSHT31<Stm32HalI2CTransport>`, and its `SeaLevelBarometer`. The commits are
in `iTransport/SOURCE.txt`. A console runs on the ST-LINK's USB port, and a debug
log goes out on a second UART.

The project folder is still called `stm32-gps-clock-example`: it started as the
L432 GPS clock (GPS, OLED, W5500, NTP server, PPS), which is in this
repository's history at commit `eb4b352`. The earlier bring-ups are there too:

* the DS1307 at `619ac6c`;
* the BME280 at `2ea91b7`;
* the BMP390 and SHT31 at `c6565f1`.

The BME280 and BMP390 drivers' files are still vendored here, unused.

## Wiring (NUCLEO-L432KC)

| BME680 board | Nucleo pin | MCU pin | |
|--------------|------------|---------|---|
| SCK (SCL) | **D1** | PA9, I2C1_SCL | |
| SDI (SDA) | **D0** | PA10, I2C1_SDA | |
| VIN / VCC | **3V3** | | |
| GND | **GND** | | |
| SDO | (leave it) | | sets the address: low 0x76, high 0x77. The firmware finds either |
| CS | (leave it) | | high, or open on boards that pull it up, selects I2C |

An SHT31 (optional) goes on the same D1/D0, 3V3 and GND. Its ADR pin chooses
0x44 (low, most boards) or 0x45, and either is found.

| Debug log | Nucleo pin | MCU pin | |
|-----------|------------|---------|---|
| TX out | **D5** | PB6, USART1_TX | to a 3.3 V USB-serial adapter's **RX**, or a Saleae channel; 921600 8N1 |
| GND | **GND** | | the adapter's GND |
| (RX in) | D4 | PB7, USART1_RX | not used; leave it open (it is pulled up) |

* **I2C pull-ups.** Breakout boards carry pull-ups on SDA and SCL. A bare chip
  needs 4.7k from each to 3V3.
* **One Bosch chip per address.** The BME680, BME280 and BMP390 each have only
  two addresses, 0x76 and 0x77. Two chips at the same address answer
  together, and a read gives the AND of their replies.
* **The Nucleo pins.** D0/D1 = PA10/PA9 and D4/D5 = PB7/PB6 are from memory of
  ST's UM1956, not checked here. The console is USART2 (PA2/PA15) on the
  ST-LINK's virtual COM port, so D0/D1 are free.

## Using it

1. Flash it (STM32CubeIDE, or STM32CubeProgrammer with the `.elf`).
2. Open the ST-LINK's virtual COM port at 115200 8N1, with local echo on.
3. At start-up the board scans the I2C bus. At 0x76 and 0x77 it reads two ID
   registers: 0xD0 (the BME680's, and the older Bosch chips') and 0x00 (the
   BMP390's). The output looks like this:

       I2C scan: 0x77
         0x77: chip ID 0x61 (register 0xD0) = BME680 (or BME688)
       BME680 at 0x77: forced every 3 s, T x2, P x4, H x1, filter 3, heater 320 C for 150 ms

   * **0x61 in register 0xD0** is a BME680, or a BME688, which the same driver
     runs.
   * **0x60 or 0x58 in 0xD0** (a BME280 or BMP280), or **0x60 in 0x00** (a
     BMP390), is reported, but not driven.
   * **Anything else** is reported as unknown. Two chips at one address look
     like this.

   The scan repeats every 2 s until a BME680 or an SHT31 answers.
4. Then, every 3 s, a BME680 line:

       BME680 T 25.31 C (77.6 F)  H 36.2 %RH  P 1029.90 hPa = 30.55 inHg / 1034.48 hPa at sea level (40 m, cal 0.00 hPa)  gas 85.3 kOhm | 12 readings, T 25.28..25.33 C

   These numbers are only an example. An SHT31 adds a line once a second, with
   the BME680's difference from it.

   * **Gas** is the resistance of the heated sensor plate, in kOhm. Higher
     means cleaner air.
     * It drifts upward for the first minutes, and over the first day or two
       of a new sensor's life.
     * There is no IAQ index: that is Bosch's closed-source BSEC library.
     * `(no gas reading)` or `(heater not stable)` means the chip flagged the
       gas conversion, and the value is `nan`.
   * `(?)` after a value means it is pinned at a limit of the compensation
     (-40 or 85 C; 0 or 100 %RH).
   * `no reading ... driver state <n>` means the driver isn't getting through.
     The debug log says why. The state numbers are `bme680_state_t` in
     `isensor/bme680/inc/bme680.h`, where 31 is the error state.
5. LD3 (green) toggles on each BME680 reading.

### Why every 3 s

Each measurement heats the gas plate to 320 C for 150 ms, which warms the whole
chip a little. Measuring every 3 s keeps that small. Bosch's low-power mode
uses the same 3 s, from memory of their BSEC documentation. Even so, expect
the BME680's temperature to read a little high.

### Sea-level pressure

The BME680 measures the *station* pressure, the air pressure where it sits.
The `SeaLevelBarometer` (Claude-iTransport's `isensor/inc/SeaLevelBarometer.h`)
gives the **altimeter setting** from it: NOAA's formula from the station
pressure and the elevation, which is what weather.gov's "Barometer" and the
Davis console show.

1. **Set your elevation.** `STATION_ELEVATION_M` in `app.cpp` is **40 m**
   (131 ft, this board's station, from elevationfinder.net). Change it there,
   or for the session with `e 131` (feet) or `e 40m` (metres).
2. **Calibrate if you like.** When the elevation is right, type the current
   reference reading, e.g. `b 30.53` from the Davis. The difference becomes
   an offset added to the station pressure, and stays right as the weather
   changes. `o` clears it. Neither `e` nor `b` is kept across a reset.

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
analyser on D5. Sample at 25 MS/s or more: at 10 MS/s the odd character comes
out wrong. Each line is `<seq> <ms> <tag> <what> [<value>...]`
(Claude-iTransport's `DebugLog.h`, at `ITRANSPORT_DEBUG` 2, which is set in the
project's C and C++ defines). The lines are:

* `app start <rev> <dev>`: the chip's revision and device IDs.
* `app chip-id <address> <id 0x00> <id 0xD0>`: what each scan read at an
  address that answered.
* `bme680 st <from> <to>`: the BME680 driver's state changes, as
  `bme680_state_t` numbers. `bme680 fail`, `bus xfer-fail` and `bus-err` are
  I2C faults.
* `app reading <T x100> <P Pa> <H x100> <gas ohm> <flags>`: each BME680
  reading, as integers. The gas is -1 when it is NAN. Flags: 1 gas_valid,
  2 heat_stable, so 3 means a good gas reading. For example,
  `app reading 2531 102990 3620 85300 3` is 25.31 C, 1029.90 hPa, 36.20 %RH,
  85.3 kOhm.
* `app slp <Pa>`: the sea-level reading, with each BME680 line.
* `app cal <offset Pa> <elevation m>`: a `b` calibration.
* `sht31 st ...` and `app sht <T x100> <H x100>`: the SHT31, if fitted.
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
`main.c` never needs renaming to `.cpp`. Only `app.cpp` is C++. If a
regeneration ever loses the calls, put the three lines back in those blocks.

## Inside

* **sensorTask** (`app_sensor_task()`):
  * scans for the BME680 (and an SHT31), then builds the BME680's driver at
    the address it found;
  * runs `xbme680::main()`, which sleeps between steps. At start-up: chip ID,
    soft reset, ID again, variant, 42 calibration bytes, the control
    registers, then the configuration. Then a forced measurement every 3 s,
    with Bosch's measurement time plus the heater's, reading the data until
    new_data is set;
  * takes the console's commands, and prints each reading.
  * A scan asked for while the driver has a transfer outstanding waits for it
    to land. The driver holds the bus mutex across a transfer, and the mutex
    isn't recursive.
* **shtTask:** started by sensorTask only when the first scan finds an SHT31.
  * It runs `xSHT31::main()` in periodic mode, once a second, and publishes
    each reading through atomics.
  * Each start begins with a break: the chip keeps its power, and its periodic
    mode, while the board is reset.
* **consoleTask** (`app_console_task()`):
  * hands the console ring to USART2 through `Stm32HalUartTransport`, which
    also takes the keys;
  * calls `dbg::poll()`, which sends the debug log to USART1.
* **The system clock** is 80 MHz (MSI + PLL, trimmed by the LSE crystal X2).
  Without the LSE, `SystemClock_Config()` stops in `Error_Handler()`. From
  cold, the LSE takes about 145 ms to start, so boot is that much later.
* **I2C1** runs at 100 kHz.

## Not verified

* **On the board:** this firmware ran on the NUCLEO-L432KC on 2026-10-18,
  checked with Saleae captures.
  * The BME680 was found at 0x77 (0x61 in 0xD0, variant 0x00), and the first
    reading came at 206 ms.
  * A reading every 3 s, with no faults:
    * 25.66 C (78.2 F), 32 %RH;
    * 1030.6 hPa, which is 30.57 inHg at sea level for 40 m;
    * the Davis console at the same time: 79 F and 30.53 inHg.
  * The gas resistance was 1.2-2.5 kOhm in the first run and 24-28 kOhm in
    the next, still rising: a new sensor settling in. One reading's gas value
    was checked by hand from the bytes on the bus, with Bosch's formula
    (1240 ohm, as the driver gave).
  * After 1 h 43 min of running (2026-10-19): gas 208-211 kOhm, settled
    from the first runs' 1-28 kOhm. 25.8 C (78.4 F), 32.5-32.9 %RH,
    1031.4 hPa (30.59 inHg at sea level). Still a reading every 3000 ms,
    with no faults.
  * That capture showed one console line in about ten going unprinted: the
    once-a-second print only took a reading less than a second old, and its
    second drifts against the reading's. Fixed (a "not printed yet" flag),
    but the fix hasn't run on the board yet.
  * Not yet checked: anything against a calibrated reference.
  * Earlier bring-ups ran on this NUCLEO-L432KC on 2026-10-18: the BME280,
    then the BMP390 and SHT31, checked with Saleae captures.
* **Build:** built with arm-none-eabi-gcc 13 using the CubeIDE flags, without
  warnings: 70 KB flash at -Os (105 KB at -O0), 27 KB RAM.
* **Host tests:** `bme680_test`, `SHT31_test` and `sea_level_barometer_test`
  pass in Claude-iTransport at `35cfc4f`. `bme680_ref_int_test` and
  `bme680_ref_float_test` run Bosch's BME68x API on the same simulated chip as
  the driver. They passed when the driver was added, but weren't re-run for
  this change: they need Bosch's source.
* **Chip IDs:** 0x61 for the BME680/BME688 is from Bosch's BME68x API. The
  others are from Bosch's APIs and datasheets, as the drivers and general
  knowledge have them.
* **The `.ioc`** was edited by hand to match:
  * I2C1, USART1 (921600, PB6/PB7), and USART2 with its interrupt;
  * LD3;
  * sensorTask, consoleTask and their mutexes.

  shtTask isn't in it: sensorTask starts it at run time with `osThreadNew()`.
  Open the `.ioc` in CubeMX and check it before generating code over the
  project.
