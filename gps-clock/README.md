# STM32L432KC BME280 + BMP390 + SHT31 Bring-up

STM32CubeIDE / FreeRTOS project for the NUCLEO-L432KC, for live testing:

* a BME280 (temperature, pressure, humidity) on I2C;
* a BMP390 (temperature, pressure) and an SHT31 (temperature, humidity) beside
  it, to compare against;
* each barometer's pressure brought to sea level for your elevation, the
  "barometer" figure a Davis console or weather.gov shows.

It runs Claude-iTransport's non-blocking drivers, `xbme280<Stm32HalI2CTransport>`,
`xbmp390<Stm32HalI2CTransport>` and `xSHT31<Stm32HalI2CTransport>`, and its
`SeaLevelBarometer`. The commits are
in `iTransport/SOURCE.txt`. A console runs on the ST-LINK's USB port, and a debug
log goes out on a second UART.

The project folder is still called `stm32-gps-clock-example`: it started as the
L432 GPS clock (GPS, OLED, W5500, NTP server, PPS), which is in this
repository's history at commit `eb4b352`. The DS1307 bring-up that came between
is at `619ac6c`.

## Wiring (NUCLEO-L432KC)

| BME280 board | Nucleo pin | MCU pin | |
|--------------|------------|---------|---|
| SCL (or SCK) | **D1** | PA9, I2C1_SCL | |
| SDA (or SDI) | **D0** | PA10, I2C1_SDA | |
| VCC / VIN | **3V3** | | most boards also take 5 V on VIN through a regulator; 3V3 always works |
| GND | **GND** | | |
| SDO | see the BMP390 | | sets the address: low 0x76, high 0x77. The firmware finds either |
| CSB | (leave it) | | high, or open on boards that pull it up, selects I2C |

| BMP390 board (optional) | Nucleo pin | |
|-------------------------|------------|---|
| SCK (SCL) | **D1** | the same bus as the BME280 |
| SDI (SDA) | **D0** | |
| VIN / VCC | **3V3** | |
| GND | **GND** | |
| SDO | GND or 3V3 | low 0x76, high 0x77: **not the BME280's address**. Both chips can only be told apart at different addresses |
| CS | (leave it) | high, or open on boards that pull it up, selects I2C |

The BME280 ran at 0x77 on 2026-10-18 (its SDO high, as on most boards), so put
the BMP390 at 0x76 (its SDO to GND), or swap them round. Adafruit's BMP390 board
is 0x77 unless SDO is tied to GND (from memory of Adafruit's guide, not checked
here).

| SHT31 board (optional) | Nucleo pin | |
|------------------------|------------|---|
| SCL | **D1** | the same bus as the BME280 |
| SDA | **D0** | |
| VIN / VCC | **3V3** | |
| GND | **GND** | |
| ADR / ADDR | (leave it) | low 0x44 (most boards), high 0x45: either is found |

| Debug log | Nucleo pin | MCU pin | |
|-----------|------------|---------|---|
| TX out | **D5** | PB6, USART1_TX | to a 3.3 V USB-serial adapter's **RX**, or a Saleae channel; 921600 8N1 |
| GND | **GND** | | the adapter's GND |
| (RX in) | D4 | PB7, USART1_RX | not used; leave it open (it is pulled up) |

* **I2C pull-ups.** Breakout boards (Adafruit's, and the purple "GY-BME280"
  ones) carry pull-ups on SDA and SCL. A bare chip needs 4.7k from each to
  3V3.
* **The Nucleo pins.** D0/D1 = PA10/PA9 and D4/D5 = PB7/PB6 are from memory of
  ST's UM1956, not checked here. The console is USART2 (PA2/PA15) on the
  ST-LINK's virtual COM port, so D0/D1 are free.

## Using it

1. Flash it (STM32CubeIDE, or STM32CubeProgrammer with the `.elf`).
2. Open the ST-LINK's virtual COM port at 115200 8N1, with local echo on.
3. At start-up the board scans the I2C bus and reads two ID registers at 0x76
   and 0x77: 0xD0 (the BME280's) and 0x00 (the BMP390's). Both chips say 0x60,
   each in its own register:

       I2C scan: 0x44 0x76 0x77
         0x76: chip ID 0x60 (register 0x00) = BMP390
         0x77: chip ID 0x60 (register 0xD0) = BME280
         0x44: answers where an SHT31 would be (it has no ID register to check)
       BME280 at 0x77: forced mode, pressure x4, temperature x1, humidity x1, no filter
       SHT31 at 0x44: periodic mode, 1 a second, high repeatability
       BMP390 at 0x76: forced, 1 a second, pressure x8, temperature x1, no filter

   * **0x60 in 0xD0** is a BME280. The driver starts.
   * **0x60 in 0x00** is a BMP390 (**0x50** a BMP388, which the same driver
     runs). The BME280's register is checked first.
   * **0x58** in 0xD0 is a **BMP280**. It has no humidity, and many boards sold as
     "BME280" carry one. The driver isn't started for it, and the scan repeats.
   * **0x61** is a BME680, which has a different driver (`isensor/bme680`).
   * Nothing at all means check SDA/SCL, 3V3 and GND. The scan repeats every
     2 s until a BME280 answers.
   The SHT31 and the BMP390 are looked for only in the first scan that finds
   the BME280: connect them before power-up.
4. Then, once a second, the latest readings:

       BME280 T 25.39 C (77.7 F)  H 35.5 %RH  P 1030.26 hPa = 30.56 inHg / 1034.84 hPa at sea level (40 m, cal 0.00 hPa) | 27 readings, T 25.38..25.40 C
       SHT31  T 25.80 C (78.4 F)  H 34.1 %RH | BME280 - SHT31: T -0.41 C, H 1.4 %RH | 5 readings
       BMP390 T 25.61 C (78.1 F)  P 1030.48 hPa = 30.57 inHg / 1035.06 hPa at sea level (cal 0.00 hPa) | BME280 - BMP390: T -0.22 C, P -0.22 hPa | 5 readings

   The BME280's station pressure and temperature are the board's real numbers
   from 2026-10-18 (worked out again at 40 m). The SHT31 and BMP390 lines are
   only examples.

   * `(?)` after a value means it is outside the BME280's range (-40..85 C,
     300..1100 hPa), or the humidity is pinned at 0 or 100 %, the compensation
     formula's limits.
   * The count and the min..max show the readings keep coming, about five a
     second.
   * `no reading ... driver state <n>` means the driver isn't getting through.
     The debug log says why. The state numbers are `bme280_state_t` in
     `isensor/bme280/inc/BME280.h`; 32 is the error state.
5. LD3 (green) toggles on each BME280 reading.

### Sea-level pressure

The BME280 measures the *station* pressure, the air pressure where it sits.
Weather reports and the Davis console show it brought to sea level. The
`SeaLevelBarometer` (Claude-iTransport's `isensor/inc/SeaLevelBarometer.h`) does
the same: it gives the **altimeter setting**, NOAA's formula from the station
pressure and the elevation, which is what weather.gov's "Barometer" is.

1. **Set your elevation:** `STATION_ELEVATION_M` in `main.cpp` is **40 m**
   (131 ft, this board's station, from elevationfinder.net). Change it there,
   or for the session with `e 131` (feet) or `e 40m` (metres); `e` isn't kept
   across a reset. It applies to both barometers.
2. **Calibrate if you like:** when the elevation is right, type the current
   reference reading, e.g. `b 30.54` from weather.gov. Each barometer gets its
   own offset, added to its station pressure: that is the sensor's own error,
   so it stays right as the weather changes, and after a `b` the two show how
   far they drift apart. `o` clears both.

On 2026-10-18 the board read 1030.26 hPa at the sensor, and weather.gov showed
30.54 inHg. That pair works out to an elevation of about 35 m (115 ft). At 40 m
the BME280 reads 30.56 inHg, 0.02 inHg (about 0.6 hPa) high: the BME280's
own error, within its datasheet's ±1 hPa (general knowledge), and what `b`
takes out.


**What to expect.**

* **Temperature.** Room temperature, usually a degree or two high. The chip
  warms itself, and the board warms it more if it's mounted near the regulator.
* **Pressure.** Your local station pressure. It is lower than the sea-level
  figure weather reports give, by about 12 hPa per 100 m of altitude.
* **Humidity.** Should follow a breath on the sensor up and back down within
  a few seconds.

### Commands

| Command | |
|---------|---|
| `s` | scan the I2C bus again, and read the chip IDs at 0x76 and 0x77 |
| `e 131` / `e 40m` | the station's elevation, in feet or metres |
| `b 30.54` | calibrate each sea-level reading to this many inHg now |
| `o` | clear the calibration offsets |
| `h` | the help |

### The debug log (D5)

Open the USB-serial adapter at **921600 8N1**, or put the Saleae's async serial
analyser on D5. Each line is `<seq> <ms> <tag> <what> [<value>...]`
(Claude-iTransport's `DebugLog.h`, at `ITRANSPORT_DEBUG` 2, which is set in the
project's C and C++ defines).

* `app start <rev> <dev>`: the chip's revision and device IDs.
* `app chip-id <address> <id 0xD0> <id 0x00>`: what each scan found at 0x76
  and 0x77, in the BME280's and the BMP390's ID registers.
* `bme280 st <from> <to>`: the driver's state changes, as `bme280_state_t`
  numbers. `bme280 fail`, `bus xfer-fail` and `bus-err` are I2C faults.
* `app reading <T x100> <P Pa> <H x100>`: each BME280 reading, as integers.
  For example, `app reading 2341 100987 4120` is 23.41 C, 1009.87 hPa,
  41.20 %RH.
* `sht31 st <from> <to>`: the SHT31 driver's states (`sht31_state_t` in
  `SHT31.h`; 37 is its error state).
* `app sht <T x100> <H x100>`: each SHT31 reading.
* `bmp390 st <from> <to>`: the BMP390 driver's states (`bmp390_state_t` in
  `bmp390.h`; 28 is its error state).
* `app bmp <T x100> <P Pa>`: each BMP390 reading.
* `app slp <Pa>`: the sea-level reading, once a second.
* `app cal <offset Pa> <elevation m>`: a `b` calibration (the BME280's), and
  `app bmp-cal <offset Pa>` the BMP390's.
* `app cmd <char>`: a console command.

A gap in `seq` means lines were dropped (the 1 KB ring was full). Set
`ITRANSPORT_DEBUG` to 3 to see every I2C transfer as well, or to 0 to turn the
log off.

## Inside

* **sensorTask:**
  * scans for the BME280, then builds the driver at the address it found;
  * runs `xbme280::main()`, which sleeps between steps: soft reset, chip ID,
    calibration trim, then a forced measurement roughly every 0.2 s;
  * takes the console's commands, and prints the reading once a second.
  * A scan asked for while the driver has a transfer outstanding waits for it
    to land: the driver holds the bus mutex across a transfer, and the mutex
    isn't recursive.
* **shtTask:** started by sensorTask when the first scan finds an SHT31. It
  runs `xSHT31::main()` in periodic mode (the chip measures once a second; the
  driver fetches each result), and publishes each reading through atomics.
* **bmpTask:** likewise for a BMP390: `xbmp390::main()` (soft reset, chip ID,
  calibration, then a forced measurement once a second), each reading published
  as floats. sensorTask's second `SeaLevelBarometer` reads the published
  pressure, since the driver's own double can't be read whole from another
  thread.
* **consoleTask:**
  * hands the console ring to USART2 through `Stm32HalUartTransport`, which
    also takes the keys;
  * calls `dbg::poll()`, which sends the debug log to USART1.
* **The system clock** is 80 MHz (MSI + PLL, trimmed by the LSE crystal X2).
  Without the LSE, `SystemClock_Config()` stops in `Error_Handler()`.
* **I2C1** runs at 100 kHz. The BME280 and BMP390 also take 400 kHz.

## Not verified

* **On the board:** the BME280 part ran on a NUCLEO-L432KC on 2026-10-18, and a
  Saleae capture checked it:
  * the chip at 0x77 with ID 0x60;
  * the configuration bytes;
  * a reading every 201-202 ms with no faults;
  * the compensation, recomputed by hand from the raw bytes.

  The firmware with the SHT31, the BMP390 and the sea-level readings hasn't
  run yet.
  * Built with arm-none-eabi-gcc 13 using the CubeIDE flags, without warnings:
    73 KB flash at -Os (112 KB at -O0), 27 KB RAM. `strtof` and `pow` (newlib)
    account for about 7 KB of it.
  * The drivers' host tests pass in Claude-iTransport at `e0c5a1a`:
    `BME280_test`, `SHT31_test`, `bmp390_test` (simulated chips), and
    `sea_level_barometer_test`. `bmp390_ref_test` runs Bosch's BMP3 API on the
    same simulated BMP390; the driver agrees with it.
  * Neither the SHT31 nor the BMP390 has been run on a chip by this firmware,
    and the BMP390 driver hasn't run on a chip at all.
* The chip IDs (0x60 BME280, 0x58 BMP280, 0x61 BME680) and the BME280's
  ranges are from Bosch's datasheets as the drivers and general knowledge have
  them, not re-read here. The BMP390's (0x60 in register 0x00, 0x50 BMP388) are
  from Bosch's BMP3 Sensor API. What a BME280 holds in register 0x00, and a
  BMP390 in 0xD0, isn't known; the scan trusts 0xD0 first.
* The `.ioc` was edited by hand to match:
  * I2C1, USART1 (921600, PB6/PB7), and USART2 with its interrupt;
  * LD3;
  * sensorTask, consoleTask and their mutexes.

  shtTask and bmpTask aren't in it: sensorTask starts them at run time with
  `osThreadNew()`.

  Open it in CubeMX and check it before generating code over the project.
