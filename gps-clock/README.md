# STM32L432KC BME280 Bring-up

STM32CubeIDE / FreeRTOS project for the NUCLEO-L432KC with a BME280
(temperature, pressure, humidity) on I2C, for live testing. It runs
Claude-iTransport's non-blocking BME280 driver (`isensor/bme280`, as
`xbme280<Stm32HalI2CTransport>`; the commit is in `iTransport/SOURCE.txt`). A
console runs on the ST-LINK's USB port, and a debug log goes out on a second UART.

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
| SDO | (leave it) | | sets the address: low 0x76, high 0x77. The firmware finds either |
| CSB | (leave it) | | high, or open on boards that pull it up, selects I2C |

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
3. At start-up the board scans the I2C bus and reads the chip ID (register
   0xD0) at 0x76 and 0x77:

       I2C scan: 0x76
         0x76: chip ID 0x60 = BME280
       BME280 at 0x76: forced mode, pressure x4, temperature x1, humidity x1, no filter

   * **0x60** is a BME280. The driver starts.
   * **0x58** is a **BMP280**. It has no humidity, and many boards sold as
     "BME280" carry one. The driver isn't started for it, and the scan repeats.
   * **0x61** is a BME680, which has a different driver (`isensor/bme680`).
   * Nothing at all means check SDA/SCL, 3V3 and GND. The scan repeats every
     2 s until a BME280 answers.
4. Then, once a second, the latest reading:

       T 23.41 C  P 1009.87 hPa  H 41.2 %RH | 25 readings, T 23.38..23.45 C

   * `(?)` after a value means it is outside the BME280's range (-40..85 C,
     300..1100 hPa), or the humidity is pinned at 0 or 100 %, the compensation
     formula's limits.
   * The count and the min..max show the readings keep coming, about five a
     second.
   * `no reading ... driver state <n>` means the driver isn't getting through.
     The debug log says why. The state numbers are `bme280_state_t` in
     `isensor/bme280/inc/BME280.h`; 32 is the error state.
5. LD3 (green) toggles on each reading.

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
| `s` | scan the I2C bus again, and read the chip ID at 0x76 and 0x77 |
| `h` | the help |

### The debug log (D5)

Open the USB-serial adapter at **921600 8N1**, or put the Saleae's async serial
analyser on D5. Each line is `<seq> <ms> <tag> <what> [<value>...]`
(Claude-iTransport's `DebugLog.h`, at `ITRANSPORT_DEBUG` 2, which is set in the
project's C and C++ defines).

* `app start <rev> <dev>`: the chip's revision and device IDs.
* `app chip-id <address> <id>`: what each scan found at 0x76 and 0x77.
* `bme280 st <from> <to>`: the driver's state changes, as `bme280_state_t`
  numbers. `bme280 fail`, `bus xfer-fail` and `bus-err` are I2C faults.
* `app reading <T x100> <P Pa> <H x100>`: each reading, as integers. For
  example, `app reading 2341 100987 4120` is 23.41 C, 1009.87 hPa, 41.20 %RH.
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
* **consoleTask:**
  * hands the console ring to USART2 through `Stm32HalUartTransport`, which
    also takes the keys;
  * calls `dbg::poll()`, which sends the debug log to USART1.
* **The system clock** is 80 MHz (MSI + PLL, trimmed by the LSE crystal X2).
  Without the LSE, `SystemClock_Config()` stops in `Error_Handler()`.
* **I2C1** runs at 100 kHz. The BME280 also takes 400 kHz.

## Not verified

* Not run on the board yet.
  * Built with arm-none-eabi-gcc 13 using the CubeIDE flags, without warnings:
    42 KB flash at -Os (70 KB at -O0), 26 KB RAM.
  * The driver's host test (`BME280_test`, a simulated chip) passes in
    Claude-iTransport at `ca4311e`.
* The chip IDs (0x60 BME280, 0x58 BMP280, 0x61 BME680) and the BME280's
  ranges are from Bosch's datasheets as the drivers and general knowledge have
  them, not re-read here.
* The `.ioc` was edited by hand to match:
  * I2C1, USART1 (921600, PB6/PB7), and USART2 with its interrupt;
  * LD3;
  * the two tasks and their mutexes.

  Open it in CubeMX and check it before generating code over the project.
