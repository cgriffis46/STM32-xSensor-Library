# STM32L432KC Sensor Bring-up

One STM32CubeIDE / FreeRTOS project for the NUCLEO-L432KC that tests any of
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport)'s sensor
drivers on real hardware. Plug sensors onto I2C1 and reset:

* the board scans the bus and recognises each chip by its address and, where
  it has one, its ID register;
* it starts that chip's non-blocking driver in a thread of its own;
* it prints each reading on the ST-LINK's USB serial port as it arrives.

There is no project to switch and no code to change for each sensor.

The drivers are vendored in `l432kc-sensor-bringup/iTransport` and
`l432kc-sensor-bringup/isensor` by `sync_from_claude_itransport.sh`, which
records the commit in `iTransport/SOURCE.txt`. They replace the old blocking
drivers and single-sensor examples this repository used to hold; those are in
its history, tagged `legacy-2026-10`.

## Sensors

| Address | Chip | How it is recognised | Driver | Every |
|---------|------|----------------------|--------|-------|
| 0x19 | LSM303DLHC accelerometer | answers (no ID register) | `lsm303dlhc_accel` | 1 s |
| 0x1E | LSM303DLHC magnetometer | "H43" in 0x0A..0x0C | `lsm303dlhc_mag` | 1 s |
| 0x21 | HMC6352 compass | answers | `hmc6352` | 1 s |
| 0x30 | MMC5603 / MMC5633 | product ID 0x10 in 0x39 | `mmc56x3` | 1 s |
| 0x38 | AHT20 | answers | `AHT20` | 1 s |
| 0x40 | Si7021 / Si7020 / Si7013 | electronic ID SNB_3 0x15 / 0x14 / 0x0D | `Si7021` | 2 s |
| 0x40 | HTU21D(F), or anything else there | not an Si70xx | `HTU21DF` | 1 s |
| 0x44 / 0x45 | SHT31 | answers | `SHT31` (periodic) | 1 s |
| 0x5C / 0x5D | LPS35HW | WHO_AM_I 0xB1 in 0x0F | `lps35hw` | 1 s |
| 0x60 | MPL3115A2 | WHO_AM_I 0xC4 in 0x0C | `mpl3115a2` (both modes: pressure, then altitude) | 5 s |
| 0x68 | DS3231 | registers 0x0E..0x12 as only a DS3231 has them, and the register pointer wrapping from 0x12 to the seconds (read only) | `ds3231` (time, temperature, aging offset) | 1 s |
| 0x68 | DS1307 | answers, and isn't a DS3231 | `ds1307` | 1 s |
| 0x76 / 0x77 | BME280 | 0x60 in 0xD0 | `bme280` (forced) | ~0.25 s |
| 0x76 / 0x77 | BMP280 | 0x58 in 0xD0 | `bmp280` (forced) | 1 s |
| 0x76 / 0x77 | BME680 / BME688 | 0x61 in 0xD0 | `bme680` (gas heater 320 C, 150 ms) | 3 s |
| 0x76 / 0x77 | BMP390 / BMP388 | 0x60 / 0x50 in 0x00 | `bmp390` | 1 s |
| USART1 | PMS5003 (Plantower) | build option, see below | `PM25` | ~1 s |
| USART1 | DS18B20 (one, on 1-Wire) | build option, see below | `ds18b20` | 1 s |

Notes on recognising the chips:

* **"Answers"** means the chip has no ID to check. Any chip at that address
  is taken to be it: an MPU-6050 at 0x68 would be driven as a DS1307.
* **DS3231 or DS1307** (both at 0x68): the scan only reads. A DS3231's
  status, temperature and control registers (0x0E..0x12) have bits only it
  holds at 0 and a temperature of -40..85 C; and its register pointer wraps
  from 0x12 to the seconds, where a DS1307 has RAM, so two bytes read from
  0x12 end with the seconds. At :00 that can't be told from RAM of zeros, so
  it tries again (three tries, 600 ms apart). Anything else is a DS1307.
  Checked here against simulated chips: every instant of a minute, and
  100,000 DS1307s with random RAM and 60 with RAM of zeros (none taken for a
  DS3231). The wrap, the zero bits and the temperature format are as Maxim's
  DS3231 datasheet (19-5170 Rev 10) gives them; not yet tried on a chip.
* **Only the first of each kind is driven.** A second BME280 at the other
  address is reported, not started.
* **The Bosch chips** (BME280, BMP280, BME680, BMP390) share 0x76 and 0x77:
  at most two of them on one bus, one at each address. Two chips at one
  address answer together, and a read gives the AND of their replies; the scan
  says "two chips at this address?" when an ID makes no sense.
* **The Si70xx ID** values, and the HTU21D(F) using the same commands, are
  from general knowledge of the Si7021 and HTU21D datasheets, not checked
  here.

## Wiring (NUCLEO-L432KC)

| Signal | Nucleo pin | MCU pin | |
|--------|------------|---------|---|
| I2C SCL | **D1** | PA9, I2C1_SCL | every sensor's SCL (SCK) |
| I2C SDA | **D0** | PA10, I2C1_SDA | every sensor's SDA (SDI) |
| Power | **3V3**, **GND** | | every sensor's VIN, GND |
| Debug log | **D5** | PB6, USART1_TX | to a 3.3 V USB-serial adapter's RX, or a Saleae; 921600 8N1 |
| (USART1 RX) | D4 | PB7 | unused by default |
| Console | USB | PA2/PA15, USART2 | the ST-LINK's virtual COM port, 115200 8N1: a GUI for PuTTY (see "The console") |

* **I2C pull-ups.** Breakout boards carry pull-ups on SDA and SCL. A bare chip
  needs 4.7k from each to 3V3.
* **The Nucleo pins.** D0/D1 = PA10/PA9 and D4/D5 = PB7/PB6 are from memory of
  ST's UM1956, not checked here.

### USART1: the debug log, a PMS5003, or a DS18B20

The L432KC has no other UART free (USART2 is the console), so USART1 does one
of three things. Set `BRINGUP_USART1` in the project's C++ defines:

| `BRINGUP_USART1` | USART1 is | Wiring |
|------------------|-----------|--------|
| 0 (default) | the debug log, TX only, 921600 | D5 to the adapter's RX |
| 1 | a Plantower PMS5003 at 9600 | the PMS5003's TX to **D4**, 5 V and GND to its power pins |
| 2 | a 1-Wire bus with one DS18B20 | **D5 and D4 linked**, to the DS18B20's DQ, with 4.7k from DQ to 3V3 |

With 1 or 2 there is no debug log. For the DS18B20, D5 is set to open drain at
start-up. The UART hears what it sends through the D4 link, as Claude-iTransport's
`Stm32HalOneWireTransport` needs.

## Using it

1. Flash it (STM32CubeIDE, or STM32CubeProgrammer with the `.elf`).
2. Open the ST-LINK's virtual COM port at 115200 8N1 in PuTTY, set up as in
   "The console" below (local echo off).
3. At start-up the board scans the bus. With a BME680 and an SHT31 the
   console's lines look like this:

       I2C scan: 0x44 0x77
         0x44: SHT31
         0x77: BME680
       sht31     at 0x44: started
       bme680    at 0x77: started
       2 sensors running

   The scan repeats every 2 s until something is recognised. Sensors are
   started only by that first good scan, and I2C isn't hot-pluggable: connect
   sensors with the board unplugged, then power up. `s` scans again later,
   but only reports.

   Before the scan, the board clears a stuck bus. A reset of the STM32
   (after flashing, or the button) doesn't reset the sensors, and one caught
   sending a 0 keeps SDA low. If SDA is low at start, `app_init()` clocks SCL
   by hand, up to 9 times, until SDA is released, then sends a START and a
   STOP and gives I2C1 its pins back. If it had to, the console says so before the scan:

       I2C1: SDA was held low at start; 3 clock(s) freed it

   If SDA stays low after 9 clocks, or SCL itself is low, the console says
   to check the wiring and power. A sensor without its ground wire can do
   either.

   The bus is also watched while the board runs. Every 0.5 s sensorTask takes
   the bus mutex (so nothing is in flight) and checks the bus is idle: SCL
   and SDA high, I2C1 not busy. A sensor that misbehaves in the middle of a
   transfer can leave I2C1 holding SCL low and refusing every transfer until
   a reset, as an MMC56x3 did. If the bus isn't idle, I2C1 is reset and the
   bus cleared as at start, and the drivers start again by themselves:

       I2C1 recovered (#1): I2C1 busy; SCL low; 0 clock(s), bus free

   If the mutex can't be had for 2 s, a transfer never ended. Its driver is
   told it failed, then the same recovery runs. If a recovery leaves the bus
   still stuck (a dead chip holding SDA, say), the next one waits 5 s, then
   10, 20, 40 and 80 s, until the bus is seen idle again.
4. Each reading is printed as it comes, with a count at the end:

       sht31     T 24.97 C (76.9 F)  H 35.8 %RH | 12
       bme680    T 25.79 C (78.4 F)  H 32.5 %RH  P 1031.42 hPa  gas 209.9 kOhm = 30.59 inHg / 1035.99 hPa at sea level (40 m, cal 0.00 hPa) | 4

   These numbers are only an example.

   * **Barometers:** every barometer's line ends with its sea-level reading.
   * **No readings:** a sensor that stops reading for two periods and 2 s
     more is reported once, with its driver's state number. Look the number
     up in the driver's header, as `<driver>_state_t`, where the last value
     is the error state.
   * **LD3 (green)** toggles on each reading, from any sensor.

### The console

The console is a GUI drawn on the terminal (Claude-iTransport's iDisplay:
`xTerminal`, `xGui`, `xCell`). At the top, in a frame, is the home screen; below
it is a pane of the console's last 14 lines (the scan, each reading, the
commands' replies). Once an MPL3115A2 is found, the home screen shows its
cell:

    +----------------------------------------+
    |MPL3115A2 (I2C1)                        |
    |Temperature                     25.38 °C|
    |Pressure                     1022.99 hPa|
    |Sea level                     30.34 inHg|
    |Altitude                          35.6 m|
    |Altimeter setting             30.34 inHg|
    |> a 30.36_                              |
    +----------------------------------------+
    ...
    mpl3115a2 T 25.38 C (77.7 F)  P 1022.99 hPa  alt 35.6 m (117 ft) at 30.34 inHg

The bottom row is the command line. Type a command (see "Commands") and press
Enter; Backspace works. The arrow keys, and Enter or Backspace on an empty line,
go to the GUI, which has nothing for them yet (Up and Down would scroll the
cell if it had more labels than rows). Ctrl-L repaints everything (after reopening PuTTY, say). Until
an MPL3115A2 is found the home screen says so, and the other sensors' readings
are in the pane.

PuTTY settings:

| Where | Setting |
|-------|---------|
| Session | Serial, the ST-LINK's COM port, speed 115200 |
| Connection > Serial | 8 data bits, 1 stop bit, no parity, no flow control |
| Terminal | Local echo **Force off**, local line editing **Force off** (the board draws what you type) |
| Window | at least 80 x 24 (the default) |
| Window > Translation | Remote character set UTF-8 (the degree sign) |

The pane's lines are cut at 79 characters, so the end of a long reading (the
MPL3115A2's sea-level figure, say) is only in its cell.

### Sea-level pressure

Each barometer measures the *station* pressure, the air pressure where it sits.
A `SeaLevelBarometer` (Claude-iTransport's `isensor/inc/SeaLevelBarometer.h`)
for each one gives the **altimeter setting**: NOAA's formula from the station
pressure and the elevation. That is what weather.gov's "Barometer" and a Davis
console show.

1. **Set your elevation.** `STATION_ELEVATION_M` in `app.cpp` is **40 m**
   (131 ft, this board's station). Change it there, or for the session with
   `e 131` (feet) or `e 40m` (metres). It applies to every barometer.
2. **Calibrate if you like.** When the elevation is right, type the current
   reference reading, e.g. `b 30.53`. Each barometer gets its own offset, added
   to its station pressure, so after a `b` the barometers show how far they
   drift apart. `o` clears the offsets. Neither `e` nor `b` is kept across a
   reset.

### The MPL3115A2's altitude

The MPL3115A2 makes two measurements each period: the pressure (barometer
mode), then its own altitude (altimeter mode), reckoned from the pressure and
the sea-level reference it is given (its BAR_IN register). Give it the current
altimeter setting with `a`, e.g. weather.gov's `a 30.36`, and its altitude is
an estimate of the station's elevation, independent of `e`:

    mpl3115a2 T 24.00 C (75.2 F)  P 1023.32 hPa  alt 39.4 m (129 ft) at 30.36 inHg = ...

Until then it uses the chip's own reference, 29.92 inHg: pressure altitude, as
an aircraft's altimeter shows it with 29.92 set. `a` isn't kept across a reset;
the driver writes it to the chip again whenever the chip is reset. `b`'s
offset doesn't change the altitude (the chip only sees its own pressure).

### Commands

| Command | |
|---------|---|
| `s` | scan the I2C bus again (reports only) |
| `e 131` / `e 40m` | the station's elevation, in feet or metres |
| `b 30.54` | calibrate each barometer's sea-level reading to this many inHg now |
| `o` | clear the calibration offsets |
| `a 30.36` | the MPL3115A2's altimeter setting (its sea-level reference), for its altitude |
| `t 2026-10-19 14:05:00` | set the RTC's time (24 hour): a DS1307's oscillator starts; a DS3231's OSF is cleared |
| `g -5` | the DS3231's aging offset, -128..127 (about 0.1 ppm a step; plus slows it), applied by a temperature conversion |
| `h` | the help |

### The debug log (D5)

Open the USB-serial adapter at **921600 8N1**, or put the Saleae's async serial
analyser on D5. Sample at 25 MS/s or more: at 10 MS/s the odd character comes
out wrong. Each line is `<seq> <ms> <tag> <what> [<value>...]`
(Claude-iTransport's `DebugLog.h`, at `ITRANSPORT_DEBUG` 2, set in the project's
C and C++ defines). The lines are:

* `app start <rev> <dev>`: the chip's revision and device IDs.
* `app bus-clear <n>`: SDA was low at start. A positive `n` is how many clocks
  freed it; -1 means it was still low after 9; -2 means SCL was held low.
  Nothing is logged when the bus was free.
* `app bus-recover <n> <why> <after>`: I2C1 was reset while running. `n` as
  for `bus-clear`; `why` adds 1 (I2C1 busy), 2 (SCL low), 4 (SDA low) and 8
  (a transfer never ended); `after` is the same for the bus afterwards, 0 if
  it is idle.
* `app bus-stall <why>`: a transfer never ended, and its driver still held
  the bus a second after being told it failed. Tried again later.
* `app chip-id <address> <id 0x00> <id 0xD0>`: what the scan read at 0x76 and
  0x77.
* `app sensor <address> <count>`: a sensor's thread started.
* `<sensor> st <from> <to>`: that driver's state changes. `<sensor> fail`,
  `bus xfer-fail` and `bus-err` are I2C faults.
* `<sensor> reading <values...>`: each reading, the values as integers at the
  decimals printed (x100 for 2 places, x10 for 1). The sea-level pressure in
  Pa comes last, for a barometer. For example,
  `bme680 reading 2579 325 103140 2110 103598` is 25.79 C, 32.5 %RH,
  1031.40 hPa, 211.0 kOhm, and 1035.98 hPa at sea level.
* `<sensor> cal <offset Pa> <elevation m>`: a `b` calibration.
* `app silent <index> <state>`: a sensor that stopped reading.
* `app cmd <char>`: a console command.

A gap in `seq` means lines were dropped (the 1 KB ring was full).

## Regenerating code in CubeMX

`Core/Src/main.c` is CubeMX's own file. The application is in
`Core/Src/app.cpp`, and `main.c` only calls it from three USER CODE blocks
(declared in `Core/Inc/app.h`):

| main.c | USER CODE block | call |
|--------|-----------------|------|
| `main()`, after the `MX_..._Init()` calls | `2` | `app_init();` |
| `StartSensorTask()` | `5` | `app_sensor_task(argument);` |
| `StartConsoleTask()` | `StartConsoleTask` | `app_console_task(argument);` |

CubeMX keeps USER CODE blocks when it regenerates `main.c`, so the calls survive
regeneration, and `main.c` never needs renaming to `.cpp`. If a regeneration
ever loses the calls, put the three lines back in those blocks.

## Updating the drivers

    sh sync_from_claude_itransport.sh /path/to/Claude-iTransport

That copies the drivers (the list is at the top of the script) and rewrites
`iTransport/SOURCE.txt`. Fix bugs in Claude-iTransport, then sync again; don't
edit the copies here.

## Inside

* **sensorTask** (`app_sensor_task()`):
  * scans until something is recognised, then starts a thread for each
    sensor found;
  * then it only takes the console's commands, and reports sensors that have
    gone quiet.
  * The scan's blocking HAL calls hold the bus mutex. The sensor threads
    simply wait for it.
* **A thread per sensor**, made with `osThreadNew()` at run time, so they
  aren't in the `.ioc`:
  * each runs its driver's `x*` variant (`main()`, which sleeps between
    steps), formats each new reading and prints it;
  * barometers also get their sea-level reading, and act on `e`, `b` and
    `o`;
  * 2 KB of stack each.
  * The drivers can't share one thread: each holds the bus mutex from issuing
    a transfer until it lands, and the mutex isn't recursive.
  * The driver objects are built only for the sensors found: function-local
    statics, never freed.
* **consoleTask** (`app_console_task()`): builds the console's GUI on USART2
  (an `xTerminal`, 40 x 7 framed, with a 14 line log pane 79 wide), starts its
  task, takes the keys, and calls `dbg::poll()`, which sends the debug log to
  USART1.
* **The GUI task** (`xGui`, 1.5 KB of stack, below normal priority) is the only
  thing that writes to USART2. It draws when asked: a new console line, a new
  MPL3115A2 reading, a key. `consolePrint()` adds to the log pane (under
  `console_mutex`, the pane's own lock inside it); the MPL3115A2's thread
  stores its readings in atomics, which the cell's labels read.
* **SRAM2:** the terminal's buffers and the log pane (5 KB) are in SRAM2,
  through a `.ram2` section added by hand to `STM32L432KCUX_FLASH.ld`
  (`NOLOAD`: not cleared at start-up, which they don't need). SRAM1 (48 KB) is
  otherwise nearly full: 44 KB, 32 KB of it the FreeRTOS heap. If CubeIDE ever
  writes a new linker script, put the section back.
* **The FreeRTOS heap** is 32 KB, enough for about a dozen sensor threads.
* **The system clock** is 80 MHz (MSI + PLL, trimmed by the LSE crystal X2).
  From cold, the LSE takes about 145 ms to start.
* **I2C1** runs at 100 kHz.
* **The bus clear** (`busClear()`, from `app_init()`) runs before the
  scheduler and before the debug log is sent: the pins are open drain GPIOs
  for at most 9 clocks of at least 10 us each. A free bus is only read, never
  touched.
* **The bus watch** (`busWatch()`, from sensorTask's loop and its scan
  retries) checks only while holding the bus mutex, which each driver holds
  from issuing a transfer until it lands. A recovery de-initialises I2C1,
  pulses its RCC reset, runs the same clear, and starts it again.

## Not verified

* **On the board:**
  * This firmware ran on the NUCLEO-L432KC on 2026-10-19, checked with Saleae
    captures, with these sensors:
    * the BME680;
    * the AHT20, HTU21D and BME680 together;
    * the Si7021, LPS35HW and BMP280 together.

    Every one was recognised by the scan and started, and read on schedule
    with no faults. The intervals were 1001 ms, 2001 ms for the Si7021 and
    3001 ms for the BME680; the extra 1 ms is the thread loop's
    `osDelay(1)`.
  * Earlier versions of this project ran the BME280, the BMP390 and the SHT31
    on the same board (2026-10-18), with the same drivers and settings.
  * The MPL3115A2 ran on 2026-10-19 (Saleae captures and the console), in
    barometer and both modes.
  * The console's GUI ran on the board in PuTTY on Windows on 2026-10-19:
    the frame, the MPL3115A2's cell (inverse title, degree sign, values
    matching the readings), the log pane, typed commands on the bottom row
    with Backspace, and Ctrl-L. The arrow keys do nothing visible: the
    home screen only scrolls the cell, whose five labels already fit, so
    whether they arrive hasn't been seen.
  * Not yet run on a chip: HMC6352, DS1307, DS3231, MMC56x3, LSM303DLHC,
    the BMP388, and the PM25 and DS18B20 options. Those drivers have run only
    in Claude-iTransport's host tests, against simulated chips.
* **The bus clear** has run only on the PC: its code against a simulated
  sensor caught at each bit of a byte, one that never lets go, and SCL held
  low. It hasn't yet freed a real stuck bus, and that a stuck bus is why a
  second flash is sometimes needed is a guess, not yet seen in a capture.
  The 9 clocks are from general knowledge of the I2C specification, not
  checked against it here.
* **The bus watch** has run only on the PC, against a simulated I2C1 and
  bus: I2C1 stuck holding SCL low, a sensor holding SDA mid-byte, a dead
  chip holding SDA (and the backoff), a sensor holding SCL, a transfer that
  never ends, and an idle bus left alone. On the board, nobody has yet seen
  it bring I2C1 back.
* **Build:** built with arm-none-eabi-gcc 13 using the CubeIDE flags, without
  warnings, in all three USART1 settings: about 108 KB flash at -Os (180 KB at
  -O0) of 256 KB, and 47 KB RAM of 64 KB, 32 KB of it the FreeRTOS heap.
  Rebuilt on 2026-10-26 with the xPack arm-none-eabi-gcc 13.3.1, the same
  flags (-Os, ITRANSPORT_DEBUG=2), without warnings in all three settings:
  117 to 120 KB flash.
* **Stacks:** estimated from `-fstack-usage`, about 1.3 KB of each sensor
  thread's 2 KB, and less for sensorTask. Newlib's printf frames are a guess,
  not measured.
* **Identification:** how each chip is recognised (the table above) is from
  the drivers and general knowledge. Only the BME280, BMP390, BME680 and
  SHT31 identifications have been seen working on the board.
* **The PMS5003 and DS18B20 options** were built, never wired up.
* **The `.ioc`** was edited by hand to match: I2C1, USART1 (921600, PB6/PB7),
  USART2 with its interrupt, LD3, sensorTask, consoleTask and their mutexes,
  and the 32 KB heap. Open it in CubeMX and check it before generating code
  over the project.

## Fixes

* 2026-10-26: `app_sensor_task()`'s `started[]` was sized
  `F_LSM303_M + 1`, one short since `F_DS3231` was added after it, so a
  DS3231 found at 0x68 wrote a byte past the array on sensorTask's stack.
  `enum Found` now ends with `F_COUNT`, which sizes the array (as the ESP32
  port, ESP32-xSensor-Library's `esp32s3-sensor-bringup`, already did).
  Built, not run on the board.
