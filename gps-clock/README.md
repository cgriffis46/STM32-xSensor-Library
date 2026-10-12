# STM32L432KC GPS Clock

STM32CubeIDE / FreeRTOS example for the STM32L432KC (NUCLEO-L432KC) that combines:

| Part | Library | Interface |
|------|---------|-----------|
| DS1307 real-time clock | `isensor/ds1307`: `xds1307<Stm32HalI2CTransport>` | I2C1, address 0x68 |
| SSD1306 128x64 OLED | `iDisplay/ssd1306`: `xssd1306<Stm32HalI2CTransport>` | I2C1, address 0x3C |
| MTK3333 GPS (e.g. CDtop PA1616D, Quectel L80/L86) | `isensor/mtk3339`: `xmtk3339<Stm32HalUartTransport>` | USART1, 9600 baud |
| W5500 Ethernet (WIZ850io, W5500-Lite, ...) | `iNetTransport/w5500`: `w5500<Stm32HalSpiBlockTransport>` behind `xEthernet` | SPI1 with DMA, 10 MHz |
| NTP server | `iNetTransport/sntp`: `NtpServer`, on the W5500's UDP responder (socket 6) | UDP port 123 |
| STM32 RTC | `Core/Src/stm32_rtc.cpp` | LSE, PREDIV 7/4095 |
| GPS PPS (optional) | `Core/Inc/pps_math.h`, TIM2 input capture | PA0 (A0) |

The drivers are the non-blocking ones from
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport), copied into
the project unchanged (`iTransport/`, `isensor/`, `iDisplay/`; the commits are in
`iTransport/SOURCE.txt`). Written for this example and in review in
Claude-iTransport ([cgriffis46/Claude-iTransport#34](https://github.com/cgriffis46/Claude-iTransport/pull/34)):
the ds1307 driver, and the W5500's UDP responder and `NtpServer`. The
mtk3339 driver is written for the MT3339; the MT3333 uses the same NMEA and PMTK
commands.

Time flows GPS -> STM32 RTC -> DS1307, and the NTP server serves the STM32 RTC:

* Once the GPS has a fix, its UTC time is applied to the STM32L432's own RTC (on
  the LSE crystal, 1/4096 s resolution) to a fraction of a second, then copied to
  the DS1307 at the start of the next second. Re-synced every hour.
* The display shows the STM32 RTC.
* The W5500 gets an address by DHCP (shown on the display) and answers NTP
  requests on UDP port 123 from the STM32 RTC: stratum 1, reference `GPS`.
* Without a GPS fix, network time (SNTP from `pool.ntp.org`, or the server DHCP
  names) sets both RTCs instead, and the NTP server says stratum 3. A clock set
  from the network is set again from the GPS as soon as there is a fix.
* With the GPS's PPS wired to PA0, the STM32 RTC is disciplined to it (see
  "PPS" below): within about half a millisecond, and the NTP server says
  stratum 1, reference `PPS`, 1 ms dispersion. NMEA timing is the fallback.
* After a power cycle the STM32 RTC starts empty (on the Nucleo, VBAT is tied to
  VDD) and is seeded from the battery-backed DS1307; the NTP server says "not
  synchronised" until GPS or network time arrives. A reset keeps the RTC and,
  in backup registers, where its time came from.

The system clock is 80 MHz (MSI + PLL, trimmed by the LSE crystal).

## Wiring (NUCLEO-L432KC)

| Signal | MCU pin | Nucleo header | Connect to |
|--------|---------|---------------|------------|
| I2C1_SCL | PA9 | D1 | DS1307 SCL, SSD1306 SCL |
| I2C1_SDA | PA10 | D0 | DS1307 SDA, SSD1306 SDA |
| USART1_TX | PB6 | D5 | GPS RX |
| USART1_RX | PB7 | D4 | GPS TX |
| USART2_TX | PA2 | (ST-LINK VCP) | debug output, 115200 8N1 |
| USART2_RX | PA15 | (ST-LINK VCP) | |
| SPI1_SCK | PB3 | D13 | W5500 SCLK (also drives LD3, which flickers) |
| SPI1_MISO | PB4 | D12 | W5500 MISO |
| SPI1_MOSI | PB5 | D11 | W5500 MOSI |
| GPIO out | PA4 | A3 | W5500 SCSn |
| GPIO out | PA3 | A2 | W5500 RSTn |
| EXTI1 | PA1 | A1 | W5500 INTn (pulled up; optional, the driver polls without it) |
| TIM2_CH1 | PA0 | A0 | GPS PPS / 1PPS / FIX output (optional; pulled down; 3.3 V logic) |
| 3V3, GND | | | W5500 3.3 V (about 130 mA) and GND |

* I2C needs pull-ups to 3.3 V (most DS1307 and SSD1306 breakouts already have them).
  If the DS1307 board has pull-ups to 5 V, remove them or use a level shifter.
* The DS1307 needs 5 V on VCC and a backup battery to keep time.
* SPI1 is on PB3/PB4/PB5, not the PA5/PA6/PA7 that Claude-iTransport's
  `stm32l432kc_bringup` uses: on the NUCLEO-L432KC the solder bridges SB16/SB18
  join PA5/PA6 to PB7/PB6, which are the GPS UART here. PA5/PA6 are left
  unconfigured.
* Keep the SPI wires short. SPI1 runs at 10 MHz (prescaler /8); LD3's load on
  SCK is why it isn't faster.

## Display

```
GPS: fix  8 sats
----------------
    12:34:56          <- double size
 Sun 2026-10-11 UTC
 37.77490N 122.41940W
IP 192.168.1.77
RTC synced 12:30Z GPS
```

The last line says where the STM32 RTC's time came from: `GPS`, `NTP` (network),
`RTC from DS1307`, or `RTC not set`.

The DS1307 is kept in UTC. To show local time, set `UTC_OFFSET_MINUTES` in
`Core/Src/main.cpp` (fixed offset, no daylight saving).

## Tasks

* `gpsTask`: runs `xmtk3339::main()`, which sleeps until the UART interrupt
  wakes it at the end of each NMEA line. On start-up (and whenever the module
  reports `$PMTK010,001`) the driver sends PMTK314 (RMC + GGA), PMTK220 (1 Hz)
  and PMTK300, waiting for each `$PMTK001` ack. It publishes the fix, the
  satellites and, with a fix, the UTC time from each RMC.
* `displayTask`: owns the DS1307 and the SSD1306, since neither driver is
  locked. `xds1307::main()` reads the clock every 100 ms in one 7-byte
  transfer. On the first GPS time with a fix (and hourly after) it calls
  `setTime()`, which writes all seven registers in one transfer and reads them
  back. When the second changes it draws the screen into the SSD1306's frame
  buffer and runs `xssd1306::main()` until the flush is out; the driver sends
  only the pages that changed, 32 bytes per interrupt-driven I2C transfer. It
  also prints the time and GPS state to the debug UART.

* `ethTask`: the W5500's driver thread. Pulses RSTn, starts `xEthernet` with
  DHCP and a MAC made from the MCU's unique ID (`02:08:DC:xx:xx:xx`), then runs
  `eth.run()` for good. The W5500 driver runs DHCP, DNS and SNTP on its socket 7;
  sockets 0 and 1 are left for the application (`xClient`). INTn wakes it at once
  through `HAL_GPIO_EXTI_Callback` -> `interruptFromIsr()`.

`displayTask` is the clock task. It owns the DS1307 and the SSD1306, and sets
the STM32 RTC:

1. **GPS** (when due, or whenever the clock came from anywhere else): the RMC's
   second, plus the time since the sentence was parsed and `GPS_NMEA_DELAY_MS`.
   `StmRtc_Set()` writes the date and time in one init-mode session (the second
   starts as it ends), then moves the RTC forward by the fraction with a
   sub-second shift (`RTC_SHIFTR`). The NTP server is told: stratum 1, `GPS`,
   root dispersion 100 ms (NMEA timing without PPS).
2. **Network time**, when the GPS has no fix (and the clock isn't GPS time still
   within 24 h of holdover): `eth.unixTimeMs()`, to the ms. Stratum 3.
3. **The DS1307**, only when the STM32 RTC is empty after a power cycle. Its
   phase within the second is unknown, so it's set to the middle of the second;
   NTP stays "not synchronised".

After 1 or 2 the DS1307 is written from the STM32 RTC in the first 100 ms of a
second. After 24 h without GPS or network time the NTP server goes back to "not
synchronised".

### PPS

Most GPS modules have a pulse-per-second output whose rising edge starts each
UTC second, to well under a microsecond; the RMC sentence that follows names
that second (general MTK behaviour, not checked against the module's datasheet:
the debug line's `nmea=` shows the gap, which should be 100-900 ms). Many MTK3333
modules only pulse once they have a fix. On some breakouts the pin is labelled
`1PPS`, `PPS` or `FIX`.

* TIM2 runs free at 1 MHz (32-bit) and latches each edge in hardware (input
  capture on PA0), so interrupt latency doesn't matter. The last four edges are
  kept.
* Each RMC (timestamped with TIM2 as it is parsed) is paired with the edge just
  before it, within a second; that edge must itself be a second after the one
  before, so a lone or glitched edge is ignored.
* The clock task reads the STM32 RTC and TIM2 together (interrupts masked) and
  works out the RTC's error against the edge. `ppsmath::Discipline` then:
  * sets the RTC outright if it is 200 ms or more off (at start-up);
  * shifts it (`RTC_SHIFTR`) when it is more than 400 us off;
  * every 1024 s, measures the drift (corrections taken out) and adjusts the
    RTC's smooth calibration (`RTC_CALR`, 0.95 ppm steps) to cancel it. The
    calibration survives a reset.
* While locked (under 1 ms), every edge renews the NTP server's reference:
  stratum 1, `PPS`, root dispersion 1 ms. The DS1307 is written hourly.
* No paired edge for 3 s: back to NMEA timing, and the RTC holds the time and
  calibration it was locked to. The NTP server's dispersion grows by 20 ppm
  from the last edge.

`test/pps_math_test.cpp` (on a PC) runs the loop against a simulated RTC whose
crystal is off by -48 to +23 ppm: after six hours each is within 0.5 ppm, and the
phase within 0.54 ms. The display's last line reads `PPS lock -23.8ppm` (the
calibration) while locked; the debug line shows the error, calibration and NMEA
delay every second. Measure that NMEA delay with PPS, then put it in
`GPS_NMEA_DELAY_MS` for when PPS is absent.

The NTP server (`NtpServer`) runs in `ethTask`: the W5500 driver hands it each
request on socket 6 as it is read, it reads the STM32 RTC (receive timestamp,
then again for the transmit timestamp) and the reply goes straight back. Only
client requests (mode 3, NTP v1-v4) are answered; control and private modes
(`ntpq`, `monlist`) get nothing. The debug UART line shows its request and reply
counts. Test it from a PC with `ntpdate -q <board IP>`, `sntp <board IP>` or
`chronyc`/`ntpq` pointed at it.

The DS1307 and SSD1306 transports share I2C1 and `i2c1_mutex`; each holds it
for one transfer at a time. The W5500 has SPI1 and `spi1_mutex` to itself. The
STM32 RTC is read and set under `rtc_mutex` (the NTP server waits at most 2 ms
for it, then drops the request; the client asks again).

Static addressing: set `ETH_DHCP` to 0 and `ETH_STATIC_IP/MASK/GW` in `main.cpp`.

## Requirements of the iTransport drivers

* I2C1 event and error, USART1, SPI1, DMA1 channel 2/3 and EXTI1 interrupts
  enabled, priority 5
  (numerically at or above `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`, since they wake
  FreeRTOS threads). Set in the .ioc and `stm32l4xx_hal_msp.c`.
* `USE_HAL_I2C_REGISTER_CALLBACKS` and `USE_HAL_UART_REGISTER_CALLBACKS` are 0:
  the HAL callbacks are defined in `iTransport/hw/stm32/src/Stm32*ItCallbacks.cpp`
  (`USE_HAL_SPI_REGISTER_CALLBACKS` too).
  Don't define `HAL_UART_RxCpltCallback` etc. in `main.cpp`.
* The drivers are built inside their tasks (function statics), after
  `i2c1_mutex`/`spi1_mutex` have been created and the peripherals initialised.
* SPI1_RX on DMA1 channel 2 and SPI1_TX on DMA1 channel 3 (request 1, normal
  mode, bytes): the W5500 moves up to 1 KB per transfer.
* The FreeRTOS heap is 28 KB: the task stacks and the interface's queue, event
  groups and stream buffers (2 sockets x 2 x 1 KB) come from it.
* W5500 socket buffers: 2 KB each way for sockets 0, 1 (application), 6 (NTP)
  and 7 (DHCP/DNS/SNTP client).

## Not verified

* Not run on hardware yet; only built with arm-none-eabi-gcc 13 using the
  CubeIDE flags (146 KB flash at -O0, 78 KB at -Os; 43 KB RAM). The drivers' own
  host tests (`ds1307_test`, `mtk3339_test`, `nmea_parser_test`, `ssd1306_test`,
  `W5500_test` with the UDP responder cases, `NtpServer_test`, `DhcpClient_test`,
  `DnsClient_test`, `SntpClient_test`, `xNet_test`) pass at the vendored commits,
  and on a PC `test/stm32_rtc_math_test.cpp` (the RTC register arithmetic and the
  sub-second shift, against a model of the RTC from RM0394) and
  `test/pps_math_test.cpp` (pairing, error, shift and calibration registers, and
  the PPS loop against a drifting simulated RTC) pass.
* Without PPS, NTP accuracy is limited by the GPS's NMEA timing: the RMC sentence
  arrives some 100-500 ms after the second it names, depending on the module and
  baud rate. Set `GPS_NMEA_DELAY_MS` (measure it with PPS: `nmea=` in the debug
  line). With PPS the limit is the RTC's 244 us step and the loop's 400 us band.
* The PPS pin and its pairing with RMC haven't been tried on a module.
* The .ioc was edited by hand for SPI1, DMA, the W5500 pins, the 80 MHz clock,
  the RTC (LSE, PREDIV 7/4095), `rtc_mutex`, TIM2 input capture on PA0 and the
  third task. Open it in CubeMX and check it before generating code over
  this project.
* The clock setup expects the LSE crystal (X2, fitted on the NUCLEO-L432KC).
  Without it `SystemClock_Config()` stops in `Error_Handler()`.
* The DS1307 is a 100 kHz part. I2C1 runs at about 100 kHz (`Timing`
  0x10909CEC at an 80 MHz PCLK1); keep it in standard mode if the clock changes.
* Whether every MT3333 module acknowledges PMTK300 has not been checked. If one
  rejects it, the driver reports the configuration as `rejected` on the debug UART
  and carries on with what the module sends.
