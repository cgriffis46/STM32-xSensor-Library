# STM32L432KC GPS Clock

STM32CubeIDE / FreeRTOS example for the STM32L432KC (NUCLEO-L432KC) that combines:

| Part | Library | Interface |
|------|---------|-----------|
| DS1307 real-time clock | `isensor/ds1307`: `xds1307<Stm32HalI2CTransport>` | I2C1, address 0x68 |
| SSD1306 128x64 OLED | `iDisplay/ssd1306`: `xssd1306<Stm32HalI2CTransport>` | I2C1, address 0x3C |
| MTK3333 GPS (e.g. CDtop PA1616D, Quectel L80/L86) | `isensor/mtk3339`: `xmtk3339<Stm32HalUartTransport>` | USART1, 9600 baud |
| W5500 Ethernet (WIZ850io, W5500-Lite, ...) | `iNetTransport/w5500`: `w5500<Stm32HalSpiBlockTransport>` behind `xEthernet` | SPI1 with DMA, 10 MHz |

All four drivers are the non-blocking ones from
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport), copied into
the project unchanged (`iTransport/`, `isensor/`, `iDisplay/`; the commits are in
`iTransport/SOURCE.txt`). The ds1307 driver was written for this example and is on
Claude-iTransport's `claude/ds1307-driver` branch, not yet merged there. The
mtk3339 driver is written for the MT3339; the MT3333 uses the same NMEA and PMTK
commands.

The DS1307 time is shown on the SSD1306 once a second. Once the GPS has a valid fix,
the DS1307 is set from GPS time (UTC), then re-synced every hour. The W5500 gets an
address by DHCP (shown on the display) and keeps network time by SNTP
(`pool.ntp.org`, or the server DHCP names); when the GPS has no fix, that time
sets the DS1307 instead.

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

`displayTask` shows the address (or `ETH: no link` / `waiting for DHCP` /
`no W5500`) and, when the GPS has no fix, sets the DS1307 from
`eth.unixTimeMs()`, only in the first 150 ms of a second since writing the seconds
register restarts the DS1307's 1 Hz divider. The status line says which source
set the clock (`GPS` or `NTP`). A clock set from NTP is set again from the GPS
as soon as the GPS has a fix.

The DS1307 and SSD1306 transports share I2C1 and `i2c1_mutex`; each holds it
for one transfer at a time. The W5500 has SPI1 and `spi1_mutex` to itself.

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
* The FreeRTOS heap is 24 KB: the interface's queue, event groups and stream
  buffers (2 sockets x 2 x 1 KB) come from it.

## Not verified

* Not run on hardware yet; only built with arm-none-eabi-gcc 13 using the
  CubeIDE flags (126 KB flash at -O0, 69 KB at -Os; 38.7 KB RAM). The drivers' own
  host tests (`ds1307_test`, `mtk3339_test`, `nmea_parser_test`, `ssd1306_test`,
  `W5500_test`, `DhcpClient_test`, `DnsClient_test`, `SntpClient_test`, `xNet_test`)
  pass at the vendored commits.
* The .ioc was edited by hand for SPI1, DMA, the W5500 pins, the 80 MHz clock and
  the third task. Open it in CubeMX and check it before generating code over
  this project.
* The clock setup expects the LSE crystal (X2, fitted on the NUCLEO-L432KC).
  Without it `SystemClock_Config()` stops in `Error_Handler()`.
* The DS1307 is a 100 kHz part. I2C1 runs at about 100 kHz (`Timing`
  0x10909CEC at an 80 MHz PCLK1); keep it in standard mode if the clock changes.
* Whether every MT3333 module acknowledges PMTK300 has not been checked. If one
  rejects it, the driver reports the configuration as `rejected` on the debug UART
  and carries on with what the module sends.
