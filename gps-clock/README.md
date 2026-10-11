# STM32L432KC GPS Clock

STM32CubeIDE / FreeRTOS example for the STM32L432KC (NUCLEO-L432KC) that combines:

| Part | Library | Interface |
|------|---------|-----------|
| DS1307 real-time clock | `isensor/ds1307`: `xds1307<Stm32HalI2CTransport>` | I2C1, address 0x68 |
| SSD1306 128x64 OLED | `iDisplay/ssd1306`: `xssd1306<Stm32HalI2CTransport>` | I2C1, address 0x3C |
| MTK3333 GPS (e.g. CDtop PA1616D, Quectel L80/L86) | `isensor/mtk3339`: `xmtk3339<Stm32HalUartTransport>` | USART1, 9600 baud |

All three drivers are the non-blocking ones from
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport), copied into
the project unchanged (`iTransport/`, `isensor/`, `iDisplay/`; the commits are in
`iTransport/SOURCE.txt`). The ds1307 driver was written for this example and is on
Claude-iTransport's `claude/ds1307-driver` branch, not yet merged there. The
mtk3339 driver is written for the MT3339; the MT3333 uses the same NMEA and PMTK
commands.

The DS1307 time is shown on the SSD1306 once a second. Once the GPS has a valid fix,
the DS1307 is set from GPS time (UTC), then re-synced every hour.

## Wiring (NUCLEO-L432KC)

| Signal | MCU pin | Nucleo header | Connect to |
|--------|---------|---------------|------------|
| I2C1_SCL | PA9 | D1 | DS1307 SCL, SSD1306 SCL |
| I2C1_SDA | PA10 | D0 | DS1307 SDA, SSD1306 SDA |
| USART1_TX | PB6 | D5 | GPS RX |
| USART1_RX | PB7 | D4 | GPS TX |
| USART2_TX | PA2 | (ST-LINK VCP) | debug output, 115200 8N1 |
| USART2_RX | PA15 | (ST-LINK VCP) | |

* I2C needs pull-ups to 3.3 V (most DS1307 and SSD1306 breakouts already have them).
  If the DS1307 board has pull-ups to 5 V, remove them or use a level shifter.
* The DS1307 needs 5 V on VCC and a backup battery to keep time.

## Display

```
GPS: fix  8 sats
----------------
    12:34:56          <- double size
 Sun 2026-10-11 UTC
 37.77490N 122.41940W
RTC synced 12:30Z
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

The DS1307 and SSD1306 transports share I2C1 and `i2c1_mutex`; each holds it
for one transfer at a time.

## Requirements of the iTransport drivers

* I2C1 event and error interrupts and the USART1 interrupt enabled, priority 5
  (numerically at or above `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`, since they wake
  FreeRTOS threads). Set in the .ioc and `stm32l4xx_hal_msp.c`.
* `USE_HAL_I2C_REGISTER_CALLBACKS` and `USE_HAL_UART_REGISTER_CALLBACKS` are 0:
  the HAL callbacks are defined in `iTransport/hw/stm32/src/Stm32*ItCallbacks.cpp`.
  Don't define `HAL_UART_RxCpltCallback` etc. in `main.cpp`.
* The drivers are built inside their tasks (function statics), after
  `i2c1_mutex` has been created and the UART has been initialised.

## Not verified

* Not run on hardware yet; only built with arm-none-eabi-gcc 13 using the
  CubeIDE flags. The drivers' own host tests (`ds1307_test`, `mtk3339_test`,
  `nmea_parser_test`, `ssd1306_test`) pass at the vendored commits.
* The DS1307 is a 100 kHz part. I2C1 runs at about 100 kHz (`Timing`
  0x00100D14 at a 4 MHz PCLK1); keep it in standard mode if the clock changes.
* Whether every MT3333 module acknowledges PMTK300 has not been checked. If one
  rejects it, the driver reports the configuration as `rejected` on the debug UART
  and carries on with what the module sends.
