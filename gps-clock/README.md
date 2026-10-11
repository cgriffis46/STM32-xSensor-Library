# STM32L432KC GPS Clock

STM32CubeIDE / FreeRTOS example for the STM32L432KC (NUCLEO-L432KC) that combines:

| Part | Library | Interface |
|------|---------|-----------|
| DS1307 real-time clock | `RTC-lib` (RTClib port) | I2C1, address 0x68 |
| SSD1306 128x64 OLED | `SSD1306` | I2C1, address 0x3C |
| MTK3333 GPS (e.g. Adafruit Ultimate GPS, Quectel L80/L86) | `MTK3333` | USART1, 9600 baud |

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

* `gpsTask`: parses NMEA from the GPS and sets the DS1307 when there is a valid fix.
* `displayTask`: reads the DS1307 and redraws the SSD1306 when the second changes,
  and prints the time to the debug UART.

The DS1307 and SSD1306 share I2C1, so every I2C access holds `i2c1_mutex`.

## Libraries

### SSD1306 (`SSD1306/`)

```cpp
SSD1306 oled(&hi2c1);               // 0x3C, 128x64
oled.begin();
oled.clear();
oled.drawString(0, 0, "Hello");
oled.drawStringCentered(20, "12:34", 2);   // 2x scale
oled.display();                     // push the frame buffer over I2C
```

### MTK3333 (`MTK3333/`)

Bytes are received by interrupt into a ring buffer; `update()` parses RMC and GGA
sentences (any talker: GP/GN/GL).

```cpp
MTK3333 gps(&huart1);

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { gps.rxCpltCallback(huart); }
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)  { gps.errorCallback(huart); }

gps.begin();
gps.setNmeaOutput(true, true);      // RMC + GGA
gps.setUpdateRate(1000);            // 1 Hz

for (;;) {
  gps.update();
  if (gps.newData()) {
    GPS_Data d;
    gps.getData(&d);                // d.fix, d.hour..., d.latitudeE6, d.satellites, ...
  }
  osDelay(20);
}
```

The USART1 interrupt must be enabled (done in `stm32l4xx_hal_msp.c` / the .ioc).
