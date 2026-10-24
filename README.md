# STM32 xSensor Library

STM32CubeIDE / FreeRTOS examples built on
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport)'s
non-blocking sensor, network and display drivers. The drivers live in
Claude-iTransport, with their host tests; each project here carries a copy of
the ones it uses, recorded in its `iTransport/SOURCE.txt`.

| Folder | Board | What it is |
|--------|-------|------------|
| [`l432kc-sensor-bringup/`](l432kc-sensor-bringup/README.md) | NUCLEO-L432KC | Plug in any supported I2C sensor, reset, and read it. 15 kinds of chip are recognised by address and ID: AHT20, BME280, BME680, BMP280, BMP390, DS1307, DS3231, HMC6352, HTU21D, LPS35HW, LSM303DLHC, MMC56x3, MPL3115A2, SHT31 and Si7021. A PMS5003 or a DS18B20 is a build option. |
| [`gps-clock-f207/`](gps-clock-f207/README.md) | NUCLEO-F207ZG + X-NUCLEO-GNSS2A1 | GPS clock and NTP server on the board's own Ethernet |
| [`gps-clock-f405/`](gps-clock-f405/README.md) | Adafruit Feather STM32F405 | GPS clock and NTP server on a W5500, with an OLED |
| [`STM32L476RG_MC2515_Example/`](STM32L476RG_MC2515_Example/README.md) | NUCLEO-L476RG + SparkFun CAN-BUS shield | Sends the shield's joystick over CAN (MCP2515, 500 kbit/s), for an L432KC with an OLED to show |

## The old examples

This repository used to hold one CubeIDE project per sensor for the
STM32L432KC, each with its own blocking driver: aht20, bme280, bmp280, bmp390,
HMC6352, htu21df, lps35hw, mpl3115a2, PM25, sht31, si7021, rtc-lib (DS1307),
DS18B20, and can_test. They were retired on 2026-10-19 in favour of the bring-up
project above, which runs every one of those sensors on Claude-iTransport's
drivers.

The tag `legacy-2026-10` marks the repository as it was before:

    git checkout legacy-2026-10
