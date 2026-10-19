# Feather STM32F405 GPS Clock

STM32CubeIDE / FreeRTOS GPS clock and NTP server on an Adafruit Feather stack,
the next step after the NUCLEO-L432KC proof of concept (in this repository's
history at `eb4b352`, in `gps-clock/`; that project is now `l432kc-sensor-bringup/`). Same
clock: the STM32's own RTC, set from GPS time and disciplined to the GPS's PPS,
served by an NTP server on a W5500. New here: the DS3231's SQW latched on the
same timebase as the PPS, a probe that finds which SQW edge the DS3231's seconds
tick on (with a Saleae logic analyser checking it on the wires), the 128x32 OLED
FeatherWing's buttons, and a USB console.

| Part | Library | Interface |
|------|---------|-----------|
| [Feather STM32F405 Express](https://www.adafruit.com/product/4382) | STM32F405RGT6, 168 MHz from its 12 MHz crystal; 32.768 kHz crystal for the RTC | |
| Ultimate GPS FeatherWing (MTK3333) | `isensor/mtk3339`: `xmtk3339<Stm32HalUartTransport>` | USART3 (Feather TX/RX), 9600 baud |
| GPS 1PPS | `Core/Inc/pps_math.h`, TIM2 input capture | A1 (PA5), **wire needed** |
| Ethernet FeatherWing (W5500) | `iNetTransport/w5500`: `w5500<Stm32HalSpiBlockTransport>` behind `xEthernet` | SPI2 with DMA, 10.5 MHz |
| NTP server | `iNetTransport/sntp`: `NtpServer`, on the W5500's UDP responder | UDP port 123 |
| DS3231 Precision RTC FeatherWing | `Core/Inc/rtc_probe.h` (a probe; a driver comes after the measurement) | I2C1, 0x68 |
| DS3231 SQW (1 Hz) | `Core/Inc/rtc_edge.h`, TIM3 input capture in step with TIM2 | A2 (PA6), **wire needed** |
| 128x32 OLED FeatherWing (SSD1306) + buttons A/B/C | `iDisplay/ssd1306`: `xssd1306<Stm32HalI2CTransport, 32>` | I2C1, 0x3C; D9, D6, D5 |
| Console | USB CDC (virtual COM port) | the Feather's USB-C |

The drivers are copied from [Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport)
as in the L432 example (`feather-f405-gps-clock/iTransport/SOURCE.txt`, with the
ST HAL, CMSIS and USB device library versions).

## Wiring

Stack the wings (a FeatherWing Tripler or Quad, or stacking headers). Most
signals reach the Feather through its headers. Four don't: Adafruit's
schematics put them on pads with no Feather pin, so they need a wire each:

| From | Pad | To Feather pin | MCU pin |
|------|-----|----------------|---------|
| GPS FeatherWing | `1PPS` (JP1) | A1 | PA5, TIM2_CH1 |
| DS3231 FeatherWing | `SQW` (JP2) | A2, plus a 10k pull-up to 3V | PA6, TIM3_CH1 |
| Ethernet FeatherWing | `IRQ` (JP1) | D11 | PC3, EXTI3 |
| Ethernet FeatherWing | `RESET` (JP1) | D12 | PC2 |

Through the headers already:

| Signal | Feather pin | MCU pin |
|--------|-------------|---------|
| GPS TX/RX (via SJ2/SJ3 on the GPS wing) | RX/TX | PB11/PB10, USART3 |
| I2C (DS3231, OLED; both wings carry pull-ups) | SDA/SCL | PB7/PB6, I2C1 at 400 kHz |
| W5500 SPI | SCK/MISO/MOSI | PB13/PB14/PB15, SPI2 |
| W5500 CS (via SJ4 on the Ethernet wing) | D10 | PB9 |
| OLED buttons A, B, C (to ground) | D9, D6, D5 | PB8, PC6, PC7 |
| Red LED: on while locked to the PPS | D13 | PC1 |

Pin facts came from the Eagle schematics in Adafruit's `Adafruit-Feather-STM32F405-Express-PCB`,
`Adafruit-Ultimate-GPS-FeatherWing-PCB`, `Adafruit-DS3231-Precision-RTC-FeatherWing-PCB`,
`Adafruit-Ethernet-FeatherWing-PCB` and `Adafruit-OLED-FeatherWing-PCB` repositories,
CircuitPython's `feather_stm32f405_express/pins.c`, and the alternate functions
from ST's `STM32_open_pin_data` (STM32F405RGTx). A1 (PA5) is the only header pin
on a 32-bit timer; TIM3 (16-bit) is started by TIM2 and counts in step with it,
and each SQW capture is widened to TIM2's 32 bits, so both edges are on one
microsecond timebase. The console prints the two counters' difference at start-up
(it should be 0 or 1).

* SQW/INT is open drain and the DS3231 wing has no pull-up on it: fit 10k to 3V.
  PA6's own pull-up is on too, but it is weak.
* The STM32's VBAT goes only to test pad TP3 on the Feather. With a coin cell
  there the STM32 RTC (and its calibration) survives a power cycle; without it
  the RTC starts empty after power-up, as on the Nucleo.

## Using it

Plug in USB and open the serial port (any baud rate). A terminal opening the
port gets the greeting and the help; then one status line a second:

    RTC 2026-10-12 18:04:07 UTC (PPS) | GPS fix sats=9 cfg=done | IP 192.168.1.73 | NTP synced req=4 rep=4 | PPS locked err=61us cal=-23.8ppm nmea=412ms | DS3231 ok SQW seen at PPS+237.10ms

Keys: `p` runs the probe, `h` prints the help. Buttons: A next page, C previous,
B runs the probe. Pages: the clock (time, date, PPS lock), GPS (fix, position,
PPS error), network (address, NTP counts, NMEA delay), and the DS3231 (its time,
SQW against the PPS, the probe's result).

## Finding the SQW edge with a Saleae

The DS3231's seconds are taken to tick at an SQW edge, but which one, and how
soon after it, is unverified here (its datasheet couldn't be fetched). Two
independent measurements, from the same run:

1. Clip the Saleae on: **SQW** (A2) on channel 0, **SCL** and **SDA** on
   channels 1 and 2, **1PPS** (A1) on channel 3. Ground to the Feather's GND.
   Sample digital at 10 MS/s or more.
2. Add the I2C analyser (SCL 1, SDA 2). Start a capture of about 20 s.
3. Press **B** (or type `p`). For 12 s the firmware reads the DS3231's seconds
   every 20 ms and prints every read, every SQW edge and every bracketed tick
   (`probe ...` lines), then its own answer, for example:

       probe result: the seconds follow the falling edge of SQW, -700..+5550 us after it (12 ticks, 601 reads)

4. In Logic 2 export the I2C analyser's table (CSV) and the digital channels
   (CSV), then run Claude-iTransport's tool:

       python3 tools/saleae/saleae_log.py --rtc-edge --i2c i2c.csv --digital digital.csv --sqw 0 --pps 3

   It brackets the ticks from the reads on the wire, and also reports the SQW
   against the PPS (offset and drift in ppm).

The two answers should agree. If they do, the DS3231 driver will take that edge
(and its delay) as fact. A narrower bracket comes from a faster probe
(`RTC_PROBE_PERIOD_MS`) or a longer one. Send the CSV exports if anything
disagrees: the export formats the tool reads are from memory of Saleae's
documentation and haven't met a real export yet.

A square wave can't, by itself, tell "0.3 ms after the falling edge" from
"500.3 ms after the rising edge". Both tools take the edge whose bracket lies
nearest zero.

## Host tests

`test/rtc_edge_test.cpp` (the probe's bracketing, against a simulated RTC
ticking on either edge, across the timer's wrap), `test/pps_math_test.cpp` and
`test/stm32_rtc_math_test.cpp` (as in the L432 example). Build commands are in
each file's header.

## Not verified

* Not run on hardware. Built with arm-none-eabi-gcc 13 using the CubeIDE flags,
  no warnings: 89 KB flash at -Os (171 KB at -O0), 75 KB RAM of 128 KB (48 KB of
  it the FreeRTOS heap, 8 KB the console ring).
* `feather-f405-gps-clock.ioc` was written by hand, not by CubeMX, and there is
  no `.mxproject`: open it in CubeMX and check it before generating code over the
  project. The USB device files (`USB_DEVICE/`) were written in CubeMX's shape,
  not generated.
* From general knowledge, not checked in RM0090 here: TIM3's ITR1 is TIM2's
  trigger output (the start-up line shows whether the counters are in step), and
  SPI2's DMA requests are DMA1 stream 3 (RX) and stream 4 (TX), channel 0.
* The DS3231 control value (0x0E = 0x00: oscillator on, 1 Hz on SQW/INT) is from
  memory of its datasheet; the probe shows whether SQW appears.
* USB: ST's development VID/PID (0x0483/0x5740), as CubeMX uses for a virtual COM
  port. No VBUS sensing (the Feather doesn't take VBUS to PA9).
* The STM32 RTC code is the L432's: the F405's RTC has the same registers here
  (sub-seconds, shift, smooth calibration), and the F4 HAL leaves a running
  calendar alone in `HAL_RTC_Init()` too, but it has not run on an F405.
