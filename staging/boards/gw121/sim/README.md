# A stand-in 121GW: the simulator on an STM32 and a BLE module

`Gw121Simulator.h` runs our own 121GW firmware's meter stack (`meter::Meter`,
`MeterTags`, the key `Controller`, the Bluetooth codec) on made-up readings.
It sends what a 121GW sends: a packet every 250 ms. It takes keys and the
clock as a 121GW does, echo included. With a BLE UART module on an STM32,
the Web Bluetooth page (`staging/tools/web-bluetooth`) and EEVblog's app can
talk to it as they would to a meter, before our firmware is on one.

There is no rotary switch. **SETUP steps the position** (V, mV, Hz, ohms,
µA, mA/A), and MODE steps the functions in it. RANGE (long press: auto),
HOLD and REL work as on the meter.

## Which Adafruit module: the UART one

Use the UART module (Bluefruit LE UART Friend).
* **It is how the 121GW itself is built.** The BLE112 is a transparent UART
  on the STM32's USART2, so the simulator's byte stream is exactly the
  meter's.
* **It needs no driver.** In UART (data) mode, bytes in are bytes out over
  BLE (the Nordic UART service).
* **The SPI module (Bluefruit LE SPI Friend) speaks SDEP,** Adafruit's own
  framing with an IRQ line, which would need a driver first. Keep it for
  later, or for a board with no UART to spare.

### Wiring: a NUCLEO-L432KC (any STM32 with a free UART will do)

| UART Friend | Nucleo | |
|-------------|--------|---|
| VIN | 3V3 | (3.3–16 V) |
| GND | GND | |
| RXI | D1 (PA9, USART1_TX) | |
| TXO | D0 (PA10, USART1_RX) | |
| CTS | GND | the module only takes data while CTS is low |
| MODE switch | UART | data mode, not command mode |

USART1: 9600 baud, 8N1, no flow control (the module's default), with its
global interrupt on. USART2 stays on the ST-LINK's virtual COM port, for
debugging.

The module advertises as "Adafruit Bluefruit LE". The page's Connect lists
it, and finds the Nordic UART service.

If yours is an nRF52 Feather rather than a UART Friend, load Adafruit's
`bleuart` Arduino example on it and wire it the same way: it is the same
transparent UART.

## The CubeIDE project

1. **Create a project** for the NUCLEO-L432KC: STM32CubeMX defaults, C++.
   Enable USART1 on PA9 / PA10 (asynchronous, 9600) and its global
   interrupt.
2. **Add the sources:**
   * `staging/boards/gw121/sim/stm32/gw121_sim_stm32.cpp`
   * `staging/imeter/src/Meter.cpp`
3. **Add the include paths:**
   * `staging/boards/gw121/sim`, `staging/boards/gw121/sim/stm32`, `staging/boards/gw121/inc`
   * `staging/imeter/inc`
   * `staging/isensor/hy3131/inc`
   * `staging/idisplay/input/inc`, `staging/idisplay/segment/inc`
   * an `iDisplay/hmi/inc` and an `iTransport/inc` / `isensor/inc` (any of this repository's vendored copies)
4. **In `main.c`:**

   ```c
   #include "gw121_sim_stm32.h"
   ...
   MX_USART1_UART_Init();
   gw121SimStart(&huart1);
   while (1) { gw121SimLoop(); }
   ```

`gw121_sim_stm32.cpp` defines `HAL_UART_RxCpltCallback`, `TxCpltCallback`
and `ErrorCallback`. If the project has its own, build with
`GW121_SIM_NO_HAL_CALLBACKS` and call `gw121SimUartRxDone` / `TxDone` /
`Error` from them. Both directions are interrupt-driven through ring
buffers, so the loop never waits on the UART. It compiles for the Cortex-M4
with the STM32L4 HAL at about 11 KB of code.

## Tests

`make -C staging/boards/gw121/sim/test test` checks:
* packets, and the echo of each key;
* MODE (AC+DC is skipped, as it isn't a function yet), HOLD, RANGE manual
  and auto, SETUP through the positions, REL;
* the clock.

`make packets` also writes the packets, for the page's decoder test.
