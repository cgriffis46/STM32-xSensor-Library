# NUCLEO-L476RG CAN joystick (MCP2515)

A NUCLEO-L476RG with a SparkFun CAN-BUS shield (v13) reads the shield's
joystick and sends its position over CAN. The second node, an NUCLEO-L432KC
with a TJA1051T/3 transceiver and an SSD1306 OLED, shows it (a separate
project, not written yet).

The CAN controller is the shield's MCP2515 (16 MHz crystal), driven by
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport)'s
non-blocking `xmcp2515` driver in a FreeRTOS thread of its own. The drivers
are vendored in `STM32L476RG_MC2515_Example/iTransport`, `isensor` and
`iCan` by `sync_from_claude_itransport.sh`, which records the commit in
`iTransport/SOURCE.txt`:

    sh sync_from_claude_itransport.sh /path/to/Claude-iTransport

## On the bus

500 kbit/s, sample point 87.5 %: from the 16 MHz crystal, prescaler 1 (TQ = 2 / 16 MHz = 125 ns), 16 TQ a bit (sync 1, prop 6, PS1 7, PS2 2), SJW 1.

| ID | DLC | Data |
|----|-----|------|
| 0x100 (standard) | 1 | bit 0 up, bit 1 down, bit 2 left, bit 3 right, bit 4 click (1 = pressed) |

Sent when the position changes (read every 10 ms, taken once two reads in a
row agree) and every second as a heartbeat. A receiver that hears nothing for
2 s should say so. Only one frame waits to be sent at a time: a change made
while one is still waiting goes right after it.

## The console

USART2, the ST-LINK's virtual COM port, 115200 8N1. It prints the bit timing
at start-up, each change of the joystick, each frame received, and the
controller's state every 5 s:

    can error-active  tec 0 rec 0  tx 42 rx 0  pending 0  dropped 0 overflows 0  bus-off 0 restarts 0 faults 0

Keys: `s` status now, `h` help.

## LEDs (on the shield)

* LED1 (D7, PA9) toggles on each frame another node acknowledged.
* LED2 (D8, PA8) is lit while the controller isn't error-active.

With no other node on the bus, no one acknowledges. The MCP2515 then sends
the same frame again and again and goes error-passive (TEC 128): LED2 lights
and LED1 stays put. That is expected until the second node is connected.

## Loopback

Add `APP_CAN_LOOPBACK=1` to the G++ compiler's defines (Project > Properties >
C/C++ Build > Settings > MCU/MPU G++ Compiler > Preprocessor). The MCP2515
then hears its own frames and nothing goes on the bus: a test of the shield
and the SPI alone. Each frame sent shows up on the console as `rx 0x100 [1] ..`.

## Wiring and CubeMX

| Signal | Shield | Nucleo pin | CubeMX |
|--------|--------|------------|--------|
| SPI1 SCK, MISO, MOSI | D13, D12, D11 | PA5, PA6, PA7 | SPI1, full-duplex master, mode 0, prescaler 16 (5 MHz), global interrupt on (priority 5) |
| CAN_CS | D10 | PB6 | GPIO output, starts high |
| CAN_INT | D2 | PA10 | GPIO EXTI, falling edge; EXTI15_10 on (priority 5) |
| JOY_UP, JOY_LEFT, JOY_DOWN | A1, A2, A3 | PA1, PA4, PB0 | GPIO input, pull-up |
| JOY_CLICK, JOY_RIGHT | A4, A5 | PC1, PC0 | GPIO input, pull-up |
| LED2, LED1 | D8, D7 | PA8, PA9 | GPIO output |
| Console | - | PA2, PA3 | USART2, 115200, global interrupt on (priority 5) |

The joystick switches pull to ground and the shield has no pull-ups, so the
MCU's own are used. FreeRTOS (CMSIS_V2) with a 16 KB heap; TIM6 as the HAL's
timebase; the default task's stack 512 words. Keep
`USE_HAL_SPI_REGISTER_CALLBACKS` and `USE_HAL_UART_REGISTER_CALLBACKS` at 0:
the callbacks are in `iTransport/hw/stm32/src`.

CAN_H and CAN_L go to the shield's screw terminal (or the DB9). Put 120 ohm
across them at each end of the bus.

The shield runs the MCP2515 and the MCP2551 on 5 V. At 5 V the MCP2515 wants
3.5 V for a high on SCK, SI and CS (datasheet table 13-1), so the L476's 3.3 V
is out of its specification (it often works; a 74HCT buffer fixes it), and SO
(MISO) and INT come back at 5 V, so PA6 and PA10 must be 5 V tolerant (FT)
pins: check the L476's datasheet.

## Code

CubeMX's files are untouched except for one call in `Core/Src/freertos.c`'s
USER CODE block (`app_default_task(argument)`), so regenerating code keeps
it. Everything else is in `Core/Src/app.cpp`:

* the default task starts the CAN thread, then reads the joystick, sends,
  prints and runs the LEDs every 10 ms;
* the CAN thread runs `xmcp2515::main()`, sleeping until INT, a request or
  its 10 ms poll;
* `HAL_GPIO_EXTI_Callback` passes INT's falling edge to the driver.

Bus-off: the driver goes back on the bus after a second by itself
(`autoRestart`).

## Status

Built with arm-none-eabi-gcc from the `.cproject`'s settings (-O0 and -Os,
with and without loopback, no warnings in the application or the drivers):
38 KB flash at -Os, 26 KB RAM with the 16 KB FreeRTOS heap. Not opened in
STM32CubeIDE here, and not run on the board yet. The driver's own tests (a
simulated MCP2515 and bus) are in Claude-iTransport.
