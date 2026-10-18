# STM32L432KC DS1307 Bring-up

STM32CubeIDE / FreeRTOS project for the NUCLEO-L432KC with a DS1307 real-time
clock and nothing else: the first live test of the GPS clock's parts. It runs the
non-blocking DS1307 driver from
[Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport)
(`isensor/ds1307`, as `xds1307<Stm32HalI2CTransport>`; `iTransport/SOURCE.txt`
has the commit). A console runs on the ST-LINK's USB port.

The full L432 GPS clock this project used to be (MTK3333 GPS, SSD1306 OLED,
W5500, NTP server, PPS discipline) is in this repository's history at commit
`eb4b352`. The Feather F405 and NUCLEO-F207ZG examples carry that design on.

## Wiring (NUCLEO-L432KC)

| DS1307 board | Nucleo pin | MCU pin | |
|--------------|------------|---------|---|
| SCL | **D1** | PA9, I2C1_SCL | |
| SDA | **D0** | PA10, I2C1_SDA | |
| VCC | **5V** | | the DS1307 needs 4.5-5.5 V |
| GND | **GND** | | |
| SQW (or SQW/OUT) | **A1** | PA1, TIM2_CH2 | optional; needs a 10k pull-up to **3V3** |

* **I2C pull-ups.** SDA and SCL need pull-ups. Most DS1307 boards fit them
  (often 3.3k-10k) to the board's VCC, which is 5 V here. PA9 and PA10 are
  5 V tolerant, and the DS1307 reads 3.3 V as high, so this works (general
  knowledge of the L432's datasheet, not checked here). If the board has no
  pull-ups, fit 4.7k from each line to **3V3**.
* **SQW is open drain.** Pull it up to 3V3, never to 5V. PA1's own weak
  pull-up is on as well.
* **The battery.** A CR2032 (or the board's cell) keeps the time and the RAM
  through a power cycle. Many "Tiny RTC" boards charge a rechargeable LIR2032
  through a diode and resistor; with a plain CR2032 in one, remove that
  charging path, or the cell can be damaged (general knowledge).
* **The Nucleo pins.** D0/D1 = PA10/PA9 and A1 = PA1 are from memory of
  ST's UM1956, not checked here. The console is USART2 (PA2/PA15) on the
  ST-LINK's virtual COM port, so D0/D1 are free.

## Using it

1. Flash it (STM32CubeIDE, or STM32CubeProgrammer with the `.elf`).
2. Open the ST-LINK's virtual COM port at 115200 8N1. Turn on local echo: the
   board doesn't echo what you type.
3. After the help text, it prints one line per DS1307 second:

       Sun 2026-10-18 12:34:56 | DS1307 ok | SQW 1000003us high=500001us avg +2.850ppm/64s | tick +0.120..+52.480ms after falling edge (60)

   * `DS1307 ok` means it answers at 0x68. `NOT ANSWERING` means check SDA/SCL,
     the pull-ups and 5 V.
   * `STOPPED (CH set)` means the oscillator isn't running. That's normal for a
     new chip or one whose battery was out: set the time and it starts.
   * `SQW ...` is the 1 Hz square wave on A1:
     * its last period in µs;
     * how long it was high;
     * the average rate against the Nucleo's 32.768 kHz crystal (which trims
       the 80 MHz clock that TIM2 counts). Both crystals are ±20 ppm or so, so
       this is the two crystals' difference, not the DS1307's own error.
   * `tick ...` is where the DS1307's seconds change against its SQW edges.
     It comes from the 50 ms reads and the latched edges (`Core/Inc/rtc_edge.h`,
     from the F405 example), and it narrows as more ticks are seen. The answer
     expected is a small window just after the **falling** edge (general
     knowledge, not from the datasheet).
4. The LED LD3 (green) toggles each second.
5. At start-up the board reads a start counter from the DS1307's
   battery-backed RAM, adds one and writes it back. A count that keeps rising
   across a power cycle shows that the RAM, and so the battery, works.

### Commands

| Command | |
|---------|---|
| `t 2026-10-18 12:34:56` | set the DS1307 (24 hour; kept as UTC by the clock examples). Starts a stopped oscillator. |
| `b` | set it to this firmware's build time (the build PC's local time) |
| `r` | show the 56 bytes of RAM (0x08..0x3F) |
| `p` | start the tick measurement again |
| `h` | the help |

### With the Saleae

To check the falling edge precisely, put channels on the following lines:

* SQW (A1)
* SCL (D1)
* SDA (D0)

Sample at 10 MS/s, add the I2C analyser, and capture 20 s or more. Then export
the I2C table and the digital channels as CSV and run Claude-iTransport's tool:

    python3 tools/saleae/saleae_log.py --rtc-edge --i2c i2c.csv --digital digital.csv --sqw 0

The board reads every 50 ms, which is enough for the tool to bracket each tick.
Its answer and the board's `tick` line should agree. Send the exports if they
don't. The export formats the tool reads haven't met a real export yet.

## Inside

* **rtcTask:**
  * runs `xds1307::main()`, which reads the seven time registers in one
    transfer every 50 ms and writes the control register (0x10: 1 Hz on
    SQW/OUT) at start;
  * takes the console's commands (`setTime()`, `readRam()`, `writeRam()`);
  * follows the SQW edges;
  * prints the status line.
* **consoleTask:** hands the console ring to USART2 through
  `Stm32HalUartTransport`, which also takes the keys.
* **TIM2** runs free at 1 MHz and latches both SQW edges on PA1 in hardware.
* **The system clock** is 80 MHz (MSI + PLL, trimmed by the LSE crystal X2).
  Without the LSE, `SystemClock_Config()` stops in `Error_Handler()`.
* **I2C1** runs at 100 kHz (`Timing` 0x10909CEC), the DS1307's limit.

## Host tests

`test/rtc_edge_test.cpp` (the tick bracketing, against a simulated RTC ticking on
either edge, across the timer's wrap). The build command is in its header. The
DS1307 driver's own `ds1307_test` is in Claude-iTransport.

## Not verified

* Not run on the board yet. Built with arm-none-eabi-gcc 13 using the CubeIDE
  flags, without warnings: 47 KB flash at -Os (76 KB at -O0) and 26 KB RAM.
* The `.ioc` was edited by hand to match: I2C1, USART2 (with its interrupt),
  TIM2 channel 2 on both edges, LD3, and the two tasks and mutexes. Open it in
  CubeMX and check it before generating code over the project.
* The DS1307's control value (0x10 for 1 Hz) and its register layout are from
  the datasheet (Maxim 19-5215), via the driver. That its seconds tick at the
  falling edge of SQW is general knowledge: this test is how it gets checked.
