# NUCLEO-F207ZG GPS Clock

STM32CubeIDE / FreeRTOS GPS clock and NTP server on a NUCLEO-F207ZG, with ST's
X-NUCLEO-GNSS2A1 (Teseo-VIC3DA) on its Arduino connector and the board's own
Ethernet. It follows `gps-clock/` (NUCLEO-L432KC) and `gps-clock-f405/` (Feather
STM32F405). What's new here:

* **The clock is the Ethernet MAC's IEEE 1588 system time, not the RTC.** The
  F2's RTC has no sub-second register, no shift and no smooth calibration
  (checked in ST's `stm32f207xx.h`: no `RTC_SSR`, `RTC_SHIFTR` or `RTC_CALR`), so
  the F405's way of disciplining the RTC can't work here. The MAC's system time
  counts seconds and 2^-31 s from HCLK. Its rate is set by an addend, to about
  half a part per billion. A PI loop steers that rate to the GPS's PPS: phase
  errors are slewed out, not stepped. This is also the first step towards PTP
  hardware time stamps.
* **lwIP 2.2.1 on the F207's MAC and the LAN8742A PHY** (the setup from ST's
  NUCLEO-F207ZG example), with the NTP server on lwIP's raw UDP API: requests are
  answered in lwIP's tcpip thread.
* **The Teseo driver** (`isensor/teseo`, from Claude-iTransport): it sets the
  NMEA messages over `$PSTM` commands.

| Part | Library | Interface |
|------|---------|-----------|
| [NUCLEO-F207ZG](https://www.st.com/en/evaluation-tools/nucleo-f207zg.html) | STM32F207ZG, 120 MHz from the ST-LINK's 8 MHz; 32.768 kHz LSE for the RTC | |
| X-NUCLEO-GNSS2A1 (Teseo-VIC3DA) | `isensor/teseo`: `xteseo<Stm32HalUartTransport>` | USART6 (D0/D1), 115200 baud |
| GNSS PPS | `Core/Inc/ptp_math.h`, TIM1 input capture in step with TIM2 | D6 (PE9, TIM1_CH1) |
| GNSS reset | | D7 (PF13) |
| Ethernet (LAN8742A, RMII) | lwIP 2.2.1, `LWIP/Target/ethernetif.c` | the board's RJ45 |
| The clock | `Core/Inc/ptp_clock.h`: the MAC's IEEE 1588 system time | |
| NTP server | `iNetTransport/sntp`: `NtpServer`, on lwIP's raw UDP API | UDP port 123 |
| RTC (whole seconds across a reset) | `Core/Inc/f2_rtc.h` | LSE |
| Console | `Stm32HalUartTransport` | USART3, the ST-LINK's virtual COM port, 115200 8N1 |
| LEDs | LD1 green: locked to the PPS; LD2 blue: the network has an address; LD3 red: a fault | PB0, PB7, PB14 |

The drivers are copied from [Claude-iTransport](https://github.com/cgriffis46/Claude-iTransport)
as in the other examples (`nucleo-f207zg-gps-clock/iTransport/SOURCE.txt`, with the
ST HAL, CMSIS, FreeRTOS and lwIP versions).

## The shield's jumpers

Set them as ST's X-CUBE-GNSS1 readme gives them for a Nucleo-144:

* **1-2:** J23, J25, J28, J29
* **2-3:** J24, J26, J27, J30
* **Closed:** J11, J12, J14, J15

That puts:

* the module's UART on D0/D1;
* the PPS on D6 (J29 1-2);
* the reset on D7 (J25 1-2).

The jumper table is in ST's UM3020.

| Signal | Arduino pin | MCU pin |
|--------|-------------|---------|
| Teseo TX -> MCU RX | D0 | PG9, USART6_RX |
| MCU TX -> Teseo RX | D1 | PG14, USART6_TX |
| PPS | D6 | PE9, TIM1_CH1 (AF1) |
| Reset (low = reset) | D7 | PF13 |

The Nucleo's Arduino pin mapping and the ST-LINK virtual COM port on USART3
(PD8/PD9) are from memory of ST's UM1974, not checked here.

## How the time is kept

1. **At start-up.** If the RTC has been set (it keeps running through a reset),
   the clock task waits for the RTC's next second and starts the system time
   from it. If the backup registers say the time came from GPS within the last
   24 hours, the NTP server serves it straight away, as stratum 1 with a 3 ms
   root dispersion. Otherwise it answers "unsynchronised" (stratum 16), which
   clients ignore.
2. **With a fix and the PPS.** Each PPS edge is latched by TIM1. TIM1 is 16-bit
   and is started by TIM2 (32-bit, 1 MHz), so the two count in step. The
   capture interrupt reads the system time together with TIM2, so the clock's
   error at the edge is known to about a microsecond, however late the
   interrupt runs. The RMC that follows names the second.
   * An error of 1 ms or more sets the clock outright.
   * A smaller one goes through the PI loop (`ptpmath::Discipline`):
     * the frequency integrates the error at 1000/256 ppb per microsecond;
     * the phase term is -125 ppb per microsecond;
     * the loop is critically damped, with a time constant of about 16 s.
   * After 8 seconds within 10 µs it is **locked**. The NTP server is then
     stratum 1, reference "PPS", root dispersion 10 µs.
   * While locked, the RTC is restarted on a whole second of the clock every
     minute, and the frequency is saved to a backup register every hour.
3. **Without the PPS** (no fix yet, or J29 open):
   * When the PPS stops, the clock runs on at the learned frequency.
   * After an hour without it, NMEA time takes over. It is set from each RMC
     every 10 minutes (`GPS_NMEA_DELAY_MS` corrects for how late the RMC
     comes) and served as stratum 1, "GPS", with a 100 ms root dispersion.
4. **Holdover.** If the time was last set from GPS more than 24 hours ago, the
   server says "unsynchronised" again.

The host test `test/ptp_math_test.cpp` simulates the MAC's system time on a
crystal 37 ppm fast, with the clock set 0.3 s wrong:

* It locks in about 100 s and stays within 3 µs.
* It finds the frequency to within a few ppb.
* It holds for a minute with no PPS.
* It follows a 2 ppm step within 12 µs.

## Using it

Plug in the ST-LINK USB and open its virtual COM port at 115200 8N1. At start-up
the board prints a greeting, the Teseo's firmware version, its configuration
result and the network state. Then it prints one line a second:

    2026-10-12 18:04:07.000 UTC (PPS) | GPS fix sats=9 cfg=done | IP 192.168.1.73 (100M FD) | NTP synced req=4 rep=4 | PPS locked err=1us rate=-12.345ppm nmea=412ms

* `err` is the clock against the last PPS edge.
* `rate` is the correction the clock runs at (the HSE crystal's error, turned
  around).
* `nmea` is how long after its PPS edge the RMC arrived.
* `h` prints the help again.

The greeting also shows TIM1 against TIM2 at start-up: it should be within one
count. If it says MISMATCH, TIM1's trigger isn't TIM2's (see below).

From a PC on the same network:

    ntpdate -q 192.168.1.73          # or: sntp 192.168.1.73
    chronyc -h ... / chrony.conf: server 192.168.1.73 iburst

Replies are time stamped in software, in the tcpip thread. Expect tens of
microseconds of asymmetry on top of the network's own. PTP hardware time stamps
(the MAC's receive and transmit descriptors) would remove that, and are the
next step.

## Host tests

* `test/ptp_math_test.cpp`: the system time's arithmetic (units, increment and
  addend, error measurement, PPS pairing and widening) and the control loop
  against a simulated MAC on a crystal that is off frequency.
* `test/stm32_rtc_math_test.cpp`: the L432 example's RTC arithmetic. Only its
  TR/DR conversions are used on the F2.

Build commands are in each file's header.

## Not verified

* **Not run on hardware.**
  * Built with arm-none-eabi-gcc 13 using the CubeIDE flags (Cortex-M3, no
    FPU, gnu++14), with no warnings.
  * Size: 66 KB flash at -Os (117 KB at -O0) and 74 KB RAM of 128 KB.
  * Of the RAM, 24 KB is the FreeRTOS heap, 15 KB the Ethernet DMA buffers
    and about 20 KB lwIP's.
* **The MAC's system time.** Register and bit names are from ST's
  `stm32f207xx.h`. How they behave (fine update, the addend, TSSTI, binary
  rollover with 2^31 sub-seconds) is from general knowledge of this MAC (the
  F407 has the same one) and ST's AN3411 code, not from RM0033. Its update
  form for subtracting is never used, because sources describe it
  differently.
* **TIM1's ITR1 is TIM2's trigger output.** This is RM0033's internal trigger
  table from general knowledge; the start-up line shows whether the counters
  are in step.
* **The Teseo-VIC3DA's PPS.** It is assumed on by default, rising at the start
  of the UTC second that the following RMC names. ST's code doesn't set it, and
  this is not checked against ST's documents. If the console says `PPS none`
  while the fix is good, check J29 and the module's PPS settings. The Teseo
  driver's own unverified points (the `$PSTMSETPAR` value format, whether
  `$PSTMSRR` is needed after a save) are in its header.
* **Ethernet.**
  * `HAL_ETH_Init()` is called with autonegotiation off and the PHY switched
    back to autonegotiation after it, so the 10 s wait doesn't happen and a
    late cable never needs `HAL_ETH_Init()` again (that would reset the
    system time).
  * Speed and duplex come from the LAN8742A's register 31, as ST's HAL reads
    them.
* **The RTC.**
  * The LSE crystal (X2) is assumed fitted. If it doesn't start, there is no
    RTC; the clock still runs.
  * The second starts about 4 LSE cycles after init mode ends (general
    knowledge).
  * The NUCLEO-F207ZG ties VBAT to VDD, so the RTC and backup registers survive
    a reset but not a power cycle (general knowledge of UM1974).
* **CubeMX.** There is no `.ioc`, and the project is not meant to be
  regenerated by CubeMX: lwIP and `ethernetif.c` are set up by hand, not by
  CubeMX's LWIP middleware, which would write over them. The project's
  STM32Cube nature was removed for that reason. The pins are listed above and
  in `Core/Inc/main.h`. It has not been opened in STM32CubeIDE.
* **PPS output.** The F207's ETH_PPS_OUT is not used. ST's F2 header has no PPS
  control register, and whether the MAC drives that pin isn't known here. Its
  1 Hz output would be the way to check the clock against the GPS's PPS on a
  logic analyser.
