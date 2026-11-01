# HY3131 / HY3130 DMM analog front end

A non-blocking, interrupt-driven driver for Hycon's HY3131 (50,000 counts) and
HY3130 (5,000 counts) multimeter front ends, written to Claude-iTransport's
conventions: a template on its transport (`iBlockTransport`, so
`Stm32HalSpiBlockTransport` on STM32), a `SensorStateMachine` that never waits
on the bus, and `xhy3131` for CMSIS-RTOS2 (FreeRTOS, Zephyr).

It is here until it moves to Claude-iTransport as `isensor/hy3131`. This
session couldn't reach that repository.

Written from Hycon's datasheet DS-HY3131-V10_EN. **Not yet run against a
chip.** Tested on the host against a simulated chip (`test/`).

| File | |
|------|---|
| `inc/Hy3131Regs.h` | Registers, bits, SPI framing, value decoding |
| `inc/hy3131.h`, `src/hy3131.tpp` | `hy3131_dmm<TTransport>`, `hy3131::Config`, `Sample`, `Count`, `iListener` |
| `inc/xhy3131.h` | The CMSIS-RTOS2 version: sleeps on a thread flag, `onIrq()` wakes it |
| `test/` (with `SimHy3131.h`, the simulated chip) | `make -C staging/isensor/hy3131/test test` |

## Using it

```cpp
hy3131_param_t p = hy3131_default_param();
p.irq = true;                       // DO's rising edge is on an EXTI line
p.doLevel = readDoPin;              // optional: bool readDoPin(void*) reads DO
xhy3131<Stm32HalSpiBlockTransport> dmm(p, &hspi2, DMM_CS_GPIO_Port, DMM_CS_Pin, spi2Mutex);

hy3131::Config cfg = hy3131::defaultConfig();
// Switch network, inputs and gains for the meter function you want:
cfg.r(hy3131::kR2A) = ...;          // PS/DS/FS/SS of PA<1:0>
cfg.ad1Input(0);                    // AD1 from the pre-filter
dmm.start(cfg);

// thread:      for(;;) dmm.main(HAL_GetTick());
// EXTI on DO:  dmm.onIrq();
// anywhere:    hy3131::Sample s; if (dmm.latest(&s)) { ... s.ad1 ... }
```

`hy3131::Config` is the whole configuration image (0x20..0x36), plus INTE and
the counter's gate. The driver loads it, checks it, keeps it loaded and reads
the results. Which switches and multiplexers make DCV, ACV, ohms, capacitance
and so on depends on the resistor network around the chip, so that is up to
the application. `defaultConfig()` brings up the oscillator, bias and
references. It runs AD1 at 5 Hz on the internal temperature sensor, which
needs nothing outside the chip.

Results are published two ways:
* `latest()` / `latestCount()` return the most recent result and can be called
  from any thread (it is a sequence lock).
* An `iListener` gets `onSample()`, `onCount()`, `onFault()` and `onRunning()`
  in the driver's thread.

## What the datasheet doesn't settle, and what the driver assumes

* **Reads are a bit late.** After the read command there is one blank clock,
  so each data byte straddles two SPI bytes. The driver clocks one extra byte
  and shifts everything back (`unshiftRead()`). So a read ends 7 clocks into
  the register after the last one it wants. INTF is cleared by a read, and
  CTBOV by a read of CTSTA. The datasheet doesn't say whether a partly
  clocked register counts as read. The driver assumes it does, and never ends
  a read just before either of those registers:
  * The counter's results are read up to and including INTF.
  * With the counter on, peak hold is read only together with the counter.

  The simulated chip in the test treats partial reads the same way.

  The EEVblog 121GW's stock firmware confirms the extra clock. It bit-bangs
  the HY3131 and gives exactly one clock between the read command and the
  data (121gw-re, `send_byte_to_hy3131()`). Bit-banged
  (`BitBangSpiBlockTransport`), the extra byte still costs only the 7
  trailing clocks.
* **Reset and oscillator timing.** The datasheet gives no time for either. The
  driver waits 10 ms after the reset and 20 ms after starting the oscillator.
* **Power-on register values.** These aren't given either. The driver always
  resets the chip (R37 = 0x60) and writes the whole image.
* **Read-back.** The configuration is compared only on the bits the register
  list defines. ENCTR is left out because the chip might clear it when a
  count ends. If a chip disagrees on some bit, `param.verifyConfig = false`
  turns the comparison off.
* **AD1 defaults.** Input gain 3.6 and reference gain 0.333 are how this
  driver reads the datasheet's "AD1RG: 1" in 12.4.

## Wiring

* **SPI:** mode 0, MSB first, a few MHz at most. The minimum clock high time
  is 20 ns and the minimum low time 60 ns.
* **DO is never released.** While CS is high, DO is the IRQ output. So the
  HY3131 needs an SPI bus of its own, or a tri-state buffer on DO enabled by
  its CS.
* **IRQ:** DO goes high when an enabled flag is set. On STM32, MISO can also
  feed an EXTI line (set the EXTI source to the MISO pin in SYSCFG). You can
  also wire DO to a second GPIO.
  * Edges during the driver's own transfers are ignored.
  * An event that lands at the very end of a transfer is caught by
    `param.doLevel` if you give it one. Otherwise the poll catches it
    (`param.pollMs`, 50 ms by default).
* **AD2 at full rate:** don't turn on its IRQ. At OSR 32 the ADC makes 76,800
  results a second. Use the LPF or RMS event instead, which come once per
  128 to 16,384 AD2 results.
