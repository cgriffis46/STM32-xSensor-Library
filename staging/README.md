# Staging: drivers on their way to Claude-iTransport

These are laid out the way Claude-iTransport lays them out, so moving them
there is a copy. Everything here is aimed at a 121GW-class meter (HY3131 front
end, STM32L152D), and none of it is tied to that meter.

**Not yet run on hardware.** Each part has a host test, and the whole stack
builds for the Cortex-M3 against ST's real STM32L152xD headers and
CMSIS-RTOS2.

| Folder | What | Test |
|--------|------|------|
| [`isensor/hy3131/`](isensor/hy3131/README.md) | HY3131 / HY3130 DMM front-end driver: `hy3131_dmm<TTransport>`, `xhy3131` | `make -C staging/isensor/hy3131/test test` |
| `itransport/inc/` | `BitBangSpiBlockTransport<TPins>`: SPI mode 0 on four GPIOs (the 121GW's HY3131 is on PF0..PF3, which have no SPI) | `make -C staging/itransport/test test` |
| `itransport/hw/stm32l1/` | STM32L1 SPI: `Stm32L1SpiPort` (register level, interrupt per byte, no HAL SPI) and `Stm32L1SpiBlockTransport<TBus>` | `make -C staging/itransport/hw/stm32l1/test test` (and `cross`) |
| `imeter/` | The meter layer: `iMeter`, `iMeterFrontEnd`, `iCalibrationStore`, `iMeterListener`; `meter::Meter` / `xMeter`; `Hy3131FrontEnd<TDmm>` | `make -C staging/imeter/test test` |
| `idisplay/segment/` | A segment LCD as an HMI. Each part of the glass is a field bound to tags read through `idisplay::iTagSource`: `NumberField` (7-segment digits, point, minus), `IconField`, `BarField`, `ChoiceField`. `xSegmentHmi` draws them and writes the LCD only when the frame changes. `xTagChain` joins tag sources | `make -C staging/idisplay/segment/test test` |
| `idisplay/hw/stm32l1/` | `Stm32L1Lcd`: the STM32L1's LCD controller as an `iSegmentLcd`, register level and non-blocking | `make -C staging/idisplay/hw/stm32l1/test test` |
| [`boards/gw121/`](boards/gw121/README.md) | The EEVblog 121GW: HY3131 range images and nominal scale, factory calibration from EEPROM, analog switch rules, rotary switch, pins | `make -C staging/boards/gw121/test test` |
| `cross/` | The stack instantiated for the STM32L152xD, compiled with `arm-none-eabi-g++` (size check, nothing linked) | `make -C staging/cross cross` |

The cross builds need ST's `cmsis_device_l1` headers:

```bash
git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_l1.git /tmp/cmsis_device_l1
```

Or give `L1_DEVICE=...`. Everything else they need (CMSIS core, FreeRTOS's
CMSIS-RTOS2) is taken from the MCP2515 example in this repository.

## How it fits together

```
   HMI / rotary switch / PLC tags
              |  iMeter: setFunction(), setRange(), hold(), relative(), latest()
        meter::Meter (xMeter)  -- auto range, settle, calibration, hold, REL, min/max
              |  iMeterFrontEnd: select(fn, range, tag), result()
   meter::Hy3131FrontEnd<TDmm> -- the board's route table: (fn, range) -> chip config + which result
              |  hy3131::Config, hy3131::iListener
        hy3131_dmm (xhy3131)   -- reset, configure, verify, IRQ / poll service, fault recovery
              |  iBlockTransport
   Stm32L1SpiBlockTransport<...>  (an SPI peripheral)   or   BitBangSpiBlockTransport<Stm32L1GpioSpiPins>  (GPIO: the 121GW)
```

On the display side, `meter::MeterTags` publishes the meter as tags
(`meter.value`, `meter.unit`, `meter.hold`, ...). The control tags are
writable and drive the meter. `xSegmentHmi` shows those tags on the glass,
and the same tags can feed a PLC tag server, the shell or a web page:

```
   meter::Meter --> meter::MeterTags --+
   board tags (xTagTable, PLC tags) ---+--> xTagChain --> xSegmentHmi (gw121::LcdLayout) --> Stm32L1Lcd
```

On the 121GW, `Hy3131FrontEnd` also sets the board's analog switches
(`iMeterPaths`: `gw121::Paths`), and `Meter` reads the factory calibration
(`iCalibrationStore`: `gw121::StockCalibration`).

Each layer only knows the interface below it. Another meter chip needs its
own front end, another MCU its own transport, another RTOS its own `x...`
wrapper. Bare metal uses the base classes and polling.

## The meter layer

* **Requests are one-deep mailboxes.** A newer `setFunction()` / `setRange()`
  replaces one that hasn't been taken yet. So sweeping the rotary switch
  causes one reconfiguration, not one per position. `setFunction()` followed
  by `setRange()` means that range of that function.
* **Auto range** starts on the highest range, the safe one on an unknown
  input:
  * It goes up on overload or past full scale.
  * It goes down when the value is under 90 % of the next range down's full
    scale.
* **Every selection is tagged.** Results from an older selection are
  dropped, and the first `settleResults()` of a new one too.
* **Values:** `value = (raw - offset) * gain * unitsPerRaw`.
  * The calibration (gain, offset) comes from an `iCalibrationStore`, per
    function and range.
  * Without one, the reading is flagged `kUncal`.
* `Reading::status` carries `kValid`, `kOverload` (OL), `kAutoRange`,
  `kSettling`, `kUncal`, `kHold`, `kRelative` and `kFault`.

## What's missing

* **The rest of the 121GW.** See [`boards/gw121/`](boards/gw121/README.md).
* **Range changes go through a chip reset** (`hy3131_dmm::start()`), which
  costs tens of ms. A reconfigure path without the reset is the next
  improvement to the driver.
* `hy3131_dmm::start()` copies the new configuration without a lock. Don't
  call it from two threads at once. `Meter` only calls it from its own
  thread.
* **No DMA on the L1 port yet.** An SD card or a W5500 will want DMA. The
  121GW's SD card is on SDIO anyway.
* **The L1 port is checked against fake registers and compiles against ST's
  headers.** It has not run on an L1. To do on a board, with a logic
  analyser:
  * BSY before chip-select goes up
  * OVR / MODF recovery
