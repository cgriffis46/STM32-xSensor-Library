# 121GW LCD bring-up firmware

A first firmware of ours on the 121GW itself. It runs the LCD, keys, buzzer
and rotary switch. It doesn't use the HY3131 or anything analog: the
readings on the HMI screen are made up (the BLE stand-in's `SimFrontEnd`).

* `LcdBringup.h`: the app. It is hardware and OS agnostic and non-blocking,
  and is host-tested (`make -C test test`).
* `stm32l1/`: the board side for the meter's STM32L152ZD. It is bare metal
  with a 1 ms SysTick. `make -C stm32l1` builds `build/EEVBlog.bin`.

**Not yet run on a meter.**

## Before you load it

* **Keep a copy of the stock firmware** to go back to. Use the `EEVBlog.bin`
  for your meter's version from EEVblog's site, or `binaries/EEVBlog-102.bin`
  in [tpwrules/121gw-re](https://github.com/tpwrules/121gw-re).
* **No test leads in the jacks.** This firmware sets the analog switch pins
  as the stock firmware does at power on, but nothing measures, guards or
  warns.
* **What it doesn't touch:** the bootloader (0x08000000–0x08006000), the
  data EEPROM (the calibration, 0x08080000) and the option bytes. It only
  replaces the firmware at 0x08006000, as any firmware update does.

## Loading it

The stock bootloader's SD card update (manual page 63, "Firmware Update"):

1. Copy `EEVBlog.bin` to the root of the SD card.
2. With the meter off, insert the card.
3. Hold the two buttons shown in the manual and switch the meter on.
4. At "IAP", press the button the manual shows, then press it again to
   start.
5. The meter restarts into the new firmware when it's done.

To go back to the stock firmware, do the same with the stock `EEVBlog.bin`.

## What it does

It starts with a long beep and every segment on. **SETUP** steps through
the screens:

| Screen | What you should see | Keys |
|---|---|---|
| 1 ALL | Every segment of the glass | |
| 2 DIGITS | Both displays count 0 to 9 in every place, every 0.7 s. The point moves along, and the minus sign is on for odd digits. Then `-8.8.8.8.8` with every point | |
| 3 WALK | One segment at a time, with its number on the sub display (on the main display while the segment is part of the sub display). Compare with the table below | RANGE next, REL back. Hold for 10 at a time |
| 4 HMI | The real HMI on our meter stack, with made-up readings. The sub display shows the rotary switch's position | Rotary switch, RANGE, HOLD, REL, MODE |

On every screen, **PEAK** raises the contrast and **MEM** lowers it (0–7,
starting at 4, the stock setting). A key that did something beeps once. A
key that can't do anything on this screen beeps twice.

In HMI, the sub display should read:

| Position | Low Z | V | mV | Hz | Ω | VA | µA | mA/A |
|---|---|---|---|---|---|---|---|---|
| Sub display | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |

The main display should match: DC V 1.2xxx at V, 12.xxx mV at mV, 50.00 Hz, 3.3000 kΩ, and so on.
If they come out reversed (V showing 6), the rotary switch is wired the
schematic's way, not the stock firmware's (`Gw121RangeSwitch.h`).

## What to report back

* **ALL:** any segment that's dark, or anything lit that shouldn't be.
* **DIGITS:** any digit that draws wrongly, and which place it's in.
* **WALK:** the numbers whose lit segment isn't the one in the table. Those
  are wrong in `Gw121Glass.h`.
* **Contrast:** the setting that looks best.
* **HMI:** whether the rotary switch positions read 0 to 7 in order.
* **The buzzer and each key:** whether they work.

## WALK: the segment numbers

Names as in `Gw121Glass.h`: MS main display, SS sub display, BG bargraph.
Digits are `<display>_<place>_<stroke>`, place 10000 (left) to 1, strokes
A (top) to G (middle). `POINT_d0` is the point with one digit after it.

| # | Segment | # | Segment | # | Segment | # | Segment |
|--:|---|--:|---|--:|---|--:|---|
| 0 | `ICON_REL` | 40 | `BG_POSITIVE` | 80 | `BG_NEGATIVE` | 120 | `MS_mSEC` |
| 1 | `BG_SCALE` | 41 | `BG_B1` | 81 | `BG_B2` | 121 | `BG_B3` |
| 2 | `BG_B7` | 42 | `BG_B6` | 82 | `BG_B5` | 122 | `BG_B4` |
| 3 | `BG_B8` | 43 | `BG_B9` | 83 | `BG_B10` | 123 | `BG_B11` |
| 4 | `BG_B15` | 44 | `BG_B14` | 84 | `BG_B13` | 124 | `BG_B12` |
| 5 | `BG_B16` | 45 | `BG_B17` | 85 | `BG_B18` | 125 | `BG_B19` |
| 6 | `BG_B23` | 46 | `BG_B22` | 86 | `BG_B21` | 126 | `BG_B20` |
| 7 | `BG_B24` | 47 | `BG_B25` | 87 | `BG_RANGE_5` | 127 | `BG_RANGE_0_FOR_50` |
| 8 | `MS_KILO` | 48 | `MS_OHMS` | 88 | `MS_HERTZ` | 128 | `BG_RANGE_1000` |
| 9 | `MS_MICRO` | 49 | `MS_NANO` | 89 | `MS_FARADS` | 129 | `MS_AMPS_NEXT_TO_F` |
| 10 | `BG_SCALE_10` | 50 | `MS_MILLI` | 90 | `MS_VOLTS` | 130 | `MS_AMPS` |
| 11 | `MS_MEGA` | 51 | `BG_RANGE_0_FOR_500` | 91 | `MS_DEG_C` | 131 | `MS_DEG_F` |
| 12 | `BG_SCALE_5` | 52 | `MS_1_C` | 92 | `MS_1_G` | 132 | `MS_1_B` |
| 13 | `MS_1_D` | 53 | `MS_1_E` | 93 | `MS_1_F` | 133 | `MS_1_A` |
| 14 | `MS_POINT_d0` | 54 | `MS_10_C` | 94 | `MS_10_G` | 134 | `MS_10_B` |
| 15 | `MS_10_D` | 55 | `MS_10_E` | 95 | `MS_10_F` | 135 | `MS_10_A` |
| 16 | `ICON_MEM` | 56 | `ICON_AUTO_OF_HOLD` | 96 | `ICON_HOLD` | 136 | `ICON_ONE_OF_1ms_PEAK` |
| 17 | `MS_DUTY_PERCENT` | 57 | `SS_VOLTS` | 97 | `SS_KILO` | 137 | `SS_HERTZ` |
| 18 | `SS_MILLI` | 58 | `SS_AMPS` | 98 | `SS_nS` | 138 | `SS_OHMS` |
| 19 | `SS_dB` | 59 | `SS_1_C` | 99 | `SS_1_G` | 139 | `SS_1_B` |
| 20 | `SS_1_D` | 60 | `SS_1_E` | 100 | `SS_1_F` | 140 | `SS_1_A` |
| 21 | `SS_POINT_d0` | 61 | `SS_10_C` | 101 | `SS_10_G` | 141 | `SS_10_B` |
| 22 | `SS_10_D` | 62 | `SS_10_E` | 102 | `SS_10_F` | 142 | `SS_10_A` |
| 23 | `SS_POINT_d00` | 63 | `SS_100_C` | 103 | `SS_100_G` | 143 | `SS_100_B` |
| 24 | `SS_100_D` | 64 | `SS_100_E` | 104 | `SS_100_F` | 144 | `SS_100_A` |
| 25 | `SS_POINT_d000` | 65 | `SS_1000_C` | 105 | `SS_1000_G` | 145 | `SS_1000_B` |
| 26 | `SS_POINT_d0000` | 66 | `SS_10000_C` | 106 | `SS_10000_G` | 146 | `SS_10000_B` |
| 27 | `SS_10000_D` | 67 | `SS_10000_E` | 107 | `SS_10000_F` | 147 | `SS_10000_A` |
| 28 | `ICON_TEST` | 68 | `SS_DC` | 108 | `SS_NEGATIVE` | 148 | `SS_AC` |
| 29 | `ICON_LOWZ` | 69 | `ICON_AVG` | 109 | `ICON_MAX` | 149 | `ICON_MIN` |
| 30 | `ICON_BT` | 70 | `ICON_LPF` | 110 | `ICON_CONTINUITY` | 150 | `ICON_DIODE` |
| 31 | `MS_AC` | 71 | `MS_PLUS_OF_AC_DC` | 111 | `MS_DC` | 151 | `ICON_AUTO_RANGE` |
| 32 | `MS_POINT_d00` | 72 | `MS_100_C` | 112 | `MS_100_G` | 152 | `MS_100_B` |
| 33 | `MS_100_D` | 73 | `MS_100_E` | 113 | `MS_100_F` | 153 | `MS_100_A` |
| 34 | `MS_POINT_d000` | 74 | `MS_1000_C` | 114 | `MS_1000_G` | 154 | `MS_1000_B` |
| 35 | `MS_1000_D` | 75 | `MS_1000_E` | 115 | `MS_1000_F` | 155 | `MS_1000_A` |
| 36 | `MS_POINT_d0000` | 76 | `MS_10000_C` | 116 | `MS_10000_G` | 156 | `MS_10000_B` |
| 37 | `MS_10000_D` | 77 | `MS_10000_E` | 117 | `MS_10000_F` | 157 | `MS_10000_A` |
| 38 | `ICON_DANGER` | 78 | `MS_NEGATIVE` | 118 | `ICON_APO` | 158 | `ICON_BATT` |
| 39 | `SS_1000_D` | 79 | `SS_1000_E` | 119 | `SS_1000_F` | 159 | `SS_1000_A` |
