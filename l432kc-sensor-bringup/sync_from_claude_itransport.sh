#!/bin/sh
# Copies the drivers this project uses from a Claude-iTransport checkout
# into l432kc-sensor-bringup/ and records the commit in iTransport/SOURCE.txt.
#
#   sh sync_from_claude_itransport.sh /path/to/Claude-iTransport
#
# Run it from this folder (the one holding this script). Files are copied
# unchanged; fix bugs in Claude-iTransport, then sync again.
set -eu

SRC=${1:?usage: sh sync_from_claude_itransport.sh /path/to/Claude-iTransport}
HERE=$(cd "$(dirname "$0")" && pwd)
DST="$HERE/l432kc-sensor-bringup"
IT="$SRC/iTransport/itransport"

# Sensor drivers: isensor/<folder>, inc/ and src/ each
SENSORS="aht20 bme280 bme680 bmp280 bmp390 DS18B20 ds1307 ds3231 HMC6352 htu21df lps35hw lsm303dlhc mmc56x3 mpl3115a2 PM25 sht31 si7021"

# itransport: the bus classes and the debug log
IT_INC="BusTransport.h DebugLog.h I2CTransport.h ISensorTransport.h OneWireUartTransport.h SPITransport.h iTransport.h iTransportOneWire.h"
IT_SRC="BusTransport.cpp DebugLog.cpp I2CTransport.cpp OneWireUartTransport.cpp"
# its STM32 HAL and FreeRTOS parts, with the interrupt callbacks
STM32_INC="Stm32DebugPort.h Stm32HalI2CTransport.h Stm32HalOneWireTransport.h Stm32HalUartTransport.h"
STM32_SRC="Stm32HalI2CTransport.cpp Stm32HalOneWireTransport.cpp Stm32HalUartTransport.cpp Stm32I2CItCallbacks.cpp Stm32UartItCallbacks.cpp"
# isensor's own headers: the state machine and the sensor interfaces
IS_INC="SensorStateMachine.h Barometer.h Thermometer.h Hygrometer.h SeaLevelBarometer.h RtcDate.h"
# iDisplay: the console's GUI on the PuTTY terminal (xTerminal, xGui, xCell).
# Whole folders: the linker's --gc-sections drops what isn't called.
ID_DIRS="inc src gui/inc gui/src hw/freertos/inc hw/freertos/src hmi/inc hmi/src terminal/inc terminal/src"

test -d "$IT" || { echo "no $IT: is $SRC a Claude-iTransport checkout?" >&2; exit 1; }

rm -rf "$DST/iTransport/inc" "$DST/iTransport/src" "$DST/iTransport/hw" "$DST/isensor" "$DST/iDisplay"
mkdir -p "$DST/iTransport/inc" "$DST/iTransport/src" "$DST/iTransport/hw/freertos/inc" \
         "$DST/iTransport/hw/stm32/inc" "$DST/iTransport/hw/stm32/src" "$DST/isensor/inc"

for f in $IT_INC; do cp "$IT/inc/$f" "$DST/iTransport/inc/"; done
for f in $IT_SRC; do cp "$IT/src/$f" "$DST/iTransport/src/"; done
cp "$IT/hw/freertos/inc/FreeRtosTransport.h" "$DST/iTransport/hw/freertos/inc/"
for f in $STM32_INC; do cp "$IT/hw/stm32/inc/$f" "$DST/iTransport/hw/stm32/inc/"; done
for f in $STM32_SRC; do cp "$IT/hw/stm32/src/$f" "$DST/iTransport/hw/stm32/src/"; done
for f in $IS_INC; do cp "$SRC/isensor/inc/$f" "$DST/isensor/inc/"; done
for s in $SENSORS; do
  mkdir -p "$DST/isensor/$s"
  cp -r "$SRC/isensor/$s/inc" "$SRC/isensor/$s/src" "$DST/isensor/$s/"
done
for d in $ID_DIRS; do
  mkdir -p "$DST/iDisplay/$d"
  cp "$SRC/iDisplay/$d/"* "$DST/iDisplay/$d/"
done
cp "$SRC/LICENSE" "$DST/iTransport/LICENSE"

COMMIT=$(git -C "$SRC" rev-parse --short HEAD)
BRANCH=$(git -C "$SRC" rev-parse --abbrev-ref HEAD)
DATE=$(git -C "$SRC" log -1 --format=%cs)
cat > "$DST/iTransport/SOURCE.txt" <<EOF
Vendored from https://github.com/cgriffis46/Claude-iTransport
commit $COMMIT ($BRANCH, $DATE; MIT, see iTransport/LICENSE), by
../sync_from_claude_itransport.sh. Copied unchanged:

  iTransport/inc, src            <- iTransport/itransport/inc, src
                                    ($IT_INC
                                     $IT_SRC)
  iTransport/hw/freertos/inc     <- iTransport/itransport/hw/freertos/inc (FreeRtosTransport.h)
  iTransport/hw/stm32/inc, src   <- iTransport/itransport/hw/stm32
                                    ($STM32_INC
                                     $STM32_SRC)
  isensor/inc                    <- isensor/inc ($IS_INC)
  isensor/<driver>/inc, src      <- isensor/<driver>, for:
                                    $SENSORS
  iDisplay/<folder>              <- iDisplay/<folder>, for:
                                    $ID_DIRS

From ST (BSD-3): Drivers/STM32L4xx_HAL_Driver is STMicroelectronics/stm32l4xx_hal_driver
v1.13.6.

To update: run the script again against a newer Claude-iTransport checkout.
Fix bugs in Claude-iTransport, not here.
EOF
echo "synced from $COMMIT ($BRANCH)"
