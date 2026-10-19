#!/bin/sh
# Copies the drivers this project uses from a Claude-iTransport checkout
# into STM32L476RG_MC2515_Example/ and records the commit in
# iTransport/SOURCE.txt.
#
#   sh sync_from_claude_itransport.sh /path/to/Claude-iTransport
#
# Run it from this folder (the one holding this script). Files are copied
# unchanged; fix bugs in Claude-iTransport, then sync again.
set -eu

SRC=${1:?usage: sh sync_from_claude_itransport.sh /path/to/Claude-iTransport}
HERE=$(cd "$(dirname "$0")" && pwd)
DST="$HERE/STM32L476RG_MC2515_Example"
IT="$SRC/iTransport/itransport"

# itransport: the SPI block transport under the MCP2515, and the debug log
# (the 1-Wire transport only because Stm32UartItCallbacks.cpp serves it too)
IT_INC="BusTransport.h DebugLog.h ISensorTransport.h OneWireUartTransport.h SPITransport.h SpiBlockTransport.h iBlockTransport.h iClock.h iTransport.h iTransportOneWire.h"
IT_SRC="BusTransport.cpp DebugLog.cpp OneWireUartTransport.cpp SpiBlockTransport.cpp"
# its STM32 HAL and FreeRTOS parts, with the interrupt callbacks
STM32_INC="Stm32DebugPort.h Stm32HalSpiBlockTransport.h Stm32HalUartTransport.h"
STM32_SRC="Stm32HalSpiBlockTransport.cpp Stm32HalUartTransport.cpp Stm32SpiItCallbacks.cpp Stm32UartItCallbacks.cpp"
# isensor's state machine (the MCP2515 driver is written on it)
IS_INC="SensorStateMachine.h"
# iCan: the interface and the MCP2515 driver (header only)
CAN_INC="iCan.h"

test -d "$IT" || { echo "no $IT: is $SRC a Claude-iTransport checkout?" >&2; exit 1; }
test -d "$SRC/iCan" || { echo "no $SRC/iCan: Claude-iTransport is too old" >&2; exit 1; }

rm -rf "$DST/iTransport/inc" "$DST/iTransport/src" "$DST/iTransport/hw" "$DST/isensor" "$DST/iCan"
mkdir -p "$DST/iTransport/inc" "$DST/iTransport/src" "$DST/iTransport/hw/freertos/inc" \
         "$DST/iTransport/hw/stm32/inc" "$DST/iTransport/hw/stm32/src" "$DST/isensor/inc" \
         "$DST/iCan/inc" "$DST/iCan/mcp2515"

for f in $IT_INC; do cp "$IT/inc/$f" "$DST/iTransport/inc/"; done
for f in $IT_SRC; do cp "$IT/src/$f" "$DST/iTransport/src/"; done
cp "$IT/hw/freertos/inc/FreeRtosTransport.h" "$DST/iTransport/hw/freertos/inc/"
for f in $STM32_INC; do cp "$IT/hw/stm32/inc/$f" "$DST/iTransport/hw/stm32/inc/"; done
for f in $STM32_SRC; do cp "$IT/hw/stm32/src/$f" "$DST/iTransport/hw/stm32/src/"; done
for f in $IS_INC; do cp "$SRC/isensor/inc/$f" "$DST/isensor/inc/"; done
for f in $CAN_INC; do cp "$SRC/iCan/inc/$f" "$DST/iCan/inc/"; done
cp -r "$SRC/iCan/mcp2515/inc" "$SRC/iCan/mcp2515/src" "$DST/iCan/mcp2515/"
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
  iCan/inc                       <- iCan/inc ($CAN_INC)
  iCan/mcp2515/inc, src          <- iCan/mcp2515/inc, src

To update: run the script again against a newer Claude-iTransport checkout.
Fix bugs in Claude-iTransport, not here.
EOF
echo "synced from $COMMIT ($BRANCH)"
