#include "main.h" // the HAL header for this project's STM32 family — see Stm32HalI2CTransport.h
#include "I2CTransport.h"

// STM32 HAL calls these (weak, overridable) from the I2C interrupt
// handler once an interrupt-driven transfer finishes or faults.
// This is the one piece of glue code that has to live outside the
// class — HAL callbacks are free C functions per peripheral, not
// per-instance, so BusTransport::onTransferComplete() does the work
// of finding which instance was actually waiting.
//
// These must be linked into the application as object files. Inside a
// static library the linker has no reason to pull them in over the
// HAL's own weak defaults, and no transfer would ever complete.
extern "C" {

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    I2CTransport::onTransferComplete(hi2c, /*failed=*/false);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    I2CTransport::onTransferComplete(hi2c, /*failed=*/false);
}

// The same two events for HAL_I2C_Master_Transmit_IT/
// HAL_I2C_Master_Receive_IT (writeBytes()/readBytes()) — HAL reports
// those through a separate pair of callbacks from the Mem ones above.
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    I2CTransport::onTransferComplete(hi2c, /*failed=*/false);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    I2CTransport::onTransferComplete(hi2c, /*failed=*/false);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    I2CTransport::onTransferComplete(hi2c, /*failed=*/true);
}

} // extern "C"
