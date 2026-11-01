// Compiled by `make cross` only: instantiates the transport for the
// Cortex-M3 against ST's real STM32L152xD header.
#include "Stm32L1SpiBlockTransport.h"

static Stm32L1SpiPort spi2(SPI2);
static Stm32L1SpiBlockTransport<SpiBlockTransport> dev(spi2, GPIOB, 1u << 12, nullptr);
extern "C" void SPI2_IRQHandler(void) { spi2.irq(); }
bool crossCheck(uint8_t* rx) { const uint8_t h = 0x01; return dev.beginRead(&h, 1, rx, 4) && !dev.isBusy(); }
