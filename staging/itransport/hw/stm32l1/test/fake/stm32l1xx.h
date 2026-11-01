// Host stand-in for the CMSIS device header: the registers are plain
// memory that the test plays the peripheral against. Bit positions are
// copied from ST's stm32l152xd.h.
#pragma once
#include <cstdint>
#define __IO volatile
typedef struct { __IO uint32_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR; } SPI_TypeDef;
typedef struct { __IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR; } GPIO_TypeDef;
#define SPI_CR1_CPHA      (1u << 0)
#define SPI_CR1_CPOL      (1u << 1)
#define SPI_CR1_MSTR      (1u << 2)
#define SPI_CR1_BR_Pos    3u
#define SPI_CR1_SPE       (1u << 6)
#define SPI_CR1_SSI       (1u << 8)
#define SPI_CR1_SSM       (1u << 9)
#define SPI_CR2_ERRIE     (1u << 5)
#define SPI_CR2_RXNEIE    (1u << 6)
#define SPI_SR_RXNE       (1u << 0)
#define SPI_SR_TXE        (1u << 1)
#define SPI_SR_MODF       (1u << 5)
#define SPI_SR_OVR        (1u << 6)
#define SPI_SR_BSY        (1u << 7)
