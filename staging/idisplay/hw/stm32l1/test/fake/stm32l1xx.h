// Host stand-in for the CMSIS device header: the LCD's registers as plain
// memory. Bit positions copied from ST's stm32l152xd.h.
#pragma once
#include <cstdint>
#define __IO volatile
typedef struct { __IO uint32_t CR, FCR, SR, CLR; uint32_t RESERVED; __IO uint32_t RAM[16]; } LCD_TypeDef;
extern LCD_TypeDef g_fakeLcd;
#define LCD (&g_fakeLcd)
#define LCD_CR_LCDEN      (1u << 0)
#define LCD_CR_VSEL       (1u << 1)
#define LCD_CR_DUTY_Pos   2u
#define LCD_CR_BIAS_Pos   5u
#define LCD_CR_MUX_SEG    (1u << 7)
#define LCD_FCR_PON_Pos   4u
#define LCD_FCR_DEAD_Pos  7u
#define LCD_FCR_CC_Pos    10u
#define LCD_FCR_CC        (7u << 10)
#define LCD_FCR_DIV_Pos   18u
#define LCD_FCR_PS_Pos    22u
#define LCD_SR_ENS        (1u << 0)
#define LCD_SR_UDR        (1u << 2)
#define LCD_SR_RDY        (1u << 4)
#define LCD_SR_FCRSR      (1u << 5)
