#pragma once
#include <stdint.h>
#include "stm32l1xx.h"
#include "SegmentFrame.h"

namespace idisplay {

// The STM32L1's LCD controller as an iSegmentLcd, at register level.
//
//   Stm32L1Lcd lcd;                 // LCD
//   lcd.start(Stm32L1Lcd::Config{}); // once, after the LCD clock (RTC clock source, APB1 LCDEN)
//                                    // and the SEG/COM pins (AF11) are set up
//   xSegmentHmi hmi(lcd, tags, fields, n);
//
// write() copies a frame to the display RAM and asks for an update (UDR).
// While that request is pending, the RAM is locked, and write() returns
// false without waiting. busy() says so. RAM[2 * com] holds SEG0..31 and
// RAM[2 * com + 1] holds SEG32..43, as SegmentFrame::row(com) bits 0..43.
//
// Config's defaults are the EEVblog 121GW's, from its stock firmware
// (lcd_init()): 1/4 duty, 1/3 bias, internal step-up, PS = 3, DIV = 0,
// pulse-on 4, no dead time, contrast CC 4 (the stock setting 14 of 10..17).
class Stm32L1Lcd : public iSegmentLcd {
public:
    struct Config {
        uint8_t duty = 3;        // CR.DUTY: 0 static, 1 1/2, 2 1/3, 3 1/4, 4 1/8
        uint8_t bias = 2;        // CR.BIAS: 0 1/4, 1 1/2, 2 1/3
        bool    externalVlcd = false;   // CR.VSEL
        bool    muxSeg = false;  // CR.MUX_SEG
        uint8_t ps = 3;          // FCR.PS: ck_ps = LCDCLK / 2^ps
        uint8_t div = 0;         // FCR.DIV: ck_div = ck_ps / (16 + div)
        uint8_t contrast = 4;    // FCR.CC 0..7
        uint8_t pulseOn = 4;     // FCR.PON 0..7
        uint8_t deadTime = 0;    // FCR.DEAD 0..7
        uint8_t coms = 4;        // common lines used: 1, 2, 3, 4 (or 8 with duty 1/8)
    };

    explicit Stm32L1Lcd(LCD_TypeDef* lcd = LCD) : lcd_(lcd) {}

    // Bounded waits for FCRSR, ENS and RDY: start-up only. False if the
    // controller didn't come up (no LCD clock, no VLCD).
    bool start(const Config& c) {
        cfg_ = c;
        lcd_->CR &= ~LCD_CR_LCDEN;
        lcd_->FCR = ((uint32_t)(c.ps & 0xF) << LCD_FCR_PS_Pos) | ((uint32_t)(c.div & 0xF) << LCD_FCR_DIV_Pos) |
                    ((uint32_t)(c.contrast & 7) << LCD_FCR_CC_Pos) | ((uint32_t)(c.deadTime & 7) << LCD_FCR_DEAD_Pos) |
                    ((uint32_t)(c.pulseOn & 7) << LCD_FCR_PON_Pos);
        if (!waitFor(LCD_SR_FCRSR)) return false;
        lcd_->CR = ((uint32_t)(c.duty & 7) << LCD_CR_DUTY_Pos) | ((uint32_t)(c.bias & 3) << LCD_CR_BIAS_Pos) |
                   (c.externalVlcd ? LCD_CR_VSEL : 0u) | (c.muxSeg ? LCD_CR_MUX_SEG : 0u);
        lcd_->CR |= LCD_CR_LCDEN;
        return waitFor(LCD_SR_ENS) && waitFor(LCD_SR_RDY);
    }

    // 0..7. Takes effect at the next frame (FCR is resynchronised by the controller).
    void setContrast(uint8_t cc) {
        lcd_->FCR = (lcd_->FCR & ~LCD_FCR_CC) | ((uint32_t)(cc & 7) << LCD_FCR_CC_Pos);
        cfg_.contrast = cc & 7;
    }

    bool write(const SegmentFrame& f) override {
        if (busy()) return false;
        const uint8_t n = cfg_.coms > 8 ? 8 : cfg_.coms;
        for (uint8_t c = 0; c < n; c++) {
            const uint64_t r = f.row(c);
            lcd_->RAM[2 * c] = (uint32_t)r;
            lcd_->RAM[2 * c + 1] = (uint32_t)(r >> 32) & 0xFFFu;
        }
        lcd_->SR = LCD_SR_UDR;   // writing 1 requests the update; the controller clears it when done
        return true;
    }

    bool busy() const override { return (lcd_->SR & LCD_SR_UDR) != 0U; }

private:
    bool waitFor(uint32_t bit) {
        for (uint32_t i = 0; i < 1000000u; i++) if (lcd_->SR & bit) return true;
        return false;
    }

    LCD_TypeDef* lcd_;
    Config cfg_;
};

} // namespace idisplay
