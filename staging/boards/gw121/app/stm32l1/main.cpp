// The bring-up firmware on the 121GW's STM32L152ZD: the board's side of
// gw121::LcdBringup (../LcdBringup.h). Bare metal, no RTOS: a 1 ms SysTick
// and a main loop that never waits.
//
// The HY3131 is on PF0..PF3, bit-banged (BitBangSpiBlockTransport), DO polled
// as the stock firmware does. Its driver, hy3131_dmm, runs without an OS
// (its sleep() does nothing here). Hy3131FrontEnd puts it under the meter
// with the 121GW's route table, its analog switches (Gw121Paths) and the
// meter's own factory calibration, read from the data EEPROM.
//
// Built for 0x08006000 and loaded by the stock bootloader from the SD card
// as EEVBlog.bin, so the bootloader stays, and the stock firmware goes back
// on the same way. Nothing here writes flash, the data EEPROM (where the
// calibration is) or the option bytes.
//
// Clocks as the stock firmware: MSI 4.194 MHz, voltage range 1; the LCD
// from the LSI. Pins as the stock firmware leaves them at power on
// (121gw-re: gpio_config_meter_hardware(), gpio_config_lcd_pins()), the
// HY3131 deselected until its driver starts, the buzzer and backlight off.
#include <stdint.h>
#include "stm32l1xx.h"
#include "Stm32L1Lcd.h"
#include "BitBangSpiBlockTransport.h"
#include "Stm32L1GpioSpiPins.h"
#include "hy3131.h"
#include "Hy3131FrontEnd.h"
#include "Gw121Routes.h"
#include "Gw121Calibration.h"
#include "Gw121Paths.h"
#include "Gw121L1Gpio.h"
#include "LcdBringup.h"

static volatile uint32_t g_ms = 0;
extern "C" void SysTick_Handler(void) { g_ms = g_ms + 1; }

namespace {

constexpr uint32_t kSysClkHz = 4194304;   // MSI range 6

// The bootloader ran first: take the chip back to a known state.
void takeOver() {
	__disable_irq();
	SCB->VTOR = 0x08006000u;
	SysTick->CTRL = 0;
	for (uint32_t i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
}

void clocks() {
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// Voltage range 1 (1.8 V), as the stock firmware.
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
	while (PWR->CSR & PWR_CSR_VOSF) {}
	// MSI at 4.194 MHz as the system clock. Flash wait states are left as the
	// bootloader set them: more than needed is harmless.
	RCC->CR |= RCC_CR_MSION;
	while (!(RCC->CR & RCC_CR_MSIRDY)) {}
	RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | RCC_ICSCR_MSIRANGE_6;
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | RCC_CFGR_SW_MSI;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI) {}

	// The LCD's clock is the RTC's: the LSI, as the stock firmware.
	PWR->CR |= PWR_CR_DBP;
	RCC->CSR |= RCC_CSR_RTCRST;
	RCC->CSR &= ~RCC_CSR_RTCRST;
	RCC->CSR |= RCC_CSR_LSION;
	while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}
	RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | RCC_CSR_RTCSEL_LSI | RCC_CSR_RTCEN;
	RCC->APB1ENR |= RCC_APB1ENR_LCDEN;

	SysTick->LOAD = kSysClkHz / 1000u - 1u;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// MODER 01 output, 00 input, 10 alternate function; PUPDR 01 pull-up.
void output(GPIO_TypeDef* p, uint16_t pins, uint16_t high) {
	p->BSRR = (uint32_t)(pins & high) | ((uint32_t)(pins & ~high) << 16);
	for (uint8_t i = 0; i < 16; i++) if (pins & (1u << i)) {
		p->OTYPER &= ~(1u << i);
		p->OSPEEDR = (p->OSPEEDR & ~(3u << 2 * i)) | (1u << 2 * i);   // medium
		p->PUPDR &= ~(3u << 2 * i);
		p->MODER = (p->MODER & ~(3u << 2 * i)) | (1u << 2 * i);
	}
}
void input(GPIO_TypeDef* p, uint16_t pins, bool pullUp) {
	for (uint8_t i = 0; i < 16; i++) if (pins & (1u << i)) {
		p->MODER &= ~(3u << 2 * i);
		p->PUPDR = (p->PUPDR & ~(3u << 2 * i)) | ((pullUp ? 1u : 0u) << 2 * i);
	}
}
void lcdPins(GPIO_TypeDef* p, uint16_t pins) {
	for (uint8_t i = 0; i < 16; i++) if (pins & (1u << i)) {
		p->MODER = (p->MODER & ~(3u << 2 * i)) | (2u << 2 * i);
		p->PUPDR &= ~(3u << 2 * i);
		p->OTYPER &= ~(1u << i);
		volatile uint32_t& afr = p->AFR[i >> 3];
		afr = (afr & ~(0xFu << 4 * (i & 7))) | (11u << 4 * (i & 7));   // AF11: LCD
	}
}

void pins() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN |
	               RCC_AHBENR_GPIOEEN | RCC_AHBENR_GPIOFEN | RCC_AHBENR_GPIOGEN | RCC_AHBENR_GPIOHEN;
	(void)RCC->AHBENR;

	// Rotary switch PG0..7 and keys PG8..15: inputs, pull-up. Jack detectors PE10, PE11: no pull.
	input(GPIOG, 0xFFFF, true);
	input(GPIOE, (1u << 10) | (1u << 11), false);
	// As the stock firmware at power on. High: PD7 DCmV_CTL, PE9 PWR_CTL,
	// PF2 HY_DI, PA12 FRE_CTL. PF1 HY_CS high too (deselected) here.
	output(GPIOD, (1u << 0) | (1u << 1) | (1u << 7), 1u << 7);                 // CTL_A, PWR_CTL2, DCmV_CTL
	output(GPIOE, (1u << 5) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 12) | (1u << 13) | (1u << 15),
	       1u << 9);                                                           // A_CTL, BUZZER, BL_OUT, PWR_CTL, SHDN_A, -, LED_CK
	output(GPIOF, (1u << 0) | (1u << 1) | (1u << 2), (1u << 1) | (1u << 2));  // HY_CK, HY_CS, HY_DI
	input(GPIOF, 1u << 3, true);                                               // HY_DO
	output(GPIOB, (1u << 6) | (1u << 7), 0);                                   // EE_CLK, EE_DO
	output(GPIOC, (1u << 11) | (1u << 13), 0);                                 // CTL_B, RTC_CE
	output(GPIOF, (1u << 4) | (1u << 5) | (1u << 6), 0);                       // CTL_D, CTL_E, ENB
	output(GPIOH, 1u << 2, 0);                                                 // TEMP_CTL
	output(GPIOA, (1u << 11) | (1u << 12), 1u << 12);                          // VA_CTL, FRE_CTL

	// The LCD's COM and SEG pins, AF11 (gpio_config_lcd_pins()).
	lcdPins(GPIOA, 0x87CE);   // PA1..3, 6..10, 15
	lcdPins(GPIOB, 0xFF3B);   // PB0, 1, 3..5, 8..15
	lcdPins(GPIOC, 0x06FF);   // PC0..7, 9, 10
	lcdPins(GPIOD, 0xFF00);   // PD8..15
	lcdPins(GPIOE, 0x000F);   // PE0..3
}

struct Contrast : gw121::iLcdContrast {
	idisplay::Stm32L1Lcd& lcd;
	explicit Contrast(idisplay::Stm32L1Lcd& l) : lcd(l) {}
	void setContrast(uint8_t cc) override { lcd.setContrast(cc); }
};

// The HY3131: DO read with CS high says a result is waiting.
using HySpi = BitBangSpiBlockTransport<Stm32L1GpioSpiPins>;
using Dmm   = hy3131_dmm<HySpi>;
using Fe    = meter::Hy3131FrontEnd<Dmm>;

Stm32L1GpioSpiPins g_doPins = gw121::hyPins();
hy3131_param_t hyParam() {
	hy3131_param_t p = hy3131_default_param();
	p.doLevel = &Stm32L1GpioSpiPins::doLevelOf;
	p.doCtx = &g_doPins;
	return p;
}
Dmm g_dmm(hyParam(), gw121::hyPins());
gw121::Paths<gw121::L1Gpio> g_paths;
Fe g_fe(g_dmm, gw121::kRoutes, gw121::kRouteCount, hy3131::kXtalHz, &g_paths);
gw121::MappedEeprom g_eeprom;
gw121::StockCalibration g_cal(g_eeprom);

struct HyDiag : gw121::iFrontEndDiag {
	void status(gw121::FrontEndStatus* out) override {
		out->running = g_dmm.running();
		out->state = (uint8_t)g_dmm.driverState();
		out->faults = g_dmm.faults();
		hy3131::Sample s;
		if (g_dmm.latest(&s)) { out->samples = s.seq; out->ad1 = s.ad1; }
		else { out->samples = 0; out->ad1 = 0; }
	}
};
HyDiag g_diag;

idisplay::Stm32L1Lcd g_lcd;
Contrast g_contrast(g_lcd);
idisplay::Buzzer<gw121::BuzzerPin> g_buzzer;
idisplay::KeyScanner g_keys(gw121::keyConfig());
gw121::RangeSwitch g_rotary;
gw121::LcdBringup g_app(g_lcd, g_buzzer, g_fe, &g_cal, &g_contrast, &g_diag);

} // namespace

int main() {
	takeOver();
	clocks();
	pins();
	g_lcd.start(idisplay::Stm32L1Lcd::Config{});
	__enable_irq();
	g_app.start();

	uint32_t last1 = g_ms, last10 = g_ms;
	for (;;) {
		IWDG->KR = 0xAAAA;   // in case the bootloader started the watchdog; harmless if not
		const uint32_t now = g_ms;
		if (now != last1) { last1 = now; g_buzzer.tick(now); }
		if (now - last10 >= 10) {
			last10 += 10;
			const uint16_t g = (uint16_t)GPIOG->IDR;
			g_keys.sample(gw121::keysFromIdr(g));
			g_rotary.sample(g);
		}
		idisplay::KeyEvent e;
		if (g_keys.poll(&e)) g_app.onKey(e);
		gw121::Position p;
		if (g_rotary.poll(&p)) g_app.onPosition(p);
		g_dmm.main(now);   // the HY3131: a few register reads when DO says so
		g_app.main(now);   // the meter on its results, and the screen
	}
}
