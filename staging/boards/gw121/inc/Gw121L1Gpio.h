/*
 * Gw121L1Gpio.h
 *
 *  The 121GW's pins on its STM32L152ZD (from the schematic, and the stock
 *  firmware's use of them): the HY3131's bit-banged SPI, and the switch
 *  signals of Gw121Paths.h. STM32L1 only: includes the CMSIS device header.
 *
 *      gw121::L1Gpio paths gpio;                       // Gw121Paths' TGpio
 *      Stm32L1GpioSpiPins hy = gw121::hyPins();        // BitBangSpiBlockTransport's TPins
 *
 *  All of these must be set up as GPIO (outputs, but HY_DO and the rotary
 *  switch's PG0..PG7, inputs with pull-ups as the stock firmware has them)
 *  before use. Clocks for GPIOA, C, D, E, F, G, H.
 */

#ifndef GW121_L1_GPIO_H_
#define GW121_L1_GPIO_H_

#include "stm32l1xx.h"
#include "Stm32L1GpioSpiPins.h"
#include "Gw121Paths.h"

namespace gw121 {

// HY3131: CK PF0, CS PF1, DI PF2, DO PF3 (DO high with CS high: an IRQ).
inline Stm32L1GpioSpiPins hyPins(uint32_t delayLoops = 2) {
	return Stm32L1GpioSpiPins{ {GPIOF, 1u << 0}, {GPIOF, 1u << 1}, {GPIOF, 1u << 2}, {GPIOF, 1u << 3}, delayLoops };
}

struct L1Gpio {
	struct Pin { GPIO_TypeDef* port; uint16_t mask; };

	// In Signal order.
	Pin pins[kSignalCount] = {
		{GPIOD, 1u << 0},    // CTL_A    PD0
		{GPIOC, 1u << 11},   // CTL_B    PC11
		{GPIOF, 1u << 4},    // CTL_D    PF4
		{GPIOF, 1u << 5},    // CTL_E    PF5
		{GPIOD, 1u << 7},    // DCmV_CTL PD7
		{GPIOE, 1u << 15},   // LED_CK   PE15
		{GPIOF, 1u << 6},    // ENB      PF6
		{GPIOA, 1u << 11},   // VA_CTL   PA11
		{GPIOH, 1u << 2},    // TEMP_CTL PH2
		{GPIOA, 1u << 12},   // FRE_CTL  PA12
	};

	void set(Signal s, bool high) {
		const Pin& p = pins[s];
		p.port->BSRR = high ? (uint32_t)p.mask : (uint32_t)p.mask << 16;
	}
};

// The rotary switch's port (PG0..PG7): RangeSwitch::sample(rangeSwitchIdr()).
inline uint16_t rangeSwitchIdr() { return (uint16_t)GPIOG->IDR; }

} // namespace gw121

#endif /* GW121_L1_GPIO_H_ */
