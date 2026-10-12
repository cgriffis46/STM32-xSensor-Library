/*
 * xssd1306.h
 *
 *  SSD1306 for FreeRTOS (CMSIS-RTOS2). The state machine is ssd1306's,
 *  unchanged. Only sleep() differs: while a transfer is in flight it
 *  gives up the CPU instead of returning at once, so the thread that
 *  flushes (the GUI task, iDisplay/hw/freertos/xGui.h) does not spin.
 *
 *      xssd1306<Stm32HalI2CTransport, 64> oled(param, &hi2c1, SSD1306_I2C_ADDR, i2c1Mutex);
 *      xssd1306_spi<Stm32HalSPITransport, 64> oled(param, oledDc, &hspi1, CS_Port, CS_Pin, spi1Mutex);
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XSSD1306_H_
#define XSSD1306_H_

#include "cmsis_os2.h"
#include "ssd1306.h"

namespace SSD1306 {

template <typename TTransport, uint8_t H = 64>
class xssd1306 : public ssd1306<TTransport, H> {
public:
	using ssd1306<TTransport, H>::ssd1306;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

template <typename TTransport, uint8_t H = 64>
class xssd1306_spi : public ssd1306_spi<TTransport, H> {
public:
	using ssd1306_spi<TTransport, H>::ssd1306_spi;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace SSD1306 */

#endif /* XSSD1306_H_ */
