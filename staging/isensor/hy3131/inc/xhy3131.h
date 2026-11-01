/*
 * xhy3131.h
 *
 *  hy3131_dmm for CMSIS-RTOS2 (FreeRTOS, or Zephyr's CMSIS-RTOS2 layer). The
 *  driver is hy3131_dmm's, unchanged. Only the two hooks differ: sleep() puts
 *  the thread to sleep, and wake(), called from onIrq() in DO's EXTI interrupt
 *  and from start() and stop(), wakes it again, so a result is read at once
 *  rather than at the next poll.
 *
 *      xhy3131<Stm32HalSpiBlockTransport> dmm(param, &hspi2, DMM_CS_GPIO_Port, DMM_CS_Pin, spi2Mutex);
 *      // in HAL_GPIO_EXTI_Callback, DO's rising edge: dmm.onIrq();
 *
 *      for(;;){ dmm.main(HAL_GetTick()); }   // its own thread
 *
 *  sleep() is in ms, turned into kernel ticks; nowMs must be in ms too (on
 *  Zephyr k_uptime_get_32(), not osKernelGetTickCount()).
 */

#ifndef XHY3131_H_
#define XHY3131_H_

#include "cmsis_os2.h"
#include "hy3131.h"

template <typename TTransport>
class xhy3131 : public hy3131_dmm<TTransport> {
public:
	using hy3131_dmm<TTransport>::hy3131_dmm;

protected:
	// The thread is noted first and the pending work checked second: a
	// wake-up before the check is seen by it, one after it sets the flag
	// and ends the wait at once.
	void sleep(uint32_t ms) override {
		_thread = osThreadGetId();
		if (!this->workPending()) {
			osThreadFlagsWait(kWakeFlag, osFlagsWaitAny, msToTicks(ms));
		}
	}

	void wake() override {
		osThreadId_t t = _thread;
		if (t != nullptr) osThreadFlagsSet(t, kWakeFlag);
	}

private:
	// Not xmcp2515's (0x02000000), xrfm95's (0x04000000), xLoRaWanMac's
	// (0x08000000), xMeshNode's (0x10000000), xsx126x's (0x20000000) or
	// FreeRtosTransport's (0x40000000), which may share the thread.
	static const uint32_t kWakeFlag = 0x01000000u;

	static uint32_t msToTicks(uint32_t ms) {
		const uint32_t hz = osKernelGetTickFreq();
		const uint64_t t = ((uint64_t)ms * hz + 999u) / 1000u;
		return t == 0 ? 1u : (uint32_t)t;
	}

	osThreadId_t volatile _thread = nullptr;
};

#endif /* XHY3131_H_ */
