/*
 * xmcp2515.h
 *
 *  mcp2515 for CMSIS-RTOS2 (FreeRTOS, or Zephyr's CMSIS-RTOS2 layer). The
 *  driver is mcp2515's, unchanged. Only the two hooks differ: sleep() puts
 *  the thread to sleep, and wake(), called from onInt() in INT's interrupt
 *  and from send(), start() and stop(), wakes it again, so a frame is read
 *  or sent at once rather than at the next poll.
 *
 *      xmcp2515<Stm32HalSpiBlockTransport> can(param, &hspi1, CAN_CS_GPIO_Port, CAN_CS_Pin, spi1Mutex);
 *      // in the EXTI callback, INT's falling edge: can.onInt(HAL_GetTick());
 *
 *      for(;;){ can.main(HAL_GetTick()); }   // its own thread
 *
 *  Give the driver its own thread, created with osThreadNew(). send() and
 *  receive() may be called from other threads (one each). sleep() is in
 *  ms, turned into kernel ticks; nowMs must be in ms too (on Zephyr
 *  k_uptime_get_32(), not osKernelGetTickCount()).
 */

#ifndef XMCP2515_H_
#define XMCP2515_H_

#include "cmsis_os2.h"
#include "mcp2515.h"

template <typename TTransport>
class xmcp2515 : public mcp2515<TTransport> {
public:
	using mcp2515<TTransport>::mcp2515;

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
	// Not xrfm95's (0x04000000), xLoRaWanMac's (0x08000000), xMeshNode's
	// (0x10000000), xsx126x's (0x20000000) or FreeRtosTransport's
	// (0x40000000), which may share the thread.
	static const uint32_t kWakeFlag = 0x02000000u;

	// Rounded up, at least a tick: a kernel counting 32768 ticks a second
	// (Zephyr on the nRF52) and one counting 1000 both sleep about ms.
	static uint32_t msToTicks(uint32_t ms) {
		const uint32_t hz = osKernelGetTickFreq();
		const uint64_t t = ((uint64_t)ms * hz + 999u) / 1000u;
		return t == 0 ? 1u : (uint32_t)t;
	}

	osThreadId_t volatile _thread = nullptr;
};

#endif /* XMCP2515_H_ */
