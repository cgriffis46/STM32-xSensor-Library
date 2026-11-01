/*
 * xMeter.h
 *
 *  meter::Meter for CMSIS-RTOS2 (FreeRTOS, or Zephyr's CMSIS-RTOS2 layer).
 *  sleep() waits on a thread flag; wake() (from a request, or from
 *  notifyResult()) sets it.
 *
 *      meter::xMeter m(fe, &cal);
 *      for(;;){ m.main(HAL_GetTick()); }   // its own thread
 */

#ifndef XMETER_H_
#define XMETER_H_

#include "cmsis_os2.h"
#include "Meter.h"

namespace meter {

class xMeter : public Meter {
public:
	using Meter::Meter;

protected:
	void sleep(uint32_t ms) override {
		_thread = osThreadGetId();
		if (!this->workPending()) osThreadFlagsWait(kWakeFlag, osFlagsWaitAny, msToTicks(ms));
	}

	void wake() override {
		osThreadId_t t = _thread;
		if (t != nullptr) osThreadFlagsSet(t, kWakeFlag);
	}

private:
	// Not xhy3131's (0x01000000), xmcp2515's (0x02000000) or the others'
	// listed there, nor FreeRtosTransport's (0x40000000).
	static const uint32_t kWakeFlag = 0x00800000u;

	static uint32_t msToTicks(uint32_t ms) {
		const uint32_t hz = osKernelGetTickFreq();
		const uint64_t t = ((uint64_t)ms * hz + 999u) / 1000u;
		return t == 0 ? 1u : (uint32_t)t;
	}

	osThreadId_t volatile _thread = nullptr;
};

} // namespace meter

#endif /* XMETER_H_ */
