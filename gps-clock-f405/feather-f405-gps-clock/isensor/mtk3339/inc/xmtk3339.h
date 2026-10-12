/*
 * xmtk3339.h
 *
 *  mtk3339 for FreeRTOS (CMSIS-RTOS2). The driver is mtk3339's,
 *  unchanged. Only the two hooks differ: sleep() puts the thread to
 *  sleep, and wake(), called from the UART interrupt at the end of
 *  each line (and every 64 bytes), wakes it again.
 *
 *      xmtk3339<Stm32HalUartTransport> gps(param, &huart1);
 *
 *      for(;;){
 *          gps.main(osKernelGetTickCount());
 *      }
 *
 *  Give the receiver its own thread: main() must always be called from
 *  the same one, since that is the thread wake() wakes. The UART
 *  interrupt must be allowed to call FreeRTOS (its priority numerically
 *  at or above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY).
 *
 *  sleep() is asked for ms and CMSIS-RTOS2 waits in ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XMTK3339_H_
#define XMTK3339_H_

#include "cmsis_os2.h"
#include "mtk3339.h"

template <typename TTransport>
class xmtk3339 : public mtk3339<TTransport> {
public:
	using mtk3339<TTransport>::mtk3339;

protected:
	// The thread is noted first and the waiting bytes checked second,
	// as in xPM25: a wake() before the check is seen by it, one after
	// it sets the flag and ends the wait at once.
	void sleep(uint32_t ms) override {
		_thread = osThreadGetId();
		if (!this->wakePending()) {
			osThreadFlagsWait(xmtk3339_wake_flag, osFlagsWaitAny, ms);
		}
	}

	// Interrupt context.
	void wake() override {
		if (_thread != nullptr) {
			osThreadFlagsSet(_thread, xmtk3339_wake_flag);
		}
	}

private:
	// Not FreeRtosTransport's flag, nor xPM25's or xublox_gps's.
	static const uint32_t xmtk3339_wake_flag = 0x08000000u;

	osThreadId_t _thread = nullptr;
};

#endif /* XMTK3339_H_ */
