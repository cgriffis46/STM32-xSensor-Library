/*
 * xteseo.h
 *
 *  teseo for FreeRTOS (CMSIS-RTOS2). The driver is teseo's,
 *  unchanged. Only the two hooks differ: sleep() puts the thread to
 *  sleep, and wake(), called from the UART interrupt at the end of
 *  each line (and every 64 bytes), wakes it again.
 *
 *      xteseo<Stm32HalUartTransport> gps(param, &huart6);
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

#ifndef XTESEO_H_
#define XTESEO_H_

#include "cmsis_os2.h"
#include "teseo.h"

template <typename TTransport>
class xteseo : public teseo<TTransport> {
public:
	using teseo<TTransport>::teseo;

protected:
	// The thread is noted first and the waiting bytes checked second,
	// as in xPM25: a wake() before the check is seen by it, one after
	// it sets the flag and ends the wait at once.
	void sleep(uint32_t ms) override {
		_thread = osThreadGetId();
		if (!this->wakePending()) {
			osThreadFlagsWait(xteseo_wake_flag, osFlagsWaitAny, ms);
		}
	}

	// Interrupt context.
	void wake() override {
		if (_thread != nullptr) {
			osThreadFlagsSet(_thread, xteseo_wake_flag);
		}
	}

private:
	// Not FreeRtosTransport's flag, nor xPM25's, xublox_gps's, xmtk3339's or the radios'.
	static const uint32_t xteseo_wake_flag = 0x00800000u;

	osThreadId_t _thread = nullptr;
};

#endif /* XTESEO_H_ */
