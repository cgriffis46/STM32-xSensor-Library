/*
 * xPM25.h
 *
 *  PM25 for FreeRTOS (CMSIS-RTOS2). The driver is PM25's, unchanged.
 *  Only the two hooks differ: sleep() puts the thread to sleep, and
 *  wake(), called from the UART interrupt when a good frame has
 *  arrived, wakes it again. So the thread runs once per frame, when
 *  the frame is ready, and does not poll for it.
 *
 *      xPM25<Stm32HalUartTransport> sensor(&huart1);
 *
 *      for(;;){
 *          sensor.main(osKernelGetTickCount());
 *          osDelay(1);    // give up the CPU every cycle
 *      }
 *
 *  Give the sensor its own thread: main() must always be called from
 *  the same one, since that is the thread wake() wakes.
 *
 *  The UART interrupt must be allowed to call FreeRTOS: its priority
 *  has to be numerically at or above
 *  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 *
 *  sleep() is asked for ms and CMSIS-RTOS2 waits in ticks. They are
 *  the same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XPM25_H_
#define XPM25_H_

#include "cmsis_os2.h"
#include "PM25.h"

template <typename TTransport>
class xPM25 : public PM25<TTransport> {
public:
	using PM25<TTransport>::PM25;

protected:
	// The thread is noted first and the waiting frame checked second.
	// A frame that arrives before the check is seen by it; one that
	// arrives after it finds the thread noted and sets its flag, which
	// ends the wait at once. Either way it is not slept through.
	void sleep(uint32_t ms) override {
		_thread = osThreadGetId();
		if(!this->framePending()){
			osThreadFlagsWait(xpm25_wake_flag, osFlagsWaitAny, ms);
		}
	}

	// Interrupt context.
	void wake() override {
		if(_thread != nullptr){
			osThreadFlagsSet(_thread, xpm25_wake_flag);
		}
	}

private:
	// Not the flag FreeRtosTransport uses, so a thread that also
	// drives an I2C or SPI sensor keeps the two apart.
	static const uint32_t xpm25_wake_flag = 0x20000000u;

	osThreadId_t _thread = nullptr;
};

#endif /* XPM25_H_ */
