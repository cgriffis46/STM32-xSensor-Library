/*
 * xlps35hw.h
 *
 *  LPS35HW for FreeRTOS (CMSIS-RTOS2). The state machine is lps35hw's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xlps35hw<Stm32HalI2CTransport> sensor(param, &hi2c1, LPS35HW_ADDR, i2c1Mutex);
 *
 *      for(;;){
 *          sensor.main(osKernelGetTickCount());
 *          osDelay(1);    // give up the CPU every cycle
 *      }
 *
 *  Give the sensor its own thread. While it sleeps between
 *  measurements nothing else in that thread's loop runs.
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XLPS35HW_H_
#define XLPS35HW_H_

#include "cmsis_os2.h"
#include "lps35hw.h"

namespace LPS35HW {

template <typename TTransport>
class xlps35hw : public lps35hw<TTransport> {
public:
	using lps35hw<TTransport>::lps35hw;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace LPS35HW */

#endif /* XLPS35HW_H_ */
