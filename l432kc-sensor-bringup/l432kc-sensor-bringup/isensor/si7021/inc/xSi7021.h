/*
 * xSi7021.h
 *
 *  Si7021 for FreeRTOS (CMSIS-RTOS2). The state machine is Si7021's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xSi7021<Stm32HalI2CTransport> sensor(param, &hi2c1, SI7021_I2C_ADDR, i2c1Mutex);
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

#ifndef XSI7021_H_
#define XSI7021_H_

#include "cmsis_os2.h"
#include "Si7021.h"

template <typename TTransport>
class xSi7021 : public Si7021<TTransport> {
public:
	using Si7021<TTransport>::Si7021;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

#endif /* XSI7021_H_ */
