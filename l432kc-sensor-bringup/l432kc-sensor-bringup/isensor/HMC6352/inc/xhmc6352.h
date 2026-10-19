/*
 * xhmc6352.h
 *
 *  HMC6352 for FreeRTOS (CMSIS-RTOS2). The state machine is hmc6352's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xhmc6352<Stm32HalI2CTransport> sensor(param, &hi2c1, HMC6352_I2C_ADDR, i2c1Mutex);
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

#ifndef XHMC6352_H_
#define XHMC6352_H_

#include "cmsis_os2.h"
#include "hmc6352.h"

namespace HMC6352 {

template <typename TTransport>
class xhmc6352 : public hmc6352<TTransport> {
public:
	using hmc6352<TTransport>::hmc6352;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace HMC6352 */

#endif /* XHMC6352_H_ */
