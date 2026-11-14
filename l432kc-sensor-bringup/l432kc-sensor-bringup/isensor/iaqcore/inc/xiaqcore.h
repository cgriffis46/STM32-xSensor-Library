/*
 * xiaqcore.h
 *
 *  iAQ-Core for FreeRTOS (CMSIS-RTOS2). The state machine is iaqcore's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xiaqcore<Stm32HalI2CTransport> sensor(param, &hi2c1, IAQCORE_I2C_ADDR, i2c1Mutex);
 *
 *      for(;;){
 *          sensor.main(osKernelGetTickCount());
 *          osDelay(1);    // give up the CPU every cycle
 *      }
 *
 *  Give the sensor its own thread. While it sleeps between readings
 *  nothing else in that thread's loop runs.
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XIAQCORE_H_
#define XIAQCORE_H_

#include "cmsis_os2.h"
#include "iaqcore.h"

namespace IAQCORE {

template <typename TTransport>
class xiaqcore : public iaqcore<TTransport> {
public:
	using iaqcore<TTransport>::iaqcore;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace IAQCORE */

#endif /* XIAQCORE_H_ */
