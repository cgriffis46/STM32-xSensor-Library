/*
 * xSHT31.h
 *
 *  SHT31 for FreeRTOS (CMSIS-RTOS2). The state machine is SHT31's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xSHT31<Stm32HalI2CTransport> sensor(param, &hi2c1, sht31_i2c_addr1, i2c1Mutex);
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

#ifndef XSHT31_H_
#define XSHT31_H_

#include "cmsis_os2.h"
#include "SHT31.h"

template <typename TTransport>
class xSHT31 : public SHT31<TTransport> {
public:
	using SHT31<TTransport>::SHT31;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

#endif /* XSHT31_H_ */
