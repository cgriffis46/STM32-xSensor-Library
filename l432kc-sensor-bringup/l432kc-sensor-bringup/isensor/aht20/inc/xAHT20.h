/*
 * xAHT20.h
 *
 *  AHT20 for FreeRTOS (CMSIS-RTOS2). The state machine is AHT20's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xAHT20<Stm32HalI2CTransport> sensor(&hi2c1, AHT20_I2C_Addr, i2c1Mutex);
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

#ifndef XAHT20_H_
#define XAHT20_H_

#include "cmsis_os2.h"
#include "AHT20.h"

template <typename TTransport>
class xAHT20 : public AHT20<TTransport> {
public:
	using AHT20<TTransport>::AHT20;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

#endif /* XAHT20_H_ */
