/*
 * xHTU21DF.h
 *
 *  HTU21D-F for FreeRTOS (CMSIS-RTOS2). The state machine is HTU21DF's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xHTU21DF<Stm32HalI2CTransport> sensor(&hi2c1, HTU21DF_I2CADDR, i2c1Mutex);
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

#ifndef XHTU21DF_H_
#define XHTU21DF_H_

#include "cmsis_os2.h"
#include "HTU21DF.h"

template <typename TTransport>
class xHTU21DF : public HTU21DF<TTransport> {
public:
	using HTU21DF<TTransport>::HTU21DF;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

#endif /* XHTU21DF_H_ */
