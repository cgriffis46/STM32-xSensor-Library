/*
 * xbmp390.h
 *
 *  BMP390 (and BMP388) for FreeRTOS (CMSIS-RTOS2). The state machine is bmp390's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xbmp390<Stm32HalI2CTransport> sensor(param, &hi2c1, BMP390_I2C_ADDR_HIGH, i2c1Mutex);
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

#ifndef XBMP390_H_
#define XBMP390_H_

#include "cmsis_os2.h"
#include "bmp390.h"

namespace BMP390 {

template <typename TTransport>
class xbmp390 : public bmp390<TTransport> {
public:
	using bmp390<TTransport>::bmp390;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace BMP390 */

#endif /* XBMP390_H_ */
