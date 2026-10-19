/*
 * xmmc56x3.h
 *
 *  MMC56x3 for FreeRTOS (CMSIS-RTOS2). The state machine is mmc56x3's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xmmc56x3<Stm32HalI2CTransport> sensor(param, &hi2c1, MMC56X3_I2C_ADDR, i2c1Mutex);
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

#ifndef XMMC56X3_H_
#define XMMC56X3_H_

#include "cmsis_os2.h"
#include "mmc56x3.h"

namespace MMC56X3 {

template <typename TTransport>
class xmmc56x3 : public mmc56x3<TTransport> {
public:
	using mmc56x3<TTransport>::mmc56x3;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace MMC56X3 */

#endif /* XMMC56X3_H_ */
