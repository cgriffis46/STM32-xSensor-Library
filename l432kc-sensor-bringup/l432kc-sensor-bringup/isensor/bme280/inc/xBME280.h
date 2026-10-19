/*
 * xBME280.h
 *
 *  BME280 for FreeRTOS (CMSIS-RTOS2). The state machine is bme280's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xbme280<Stm32HalI2CTransport> sensor(param, &hi2c1, bme280_i2c_addr_2, i2c1Mutex);
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

#ifndef XBME280_H_
#define XBME280_H_

#include "cmsis_os2.h"
#include "BME280.h"

namespace BME280 {

template <typename TTransport>
class xbme280 : public bme280<TTransport> {
public:
	using bme280<TTransport>::bme280;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace BME280 */

#endif /* XBME280_H_ */
