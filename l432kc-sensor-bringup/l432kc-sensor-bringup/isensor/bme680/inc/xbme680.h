/*
 * xbme680.h
 *
 *  BME680 for FreeRTOS (CMSIS-RTOS2). The state machine is bme680's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xbme680<Stm32HalI2CTransport> sensor(param, &hi2c1, BME680_I2C_ADDR_HIGH, i2c1Mutex);
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

#ifndef XBME680_H_
#define XBME680_H_

#include "cmsis_os2.h"
#include "bme680.h"

namespace BME680 {

template <typename TTransport>
class xbme680 : public bme680<TTransport> {
public:
	using bme680<TTransport>::bme680;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace BME680 */

#endif /* XBME680_H_ */
