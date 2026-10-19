/*
 * xDS18B20.h
 *
 *  DS18B20 for FreeRTOS (CMSIS-RTOS2). The state machine is ds18b20's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for, so the thread wakes when there is
 *  something to do.
 *
 *      xds18b20<Stm32HalOneWireTransport> sensor(&huart1, uart1_mutexHandle);
 *
 *      for(;;){
 *          sensor.main(osKernelGetTickCount());
 *          osDelay(1);    // give up the CPU every cycle
 *      }
 *
 *  Give the sensor its own thread. While it sleeps through a
 *  conversion nothing else in that thread's loop runs.
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XDS18B20_H_
#define XDS18B20_H_

#include "cmsis_os2.h"
#include "DS18B20.h"

namespace DS18B20 {

template <typename TTransport>
class xds18b20 : public ds18b20<TTransport> {
public:
	using ds18b20<TTransport>::ds18b20;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace DS18B20 */

#endif /* XDS18B20_H_ */
