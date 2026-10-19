/*
 * xmpl3115a2.h
 *
 *  MPL3115A2 for FreeRTOS (CMSIS-RTOS2). The state machine is
 *  mpl3115a2's, unchanged. Only sleep() differs: it gives up the CPU
 *  for the time the state machine asked for, so the thread wakes when
 *  there is something to do.
 *
 *      mpl3115a2::xmpl3115a2<Stm32HalI2CTransport> sensor(param, &hi2c1, mpl3115a2::MPL3115A2_DEV_ADDRESS, i2c1Mutex);
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

#ifndef XMPL3115A2_H_
#define XMPL3115A2_H_

#include "cmsis_os2.h"
#include "mpl3115a2.h"

namespace mpl3115a2 {

template <typename TTransport>
class xmpl3115a2 : public mpl3115a2<TTransport> {
public:
	using mpl3115a2<TTransport>::mpl3115a2;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace mpl3115a2 */

#endif /* XMPL3115A2_H_ */
