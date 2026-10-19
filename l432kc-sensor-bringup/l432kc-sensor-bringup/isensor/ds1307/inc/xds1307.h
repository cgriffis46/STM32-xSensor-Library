/*
 * xds1307.h
 *
 *  DS1307 for FreeRTOS (CMSIS-RTOS2). The state machine is ds1307's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for (never more than ds1307_request_poll_ms
 *  between reads).
 *
 *      xds1307<Stm32HalI2CTransport> rtc(param, &hi2c1, DS1307_ADDR, i2c1Mutex);
 *
 *      for(;;){
 *          rtc.main(osKernelGetTickCount());
 *      }
 *
 *  Nothing in the driver is locked: call setTime(), readNow() and
 *  getTime() from the thread that runs main().
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XDS1307_H_
#define XDS1307_H_

#include "cmsis_os2.h"
#include "ds1307.h"

namespace DS1307 {

template <typename TTransport>
class xds1307 : public ds1307<TTransport> {
public:
	using ds1307<TTransport>::ds1307;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace DS1307 */

#endif /* XDS1307_H_ */
