/*
 * xds3231.h
 *
 *  DS3231 for FreeRTOS (CMSIS-RTOS2). The state machine is ds3231's,
 *  unchanged. Only sleep() differs: it gives up the CPU for the time
 *  the state machine asked for (never more than ds3231_request_poll_ms
 *  between reads).
 *
 *      xds3231<Stm32HalI2CTransport> rtc(param, &hi2c1, DS3231_ADDR, i2c1Mutex);
 *
 *      for(;;){
 *          rtc.main(osKernelGetTickCount());
 *      }
 *
 *  Nothing in the driver is locked: call setTime(), setAlarm(),
 *  setAging(), getTime() and alarmFired() from the thread that runs
 *  main(). readNow() only sets a flag, so the INT pin's interrupt may
 *  call it (the next read is then at most ds3231_request_poll_ms away).
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XDS3231_H_
#define XDS3231_H_

#include "cmsis_os2.h"
#include "ds3231.h"

namespace DS3231 {

template <typename TTransport>
class xds3231 : public ds3231<TTransport> {
public:
	using ds3231<TTransport>::ds3231;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace DS3231 */

#endif /* XDS3231_H_ */
