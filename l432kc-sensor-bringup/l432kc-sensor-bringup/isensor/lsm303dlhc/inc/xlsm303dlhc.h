/*
 * xlsm303dlhc.h
 *
 *  LSM303DLHC for FreeRTOS (CMSIS-RTOS2). The state machines are
 *  lsm303dlhc_accel's and lsm303dlhc_mag's, unchanged. Only sleep()
 *  differs: it gives up the CPU for the time the state machine asked
 *  for, so the thread wakes when there is something to do.
 *
 *      xlsm303dlhc_accel<Stm32HalI2CTransport> accel(aparam, &hi2c1, LSM303DLHC_ACCEL_ADDR, i2c1Mutex);
 *      xlsm303dlhc_mag<Stm32HalI2CTransport>   mag(mparam, &hi2c1, LSM303DLHC_MAG_ADDR, i2c1Mutex);
 *
 *      for(;;){
 *          accel.main(osKernelGetTickCount());
 *          osDelay(1);    // give up the CPU every cycle
 *      }
 *
 *  Give each its own thread. While one sleeps between readings
 *  nothing else in that thread's loop runs.
 *
 *  sleep() is asked for ms and osDelay() takes ticks. They are the
 *  same at the usual 1 kHz tick. Convert here if yours differs.
 */

#ifndef XLSM303DLHC_H_
#define XLSM303DLHC_H_

#include "cmsis_os2.h"
#include "lsm303dlhc.h"

namespace LSM303DLHC {

template <typename TTransport>
class xlsm303dlhc_accel : public lsm303dlhc_accel<TTransport> {
public:
	using lsm303dlhc_accel<TTransport>::lsm303dlhc_accel;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

template <typename TTransport>
class xlsm303dlhc_mag : public lsm303dlhc_mag<TTransport> {
public:
	using lsm303dlhc_mag<TTransport>::lsm303dlhc_mag;

protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

} /* namespace LSM303DLHC */

#endif /* XLSM303DLHC_H_ */
