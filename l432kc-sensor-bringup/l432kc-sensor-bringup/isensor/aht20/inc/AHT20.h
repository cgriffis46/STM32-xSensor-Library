/*
 * AHT20.h
 *
 *  Created on: May 31, 2025
 *      Author: coryg
 *
 *  AHT20 temperature / humidity sensor, non-blocking state machine.
 *
 *  The bus is chosen by the template argument. AHT20<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      AHT20<Stm32HalI2CTransport>  sensor(&hi2c1, AHT20_I2C_Addr, i2c1Mutex);
 *
 *  The constructor arguments all go to the transport's own
 *  constructor. This file includes no HAL and no RTOS header.
 *
 *  The AHT20 has no registers. It takes commands and is read back as
 *  a plain run of bytes, so this driver uses the transport's
 *  writeBytes() and readBytes().
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each command and each read is one state that issues it and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xAHT20.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 *
 *  Measures once every aht20_period_ms, and at once when
 *  StartMeasurement() asks.
 */

#ifndef AHT20_H_
#define AHT20_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

const uint8_t AHT20_I2C_Addr = 0x38;
const uint8_t AHT20_MeasurementCmd = 0xAC;
const uint8_t AHT20_Status_Reg = 0x71;
const uint8_t AHT20_Init_Cmd = 0xBE;
const uint8_t AHT20_Init_High = 0x08;
const uint8_t AHT20_Init_Low = 0x00;
const uint8_t AHT20_MeasurementCmdHigh = 0x33;
const uint8_t AHT20_MeasurementCmdLow = 0x00;
const uint8_t AHT20_SoftResetCmd = 0xBA;

const uint8_t AHT20_Status_Busy = 0b10000000;		// a measurement is in progress
const uint8_t AHT20_Status_Calibrated = 0x18;		// both bits set once the chip is ready for use

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum AHT20_state_t{
	aht20_init,						// check a device answers
	aht_soft_reset,					// issue the soft reset
	aht_wait_soft_reset,
	aht_reset,						// pause while the reset, or the re-init, takes effect
	aht_status_cmd,					// issue the status command
	aht_wait_status_cmd,
	aht_read_status,				// issue the 1 byte read
	aht_wait_status,
	aht_reinit,						// issue the initialisation command
	aht_wait_reinit,
	aht_sleep,						// between measurements
	aht_start_measurement,			// issue the measurement command
	aht_wait_start_measurement,
	aht_measuring,					// conversion time
	aht_read_measurement,			// issue the 7 byte read
	aht_wait_measurement,
	aht_busy_pause,					// the chip was still busy. pause, then read again
	aht_error						// pause, then start again from aht20_init
}AHT20_state_t;

// All times in ms.
const uint32_t aht20_bus_timeout_ms = 100;		// one bus operation, to issue or to land
const uint32_t aht20_reset_ms = 100;			// after a soft reset or a re-init
const uint32_t aht20_phase_timeout_ms = 1000;	// start up, or one whole measurement
const uint32_t aht20_period_ms = 1000;			// between measurements
const uint32_t aht20_request_poll_ms = 100;		// longest sleep while a StartMeasurement() could arrive
const uint32_t aht20_conversion_ms = 100;		// before the first read
const uint32_t aht20_busy_poll_ms = 10;			// between reads while the chip is still busy
const uint32_t aht20_error_backoff_ms = 100;	// in aht_error before starting again

template <typename TTransport>
class AHT20 : public SensorStateMachine<TTransport, AHT20_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"AHT20<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, AHT20_state_t> base;
protected:
	// Names from a template base class have to be brought in before
	// they can be used unqualified.
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::issued;
	using base::landed;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit AHT20(TArgs&&... transportArgs);
	virtual ~AHT20() {}

	void StartMeasurement(); // ask for a measurement now. only taken up between measurements
	void main(uint32_t nowMs);
	bool newData();	// true once per measurement
	void getTempHumidity(float *temp, float *humidity); // returns the values already read from the sensor. NAN until the first reading, and after a failure
private:
	uint8_t status_word;
	uint32_t phase_start;	// when start up, or the current measurement, began
	bool _start_measure;
	bool _newData;
	float _temp,_humidity;

	// The transport fills this in after main() has returned, so it
	// has to be a member and not a local.
	uint8_t dataReg[7];

	// A failure leaves no valid reading, as it did when a failed read
	// came back as NAN.
	void onFail() override { _temp=NAN; _humidity=NAN; }

	bool phaseExpired(uint32_t nowMs) const;
	void convert();
	static uint8_t crc8(const uint8_t *data, int len);
};

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/AHT20.tpp"

#endif /* AHT20_H_ */
