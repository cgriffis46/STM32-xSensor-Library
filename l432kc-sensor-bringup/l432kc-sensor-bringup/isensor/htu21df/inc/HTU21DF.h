/*
 * HTU21DF.h
 *
 *  Created on: Mar 20, 2024
 *      Author: coryg
 *
 *  HTU21D-F temperature / humidity sensor, non-blocking state machine.
 *
 *  The bus is chosen by the template argument. HTU21DF<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      HTU21DF<Stm32HalI2CTransport>  sensor(&hi2c1, HTU21DF_I2CADDR, i2c1Mutex);
 *
 *  The constructor arguments all go to the transport's own
 *  constructor. This file includes no HAL and no RTOS header.
 *
 *  The HTU21D-F takes one byte commands and is read back as a plain
 *  run of bytes, so this driver uses the transport's writeBytes() and
 *  readBytes().
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each command and each read is one state that issues it and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xHTU21DF.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 *
 *  Measures temperature then humidity once every htu21df_period_ms,
 *  and at once when startTempMeasurement() or
 *  startHumidityMeasurement() asks.
 */

#ifndef HTU21DF_HTU21DF_H_
#define HTU21DF_HTU21DF_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

/** Default I2C address for the HTU21D. */
static const uint16_t HTU21DF_I2CADDR = (0x40);

/** Read temperature register. */
#define HTU21DF_READTEMP (0xE3)

/** Read humidity register. */
#define HTU21DF_READHUM (0xE5)

/** Write register command. */
#define HTU21DF_WRITEREG (0xE6)

/** Read register command. */
#define HTU21DF_READREG (0xE7)

/** Reset command. */
#define HTU21DF_RESET (0xFE)

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum HTU21DF_state_t{
	htu21df_init_state,						// check a device answers
	htu21df_send_reset_state,				// issue the reset command
	htu21df_wait_send_reset_state,
	htu21df_reset_state,					// pause while the reset takes effect
	htu21df_sleep_state,					// between measurements
	htu21df_measure_temp_state,				// issue the temperature command
	htu21df_wait_measure_temp_state,
	htu21df_temp_conversion_state,			// conversion time
	htu21df_read_temp_state,				// issue the 3 byte read
	htu21df_wait_read_temp_state,
	htu21df_measure_humidity_state,			// issue the humidity command
	htu21df_wait_measure_humidity_state,
	htu21df_humidity_conversion_state,		// conversion time
	htu21df_read_humidity_state,			// issue the 3 byte read
	htu21df_wait_read_humidity_state,
	htu21df_error_state						// pause, then start again from htu21df_init_state
}HTU21DF_state_t;

// All times in ms.
const uint32_t htu21df_bus_timeout_ms = 100;	// one bus operation, to issue or to land
const uint32_t htu21df_reset_ms = 15;			// after the reset command
const uint32_t htu21df_period_ms = 1000;		// between measurements
const uint32_t htu21df_request_poll_ms = 100;	// longest sleep while a request could arrive
const uint32_t htu21df_conversion_ms = 50;		// before reading each result
const uint32_t htu21df_error_backoff_ms = 100;	// in htu21df_error_state before starting again

/**
 * Driver for the Adafruit HTU21DF breakout board.
 */
template <typename TTransport>
class HTU21DF : public SensorStateMachine<TTransport, HTU21DF_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"HTU21DF<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, HTU21DF_state_t> base;
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
  explicit HTU21DF(TArgs&&... transportArgs);
  virtual ~HTU21DF() {}

  bool readTemperature(float *temperature);	// degrees C. returns whether the value is new
  bool readHumidity(float *humidity);		// %RH. returns whether the value is new
  void main(uint32_t nowMs);
  void startTempMeasurement();		// ask for a measurement now. either call
  void startHumidityMeasurement();	// measures both, temperature then humidity
  bool newData();
private:
  float _humidity, _temp;
  bool _start_measure;
  bool _newData;

  // The transport fills this in after main() has returned, so it
  // has to be a member and not a local.
  uint8_t dataReg[3];

  bool measurementValid() const;
  uint16_t measurementCode() const;
  static uint8_t crc8(const uint8_t *data, int len);
};

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/HTU21DF.tpp"

#endif /* HTU21DF_HTU21DF_H_ */
