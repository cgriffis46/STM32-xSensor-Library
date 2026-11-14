/*
 * iaqcore.h
 *
 *  ams iAQ-Core indoor air quality module (iAQ-Core C and iAQ-Core P),
 *  non-blocking state machine. A MEMS metal oxide VOC sensor with its
 *  own microcontroller, which gives a CO2 equivalent (ppm), a TVOC
 *  equivalent (ppb) and the sensor's resistance (Ohm).
 *
 *  The bus is chosen by the template argument. iaqcore<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      iaqcore<Stm32HalI2CTransport>  sensor(param, &hi2c1, IAQCORE_I2C_ADDR, i2c1Mutex);
 *
 *  Everything after param goes to the transport's constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  READ ONLY. The datasheet (ams iAQ-Core, v1-00, 2015-Apr-30, "Address")
 *  says: "Please avoid addressing the iAQ-Core with write bit. This could
 *  cause a loss of communication relevant information on modules side and
 *  the iAQ-Core is no longer contactable." So this driver makes one kind
 *  of bus access only, readBytes() of the module's 9 bytes, and never
 *  calls checkDevice() (I2CTransport's is an address with the write bit),
 *  writeBytes(), writeReg(), writeRegs() or readRegs() (which sends the
 *  register number first, with the write bit). A module that isn't there
 *  shows as a read that isn't acknowledged. The application must keep to
 *  the same rule: no bus scan that addresses 0x5A to write.
 *
 *  The bus (datasheet, "I2C Interface Description"): 100 kHz, external
 *  pull-ups, and the master must support clock stretching. The module
 *  takes 3.3 V +/- 0.1 V.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus: the
 *  read is one state that issues it and one that waits for it to land.
 *  When a state has nothing to do it calls sleep(), an empty stub here.
 *  Override sleep() for an OS (see xiaqcore.h).
 *
 *  What it does: reads the 9 bytes every period (1 s for the continuous
 *  iAQ-Core C, 11 s for the pulsed iAQ-Core P: the module's own
 *  measurement intervals), and at once when startMeasurement() asks.
 *  What it does with them goes by the status byte:
 *    OK      the three values are the new reading
 *    RUNIN   warming up (the first 5 minutes after power-up): no reading
 *    BUSY    the module was updating its values: read again, up to
 *            iaqcore_busy_retries times, iaqcore_busy_retry_ms apart
 *    ERROR   no reading. The datasheet: constantly or very often, the
 *            sensor element is probably defective. errorStreak() counts
 *            them in a row.
 *  The module corrects its own baseline; there is nothing to configure
 *  and no command to send.
 *
 *  From the datasheet: the address, the 9 byte layout, big-endian values,
 *  the status values, the intervals and the warm-up time. Not from it
 *  (own choices): the BUSY retry count and gap, the bus timeout, the
 *  error back-off, and treating any other status value as a bad read.
 *  Not run against a module.
 */

#ifndef IAQCORE_H_
#define IAQCORE_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace IAQCORE {

#define IAQCORE_I2C_ADDR 0x5A	// 7 bit. No other address can be set

typedef enum iaqcore_state_t{
	iaqcore_init_state,				// forget the last reading. No bus access: there is no ID to read
	iaqcore_read_state,				// issue the 9 byte read
	iaqcore_wait_read_state,
	iaqcore_busy_retry_state,		// the module said BUSY: a short gap, then read again
	iaqcore_done_state,				// between readings
	iaqcore_error_state				// pause, then start again from iaqcore_init_state
}iaqcore_state_t;

// The status byte (byte 2), as the datasheet's interface protocol table
// gives it.
typedef enum iaqcore_status_t{
	iaqcore_status_ok = 0x00,		// data valid
	iaqcore_status_busy = 0x01,		// re-read multi byte data
	iaqcore_status_runin = 0x10,	// module in warm up phase
	iaqcore_status_error = 0x80,	// if constant: replace sensor
	iaqcore_status_none = 0xFF		// this driver's own: nothing read yet, or the last read failed
}iaqcore_status_t;

typedef enum iaqcore_variant_t{
	iaqcore_continuous,				// iAQ-Core C: measures every second, 66 mW at most
	iaqcore_pulsed					// iAQ-Core P: measures every 11 s at most, 9 mW at most
}iaqcore_variant_t;

// All times in ms.
const uint32_t iaqcore_bus_timeout_ms = 100;		// the read, to issue or to land. 9 bytes at 100 kHz are 1 ms; the rest is for clock stretching
const uint32_t iaqcore_continuous_period_ms = 1000;	// the iAQ-Core C's measurement interval
const uint32_t iaqcore_pulsed_period_ms = 11000;	// the iAQ-Core P's
const uint32_t iaqcore_warmup_ms = 300000;			// first functional reading after start up: 5 minutes. For the application; the driver goes by the status byte
const uint32_t iaqcore_busy_retry_ms = 10;			// gap before reading again after BUSY
const uint8_t  iaqcore_busy_retries = 5;			// BUSY this many times running after the first: a failure
const uint32_t iaqcore_request_poll_ms = 100;		// longest sleep while a startMeasurement() could arrive
const uint32_t iaqcore_error_backoff_ms = 1000;		// in iaqcore_error_state before starting again

const uint8_t iaqcore_frame_len = 9;

// The bus handle and device address belong to the transport and go to
// its constructor.
typedef struct iaqcore_param_t{
	iaqcore_variant_t variant;
	uint32_t period_ms;				// between reads. 0: the variant's own measurement interval
}iaqcore_param_t;

template <typename TTransport>
class iaqcore : public SensorStateMachine<TTransport, iaqcore_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"iaqcore<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, iaqcore_state_t> base;
protected:
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::nextPeriod;
	using base::issued;
	using base::landed;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit iaqcore(const iaqcore_param_t &param, TArgs&&... transportArgs);
	virtual ~iaqcore() {}

	void main(uint32_t nowMs);
	void startMeasurement();			// read now, not at the end of the period

	// A reading: a read whose status was OK. The three getters give NAN
	// until the first one, during warm up, after an ERROR status and
	// after a failed read. Each clears newData().
	bool newData();
	void getCO2(float *ppm);			// CO2 equivalent, ppm. 450 in clean air; the module's range is 450 to 2000 and it reports above that too
	void getTVOC(float *ppb);			// TVOC equivalent, ppb. 125 in clean air; range 125 to 600, and above
	void getResistance(float *ohm);		// the sensor element's resistance, Ohm

	// The module's status, at every read whatever it said. statusChanged()
	// is true when it differs from the one getStatus() last gave.
	bool statusChanged();
	iaqcore_status_t getStatus();
	bool warmingUp() const { return _status==iaqcore_status_runin; }
	// ERROR statuses in a row, 0 again at the next OK or RUNIN. The
	// datasheet gives no number for "constantly (or very frequently)".
	uint32_t errorStreak() const { return _errorStreak; }

	uint32_t periodMs() const { return _period_ms; }

	// The 9 bytes decoded, for the host test and anything else holding a
	// frame. False if the status is none of the datasheet's four.
	static bool decode(const uint8_t *frame, uint16_t *co2, iaqcore_status_t *status, int32_t *resistance, uint16_t *tvoc);
protected:
	void onFail() override;
private:
	uint32_t _period_ms;
	uint32_t _periodStart;				// when this period's read began
	bool _start_measurement;
	bool _newData;
	bool _statusChanged;
	iaqcore_status_t _status;
	uint32_t _errorStreak;
	uint8_t _busyCount;
	float _co2;
	float _tvoc;
	float _resistance;

	// The transport fills this in after main() has returned, so it has
	// to be a member and not a local.
	uint8_t frame[iaqcore_frame_len];

	void setStatus(iaqcore_status_t s);
	void invalidate();
};

} /* namespace IAQCORE */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/iaqcore.tpp"

#endif /* IAQCORE_H_ */
