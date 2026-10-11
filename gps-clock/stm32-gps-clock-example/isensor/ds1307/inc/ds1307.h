/*
 * ds1307.h
 *
 *  DS1307 real-time clock (Maxim/Analog Devices), non-blocking state
 *  machine on SensorStateMachine like the sensor drivers.
 *
 *  The bus is chosen by the template argument. ds1307<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      ds1307_param_t param;            // read once a second, SQW/OUT low
 *      ds1307<Stm32HalI2CTransport> rtc(param, &hi2c1, DS1307_ADDR, i2c1Mutex);
 *
 *      for(;;){ rtc.main(HAL_GetTick()); ... }
 *      if(rtc.newData()){ ds1307_time_t t; if(rtc.getTime(&t)) ... }
 *      rtc.setTime(t);                  // written at the next main()
 *
 *  Everything after param goes to the transport's own constructor, so
 *  the DS1307 can share an I2C bus (and its mutex) with other drivers.
 *
 *  What it does: checks the chip answers, writes the control register
 *  (the SQW/OUT pin), then reads the seven time registers every
 *  param.period_ms in one transfer. The DS1307 copies its counters into
 *  the registers it is read from on each I2C START, so the seven bytes
 *  are from one instant. setTime() writes all seven in one transfer
 *  (the seconds write restarts the chip's 1 Hz divider, and the rest
 *  must follow within the second) and reads them back at once.
 *
 *  Times are 24 hour. A chip left in 12 hour mode by other code is read
 *  correctly; setTime() puts it back in 24 hour mode. The day of the
 *  week is worked out from the date (0 Sunday .. 6 Saturday) and
 *  written as 1..7 (1 Sunday), since the chip only counts it.
 *
 *  A new or battery-less DS1307 powers up with its oscillator stopped
 *  (the CH bit, bit 7 of the seconds register). It is not started here:
 *  running() reports it, and the first setTime() starts it. The time a
 *  stopped chip reports is stale, but getTime() still returns it.
 *
 *  Not done: the 56 bytes of battery-backed RAM (0x08..0x3F).
 *
 *  Register layout, the CH bit, the 12/24 hour bit, the control
 *  register and the START latching are from the DS1307 datasheet
 *  (Maxim 19-5215). The DS1307 is a 100 kHz (standard mode) part.
 *  Not yet run against a chip.
 */

#ifndef DS1307_H_
#define DS1307_H_

#include <stdint.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace DS1307 {

#define DS1307_ADDR 0x68	// 7 bit, fixed

// Every bus access is two states: one that issues the transfer and a
// wait_ state that it lands in.
typedef enum ds1307_state_t{
	ds1307_init_state,				// check a device answers
	ds1307_control_state,			// issue the control register write
	ds1307_wait_control_state,
	ds1307_done_state,				// between reads; a setTime() is taken up here
	ds1307_read_state,				// issue the time read
	ds1307_wait_read_state,
	ds1307_write_time_state,		// issue the time write
	ds1307_wait_write_time_state,
	ds1307_err_state				// pause, then start again from ds1307_init_state
}ds1307_state_t;

typedef enum ds1307_reg_addr_t{
	ds1307_seconds_addr = 0x00,		// bit 7: CH, clock halt
	ds1307_minutes_addr = 0x01,
	ds1307_hours_addr = 0x02,		// bit 6: 12 hour mode; then bit 5 is PM
	ds1307_day_addr = 0x03,			// day of the week, 1..7
	ds1307_date_addr = 0x04,
	ds1307_month_addr = 0x05,
	ds1307_year_addr = 0x06,		// 00..99
	ds1307_control_addr = 0x07,
	ds1307_ram_addr = 0x08			// 56 bytes of RAM, 0x08..0x3F
}ds1307_reg_addr_t;

// The SQW/OUT pin (control register). Low and High hold it at that
// level; the rest are square waves. Open drain: it needs a pull-up.
enum class ds1307_sqw_t : uint8_t {
	Low = 0x00,
	High = 0x80,			// OUT
	Hz1 = 0x10,				// SQWE, RS = 00
	Hz4096 = 0x11,
	Hz8192 = 0x12,
	Hz32768 = 0x13
};

// A date and time, 24 hour. year 2000..2099. weekday 0 Sunday .. 6
// Saturday (setTime() works it out; the one passed in is ignored).
struct ds1307_time_t {
	uint16_t year;
	uint8_t month, day;
	uint8_t hour, minute, second;
	uint8_t weekday;
};

// All times in ms.
const uint32_t ds1307_bus_timeout_ms = 100;		// one transfer, to issue or to land
const uint32_t ds1307_error_backoff_ms = 500;	// in ds1307_err_state before starting again
const uint32_t ds1307_request_poll_ms = 100;	// longest sleep, so setTime()/readNow() are seen promptly

struct ds1307_param_t {
	uint32_t period_ms;		// between reads
	ds1307_sqw_t sqw;		// what the SQW/OUT pin does

	ds1307_param_t() : period_ms(1000), sqw(ds1307_sqw_t::Low) {}
};

// What became of the last setTime().
enum class ds1307_set_status_t : uint8_t {
	None,		// no setTime() yet
	Pending,	// waiting to be written
	Done,		// written (and read back at once by the next read)
	Failed		// the write failed; not retried (the time would be stale): call setTime() again
};

// ---- date helpers, usable on their own ----

// 0 Sunday .. 6 Saturday, for 2000-01-01 onwards.
inline uint8_t ds1307_weekday(uint16_t year, uint8_t month, uint8_t day);
// The date and time are real (2000..2099, day within its month, 24 hour).
inline bool ds1307_valid(const ds1307_time_t& t);
// Seconds since 1970-01-01 00:00:00, the time taken as UTC. t must be valid.
inline uint32_t ds1307_to_unix(const ds1307_time_t& t);
// The reverse, weekday included. Times before 2000 or after 2099 give
// a time ds1307_valid() refuses.
inline void ds1307_from_unix(uint32_t unixSeconds, ds1307_time_t* t);

template <typename TTransport>
class ds1307 : public SensorStateMachine<TTransport, ds1307_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"ds1307<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, ds1307_state_t> base;
protected:
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
	explicit ds1307(const ds1307_param_t& param, TArgs&&... transportArgs);
	virtual ~ds1307() {}

	void main(uint32_t nowMs);

	// A read has landed since the last getTime().
	bool newData() const { return _newData; }
	// The last time read. False (t untouched) until a read with a
	// sensible date and time has landed, and after a failure. Clears
	// newData().
	bool getTime(ds1307_time_t* t);
	// The oscillator was running at the last read (CH clear). False
	// until the first read.
	bool running() const { return _haveTime && _running; }

	// Queues t to be written at the next main(), replacing one still
	// waiting. False, and nothing queued, if t is not a real date and
	// time (ds1307_valid()). Starts a stopped oscillator.
	bool setTime(const ds1307_time_t& t);
	ds1307_set_status_t setStatus() const { return _setStatus; }

	// Read at the next main() instead of waiting out the period.
	void readNow() { _readNow = true; }

protected:
	void onFail() override;

private:
	bool decode();					// _regs -> _time, _running; false if not a real time
	void encode(const ds1307_time_t& t, uint8_t* out) const;

	ds1307_param_t _param;
	uint32_t _lastRead;				// when the last read was issued

	// The transport fills this after main() has returned: a member.
	uint8_t _regs[7];
	uint8_t _setRegs[7];			// the time setTime() asked for, encoded

	ds1307_time_t _time;
	bool _haveTime;
	bool _running;
	bool _newData;
	bool _readNow;
	bool _setWanted;
	ds1307_set_status_t _setStatus;
};

} /* namespace DS1307 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/ds1307.tpp"

#endif /* DS1307_H_ */
