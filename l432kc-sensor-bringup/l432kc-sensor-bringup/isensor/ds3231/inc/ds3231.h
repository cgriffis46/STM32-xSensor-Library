/*
 * ds3231.h
 *
 *  DS3231 real-time clock (Maxim/Analog Devices; a temperature-
 *  compensated crystal), non-blocking state machine on SensorStateMachine
 *  like the sensor drivers. Also a Thermometer: the chip's own sensor.
 *
 *  The bus is chosen by the template argument. ds3231<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      ds3231_param_t param;            // read once a second, INT/SQW as the alarm interrupt
 *      ds3231<Stm32HalI2CTransport> rtc(param, &hi2c1, DS3231_ADDR, i2c1Mutex);
 *
 *      for(;;){ rtc.main(HAL_GetTick()); ... }
 *      if(rtc.newData()){ rtc_time_t t; if(rtc.getTime(&t)) ... }
 *      rtc.setTime(t);                  // written at the next main()
 *
 *  Everything after param goes to the transport's own constructor, so
 *  the DS3231 can share an I2C bus (and its mutex) with other drivers.
 *
 *  What it does: checks the chip answers, writes the control register
 *  (built from param and the alarms, never read back and modified) and
 *  the status register's 32kHz enable, then every param.period_ms (start
 *  to start) reads all 19 registers, 0x00..0x12, in one transfer: the
 *  time, both alarms, control, status, aging offset and temperature. The
 *  time registers are latched on the I2C START, so the seven bytes are
 *  from one instant.
 *
 *  Each read is checked: control not what was written (the chip lost
 *  power without a battery, or something else wrote it) is written
 *  again and counted (reconfigs()); the 32kHz enable likewise. OSF, the
 *  oscillator-stop flag, set means the time can't be trusted (a new
 *  chip, a flat battery): timeValid() is false until setTime(), which
 *  writes the seven time registers and then clears OSF.
 *
 *  Times are 24 hour and 2000..2099 (rtc_time_t, RtcDate.h). A chip in
 *  12 hour mode is read correctly; setTime() puts it back in 24 hour
 *  mode. The century bit (month register, bit 7) is ignored and written
 *  0. The day of the week is worked out from the date and written 1..7
 *  (1 Sunday), as the DS1307 driver does; alarms use the same numbering.
 *
 *  Alarms: setAlarm(1 or 2, ...) writes that alarm's registers, clears
 *  its flag and enables it (AxIE). Each read looks at the flags: one set
 *  for an enabled alarm is latched for alarmFired() and cleared on the
 *  chip at once. With param.sqw Off the INT/SQW pin goes low while an
 *  enabled alarm's flag is set (open drain, active low): wire it to an
 *  interrupt and call readNow() from it to see the alarm at once rather
 *  than at the next read. With a square wave on the pin the flags are
 *  still polled. Alarm 1 can match seconds; alarm 2 has no seconds
 *  register and fires at :00. The driver owns the alarms: at start-up
 *  both are disabled until setAlarm() is called.
 *
 *  Status writes: a flag (OSF, A1F, A2F) is cleared by writing 0 and
 *  left as it is by writing 1, so a status write never clears a flag it
 *  doesn't mean to, even one set since the last read.
 *
 *  Temperature: the chip converts every 64 s by itself (0.25 C steps,
 *  10 bit two's complement; 0 C after power-on until the first).
 *  getTemperature() is the last conversion, NAN until the first read.
 *  The aging offset (setAging(), -128..127, about 0.1 ppm a step at 25 C,
 *  plus slows the clock) is applied by a conversion; the driver starts
 *  one (CONV) after writing it, or, while one is running (BSY or CONV:
 *  BSY rises only about 2 ms after CONV is set), as soon as a read shows
 *  it finished. A CONV written during a conversion would not start one.
 *
 *  Checked against Maxim's DS3231 datasheet (19-5170, Rev 10, 3/15): the
 *  address (1101000), the register map and its wrap from 12h to 00h, the
 *  time registers latched on START, the alarm mask table (Table 2), the
 *  control and status bits and power-on values (0x1C, 0x88), A1F/A2F
 *  written 1 left unchanged, OSF held until written 0, CONV and BSY, the
 *  aging offset's sign and step, the temperature format. The datasheet
 *  doesn't say what writing 1 to OSF does; it is written 1 (unchanged,
 *  presumably) except to clear it, the safe way round. On first power the
 *  time reads 2000-01-01 00:00:00, day 1; the alarm registers have no
 *  power-on value (the driver disables both at start-up). The DS3231 is a
 *  400 kHz part. Not yet run against a chip.
 */

#ifndef DS3231_H_
#define DS3231_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"
#include "Thermometer.h"
#include "RtcDate.h"

namespace DS3231 {

#define DS3231_ADDR 0x68	// 7 bit, fixed (the DS1307's too)

using rtc::rtc_time_t;

typedef enum ds3231_state_t{
	ds3231_init_state,				// check a device answers, queue the configuration
	ds3231_done_state,				// between reads; queued writes are started from here
	ds3231_read_state,				// issue the read of 0x00..0x12
	ds3231_wait_read_state,
	ds3231_write_state,				// issue the queued write chosen in done
	ds3231_wait_write_state,
	ds3231_err_state				// pause, then start again from ds3231_init_state
}ds3231_state_t;

typedef enum ds3231_reg_addr_t{
	ds3231_seconds_addr = 0x00,
	ds3231_minutes_addr = 0x01,
	ds3231_hours_addr = 0x02,		// bit 6: 12 hour mode; then bit 5 is PM
	ds3231_day_addr = 0x03,			// day of the week, 1..7
	ds3231_date_addr = 0x04,
	ds3231_month_addr = 0x05,		// bit 7: century
	ds3231_year_addr = 0x06,		// 00..99
	ds3231_alarm1_addr = 0x07,		// seconds, minutes, hours, day/date: bit 7 of each is a mask bit
	ds3231_alarm2_addr = 0x0B,		// minutes, hours, day/date
	ds3231_control_addr = 0x0E,
	ds3231_status_addr = 0x0F,
	ds3231_aging_addr = 0x10,		// two's complement
	ds3231_temp_msb_addr = 0x11,	// two's complement, whole degrees
	ds3231_temp_lsb_addr = 0x12		// bits 7..6: quarter degrees
}ds3231_reg_addr_t;

const uint8_t ds3231_reg_count = 0x13;		// 0x00..0x12, read in one transfer

// Control register (0x0E) bits
const uint8_t DS3231_CTRL_EOSC = 0x80;		// 1 stops the oscillator on battery: never set here
const uint8_t DS3231_CTRL_BBSQW = 0x40;		// square wave on battery too
const uint8_t DS3231_CTRL_CONV = 0x20;		// start a temperature conversion
const uint8_t DS3231_CTRL_RS_MASK = 0x18;	// RS2..RS1, the square wave's rate
const uint8_t DS3231_CTRL_INTCN = 0x04;		// 1: INT/SQW is the alarm interrupt
const uint8_t DS3231_CTRL_A2IE = 0x02;
const uint8_t DS3231_CTRL_A1IE = 0x01;
// Status register (0x0F) bits
const uint8_t DS3231_STAT_OSF = 0x80;		// the oscillator stopped at some time
const uint8_t DS3231_STAT_EN32KHZ = 0x08;
const uint8_t DS3231_STAT_BSY = 0x04;		// a temperature conversion is running
const uint8_t DS3231_STAT_A2F = 0x02;
const uint8_t DS3231_STAT_A1F = 0x01;

// The INT/SQW pin (open drain). Off: it is the alarm interrupt (INTCN 1,
// the power-on state). The rest are square waves (INTCN 0; RS2..RS1), and
// the alarms are only polled. The 1 Hz wave rises 500 ms after the seconds
// are written.
enum class ds3231_sqw_t : uint8_t {
	Off,
	Hz1,
	Hz1024,
	Hz4096,
	Hz8192
};

// What an alarm compares. Each one also matches everything finer:
// Hour is hours, minutes and (alarm 1) seconds.
enum class ds3231_alarm_match_t : uint8_t {
	EverySecond,	// alarm 1 only
	EveryMinute,	// at :00 (alarm 1: seconds 0)
	Second,			// alarm 1 only: the second of every minute
	Minute,			// the minute (and second) of every hour
	Hour,			// the time of every day
	Date,			// the day of the month and the time
	Weekday			// the day of the week (0 Sunday) and the time
};

struct ds3231_alarm_t {
	ds3231_alarm_match_t match;
	uint8_t day;		// Date: 1..31; Weekday: 0 Sunday .. 6 Saturday; else ignored
	uint8_t hour, minute, second;	// alarm 2 ignores second

	ds3231_alarm_t() : match(ds3231_alarm_match_t::Hour), day(1), hour(0), minute(0), second(0) {}
};

// All times in ms.
const uint32_t ds3231_bus_timeout_ms = 100;		// one transfer, to issue or to land
const uint32_t ds3231_error_backoff_ms = 500;	// in ds3231_err_state before starting again
const uint32_t ds3231_request_poll_ms = 100;	// longest sleep, so requests are seen promptly

struct ds3231_param_t {
	uint32_t period_ms;		// between reads, start to start
	ds3231_sqw_t sqw;		// the INT/SQW pin
	bool sqwOnBattery;		// BBSQW: the square wave on battery too (with sqw not Off); else the pin is high impedance on battery
	bool out32kHz;			// EN32kHz: the 32kHz pin's output (the chip starts with it on)

	ds3231_param_t() : period_ms(1000), sqw(ds3231_sqw_t::Off), sqwOnBattery(false), out32kHz(false) {}
};

// What became of the last setTime().
enum class ds3231_set_status_t : uint8_t {
	None,		// no request yet
	Pending,	// waiting to be written
	Done,		// written (and read back at once by the next read)
	Failed		// the write failed; not retried (the time would be stale): call setTime() again
};

// The alarm registers of a, as alarm n (1 or 2) holds them: 4 bytes for
// alarm 1 (seconds first), 3 for alarm 2. False if a can't be an alarm
// n (EverySecond or Second on alarm 2, a field out of range).
inline bool ds3231_encode_alarm(uint8_t n, const ds3231_alarm_t& a, uint8_t* out);

// A read-only look at registers 0x0E..0x12 (5 bytes) for what only a
// DS3231 shows, to tell it from a DS1307 (the same address, RAM there):
// status bits 6..4 and the temperature LSB's bits 5..0 read 0, and the
// temperature is -40..85 C. A DS1307 whose RAM happens to match passes:
// use it with another check (see the L432KC bring-up).
inline bool ds3231_probe_regs(const uint8_t* r0e);

template <typename TTransport>
class ds3231 : public SensorStateMachine<TTransport, ds3231_state_t>, public Thermometer {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"ds3231<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, ds3231_state_t> base;
protected:
	using base::enter;
	using base::fail;
	using base::issued;
	using base::landed;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
public:
	template <typename... TArgs>
	explicit ds3231(const ds3231_param_t& param, TArgs&&... transportArgs);
	virtual ~ds3231() {}

	void main(uint32_t nowMs);

	// A read has landed since the last getTime().
	bool newData() const { return _newData; }
	// The last time read. False (t untouched) until a read with a real
	// date and time has landed, and after a failure. Clears newData().
	bool getTime(rtc_time_t* t);
	// The time can be trusted: OSF was clear at the last read. False
	// until the first read.
	bool timeValid() const { return _haveTime && !_osf; }

	// Queues t to be written at the next main(), replacing one still
	// waiting, then OSF cleared. False, and nothing queued, if t is not a
	// real date and time (rtc::valid()).
	bool setTime(const rtc_time_t& t);
	ds3231_set_status_t setStatus() const { return _setStatus; }

	// Read at the next main() instead of waiting out the period (from
	// the INT pin's interrupt, say: it only sets a flag).
	void readNow() { _readNow = true; }

	// Thermometer: degrees C, the chip's last conversion. NAN until the
	// first read and after a failure.
	float getTemperature() const override { return _temperature; }

	// Alarm n (1 or 2): written, its flag cleared and enabled at the next
	// main(). False, nothing queued, if a can't be an alarm n.
	bool setAlarm(uint8_t n, const ds3231_alarm_t& a);
	// Alarm n disabled (AxIE off) and its flag cleared.
	void disableAlarm(uint8_t n);
	bool alarmEnabled(uint8_t n) const { return n == 1 ? _alarmOn[0] : n == 2 ? _alarmOn[1] : false; }
	// Alarm n fired since the last call (an enabled alarm's flag was seen).
	bool alarmFired(uint8_t n);

	// The aging offset, written at the next main(), then a conversion.
	void setAging(int8_t offset);
	// The aging register at the last read. False until the first read.
	bool getAging(int8_t* offset) const;

	// Control or the 32kHz enable found changed and written again.
	uint32_t reconfigs() const { return _reconfigs; }
	// The registers of the last read (19 bytes, 0x00..0x12), for a
	// diagnostic dump. Valid once newData() has been true.
	const uint8_t* regs() const { return _regs; }

protected:
	void onFail() override;

private:
	// Queued writes, started in this order from ds3231_done_state.
	enum Op : uint8_t {
		OpTime = 0x01,
		OpAlarm1 = 0x02,
		OpAlarm2 = 0x04,
		OpStatus = 0x08,		// OSF and alarm flags cleared, EN32kHz
		OpControl = 0x10,
		OpAging = 0x20,
		OpConvert = 0x40		// control with CONV
	};

	bool decode();					// _regs -> _time, _osf, temperature...; false if not a real time
	uint8_t control(bool conv) const;
	uint8_t pickOp() const;
	uint8_t build(uint8_t op);		// fills _out; returns the length

	ds3231_param_t _param;
	uint32_t _lastRead;

	// The transport fills this after main() has returned: a member.
	uint8_t _regs[ds3231_reg_count];
	uint8_t _setRegs[7];			// the time setTime() asked for, encoded
	uint8_t _alarmRegs[2][4];		// what setAlarm() asked for, encoded
	uint8_t _out[7];				// the write in flight (the transport copies it)
	uint8_t _outAddr;
	uint8_t _outOp;

	rtc_time_t _time;
	float _temperature;
	int8_t _aging;
	int8_t _agingWanted;
	bool _haveTime;
	bool _haveRegs;
	bool _osf;
	bool _busy;						// a conversion running: BSY or CONV at the last read, or CONV just written
	bool _convWanted;				// a conversion to start once none is running (after an aging change)
	bool _newData;
	bool _readNow;
	uint8_t _pending;				// Op bits
	uint8_t _clearFlags;			// status flags (OSF, A1F, A2F) to clear at the next OpStatus
	bool _alarmOn[2];
	bool _fired[2];
	ds3231_set_status_t _setStatus;
	uint32_t _reconfigs;
};

} /* namespace DS3231 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/ds3231.tpp"

#endif /* DS3231_H_ */
