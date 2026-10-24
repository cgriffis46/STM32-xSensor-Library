/*
 * ds3231.tpp
 *
 *  ds3231's member definitions and helpers. ds3231.h includes this file
 *  at its end, since templates have to be visible wherever they are
 *  used: include ds3231.h, not this file.
 */

#ifndef DS3231_TPP_
#define DS3231_TPP_

#include "../inc/ds3231.h"

namespace DS3231 {

// ---- helpers ----

// Alarm mask bits (bit 7 of each alarm register) and DY/DT (bit 6 of the
// day/date register), the datasheet's Table 2 (configurations it doesn't
// list are never written):
//   alarm 1 (A1M1..A1M4): 1111 every second, 1110 seconds, 1100 minutes and
//     seconds, 1000 hours..., 0000 day/date...
//   alarm 2 (A2M2..A2M4): 111 every minute (at :00), 110 minutes, 100
//     hours and minutes, 000 day/date...
inline bool ds3231_encode_alarm(uint8_t n, const ds3231_alarm_t& a, uint8_t* out) {
	if (n != 1 && n != 2) return false;
	if (a.hour > 23 || a.minute > 59 || a.second > 59) return false;
	uint8_t sec = a.second;
	uint8_t masks;			// bit 0 the seconds' mask .. bit 3 the day's
	bool dy = false;
	uint8_t day = 1;
	switch (a.match) {
	case ds3231_alarm_match_t::EverySecond: if (n == 2) return false; masks = 0x0F; break;
	case ds3231_alarm_match_t::EveryMinute: masks = 0x0E; sec = 0; break;
	case ds3231_alarm_match_t::Second:      if (n == 2) return false; masks = 0x0E; break;
	case ds3231_alarm_match_t::Minute:      masks = 0x0C; break;
	case ds3231_alarm_match_t::Hour:        masks = 0x08; break;
	case ds3231_alarm_match_t::Date:
		if (a.day < 1 || a.day > 31) return false;
		masks = 0x00; day = a.day;
		break;
	case ds3231_alarm_match_t::Weekday:
		if (a.day > 6) return false;
		masks = 0x00; dy = true; day = (uint8_t)(a.day + 1);	// 1 Sunday .. 7 Saturday
		break;
	default:
		return false;
	}
	uint8_t regs[4];
	regs[0] = (uint8_t)(rtc::toBcd(sec) | ((masks & 0x01) ? 0x80 : 0));
	regs[1] = (uint8_t)(rtc::toBcd(a.minute) | ((masks & 0x02) ? 0x80 : 0));
	regs[2] = (uint8_t)(rtc::toBcd(a.hour) | ((masks & 0x04) ? 0x80 : 0));	// bit 6 clear: 24 hour
	regs[3] = (uint8_t)(rtc::toBcd(day) | (dy ? 0x40 : 0) | ((masks & 0x08) ? 0x80 : 0));
	const uint8_t first = n == 1 ? 0 : 1;	// alarm 2 has no seconds register
	for (uint8_t i = first; i < 4; ++i) out[i - first] = regs[i];
	return true;
}

inline bool ds3231_probe_regs(const uint8_t* r0e) {
	const int8_t whole = (int8_t)r0e[3];					// 0x11
	return (r0e[1] & 0x70) == 0							// status bits 6..4
		&& (r0e[4] & 0x3F) == 0							// temperature LSB bits 5..0
		&& whole >= -40 && whole <= 85;
}

// ---- ds3231 ----

template <typename TTransport>
template <typename... TArgs>
ds3231<TTransport>::ds3231(const ds3231_param_t& param, TArgs&&... transportArgs)
	: base(ds3231_init_state, ds3231_err_state, ds3231_bus_timeout_ms, std::forward<TArgs>(transportArgs)...),
	  _param(param), _lastRead(0), _outAddr(0), _outOp(0), _time(), _temperature(NAN), _aging(0),
	  _agingWanted(0), _haveTime(false), _haveRegs(false), _osf(true), _busy(false), _convWanted(false), _newData(false),
	  _readNow(true), _pending(0), _clearFlags(0), _setStatus(ds3231_set_status_t::None), _reconfigs(0) {
	for (uint8_t i = 0; i < ds3231_reg_count; ++i) _regs[i] = 0;
	for (uint8_t i = 0; i < 7; ++i) { _setRegs[i] = 0; _out[i] = 0; }
	for (uint8_t i = 0; i < 4; ++i) { _alarmRegs[0][i] = 0; _alarmRegs[1][i] = 0; }
	_alarmOn[0] = _alarmOn[1] = false;
	_fired[0] = _fired[1] = false;
	this->setDebugTag("ds3231");
}

// The control register, built from the parameter and the alarms every
// time: the oscillator always runs (EOSC 0).
template <typename TTransport>
uint8_t ds3231<TTransport>::control(bool conv) const {
	uint8_t c = 0;
	if (_param.sqwOnBattery) c |= DS3231_CTRL_BBSQW;
	if (conv) c |= DS3231_CTRL_CONV;
	switch (_param.sqw) {
	case ds3231_sqw_t::Off:    c |= DS3231_CTRL_INTCN; break;
	case ds3231_sqw_t::Hz1:    break;						// RS 00
	case ds3231_sqw_t::Hz1024: c |= 0x08; break;			// RS 01
	case ds3231_sqw_t::Hz4096: c |= 0x10; break;			// RS 10
	case ds3231_sqw_t::Hz8192: c |= 0x18; break;			// RS 11
	}
	if (_alarmOn[0]) c |= DS3231_CTRL_A1IE;
	if (_alarmOn[1]) c |= DS3231_CTRL_A2IE;
	return c;
}

template <typename TTransport>
uint8_t ds3231<TTransport>::pickOp() const {
	static const uint8_t order[] = {OpTime, OpAlarm1, OpAlarm2, OpStatus, OpControl, OpAging, OpConvert};
	for (uint8_t op : order) if (_pending & op) return op;
	return 0;
}

template <typename TTransport>
uint8_t ds3231<TTransport>::build(uint8_t op) {
	switch (op) {
	case OpTime:
		_outAddr = ds3231_seconds_addr;
		for (uint8_t i = 0; i < 7; ++i) _out[i] = _setRegs[i];
		return 7;
	case OpAlarm1:
		_outAddr = ds3231_alarm1_addr;
		for (uint8_t i = 0; i < 4; ++i) _out[i] = _alarmRegs[0][i];
		return 4;
	case OpAlarm2:
		_outAddr = ds3231_alarm2_addr;
		for (uint8_t i = 0; i < 3; ++i) _out[i] = _alarmRegs[1][i];
		return 3;
	case OpStatus: {
		// A flag written 1 is left as it is; only those asked for are
		// cleared (written 0). BSY is read-only.
		const uint8_t flags = DS3231_STAT_OSF | DS3231_STAT_A2F | DS3231_STAT_A1F;
		_outAddr = ds3231_status_addr;
		_out[0] = (uint8_t)((flags & ~_clearFlags) | (_param.out32kHz ? DS3231_STAT_EN32KHZ : 0));
		return 1;
	}
	case OpControl:
		_outAddr = ds3231_control_addr;
		_out[0] = control(false);
		return 1;
	case OpAging:
		_outAddr = ds3231_aging_addr;
		_out[0] = (uint8_t)_agingWanted;
		return 1;
	case OpConvert:
		_outAddr = ds3231_control_addr;
		_out[0] = control(true);
		return 1;
	default:
		return 0;
	}
}

template <typename TTransport>
void ds3231<TTransport>::main(uint32_t nowMs) {
	switch (_state) {
	case ds3231_init_state:
		if (this->checkDevice()) {
			// The chip is configured from scratch: control and the 32kHz
			// enable, and stale alarm flags cleared (OSF is left: it says
			// whether the time can be trusted).
			_pending |= OpControl | OpStatus;
			_clearFlags |= DS3231_STAT_A1F | DS3231_STAT_A2F;
			_readNow = true;
			enter(ds3231_done_state, nowMs);
		} else {
			fail(nowMs);
		}
		break;

	case ds3231_done_state: {
		const uint8_t op = pickOp();
		if (op != 0) {
			_outOp = op;
			enter(ds3231_write_state, nowMs);
		} else if (_readNow || (nowMs - _lastRead) >= _param.period_ms) {
			this->nextPeriod(_lastRead, nowMs, _param.period_ms);
			_readNow = false;
			enter(ds3231_read_state, nowMs);
		} else {
			// No longer than ds3231_request_poll_ms at a time, so a request
			// is seen without waiting out the period.
			const uint32_t left = _param.period_ms - (nowMs - _lastRead);
			sleep(left < ds3231_request_poll_ms ? left : ds3231_request_poll_ms);
		}
		break;
	}

	case ds3231_read_state:
		issued(this->readRegs(ds3231_seconds_addr, _regs, ds3231_reg_count), ds3231_wait_read_state, nowMs);
		break;

	case ds3231_wait_read_state:
		if (landed(nowMs)) {
			if (decode()) {
				_haveTime = true;
				_newData = true;
			} else {
				_haveTime = false;	// not a real time: nothing to report
			}
			enter(ds3231_done_state, nowMs);
		}
		break;

	case ds3231_write_state: {
		// Built when issued, so it carries the newest request. The
		// transport copies a write, so a request made while it is in
		// flight is queued for the next one.
		const uint8_t len = build(_outOp);
		const bool started = this->writeRegs(_outAddr, _out, len);
		if (started) {
			_pending &= (uint8_t)~_outOp;
			if (_outOp == OpStatus) _clearFlags = 0;
			if (_outOp == OpConvert) {
				_convWanted = false;
				_busy = true;			// until a read shows CONV and BSY both clear
			}
		}
		issued(started, ds3231_wait_write_state, nowMs);
		break;
	}

	case ds3231_wait_write_state:
		if (landed(nowMs)) {
			if (_outOp == OpTime) {
				if (!(_pending & OpTime)) _setStatus = ds3231_set_status_t::Done;
				_readNow = true;			// read it back at once
			} else if (_outOp == OpAging) {
				// Applied by a conversion: one now (not at the next 64 s), or,
				// while one is running, once a read shows it finished.
				_convWanted = true;
				if (!_busy) _pending |= OpConvert;
			}
			enter(ds3231_done_state, nowMs);
		}
		break;

	case ds3231_err_state:
		if (errorCleared(nowMs, ds3231_error_backoff_ms)) enter(ds3231_init_state, nowMs);
		break;

	default:
		enter(ds3231_init_state, nowMs);
		break;
	}
}

template <typename TTransport>
void ds3231<TTransport>::onFail() {
	_haveTime = false;
	_newData = false;
	_temperature = NAN;
	const bool inFlight = _state == ds3231_write_state || _state == ds3231_wait_write_state;
	// A time written after the back-off would be late by then: drop it,
	// and the OSF clear that goes with it.
	if ((_pending & OpTime) || (inFlight && _outOp == OpTime)) {
		_pending &= (uint8_t)~OpTime;
		_clearFlags &= (uint8_t)~DS3231_STAT_OSF;
		_setStatus = ds3231_set_status_t::Failed;
	} else if (inFlight) {
		_pending |= _outOp;			// the rest are tried again after the restart
	}
}

template <typename TTransport>
bool ds3231<TTransport>::decode() {
	const uint8_t* r = _regs;
	const uint8_t status = r[ds3231_status_addr];
	_haveRegs = true;
	_osf = (status & DS3231_STAT_OSF) != 0;
	// A conversion runs while BSY or CONV is set: CONV stays 1 until a
	// conversion it started is done, while BSY only rises about 2 ms after
	// (datasheet, control register bit 5).
	_busy = (status & DS3231_STAT_BSY) != 0 || (r[ds3231_control_addr] & DS3231_CTRL_CONV) != 0;
	if (_convWanted && !_busy) _pending |= OpConvert;
	_aging = (int8_t)r[ds3231_aging_addr];
	_temperature = (float)(int8_t)r[ds3231_temp_msb_addr] + (float)(r[ds3231_temp_lsb_addr] >> 6) * 0.25f;

	// An enabled alarm's flag: latched for alarmFired(), cleared on the chip.
	static const uint8_t flag[2] = {DS3231_STAT_A1F, DS3231_STAT_A2F};
	for (uint8_t i = 0; i < 2; ++i) {
		if (_alarmOn[i] && (status & flag[i]) && !(_clearFlags & flag[i])) {
			_fired[i] = true;
			_clearFlags |= flag[i];
			_pending |= OpStatus;
			DBG_EVENT("ds3231", "alarm", (int32_t)(i + 1));
		}
	}
	// Not what was written (unless a write of it is still to come):
	// written again. CONV clears itself.
	if (!(_pending & OpControl) && (r[ds3231_control_addr] & (uint8_t)~DS3231_CTRL_CONV) != control(false)) {
		_pending |= OpControl;
		++_reconfigs;
		DBG_EVENT("ds3231", "reconfig", (int32_t)r[ds3231_control_addr]);
	}
	if (!(_pending & OpStatus) && ((status & DS3231_STAT_EN32KHZ) != 0) != _param.out32kHz) {
		_pending |= OpStatus;
		++_reconfigs;
		DBG_EVENT("ds3231", "reconfig", (int32_t)status);
	}

	rtc_time_t t;
	uint8_t year;
	if (!rtc::fromBcd(r[0] & 0x7F, &t.second) || !rtc::fromBcd(r[1] & 0x7F, &t.minute)
			|| !rtc::fromBcd(r[4] & 0x3F, &t.day) || !rtc::fromBcd(r[5] & 0x1F, &t.month)
			|| !rtc::fromBcd(r[6], &year) || !rtc::hoursFromReg(r[2], &t.hour)) {
		return false;
	}
	t.year = (uint16_t)(2000 + year);
	if (!rtc::valid(t)) return false;
	t.weekday = rtc::weekday(t.year, t.month, t.day);
	_time = t;
	return true;
}

template <typename TTransport>
bool ds3231<TTransport>::getTime(rtc_time_t* t) {
	_newData = false;
	if (!_haveTime) return false;
	*t = _time;
	return true;
}

template <typename TTransport>
bool ds3231<TTransport>::setTime(const rtc_time_t& t) {
	if (!rtc::valid(t)) return false;
	_setRegs[0] = rtc::toBcd(t.second);
	_setRegs[1] = rtc::toBcd(t.minute);
	_setRegs[2] = rtc::toBcd(t.hour);		// bit 6 clear: 24 hour mode
	_setRegs[3] = (uint8_t)(rtc::weekday(t.year, t.month, t.day) + 1);	// 1 Sunday .. 7 Saturday
	_setRegs[4] = rtc::toBcd(t.day);
	_setRegs[5] = rtc::toBcd(t.month);		// century bit clear
	_setRegs[6] = rtc::toBcd((uint8_t)(t.year - 2000));
	_pending |= OpTime | OpStatus;			// the time, then OSF cleared
	_clearFlags |= DS3231_STAT_OSF;
	_setStatus = ds3231_set_status_t::Pending;
	return true;
}

template <typename TTransport>
bool ds3231<TTransport>::setAlarm(uint8_t n, const ds3231_alarm_t& a) {
	uint8_t regs[4];
	if (!ds3231_encode_alarm(n, a, regs)) return false;
	const uint8_t i = (uint8_t)(n - 1);
	for (uint8_t k = 0; k < 4; ++k) _alarmRegs[i][k] = regs[k];
	_alarmOn[i] = true;
	_fired[i] = false;
	_clearFlags |= i == 0 ? DS3231_STAT_A1F : DS3231_STAT_A2F;
	_pending |= (i == 0 ? OpAlarm1 : OpAlarm2) | OpStatus | OpControl;
	return true;
}

template <typename TTransport>
void ds3231<TTransport>::disableAlarm(uint8_t n) {
	if (n != 1 && n != 2) return;
	const uint8_t i = (uint8_t)(n - 1);
	_alarmOn[i] = false;
	_fired[i] = false;
	_clearFlags |= i == 0 ? DS3231_STAT_A1F : DS3231_STAT_A2F;
	_pending |= OpStatus | OpControl;
}

template <typename TTransport>
bool ds3231<TTransport>::alarmFired(uint8_t n) {
	if (n != 1 && n != 2) return false;
	const bool f = _fired[n - 1];
	_fired[n - 1] = false;
	return f;
}

template <typename TTransport>
void ds3231<TTransport>::setAging(int8_t offset) {
	_agingWanted = offset;
	_pending |= OpAging;
}

template <typename TTransport>
bool ds3231<TTransport>::getAging(int8_t* offset) const {
	if (!_haveRegs) return false;
	*offset = _aging;
	return true;
}

} /* namespace DS3231 */

#endif /* DS3231_TPP_ */
