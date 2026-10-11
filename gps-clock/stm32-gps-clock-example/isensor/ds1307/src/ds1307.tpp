/*
 * ds1307.tpp
 *
 *  ds1307's member definitions and the date helpers. ds1307.h includes
 *  this file at its end, since templates have to be visible wherever
 *  they are used: include ds1307.h, not this file.
 */

#ifndef DS1307_TPP_
#define DS1307_TPP_

#include "../inc/ds1307.h"

namespace DS1307 {

// ---- date helpers ----

// Days since 1970-01-01 of a date in the proleptic Gregorian calendar
// (Howard Hinnant's days_from_civil; general knowledge, checked by the
// test against known dates).
inline int32_t ds1307DaysFromCivil(int32_t y, uint32_t m, uint32_t d) {
	y -= m <= 2;
	const int32_t era = (y >= 0 ? y : y - 399) / 400;
	const uint32_t yoe = (uint32_t)(y - era * 400);
	const uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
	const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097 + (int32_t)doe - 719468;
}

inline uint8_t ds1307DaysInMonth(uint16_t year, uint8_t month) {
	static const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	if (month == 2 && (year % 4 == 0) && (year % 100 != 0 || year % 400 == 0)) return 29;
	return days[month - 1];
}

inline uint8_t ds1307_weekday(uint16_t year, uint8_t month, uint8_t day) {
	// 1970-01-01 was a Thursday (4).
	return (uint8_t)((ds1307DaysFromCivil(year, month, day) + 4) % 7);
}

inline bool ds1307_valid(const ds1307_time_t& t) {
	return t.year >= 2000 && t.year <= 2099
		&& t.month >= 1 && t.month <= 12
		&& t.day >= 1 && t.day <= ds1307DaysInMonth(t.year, t.month)
		&& t.hour <= 23 && t.minute <= 59 && t.second <= 59;
}

inline uint32_t ds1307_to_unix(const ds1307_time_t& t) {
	const uint32_t days = (uint32_t)ds1307DaysFromCivil(t.year, t.month, t.day);
	return days * 86400u + t.hour * 3600u + t.minute * 60u + t.second;
}

inline void ds1307_from_unix(uint32_t s, ds1307_time_t* t) {
	const uint32_t days = s / 86400u;
	const uint32_t secs = s % 86400u;
	t->hour = (uint8_t)(secs / 3600u);
	t->minute = (uint8_t)(secs / 60u % 60u);
	t->second = (uint8_t)(secs % 60u);
	t->weekday = (uint8_t)((days + 4) % 7);
	// civil_from_days (Howard Hinnant), for days >= 0
	const uint32_t z = days + 719468;
	const uint32_t era = z / 146097;
	const uint32_t doe = z - era * 146097;
	const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	const uint32_t mp = (5 * doy + 2) / 153;
	const uint32_t d = doy - (153 * mp + 2) / 5 + 1;
	const uint32_t m = mp < 10 ? mp + 3 : mp - 9;
	const uint32_t y = yoe + era * 400 + (m <= 2);
	t->year = (uint16_t)(y > 0xFFFF ? 0xFFFF : y);
	t->month = (uint8_t)m;
	t->day = (uint8_t)d;
}

// BCD, refusing a nibble above 9 (a chip that was never set, or a bad read).
inline bool ds1307Bcd(uint8_t v, uint8_t* out) {
	if ((v & 0x0F) > 9 || (v >> 4) > 9) return false;
	*out = (uint8_t)((v >> 4) * 10 + (v & 0x0F));
	return true;
}

inline uint8_t ds1307ToBcd(uint8_t v) {
	return (uint8_t)(((v / 10) << 4) | (v % 10));
}

// ---- ds1307 ----

template <typename TTransport>
template <typename... TArgs>
ds1307<TTransport>::ds1307(const ds1307_param_t& param, TArgs&&... transportArgs)
	: base(ds1307_init_state, ds1307_err_state, ds1307_bus_timeout_ms, std::forward<TArgs>(transportArgs)...),
	  _param(param), _lastRead(0), _time(), _haveTime(false), _running(false), _newData(false),
	  _readNow(false), _setWanted(false), _setStatus(ds1307_set_status_t::None) {
	for (uint8_t i = 0; i < 7; ++i) { _regs[i] = 0; _setRegs[i] = 0; }
	this->setDebugTag("ds1307");
}

template <typename TTransport>
void ds1307<TTransport>::main(uint32_t nowMs) {
	switch (_state) {
	case ds1307_init_state:
		if (this->checkDevice()) {
			enter(ds1307_control_state, nowMs);
		} else {
			fail(nowMs);
		}
		break;

	case ds1307_control_state:
		// Built from the parameter each time, never read back and modified.
		issued(this->writeReg(ds1307_control_addr, (uint8_t)_param.sqw), ds1307_wait_control_state, nowMs);
		break;

	case ds1307_wait_control_state:
		if (landed(nowMs)) enter(ds1307_read_state, nowMs);
		break;

	case ds1307_done_state:
		if (_setWanted) {
			enter(ds1307_write_time_state, nowMs);
		} else if (_readNow || (nowMs - _lastRead) >= _param.period_ms) {
			enter(ds1307_read_state, nowMs);
		} else {
			// No longer than ds1307_request_poll_ms at a time, so a
			// setTime() or readNow() is seen without waiting out the period.
			const uint32_t left = _param.period_ms - (nowMs - _lastRead);
			sleep(left < ds1307_request_poll_ms ? left : ds1307_request_poll_ms);
		}
		break;

	case ds1307_read_state:
		_lastRead = nowMs;
		issued(this->readRegs(ds1307_seconds_addr, _regs, 7), ds1307_wait_read_state, nowMs);
		break;

	case ds1307_wait_read_state:
		if (landed(nowMs)) {
			_readNow = false;
			if (decode()) {
				_haveTime = true;
				_newData = true;
			} else {
				_haveTime = false;	// not a real time: nothing to report
			}
			enter(ds1307_done_state, nowMs);
		}
		break;

	case ds1307_write_time_state: {
		// The transport copies a write, so a setTime() made while this
		// one is in flight is kept for the next write.
		const bool started = this->writeRegs(ds1307_seconds_addr, _setRegs, 7);
		if (started) _setWanted = false;
		issued(started, ds1307_wait_write_time_state, nowMs);
		break;
	}

	case ds1307_wait_write_time_state:
		if (landed(nowMs)) {
			if (!_setWanted) _setStatus = ds1307_set_status_t::Done;
			enter(ds1307_read_state, nowMs);	// read it back at once
		}
		break;

	case ds1307_err_state:
		if (errorCleared(nowMs, ds1307_error_backoff_ms)) enter(ds1307_init_state, nowMs);
		break;

	default:
		enter(ds1307_init_state, nowMs);
		break;
	}
}

template <typename TTransport>
void ds1307<TTransport>::onFail() {
	_haveTime = false;
	_newData = false;
	// A time written after the back-off would be late by then: drop it.
	if (_setWanted || _state == ds1307_write_time_state || _state == ds1307_wait_write_time_state) {
		_setWanted = false;
		_setStatus = ds1307_set_status_t::Failed;
	}
}

template <typename TTransport>
bool ds1307<TTransport>::decode() {
	ds1307_time_t t;
	uint8_t year, hour;
	if (!ds1307Bcd(_regs[0] & 0x7F, &t.second) || !ds1307Bcd(_regs[1] & 0x7F, &t.minute)
			|| !ds1307Bcd(_regs[4] & 0x3F, &t.day) || !ds1307Bcd(_regs[5] & 0x1F, &t.month)
			|| !ds1307Bcd(_regs[6], &year)) {
		return false;
	}
	if (_regs[2] & 0x40) {
		// 12 hour mode: 1..12, bit 5 PM. 12 AM is 0, 12 PM is 12.
		if (!ds1307Bcd(_regs[2] & 0x1F, &hour) || hour < 1 || hour > 12) return false;
		t.hour = (uint8_t)(hour % 12 + ((_regs[2] & 0x20) ? 12 : 0));
	} else {
		if (!ds1307Bcd(_regs[2] & 0x3F, &t.hour)) return false;
	}
	t.year = (uint16_t)(2000 + year);
	if (!ds1307_valid(t)) return false;
	t.weekday = ds1307_weekday(t.year, t.month, t.day);
	_time = t;
	_running = (_regs[0] & 0x80) == 0;
	return true;
}

template <typename TTransport>
void ds1307<TTransport>::encode(const ds1307_time_t& t, uint8_t* out) const {
	out[0] = ds1307ToBcd(t.second);		// CH clear: the oscillator runs
	out[1] = ds1307ToBcd(t.minute);
	out[2] = ds1307ToBcd(t.hour);		// bit 6 clear: 24 hour mode
	out[3] = (uint8_t)(ds1307_weekday(t.year, t.month, t.day) + 1);	// 1 Sunday .. 7 Saturday
	out[4] = ds1307ToBcd(t.day);
	out[5] = ds1307ToBcd(t.month);
	out[6] = ds1307ToBcd((uint8_t)(t.year - 2000));
}

template <typename TTransport>
bool ds1307<TTransport>::getTime(ds1307_time_t* t) {
	_newData = false;
	if (!_haveTime) return false;
	*t = _time;
	return true;
}

template <typename TTransport>
bool ds1307<TTransport>::setTime(const ds1307_time_t& t) {
	if (!ds1307_valid(t)) return false;
	encode(t, _setRegs);
	_setWanted = true;
	_setStatus = ds1307_set_status_t::Pending;
	return true;
}

} /* namespace DS1307 */

#endif /* DS1307_TPP_ */
