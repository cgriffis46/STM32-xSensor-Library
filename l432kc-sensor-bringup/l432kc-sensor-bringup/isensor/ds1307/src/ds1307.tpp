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

// ---- date helpers: RtcDate.h's ----

inline uint8_t ds1307_weekday(uint16_t year, uint8_t month, uint8_t day) { return rtc::weekday(year, month, day); }
inline bool ds1307_valid(const ds1307_time_t& t) { return rtc::valid(t); }
inline uint32_t ds1307_to_unix(const ds1307_time_t& t) { return rtc::toUnix(t); }
inline void ds1307_from_unix(uint32_t s, ds1307_time_t* t) { rtc::fromUnix(s, t); }

// ---- ds1307 ----

template <typename TTransport>
template <typename... TArgs>
ds1307<TTransport>::ds1307(const ds1307_param_t& param, TArgs&&... transportArgs)
	: base(ds1307_init_state, ds1307_err_state, ds1307_bus_timeout_ms, std::forward<TArgs>(transportArgs)...),
	  _param(param), _lastRead(0), _time(), _haveTime(false), _running(false), _newData(false),
	  _readNow(false), _setWanted(false), _setStatus(ds1307_set_status_t::None),
	  _ramOutOffset(0), _ramOutLen(0), _ramWriteWanted(false), _ramReadWanted(false),
	  _ramWriteStatus(ds1307_set_status_t::None), _ramReadStatus(ds1307_set_status_t::None) {
	for (uint8_t i = 0; i < 7; ++i) { _regs[i] = 0; _setRegs[i] = 0; }
	for (uint8_t i = 0; i < ds1307_ram_size; ++i) _ram[i] = 0;
	for (uint8_t i = 0; i < ISensorTransport::kMaxWriteLen; ++i) _ramOut[i] = 0;
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
		} else if (_ramWriteWanted) {
			enter(ds1307_ram_write_state, nowMs);
		} else if (_ramReadWanted) {
			enter(ds1307_ram_read_state, nowMs);
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

	case ds1307_ram_write_state: {
		const bool started = this->writeRegs((uint8_t)(ds1307_ram_addr + _ramOutOffset), _ramOut, _ramOutLen);
		if (started) _ramWriteWanted = false;
		issued(started, ds1307_wait_ram_write_state, nowMs);
		break;
	}

	case ds1307_wait_ram_write_state:
		if (landed(nowMs)) {
			if (!_ramWriteWanted) _ramWriteStatus = ds1307_set_status_t::Done;
			enter(ds1307_done_state, nowMs);
		}
		break;

	case ds1307_ram_read_state: {
		const bool started = this->readRegs(ds1307_ram_addr, _ram, ds1307_ram_size);
		if (started) _ramReadWanted = false;
		issued(started, ds1307_wait_ram_read_state, nowMs);
		break;
	}

	case ds1307_wait_ram_read_state:
		if (landed(nowMs)) {
			if (!_ramReadWanted) _ramReadStatus = ds1307_set_status_t::Done;
			enter(ds1307_done_state, nowMs);
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
	if (_ramWriteWanted || _state == ds1307_ram_write_state || _state == ds1307_wait_ram_write_state) {
		_ramWriteWanted = false;
		_ramWriteStatus = ds1307_set_status_t::Failed;
	}
	if (_ramReadWanted || _state == ds1307_ram_read_state || _state == ds1307_wait_ram_read_state) {
		_ramReadWanted = false;
		_ramReadStatus = ds1307_set_status_t::Failed;
	}
}

template <typename TTransport>
bool ds1307<TTransport>::decode() {
	ds1307_time_t t;
	uint8_t year;
	if (!rtc::fromBcd(_regs[0] & 0x7F, &t.second) || !rtc::fromBcd(_regs[1] & 0x7F, &t.minute)
			|| !rtc::fromBcd(_regs[4] & 0x3F, &t.day) || !rtc::fromBcd(_regs[5] & 0x1F, &t.month)
			|| !rtc::fromBcd(_regs[6], &year) || !rtc::hoursFromReg(_regs[2], &t.hour)) {
		return false;
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
	out[0] = rtc::toBcd(t.second);		// CH clear: the oscillator runs
	out[1] = rtc::toBcd(t.minute);
	out[2] = rtc::toBcd(t.hour);		// bit 6 clear: 24 hour mode
	out[3] = (uint8_t)(ds1307_weekday(t.year, t.month, t.day) + 1);	// 1 Sunday .. 7 Saturday
	out[4] = rtc::toBcd(t.day);
	out[5] = rtc::toBcd(t.month);
	out[6] = rtc::toBcd((uint8_t)(t.year - 2000));
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

template <typename TTransport>
bool ds1307<TTransport>::writeRam(uint8_t offset, const uint8_t* data, uint8_t len) {
	if (data == nullptr || len == 0 || len > ISensorTransport::kMaxWriteLen
			|| offset >= ds1307_ram_size || len > ds1307_ram_size - offset) {
		return false;
	}
	for (uint8_t i = 0; i < len; ++i) _ramOut[i] = data[i];
	_ramOutOffset = offset;
	_ramOutLen = len;
	_ramWriteWanted = true;
	_ramWriteStatus = ds1307_set_status_t::Pending;
	return true;
}

template <typename TTransport>
void ds1307<TTransport>::readRam() {
	_ramReadWanted = true;
	_ramReadStatus = ds1307_set_status_t::Pending;
}

template <typename TTransport>
bool ds1307<TTransport>::getRam(uint8_t* out) const {
	if (out == nullptr || _ramReadStatus != ds1307_set_status_t::Done) return false;
	for (uint8_t i = 0; i < ds1307_ram_size; ++i) out[i] = _ram[i];
	return true;
}

} /* namespace DS1307 */

#endif /* DS1307_TPP_ */
