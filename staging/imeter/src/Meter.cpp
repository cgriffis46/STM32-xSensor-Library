#include "Meter.h"
#include <math.h>

namespace meter {

// ---- requests (any thread) ----

bool Meter::setFunction(Function f) {
	if (f != Function::None && !_fe.supports(f)) return false;
	_req.store(pack(f, false, 0), std::memory_order_release);
	wake();
	return true;
}

// Keeps a function request still waiting: setFunction() then setRange()
// in quick succession means that range of that function.
bool Meter::setRange(uint8_t range) {
	uint32_t old = _req.load(std::memory_order_acquire);
	for (;;) {
		const Function f = (old & kReqValid) ? (Function)((old >> 8) & 0xFF) : Function::None;
		const uint32_t next = pack(f, true, range);
		if (_req.compare_exchange_weak(old, next, std::memory_order_acq_rel)) break;
	}
	wake();
	return true;
}

void Meter::setOpt(uint32_t change, uint32_t valueBit, bool on) {
	uint32_t old = _opt.load(std::memory_order_acquire);
	for (;;) {
		const uint32_t next = (old & ~valueBit) | change | (on ? valueBit : 0u);
		if (_opt.compare_exchange_weak(old, next, std::memory_order_acq_rel)) break;
	}
	wake();
}

void Meter::setAutoRange(bool on) { setOpt(kOptAuto, kOptAutoOn, on); }
void Meter::hold(bool on)         { setOpt(kOptHold, kOptHoldOn, on); }
void Meter::relative(bool on)     { setOpt(kOptRel, kOptRelOn, on); }
void Meter::resetMinMax()         { setOpt(kOptReset, 0, true); }

// ---- the meter's thread ----

void Meter::takeRequests(uint32_t nowMs) {
	const uint32_t req = _req.exchange(0, std::memory_order_acq_rel);
	const uint32_t opt = _opt.exchange(0, std::memory_order_acq_rel);

	bool reselect = false;
	if (req & kReqValid) {
		const Function f = (Function)((req >> 8) & 0xFF);
		const bool manual = (req & kReqManual) != 0;
		if (f != Function::None && f != _fn) {
			_fn = f;
			_rel = false;
			_relArmed = false;
			_mmCount = 0;
			_hold = false;
			_haveHeld = false;
			reselect = true;
			_auto = !manual;
			_range = (uint8_t)(_fe.rangeCount(f) - 1);   // the safe one; a manual range below
		} else if (f == Function::None && !manual) {
			_fn = Function::None;   // off
			_state = State::Off;
			publishStatus(0);
		}
		if (manual && _fn != Function::None) {
			const uint8_t r = (uint8_t)(req >> 16);
			if (r < _fe.rangeCount(_fn) && (r != _range || _auto || reselect)) {
				_range = r;
				_auto = false;
				reselect = true;
			}
		}
	}
	if (opt & kOptAuto) {
		const bool on = (opt & kOptAutoOn) != 0;
		if (on != _auto) {
			_auto = on;
			if (_listener && _fn != Function::None) _listener->onFunctionChanged(_fn, _range, _auto);
		}
	}
	if (opt & kOptHold) {
		_hold = (opt & kOptHoldOn) != 0;
		_haveHeld = false;
	}
	if (opt & kOptRel) {
		_rel = (opt & kOptRelOn) != 0;
		_relArmed = _rel;
		_relRef = 0.0f;
		_mmCount = 0;
	}
	if (opt & kOptReset) _mmCount = 0;

	if (reselect && _fn != Function::None) beginSelect(nowMs);
}

void Meter::beginSelect(uint32_t nowMs) {
	(void)nowMs;
	_state = State::Selecting;
	loadCalibration();
}

void Meter::loadCalibration() {
	_calib = Calibration{1.0f, 0.0f, false};
	Calibration c;
	if (_cal && _cal->load(_fn, _range, &c) && c.valid) _calib = c;
}

void Meter::main(uint32_t nowMs) {
	_nowMs = nowMs;
	takeRequests(nowMs);

	switch (_state) {
	case State::Off:
		sleep(kIdleSleepMs);
		return;

	case State::Selecting:
		if (!_fe.select(_fn, _range, (uint8_t)(_tag + 1))) { sleep(1); return; }
		_tag = (uint8_t)(_tag + 1);
		_settleLeft = _fe.settleResults(_fn, _range);
		_avgTarget = _fe.averageResults(_fn, _range);
		if (_avgTarget == 0) _avgTarget = 1;
		_avgCount = 0;
		_avgSum = 0.0;
		_state = _settleLeft ? State::Settling : State::Measuring;
		if (_listener) _listener->onFunctionChanged(_fn, _range, _auto);
		publishStatus(kSettling);
		return;

	case State::Settling:
	case State::Measuring: {
		RawResult r;
		bool any = false;
		while (_fe.result(_lastSeq, &r)) {
			_lastSeq = r.seq;
			any = true;
			if (r.rangeTag != _tag) continue;   // from before the last select()
			if (_faultShown) _faultShown = false;
			if (_state == State::Settling) {
				if (_settleLeft) _settleLeft--;
				if (_settleLeft == 0) _state = State::Measuring;
				continue;
			}
			averageResult(r);
			if (_state != State::Measuring) break;   // ranging: select again first
		}
		if (_fe.faulted() && !_faultShown) {
			_faultShown = true;
			publishStatus(kFault);
			if (_listener) _listener->onFault();
		}
		if (!any) sleep(kIdleSleepMs);
		return;
	}
	}
}

// Over full scale goes straight through: OL, or auto range, needn't wait
// for the rest of an average that is wrong anyway.
void Meter::averageResult(const RawResult& r) {
	const RangeSpec* spec = _fe.rangeSpec(_fn, _range);
	bool over = r.overload;
	if (spec && !over) {
		const float olRaw = spec->overloadRaw > 0.0f ? spec->overloadRaw : spec->fullScale / spec->unitsPerRaw;
		over = fabsf((r.raw - _calib.offset) * _calib.gain) >= olRaw;
	}
	if (_avgTarget <= 1 || over) {
		_avgCount = 0;
		_avgSum = 0.0;
		handleResult(r);
		return;
	}
	_avgSum += (double)r.raw;
	if (++_avgCount < _avgTarget) return;
	RawResult avg = r;
	avg.raw = (float)(_avgSum / _avgCount);
	_avgCount = 0;
	_avgSum = 0.0;
	handleResult(avg);
}

void Meter::handleResult(const RawResult& r) {
	const RangeSpec* spec = _fe.rangeSpec(_fn, _range);
	if (!spec) return;
	const float olRaw = spec->overloadRaw > 0.0f ? spec->overloadRaw : spec->fullScale / spec->unitsPerRaw;
	const float value = (r.raw - _calib.offset) * _calib.gain * spec->unitsPerRaw;
	const bool ol = r.overload || fabsf(r.raw) >= olRaw || fabsf(value) > spec->fullScale;

	if (_auto) {
		const uint8_t n = _fe.rangeCount(_fn);
		if (ol && _range + 1 < n) {
			_range++;
			beginSelect(_nowMs);
			publishStatus(kSettling | kAutoRange);
			return;
		}
		if (!ol && _range > 0) {
			const RangeSpec* below = _fe.rangeSpec(_fn, (uint8_t)(_range - 1));
			if (below && fabsf(value) < kDownFraction * below->fullScale) {
				_range--;
				beginSelect(_nowMs);
				publishStatus(kSettling | kAutoRange);
				return;
			}
		}
	}

	if (_hold && _haveHeld) return;   // the held reading stays as it is

	Reading rd{};
	rd.atMs = r.atMs;
	rd.function = _fn;
	rd.range = _range;
	rd.unit = spec->unit;
	rd.raw = r.raw;
	rd.status = (uint16_t)((ol ? kOverload : kValid) | (_auto ? kAutoRange : 0) | (_calib.valid ? 0 : kUncal));
	float v = value;
	if (!ol && _relArmed) { _relRef = value; _relArmed = false; }
	if (_rel) { v -= _relRef; rd.status |= kRelative; }
	rd.value = ol ? 0.0f : v;
	if (!ol) {
		if (_mmCount == 0) { _min = v; _max = v; }
		else { if (v < _min) _min = v; if (v > _max) _max = v; }
		_mmCount++;
	}
	rd.min = _min;
	rd.max = _max;
	rd.count = _mmCount;
	if (_hold) { rd.status |= kHold; _haveHeld = true; }
	publish(rd);
}

// A reading with no value: settling, a fault, off.
void Meter::publishStatus(uint16_t status) {
	Reading rd{};
	rd.atMs = _nowMs;
	rd.function = _fn;
	rd.range = _range;
	const RangeSpec* spec = _fn != Function::None ? _fe.rangeSpec(_fn, _range) : nullptr;
	rd.unit = spec ? spec->unit : Unit::None;
	rd.status = (uint16_t)(status | (_auto ? kAutoRange : 0));
	rd.min = _min;
	rd.max = _max;
	rd.count = _mmCount;
	publish(rd);
}

void Meter::publish(Reading& rd) {
	rd.seq = ++_readingSeq;
	_last = rd;
	_latest.publish(rd);
	if (_listener) _listener->onReading(rd);
}

} // namespace meter
