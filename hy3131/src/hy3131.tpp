/*
 * hy3131.tpp
 *
 *  Member definitions of hy3131_dmm<TTransport>. Included at the end of
 *  inc/hy3131.h; not compiled on its own.
 */

#include "../inc/hy3131.h"

template <typename TTransport>
template <typename... TArgs>
hy3131_dmm<TTransport>::hy3131_dmm(const hy3131_param_t& p, TArgs&&... transportArgs)
	: Base(hy3131_stopped, hy3131_error, kBusTimeoutMs, std::forward<TArgs>(transportArgs)...),
	  _param(p), _cfg(hy3131::defaultConfig()), _reqCfg(_cfg) {
	if (_param.pollMs == 0) _param.pollMs = 1;
	if (_param.sysclkHz == 0) _param.sysclkHz = hy3131::kXtalHz;
	memset(_ops, 0, sizeof _ops);
	memset(_rx, 0, sizeof _rx);
	memset(_data, 0, sizeof _data);
	memset(_peak, 0, sizeof _peak);
	memset(_countRegs, 0, sizeof _countRegs);
	memset(_back, 0, sizeof _back);
	memset(&_lastSample, 0, sizeof _lastSample);
	this->setDebugTag("hy3131");
}

// ---- requests ----

template <typename TTransport>
bool hy3131_dmm<TTransport>::start(const hy3131::Config& cfg) {
	_reqCfg = cfg;
	_req.store(Req::Start, std::memory_order_release);
	wake();
	return true;
}

template <typename TTransport>
void hy3131_dmm<TTransport>::stop() {
	_req.store(Req::Stop, std::memory_order_release);
	wake();
}

// Entered with nothing in flight.
template <typename TTransport>
bool hy3131_dmm<TTransport>::handleRequest(uint32_t nowMs) {
	const Req r = _req.exchange(Req::None, std::memory_order_acq_rel);
	if (r == Req::Start) {
		_cfg = _reqCfg;
		this->enter(hy3131_init, nowMs);
		return true;
	}
	if (r == Req::Stop) {
		opsClear();
		write1(hy3131::kR37, hy3131::kResetValue);
		runOps(hy3131_stopped, nowMs);
		return true;
	}
	return false;
}

// ---- transfers ----

template <typename TTransport>
typename hy3131_dmm<TTransport>::Op* hy3131_dmm<TTransport>::add(OpKind k) {
	if (_opCount >= sizeof(_ops) / sizeof(_ops[0])) { DBG_FAULT("hy3131", "op-overflow"); return nullptr; }
	Op* o = &_ops[_opCount++];
	memset(o, 0, sizeof *o);
	o->kind = k;
	return o;
}

template <typename TTransport>
void hy3131_dmm<TTransport>::write(uint8_t addr, const uint8_t* v, uint8_t n) {
	Op* o = add(OpKind::Write);
	if (!o) return;
	if (n > sizeof o->v) n = sizeof o->v;
	o->addr = addr;
	memcpy(o->v, v, n);
	o->len = n;
}

template <typename TTransport>
void hy3131_dmm<TTransport>::read(uint8_t addr, uint8_t* dst, uint8_t n) {
	Op* o = add(OpKind::Read);
	if (!o) return;
	if (n > sizeof(_rx) - 1) n = sizeof(_rx) - 1;
	o->addr = addr;
	o->dst = dst;
	o->len = n;
}

// A read clocks one byte more than it returns: the data is a bit late.
template <typename TTransport>
bool hy3131_dmm<TTransport>::issueOp(const Op& o) {
	if (o.kind == OpKind::Write) {
		const uint8_t hdr = hy3131::cmdWrite(o.addr);
		return this->beginWrite(&hdr, 1, o.v, o.len);
	}
	const uint8_t hdr = hy3131::cmdRead(o.addr);
	return this->beginRead(&hdr, 1, _rx, (size_t)o.len + 1);
}

template <typename TTransport>
void hy3131_dmm<TTransport>::runOps(hy3131_state_t after, uint32_t nowMs) {
	_opPos = 0;
	_afterOps = after;
	_inXfer.store(_opCount != 0, std::memory_order_release);
	this->enter(_opCount ? hy3131_ops : after, nowMs);
}

// ---- configuration ----

template <typename TTransport>
uint8_t hy3131_dmm<TTransport>::cfgLen() const {
	return _param.variant == hy3131::Variant::Hy3130 ? (uint8_t)(hy3131::kR33 - hy3131::kR20 + 1) : hy3131::kCfgCount;
}

// The read-back stops at R35 (R33 on the HY3130): one more would end the
// read inside R37, the test register.
template <typename TTransport>
uint8_t hy3131_dmm<TTransport>::backLen() const {
	return _param.variant == hy3131::Variant::Hy3130 ? (uint8_t)(hy3131::kR33 - hy3131::kR20 + 1)
	                                                 : (uint8_t)(hy3131::kR35 - hy3131::kR20 + 1);
}

template <typename TTransport>
uint32_t hy3131_dmm<TTransport>::clampGate(uint32_t g) {
	if (g < 0x100u) g = 0x100u;
	if (g > 0xFFFF00u) g = 0xFFFF00u;
	return g;
}

// CTA<23:8>, so that CTA counts up from there and overflows after the gate.
// CTA<7:0> is cleared by ENCTR = 0.
template <typename TTransport>
void hy3131_dmm<TTransport>::queueGate() {
	_ctaInit = (0x1000000u - clampGate(_cfg.gateCycles)) & 0xFFFF00u;
	const uint8_t v[2] = { (uint8_t)(_ctaInit >> 8), (uint8_t)(_ctaInit >> 16) };
	write((uint8_t)(hy3131::kCta + 1), v, 2);
}

template <typename TTransport>
void hy3131_dmm<TTransport>::queueReadBack() {
	read(hy3131::kR20, _back, backLen());
}

// ENCTR is left out: whether the chip clears it when a count finishes isn't said.
template <typename TTransport>
bool hy3131_dmm<TTransport>::configMatches() const {
	using namespace hy3131;
	for (uint8_t i = 0; i < backLen(); i++) {
		uint8_t m = kCfgDefinedHy3131[i];
		if (i == cfgIndex(kR20)) m = (uint8_t)(m & ~kR20Enctr);
		if ((_back[i] & m) != (_cfg.reg[i] & m)) {
			DBG_FAULT("hy3131", "cfg-mismatch", (int32_t)(kR20 + i), (int32_t)_back[i]);
			return false;
		}
	}
	return true;
}

// ---- service ----

template <typename TTransport>
void hy3131_dmm<TTransport>::beginService(uint32_t nowMs) {
	_lastPollMs = nowMs;
	opsClear();
	read(hy3131::kIntf, &_intf, 1);   // ends in INTE: harmless
	runOps(hy3131_check_flags, nowMs);
}

template <typename TTransport>
void hy3131_dmm<TTransport>::publishResults(uint32_t nowMs) {
	using namespace hy3131;
	const bool peakNow = _readPeak || _peakWithCount;
	if (_readData || peakNow) {
		Sample s = _lastSample;
		s.seq = ++_sampleSeq;
		s.atMs = nowMs;
		s.flags = (uint8_t)(_flags & ~kIntCt);
		if (_readData) {
			s.ad1 = le24(&_data[kAd1 - kAd1]);
			s.ad2 = le24(&_data[kAd2 - kAd1]);
			s.lpf = le24(&_data[kLpf - kAd1]);
			s.rms = le40(&_data[kRms - kAd1]);
		}
		if (peakNow) {
			const uint8_t* p = _readPeak ? _peak : _countRegs;
			s.peakMin = le24(&p[kPkhMin - kPkhMin]);
			s.peakMax = le24(&p[kPkhMax - kPkhMin]);
			s.peakValid = true;
		}
		_lastSample = s;
		publish(_sample, s);
		if (_listener) _listener->onSample(s);
	}
	if (_readCount) {
		const uint8_t* r = &_countRegs[kPeakLen];   // CTSTA at 0x14
		Count c;
		c.seq = ++_countSeq;
		c.atMs = nowMs;
		c.ctaInit = _countGate;
		c.ctsta = r[0];
		c.ctc = le24u(&r[kCtc - kCtsta]);
		c.ctb = le24u(&r[kCtb - kCtsta]);
		c.ctaFinal = le24u(&r[kCta - kCtsta]);
		c.overflow = (r[0] & kCtstaCtbov) != 0;
		publish(_count, c);
		if (_listener) _listener->onCount(c);
	}
}

template <typename TTransport>
void hy3131_dmm<TTransport>::onFail() {
	_faults++;
	opsClear();
	_carry = 0;
	_inXfer.store(false, std::memory_order_release);
	if (_listener) _listener->onFault();
}

// ---- state machine ----

template <typename TTransport>
void hy3131_dmm<TTransport>::main(uint32_t nowMs) {
	using namespace hy3131;

	switch (this->_state) {
	case hy3131_stopped:
		if (!handleRequest(nowMs)) this->sleep(kIdleSleepMs);
		break;

	case hy3131_init:
		opsClear();
		_carry = 0;
		write1(kR37, kResetValue);
		runOps(hy3131_reset_settle, nowMs);
		break;

	case hy3131_ops:
		this->issued(issueOp(_ops[_opPos]), hy3131_wait_ops, nowMs);
		break;

	case hy3131_wait_ops:
		if (this->landed(nowMs)) {
			const Op& o = _ops[_opPos];
			if (o.kind == OpKind::Read && o.dst) unshiftRead(_rx, o.dst, o.len);
			if (++_opPos < _opCount) {
				this->enter(hy3131_ops, nowMs);
			} else {
				_inXfer.store(false, std::memory_order_release);
				this->enter(_afterOps, nowMs);
			}
		}
		break;

	case hy3131_reset_settle: {
		if (!this->elapsed(nowMs, kResetMs)) { this->sleepRemaining(nowMs, kResetMs); break; }
		// Everything but the enables of the ADCs and the counter.
		uint8_t img[kCfgCount];
		memcpy(img, _cfg.reg, sizeof img);
		img[cfgIndex(kR20)] &= (uint8_t)~kR20Enctr;
		img[cfgIndex(kR23)] &= (uint8_t)~kR23Enad1;
		img[cfgIndex(kR26)] &= (uint8_t)~kR26Enad2;
		img[cfgIndex(kR34)] &= (uint8_t)~kR34Enad3;
		opsClear();
		write1(kInte, 0);
		write1(kIntf, 0);
		write(kR20, img, cfgLen());
		runOps(hy3131_osc_settle, nowMs);
		break;
	}

	case hy3131_osc_settle:
		if (!this->elapsed(nowMs, kOscSettleMs)) { this->sleepRemaining(nowMs, kOscSettleMs); break; }
		opsClear();
		if (_cfg.counterOn()) queueGate();
		write(kR20, _cfg.reg, (uint8_t)(kR26 - kR20 + 1));   // ENCTR, ENAD1, ENAD2 in one transfer
		if (_param.variant == Variant::Hy3131) write1(kR34, _cfg.r(kR34));
		queueReadBack();
		write1(kIntf, 0);
		write1(kInte, _cfg.inte);
		runOps(hy3131_check_config, nowMs);
		break;

	case hy3131_check_config:
		if (_param.verifyConfig && !configMatches()) { this->fail(nowMs); break; }
		DBG_EVENT("hy3131", "running");
		_lastPollMs = nowMs;
		_lastHealthMs = nowMs;
		_irqPending.store(true, std::memory_order_release);   // look once straight away
		if (_listener) _listener->onRunning();
		this->enter(hy3131_running, nowMs);
		break;

	case hy3131_running: {
		if (handleRequest(nowMs)) break;
		const bool irq = _irqPending.exchange(false, std::memory_order_acq_rel);
		const bool level = _param.doLevel && _param.doLevel(_param.doCtx);
		const uint32_t sincePoll = nowMs - _lastPollMs;
		if (irq || level || _carry || sincePoll >= _param.pollMs) { beginService(nowMs); break; }
		const uint32_t sinceHealth = nowMs - _lastHealthMs;
		if (_param.healthMs && sinceHealth >= _param.healthMs) {
			_lastHealthMs = nowMs;
			opsClear();
			queueReadBack();
			runOps(hy3131_check_health, nowMs);
			break;
		}
		uint32_t ms = _param.pollMs - sincePoll;
		if (_param.healthMs && _param.healthMs - sinceHealth < ms) ms = _param.healthMs - sinceHealth;
		if (ms > kIdleSleepMs) ms = kIdleSleepMs;
		this->sleep(ms);
		break;
	}

	case hy3131_check_flags: {
		const uint8_t raw = (uint8_t)(_intf | _carry);
		_carry = 0;
		if (raw & kIntBor) {
			DBG_FAULT("hy3131", "brown-out");
			this->fail(nowMs);
			break;
		}
		const uint8_t f = (uint8_t)(raw & kIntAll);
		if (f == 0) { this->enter(hy3131_running, nowMs); break; }
		_flags = f;
		const bool adc = (f & (kIntAd1 | kIntAd2 | kIntLpf | kIntRms)) != 0;
		_readData = adc;
		_readCount = (f & kIntCt) && _cfg.counterOn();
		_peakWithCount = _readCount && _cfg.peakOn();
		_readPeak = adc && _cfg.peakOn() && !_cfg.counterOn();
		opsClear();
		if (_readData) read(kAd1, _data, kDataLen);              // ends in PKHMIN
		if (_readPeak) read(kPkhMin, _peak, kPeakLen);           // ends in CTSTA: the counter is off
		if (_readCount) {
			// Through INTF, so as not to end in it; its flags are carried.
			if (_peakWithCount) read(kPkhMin, _countRegs, (uint8_t)(kPeakLen + kCountLen));
			else                read(kCtsta, &_countRegs[kPeakLen], kCountLen);
			_countGate = _ctaInit;
			write1(kR20, (uint8_t)(_cfg.r(kR20) & ~kR20Enctr));   // clears CTA<7:0>, CTB, CTC
			queueGate();
			write1(kR20, _cfg.r(kR20));
		}
		runOps(hy3131_serviced, nowMs);
		break;
	}

	case hy3131_serviced:
		publishResults(nowMs);
		if (_readCount) _carry = (uint8_t)(_countRegs[kPeakLen + kCountLen - 1] & (kIntAll | kIntBor));
		beginService(nowMs);   // INTF again, until it reads 0
		break;

	case hy3131_check_health:
		if (_param.verifyConfig && !configMatches()) { this->fail(nowMs); break; }
		this->enter(hy3131_running, nowMs);
		break;

	case hy3131_error:
		if (!this->errorCleared(nowMs, kErrorBackoffMs)) break;
		if (handleRequest(nowMs)) break;
		this->enter(hy3131_init, nowMs);
		break;
	}
}
