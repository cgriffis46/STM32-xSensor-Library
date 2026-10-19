/*
 * mcp2515.tpp
 *
 *  Member definitions of mcp2515<TTransport>. Included at the end of
 *  inc/mcp2515.h; not compiled on its own.
 */

#include "../inc/mcp2515.h"

template <typename TTransport>
template <typename... TArgs>
mcp2515<TTransport>::mcp2515(const mcp2515_param_t& p, TArgs&&... transportArgs)
	: Base(mcp2515_stopped, mcp2515_error, kBusTimeoutMs, std::forward<TArgs>(transportArgs)...),
	  _param(p), _cfg(can::defaultConfig(500000)), _reqCfg(_cfg) {
	if (_param.pollMs == 0) _param.pollMs = 1;
	memset(_ops, 0, sizeof _ops);
	memset(_stat, 0, sizeof _stat);
	memset(_cnfBack, 0, sizeof _cnfBack);
	memset(_rxCtrlBack, 0, sizeof _rxCtrlBack);
	memset(_flags, 0, sizeof _flags);
	memset(_health, 0, sizeof _health);
	memset(_rxb, 0, sizeof _rxb);
	memset(_rxq, 0, sizeof _rxq);
	memset(_txq, 0, sizeof _txq);
	this->setDebugTag("mcp2515");
}

// ---- iCan ----

template <typename TTransport>
bool mcp2515<TTransport>::start(const can::Config& cfg) {
	mcp251x::Timing t;
	if (cfg.mode != can::Mode::Normal && cfg.mode != can::Mode::ListenOnly && cfg.mode != can::Mode::Loopback) return false;
	if (!mcp251x::computeTiming(_param.oscHz, cfg.bitrate, cfg.samplePointPermille, &t)) return false;
	_reqCfg = cfg;
	_req.store(Req::Start, std::memory_order_release);
	wake();
	return true;
}

template <typename TTransport>
void mcp2515<TTransport>::stop() {
	_req.store(Req::Stop, std::memory_order_release);
	wake();
}

template <typename TTransport>
bool mcp2515<TTransport>::send(const can::Frame& f) {
	if (!can::valid(f)) return false;
	const can::State s = state();
	if (s != can::State::ErrorActive && s != can::State::ErrorWarning && s != can::State::ErrorPassive) return false;
	if (_expectMode == mcp251x::kOpListenOnly) return false;
	const uint8_t head = _txHead.load(std::memory_order_relaxed);
	const uint8_t tail = _txTail.load(std::memory_order_acquire);
	if ((uint8_t)(head - tail) >= kTxQueue) return false;
	_txq[head % kTxQueue] = f;
	_txHead.store((uint8_t)(head + 1), std::memory_order_release);
	wake();
	return true;
}

template <typename TTransport>
bool mcp2515<TTransport>::receive(can::Frame* f) {
	const uint8_t tail = _rxTail.load(std::memory_order_relaxed);
	const uint8_t head = _rxHead.load(std::memory_order_acquire);
	if (head == tail) return false;
	*f = _rxq[tail % kRxQueue];
	_rxTail.store((uint8_t)(tail + 1), std::memory_order_release);
	return true;
}

template <typename TTransport>
uint8_t mcp2515<TTransport>::txPending() const {
	const uint8_t queued = (uint8_t)(_txHead.load(std::memory_order_acquire) - _txTail.load(std::memory_order_acquire));
	const uint8_t b = _txBusy;
	return (uint8_t)(queued + (b & 1) + ((b >> 1) & 1) + ((b >> 2) & 1));
}

// ---- transfers ----

template <typename TTransport>
typename mcp2515<TTransport>::Op* mcp2515<TTransport>::add(OpKind k) {
	if (_opCount >= sizeof(_ops) / sizeof(_ops[0])) { DBG_FAULT("mcp2515", "op-overflow"); return nullptr; }   // 8 at most (startup)
	Op* o = &_ops[_opCount++];
	memset(o, 0, sizeof *o);
	o->kind = k;
	return o;
}

template <typename TTransport>
void mcp2515<TTransport>::write(uint8_t addr, const uint8_t* v, uint8_t n) {
	Op* o = add(OpKind::Write);
	if (!o) return;
	o->hdr[0] = mcp251x::kInstrWrite;
	o->hdr[1] = addr;
	o->hdrLen = 2;
	if (n > sizeof o->v) n = sizeof o->v;
	memcpy(o->v, v, n);
	o->len = n;
}

template <typename TTransport>
void mcp2515<TTransport>::read(uint8_t addr, uint8_t* dst, uint8_t n) {
	Op* o = add(OpKind::Read);
	if (!o) return;
	o->hdr[0] = mcp251x::kInstrRead;
	o->hdr[1] = addr;
	o->hdrLen = 2;
	o->dst = dst;
	o->len = n;
}

template <typename TTransport>
void mcp2515<TTransport>::bitModify(uint8_t addr, uint8_t mask, uint8_t value) {
	Op* o = add(OpKind::Write);
	if (!o) return;
	o->hdr[0] = mcp251x::kInstrBitModify;
	o->hdr[1] = addr;
	o->hdr[2] = mask;
	o->hdr[3] = value;
	o->hdrLen = 4;
}

template <typename TTransport>
void mcp2515<TTransport>::instr(uint8_t instruction) {
	Op* o = add(OpKind::Write);
	if (!o) return;
	o->hdr[0] = instruction;
	o->hdrLen = 1;
}

// The whole buffer from SIDH; ending the read clears RXnIF.
template <typename TTransport>
void mcp2515<TTransport>::readRx(uint8_t n) {
	Op* o = add(OpKind::Read);
	if (!o) return;
	o->hdr[0] = mcp251x::instrReadRx(n);
	o->hdrLen = 1;
	o->dst = _rxb[n];
	o->len = mcp251x::kFrameLen;
}

// SIDH, SIDL, EID8, EID0, DLC and the data (none for a remote frame).
template <typename TTransport>
void mcp2515<TTransport>::loadTx(uint8_t n, const can::Frame& f) {
	using namespace mcp251x;
	Op* o = add(OpKind::Write);
	if (!o) return;
	o->hdr[0] = instrLoadTx(n);
	o->hdrLen = 1;
	if (f.extended) {
		o->v[0] = (uint8_t)(f.id >> 21);
		o->v[1] = (uint8_t)((((f.id >> 18) & 0x07) << 5) | kSidlIde | ((f.id >> 16) & 0x03));
		o->v[2] = (uint8_t)(f.id >> 8);
		o->v[3] = (uint8_t)f.id;
	} else {
		o->v[0] = (uint8_t)(f.id >> 3);
		o->v[1] = (uint8_t)((f.id & 0x07) << 5);
		o->v[2] = 0;
		o->v[3] = 0;
	}
	o->v[4] = (uint8_t)((f.rtr ? kDlcRtr : 0) | (f.dlc & 0x0F));
	const uint8_t data = f.rtr ? 0 : f.dlc;
	memcpy(&o->v[5], f.data, data);
	o->len = (uint8_t)(5 + data);
}

template <typename TTransport>
bool mcp2515<TTransport>::issueOp(const Op& o) {
	if (o.kind == OpKind::Write) return this->beginWrite(o.hdr, o.hdrLen, o.v, o.len);
	return this->beginRead(o.hdr, o.hdrLen, o.dst, o.len);
}

template <typename TTransport>
void mcp2515<TTransport>::runOps(mcp2515_state_t after, uint32_t nowMs) {
	_opPos = 0;
	_afterOps = after;
	this->enter(_opCount ? mcp2515_ops : after, nowMs);
}

// ---- frames ----

template <typename TTransport>
void mcp2515<TTransport>::decode(const uint8_t* b, can::Frame* f) const {
	using namespace mcp251x;
	memset(f, 0, sizeof *f);
	const uint32_t sid = ((uint32_t)b[0] << 3) | (b[1] >> 5);
	if (b[1] & kSidlIde) {
		f->extended = true;
		f->id = (sid << 18) | ((uint32_t)(b[1] & 0x03) << 16) | ((uint32_t)b[2] << 8) | b[3];
		f->rtr = (b[4] & kDlcRtr) != 0;
	} else {
		f->id = sid;
		f->rtr = (b[1] & kSidlSrr) != 0;
	}
	uint8_t n = b[4] & 0x0F;
	if (n > 8) n = 8;   // a DLC of 9..15 still means 8 bytes in classic CAN
	f->dlc = n;
	if (!f->rtr) memcpy(f->data, &b[5], n);
}

template <typename TTransport>
void mcp2515<TTransport>::dropTx() {
	_txTail.store(_txHead.load(std::memory_order_acquire), std::memory_order_release);
	_txBusy = 0;
}

template <typename TTransport>
uint8_t mcp2515<TTransport>::freeTxBuffer() const {
	if (_cfg.inOrder) return (_txBusy & 1) ? 3 : 0;
	for (uint8_t n = 0; n < 3; ++n) if (!(_txBusy & (1u << n))) return n;
	return 3;
}

// A queued frame into a free TX buffer, then RTS.
template <typename TTransport>
bool mcp2515<TTransport>::loadNext(uint32_t nowMs) {
	const uint8_t tail = _txTail.load(std::memory_order_relaxed);
	if (_txHead.load(std::memory_order_acquire) == tail) return false;
	const uint8_t n = freeTxBuffer();
	if (n > 2) return false;
	const can::Frame f = _txq[tail % kTxQueue];
	_txTail.store((uint8_t)(tail + 1), std::memory_order_release);
	opsClear();
	loadTx(n, f);
	instr(mcp251x::instrRts(n));
	_txBusy |= (uint8_t)(1u << n);
	DBG_TRACE("mcp2515", "tx", (int32_t)n, (int32_t)f.id);
	runOps(mcp2515_running, nowMs);
	return true;
}

// ---- the chip's state ----

template <typename TTransport>
void mcp2515<TTransport>::updateState(uint8_t eflg) {
	using namespace mcp251x;
	can::State s = can::State::ErrorActive;
	if (eflg & kEflgTxBo) s = can::State::BusOff;
	else if (eflg & (kEflgTxEp | kEflgRxEp)) s = can::State::ErrorPassive;
	else if (eflg & kEflgEwarn) s = can::State::ErrorWarning;
	if (s != state()) DBG_EVENT("mcp2515", "state", (int32_t)s, (int32_t)eflg);
	setState(s);
}

// CANINTF and EFLG have landed: queue the RX buffers to read and the
// flags to clear. Only the flags seen are cleared, so one raised since is
// kept for the next read.
template <typename TTransport>
void mcp2515<TTransport>::serviceFlags(uint32_t nowMs) {
	using namespace mcp251x;
	const uint8_t intf = _flags[0], eflg = _flags[1];
	opsClear();
	_rxRead = 0;
	if (intf & kIntRx0) { readRx(0); _rxRead |= 1; }
	if (intf & kIntRx1) { readRx(1); _rxRead |= 2; }
	for (uint8_t n = 0; n < 3; ++n) {
		if ((_txBusy & (1u << n)) && (intf & intTx(n))) {
			_txBusy &= (uint8_t)~(1u << n);
			++_st.txFrames;
		}
	}
	const uint8_t clear = intf & (uint8_t)(kIntTx0 | kIntTx1 | kIntTx2 | kIntErr | kIntMerr | kIntWak);
	if (clear) bitModify(kRegCanIntF, clear, 0);
	if (intf & kIntMerr) ++_st.busErrors;
	if (intf & kIntErr) _healthDue = true;   // the counters changed: read them soon
	const uint8_t ovr = eflg & (uint8_t)(kEflgRx0Ovr | kEflgRx1Ovr);
	if (ovr) {
		_st.rxOverflows += (uint32_t)((ovr & kEflgRx0Ovr) ? 1 : 0) + ((ovr & kEflgRx1Ovr) ? 1 : 0);
		DBG_FAULT("mcp2515", "rx-overflow", (int32_t)ovr);
		bitModify(kRegEflg, ovr, 0);
	}
	updateState(eflg);
	runOps(mcp2515_serviced, nowMs);
}

// Off the bus at once: configuration mode, with ABAT so that a frame that
// can't be sent doesn't hold up the mode change.
template <typename TTransport>
void mcp2515<TTransport>::goBusOff(uint32_t nowMs) {
	setState(can::State::BusOff);
	++_st.busOffs;
	DBG_FAULT("mcp2515", "bus-off", (int32_t)_tec);
	dropTx();
	opsClear();
	write1(mcp251x::kRegCanCtrl, mcp251x::kOpConfig | mcp251x::kCanCtrlAbat);
	_phaseMs = nowMs;
	runOps(mcp2515_bus_off, nowMs);
}

// start() and stop(), at a point where nothing is half done. chipOk: the
// chip can be written (not in the error state).
template <typename TTransport>
bool mcp2515<TTransport>::handleRequest(uint32_t nowMs, bool chipOk) {
	const Req r = _req.exchange(Req::None, std::memory_order_acq_rel);
	if (r == Req::Start) {
		_cfg = _reqCfg;
		mcp251x::computeTiming(_param.oscHz, _cfg.bitrate, _cfg.samplePointPermille, &_timing);   // checked by start()
		_wanted = true;
		dropTx();
		setState(can::State::Stopped);
		this->enter(mcp2515_init, nowMs);
		return true;
	}
	if (r == Req::Stop) {
		_wanted = false;
		dropTx();
		setState(can::State::Stopped);
		if (chipOk) {
			opsClear();
			write1(mcp251x::kRegCanCtrl, mcp251x::kOpConfig | mcp251x::kCanCtrlAbat);
			runOps(mcp2515_stopped, nowMs);
		} else {
			this->enter(mcp2515_stopped, nowMs);
		}
		return true;
	}
	return false;
}

template <typename TTransport>
void mcp2515<TTransport>::onFail() {
	++_st.faults;
	setState(can::State::Fault);
	dropTx();
}

// ---- main ----

template <typename TTransport>
void mcp2515<TTransport>::main(uint32_t nowMs) {
	using namespace mcp251x;
	switch (this->_state) {
	case mcp2515_stopped:
		if (!handleRequest(nowMs, true)) this->sleep(kIdleSleepMs);
		break;

	case mcp2515_init:
		setState(can::State::Stopped);
		_txBusy = 0;
		_tec = _rec = 0;
		_phaseMs = nowMs;
		opsClear();
		instr(kInstrReset);
		runOps(mcp2515_reset_settle, nowMs);
		break;

	case mcp2515_ops:
		this->issued(issueOp(_ops[_opPos]), mcp2515_wait_ops, nowMs);
		break;

	case mcp2515_wait_ops:
		if (this->landed(nowMs)) {
			if (++_opPos < _opCount) this->enter(mcp2515_ops, nowMs);
			else this->enter(_afterOps, nowMs);
		}
		break;

	case mcp2515_reset_settle:
		// A tick more: 1 ms ticks can make a wait up to 1 ms short.
		if (this->elapsed(nowMs, kResetMs + 1)) {
			opsClear();
			read(kRegCanStat, _stat, 2);   // CANSTAT, CANCTRL
			runOps(mcp2515_check_reset, nowMs);
		} else {
			this->sleepRemaining(nowMs, kResetMs + 1);
		}
		break;

	case mcp2515_check_reset:
		if ((_stat[1] & kCanCtrlResetMask) != kCanCtrlResetValue) {
			DBG_FAULT("mcp2515", "absent", (int32_t)_stat[0], (int32_t)_stat[1]);
			this->fail(nowMs);
		} else if ((_stat[0] & kOpModeMask) != kOpConfig) {
			if (nowMs - _phaseMs > kResetTimeoutMs) {
				DBG_FAULT("mcp2515", "no-config", (int32_t)_stat[0]);
				this->fail(nowMs);
			} else {
				this->sleep(1);
				opsClear();
				read(kRegCanStat, _stat, 2);
				runOps(mcp2515_check_reset, nowMs);
			}
		} else {
			queueConfig();
			runOps(mcp2515_check_config, nowMs);
		}
		break;

	case mcp2515_check_config: {
		const bool cnfOk = _cnfBack[0] == _timing.cnf3() && _cnfBack[1] == _timing.cnf2() &&
		                   _cnfBack[2] == _timing.cnf1() && _cnfBack[3] == kInte;
		const bool rxOk = (_rxCtrlBack[0] & (kRxCtrlRxmAny | kRxCtrlBukt)) == (kRxCtrlRxmAny | kRxCtrlBukt) &&
		                  (_rxCtrlBack[1] & kRxCtrlRxmAny) == kRxCtrlRxmAny;
		if (!cnfOk || !rxOk) {
			DBG_FAULT("mcp2515", "config-readback", (int32_t)_cnfBack[2], (int32_t)_cnfBack[1], (int32_t)_cnfBack[0]);
			this->fail(nowMs);
			break;
		}
		_expectMode = _cfg.mode == can::Mode::Loopback ? kOpLoopback
		            : _cfg.mode == can::Mode::ListenOnly ? kOpListenOnly : kOpNormal;
		opsClear();
		write1(kRegCanCtrl, _expectMode);   // CLKOUT off, one-shot off, ABAT off
		read(kRegCanStat, _stat, 1);
		_phaseMs = nowMs;
		runOps(mcp2515_mode_poll, nowMs);
		break;
	}

	case mcp2515_mode_poll:
		if ((_stat[0] & kOpModeMask) == _expectMode) {
			setState(can::State::ErrorActive);
			_lastPollMs = _lastHealthMs = nowMs;
			_healthDue = false;
			_intPending.store(true, std::memory_order_release);   // read the flags once at the start
			DBG_EVENT("mcp2515", "on", (int32_t)(_cfg.bitrate / 1000), (int32_t)_expectMode, (int32_t)_timing.samplePoint());
			this->enter(mcp2515_running, nowMs);
		} else if (nowMs - _phaseMs > kModeTimeoutMs) {
			DBG_FAULT("mcp2515", "mode-timeout", (int32_t)_stat[0]);
			this->fail(nowMs);
		} else {
			this->sleep(1);
			opsClear();
			read(kRegCanStat, _stat, 1);
			runOps(mcp2515_mode_poll, nowMs);
		}
		break;

	case mcp2515_running: {
		if (handleRequest(nowMs, true)) break;
		const bool irq = _intPending.load(std::memory_order_acquire);
		if (irq || nowMs - _lastPollMs >= _param.pollMs) {
			_intPending.store(false, std::memory_order_release);   // before the read: an INT during it is kept
			_serviceTicks = irq ? _intTicks : nowTicks(nowMs);
			_lastPollMs = nowMs;
			opsClear();
			read(kRegCanIntF, _flags, 2);   // CANINTF, EFLG
			runOps(mcp2515_check_flags, nowMs);
			break;
		}
		if (_healthDue || nowMs - _lastHealthMs >= kHealthMs) {
			_healthDue = false;
			_lastHealthMs = nowMs;
			opsClear();
			read(kRegCanStat, _health, sizeof _health);   // CANSTAT 0x0E .. REC 0x1D
			runOps(mcp2515_check_health, nowMs);
			break;
		}
		if (loadNext(nowMs)) break;
		uint32_t wait = _param.pollMs - (nowMs - _lastPollMs);
		const uint32_t toHealth = kHealthMs - (nowMs - _lastHealthMs);
		if (toHealth < wait) wait = toHealth;
		if (wait > kIdleSleepMs) wait = kIdleSleepMs;
		if (!workPending()) this->sleep(wait ? wait : 1);
		break;
	}

	case mcp2515_check_flags:
		serviceFlags(nowMs);
		break;

	case mcp2515_serviced: {
		can::Frame f;
		for (uint8_t n = 0; n < 2; ++n) {
			if (!(_rxRead & (1u << n))) continue;
			decode(_rxb[n], &f);
			f.ticks = _serviceTicks;
			const uint8_t head = _rxHead.load(std::memory_order_relaxed);
			if ((uint8_t)(head - _rxTail.load(std::memory_order_acquire)) >= kRxQueue) {
				++_st.rxDropped;
				DBG_FAULT("mcp2515", "rx-dropped", (int32_t)f.id);
				continue;
			}
			_rxq[head % kRxQueue] = f;
			_rxHead.store((uint8_t)(head + 1), std::memory_order_release);
			++_st.rxFrames;
		}
		if (_flags[1] & kEflgTxBo) {
			goBusOff(nowMs);
		} else if (_flags[0] & (uint8_t)~kIntWak) {
			// Something was pending: read again until nothing is, so INT goes
			// high and an edge-triggered interrupt sees the next frame.
			_intPending.store(true, std::memory_order_release);
			this->enter(mcp2515_running, nowMs);
		} else {
			this->enter(mcp2515_running, nowMs);
		}
		break;
	}

	case mcp2515_check_health: {
		// A chip that reset itself or lost power is back in configuration mode.
		if ((_health[0] & kOpModeMask) != _expectMode || (_health[1] & kOpModeMask) != _expectMode) {
			DBG_FAULT("mcp2515", "mode-lost", (int32_t)_health[0], (int32_t)_health[1]);
			this->fail(nowMs);
			break;
		}
		_tec = _health[14];
		_rec = _health[15];
		this->enter(mcp2515_running, nowMs);
		break;
	}

	case mcp2515_bus_off:
		if (handleRequest(nowMs, true)) break;
		if (_cfg.autoRestart) {
			if (nowMs - _phaseMs >= _cfg.busOffRestartMs) {
				++_st.restarts;
				this->enter(mcp2515_init, nowMs);
			} else {
				const uint32_t left = _cfg.busOffRestartMs - (nowMs - _phaseMs);
				this->sleep(left < kIdleSleepMs ? left : kIdleSleepMs);
			}
		} else {
			this->sleep(kIdleSleepMs);
		}
		break;

	case mcp2515_error:
		if (handleRequest(nowMs, false)) break;
		if (this->errorCleared(nowMs, kErrorBackoffMs)) {
			if (_wanted) {
				++_st.restarts;
				this->enter(mcp2515_init, nowMs);
			} else {
				setState(can::State::Stopped);
				this->enter(mcp2515_stopped, nowMs);
			}
		}
		break;

	default:
		this->enter(_wanted ? mcp2515_init : mcp2515_stopped, nowMs);
		break;
	}
}

// Startup, after the reset checked out: the bit timing and interrupts,
// the flags cleared, both RX buffers open, the buffer pins off, all read
// back.
template <typename TTransport>
void mcp2515<TTransport>::queueConfig() {
	using namespace mcp251x;
	const uint8_t cnf[4] = {_timing.cnf3(), _timing.cnf2(), _timing.cnf1(), kInte};
	const uint8_t zero[2] = {0, 0};
	write(kRegCnf3, cnf, 4);          // CNF3, CNF2, CNF1, CANINTE
	write(kRegCanIntF, zero, 2);      // CANINTF, EFLG
	write1(kRegRxb0Ctrl, kRxCtrlRxmAny | kRxCtrlBukt);
	write1(kRegRxb1Ctrl, kRxCtrlRxmAny);
	write(kRegBfpCtrl, zero, 2);      // BFPCTRL, TXRTSCTRL
	read(kRegCnf3, _cnfBack, 4);
	read(kRegRxb0Ctrl, &_rxCtrlBack[0], 1);
	read(kRegRxb1Ctrl, &_rxCtrlBack[1], 1);
}
