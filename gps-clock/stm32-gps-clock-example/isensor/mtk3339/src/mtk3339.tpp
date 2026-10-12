/*
 * mtk3339.tpp
 *
 *  mtk3339's member definitions. mtk3339.h includes this file at its
 *  end, since templates have to be visible wherever they are used:
 *  include mtk3339.h, not this file.
 */

#ifndef MTK3339_TPP_
#define MTK3339_TPP_

#include "../inc/mtk3339.h"

template <typename TTransport>
template <typename... TArgs>
mtk3339<TTransport>::mtk3339(const mtk3339_param_t& param, TArgs&&... transportArgs)
	: base(mtk3339_init_state, mtk3339_error_state, 0, std::forward<TArgs>(transportArgs)...),
	  _param(param), _wakePending(false), _sinceWake(0), _txLen(0), _cfgStep(0), _cfgTries(0),
	  _waitCmd(0), _ack(Ack::Waiting), _rejectFlag(pmtk::AckFlag::Succeeded),
	  _configStatus(mtk3339_config_status_t::NotDone), _cfgResends(0), _restarts(0),
	  _restartSeen(false), _antenna(mtk3339_antenna_t::Unknown),
	  _lastGoodMs(0), _haveData(false), _newData(false) {
	memset(_tx, 0, sizeof(_tx));
	pmtk::antennaStatus(_txAntenna, sizeof(_txAntenna), true);
	if (_param.updateMs < pmtk::kMinUpdateMs) _param.updateMs = pmtk::kMinUpdateMs;
	if (_param.updateMs > pmtk::kMaxUpdateMs) _param.updateMs = pmtk::kMaxUpdateMs;
	this->setDebugTag("mtk3339");
}

// ---- interrupt side ----

template <typename TTransport>
void mtk3339<TTransport>::onByteReceived(uint8_t byte) {
	if (!_rx.push(byte)) return;   // full: dropped and counted
	if (byte == '\n' || ++_sinceWake >= 64) {
		_sinceWake = 0;
		_wakePending.store(true, std::memory_order_release);
		wake();
	}
}

// ---- main() side ----

template <typename TTransport>
void mtk3339<TTransport>::drain(uint32_t nowMs) {
	_wakePending.store(false, std::memory_order_release);
	uint8_t b;
	while (_rx.pop(&b)) {
		const NmeaParser::Sentence s = _nmea.feed(b);
		if (s == NmeaParser::Sentence::Bad) {
			DBG_TRACE("mtk3339", "nmea-bad", (int32_t)_nmea.stats().checksumErrors, (int32_t)_nmea.stats().malformed);
			continue;
		}
		if (s == NmeaParser::Sentence::None) continue;
		DBG_TRACE("mtk3339", "nmea", (int32_t)s);
		_lastGoodMs = nowMs;   // any good sentence shows the module is there
		if (s == NmeaParser::Sentence::GGA || s == NmeaParser::Sentence::RMC) {
			_haveData = true;
			_newData = true;
		} else if (s == NmeaParser::Sentence::Other) {
			proprietary(nowMs);
		}
	}
#if ITRANSPORT_DEBUG
	const uint32_t lost = _rx.overflows();
	if (lost != _dbgOverflows) { DBG_FAULT("mtk3339", "rx-overflow", (int32_t)(lost - _dbgOverflows)); _dbgOverflows = lost; }
#endif
}

static inline bool mtk3339Is(const char* s, size_t n, const char* want) {
	const size_t w = strlen(want);
	return n == w && memcmp(s, want, w) == 0;
}

template <typename TTransport>
void mtk3339<TTransport>::proprietary(uint32_t nowMs) {
	(void)nowMs;
	const char *a, *f1, *f2;
	size_t na, n1, n2;
	uint32_t v1 = 0, v2 = 0;
	_nmea.address(&a, &na);
	_nmea.field(1, &f1, &n1);
	_nmea.field(2, &f2, &n2);
	const bool num1 = NmeaParser::parseUint(f1, n1, &v1);
	const bool num2 = NmeaParser::parseUint(f2, n2, &v2);

	if (mtk3339Is(a, na, "PMTK001")) {
		// $PMTK001,cmd,flag: the answer to a command.
		if (num1 && num2 && _state == mtk3339_cfg_wait_state && v1 == _waitCmd) {
			if (v2 == (uint32_t)pmtk::AckFlag::Succeeded) {
				DBG_EVENT("mtk3339", "cfg-ack", (int32_t)v1);
				_ack = Ack::Acked;
			} else {
				DBG_FAULT("mtk3339", "cfg-reject", (int32_t)v1, (int32_t)v2);
				_rejectFlag = (pmtk::AckFlag)(v2 > 3 ? 0 : v2);
				_ack = Ack::Refused;
			}
		}
	} else if (mtk3339Is(a, na, "PMTK010")) {
		// $PMTK010,001: the module has just started.
		if (num1 && v1 == pmtk::kSystemStartup) {
			DBG_EVENT("mtk3339", "module-start");
			_restartSeen = true;
		}
	} else if (mtk3339Is(a, na, "PGTOP") || mtk3339Is(a, na, "PCD")) {
		// $PGTOP,11,x: 1 shorted, 2 internal, 3 external (GlobalTop).
		// $PCD,11,x:   1 internal, 2 external, 3 shorted (CDTop).
		if (num1 && v1 == 11 && num2 && n2 == 1) {
			const bool cd = na == 3;
			static const mtk3339_antenna_t gt[] = {mtk3339_antenna_t::Unknown, mtk3339_antenna_t::Shorted,
			                                       mtk3339_antenna_t::Internal, mtk3339_antenna_t::External};
			static const mtk3339_antenna_t ct[] = {mtk3339_antenna_t::Unknown, mtk3339_antenna_t::Internal,
			                                       mtk3339_antenna_t::External, mtk3339_antenna_t::Shorted};
			if (v2 <= 3) {
				const mtk3339_antenna_t a2 = cd ? ct[v2] : gt[v2];
				if (a2 != _antenna) DBG_EVENT("mtk3339", "antenna", (int32_t)a2);
				_antenna = a2;
			}
		}
	}
}

// The configuration, one step at a time: [antenna report,] PMTK314,
// PMTK220, PMTK300.
template <typename TTransport>
bool mtk3339<TTransport>::nextCommand() {
	switch (_cfgStep) {
	case 0: {
		uint8_t rates[pmtk::kOutputFields] = {0};
		for (uint8_t i = 0; i < pmtk::kOutputFields; ++i) {
			if (_param.sentences & (1u << i)) rates[i] = 1;
		}
		_txLen = pmtk::setNmeaOutput(_tx, sizeof(_tx), rates);
		_waitCmd = pmtk::kCmdNmeaOutput;
		return true;
	}
	case 1:
		_txLen = pmtk::setUpdateRate(_tx, sizeof(_tx), _param.updateMs);
		_waitCmd = pmtk::kCmdUpdateRate;
		return true;
	case 2:
		_txLen = pmtk::setFixInterval(_tx, sizeof(_tx),
		                              _param.updateMs < pmtk::kMinFixMs ? pmtk::kMinFixMs : _param.updateMs);
		_waitCmd = pmtk::kCmdFixCtl;
		return true;
	default:
		return false;
	}
}

template <typename TTransport>
void mtk3339<TTransport>::startConfig(uint32_t nowMs) {
	_cfgStep = 0;
	_cfgTries = 0;
	_configStatus = mtk3339_config_status_t::NotDone;
	_rejectFlag = pmtk::AckFlag::Succeeded;
	if (_param.antennaStatus) {
		// Nothing answers it; if the UART is busy it is simply skipped
		// (the module repeats the report once asked, so it can be asked
		// again at the next start).
		this->write((const uint8_t*)_txAntenna, strlen(_txAntenna));
	}
	enter(mtk3339_cfg_send_state, nowMs);
}

template <typename TTransport>
void mtk3339<TTransport>::main(uint32_t nowMs) {
	drain(nowMs);

	if (_restartSeen) {
		_restartSeen = false;
		++_restarts;
		if (_param.configure && _state != mtk3339_init_state && _state != mtk3339_error_state) {
			startConfig(nowMs);   // its settings may be gone: set them again
		}
	}

	switch (_state) {
	case mtk3339_init_state:
		// From here on the transport calls onByteReceived(). This also
		// starts its receiver if it is not already running.
		this->setRxSink(*this);
		_lastGoodMs = nowMs;
		if (_param.configure) {
			startConfig(nowMs);
		} else {
			enter(mtk3339_listening_state, nowMs);
		}
		break;

	case mtk3339_cfg_send_state:
		// Built again on every pass until write() takes it; the bytes
		// are the same each time, since the step has not moved on.
		if (!nextCommand()) {
			DBG_EVENT("mtk3339", "cfg-done");
			_configStatus = mtk3339_config_status_t::Done;
			enter(mtk3339_listening_state, nowMs);
			break;
		}
		_ack = Ack::Waiting;
		if (this->write((const uint8_t*)_tx, _txLen)) {
			DBG_EVENT("mtk3339", "cfg-send", _waitCmd, _cfgStep, _cfgTries);
			enter(mtk3339_cfg_wait_state, nowMs);
		} else if (elapsed(nowMs, mtk3339_write_timeout_ms)) {
			DBG_FAULT("mtk3339", "cfg-uart-busy", _waitCmd);
			_configStatus = mtk3339_config_status_t::NoAnswer;
			enter(mtk3339_listening_state, nowMs);
		} else {
			sleep(1);
		}
		break;

	case mtk3339_cfg_wait_state:
		if (_ack == Ack::Acked) {
			++_cfgStep;
			_cfgTries = 0;
			enter(mtk3339_cfg_send_state, nowMs);
		} else if (_ack == Ack::Refused) {
			_configStatus = mtk3339_config_status_t::Rejected;
			enter(mtk3339_listening_state, nowMs);
		} else if (elapsed(nowMs, mtk3339_ack_timeout_ms)) {
			if (++_cfgTries < mtk3339_cfg_tries) {
				++_cfgResends;
				enter(mtk3339_cfg_send_state, nowMs);   // the same command again
			} else {
				DBG_FAULT("mtk3339", "cfg-noanswer", _waitCmd);
				_configStatus = mtk3339_config_status_t::NoAnswer;
				enter(mtk3339_listening_state, nowMs);
			}
		} else {
			// PMTK001 ends in a newline, which wakes an OS thread.
			const uint32_t left = mtk3339_ack_timeout_ms - (nowMs - last_update);
			sleep(left < mtk3339_max_sleep_ms ? left : mtk3339_max_sleep_ms);
		}
		break;

	case mtk3339_listening_state:
		if (nowMs - _lastGoodMs >= _param.silenceMs) {
			DBG_FAULT("mtk3339", "silent", (int32_t)(nowMs - _lastGoodMs));
			fail(nowMs);
		} else {
			const uint32_t left = _param.silenceMs - (nowMs - _lastGoodMs);
			sleep(left < mtk3339_max_sleep_ms ? left : mtk3339_max_sleep_ms);
		}
		break;

	case mtk3339_error_state:
		if (elapsed(nowMs, mtk3339_retry_ms)) {
			enter(mtk3339_init_state, nowMs);   // attach again and configure again
		} else {
			sleepRemaining(nowMs, mtk3339_retry_ms);
		}
		break;

	default:
		enter(mtk3339_init_state, nowMs);
		break;
	}
}

template <typename TTransport>
void mtk3339<TTransport>::onFail() {
	_nmea.invalidate();
	_haveData = false;
	_newData = false;
	_antenna = mtk3339_antenna_t::Unknown;
}

template <typename TTransport>
bool mtk3339<TTransport>::getData(GnssData* out) {
	*out = _nmea.data();
	_newData = false;
	return _haveData;
}

template <typename TTransport>
bool mtk3339<TTransport>::position(int32_t* latE7, int32_t* lonE7) const {
	const GnssData& d = _nmea.data();
	if (!_haveData || !d.positionValid) return false;
	*latE7 = d.latitudeE7;
	*lonE7 = d.longitudeE7;
	return true;
}

template <typename TTransport>
typename mtk3339<TTransport>::Stats mtk3339<TTransport>::stats() const {
	Stats s;
	s.rxOverflows = _rx.overflows();
	s.cfgResends = _cfgResends;
	s.restarts = _restarts;
	return s;
}

#endif /* MTK3339_TPP_ */
