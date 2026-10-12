/*
 * teseo.tpp
 *
 *  teseo's member definitions. teseo.h includes this file at its end,
 *  since templates have to be visible wherever they are used: include
 *  teseo.h, not this file.
 */

#ifndef TESEO_TPP_
#define TESEO_TPP_

#include "../inc/teseo.h"

template <typename TTransport>
template <typename... TArgs>
teseo<TTransport>::teseo(const teseo_param_t& param, TArgs&&... transportArgs)
	: base(teseo_init_state, teseo_error_state, 0, std::forward<TArgs>(transportArgs)...),
	  _param(param), _wakePending(false), _sinceWake(0), _txLen(0), _step(teseo_step_t::Version),
	  _tries(0), _ack(Ack::Waiting), _match(false), _lowOk(false), _highOk(false), _savedThisStart(false),
	  _configStatus(teseo_config_status_t::NotDone), _cfgResends(0), _saves(0),
	  _lastGoodMs(0), _haveData(false), _newData(false) {
	memset(_tx, 0, sizeof(_tx));
	memset(_version, 0, sizeof(_version));
	this->setDebugTag("teseo");
}

// ---- interrupt side ----

template <typename TTransport>
void teseo<TTransport>::onByteReceived(uint8_t byte) {
	if (!_rx.push(byte)) return;   // full: dropped and counted
	if (byte == '\n' || ++_sinceWake >= 64) {
		_sinceWake = 0;
		_wakePending.store(true, std::memory_order_release);
		wake();
	}
}

// ---- main() side ----

template <typename TTransport>
void teseo<TTransport>::drain(uint32_t nowMs) {
	_wakePending.store(false, std::memory_order_release);
	uint8_t b;
	while (_rx.pop(&b)) {
		const NmeaParser::Sentence s = _nmea.feed(b);
		if (s == NmeaParser::Sentence::Bad) {
			DBG_TRACE("teseo", "nmea-bad", (int32_t)_nmea.stats().checksumErrors, (int32_t)_nmea.stats().malformed);
			continue;
		}
		if (s == NmeaParser::Sentence::None) continue;
		DBG_TRACE("teseo", "nmea", (int32_t)s);
		_lastGoodMs = nowMs;   // any good sentence shows the module is there
		if (s == NmeaParser::Sentence::GGA || s == NmeaParser::Sentence::RMC) {
			_haveData = true;
			_newData = true;
		} else if (s == NmeaParser::Sentence::Other) {
			proprietary();
		}
	}
#if ITRANSPORT_DEBUG
	const uint32_t lost = _rx.overflows();
	if (lost != _dbgOverflows) { DBG_FAULT("teseo", "rx-overflow", (int32_t)(lost - _dbgOverflows)); _dbgOverflows = lost; }
#endif
}

static inline bool teseoIs(const char* s, size_t n, const char* want) {
	const size_t w = strlen(want);
	return n == w && memcmp(s, want, w) == 0;
}

template <typename TTransport>
uint16_t teseo<TTransport>::waitCdb() const {
	return (_step == teseo_step_t::ReadLow || _step == teseo_step_t::VerifyLow) ? pstm::kCdbMsgListLow
	                                                                            : pstm::kCdbMsgListHigh;
}

// The answers this driver waits for. Anything else proprietary is ignored.
template <typename TTransport>
void teseo<TTransport>::proprietary() {
	if (_state != teseo_cfg_wait_state || _ack != Ack::Waiting) return;
	const char *a, *f1, *f2;
	size_t na, n1, n2;
	_nmea.address(&a, &na);
	_nmea.field(1, &f1, &n1);
	_nmea.field(2, &f2, &n2);

	switch (_step) {
	case teseo_step_t::Version:
		if (teseoIs(a, na, "PSTMVER")) {
			const size_t n = n1 < kVersionLen - 1 ? n1 : kVersionLen - 1;
			memcpy(_version, f1, n);
			_version[n] = 0;
			_ack = Ack::Ok;
		}
		break;
	case teseo_step_t::ReadLow:
	case teseo_step_t::ReadHigh:
	case teseo_step_t::VerifyLow:
	case teseo_step_t::VerifyHigh: {
		// "$PSTMSETPAR,<cdb>,<value>": the answer to $PSTMGETPAR,<cdb>
		uint32_t cdb = 0;
		if (teseoIs(a, na, "PSTMSETPAR") && NmeaParser::parseUint(f1, n1, &cdb)
		    && (cdb == waitCdb() || cdb == waitCdb() % 1000u)) {
			const uint32_t want = waitCdb() == pstm::kCdbMsgListLow ? _param.messagesLow : _param.messagesHigh;
			_match = pstm::valueIs(f2, n2, want);
			_ack = Ack::Ok;
		}
		break;
	}
	case teseo_step_t::WriteList:
		if (teseoIs(a, na, "PSTMCFGMSGLOK")) _ack = Ack::Ok;
		else if (teseoIs(a, na, "PSTMCFGMSGLERROR")) _ack = Ack::Error;
		break;
	case teseo_step_t::Save:
		if (teseoIs(a, na, "PSTMSAVEPAROK")) _ack = Ack::Ok;
		else if (teseoIs(a, na, "PSTMSAVEPARERROR")) _ack = Ack::Error;
		break;
	default:
		break;
	}
}

template <typename TTransport>
bool teseo<TTransport>::buildCommand() {
	const bool ck = _param.checksum;
	switch (_step) {
	case teseo_step_t::Version:
		_txLen = pstm::getSwVersion(_tx, sizeof(_tx), ck);
		return true;
	case teseo_step_t::ReadLow:
	case teseo_step_t::ReadHigh:
	case teseo_step_t::VerifyLow:
	case teseo_step_t::VerifyHigh:
		_txLen = pstm::getPar(_tx, sizeof(_tx), waitCdb(), ck);
		return true;
	case teseo_step_t::WriteList:
		_txLen = pstm::cfgMsgList(_tx, sizeof(_tx), 0, 1, _param.messagesLow, _param.messagesHigh, ck);
		return true;
	case teseo_step_t::Save:
		_txLen = pstm::savePar(_tx, sizeof(_tx), ck);
		return true;
	case teseo_step_t::Reset:
		_txLen = pstm::systemReset(_tx, sizeof(_tx), ck);
		return true;
	default:
		return false;
	}
}

template <typename TTransport>
void teseo<TTransport>::startConfig(uint32_t nowMs) {
	_step = teseo_step_t::Version;
	_tries = 0;
	_lowOk = _highOk = false;
	_configStatus = teseo_config_status_t::NotDone;
	enter(teseo_cfg_send_state, nowMs);
}

template <typename TTransport>
void teseo<TTransport>::finish(teseo_config_status_t s, uint32_t nowMs) {
	DBG_EVENT("teseo", "cfg-done", (int32_t)s, (int32_t)_step);
	_configStatus = s;
	if (s == teseo_config_status_t::Done) _step = teseo_step_t::Finished;
	enter(teseo_listening_state, nowMs);
}

// The step just answered: what follows it.
template <typename TTransport>
void teseo<TTransport>::advance(uint32_t nowMs) {
	_tries = 0;
	switch (_step) {
	case teseo_step_t::Version:
		_step = teseo_step_t::ReadLow;
		break;
	case teseo_step_t::ReadLow:
	case teseo_step_t::VerifyLow:
		DBG_EVENT("teseo", _match ? "cfg-match" : "cfg-differ", (int32_t)waitCdb());
		_lowOk = _match;
		_step = _step == teseo_step_t::ReadLow ? teseo_step_t::ReadHigh : teseo_step_t::VerifyHigh;
		break;
	case teseo_step_t::ReadHigh:
	case teseo_step_t::VerifyHigh: {
		DBG_EVENT("teseo", _match ? "cfg-match" : "cfg-differ", (int32_t)waitCdb());
		_highOk = _match;
		if (_lowOk && _highOk) { finish(teseo_config_status_t::Done, nowMs); return; }
		if (_step == teseo_step_t::VerifyHigh || _savedThisStart) {
			finish(teseo_config_status_t::Unverified, nowMs);   // never a second save this start
			return;
		}
		_step = teseo_step_t::WriteList;
		break;
	}
	case teseo_step_t::WriteList:
		_step = teseo_step_t::Save;
		break;
	case teseo_step_t::Save:
		_savedThisStart = true;
		++_saves;
		_step = teseo_step_t::Reset;
		break;
	default:
		break;
	}
	enter(teseo_cfg_send_state, nowMs);
}

template <typename TTransport>
void teseo<TTransport>::main(uint32_t nowMs) {
	drain(nowMs);

	switch (_state) {
	case teseo_init_state:
		// From here on the transport calls onByteReceived(); this also
		// starts its receiver if it is not already running.
		this->setRxSink(*this);
		_lastGoodMs = nowMs;
		_savedThisStart = false;
		if (_param.configure) {
			startConfig(nowMs);
		} else {
			enter(teseo_listening_state, nowMs);
		}
		break;

	case teseo_cfg_send_state:
		// Built again on every pass until write() takes it: the same bytes.
		if (!buildCommand()) { finish(teseo_config_status_t::Done, nowMs); break; }
		_ack = Ack::Waiting;
		_match = false;
		if (this->write((const uint8_t*)_tx, _txLen)) {
			DBG_EVENT("teseo", "cfg-send", (int32_t)_step, (int32_t)_tries);
			if (_step == teseo_step_t::Reset) {
				DBG_EVENT("teseo", "cfg-reset");
				enter(teseo_boot_wait_state, nowMs);   // no answer: it reboots
			} else {
				enter(teseo_cfg_wait_state, nowMs);
			}
		} else if (elapsed(nowMs, teseo_write_timeout_ms)) {
			DBG_FAULT("teseo", "cfg-uart-busy", (int32_t)_step);
			finish(teseo_config_status_t::NoAnswer, nowMs);
		} else {
			sleep(1);
		}
		break;

	case teseo_cfg_wait_state:
		if (_ack == Ack::Ok) {
			DBG_EVENT("teseo", "cfg-ok", (int32_t)_step);
			advance(nowMs);
		} else if (_ack == Ack::Error) {
			DBG_FAULT("teseo", "cfg-error", (int32_t)_step);
			finish(teseo_config_status_t::Rejected, nowMs);
		} else if (elapsed(nowMs, teseo_ack_timeout_ms)) {
			if (++_tries < teseo_cfg_tries) {
				++_cfgResends;
				enter(teseo_cfg_send_state, nowMs);   // the same command again
			} else {
				DBG_FAULT("teseo", "cfg-noanswer", (int32_t)_step);
				finish(teseo_config_status_t::NoAnswer, nowMs);
			}
		} else {
			// Every answer ends in a newline, which wakes an OS thread.
			const uint32_t left = teseo_ack_timeout_ms - (nowMs - last_update);
			sleep(left < teseo_max_sleep_ms ? left : teseo_max_sleep_ms);
		}
		break;

	case teseo_boot_wait_state:
		// Back once a good sentence has arrived teseo_boot_min_ms after the
		// reset (what came before may still be the old firmware's).
		if (elapsed(nowMs, teseo_boot_min_ms) && (int32_t)(_lastGoodMs - last_update) >= (int32_t)teseo_boot_min_ms) {
			_step = teseo_step_t::VerifyLow;
			_tries = 0;
			enter(teseo_cfg_send_state, nowMs);
		} else if (elapsed(nowMs, teseo_boot_max_ms)) {
			DBG_FAULT("teseo", "cfg-noanswer", (int32_t)_step);
			finish(teseo_config_status_t::NoAnswer, nowMs);
		} else {
			sleep(teseo_max_sleep_ms);
		}
		break;

	case teseo_listening_state:
		if (nowMs - _lastGoodMs >= _param.silenceMs) {
			DBG_FAULT("teseo", "silent", (int32_t)(nowMs - _lastGoodMs));
			fail(nowMs);
		} else if (_param.configure && _configStatus == teseo_config_status_t::NoAnswer
		           && elapsed(nowMs, teseo_noanswer_retry_ms)) {
			startConfig(nowMs);
		} else {
			const uint32_t left = _param.silenceMs - (nowMs - _lastGoodMs);
			sleep(left < teseo_max_sleep_ms ? left : teseo_max_sleep_ms);
		}
		break;

	case teseo_error_state:
		if (elapsed(nowMs, teseo_retry_ms)) {
			enter(teseo_init_state, nowMs);   // attach again and configure again
		} else {
			sleepRemaining(nowMs, teseo_retry_ms);
		}
		break;

	default:
		enter(teseo_init_state, nowMs);
		break;
	}
}

template <typename TTransport>
void teseo<TTransport>::onFail() {
	_nmea.invalidate();
	_haveData = false;
	_newData = false;
}

template <typename TTransport>
bool teseo<TTransport>::getData(GnssData* out) {
	*out = _nmea.data();
	_newData = false;
	return _haveData;
}

template <typename TTransport>
bool teseo<TTransport>::position(int32_t* latE7, int32_t* lonE7) const {
	const GnssData& d = _nmea.data();
	if (!_haveData || !d.positionValid) return false;
	*latE7 = d.latitudeE7;
	*lonE7 = d.longitudeE7;
	return true;
}

template <typename TTransport>
typename teseo<TTransport>::Stats teseo<TTransport>::stats() const {
	Stats s;
	s.rxOverflows = _rx.overflows();
	s.cfgResends = _cfgResends;
	s.saves = _saves;
	return s;
}

#endif /* TESEO_TPP_ */
