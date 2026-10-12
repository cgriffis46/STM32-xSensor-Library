/*
 * rtc_probe.h
 *
 *  A small non-blocking reader for an I2C real-time clock's time registers,
 *  for measuring where its seconds tick against its square wave (see
 *  rtc_edge.h), not a driver: it writes one control register at start (to
 *  turn the 1 Hz square wave on), then reads the seven time registers
 *  (0x00..0x06, BCD, as the DS1307 and DS3231 lay them out) once a second,
 *  or every probePeriodMs while a probe runs. Each read's I2C START is
 *  bracketed by the counter given in the parameters, read just before and
 *  just after the call that issues it.
 *
 *  Written like the isensor drivers (SensorStateMachine; the transport is
 *  inherited). A DS3231 driver proper is to follow once the edge has been
 *  measured on the board.
 *
 *  Control register values (general knowledge of the datasheets, which
 *  couldn't be fetched here; the probe shows whether SQW appears):
 *    DS3231: 0x0E <- 0x00  oscillator on, INTCN 0, RS 00: SQW/INT is 1 Hz
 *    DS1307: 0x07 <- 0x10  SQWE, RS 00: SQW/OUT is 1 Hz
 */

#ifndef RTC_PROBE_H_
#define RTC_PROBE_H_

#include <stdint.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"
#include "cmsis_os.h"

typedef enum rtc_probe_state_t {
	rtc_probe_init_state,
	rtc_probe_control_state,
	rtc_probe_wait_control_state,
	rtc_probe_idle_state,
	rtc_probe_read_state,
	rtc_probe_wait_read_state,
	rtc_probe_err_state
} rtc_probe_state_t;

struct rtc_probe_param_t {
	uint8_t controlReg = 0x0E;               // DS3231
	uint8_t controlValue = 0x00;
	uint32_t periodMs = 1000;                // between reads normally
	uint32_t probePeriodMs = 20;             // and while probing
	const volatile uint32_t *counter = nullptr;   // the 1 MHz timer (TIM2->CNT)
};

// One read: the seven time registers, and when its START could have been.
struct rtc_probe_read_t {
	uint32_t from, to;     // counter before and after issuing it
	uint8_t regs[7];
};

const uint32_t rtc_probe_bus_timeout_ms = 50;
const uint32_t rtc_probe_backoff_ms = 1000;

template <typename TTransport>
class rtc_probe : public SensorStateMachine<TTransport, rtc_probe_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
	              "rtc_probe<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, rtc_probe_state_t> base;
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
	explicit rtc_probe(const rtc_probe_param_t &param, TArgs &&...transportArgs)
		: base(rtc_probe_init_state, rtc_probe_err_state, rtc_probe_bus_timeout_ms,
		       std::forward<TArgs>(transportArgs)...),
		  _param(param) {
		this->setDebugTag("rtcprobe");
	}

	void main(uint32_t nowMs) {
		switch (_state) {
		case rtc_probe_init_state:
			if (this->checkDevice()) enter(rtc_probe_control_state, nowMs);
			else fail(nowMs);
			break;
		case rtc_probe_control_state:
			issued(this->writeReg(_param.controlReg, _param.controlValue), rtc_probe_wait_control_state, nowMs);
			break;
		case rtc_probe_wait_control_state:
			if (landed(nowMs)) {
				_present = true;
				enter(rtc_probe_read_state, nowMs);
			}
			break;
		case rtc_probe_idle_state: {
			if (_probing && (int32_t)(nowMs - _probeEnd) >= 0) _probing = false;
			const uint32_t period = _probing ? _param.probePeriodMs : _param.periodMs;
			const uint32_t since = nowMs - _lastRead;
			if (since >= period) {
				enter(rtc_probe_read_state, nowMs);
			} else {
				const uint32_t left = period - since;
				sleep(left < (_probing ? 5u : 50u) ? left : (_probing ? 5u : 50u));   // a probe request is seen soon
			}
			break;
		}
		case rtc_probe_read_state: {
			_lastRead = nowMs;
			const uint32_t before = _param.counter ? *_param.counter : 0;
			const bool started = this->readRegs(0x00, _buf, 7);
			const uint32_t after = _param.counter ? *_param.counter : 0;
			if (started) { _from = before; _to = after; }
			issued(started, rtc_probe_wait_read_state, nowMs);
			break;
		}
		case rtc_probe_wait_read_state:
			if (landed(nowMs)) {
				_last.from = _from;
				_last.to = _to;
				for (int i = 0; i < 7; ++i) _last.regs[i] = _buf[i];
				_newRead = true;
				++_reads;
				enter(rtc_probe_idle_state, nowMs);
			}
			break;
		case rtc_probe_err_state:
			if (errorCleared(nowMs, rtc_probe_backoff_ms)) enter(rtc_probe_init_state, nowMs);
			break;
		default:
			enter(rtc_probe_init_state, nowMs);
			break;
		}
	}

	// Read every probePeriodMs for durationMs, starting now.
	void startProbe(uint32_t nowMs, uint32_t durationMs) {
		_probeEnd = nowMs + durationMs;
		_probing = true;
	}
	bool probing() const { return _probing; }

	// The chip answered and took the control register (until a failure).
	bool present() const { return _present; }
	uint32_t reads() const { return _reads; }

	// The latest read, once: false if none landed since the last call.
	bool takeRead(rtc_probe_read_t *out) {
		if (!_newRead) return false;
		_newRead = false;
		*out = _last;
		return true;
	}

protected:
	void onFail() override {
		_present = false;
		_newRead = false;
	}

private:
	rtc_probe_param_t _param;
	uint8_t _buf[7] = {0};            // the transport fills it after main() returns: a member
	rtc_probe_read_t _last = {};
	uint32_t _from = 0, _to = 0;
	uint32_t _lastRead = 0, _probeEnd = 0, _reads = 0;
	bool _probing = false, _newRead = false, _present = false;
};

// Under CMSIS-RTOS2: sleeps with osDelay (FreeRTOS ticks are 1 ms here).
template <typename TTransport>
class xrtc_probe : public rtc_probe<TTransport> {
public:
	template <typename... TArgs>
	explicit xrtc_probe(const rtc_probe_param_t &param, TArgs &&...transportArgs)
		: rtc_probe<TTransport>(param, std::forward<TArgs>(transportArgs)...) {}
protected:
	void sleep(uint32_t ms) override { osDelay(ms); }
};

#endif /* RTC_PROBE_H_ */
