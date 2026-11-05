/*
 * Gw121Simulator.h
 *
 *  A 121GW on the Bluetooth side, without the meter: our own firmware's
 *  meter stack (meter::Meter, MeterTags, gw121::Controller, the Bluetooth
 *  codec) on a simulated front end. Put it on any MCU with a BLE UART
 *  module (an Adafruit Bluefruit LE UART Friend, say), and a phone app or
 *  the Web Bluetooth page (staging/tools/web-bluetooth) sees a 121GW
 *  running our firmware: packets out (one a second by default), keys and
 *  the clock in.
 *
 *      Gw121Simulator sim(uart);            // uart: an iByteSink
 *      for every byte received:  sim.feed(b);
 *      in the loop:              sim.main(nowMs);
 *
 *  The signals are made up, per function: a base value, a sine on top,
 *  setSignal() to change them. Ranges and scaling are the real 121GW's
 *  (Gw121Routes.h), so auto range, decimals and units are as on the meter.
 *
 *  There's no rotary switch, so SETUP (from the phone or the page) steps
 *  the simulated switch position: V, mV, Hz, ohms, uA, mA/A. MODE then
 *  steps the functions of that position, as on the meter.
 *
 *  Hardware and OS agnostic, non-blocking: main() does what is due and
 *  returns.
 */

#ifndef GW121_SIMULATOR_H_
#define GW121_SIMULATOR_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "Meter.h"
#include "MeterTags.h"
#include "Gw121Routes.h"
#include "Gw121Bluetooth.h"
#include "Gw121Keys.h"
#include "Gw121RangeSwitch.h"

namespace gw121 {

class iByteSink {
public:
	virtual ~iByteSink() = default;
	// Queue n bytes for sending. Must not wait long: a UART with a FIFO or
	// an interrupt-driven buffer.
	virtual void write(const uint8_t* b, size_t n) = 0;
};

// A front end that makes its readings up, with the 121GW's ranges.
class SimFrontEnd : public meter::iMeterFrontEnd {
public:
	struct Signal { double base, amplitude; uint32_t periodMs; };

	SimFrontEnd() {
		for (auto& s : _signal) s = Signal{ NAN, 0, 10000 };
		using F = meter::Function;
		setSignal(F::DcVolts, 1.2345, 0.05);
		setSignal(F::AcVolts, 230.0, 0.5);
		setSignal(F::DcMilliVolts, 0.0123, 0.0005);
		setSignal(F::AcMilliVolts, 0.150, 0.002);
		setSignal(F::Ohms, 3300.0, 5.0);
		setSignal(F::Continuity, 12.3, 0.2);
		setSignal(F::DcMicroAmps, 123e-6, 2e-6);
		setSignal(F::AcMicroAmps, 45e-6, 1e-6);
		setSignal(F::DcMilliAmps, 12.3e-3, 0.2e-3);
		setSignal(F::AcMilliAmps, 4.5e-3, 0.1e-3);
		setSignal(F::DcAmps, 1.234, 0.01);
		setSignal(F::AcAmps, 0.75, 0.01);
		setSignal(F::Frequency, 50.0, 0.02);
	}

	void setSignal(meter::Function f, double base, double amplitude, uint32_t periodMs = 10000) {
		const uint8_t i = (uint8_t)f;
		if (i < kFns) _signal[i] = Signal{ base, amplitude, periodMs ? periodMs : 1 };
	}

	// One new result every resultMs.
	void tick(uint32_t nowMs) {
		if (!_selected || nowMs - _lastMs < kResultMs) return;
		_lastMs = nowMs;
		const Signal& s = _signal[(uint8_t)_fn];
		const RangeSpecPtr spec = rangeSpec(_fn, _range);
		if (!spec || isnan(s.base)) return;
		const double phase = 2.0 * 3.14159265358979 * (double)(nowMs % s.periodMs) / (double)s.periodMs;
		const double v = s.base + s.amplitude * sin(phase);
		_r.seq++;
		_r.atMs = nowMs;
		_r.raw = (float)(v / spec->unitsPerRaw);
		_r.overload = false;
		_r.rangeTag = _tag;
	}

	bool supports(meter::Function f) const override { return rangeCount(f) != 0; }
	uint8_t rangeCount(meter::Function f) const override {
		uint8_t n = 0;
		for (uint8_t i = 0; i < kRouteCount; i++) if (kRoutes[i].function == f) n++;
		return n;
	}
	const meter::RangeSpec* rangeSpec(meter::Function f, uint8_t r) const override {
		for (uint8_t i = 0; i < kRouteCount; i++) if (kRoutes[i].function == f && kRoutes[i].range == r) return &kRoutes[i].spec;
		return nullptr;
	}
	bool select(meter::Function f, uint8_t r, uint8_t tag) override {
		if (!rangeSpec(f, r)) return false;
		_fn = f; _range = r; _tag = tag; _selected = true;
		return true;
	}
	bool result(uint32_t lastSeq, meter::RawResult* out) override {
		if (_r.seq == 0 || _r.seq == lastSeq) return false;
		*out = _r;
		return true;
	}
	bool faulted() const override { return false; }
	uint8_t settleResults(meter::Function, uint8_t) const override { return 0; }

private:
	using RangeSpecPtr = const meter::RangeSpec*;
	static constexpr uint8_t  kFns = (uint8_t)meter::Function::Count_;
	static constexpr uint32_t kResultMs = 100;

	Signal  _signal[kFns];
	meter::Function _fn = meter::Function::None;
	uint8_t _range = 0, _tag = 0;
	bool    _selected = false;
	uint32_t _lastMs = 0;
	meter::RawResult _r{};
};

class Simulator {
public:
	// One packet a second by default. A BLE UART module forwards what it
	// gets only as fast as the link allows, and a PC usually allows less
	// than a phone: at 4 a second (224 bytes/s) an Adafruit UART Friend
	// without flow control dropped most of them on the way to Chrome on a PC.
	static constexpr uint32_t kDefaultPacketMs = 1000;

	explicit Simulator(iByteSink& out, uint32_t serial = 12345, uint16_t calYearMonth = 2611)
		: _out(out), _meter(_fe), _tags(_meter), _ctl(_tags), _serial(serial) {
		_state.serial = serial;
		_state.calYearMonth = calYearMonth;
		_state.bluetooth = true;
		_ctl.setPosition(Position::Volts);
		_meter.setFunction(firstFunction(Position::Volts));
	}

	// A byte from the BLE module.
	void feed(uint8_t b) {
		bt::Command c;
		if (!_parser.feed(b, &c)) return;
		_out.write(_parser.echo(), _parser.echoLen());   // as the stock meter does
		if (c.kind == bt::Command::Kind::Time) {
			_clock = c;
			_haveClock = true;
			return;
		}
		if (c.key == bt::Key::Setup && !c.longPress) {   // the simulated rotary switch
			_pos = nextPosition(_pos);
			_ctl.setPosition(_pos);
			_meter.setFunction(firstFunction(_pos));
			return;
		}
		_ctl.onBluetooth(c);
	}

	void main(uint32_t nowMs) {
		_fe.tick(nowMs);
		_meter.main(nowMs);
		if (nowMs - _lastPacketMs < _packetMs) return;
		meter::Reading r;
		if (!_meter.latest(&r)) return;
		_lastPacketMs = nowMs;
		uint8_t pkt[bt::kPacketLen];
		if (_sequenceSerial) _state.serial = _packets % 100000u;   // see setSequenceSerial()
		bt::encode(bt::frameOf(r, _state), pkt);
		_out.write(pkt, sizeof pkt);
		_packets++;
	}

	// How often a packet goes out, in ms (at least 100).
	void setPacketInterval(uint32_t ms) { _packetMs = ms < 100 ? 100 : ms; }

	// The serial number field counts the packets sent (0, 1, 2 ... 99999, 0),
	// so a receiver can tell exactly which ones went missing. The Web
	// Bluetooth page counts the gaps. On by default; a stand-in meter's serial
	// number means nothing. Off: the serial given to the constructor.
	void setSequenceSerial(bool on) { _sequenceSerial = on; if (!on) _state.serial = _serial; }
	uint32_t packetInterval() const { return _packetMs; }

	SimFrontEnd& frontEnd() { return _fe; }
	meter::Meter& meter() { return _meter; }
	meter::MeterTags& tags() { return _tags; }
	Position position() const { return _pos; }
	uint32_t packets() const { return _packets; }
	bool clock(bt::Command* c) const { if (_haveClock) *c = _clock; return _haveClock; }

private:
	static Position nextPosition(Position p) {
		switch (p) {
		case Position::Volts:      return Position::MilliVolts;
		case Position::MilliVolts: return Position::Hertz;
		case Position::Hertz:      return Position::Ohms;
		case Position::Ohms:       return Position::MicroAmps;
		case Position::MicroAmps:  return Position::Amps;
		default:                   return Position::Volts;
		}
	}

	iByteSink&       _out;
	SimFrontEnd      _fe;
	meter::Meter     _meter;
	meter::MeterTags _tags;
	Controller       _ctl;
	bt::Parser       _parser;
	bt::MeterState   _state;
	uint32_t         _serial;
	bool             _sequenceSerial = true;
	Position         _pos = Position::Volts;
	uint32_t         _lastPacketMs = 0;
	uint32_t         _packetMs = kDefaultPacketMs;
	uint32_t         _packets = 0;
	bt::Command      _clock{};
	bool             _haveClock = false;
};

} // namespace gw121

#endif /* GW121_SIMULATOR_H_ */
