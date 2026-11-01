/*
 * Gw121Keys.h
 *
 *  The 121GW's eight keys and its buzzer.
 *
 *  Keys: PG8..PG15, active low, one bit each as the stock firmware reads
 *  them (121gw-re: read_button_inputs(), meter_buttons). The schematic's
 *  labels agree. Sampled every 10 ms by idisplay::KeyScanner, with the
 *  stock firmware's timing (read_buttons()): debounce 2 samples, a short
 *  press reported on release, a long press at 0.5 s, HOLD + MEM as a chord.
 *
 *  Buzzer: PE7 high sounds it (beep_ms(), and the TIM3 1 ms interrupt).
 *
 *  Controller turns key events, from the keys or from Bluetooth (the
 *  phone's key messages, Gw121Bluetooth.h), into tag writes: the same
 *  meter.* tags an HMI or a PLC writes. So a key and a remote write take
 *  one path, and the meter doesn't know which it was. A key that did
 *  something beeps once; one that can't do anything (yet) beeps "no".
 *
 *    RANGE    short: manual range; on manual, the next range (wrapping).
 *             long: back to auto range
 *    HOLD     short: hold on / off
 *    REL      short: relative on / off
 *    MODE     short: the next function of the rotary switch's position
 *             (as the stock firmware's meter_incr_mode())
 *    MIN/MAX, PEAK, MEM, SETUP, and the long presses not above: not yet.
 */

#ifndef GW121_KEYS_H_
#define GW121_KEYS_H_

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "KeyScanner.h"
#include "Buzzer.h"
#include "iTagSource.h"
#include "Gw121RangeSwitch.h"
#include "Gw121Bluetooth.h"

namespace gw121 {

// One bit per key, as PG8..PG15 (the stock firmware's meter_buttons).
enum KeyBit : uint16_t {
	kKeyMode = 1u << 0, kKeySetup = 1u << 1, kKeyMinMax = 1u << 2, kKeyMem = 1u << 3,
	kKeyRange = 1u << 4, kKeyHold = 1u << 5, kKeyRel = 1u << 6, kKeyPeak = 1u << 7,
};

inline constexpr uint16_t kChords[] = { kKeyHold | kKeyMem };

// The keys down, from GPIOG's input register (active low, PG8..PG15).
inline uint16_t keysFromIdr(uint16_t idr) { return (uint16_t)(~idr >> 8) & 0xFF; }

inline idisplay::KeyScanner::Config keyConfig() {
	idisplay::KeyScanner::Config c;
	c.chords = kChords;
	c.chordCount = 1;
	return c;
}

// A Bluetooth key (1 RANGE .. 8 SETUP) as a key bit.
inline uint16_t keyBitOf(bt::Key k) {
	switch (k) {
	case bt::Key::Range:  return kKeyRange;
	case bt::Key::Hold:   return kKeyHold;
	case bt::Key::Rel:    return kKeyRel;
	case bt::Key::Peak:   return kKeyPeak;
	case bt::Key::Mode:   return kKeyMode;
	case bt::Key::MinMax: return kKeyMinMax;
	case bt::Key::Mem:    return kKeyMem;
	case bt::Key::Setup:  return kKeySetup;
	default:              return 0;
	}
}

class Controller {
public:
	Controller(idisplay::iTagSource& tags, idisplay::iBeeper* beeper = nullptr) : _tags(tags), _beep(beeper) {}

	// The rotary switch's position: what MODE steps through.
	void setPosition(Position p) { _pos = p; }
	Position position() const { return _pos; }

	// True if the key did something.
	bool onKey(const idisplay::KeyEvent& e) {
		const bool done = handle(e);
		if (_beep) { if (done) _beep->ok(); else _beep->no(); }
		return done;
	}

	bool onBluetooth(const bt::Command& c) {
		if (c.kind != bt::Command::Kind::Key) return false;
		idisplay::KeyEvent e;
		e.keys = keyBitOf(c.key);
		e.action = c.longPress ? idisplay::KeyEvent::Action::Held : idisplay::KeyEvent::Action::Pressed;
		return onKey(e);
	}

private:
	using A = idisplay::KeyEvent::Action;

	bool handle(const idisplay::KeyEvent& e) {
		const bool shortPress = e.action == A::Pressed;
		switch (e.keys) {
		case kKeyRange:
			if (e.action == A::Held) return set("meter.auto", true);
			if (!shortPress) return false;
			if (flag("meter.auto")) return set("meter.auto", false);
			{
				double r, n;
				if (!num("meter.range", r) || !num("meter.ranges", n) || n < 1) return false;
				return setNum("meter.range", fmod(r + 1, n));
			}
		case kKeyHold:
			return shortPress && set("meter.hold", !flag("meter.hold"));
		case kKeyRel:
			return shortPress && set("meter.rel", !flag("meter.rel"));
		case kKeyMode: {
			if (!shortPress) return false;
			double f;
			if (!num("meter.function", f)) return false;
			// The next function the meter takes: one it can't do yet is skipped.
			const meter::Function now = (meter::Function)(uint8_t)f;
			meter::Function next = now;
			for (uint8_t i = 0; i < 8; i++) {
				next = nextFunction(_pos, next);
				if (next == now) return false;
				if (setNum("meter.function", (double)(uint8_t)next)) return true;
			}
			return false;
		}
		default:
			return false;
		}
	}

	int32_t handle(const char* name) {
		for (auto& c : _cache) if (c.name && strcmp(c.name, name) == 0) return c.h >= 0 ? c.h : (c.h = _tags.find(name));
		for (auto& c : _cache) if (!c.name) { c.name = name; return c.h = _tags.find(name); }
		return _tags.find(name);
	}
	bool num(const char* n, double& out) {
		idisplay::xTagValue v;
		const int32_t h = handle(n);
		if (h < 0 || !_tags.read(h, v)) return false;
		if (v.kind == idisplay::xTagValue::Kind::Bool) { out = v.b; return true; }
		if (v.kind != idisplay::xTagValue::Kind::Number || isnan(v.number)) return false;
		out = v.number;
		return true;
	}
	bool flag(const char* n) { double v; return num(n, v) && v != 0.0; }
	bool set(const char* n, bool b) { const int32_t h = handle(n); return h >= 0 && _tags.write(h, idisplay::xTagValue::boolean(b)); }
	bool setNum(const char* n, double x) { const int32_t h = handle(n); return h >= 0 && _tags.write(h, idisplay::xTagValue::num(x)); }

	struct Cached { const char* name = nullptr; int32_t h = -1; };
	Cached _cache[8];
	idisplay::iTagSource& _tags;
	idisplay::iBeeper* _beep;
	Position _pos = Position::Volts;
};

} // namespace gw121

#endif /* GW121_KEYS_H_ */
