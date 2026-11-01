/*
 * Gw121RangeSwitch.h
 *
 *  The 121GW's rotary switch: eight contacts on PG0..PG7, active low, one
 *  closed at a time. RangeSwitch debounces them as the stock firmware does
 *  (121gw-re: read_range_switch(), update_range_switch_inputs(),
 *  exactly_one_range_is_active(), update_mode_based_on_range_switch()):
 *
 *    * sample every 10 ms (the stock firmware's TIM2 tick);
 *    * a new position counts once the same single contact has been read
 *      kStableSamples times running;
 *    * then no other change is taken for kLockoutSamples.
 *
 *  A position change is a one-deep event: poll() returns it once.
 *
 *  Which contact is which: the stock firmware reads PG7 as Low Z, PG6 V,
 *  PG5 mV, PG4 Hz, PG3 ohms, PG2 VA, PG1 uA, PG0 mA/A. The schematic's net
 *  labels (121GW EEVBlog Circuit diagram.pdf) name them the other way round:
 *  PG0 LowZ .. PG7 mA/A. The firmware is what runs on the meter, so its
 *  order is kStockPinOf. Check it on a meter before trusting either one.
 */

#ifndef GW121_RANGE_SWITCH_H_
#define GW121_RANGE_SWITCH_H_

#include <stdint.h>
#include "iMeter.h"

namespace gw121 {

enum class Position : uint8_t { LowZ = 0, Volts, MilliVolts, Hertz, Ohms, Power, MicroAmps, Amps, None = 0xFF };

// Pin (PGn) of each Position, in Position order.
inline constexpr uint8_t kStockPinOf[8]     = { 7, 6, 5, 4, 3, 2, 1, 0 };   // the stock firmware
inline constexpr uint8_t kSchematicPinOf[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };   // the schematic's net labels

// The function a position starts in, and what MODE steps through (the
// stock firmware's meter_incr_mode(), for the functions meter:: has).
// Low Z and VA aren't meter functions yet: Low Z starts as DC volts, VA as none.
inline meter::Function firstFunction(Position p) {
	using F = meter::Function;
	switch (p) {
	case Position::LowZ:       return F::DcVolts;
	case Position::Volts:      return F::DcVolts;
	case Position::MilliVolts: return F::DcMilliVolts;
	case Position::Hertz:      return F::Frequency;
	case Position::Ohms:       return F::Ohms;
	case Position::MicroAmps:  return F::DcMicroAmps;
	case Position::Amps:       return F::DcMilliAmps;   // mA or A by which jack is used: a jack detector, still to do
	default:                   return F::None;
	}
}

inline meter::Function nextFunction(Position p, meter::Function f) {
	using F = meter::Function;
	switch (p) {
	case Position::Volts:      return f == F::DcVolts ? F::AcVolts : f == F::AcVolts ? F::AcDcVolts : F::DcVolts;
	case Position::MilliVolts: return f == F::DcMilliVolts ? F::AcMilliVolts : f == F::AcMilliVolts ? F::Temperature : F::DcMilliVolts;
	case Position::Hertz:      return f == F::Frequency ? F::DutyCycle : F::Frequency;
	case Position::Ohms:       return f == F::Ohms ? F::Continuity : f == F::Continuity ? F::Diode : f == F::Diode ? F::Capacitance : F::Ohms;
	case Position::MicroAmps:  return f == F::DcMicroAmps ? F::AcMicroAmps : F::DcMicroAmps;
	case Position::Amps:       return f == F::DcMilliAmps ? F::AcMilliAmps : f == F::AcMilliAmps ? F::DcAmps : f == F::DcAmps ? F::AcAmps : F::DcMilliAmps;
	default:                   return f;
	}
}

class RangeSwitch {
public:
	static constexpr uint8_t kStableSamples  = 5;
	static constexpr uint8_t kLockoutSamples = 15;

	explicit RangeSwitch(const uint8_t (&pinOf)[8] = kStockPinOf) : _pinOf(pinOf) {}

	// One sample: the port's input register (bits PG0..PG7; the contacts are active low).
	void sample(uint16_t portIdr) {
		uint8_t closed = 0;
		for (uint8_t p = 0; p < 8; p++)
			if (!((portIdr >> _pinOf[p]) & 1u)) closed |= (uint8_t)(1u << p);
		if (_lockout) _lockout--;
		if (closed != _last) { _last = closed; _same = 0; return; }
		if (++_same < kStableSamples) return;
		_same = 0;
		const bool oneHot = closed && !(closed & (closed - 1));
		if (!oneHot || closed == _current || _lockout) return;
		_current = closed;
		_lockout = kLockoutSamples;
		uint8_t p = 0;
		while (!((closed >> p) & 1u)) p++;
		_position = (Position)p;
		_changed = true;
	}

	// The new position, once per change.
	bool poll(Position* out) {
		if (!_changed) return false;
		_changed = false;
		*out = _position;
		return true;
	}

	Position position() const { return _position; }

private:
	const uint8_t (&_pinOf)[8];
	uint8_t  _last = 0, _current = 0, _same = 0, _lockout = 0;
	Position _position = Position::None;
	bool     _changed = false;
};

} // namespace gw121

#endif /* GW121_RANGE_SWITCH_H_ */
