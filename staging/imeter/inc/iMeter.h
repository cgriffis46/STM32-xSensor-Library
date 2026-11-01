/*
 * iMeter.h
 *
 *  The abstract interfaces of a multimeter, independent of the chip that
 *  measures and of the board around it:
 *
 *    iMeter            what an application or HMI uses: pick a function and
 *                      range, read the result. Implemented by meter::Meter.
 *    iMeterFrontEnd    what a measuring front end does for Meter: configure
 *                      itself for a function and range, produce raw results.
 *                      Implemented per chip and board (Hy3131FrontEnd for
 *                      the HY3131 with a board's route table).
 *    iCalibrationStore where per-range calibration lives (EEPROM, flash, a
 *                      file on the SD card).
 *    iMeterListener    told of each reading, in the meter's thread.
 *
 *  Values are SI, in float: a 50,000 count reading needs 17 bits and float
 *  has 24. The STM32L1 has no FPU, but a few readings a second cost nothing.
 */

#ifndef IMETER_H_
#define IMETER_H_

#include <stdint.h>

namespace meter {

enum class Function : uint8_t {
	None = 0,
	DcVolts, AcVolts, AcDcVolts,
	DcMilliVolts, AcMilliVolts,
	Ohms, LowOhms, Continuity, Diode,
	Capacitance,
	Frequency, DutyCycle,
	DcAmps, AcAmps, DcMilliAmps, AcMilliAmps, DcMicroAmps, AcMicroAmps,
	Temperature,
	Count_   // not a function: how many there are
};

enum class Unit : uint8_t { None, Volt, Ampere, Ohm, Farad, Hertz, Percent, Celsius };

const char* functionName(Function f);
const char* unitSymbol(Unit u);

// One range of one function, as the board gives it.
struct RangeSpec {
	float       fullScale;     // the largest |value| the range shows, in SI units (5.0 for "5 V")
	float       unitsPerRaw;   // SI units per raw count from the front end
	float       overloadRaw;   // |raw| at or above this is overload (OL); 0: fullScale / unitsPerRaw
	Unit        unit;
	uint8_t     decimals;      // digits after the point, for display
	const char* label;         // "5V", "50mV", ...
};

// A raw result from the front end, before scale and calibration.
struct RawResult {
	uint32_t seq;          // increases with every new result
	uint32_t atMs;
	float    raw;          // counts (or Hz, or a ratio): what unitsPerRaw scales
	bool     overload;     // the front end itself saw it out of range
	uint8_t  rangeTag;     // the range the front end was in when it measured
};

// Linear: value = (raw - offset) * gain * unitsPerRaw.
struct Calibration {
	float gain;
	float offset;
	bool  valid;           // false: defaults (gain 1, offset 0) are in use
};

enum Status : uint16_t {
	kValid      = 0x0001,   // value is a measurement
	kOverload   = 0x0002,   // OL
	kAutoRange  = 0x0004,   // the range was chosen automatically
	kSettling   = 0x0008,   // a range or function change, results not in yet
	kUncal      = 0x0010,   // no calibration for this range
	kHold       = 0x0020,   // held: value is the one at hold()
	kRelative   = 0x0040,   // value is relative to the reference
	kFault      = 0x0080,   // the front end failed and is recovering
};

struct Reading {
	uint32_t seq;
	uint32_t atMs;
	Function function;
	uint8_t  range;
	Unit     unit;
	float    value;        // SI
	float    raw;
	uint16_t status;       // Status bits
	float    min, max;     // since the function, range or reset: what MIN/MAX shows
	uint32_t count;        // readings in min/max
};

class iMeterListener {
public:
	virtual ~iMeterListener() = default;
	virtual void onReading(const Reading& r) { (void)r; }
	virtual void onFunctionChanged(Function f, uint8_t range, bool autoRange) { (void)f; (void)range; (void)autoRange; }
	virtual void onFault() {}
};

// The application side.
class iMeter {
public:
	virtual ~iMeter() = default;

	// Requests, from any thread. Each is a one-deep mailbox: a newer
	// request replaces one not yet taken, so a rotary switch turned
	// quickly ends on where it stopped, not on every position in between.
	virtual bool setFunction(Function f) = 0;              // auto range on
	virtual bool setRange(uint8_t range) = 0;              // manual range
	virtual void setAutoRange(bool on) = 0;
	virtual void hold(bool on) = 0;
	virtual void relative(bool on) = 0;                    // on: the current value becomes the zero
	virtual void resetMinMax() = 0;

	virtual bool latest(Reading* out) const = 0;           // any thread
	virtual bool supports(Function f) const = 0;
	virtual uint8_t rangeCount(Function f) const = 0;
	virtual const RangeSpec* rangeSpec(Function f, uint8_t range) const = 0;

	virtual void setListener(iMeterListener* l) = 0;
};

// The measuring side. Non-blocking: select() only starts the change.
class iMeterFrontEnd {
public:
	virtual ~iMeterFrontEnd() = default;

	virtual bool supports(Function f) const = 0;
	virtual uint8_t rangeCount(Function f) const = 0;
	virtual const RangeSpec* rangeSpec(Function f, uint8_t range) const = 0;

	// Start reconfiguring for f, range. False: not possible (unsupported,
	// or busy: ask again). Results of the new setting come with
	// RawResult::rangeTag == tag.
	virtual bool select(Function f, uint8_t range, uint8_t tag) = 0;

	// The newest result, if there is one newer than lastSeq.
	virtual bool result(uint32_t lastSeq, RawResult* out) = 0;

	// The front end failed and is recovering (results will stop for a while).
	virtual bool faulted() const = 0;

	// Results to throw away after select() before trusting one (the
	// input network settles, filters fill). Per function and range.
	virtual uint8_t settleResults(Function f, uint8_t range) const { (void)f; (void)range; return 1; }
};

class iCalibrationStore {
public:
	virtual ~iCalibrationStore() = default;
	virtual bool load(Function f, uint8_t range, Calibration* out) = 0;
	virtual bool store(Function f, uint8_t range, const Calibration& c) = 0;
};

inline const char* functionName(Function f) {
	switch (f) {
	case Function::None:         return "OFF";
	case Function::DcVolts:      return "V DC";
	case Function::AcVolts:      return "V AC";
	case Function::AcDcVolts:    return "V AC+DC";
	case Function::DcMilliVolts: return "mV DC";
	case Function::AcMilliVolts: return "mV AC";
	case Function::Ohms:         return "Ohm";
	case Function::LowOhms:      return "Low Ohm";
	case Function::Continuity:   return "Continuity";
	case Function::Diode:        return "Diode";
	case Function::Capacitance:  return "Cap";
	case Function::Frequency:    return "Hz";
	case Function::DutyCycle:    return "Duty";
	case Function::DcAmps:       return "A DC";
	case Function::AcAmps:       return "A AC";
	case Function::DcMilliAmps:  return "mA DC";
	case Function::AcMilliAmps:  return "mA AC";
	case Function::DcMicroAmps:  return "uA DC";
	case Function::AcMicroAmps:  return "uA AC";
	case Function::Temperature:  return "Temp";
	default:                     return "?";
	}
}

inline const char* unitSymbol(Unit u) {
	switch (u) {
	case Unit::Volt:    return "V";
	case Unit::Ampere:  return "A";
	case Unit::Ohm:     return "Ohm";
	case Unit::Farad:   return "F";
	case Unit::Hertz:   return "Hz";
	case Unit::Percent: return "%";
	case Unit::Celsius: return "C";
	default:            return "";
	}
}

} // namespace meter

#endif /* IMETER_H_ */
