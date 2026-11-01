/*
 * MeterTags.h
 *
 *  meter::MeterTags: a meter as PLC-style tags, an idisplay::iTagSource.
 *  An HMI (the 121GW's segment LCD through xSegmentHmi, a character LCD
 *  through xHmi), the serial shell, a PLC tag server or the web server
 *  reads the meter by name, and writes the control tags to drive it:
 *
 *    meter.value      the reading in display units (prefix applied)       read
 *    meter.si         the reading in SI units                             read
 *    meter.decimals   digits after the point on the display                read
 *    meter.prefix     the display prefix as a power of ten (-9 .. 6)       read
 *    meter.unit       meter::Unit                                          read
 *    meter.valid      a measurement is shown                               read
 *    meter.ol         overload                                             read
 *    meter.settling   a range or function change is under way              read
 *    meter.fault      the front end failed and is recovering               read
 *    meter.uncal      no calibration for this range                       read
 *    meter.ac, meter.dc   AC, DC (both in AC+DC)                         read
 *    meter.acdc       AC+DC                                                read
 *    meter.min, meter.max  in display units                                read
 *    meter.bar        the reading as a fraction of full scale, -1 .. 1     read
 *    meter.barrange   full scale as 0: 5, 1: 50, 2: 500, 3: 1000           read
 *    meter.function   meter::Function                                      read, write
 *    meter.range      the range index                                      read, write
 *    meter.auto       auto range                                           read, write
 *    meter.hold       hold                                                 read, write
 *    meter.rel        relative                                             read, write
 *
 *  Reads take the meter's latest Reading (a lock-free copy) each time:
 *  cheap, and from any thread. Writes go to the meter's one-deep request
 *  mailboxes, so they never wait.
 *
 *  The display format comes from the range's full scale, for a display of
 *  displayDigits (5 on the 121GW): the prefix that puts full scale in 1..999
 *  (1000 stays 1000), and the decimals that leave room for it. 5 V: 5.0000.
 *  50 mV: 50.000 m. 5 kohm: 5.0000 k. 1000 V: 1000.0.
 */

#ifndef METER_TAGS_H_
#define METER_TAGS_H_

#include <math.h>
#include <string.h>
#include "iTagSource.h"
#include "iMeter.h"

namespace meter {

struct DisplayFormat {
	int8_t  prefix;     // power of ten
	uint8_t decimals;
	uint8_t barRange;   // 0: 5, 1: 50, 2: 500, 3: 1000
	float   scale;      // multiply SI by this for display units
};

inline DisplayFormat displayFormat(const RangeSpec& spec, uint8_t displayDigits = 5) {
	DisplayFormat d{0, 0, 0, 1.0f};
	// The smallest prefix that keeps full scale at 1000 or under: 1000 V
	// stays 1000.0 V, 5000 ohm becomes 5.0000 k.
	static const int8_t kPrefixes[] = { -9, -6, -3, 0, 3, 6 };
	const double fs = spec.fullScale;
	d.prefix = 6;
	for (int8_t p : kPrefixes) {
		if (fs / pow(10.0, p) <= 1000.0 * (1.0 + 1e-9)) { d.prefix = p; break; }
	}
	const double mant = fs / pow(10.0, d.prefix);
	int intDigits = 1;
	for (double x = mant; x >= 10.0 - 1e-9; x /= 10.0) intDigits++;
	const int dec = (int)displayDigits - intDigits;
	d.decimals = (uint8_t)(dec < 0 ? 0 : dec);
	d.barRange = mant >= 999.0 ? 3 : mant >= 99.0 ? 2 : mant >= 9.0 ? 1 : 0;
	d.scale = (float)pow(10.0, -d.prefix);
	return d;
}

class MeterTags : public idisplay::iTagSource {
public:
	enum Tag : int32_t {
		Value, Si, Decimals, Prefix, UnitTag, Valid, Ol, Settling, Fault, Uncal, Ac, Dc, AcDc,
		Min, Max, Bar, BarRange, FunctionTag, Range, Auto, Hold, Rel, kCount
	};

	explicit MeterTags(iMeter& m, uint8_t displayDigits = 5, const char* prefix = "meter.")
		: _m(m), _digits(displayDigits), _prefix(prefix) {}

	int32_t find(const char* name) override {
		const size_t pl = strlen(_prefix);
		if (strncmp(name, _prefix, pl) != 0) return -1;
		for (int32_t i = 0; i < kCount; i++) if (strcmp(name + pl, kNames[i]) == 0) return i;
		return -1;
	}

	bool read(int32_t h, idisplay::xTagValue& out) override {
		using idisplay::xTagValue;
		Reading r;
		if (h < 0 || h >= kCount || !_m.latest(&r)) return false;
		const RangeSpec* spec = r.function != Function::None ? _m.rangeSpec(r.function, r.range) : nullptr;
		DisplayFormat d{0, 0, 0, 1.0f};
		if (spec) d = displayFormat(*spec, _digits);
		const bool valid = (r.status & kValid) != 0;
		switch ((Tag)h) {
		case Value:       out = valid ? xTagValue::num((double)r.value * d.scale) : xTagValue::num(NAN); return true;
		case Si:          out = valid ? xTagValue::num(r.value) : xTagValue::num(NAN); return true;
		case Decimals:    out = xTagValue::num(d.decimals); return true;
		case Prefix:      out = xTagValue::num(d.prefix); return true;
		case UnitTag:     out = xTagValue::num((double)(uint8_t)r.unit); return true;
		case Valid:       out = xTagValue::boolean(valid); return true;
		case Ol:          out = xTagValue::boolean((r.status & kOverload) != 0); return true;
		case Settling:    out = xTagValue::boolean((r.status & kSettling) != 0); return true;
		case Fault:       out = xTagValue::boolean((r.status & kFault) != 0); return true;
		case Uncal:       out = xTagValue::boolean((r.status & kUncal) != 0); return true;
		case Ac:          out = xTagValue::boolean(isAc(r.function)); return true;
		case Dc:          out = xTagValue::boolean(isDc(r.function) || r.function == Function::AcDcVolts); return true;
		case AcDc:        out = xTagValue::boolean(r.function == Function::AcDcVolts); return true;
		case Min:         out = r.count ? xTagValue::num((double)r.min * d.scale) : xTagValue::num(NAN); return true;
		case Max:         out = r.count ? xTagValue::num((double)r.max * d.scale) : xTagValue::num(NAN); return true;
		case Bar:         out = (valid && spec) ? xTagValue::num(r.value / spec->fullScale) : xTagValue::num(NAN); return true;
		case BarRange:    out = xTagValue::num(d.barRange); return true;
		case FunctionTag: out = xTagValue::num((double)(uint8_t)r.function); return true;
		case Range:       out = xTagValue::num(r.range); return true;
		case Auto:        out = xTagValue::boolean((r.status & kAutoRange) != 0); return true;
		case Hold:        out = xTagValue::boolean((r.status & kHold) != 0); return true;
		case Rel:         out = xTagValue::boolean((r.status & kRelative) != 0); return true;
		default:          return false;
		}
	}

	bool write(int32_t h, const idisplay::xTagValue& v) override {
		using K = idisplay::xTagValue::Kind;
		const bool b = v.kind == K::Bool ? v.b : (v.kind == K::Number && v.number != 0.0);
		const bool whole = v.kind == K::Number && v.number >= 0 && v.number <= 255 && floor(v.number) == v.number;
		switch ((Tag)h) {
		case FunctionTag: return whole && v.number < (double)(uint8_t)Function::Count_ && _m.setFunction((Function)(uint8_t)v.number);
		case Range:       return whole && _m.setRange((uint8_t)v.number);
		case Auto:        if (v.kind == K::None) return false; _m.setAutoRange(b); return true;
		case Hold:        if (v.kind == K::None) return false; _m.hold(b); return true;
		case Rel:         if (v.kind == K::None) return false; _m.relative(b); return true;
		default:          return false;
		}
	}

	static const char* name(int32_t h) { return (h >= 0 && h < kCount) ? kNames[h] : nullptr; }

private:
	static bool isAc(Function f) {
		return f == Function::AcVolts || f == Function::AcDcVolts || f == Function::AcMilliVolts ||
		       f == Function::AcAmps || f == Function::AcMilliAmps || f == Function::AcMicroAmps;
	}
	static bool isDc(Function f) {
		return f == Function::DcVolts || f == Function::DcMilliVolts || f == Function::DcAmps ||
		       f == Function::DcMilliAmps || f == Function::DcMicroAmps;
	}

	static constexpr const char* kNames[kCount] = {
		"value", "si", "decimals", "prefix", "unit", "valid", "ol", "settling", "fault", "uncal", "ac", "dc", "acdc",
		"min", "max", "bar", "barrange", "function", "range", "auto", "hold", "rel",
	};

	iMeter&     _m;
	uint8_t     _digits;
	const char* _prefix;
};

} // namespace meter

#endif /* METER_TAGS_H_ */
