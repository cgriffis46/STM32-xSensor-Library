/*
 * Gw121Bluetooth.h
 *
 *  The 121GW's Bluetooth protocol, so firmware of ours works with
 *  EEVblog's phone app and anything else that reads a stock meter. The
 *  BLE112 module is a transparent UART: USART2, 19200 baud. Pure: bytes
 *  in, bytes out. What carries them is the caller's.
 *
 *  Written from the format, not from anyone's code. The format was read
 *  from the stock firmware's build_and_write_bt_packet() and
 *  bt_process_received_msg() (121gw-re's decompiled firmware), and checked
 *  against how EEVblog's app reads it (Packet121GW.cs, EEVblog/EEVblog-121GW).
 *
 *  Meter to phone, 56 bytes, every field after the first in ASCII:
 *
 *    0       0xF2
 *    1..4    calibration year and month, 4 BCD digits (YYMM)
 *    5..9    serial number, 5 BCD digits
 *    10..11  mode (the stock meter_meas_mode: DCV 1, ACV 2, ... Gw121Routes.h's StockMode)
 *    12..13  range: bit 7 overload, bit 6 negative, bits 0..5 the range
 *    14..17  main value: |display digits|, 16 bits
 *    18..25  sub field 1: mode, flags (bit 7 overload, bit 6 negative, bits 0..3 decimal point), 16 bit value
 *    26..27  bargraph: bit 4 bar off, bit 3 range 1000, bit 2 negative, bits 0..1 range (5, 50, 500, 1000)
 *    28..29  bargraph segments
 *    30..31  status 1: 0x40 LPF, 0x20 1 ms peak, 0x18 AC+DC / 0x10 AC / 0x08 DC, 0x04 auto, 0x02 APO on, 0x01 battery low
 *    32..33  status 2: 0x40 Bluetooth, 0x20 danger, 0x10 REL, 0x08 dBm, bits 0..2 MIN/MAX (or peak)
 *    34..35  status 3: 0x10 logging, bits 2..3 hold
 *    36..43  sub field 2 (AC: the input frequency; VA: volts)
 *    44..51  sub field 3 (VA: amps)
 *    52..53  checksum: XOR of the bytes in 10..51 (each pair of hex digits one byte)
 *    54..55  CR LF
 *
 *  Hex digits are upper case. The app frames on 0xF2 and keeps the next
 *  52 bytes.
 *
 *  Phone to meter (the meter echoes a valid message back):
 *    0xF4 kk kk          a key: kk twice, hex. 01 RANGE, 02 HOLD, 03 REL, 04 PEAK,
 *                        05 MODE, 06 MIN/MAX, 07 MEM, 08 SETUP; + 0x80 for a long press
 *    0xF8 YY MM DD hh mm ss cc
 *                        set the clock: six bytes, hex, cc their XOR
 */

#ifndef GW121_BLUETOOTH_H_
#define GW121_BLUETOOTH_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "iMeter.h"
#include "Gw121Routes.h"

namespace gw121 {
namespace bt {

static constexpr uint8_t kStart      = 0xF2;
static constexpr uint8_t kKeyStart   = 0xF4;
static constexpr uint8_t kTimeStart  = 0xF8;
static constexpr size_t  kPacketLen  = 56;
static constexpr size_t  kKeyLen     = 5;
static constexpr size_t  kTimeLen    = 15;

// Field bits.
static constexpr uint8_t kOverload = 0x80, kNegative = 0x40;
static constexpr uint8_t kBarOff = 0x10, kBar1000 = 0x08, kBarNegative = 0x04;
static constexpr uint8_t kS1Lpf = 0x40, kS1Peak1ms = 0x20, kS1AcDc = 0x18, kS1Ac = 0x10, kS1Dc = 0x08,
                         kS1Auto = 0x04, kS1ApoOn = 0x02, kS1BattLow = 0x01;
static constexpr uint8_t kS2Bluetooth = 0x40, kS2Danger = 0x20, kS2Rel = 0x10, kS2Dbm = 0x08;
static constexpr uint8_t kS3Logging = 0x10;
// Sub-field modes besides the main modes.
static constexpr uint8_t kSubTempC = 100, kSubTempF = 105, kSubBattery = 110, kSubApoOn = 120, kSubApoOff = 125,
                         kSubYear = 130, kSubDate = 135, kSubTime = 140, kSubBurden = 150, kSubLcd = 160,
                         kSubDbm = 180, kSubInterval = 190;

struct SubField {
	uint8_t  mode = 0;
	uint8_t  flags = 0;    // kOverload, kNegative, decimal point in bits 0..3
	uint16_t value = 0;

	static SubField of(uint8_t mode, int32_t value, uint8_t decimals, bool overload = false) {
		SubField s;
		s.mode = mode;
		s.flags = (uint8_t)((decimals & 0x0F) | (value < 0 ? kNegative : 0) | (overload ? kOverload : 0));
		const int32_t m = value < 0 ? -value : value;
		s.value = (uint16_t)(m > 0xFFFF ? 0xFFFF : m);
		return s;
	}
};

struct Frame {
	uint16_t calYearMonth = 0;   // YYMM, e.g. 1712: shown as BCD digits
	uint32_t serial = 0;         // 0..99999
	uint8_t  mode = 0;           // StockMode
	uint8_t  range = 0;          // 0..63
	bool     overload = false;
	bool     negative = false;
	uint16_t digits = 0;         // |display digits|
	SubField sub1, sub2, sub3;
	uint8_t  bar = kBarOff;      // bargraph flags and range
	uint8_t  barSegments = 0;
	uint8_t  status1 = 0, status2 = 0, status3 = 0;
};

namespace detail {
inline char hex(uint8_t n) { n &= 0x0F; return (char)(n < 10 ? '0' + n : 'A' + (n - 10)); }
inline int  unhex(uint8_t c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}
inline void putByte(uint8_t* out, size_t pos, uint8_t v, uint8_t& ck) { out[pos] = (uint8_t)hex(v >> 4); out[pos + 1] = (uint8_t)hex(v); ck ^= v; }
inline void putWord(uint8_t* out, size_t pos, uint16_t v, uint8_t& ck) { putByte(out, pos, (uint8_t)(v >> 8), ck); putByte(out, pos + 2, (uint8_t)v, ck); }
inline void putSub(uint8_t* out, size_t pos, const SubField& s, uint8_t& ck) {
	putByte(out, pos, s.mode, ck); putByte(out, pos + 2, s.flags, ck); putWord(out, pos + 4, s.value, ck);
}
inline bool getByte(const uint8_t* in, size_t pos, uint8_t* v) {
	const int h = unhex(in[pos]), l = unhex(in[pos + 1]);
	if (h < 0 || l < 0) return false;
	*v = (uint8_t)((h << 4) | l);
	return true;
}
} // namespace detail

// Writes the 56 bytes of f to out.
inline size_t encode(const Frame& f, uint8_t* out) {
	using namespace detail;
	uint8_t ck = 0, unused = 0;
	out[0] = kStart;
	const uint16_t ym = f.calYearMonth % 10000u;
	out[1] = (uint8_t)('0' + ym / 1000); out[2] = (uint8_t)('0' + ym / 100 % 10);
	out[3] = (uint8_t)('0' + ym / 10 % 10); out[4] = (uint8_t)('0' + ym % 10);
	uint32_t sn = f.serial % 100000u;
	for (int i = 9; i >= 5; i--) { out[i] = (uint8_t)('0' + sn % 10); sn /= 10; }
	putByte(out, 10, f.mode, ck);
	putByte(out, 12, (uint8_t)((f.range & 0x3F) | (f.overload ? kOverload : 0) | (f.negative ? kNegative : 0)), ck);
	putWord(out, 14, f.digits, ck);
	putSub(out, 18, f.sub1, ck);
	putByte(out, 26, f.bar, ck);
	putByte(out, 28, f.barSegments, ck);
	putByte(out, 30, f.status1, ck);
	putByte(out, 32, f.status2, ck);
	putByte(out, 34, f.status3, ck);
	putSub(out, 36, f.sub2, ck);
	putSub(out, 44, f.sub3, ck);
	putByte(out, 52, ck, unused);
	out[54] = '\r';
	out[55] = '\n';
	return kPacketLen;
}

// Reads a packet back (for tests, and for a meter of ours that listens
// to another). False if it isn't one or the checksum is wrong.
inline bool decode(const uint8_t* in, Frame* f) {
	using namespace detail;
	if (in[0] != kStart) return false;
	uint8_t ck = 0, b[21];
	for (int i = 0; i < 21; i++) { if (!getByte(in, 10 + 2 * i, &b[i])) return false; ck ^= b[i]; }
	uint8_t sent;
	if (!getByte(in, 52, &sent) || sent != ck) return false;
	uint32_t ym = 0, sn = 0;
	for (int i = 1; i <= 4; i++) { if (in[i] < '0' || in[i] > '9') return false; ym = ym * 10 + (in[i] - '0'); }
	for (int i = 5; i <= 9; i++) { if (in[i] < '0' || in[i] > '9') return false; sn = sn * 10 + (in[i] - '0'); }
	f->calYearMonth = (uint16_t)ym;
	f->serial = sn;
	f->mode = b[0];
	f->range = b[1] & 0x3F;
	f->overload = (b[1] & kOverload) != 0;
	f->negative = (b[1] & kNegative) != 0;
	f->digits = (uint16_t)((b[2] << 8) | b[3]);
	f->sub1 = SubField{}; f->sub1.mode = b[4]; f->sub1.flags = b[5]; f->sub1.value = (uint16_t)((b[6] << 8) | b[7]);
	f->bar = b[8];
	f->barSegments = b[9];
	f->status1 = b[10]; f->status2 = b[11]; f->status3 = b[12];
	f->sub2 = SubField{}; f->sub2.mode = b[13]; f->sub2.flags = b[14]; f->sub2.value = (uint16_t)((b[15] << 8) | b[16]);
	f->sub3 = SubField{}; f->sub3.mode = b[17]; f->sub3.flags = b[18]; f->sub3.value = (uint16_t)((b[19] << 8) | b[20]);
	return true;
}

// ---- From the meter ----

// meter::Function to the stock mode number.
inline uint8_t stockModeOf(meter::Function f) {
	using F = meter::Function;
	switch (f) {
	case F::DcVolts:      return kMM_DCV;
	case F::AcVolts:
	case F::AcDcVolts:    return kMM_ACV;
	case F::DcMilliVolts: return kMM_DCmV;
	case F::AcMilliVolts: return kMM_ACmV;
	case F::Temperature:  return kMM_TEMP;
	case F::Frequency:    return kMM_HZ;
	case F::DutyCycle:    return 8;
	case F::Ohms:         return kMM_OHMS;
	case F::Continuity:   return kMM_CONT;
	case F::Diode:        return 11;
	case F::Capacitance:  return 12;
	case F::AcMicroAmps:  return kMM_ACuA;
	case F::DcMicroAmps:  return kMM_DCuA;
	case F::AcMilliAmps:  return kMM_ACmA;
	case F::DcMilliAmps:  return kMM_DCmA;
	case F::AcAmps:       return kMM_ACA;
	case F::DcAmps:       return kMM_DCA;
	default:              return kMM_DCV;
	}
}

inline bool isAcFn(meter::Function f) {
	using F = meter::Function;
	return f == F::AcVolts || f == F::AcDcVolts || f == F::AcMilliVolts || f == F::AcAmps || f == F::AcMilliAmps || f == F::AcMicroAmps;
}
inline bool isDcFn(meter::Function f) {
	using F = meter::Function;
	return f == F::DcVolts || f == F::DcMilliVolts || f == F::DcAmps || f == F::DcMilliAmps || f == F::DcMicroAmps;
}

// What the meter knows besides the reading.
struct MeterState {
	uint16_t calYearMonth = 0;
	uint32_t serial = 0;
	bool     bluetooth = true;
	bool     apoOn = true;
	bool     batteryLow = false;
	bool     lowPass = false;
	bool     logging = false;
	SubField sub1;                // the stock meter shows its own temperature here when nothing else
};

// The display digit of a 121GW range: the stock firmware's unit of the
// main value. AD1 ranges: unitsPerRaw x nominal counts per digit;
// the others: a step of the range's last decimal.
inline double digitOf(meter::Function f, uint8_t range) {
	for (uint8_t i = 0; i < kRouteCount; i++) {
		if (kRoutes[i].function != f || kRoutes[i].range != range) continue;
		for (uint8_t j = 0; j < kCalCount; j++)
			if (kCal[j].function == f && kCal[j].range == range && kCal[j].nominalCountsPerDigit > 0.0f)
				return (double)kRoutes[i].spec.unitsPerRaw * (double)kCal[j].nominalCountsPerDigit;
		return pow(10.0, -(double)kRoutes[i].spec.decimals);
	}
	return 0.0;
}

// A Frame for a meter::Reading.
inline Frame frameOf(const meter::Reading& r, const MeterState& s) {
	Frame f;
	f.calYearMonth = s.calYearMonth;
	f.serial = s.serial;
	f.mode = stockModeOf(r.function);
	f.range = r.range;
	f.overload = (r.status & meter::kOverload) != 0;
	const double d = digitOf(r.function, r.range);
	if (!f.overload && (r.status & meter::kValid) && d > 0.0) {
		const double n = fabs((double)r.value) / d + 0.5;
		if (n > 65535.0) f.overload = true;
		else f.digits = (uint16_t)n;
		f.negative = r.value < 0.0f && f.digits != 0;
	}
	f.sub1 = s.sub1;
	f.bar = kBarOff;              // no bargraph yet
	const bool ac = isAcFn(r.function);
	f.status1 = (uint8_t)((s.lowPass ? kS1Lpf : 0) |
	                      (r.function == meter::Function::AcDcVolts ? kS1AcDc : ac ? kS1Ac : isDcFn(r.function) ? kS1Dc : 0) |
	                      ((r.status & meter::kAutoRange) ? kS1Auto : 0) | (s.apoOn ? kS1ApoOn : 0) | (s.batteryLow ? kS1BattLow : 0));
	f.status2 = (uint8_t)((s.bluetooth ? kS2Bluetooth : 0) | ((r.status & meter::kRelative) ? kS2Rel : 0));
	f.status3 = (uint8_t)(((r.status & meter::kHold) ? (1u << 2) : 0) | (s.logging ? kS3Logging : 0));
	return f;
}

// ---- To the meter ----

enum class Key : uint8_t { Range = 1, Hold, Rel, Peak, Mode, MinMax, Mem, Setup };

struct Command {
	enum class Kind : uint8_t { Key, Time } kind = Kind::Key;
	Key  key = Key::Range;
	bool longPress = false;
	uint8_t year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;   // as sent
};

// Byte by byte from the UART. A message starts with 0xF4 or 0xF8 and has a
// fixed length; anything else is skipped, as the stock firmware does.
// A valid message is echoed by the stock meter: echo()/echoLen().
class Parser {
public:
	bool feed(uint8_t b, Command* out) {
		if (_pos == 0) {
			if (b == kKeyStart) _len = kKeyLen;
			else if (b == kTimeStart) _len = kTimeLen;
			else return false;
		}
		_buf[_pos++] = b;
		if (_pos < _len) return false;
		_pos = 0;
		return parse(out);
	}

	const uint8_t* echo() const { return _buf; }
	size_t echoLen() const { return _len; }
	void reset() { _pos = 0; }

private:
	bool parse(Command* out) {
		using detail::getByte;
		if (_buf[0] == kKeyStart) {
			uint8_t a, b;
			if (!getByte(_buf, 1, &a) || !getByte(_buf, 3, &b) || a != b) return false;
			const uint8_t k = a & 0x7F;
			if (k < 1 || k > 8) return false;
			out->kind = Command::Kind::Key;
			out->key = (Key)k;
			out->longPress = (a & 0x80) != 0;
			return true;
		}
		uint8_t v[7];
		for (int i = 0; i < 7; i++) if (!getByte(_buf, 1 + 2 * i, &v[i])) return false;
		if ((uint8_t)(v[0] ^ v[1] ^ v[2] ^ v[3] ^ v[4] ^ v[5]) != v[6]) return false;
		out->kind = Command::Kind::Time;
		out->year = v[0]; out->month = v[1]; out->day = v[2];
		out->hour = v[3]; out->minute = v[4]; out->second = v[5];
		return true;
	}

	uint8_t _buf[kTimeLen] = {};
	size_t  _pos = 0;
	size_t  _len = 0;
};

// The phone's side: a key message, for tests and tools.
inline size_t encodeKey(Key k, bool longPress, uint8_t* out) {
	const uint8_t c = (uint8_t)((uint8_t)k | (longPress ? 0x80 : 0));
	out[0] = kKeyStart;
	out[1] = (uint8_t)detail::hex(c >> 4); out[2] = (uint8_t)detail::hex(c);
	out[3] = out[1]; out[4] = out[2];
	return kKeyLen;
}

} // namespace bt
} // namespace gw121

#endif /* GW121_BLUETOOTH_H_ */
