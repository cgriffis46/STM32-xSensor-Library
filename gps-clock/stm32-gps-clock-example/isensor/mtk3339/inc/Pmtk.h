/*
 * Pmtk.h
 *
 *  MediaTek's PMTK commands for the MT3339 (and the MT3329/MT3333
 *  family): NMEA-style text sentences, "$PMTKnnn,...*hh\r\n", answered
 *  with "$PMTK001,nnn,flag". Only what the mtk3339 driver sends is
 *  here. Header-only, no heap, no printf.
 *
 *  Sources: Adafruit's Adafruit_GPS library (its PMTK command strings
 *  with their checksums, which the tests check these builders against,
 *  and its antenna status parsing) and the field order of PMTK314 in
 *  Adafruit's CircuitPython GPS documentation. The PMTK001 flag values
 *  come from MediaTek's PMTK command manual, confirmed for 3 = success
 *  by one other source only. Not run against a module.
 */

#ifndef PMTK_H_
#define PMTK_H_

#include <stddef.h>
#include <stdint.h>

namespace pmtk {

// PMTK001's flag.
enum class AckFlag : uint8_t {
	Invalid = 0,       // not a valid packet
	Unsupported = 1,   // a valid packet the firmware does not support
	Failed = 2,        // valid and supported, but the action failed
	Succeeded = 3
};

const uint16_t kCmdAck = 1;          // PMTK001
const uint16_t kCmdSystemMsg = 10;   // PMTK010, sent by the module
const uint16_t kCmdUpdateRate = 220; // PMTK220
const uint16_t kCmdFixCtl = 300;     // PMTK300
const uint16_t kCmdNmeaOutput = 314; // PMTK314

// PMTK010's message: 001 means the module has just started (power on
// or reset), so whatever was configured in RAM may be gone.
const uint16_t kSystemStartup = 1;

// The sentences PMTK314 can choose, by their position among its 19
// fields: GLL, RMC, VTG, GGA, GSA, GSV, GRS, GST, nine reserved, ZDA,
// MCHN.
enum class Nmea : uint8_t { GLL = 0, RMC = 1, VTG = 2, GGA = 3, GSA = 4, GSV = 5, ZDA = 17 };
const uint8_t kOutputFields = 19;

// The MT3339 fixes at most 5 times a second (Adafruit's note); it can
// repeat a fix in its output faster than that.
const uint16_t kMinFixMs = 200;
const uint16_t kMinUpdateMs = 100;   // 10 Hz output
const uint16_t kMaxUpdateMs = 10000;

// Wraps body (without "$" and "*") into a whole sentence:
// "$" body "*" hh "\r\n". out needs strlen(body) + 6 bytes. Returns the
// length, or 0 if it does not fit.
inline size_t sentence(char* out, size_t cap, const char* body) {
	size_t n = 0;
	uint8_t x = 0;
	if (cap < 7) return 0;
	out[n++] = '$';
	for (const char* p = body; *p; ++p) {
		if (n + 6 > cap) return 0;   // room for "*hh\r\n" and the terminator
		out[n++] = *p;
		x ^= (uint8_t)*p;
	}
	static const char hex[] = "0123456789ABCDEF";
	out[n++] = '*';
	out[n++] = hex[x >> 4];
	out[n++] = hex[x & 15];
	out[n++] = '\r';
	out[n++] = '\n';
	out[n] = 0;
	return n;
}

// Builds a sentence body a piece at a time, without printf.
class Body {
public:
	Body() : _n(0) { _b[0] = 0; }
	Body& text(const char* s) {
		while (*s && _n + 1 < sizeof _b) _b[_n++] = *s++;
		_b[_n] = 0;
		return *this;
	}
	Body& number(uint32_t v) {
		char d[10];
		int k = 0;
		do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 10);
		while (k > 0 && _n + 1 < sizeof _b) _b[_n++] = d[--k];
		_b[_n] = 0;
		return *this;
	}
	const char* str() const { return _b; }
private:
	char   _b[64];
	size_t _n;
};

// PMTK314: rates[i] is how often field i's sentence is sent, in fixes
// (0 off, 1 every fix, up to 5).
inline size_t setNmeaOutput(char* out, size_t cap, const uint8_t rates[kOutputFields]) {
	Body b;
	b.text("PMTK314");
	for (uint8_t i = 0; i < kOutputFields; ++i) b.text(",").number(rates[i] > 5 ? 5 : rates[i]);
	return sentence(out, cap, b.str());
}

// PMTK220: output every ms (100 to 10000).
inline size_t setUpdateRate(char* out, size_t cap, uint16_t ms) {
	Body b;
	b.text("PMTK220,").number(ms);
	return sentence(out, cap, b.str());
}

// PMTK300: fix every ms (200 or more), the other four fields 0 as in
// Adafruit's commands.
inline size_t setFixInterval(char* out, size_t cap, uint16_t ms) {
	Body b;
	b.text("PMTK300,").number(ms).text(",0,0,0,0");
	return sentence(out, cap, b.str());
}

// GlobalTop's antenna status report on or off: the module then sends
// "$PGTOP,11,x". No PMTK001 answer is expected (not a PMTK packet).
inline size_t antennaStatus(char* out, size_t cap, bool on) {
	return sentence(out, cap, on ? "PGCMD,33,1" : "PGCMD,33,0");
}

} // namespace pmtk

#endif /* PMTK_H_ */
