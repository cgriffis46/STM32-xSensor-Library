/*
 * NmeaParser.tpp
 *
 *  NmeaParser's member definitions. Not a template, but kept as a
 *  header (every function inline) like the drivers around it, so
 *  isensor stays header-only: include NmeaParser.h, not this file.
 */

#ifndef NMEA_PARSER_TPP_
#define NMEA_PARSER_TPP_

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "../inc/NmeaParser.h"

inline void NmeaParser::reset() {
	_len = 0;
	_overflow = false;
	_fieldCount = 0;
	_talker = 0;
	memset(&_stats, 0, sizeof(_stats));
	invalidate();
}

inline void NmeaParser::invalidate() {
	memset(&_data, 0, sizeof(_data));
	_data.altitudeM = NAN;
	_data.geoidSeparationM = NAN;
	_data.speedMps = NAN;
	_data.courseDeg = NAN;
	_data.hdop = NAN;
	_data.pdop = NAN;
	_data.vdop = NAN;
}

inline uint8_t NmeaParser::checksum(const char* body, size_t len) {
	uint8_t x = 0;
	for (size_t i = 0; i < len; ++i) x ^= (uint8_t)body[i];
	return x;
}

static inline int nmeaHex(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}

inline NmeaParser::Sentence NmeaParser::feed(uint8_t c) {
	if (c == '$' || c == '!') {
		// A new sentence. One still open lost its end.
		const bool cutShort = _len > 0;
		_len = 0;
		_fieldCount = 0;   // the last sentence's fields go with its text
		_overflow = false;
		_buf[_len++] = (char)c;
		if (cutShort) { ++_stats.malformed; return Sentence::Bad; }
		return Sentence::None;
	}
	if (_len == 0) return Sentence::None;   // between sentences

	if (c == '\r' || c == '\n' || c < 0x20 || c > 0x7E) {
		// The end of the line, or not text at all (a UBX frame), before
		// the checksum was complete.
		_len = 0;
		++_stats.malformed;
		return Sentence::Bad;
	}
	if (_len >= kMaxSentence) {
		_len = 0;
		++_stats.malformed;
		return Sentence::Bad;
	}
	_buf[_len++] = (char)c;

	// Whole once two characters follow the "*".
	if (_len >= 4 && _buf[_len - 3] == '*') return finish();
	return Sentence::None;
}

inline NmeaParser::Sentence NmeaParser::finish() {
	const size_t star = _len - 3;
	const size_t bodyLen = star - 1;   // between "$" and "*"
	_len = 0;   // whatever happens, the next character starts afresh

	const int hi = nmeaHex(_buf[star + 1]), lo = nmeaHex(_buf[star + 2]);
	if (hi < 0 || lo < 0) { ++_stats.malformed; return Sentence::Bad; }
	if (checksum(_buf + 1, bodyLen) != (uint8_t)(hi * 16 + lo)) {
		++_stats.checksumErrors;
		return Sentence::Bad;
	}

	// Fields: the address, then each comma-separated value. A "*"
	// inside the body would have ended the sentence early, so the body
	// is all fields and commas.
	_fieldCount = 0;
	_fieldStart[_fieldCount++] = 1;
	for (size_t i = 1; i < star; ++i) {
		if (_buf[i] == ',') {
			if (_fieldCount >= kMaxFields) { ++_stats.malformed; return Sentence::Bad; }
			_fieldStart[_fieldCount++] = (uint8_t)(i + 1);
		}
	}
	_fieldStart[_fieldCount] = (uint8_t)(star + 1);   // one past the last field, as if a "," stood at the "*"

	++_stats.sentences;
	const Sentence s = dispatch();
	if (s == Sentence::Bad) { --_stats.sentences; ++_stats.malformed; }
	return s;
}

// Field i, without its comma. False if the sentence has no field i.
inline bool NmeaParser::field(uint8_t i, const char** s, size_t* len) const {
	if (i >= _fieldCount) { *s = ""; *len = 0; return false; }
	*s = _buf + _fieldStart[i];
	*len = (size_t)(_fieldStart[i + 1] - 1 - _fieldStart[i]);
	return true;
}

inline NmeaParser::Sentence NmeaParser::dispatch() {
	const char* a; size_t n;
	field(0, &a, &n);
	if (_buf[0] != '$' || n != 5 || a[0] == 'P') return Sentence::Other;   // AIS ("!"), proprietary ("$P...")

	_talker = (a[0] == 'B' && a[1] == 'D') ? 'B' : a[1];   // "BD" is BeiDou's older talker ID

	const char* t = a + 2;
	// GSV checks all of its fields before it changes anything. The
	// others write as they go, so the fields they can touch (everything
	// before the satellites) are kept, and put back if a later field
	// turns out to be malformed.
	if (!memcmp(t, "GSV", 3)) return doGSV() ? Sentence::GSV : Sentence::Bad;

	Sentence s;
	bool (NmeaParser::*parse)();
	if      (!memcmp(t, "GGA", 3)) { s = Sentence::GGA; parse = &NmeaParser::doGGA; }
	else if (!memcmp(t, "RMC", 3)) { s = Sentence::RMC; parse = &NmeaParser::doRMC; }
	else if (!memcmp(t, "GLL", 3)) { s = Sentence::GLL; parse = &NmeaParser::doGLL; }
	else if (!memcmp(t, "VTG", 3)) { s = Sentence::VTG; parse = &NmeaParser::doVTG; }
	else if (!memcmp(t, "GSA", 3)) { s = Sentence::GSA; parse = &NmeaParser::doGSA; }
	else if (!memcmp(t, "ZDA", 3)) { s = Sentence::ZDA; parse = &NmeaParser::doZDA; }
	else return Sentence::Other;

	const size_t kept = offsetof(GnssData, satelliteCount);
	uint8_t before[offsetof(GnssData, satelliteCount)];
	memcpy(before, &_data, kept);
	if ((this->*parse)()) return s;
	memcpy(&_data, before, kept);
	return Sentence::Bad;
}

// ---- field parsers ----

inline bool NmeaParser::parseUint(const char* s, size_t len, uint32_t* out) {
	if (len == 0 || len > 9) return false;
	uint32_t v = 0;
	for (size_t i = 0; i < len; ++i) {
		if (s[i] < '0' || s[i] > '9') return false;
		v = v * 10 + (uint32_t)(s[i] - '0');
	}
	*out = v;
	return true;
}

inline bool NmeaParser::parseFloat(const char* s, size_t len, float* out) {
	*out = NAN;
	size_t i = 0;
	bool neg = false;
	if (i < len && (s[i] == '-' || s[i] == '+')) { neg = s[i] == '-'; ++i; }
	uint32_t whole = 0, frac = 0, scale = 1;
	bool digits = false;
	for (; i < len && s[i] != '.'; ++i) {
		if (s[i] < '0' || s[i] > '9') return false;
		if (whole > 99999999u) return false;   // more than any NMEA value needs
		whole = whole * 10 + (uint32_t)(s[i] - '0');
		digits = true;
	}
	if (i < len) {   // the "."
		for (++i; i < len; ++i) {
			if (s[i] < '0' || s[i] > '9') return false;
			if (scale < 1000000000u / 10) {   // further digits are below float's precision anyway
				frac = frac * 10 + (uint32_t)(s[i] - '0');
				scale *= 10;
			}
			digits = true;
		}
	}
	if (!digits) return false;
	const float v = (float)whole + (float)frac / (float)scale;
	*out = neg ? -v : v;
	return true;
}

inline bool NmeaParser::parseCoordinate(const char* s, size_t len, int32_t* e7) {
	size_t dot = 0;
	while (dot < len && s[dot] != '.') ++dot;
	if (dot < 3 || dot > 5) return false;   // ddmm or dddmm, with at most 3 degree digits

	uint32_t deg = 0, minutes = 0;
	for (size_t i = 0; i < dot; ++i) {
		if (s[i] < '0' || s[i] > '9') return false;
		if (i < dot - 2) deg = deg * 10 + (uint32_t)(s[i] - '0');
		else minutes = minutes * 10 + (uint32_t)(s[i] - '0');
	}
	// Minutes' fraction, to 7 places (u-blox sends 5, or 7 in its
	// high precision mode); further places are dropped.
	uint32_t frac = 0;
	size_t places = 0;
	for (size_t i = dot + 1; i < len; ++i) {
		if (s[i] < '0' || s[i] > '9') return false;
		if (places < 7) { frac = frac * 10 + (uint32_t)(s[i] - '0'); ++places; }
	}
	while (places < 7) { frac *= 10; ++places; }
	if (minutes >= 60 || deg > 180) return false;

	// degrees * 1e7 + minutes * 1e7 / 60, rounded to the nearest unit.
	const int64_t minutesE7 = (int64_t)minutes * 10000000 + frac;
	const int64_t v = (int64_t)deg * 10000000 + (minutesE7 + 30) / 60;
	if (v > 1800000000) return false;
	*e7 = (int32_t)v;
	return true;
}

// hhmmss or hhmmss.sss in field i.
inline bool NmeaParser::parseTime(uint8_t i) {
	const char* s; size_t n;
	field(i, &s, &n);
	if (n == 0) { _data.timeValid = false; return true; }   // no time yet: not an error
	if (n < 6) return false;
	uint32_t h, m, sec;
	if (!parseUint(s, 2, &h) || !parseUint(s + 2, 2, &m) || !parseUint(s + 4, 2, &sec)) return false;
	if (h > 23 || m > 59 || sec > 60) return false;   // 60: a leap second
	uint32_t ms = 0;
	if (n > 6) {
		if (s[6] != '.') return false;
		uint32_t scale = 100;
		for (size_t k = 7; k < n; ++k) {
			if (s[k] < '0' || s[k] > '9') return false;
			ms += (uint32_t)(s[k] - '0') * scale;
			scale /= 10;
		}
	}
	_data.hour = (uint8_t)h;
	_data.minute = (uint8_t)m;
	_data.second = (uint8_t)sec;
	_data.millisecond = (uint16_t)ms;
	_data.timeValid = true;
	return true;
}

// Fields i..i+3: latitude, N/S, longitude, E/W. All empty is fine (no
// fix): the last position is kept. Anything else malformed is not.
inline bool NmeaParser::parseLatLon(uint8_t i) {
	const char *lat, *ns, *lon, *ew; size_t nlat, nns, nlon, new_;
	field(i, &lat, &nlat); field(i + 1, &ns, &nns);
	field(i + 2, &lon, &nlon); field(i + 3, &ew, &new_);
	if (nlat == 0 && nlon == 0) return true;
	int32_t la, lo;
	if (!parseCoordinate(lat, nlat, &la) || !parseCoordinate(lon, nlon, &lo)) return false;
	if (nns != 1 || (ns[0] != 'N' && ns[0] != 'S')) return false;
	if (new_ != 1 || (ew[0] != 'E' && ew[0] != 'W')) return false;
	if (la > 900000000) return false;
	_data.latitudeE7 = ns[0] == 'S' ? -la : la;
	_data.longitudeE7 = ew[0] == 'W' ? -lo : lo;
	return true;
}

static inline float nmeaFloatField(const char* s, size_t n) {
	float v;
	NmeaParser::parseFloat(s, n, &v);   // NAN if empty
	return v;
}

// ---- sentences ----

// GGA: time, lat, N/S, lon, E/W, quality, satellites, HDOP, altitude, M,
// geoid separation, M, [age of differential, station]
inline bool NmeaParser::doGGA() {
	if (_fieldCount < 13) return false;
	if (!parseTime(1) || !parseLatLon(2)) return false;
	const char* s; size_t n;
	uint32_t q = 0, sats = 0;
	field(6, &s, &n);
	if (n && !parseUint(s, n, &q)) return false;
	field(7, &s, &n);
	if (n && !parseUint(s, n, &sats)) return false;
	_data.fixQuality = (uint8_t)q;
	_data.positionValid = q > 0;
	_data.satellitesUsed = (uint8_t)(sats > 255 ? 255 : sats);
	field(8, &s, &n);  _data.hdop = nmeaFloatField(s, n);
	field(9, &s, &n);  _data.altitudeM = nmeaFloatField(s, n);
	field(11, &s, &n); _data.geoidSeparationM = nmeaFloatField(s, n);
	return true;
}

static inline bool nmeaModeIndicator(const char* s, size_t n, char* out) {
	if (n == 0) return true;
	if (n != 1 || s[0] < 'A' || s[0] > 'Z') return false;
	*out = s[0];
	return true;
}

// RMC: time, status, lat, N/S, lon, E/W, speed (knots), course, date,
// magnetic variation, E/W, [mode], [navigational status]
inline bool NmeaParser::doRMC() {
	if (_fieldCount < 12) return false;
	const char* s; size_t n;
	field(2, &s, &n);
	if (n != 1 || (s[0] != 'A' && s[0] != 'V')) return false;
	if (!parseTime(1) || !parseLatLon(3)) return false;
	_data.positionValid = s[0] == 'A';

	float knots;
	field(7, &s, &n);
	_data.speedMps = parseFloat(s, n, &knots) ? knots * 0.514444f : NAN;
	field(8, &s, &n);  _data.courseDeg = nmeaFloatField(s, n);

	field(9, &s, &n);
	if (n == 6) {
		uint32_t d, m, y;
		if (!parseUint(s, 2, &d) || !parseUint(s + 2, 2, &m) || !parseUint(s + 4, 2, &y)) return false;
		if (d < 1 || d > 31 || m < 1 || m > 12) return false;
		_data.day = (uint8_t)d;
		_data.month = (uint8_t)m;
		_data.year = (uint16_t)(y < 80 ? 2000 + y : 1900 + y);
		_data.dateValid = true;
	} else if (n == 0) {
		_data.dateValid = false;
	} else {
		return false;
	}

	field(12, &s, &n);
	return nmeaModeIndicator(s, n, &_data.modeIndicator);
}

// GLL: lat, N/S, lon, E/W, time, status, [mode]
inline bool NmeaParser::doGLL() {
	if (_fieldCount < 7) return false;
	const char* s; size_t n;
	field(6, &s, &n);
	if (n != 1 || (s[0] != 'A' && s[0] != 'V')) return false;
	if (!parseLatLon(1) || !parseTime(5)) return false;
	_data.positionValid = s[0] == 'A';
	field(7, &s, &n);
	return nmeaModeIndicator(s, n, &_data.modeIndicator);
}

// VTG: course true, T, course magnetic, M, speed knots, N, speed km/h, K, [mode]
inline bool NmeaParser::doVTG() {
	if (_fieldCount < 9) return false;
	const char* s; size_t n;
	field(1, &s, &n);  _data.courseDeg = nmeaFloatField(s, n);
	float kph, knots;
	field(7, &s, &n);
	if (parseFloat(s, n, &kph)) {
		_data.speedMps = kph / 3.6f;
	} else {
		field(5, &s, &n);
		_data.speedMps = parseFloat(s, n, &knots) ? knots * 0.514444f : NAN;
	}
	field(9, &s, &n);
	return nmeaModeIndicator(s, n, &_data.modeIndicator);
}

// GSA: mode M/A, fix type 1/2/3, 12 satellite numbers, PDOP, HDOP, VDOP, [system ID]
inline bool NmeaParser::doGSA() {
	if (_fieldCount < 18) return false;
	const char* s; size_t n;
	uint32_t type = 0;
	field(2, &s, &n);
	if (!parseUint(s, n, &type) || type < 1 || type > 3) return false;
	_data.fixType = (uint8_t)type;
	field(15, &s, &n); _data.pdop = nmeaFloatField(s, n);
	field(16, &s, &n); _data.hdop = nmeaFloatField(s, n);
	field(17, &s, &n); _data.vdop = nmeaFloatField(s, n);
	return true;
}

// GSV: number of messages, this message's number, satellites in view,
// then up to four of (number, elevation, azimuth, SNR), [signal ID].
// A system's (and signal's) first message replaces its satellites.
inline bool NmeaParser::doGSV() {
	if (_fieldCount < 4) return false;
	const char* s; size_t n;
	uint32_t total, num, inView;
	field(1, &s, &n); if (!parseUint(s, n, &total)) return false;
	field(2, &s, &n); if (!parseUint(s, n, &num)) return false;
	field(3, &s, &n); if (!parseUint(s, n, &inView)) return false;
	if (num < 1 || num > total) return false;

	const uint8_t after = (uint8_t)(_fieldCount - 4);
	const uint8_t groups = after / 4;
	if (after % 4 > 1 || groups > 4) return false;
	uint32_t signal = 0;
	if (after % 4 == 1) {
		field((uint8_t)(_fieldCount - 1), &s, &n);
		if (n && !parseUint(s, n, &signal)) return false;
	}

	// Every satellite in this message first, so a malformed one leaves
	// the list as it was.
	GnssSatellite got[4];
	uint8_t count = 0;
	for (uint8_t g = 0; g < groups; ++g) {
		const uint8_t f = (uint8_t)(4 + g * 4);
		uint32_t prn, v;
		field(f, &s, &n);
		if (n == 0) continue;   // an empty slot
		if (!parseUint(s, n, &prn) || prn > 255) return false;
		GnssSatellite& sat = got[count++];
		sat.system = _talker;
		sat.signalId = (uint8_t)signal;
		sat.prn = (uint8_t)prn;
		field(f + 1, &s, &n);
		if (n && (!parseUint(s, n, &v) || v > 90)) return false;
		sat.elevation = n ? (int8_t)v : (int8_t)-128;
		field(f + 2, &s, &n);
		if (n && (!parseUint(s, n, &v) || v > 359)) return false;
		sat.azimuth = n ? (int16_t)v : (int16_t)-1;
		field(f + 3, &s, &n);
		if (n && (!parseUint(s, n, &v) || v > 99)) return false;
		sat.snr = n ? (int8_t)v : (int8_t)-1;
	}

	if (num == 1) {   // this system's (and signal's) list starts again
		uint8_t keep = 0;
		for (uint8_t k = 0; k < _data.satelliteCount; ++k) {
			const GnssSatellite& sat = _data.satellites[k];
			if (sat.system == _talker && sat.signalId == signal) continue;
			_data.satellites[keep++] = sat;
		}
		_data.satelliteCount = keep;
	}

	for (uint8_t k = 0; k < count; ++k) {
		if (_data.satelliteCount < GnssData::kMaxSatellites) {
			_data.satellites[_data.satelliteCount++] = got[k];
		}
	}
	return true;
}

// ZDA: time, day, month, year, local zone hours, minutes
inline bool NmeaParser::doZDA() {
	if (_fieldCount < 5) return false;
	if (!parseTime(1)) return false;
	const char* s; size_t n;
	uint32_t d, m, y;
	field(2, &s, &n);
	if (n == 0) { _data.dateValid = false; return true; }
	if (!parseUint(s, n, &d)) return false;
	field(3, &s, &n); if (!parseUint(s, n, &m)) return false;
	field(4, &s, &n); if (!parseUint(s, n, &y)) return false;
	if (d < 1 || d > 31 || m < 1 || m > 12 || y < 1980 || y > 2200) return false;
	_data.day = (uint8_t)d;
	_data.month = (uint8_t)m;
	_data.year = (uint16_t)y;
	_data.dateValid = true;
	return true;
}

#endif /* NMEA_PARSER_TPP_ */
