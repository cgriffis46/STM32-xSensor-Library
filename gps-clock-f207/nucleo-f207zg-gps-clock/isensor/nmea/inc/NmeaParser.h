/*
 * NmeaParser.h
 *
 *  An NMEA 0183 parser for GNSS receivers, fed one character at a
 *  time. Pure logic: no transport, HAL or RTOS, no heap, no strtod.
 *  Used by the GNSS drivers (ublox_gps, mtk3339), and works on any
 *  receiver's NMEA.
 *
 *      NmeaParser nmea;
 *      for each byte b:  if (nmea.feed(b) == NmeaParser::Sentence::GGA) ...
 *      const GnssData& d = nmea.data();
 *
 *  Sentences understood: GGA, RMC, GLL, VTG, GSA, GSV, ZDA, from any
 *  talker (GP GPS, GL GLONASS, GA Galileo, GB/BD BeiDou, GQ QZSS, GI
 *  NavIC, GN combined). The extra fields of NMEA 4.10 and 4.11 (RMC's
 *  navigational status, GSA's system ID, GSV's signal ID) are read when
 *  present. Others (TXT, GNS, GST, and proprietary ones such as $PMTK,
 *  $PGTOP, $PUBX) are returned as Other: a driver reads its own through
 *  address() and field().
 *
 *  A sentence counts only if it is whole and its checksum matches:
 *  "$" or "!", the fields, "*hh". At most kMaxSentence characters:
 *  NMEA 0183 allows 82 with the CR LF, but u-blox's high precision mode
 *  (7 decimal places of minutes) goes over that, so a little more is
 *  allowed.
 *  A sentence with no checksum is rejected (u-blox always sends one).
 *  A "$" in the middle starts again, so a lost character costs only
 *  the sentence it was in.
 *
 *  Position is kept as integers in 1e-7 degrees (the unit u-blox uses
 *  in UBX), converted from ddmm.mmmmmmm with integer arithmetic, so no
 *  precision is lost to float and no double is needed. Everything a
 *  receiver leaves empty reads as NAN (floats) or invalid.
 *
 *  Values are taken over as each sentence arrives, so data() can mix
 *  epochs for a moment (a GGA from this second, the GSA from the last).
 *  Each value's own sentence is consistent in itself, and a sentence
 *  is taken over whole or not at all: one with a malformed field
 *  changes nothing.
 */

#ifndef NMEA_PARSER_H_
#define NMEA_PARSER_H_

#include <stddef.h>
#include <stdint.h>

// One satellite from GSV.
struct GnssSatellite {
	char    system;     // talker's second letter: P GPS, L GLONASS, A Galileo, B BeiDou, Q QZSS, I NavIC
	uint8_t signalId;   // NMEA 4.10 signal ID, 0 if not sent
	uint8_t prn;        // satellite number as the receiver numbers it
	int8_t  elevation;  // degrees, -128 if not known
	int16_t azimuth;    // degrees, -1 if not known
	int8_t  snr;        // C/N0 in dB-Hz, -1 if not tracked
};

struct GnssData {
	static const uint8_t kMaxSatellites = 40;

	// UTC, from GGA, RMC, GLL or ZDA.
	bool     timeValid;
	uint8_t  hour, minute, second;
	uint16_t millisecond;
	// UTC date, from RMC or ZDA.
	bool     dateValid;
	uint16_t year;
	uint8_t  month, day;

	// Position. positionValid: the receiver says it has a fix (GGA
	// quality > 0, or RMC/GLL status A). Values stay as last reported.
	bool     positionValid;
	int32_t  latitudeE7, longitudeE7;   // 1e-7 degrees, north and east positive
	float    altitudeM;                 // above mean sea level (GGA)
	float    geoidSeparationM;          // geoid above the WGS84 ellipsoid (GGA)

	uint8_t  fixQuality;   // GGA: 0 none, 1 GNSS, 2 DGNSS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
	uint8_t  fixType;      // GSA: 1 none, 2 2D, 3 3D; 0 until one is seen
	char     modeIndicator;// RMC/GLL/VTG: A autonomous, D differential, E estimated, N none...; 0 if not sent
	uint8_t  satellitesUsed;   // GGA

	float    speedMps;     // over ground (RMC or VTG), m/s
	float    courseDeg;    // over ground, true (RMC or VTG)
	float    hdop, pdop, vdop;

	// Satellites in view (GSV), every system together.
	uint8_t       satelliteCount;
	GnssSatellite satellites[kMaxSatellites];
};

class NmeaParser {
public:
	enum class Sentence : uint8_t {
		None,        // nothing finished yet with this character
		GGA, RMC, GLL, VTG, GSA, GSV, ZDA,
		Other,       // a good sentence of a type not used here
		Bad          // a sentence ended but was rejected (see stats())
	};

	struct Stats {
		uint32_t sentences;       // good ones, of any type
		uint32_t checksumErrors;
		uint32_t malformed;       // too long, no checksum, bad characters, fields that don't parse
	};

	static const size_t kMaxSentence = 96;   // "$" to the end of the checksum

	NmeaParser() { reset(); }

	// Clears everything: the data back to "nothing known", and the
	// sentence being assembled.
	void reset();

	// The next character. Returns what, if anything, it finished.
	Sentence feed(uint8_t c);

	// True between a "$" and the end of that sentence.
	bool inSentence() const { return _len > 0; }

	const GnssData& data() const { return _data; }
	const Stats&    stats() const { return _stats; }

	// The sentence feed() last returned as finished and good (any type,
	// Other included): its address ("GPGGA", "PMTK001") and its fields
	// (1 is the first after the address). Valid until the next "$"
	// arrives. field() is false, with an empty string, past the end.
	void address(const char** s, size_t* len) const { field(0, s, len); }
	uint8_t fieldCount() const { return _fieldCount; }
	bool field(uint8_t i, const char** s, size_t* len) const;

	// For a caller that knows the data is stale (a receiver gone
	// quiet): back to "nothing known", counters kept.
	void invalidate();

	// Building blocks, public for tests.
	static uint8_t checksum(const char* body, size_t len);   // XOR of the characters between "$" and "*"
	// ddmm.mmmmm (or dddmm.mmmmm) to 1e-7 degrees; false if malformed.
	static bool parseCoordinate(const char* s, size_t len, int32_t* e7);
	// A decimal number to float; false (and NAN) if empty or malformed.
	static bool parseFloat(const char* s, size_t len, float* out);
	// An unsigned integer; false if empty or malformed.
	static bool parseUint(const char* s, size_t len, uint32_t* out);

private:
	static const uint8_t kMaxFields = 24;

	Sentence finish();
	Sentence dispatch();
	bool parseTime(uint8_t i);
	bool parseLatLon(uint8_t i);   // fields i..i+3: lat, N/S, lon, E/W

	bool doGGA();
	bool doRMC();
	bool doGLL();
	bool doVTG();
	bool doGSA();
	bool doGSV();
	bool doZDA();

	char    _buf[kMaxSentence + 4];
	size_t  _len;           // characters in _buf, from the "$"
	bool    _overflow;
	uint8_t _fieldStart[kMaxFields + 1];
	uint8_t _fieldCount;
	char    _talker;        // second letter of the talker ID

	GnssData _data;
	Stats    _stats;
};

#include "../src/NmeaParser.tpp"

#endif /* NMEA_PARSER_H_ */
