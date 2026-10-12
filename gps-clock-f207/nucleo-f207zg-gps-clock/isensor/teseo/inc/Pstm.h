/*
 * Pstm.h
 *
 *  ST's $PSTM commands for the Teseo GNSS modules (Teseo III: the
 *  Teseo-LIV3F, and the Teseo-VIC3D/VIC3DA with dead reckoning; ST says
 *  the Teseo-LIV4F takes them too). Only what the teseo driver sends.
 *  Header-only, no heap, no printf.
 *
 *  Source: ST's X-CUBE-GNSS1 package (github.com/STMicroelectronics/
 *  x-cube-gnss1 at 0861193, April 2026: LibGNSS gnss_data.c, LibNMEA
 *  NMEA_parser.c and the GetPos application), which ST tests on its
 *  X-NUCLEO-GNSS1A1 (LIV3F) and X-NUCLEO-GNSS2A1 (VIC3DA) boards:
 *
 *    $PSTMGETSWVER         -> $PSTMVER,<version text>
 *    $PSTMGETPAR,<cdb>     -> $PSTMSETPAR,<cdb>,<value>   (the value's
 *                             format isn't shown there: see parseValue)
 *    $PSTMCFGMSGL,<list>,<rate>,<low>,<high>   (masks in hex, %x)
 *                          -> $PSTMCFGMSGLOK / $PSTMCFGMSGLERROR
 *    $PSTMSAVEPAR          -> $PSTMSAVEPAROK / $PSTMSAVEPARERROR
 *    $PSTMSRR              the firmware reboots (no answer)
 *
 *  Configuration block IDs are 4 digits: the block (1, 2 or 3, of which
 *  the GetPos application reads 1), then the 3-digit CDB-ID. CDB-ID 201
 *  is NMEA message list 0's low 32 bits, 228 its high 32 bits (ST's
 *  comments). ST sends commands as "$" body "\r\n", without a checksum,
 *  so this does too; sentence() can add one.
 *
 *  Message list bits (CDB-ID 201, from ST's comments): 0x1 GNS, 0x2 GGA,
 *  0x4 GSA, 0x8 GST, 0x40 RMC, 0x80000 GSV, 0x100000 GLL. ST's GetPos
 *  default is 0x18004F. In the high word (CDB-ID 228) ST's examples use
 *  bit 50-32 for odometer, 51-32 geofence and 52-32 datalog messages.
 *  Other bits (ZDA among them) are not in ST's code: not used here.
 */

#ifndef PSTM_H_
#define PSTM_H_

#include <stddef.h>
#include <stdint.h>

namespace pstm {

// CDB-ID 201's bits (message list 0, low word)
const uint32_t kMsgGNS = 0x1u;
const uint32_t kMsgGGA = 0x2u;
const uint32_t kMsgGSA = 0x4u;
const uint32_t kMsgGST = 0x8u;
const uint32_t kMsgRMC = 0x40u;
const uint32_t kMsgGSV = 0x80000u;
const uint32_t kMsgGLL = 0x100000u;

const uint16_t kCdbMsgListLow = 1201;    // block 1 (current), CDB-ID 201
const uint16_t kCdbMsgListHigh = 1228;   // block 1, CDB-ID 228

// "$" body, optionally "*hh", "\r\n". Returns the length, or 0 if it
// doesn't fit (out needs strlen(body) + 6).
inline size_t sentence(char* out, size_t cap, const char* body, bool checksum = false) {
	size_t n = 0;
	uint8_t x = 0;
	if (cap < 7) return 0;
	out[n++] = '$';
	for (const char* p = body; *p; ++p) {
		if (n + 6 > cap) return 0;
		out[n++] = *p;
		x ^= (uint8_t)*p;
	}
	if (checksum) {
		static const char hex[] = "0123456789ABCDEF";
		out[n++] = '*';
		out[n++] = hex[x >> 4];
		out[n++] = hex[x & 15];
	}
	out[n++] = '\r';
	out[n++] = '\n';
	out[n] = 0;
	return n;
}

// Builds a body a piece at a time, without printf.
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
	// Lower case, no leading zeros, as printf's %x
	Body& hex(uint32_t v) {
		static const char h[] = "0123456789abcdef";
		char d[8];
		int k = 0;
		do { d[k++] = h[v & 15]; v >>= 4; } while (v && k < 8);
		while (k > 0 && _n + 1 < sizeof _b) _b[_n++] = d[--k];
		_b[_n] = 0;
		return *this;
	}
	const char* str() const { return _b; }
private:
	char   _b[64];
	size_t _n;
};

inline size_t getSwVersion(char* out, size_t cap, bool checksum = false) {
	return sentence(out, cap, "PSTMGETSWVER", checksum);
}

inline size_t getPar(char* out, size_t cap, uint16_t cdb, bool checksum = false) {
	Body b;
	b.text("PSTMGETPAR,").number(cdb);
	return sentence(out, cap, b.str(), checksum);
}

// Message list `list` (0), every `rate` fixes (1), the two masks.
inline size_t cfgMsgList(char* out, size_t cap, uint8_t list, uint8_t rate, uint32_t low, uint32_t high,
                         bool checksum = false) {
	Body b;
	b.text("PSTMCFGMSGL,").number(list).text(",").number(rate).text(",").hex(low).text(",").hex(high);
	return sentence(out, cap, b.str(), checksum);
}

inline size_t savePar(char* out, size_t cap, bool checksum = false) {
	return sentence(out, cap, "PSTMSAVEPAR", checksum);
}

inline size_t systemReset(char* out, size_t cap, bool checksum = false) {
	return sentence(out, cap, "PSTMSRR", checksum);
}

// A $PSTMSETPAR value as `want` reads it: hex with or without "0x"
// (the masks are written that way), and failing that as decimal, since
// the reply's format isn't pinned down by ST's code. True if either
// reading equals want.
inline bool valueIs(const char* s, size_t n, uint32_t want) {
	if (n == 0) return false;
	size_t i = 0;
	if (n > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) i = 2;
	uint64_t h = 0;
	bool hexOk = i < n && n - i <= 8;
	for (size_t k = i; k < n && hexOk; ++k) {
		const char c = s[k];
		const int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10
		            : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
		if (d < 0) hexOk = false; else h = h * 16 + (uint64_t)d;
	}
	if (hexOk && h == want) return true;
	if (i != 0) return false;   // "0x...": hex only
	uint64_t v = 0;
	for (size_t k = 0; k < n; ++k) {
		if (s[k] < '0' || s[k] > '9' || v > 0xFFFFFFFFull) return false;
		v = v * 10 + (uint64_t)(s[k] - '0');
	}
	return v == want;
}

} // namespace pstm

#endif /* PSTM_H_ */
