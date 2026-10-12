/*
 * ds1307_ref.h
 *
 *  The DS1307 as a second, independent clock: its 1 Hz SQW/OUT output is
 *  latched by TIM2 (the same free-running 1 MHz counter as the GPS's PPS),
 *  so its phase and rate can be measured against the PPS, kept in its
 *  battery-backed RAM, and used after a power cycle to seed the STM32 RTC
 *  to within a millisecond or so instead of half a second. No HAL here, so
 *  it can be tested on a PC (test/ds1307_ref_test.cpp).
 *
 *  Assumption, not verified against the datasheet here (it could not be
 *  downloaded): the DS1307's seconds register increments on the falling
 *  edge of the 1 Hz square wave (general knowledge, from secondary
 *  sources). LabelCheck tests it on the board: reads made early and late
 *  in the second must name the same edge, or the SQW is not used.
 *
 *  Words used below:
 *  - edge: a falling edge of SQW, as a TIM2 count.
 *  - label: the DS1307's time (Unix seconds) from that edge to the next.
 *  - offset: DS1307 time minus UTC, in microseconds (the DS1307 ahead is
 *    positive). At an edge with label L and UTC u: L x 1e6 - u.
 */

#ifndef DS1307_REF_H_
#define DS1307_REF_H_

#include <stdint.h>

namespace dsref {

// ---- labelling an edge with a read ----

// Where a read's I2C START fell in the DS1307's second. The DS1307
// latches its counters on START (datasheet, through the driver's notes).
enum class ReadPhase : uint8_t {
	None,     // too near an edge, or not known closely enough: not used
	Early,    // in the first 400 ms after the edge
	Middle,
	Late      // 600 ms or more after it
};

const uint32_t kEdgeGuardUs = 10000;     // a START this close to an edge is not trusted either side
const uint32_t kStartSpreadUs = 20000;   // most uncertainty in when a START happened
const uint32_t kEarlyUs = 400000;
const uint32_t kLateUs = 600000;

// Two edges a second apart, give or take tolUs (crystals and TIM2's own clock).
inline bool edgeIntervalOk(uint32_t prevAt, uint32_t at, uint32_t tolUs = 2000) {
	const uint32_t d = at - prevAt;
	return d > 1000000u - tolUs && d < 1000000u + tolUs;
}

// A read whose START lies between timer counts startFrom and startTo,
// against the edge at edgeAt (the last one before it). Reads that land
// clearly inside one DS1307 second label that edge.
inline ReadPhase readPhase(uint32_t edgeAt, uint32_t startFrom, uint32_t startTo) {
	const uint32_t d1 = startFrom - edgeAt, d2 = startTo - edgeAt;
	if (d2 < d1 || d2 - d1 > kStartSpreadUs) return ReadPhase::None;
	if (d1 < kEdgeGuardUs || d2 > 1000000u - kEdgeGuardUs) return ReadPhase::None;
	if (d2 <= kEarlyUs) return ReadPhase::Early;
	if (d1 >= kLateUs) return ReadPhase::Late;
	return ReadPhase::Middle;
}

// Checks the falling-edge assumption. Feeds it label - edgeNumber (the
// capture interrupt counts edges): constant while the DS1307 runs, as long
// as reads early and late in the second agree on which edge started it.
// If the seconds ticked over at the rising edge instead (half a second
// later) the early reads would be one less.
class LabelCheck {
public:
	static const uint8_t kNeeded = 2;    // agreeing reads of each kind

	void restart() { early_ = late_ = 0; mismatch_ = false; }

	void add(ReadPhase ph, int64_t labelMinusEdge) {
		if (ph != ReadPhase::Early && ph != ReadPhase::Late) return;
		if (early_ == 0 && late_ == 0) value_ = labelMinusEdge;
		else if (labelMinusEdge != value_) { mismatch_ = true; return; }
		if (ph == ReadPhase::Early) { if (early_ < 255) ++early_; }
		else if (late_ < 255) ++late_;
	}

	bool verified() const { return !mismatch_ && early_ >= kNeeded && late_ >= kNeeded; }
	bool mismatch() const { return mismatch_; }
	// The kind of read still wanted: alternate them until verified.
	ReadPhase wanted() const { return early_ <= late_ ? ReadPhase::Early : ReadPhase::Late; }

private:
	int64_t value_ = 0;
	uint8_t early_ = 0, late_ = 0;
	bool mismatch_ = false;
};

// ---- the DS1307 measured against the PPS ----

// Kept in the DS1307's battery-backed RAM.
struct Record {
	int64_t offsetUs = 0;    // DS1307 minus UTC at refUnix
	uint32_t refUnix = 0;    // UTC second of that measurement
	int32_t driftPpb = 0;    // the DS1307's rate error, fast positive
	uint32_t spanS = 0;      // what driftPpb was measured over; 0: not measured
};

const uint8_t kRecordLen = 25;       // bytes at RAM offset 0
const uint8_t kRecordVersion = 1;

// CRC-8, polynomial 0x07, initial 0 (CRC-8/SMBUS)
inline uint8_t crc8(const uint8_t *p, uint8_t n) {
	uint8_t c = 0;
	while (n--) {
		c ^= *p++;
		for (int i = 0; i < 8; ++i) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x07 : c << 1);
	}
	return c;
}

// 'D' 'R' version 0, offset (8), refUnix (4), drift (4), span (4), CRC.
// Little-endian, like everything this project puts on a wire.
inline void encode(const Record &r, uint8_t *out) {
	out[0] = 'D'; out[1] = 'R'; out[2] = kRecordVersion; out[3] = 0;
	const uint64_t o = (uint64_t)r.offsetUs;
	for (int i = 0; i < 8; ++i) out[4 + i] = (uint8_t)(o >> (8 * i));
	const uint32_t w[3] = {r.refUnix, (uint32_t)r.driftPpb, r.spanS};
	for (int k = 0; k < 3; ++k)
		for (int i = 0; i < 4; ++i) out[12 + 4 * k + i] = (uint8_t)(w[k] >> (8 * i));
	out[24] = crc8(out, 24);
}

// false: not a record (a new chip's RAM is undefined).
inline bool decode(const uint8_t *in, Record *r) {
	if (in[0] != 'D' || in[1] != 'R' || in[2] != kRecordVersion || crc8(in, 24) != in[24]) return false;
	uint64_t o = 0;
	for (int i = 7; i >= 0; --i) o = (o << 8) | in[4 + i];
	uint32_t w[3];
	for (int k = 0; k < 3; ++k) {
		w[k] = 0;
		for (int i = 3; i >= 0; --i) w[k] = (w[k] << 8) | in[12 + 4 * k + i];
	}
	r->offsetUs = (int64_t)o;
	r->refUnix = w[0];
	r->driftPpb = (int32_t)w[1];
	r->spanS = w[2];
	return true;
}

// The DS1307's offset at UTC second unixSec, from a record (its drift
// carried on from refUnix). ppb x seconds / 1000 = microseconds.
// (For the UTC of a DS1307 edge use utcAtEdgeUs(), which counts in the
// DS1307's own seconds.)
inline int64_t predictOffsetUs(const Record &r, uint32_t unixSec) {
	const int64_t dt = (int64_t)(int32_t)(unixSec - r.refUnix);
	return r.offsetUs + (int64_t)r.driftPpb * dt / 1000;
}

// Measures the DS1307's offset and drift, one sample per labelled edge
// whose UTC (microseconds) is known from the PPS. The drift is the slope
// from the first sample to the latest, once they are kMinSpanS apart. A
// sample too far off the line (the DS1307 was written, or a label was
// wrong) starts over.
class Meter {
public:
	static const uint32_t kMinSpanS = 1024;
	static const int64_t kJumpUs = 5000;          // plus the drift's allowance below
	static const int64_t kUnknownDriftPpm = 200;  // allowed before the drift is measured
	static const int64_t kKnownDriftPpm = 2;      // and after

	void restart() { n_ = 0; haveDrift_ = false; }

	// false: the sample didn't fit; the measurement starts again from it.
	bool add(int64_t utcUs, int64_t offsetUs) {
		bool fit = true;
		if (n_ != 0) {
			const int64_t span = utcUs - utc0_;          // us
			const int64_t pred = haveDrift_ ? off0_ + (int64_t)drift_ * span / 1000000000 : off0_;
			const int64_t d = offsetUs - pred, mag = d < 0 ? -d : d;
			const int64_t allow = kJumpUs + (haveDrift_ ? kKnownDriftPpm : kUnknownDriftPpm) * (span < 0 ? -span : span) / 1000000;
			if (span <= 0 || mag > allow) {
				fit = false;
				++jumps_;
				restart();
			}
		}
		if (n_ == 0) { utc0_ = utcUs; off0_ = offsetUs; }
		if (n_ < 0xFFFFFFFFu) ++n_;
		utc_ = utcUs;
		off_ = offsetUs;
		const int64_t span = utcUs - utc0_;
		if (span >= (int64_t)kMinSpanS * 1000000) {
			const int64_t num = (offsetUs - off0_) * 1000000000;   // rounded to the ppb
			drift_ = (int32_t)((num + (num >= 0 ? span / 2 : -span / 2)) / span);
			haveDrift_ = true;
		}
		return fit;
	}

	bool have() const { return n_ != 0; }
	bool haveDrift() const { return haveDrift_; }
	int64_t offsetUs() const { return off_; }      // the latest sample's
	int32_t driftPpb() const { return drift_; }
	uint32_t spanS() const { return n_ ? (uint32_t)((utc_ - utc0_) / 1000000) : 0; }
	uint32_t jumps() const { return jumps_; }

	// What to keep: the offset at the whole UTC second of the latest
	// sample, with this run's drift once measured (else the kept one).
	Record record(const Record &before) const {
		Record r = before;
		if (haveDrift_) { r.driftPpb = drift_; r.spanS = spanS(); }
		r.refUnix = (uint32_t)(utc_ / 1000000);
		const int64_t frac = utc_ - (int64_t)r.refUnix * 1000000;
		r.offsetUs = off_ - (int64_t)r.driftPpb * frac / 1000000000;
		return r;
	}

private:
	uint32_t n_ = 0, jumps_ = 0;
	int64_t utc0_ = 0, utc_ = 0, off0_ = 0, off_ = 0;
	int32_t drift_ = 0;
	bool haveDrift_ = false;
};

// Do the two oscillators still agree? measuredUs: the DS1307's offset
// measured now against the STM32 RTC (no PPS); predictedUs: from the
// record; ageS since the record's refUnix. Each clock is allowed 1 ppm
// of unmeasured error on top of a few milliseconds.
const int64_t kAgreeBaseUs = 5000;
const int64_t kAgreePpm = 2;
inline bool agrees(int64_t measuredUs, int64_t predictedUs, uint32_t ageS) {
	const int64_t d = measuredUs - predictedUs;
	return (d < 0 ? -d : d) <= kAgreeBaseUs + kAgreePpm * (int64_t)ageS;
}

// UTC (microseconds since 1970) at an edge with label L, from a record.
// The offset grows by driftPpb per UTC second, and L counts DS1307
// seconds: with x the UTC time since refUnix and n = L - refUnix - offset,
// x + D x = n, so x = n - n D / (1e9 + D) (n D fits in 64 bits for years).
inline int64_t utcAtEdgeUs(const Record &r, uint32_t label) {
	const int64_t n = ((int64_t)label - (int64_t)r.refUnix) * 1000000 - r.offsetUs;
	const int64_t x = n - n * r.driftPpb / (1000000000 + (int64_t)r.driftPpb);
	return (int64_t)r.refUnix * 1000000 + x;
}

} // namespace dsref

#endif /* DS1307_REF_H_ */
