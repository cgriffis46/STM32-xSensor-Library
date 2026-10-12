/*
 * ptp_math.h
 *
 *  The arithmetic behind the clock on the F207: the Ethernet MAC's IEEE
 *  1588 system time, disciplined to the GPS's PPS. No HAL here, so it is
 *  tested on a PC (test/ptp_math_test.cpp).
 *
 *  The system time (RM0033's "IEEE 1588 time stamp" section, as far as it
 *  is used here; register and bit names checked in ST's stm32f207xx.h, how
 *  they behave from general knowledge of this MAC, the same one as the
 *  F407's, and ST's AN3411 code):
 *  - ETH_PTPTSHR holds seconds, ETH_PTPTSLR sub-seconds. With TSSSR clear
 *    (binary rollover: ETH_PTPTSCR bit 9, which ST's header names
 *    ETH_PTPTSSR_TSSSR; 0 after reset, and left so) the sub-seconds count
 *    2^31 to the second: units of 2^-31 s, half an NTP fraction step.
 *  - Fine update (TSFCU set): every HCLK cycle ETH_PTPTSAR (the addend) is
 *    added to a 32-bit accumulator; each carry out adds ETH_PTPSSIR (the
 *    increment) to the sub-seconds. The clock therefore advances
 *    increment x addend x HCLK / 2^63 seconds a second: the addend sets
 *    its rate, to a part in about 2^31 (0.56 ppb at 120 MHz).
 *  - TSSTI loads the time from ETH_PTPTSHUR/ETH_PTPTSLUR.
 *
 *  Phase is corrected by slewing (the rate), not by stepping: an NTP
 *  client sees no jumps once locked, and the "subtract" form of TSSTU,
 *  which sources describe differently, is never used. Errors of a
 *  millisecond or more are set outright with TSSTI.
 *
 *  Times on the PPS side are counts of TIM2, a free-running 32-bit 1 MHz
 *  timer (ppsmath::kTimerHz); unsigned differences are right across its
 *  wrap.
 */

#ifndef PTP_MATH_H_
#define PTP_MATH_H_

#include <stdint.h>

namespace ptpmath {

const uint64_t kSubPerSecond = 0x80000000ull;   // binary rollover: 2^31
const uint32_t kTimerHz = 1000000;               // TIM2

// The PPS edge starts a UTC second, and the RMC sentence that follows names
// it (general knowledge for the Teseo, as for the MTK modules; not checked
// against ST's documents here). So an RMC parsed at timer count rmcAt
// belongs to the PPS at ppsAt if it came after it, within a second. (These
// three are the F405 example's pps_math.h and rtc_edge.h.)
inline bool rmcFollowsPps(uint32_t ppsAt, uint32_t rmcAt) {
	const uint32_t d = rmcAt - ppsAt;
	return d > 0 && d < kTimerHz;
}

// Two PPS edges one second apart, give or take tolUs (TIM2 runs from the
// HSE: 2000 us allows a crystal far worse than any real one).
inline bool ppsIntervalOk(uint32_t prevAt, uint32_t at, uint32_t tolUs = 2000) {
	const uint32_t d = at - prevAt;
	return d > kTimerHz - tolUs && d < kTimerHz + tolUs;
}

// A 16-bit capture from a timer counting in step with a 32-bit one (same
// rate, started together), widened with the 32-bit count read after it:
// right while that read is less than 65.5 ms late.
inline uint32_t widen(uint16_t capture, uint32_t now32) {
	return now32 - (uint16_t)((uint16_t)now32 - capture);
}

// Sub-seconds (2^-31 s) <-> NTP's fraction (2^-32 s) <-> microseconds.
inline uint32_t fracFromSub(uint32_t sub) { return (sub & 0x7FFFFFFFu) << 1; }
inline uint32_t subFromFrac(uint32_t frac) { return frac >> 1; }
inline uint32_t subFromUs(uint32_t us) {        // us below 1000000
	return (uint32_t)((((uint64_t)us << 31) + 500000) / 1000000);
}
inline uint32_t usFromSub(uint32_t sub) {
	return (uint32_t)(((uint64_t)(sub & 0x7FFFFFFFu) * 1000000 + 0x40000000u) >> 31);
}

// The increment and addend that make the clock run at its nominal rate.
// updateHz: how often the sub-seconds should step (AN3411 uses 50 MHz, so
// steps of about 20 ns); it must be below hclkHz. false: impossible.
struct Base {
	uint8_t increment;
	uint32_t addend;
};
inline bool baseFor(uint32_t hclkHz, uint32_t updateHz, Base *out) {
	if (hclkHz == 0 || updateHz == 0 || updateHz >= hclkHz) return false;
	const uint64_t inc = (kSubPerSecond + updateHz / 2) / updateHz;
	if (inc == 0 || inc > 255) return false;
	// addend = 2^63 / (increment x HCLK), rounded
	const uint64_t den = inc * (uint64_t)hclkHz;
	const uint64_t add = ((1ull << 63) + den / 2) / den;
	if (add == 0 || add > 0xFFFFFFFFull) return false;
	out->increment = (uint8_t)inc;
	out->addend = (uint32_t)add;
	return true;
}

// The addend for running ppb parts per billion fast (negative: slow).
inline uint32_t addendFor(const Base &b, int32_t ppb) {
	const int64_t num = (int64_t)b.addend * (1000000000LL + ppb);
	int64_t a = (num + 500000000LL) / 1000000000LL;
	if (a < 1) a = 1;
	if (a > 0xFFFFFFFFLL) a = 0xFFFFFFFFLL;
	return (uint32_t)a;
}

// What an addend really does, in ppb against the base.
inline int32_t ppbOf(const Base &b, uint32_t addend) {
	const int64_t d = (int64_t)addend - (int64_t)b.addend;
	const int64_t num = d * 1000000000LL;
	return (int32_t)((num + (num >= 0 ? (int64_t)b.addend / 2 : -(int64_t)b.addend / 2)) / (int64_t)b.addend);
}

// The clock's error in microseconds, positive when it is ahead: a reading
// (Unix seconds, 2^-32 fraction) taken at timer count readAt, against the
// PPS at ppsAt, which started UTC second ppsSec.
inline int64_t errorUs(uint32_t sec, uint32_t frac, uint32_t readAt, uint32_t ppsSec, uint32_t ppsAt) {
	const int64_t clockUs = (int64_t)sec * 1000000 + (int64_t)(((uint64_t)frac * 1000000u + 0x80000000u) >> 32);
	const int64_t trueUs = (int64_t)ppsSec * 1000000 + (int64_t)(uint32_t)(readAt - ppsAt);
	return clockUs - trueUs;
}

// The control loop: a proportional-integral loop on the phase error, one
// call per PPS second, its output the rate to run at (ppb).
//
//   frequency += -error x 1000 / 256     ppb per second, the integral
//   rate       = frequency - error x 1000 / 8
//
// (error in us; an error of e us a second is e x 1000 ppb). These gains
// make the loop critically damped (one double pole at 15/16 a second,
// a time constant of about 16 s), with no overshoot after a set. The PPS
// is timed by a 1 MHz counter, so an error is good to a microsecond or
// so; the integral smooths that to a few ppb.
class Discipline {
public:
	struct Step {
		bool set = false;      // set the clock from the PPS: the error is too big to slew
		int32_t ppb = 0;       // the rate to run at from now
	};

	static const int64_t kSetUs = 1000;           // at or above: set, don't slew
	static const int32_t kMaxFreqPpb = 200000;    // the crystal's error, at most (200 ppm)
	static const int32_t kMaxSlewPpb = 500000;    // frequency plus phase term
	static const int64_t kLockUs = 10;            // locked: within this ...
	static const uint8_t kLockSeconds = 8;        // ... this many seconds running
	static const int64_t kUnlockUs = 100;         // and unlocked again beyond this
	static const uint32_t kMaxGapSeconds = 4;     // more between updates: start over

	// freqPpb: the last frequency correction (kept across a reset)
	explicit Discipline(int32_t freqPpb = 0) { setFrequency(freqPpb); }

	// PPS lost or the clock set another way: forget the phase history but
	// keep the frequency (holdover runs at it).
	void restart() { have_ = false; locked_ = false; good_ = 0; }

	bool locked() const { return locked_; }
	int32_t frequencyPpb() const { return (int32_t)((freqQ_ + (freqQ_ >= 0 ? 128 : -128)) / 256); }
	// The rate to run at without a PPS
	int32_t holdoverPpb() const { return frequencyPpb(); }

	Step update(uint32_t sec, int64_t errUs) {
		Step st;
		const int64_t mag = errUs < 0 ? -errUs : errUs;
		if (have_ && sec - lastSec_ > kMaxGapSeconds)
			restart();
		if (mag >= kSetUs) {
			st.set = true;
			st.ppb = frequencyPpb();
			have_ = false;
			locked_ = false;
			good_ = 0;
			return st;
		}
		have_ = true;
		lastSec_ = sec;
		// integral: 1000/256 ppb per us, kept in 1/256 ppb
		freqQ_ -= errUs * 1000;
		clampFreq();
		int64_t rate = (int64_t)frequencyPpb() - errUs * 125;
		if (rate > kMaxSlewPpb) rate = kMaxSlewPpb;
		if (rate < -kMaxSlewPpb) rate = -kMaxSlewPpb;
		st.ppb = (int32_t)rate;
		if (mag < kLockUs) {
			if (good_ < kLockSeconds) ++good_;
		} else {
			good_ = 0;
		}
		if (good_ >= kLockSeconds)
			locked_ = true;
		else if (mag >= kUnlockUs)
			locked_ = false;
		return st;
	}

private:
	void setFrequency(int32_t ppb) {
		freqQ_ = (int64_t)ppb * 256;
		clampFreq();
	}
	void clampFreq() {
		const int64_t m = (int64_t)kMaxFreqPpb * 256;
		if (freqQ_ > m) freqQ_ = m;
		if (freqQ_ < -m) freqQ_ = -m;
	}

	bool have_ = false;
	bool locked_ = false;
	uint8_t good_ = 0;
	uint32_t lastSec_ = 0;
	int64_t freqQ_ = 0;      // ppb x 256
};

} // namespace ptpmath

#endif /* PTP_MATH_H_ */
