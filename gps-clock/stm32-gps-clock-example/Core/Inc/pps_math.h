/*
 * pps_math.h
 *
 *  The arithmetic behind disciplining the STM32 RTC to the GPS's PPS, with
 *  no HAL, so it can be tested on a PC (test/pps_math_test.cpp).
 *
 *  Times on the PPS side are counts of a free-running 32-bit 1 MHz timer
 *  (TIM2), which latches the PPS edge in hardware (input capture). Unsigned
 *  differences of counts are right across its wrap (every 71 minutes).
 *
 *  RTC_SHIFTR and RTC_CALR as RM0394 describes them:
 *  - A shift with SUBFS = s delays the clock by s / (PREDIV_S + 1) seconds;
 *    with ADD1S as well it advances it by 1 - s / (PREDIV_S + 1).
 *  - Smooth calibration over a 32 s cycle: the RTC runs at
 *    F x (1 + (512 CALP - CALM) / (2^20 + CALM - 512 CALP)); CALM 0..511
 *    masks pulses (-0.954 ppm each), CALP adds 512 (+488.5 ppm).
 */

#ifndef PPS_MATH_H_
#define PPS_MATH_H_

#include <stdint.h>

namespace ppsmath {

const uint32_t kTimerHz = 1000000;

// The PPS edge starts a UTC second, and the RMC sentence that follows names
// it (MTK modules: general knowledge, not checked against a datasheet here).
// So an RMC parsed at timer count rmcAt belongs to the PPS at ppsAt if it
// came after it, within a second.
inline bool rmcFollowsPps(uint32_t ppsAt, uint32_t rmcAt) {
	const uint32_t d = rmcAt - ppsAt;
	return d > 0 && d < kTimerHz;
}

// Two PPS edges one second apart, give or take tolUs (the timer's own clock
// error: the 80 MHz MSI is trimmed by the LSE to about 0.1 %; 2000 us allows it).
inline bool ppsIntervalOk(uint32_t prevAt, uint32_t at, uint32_t tolUs = 2000) {
	const uint32_t d = at - prevAt;
	return d > kTimerHz - tolUs && d < kTimerHz + tolUs;
}

// The RTC's error in microseconds, positive when it is ahead: an RTC
// reading (Unix seconds and a 2^-32 fraction) taken at timer count readAt,
// against the PPS at ppsAt, which started UTC second ppsSec. The timer
// ticks between the two are taken at its nominal rate.
inline int64_t rtcErrorUs(uint32_t rtcSec, uint32_t rtcFrac, uint32_t readAt, uint32_t ppsSec, uint32_t ppsAt) {
	const int64_t rtcUs = (int64_t)rtcSec * 1000000 + (int64_t)(((uint64_t)rtcFrac * 1000000u + 0x80000000u) >> 32);
	const int64_t trueUs = (int64_t)ppsSec * 1000000 + (int64_t)(uint32_t)(readAt - ppsAt);
	return rtcUs - trueUs;
}

// RTC_SHIFTR for removing an error of errUs (|errUs| below a second).
struct Shift {
	bool add1s;
	uint32_t subfs;
};
// false: under half a sub-second step, nothing to shift.
inline bool shiftFor(int64_t errUs, uint32_t predivS, Shift *out) {
	const uint64_t steps = (uint64_t)predivS + 1;
	const uint64_t mag = (uint64_t)(errUs < 0 ? -errUs : errUs);
	if (mag >= 1000000) return false;
	const uint32_t n = (uint32_t)((mag * steps + 500000) / 1000000);   // sub-second steps, rounded
	if (n == 0 || n >= steps) return false;
	if (errUs > 0) {        // ahead: delay by n steps
		out->add1s = false;
		out->subfs = n;
	} else {                // behind: a second forward, then back by the rest
		out->add1s = true;
		out->subfs = (uint32_t)(steps - n);
	}
	return true;
}

// RTC_CALR for a frequency correction in parts per billion (positive:
// make the RTC run faster), to the nearest 0.954 ppm. Clamped to what the
// register can do: -487.1 .. +488.5 ppm.
inline void calibrationFor(int32_t ppb, bool *calp, uint32_t *calm) {
	// With p = 512 CALP - CALM the RTC runs p / (2^20 - p) fast, so
	// p = ppb x 2^20 / (1e9 + ppb), rounded.
	const int64_t num = (int64_t)ppb * 1048576;
	const int64_t den = 1000000000 + (int64_t)ppb;
	int64_t pulses = (num + (num >= 0 ? den / 2 : -den / 2)) / den;
	if (pulses > 512) pulses = 512;
	if (pulses < -511) pulses = -511;
	if (pulses > 0) {
		*calp = true;
		*calm = (uint32_t)(512 - pulses);
	} else {
		*calp = false;
		*calm = (uint32_t)(-pulses);
	}
}

// What RTC_CALR is doing now, in ppb (the register's exact effect).
inline int32_t calibrationPpb(bool calp, uint32_t calm) {
	const int64_t num = (calp ? 512 : 0) - (int64_t)calm;
	const int64_t den = 1048576 + (int64_t)calm - (calp ? 512 : 0);
	return (int32_t)((num * 1000000000 + (num >= 0 ? den / 2 : -den / 2)) / den);
}

// How fast the RTC is running, in ppb: its error went from err0Us to
// err1Us over seconds.
inline int32_t driftPpb(int64_t err0Us, int64_t err1Us, uint32_t seconds) {
	if (seconds == 0) return 0;
	return (int32_t)((err1Us - err0Us) * 1000 / (int64_t)seconds);
}

// The control loop, one call per PPS second with the RTC's measured error.
//
// Phase: an error of setUs or more (or the first one after the RTC was set
// some other way) means set the RTC outright; above shiftUs, shift it away.
// Frequency: every freqSeconds the drift since the last look (corrections
// taken out) adjusts the smooth calibration, if it is half a ppm or more.
// The measurement is quantised to the RTC's sub-second step (244 us), so
// shiftUs sits above it and freqSeconds is long enough to see 0.5 ppm.
class Discipline {
public:
	struct Step {
		bool set = false;        // set the RTC from the PPS (the error is too big to shift)
		bool shift = false;      // shift the RTC by -shiftUs
		int64_t shiftUs = 0;
		bool calibrate = false;  // write RTC_CALR for calPpb
		int32_t calPpb = 0;
	};

	static const int64_t kSetUs = 200000;
	static const int64_t kShiftUs = 400;
	static const int64_t kLockUs = 1000;
	static const uint32_t kFreqSeconds = 1024;
	static const int32_t kMinCalChangePpb = 500;
	static const int32_t kMaxCalPpb = 400000;

	// calPpb: what RTC_CALR holds now (it survives a reset).
	explicit Discipline(int32_t calPpb = 0) : cal_(calPpb) {}

	// PPS lost, or the RTC set another way: start the measurement again.
	void restart() { have_ = false; locked_ = false; }

	bool locked() const { return locked_; }
	int32_t calPpb() const { return cal_; }

	Step update(uint32_t sec, int64_t errUs) {
		Step st;
		const int64_t mag = errUs < 0 ? -errUs : errUs;
		if (mag >= kSetUs) {
			st.set = true;
			have_ = false;
			locked_ = false;
			return st;
		}
		if (!have_) {
			have_ = true;
			refSec_ = sec;
			refErr_ = errUs;
		}
		if (sec - refSec_ >= kFreqSeconds) {
			const int32_t drift = driftPpb(refErr_, errUs, sec - refSec_);   // positive: running fast
			if (drift >= kMinCalChangePpb || drift <= -kMinCalChangePpb) {
				int32_t want = cal_ - drift;
				if (want > kMaxCalPpb) want = kMaxCalPpb;
				if (want < -kMaxCalPpb) want = -kMaxCalPpb;
				bool calp;
				uint32_t calm;
				calibrationFor(want, &calp, &calm);
				cal_ = calibrationPpb(calp, calm);   // what the register will really do
				st.calibrate = true;
				st.calPpb = cal_;
			}
			refSec_ = sec;
			refErr_ = errUs;
		}
		if (mag > kShiftUs) {
			st.shift = true;
			st.shiftUs = errUs;
			refErr_ -= errUs;   // the drift measurement carries on across the correction
		}
		locked_ = mag < kLockUs;
		return st;
	}

private:
	bool have_ = false;
	bool locked_ = false;
	uint32_t refSec_ = 0;
	int64_t refErr_ = 0;
	int32_t cal_;
};

} // namespace ppsmath

#endif /* PPS_MATH_H_ */
