/*
 * pps_math_test.cpp
 *
 * Host test for Core/Inc/pps_math.h: pairing, the error measurement, the
 * shift and calibration registers, and the control loop against a
 * simulated RTC that drifts. Not part of the firmware build.
 *
 *   g++ -std=c++14 -Wall -Wextra -I../Core/Inc pps_math_test.cpp -o pps_math_test
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "pps_math.h"

using namespace ppsmath;

static int g_failures = 0;
static void check(bool ok, const char *what) {
	std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// The RTC: true elapsed time runs it at (1 + ppm) x (1 + calibration);
// readings come in whole sub-second steps; shifts as RM0394 describes.
struct SimRtc {
	double t = 0;              // its idea of Unix time, seconds
	double ppm = 0;            // the crystal's own error
	double calPpm = 0;         // RTC_CALR's
	uint32_t steps = 8192;             // PREDIV_S 8191, as main.cpp sets it
	void run(double seconds) { t += seconds * (1 + ppm * 1e-6) * (1 + calPpm * 1e-6); }
	void shift(const Shift &s) { t += (s.add1s ? 1.0 : 0.0) - (double)s.subfs / steps; }
	void read(uint32_t *sec, uint32_t *frac) const {
		const double whole = std::floor(t);
		const double q = std::floor((t - whole) * steps) / steps;   // whole steps
		*sec = (uint32_t)whole;
		*frac = (uint32_t)(q * 4294967296.0);
	}
};

int main() {
	std::printf("pairing\n");
	{
		check(rmcFollowsPps(1000, 1000 + 450000), "RMC 450 ms after the PPS: its second");
		check(!rmcFollowsPps(1000, 1000 + 1200000), "1.2 s after: the next PPS's");
		check(!rmcFollowsPps(1000, 900), "before it: not");
		check(rmcFollowsPps(0xFFFFFF00u, 0x00010000u), "across the timer's wrap");
		check(ppsIntervalOk(5000, 5000 + 1000300) && !ppsIntervalOk(5000, 5000 + 998000), "edges a second apart, within 2 ms");
	}

	std::printf("error measurement\n");
	{
		const uint32_t S = 1791763200u;
		// RTC read 250 ms after the PPS of second S, and reads S + 0.25: no error
		check(rtcErrorUs(S, 0x40000000u, 1250000, S, 1000000) == 0, "on time: 0");
		check(rtcErrorUs(S, 0x40000000u + 0x418937u, 1250000, S, 1000000) == 1000, "1 ms ahead: +1000");
		check(rtcErrorUs(S - 1, 0xF0000000u, 1250000, S, 1000000) == -312500, "312.5 ms behind: -312500");
		check(rtcErrorUs(S + 1, 0, 0x10000u, S, 0xFFFFFFFFu - 1000000 + 0x10000u + 1) == 0, "across the timer's wrap");
	}

	std::printf("shift and calibration registers\n");
	{
		Shift s;
		check(shiftFor(1000, 4095, &s) && !s.add1s && s.subfs == 4, "1 ms ahead: delay 4 steps (976 us)");
		check(shiftFor(-1000, 4095, &s) && s.add1s && s.subfs == 4092, "1 ms behind: ADD1S, SUBFS 4092");
		check(!shiftFor(100, 4095, &s), "100 us: under half a step, no shift");
		check(shiftFor(1000, 8191, &s) && !s.add1s && s.subfs == 8, "PREDIV_S 8191: 1 ms ahead, delay 8 steps (977 us)");
		check(shiftFor(-250, 8191, &s) && s.add1s && s.subfs == 8190, "PREDIV_S 8191: 250 us behind, ADD1S, SUBFS 8190");
		check(shiftFor(100, 8191, &s) && s.subfs == 1 && !shiftFor(50, 8191, &s), "PREDIV_S 8191: 100 us is a step, 50 us is not");
		bool p; uint32_t m;
		calibrationFor(0, &p, &m);
		check(!p && m == 0 && calibrationPpb(p, m) == 0, "0 ppb: CALP 0, CALM 0");
		calibrationFor(-23400, &p, &m);
		check(!p && m == 25 && std::abs(calibrationPpb(p, m) + 23842) < 2, "-23.4 ppm: CALM 25 (-23.84 ppm)");
		calibrationFor(10000, &p, &m);
		check(p && m == 502 && std::abs(calibrationPpb(p, m) - 9537) < 2, "+10 ppm: CALP, CALM 502 (+9.54 ppm)");
		calibrationFor(-600000, &p, &m);
		check(!p && m == 511, "-600 ppm: clamped to CALM 511");
		bool worst = true;
		for (int32_t ppb = -480000; ppb <= 480000; ppb += 777) {
			calibrationFor(ppb, &p, &m);
			if (std::abs(calibrationPpb(p, m) - ppb) > 480) worst = false;
		}
		check(worst, "-480..+480 ppm: within half a step (0.48 ppm) of what was asked");
	}

	std::printf("control loop, simulated\n");
	const double ppms[] = {23.4, -17.0, 3.1, 0.0, -48.0};
	for (double ppm : ppms) {
		SimRtc rtc;
		rtc.ppm = ppm;
		const uint32_t S0 = 1791763200u;
		rtc.t = S0 + 0.137;          // set from NMEA, 137 ms off
		Discipline d;
		int sets = 0, shifts = 0, cals = 0;
		double worstPhaseLastHour = 0;
		const uint32_t hours = 6;
		for (uint32_t k = 1; k <= hours * 3600; ++k) {
			rtc.run(1.0);            // to the next PPS edge
			const uint32_t S = S0 + k;
			// measured 300 ms after the edge, as the RMC arrives
			rtc.run(0.3);
			uint32_t sec, frac;
			rtc.read(&sec, &frac);
			const int64_t err = rtcErrorUs(sec, frac, 300000, S, 0);
			const Discipline::Step st = d.update(S, err);
			if (st.set) { rtc.t = S + 0.3; ++sets; }
			if (st.shift) { Shift sh; if (shiftFor(st.shiftUs, rtc.steps - 1, &sh)) { rtc.shift(sh); ++shifts; } }
			if (st.calibrate) { rtc.calPpm = st.calPpb / 1000.0; ++cals; }
			rtc.run(-0.3);           // back to the edge, to keep the loop simple
			if (k > (hours - 1) * 3600) {
				const double phase = std::fabs(rtc.t - S);
				if (phase > worstPhaseLastHour) worstPhaseLastHour = phase;
			}
		}
		const double residual = (1 + ppm * 1e-6) * (1 + rtc.calPpm * 1e-6) - 1;
		std::printf("    crystal %+6.1f ppm: calibration %+7.2f ppm, residual %+5.2f ppm, %d set, %d shifts, %d calibrations, worst phase in hour 6: %.0f us\n",
		            ppm, rtc.calPpm, residual * 1e6, sets, shifts, cals, worstPhaseLastHour * 1e6);
		char what[96];
		std::snprintf(what, sizeof what, "%+.1f ppm crystal: within 1 ppm and 0.5 ms after 6 h, locked", ppm);
		check(std::fabs(residual * 1e6) < 1.0 && worstPhaseLastHour < 0.0005 && d.locked(), what);
	}

	std::printf("a big error sets the RTC\n");
	{
		Discipline d;
		check(d.update(100, 250000).set && !d.locked(), "250 ms: set");
		const Discipline::Step st = d.update(101, 150);
		check(!st.set && !st.shift && d.locked(), "then 150 us: nothing to do, locked");
		d.restart();
		check(!d.locked(), "restart(): not locked until the next measurement");
	}

	std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures, g_failures == 1 ? "" : "s");
	return g_failures ? 1 : 0;
}
