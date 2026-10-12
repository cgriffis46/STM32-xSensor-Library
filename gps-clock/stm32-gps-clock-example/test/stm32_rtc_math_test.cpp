/*
 * stm32_rtc_math_test.cpp
 *
 * Host test for Core/Inc/stm32_rtc_math.h: the STM32 RTC's registers to and
 * from Unix time, and the sub-second shift. Not part of the firmware build.
 *
 *   g++ -std=c++14 -Wall -Wextra -I../Core/Inc -I../isensor/ds1307/inc \
 *       -I../iTransport/inc -I../isensor/inc stm32_rtc_math_test.cpp -o stm32_rtc_math_test
 */

#include <cstdio>
#include <cstdlib>
#include "stm32_rtc_math.h"

static int g_failures = 0;
static void check(bool ok, const char *what) {
	std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// The RTC as RM0394 describes it, for the shift: SS counts down from
// predivS each second; a shift with ADD1S adds a second and SUBFS to SS.
struct SimRtc {
	uint32_t predivS = 4095;
	uint32_t tr = 0, dr = 0, ss = 0;
	// Set: the second starts now.
	void set(uint32_t unix) { rtcmath::toRegisters(unix, &tr, &dr); ss = predivS; }
	// Time passes, in steps (SS never below 0: a new second starts).
	void advance(uint32_t steps) {
		for (uint32_t i = 0; i < steps; ++i) {
			if (ss == 0) { uint32_t s, f; rtcmath::fromRegisters(tr, dr, predivS, predivS, &s, &f);
			               rtcmath::toRegisters(s + 1, &tr, &dr); ss = predivS; }
			else --ss;
		}
	}
	void shift(uint32_t subfs) {   // ADD1S + SUBFS
		uint32_t s, f; rtcmath::fromRegisters(tr, dr, predivS, predivS, &s, &f);
		rtcmath::toRegisters(s + 1, &tr, &dr);
		ss += subfs;
	}
	double now() { uint32_t s, f; rtcmath::fromRegisters(tr, dr, ss, predivS, &s, &f); return s + f / 4294967296.0; }
};

int main() {
	std::printf("registers\n");
	{
		uint32_t tr, dr;
		check(rtcmath::toRegisters(1791763200u + 13 * 3600 + 45 * 60 + 30, &tr, &dr), "2026-10-12 13:45:30 encodes");
		check(tr == 0x134530u, "TR = 0x134530 (BCD hh mm ss, 24 hour)");
		check(dr == ((0x26u << 16) | (1u << 13) | (0x10u << 8) | 0x12u), "DR: year 26, Monday (1), month 10, day 12");
		uint32_t s, f;
		check(rtcmath::fromRegisters(tr, dr, 4095, 4095, &s, &f) && s == 1791763200u + 49530 && f == 0,
		      "and decodes, at the start of the second");
		check(rtcmath::fromRegisters(tr, dr, 2047, 4095, &s, &f) && s == 1791763200u + 49530 && f == 0x80000000u,
		      "SS half way down: half a second in");
		check(rtcmath::fromRegisters(tr, dr, 0, 4095, &s, &f) && f == (uint32_t)((4095ull << 32) / 4096),
		      "SS 0: the last step of the second");
		check(rtcmath::fromRegisters(tr, dr, 4095 + 1024, 4095, &s, &f) && s == 1791763200u + 49529
		      && f == 0xC0000000u, "SS above PREDIV_S (after a shift): a second less, 3/4 into it");
		check(rtcmath::toRegisters(1791720000u, &tr, &dr) && ((dr >> 13) & 7) == 7, "a Sunday is weekday 7");
		check(!rtcmath::toRegisters(946684799u, &tr, &dr), "1999 refused");
		check(!rtcmath::fromRegisters(0x250000u, dr, 0, 4095, &s, &f), "hour 25 refused");
		check(!rtcmath::fromRegisters(0, 0, 0, 4095, &s, &f), "year 00 month 00 (never set) refused");
		bool round = true;
		for (uint32_t u = 946684800u; u < 4102444800u - 99999; u += 99991u * 37u) {
			uint32_t a, b, s2, f2;
			if (!rtcmath::toRegisters(u, &a, &b) || !rtcmath::fromRegisters(a, b, 4095, 4095, &s2, &f2) || s2 != u) round = false;
		}
		check(round, "round trip, 2000 to 2099");
	}

	std::printf("shift\n");
	{
		check(rtcmath::shiftSubFs(0, 4095) == 0, "fraction 0: no shift");
		check(rtcmath::shiftSubFs(0x80000000u, 4095) == 2048, "half a second: SUBFS 2048 (1 s forward, 0.5 s back)");
		check(rtcmath::shiftSubFs(0x00040000u, 4095) == 0, "under half a step: no shift");
		check(rtcmath::shiftSubFs(0xFFFFFFFFu, 4095) == 0, "a whole second, rounded: no shift (the caller adds the second)");
		check(rtcmath::precisionLog2(4095) == -12 && rtcmath::precisionLog2(255) == -8, "precision -12 at 4096 steps, -8 at 256");

		// Set at T + 0.3 s, as StmRtc_Set does: whole second, then the shift.
		bool good = true;
		for (int ms = 1; ms < 1000; ms += 37) {
			SimRtc r;
			const uint32_t T = 1791806400u;
			r.set(T);
			const uint32_t frac = (uint32_t)(((uint64_t)ms << 32) / 1000);
			const uint32_t subfs = rtcmath::shiftSubFs(frac, r.predivS);
			if (subfs) r.shift(subfs);
			const double err = r.now() - (T + ms / 1000.0);
			if (std::abs(err) > 1.0 / 4096) { good = false; std::printf("    %d ms: off by %.6f s\n", ms, err); }
			r.advance(4096 * 2 + 100);   // two seconds and a bit later
			const double err2 = r.now() - (T + ms / 1000.0 + 2 + 100 / 4096.0);
			if (std::abs(err2) > 1.0 / 4096) { good = false; std::printf("    %d ms, later: off by %.6f s\n", ms, err2); }
		}
		check(good, "set at whole second, shift: within one step (244 us) of the target, and stays so");
	}

	std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures, g_failures == 1 ? "" : "s");
	return g_failures ? 1 : 0;
}
