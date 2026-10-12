/*
 * ptp_math_test.cpp
 *
 * Host test for Core/Inc/ptp_math.h: the unit conversions, the increment
 * and addend, the error measurement, and the control loop against a
 * simulated MAC system time on a crystal that is off frequency. Not part
 * of the firmware build.
 *
 *   g++ -std=c++14 -Wall -Wextra -I../Core/Inc ptp_math_test.cpp -o ptp_math_test
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "ptp_math.h"

using namespace ptpmath;

static int g_failures = 0;
static void check(bool ok, const char *what) {
	std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// The MAC's system time, run as the hardware does: per HCLK cycle the
// addend goes into a 32-bit accumulator, each carry adds the increment to
// the sub-seconds. HCLK is the nominal 120 MHz times the crystal's error.
// Rather than step 120 million cycles a second, time is advanced by the
// exact average (the accumulator only adds a sub-step of jitter).
struct SimPtp {
	double hclk = 120e6;
	double ppm = 0;            // the crystal's error (HSE, so HCLK and TIM2 alike)
	uint8_t increment = 0;
	uint32_t addend = 0;
	long double t = 0;         // its time, seconds
	void run(double trueSeconds) {
		const long double cycles = (long double)trueSeconds * hclk * (1 + ppm * 1e-6);
		t += cycles * addend / 4294967296.0L * increment / 2147483648.0L;
	}
	void read(uint32_t *sec, uint32_t *frac) const {
		const long double whole = std::floor(t);
		const uint32_t sub = (uint32_t)std::floor((t - whole) * 2147483648.0L);
		*sec = (uint32_t)whole;
		*frac = fracFromSub(sub);
	}
	void set(uint32_t sec, uint32_t frac) { t = sec + (long double)subFromFrac(frac) / 2147483648.0L; }
};

int main() {
	std::printf("units\n");
	{
		check(fracFromSub(0x40000000u) == 0x80000000u, "half a second: 2^30 sub-seconds, 2^31 fraction");
		check(fracFromSub(0x80000001u) == 2, "the sign bit of PTPTSLR is not part of the value");
		check(subFromFrac(0xFFFFFFFFu) == 0x7FFFFFFFu, "fraction to sub-seconds");
		check(subFromUs(500000) == 0x40000000u && usFromSub(0x40000000u) == 500000, "500000 us both ways");
		check(subFromUs(999999) == 0x7FFFF79Cu || subFromUs(999999) == 0x7FFFF79Du, "999999 us");
		check(usFromSub(subFromUs(123457)) == 123457, "us round trip");
	}

	std::printf("pairing and widening\n");
	{
		check(rmcFollowsPps(1000, 1000 + 450000), "RMC 450 ms after the PPS: its second");
		check(!rmcFollowsPps(1000, 1000 + 1200000) && !rmcFollowsPps(1000, 900), "1.2 s after, or before: not");
		check(rmcFollowsPps(0xFFFFFF00u, 0x00010000u), "across the timer's wrap");
		check(ppsIntervalOk(5000, 5000 + 1000300) && !ppsIntervalOk(5000, 5000 + 998000), "edges a second apart, within 2 ms");
		check(widen(0x1234, 0x00051240u) == 0x00051234u, "a capture 12 us before the read");
		check(widen(0xFFF0, 0x00060010u) == 0x0005FFF0u, "across the 16-bit wrap");
		check(widen(0x0010, 0x00010010u + 65000u) == 0x00010010u, "a read 65 ms late: still right");
	}

	std::printf("increment and addend\n");
	Base b;
	{
		check(baseFor(120000000, 50000000, &b), "120 MHz HCLK, 50 MHz updates");
		check(b.increment == 43, "increment 43 (2^31 / 50 MHz, as AN3411)");
		// 2^63 / (43 x 120e6) = 1787475200.6...
		check(b.addend == 1787475201u, "addend 2^63 / (43 x HCLK)");
		const long double rate = (long double)b.addend / 4294967296.0L * b.increment / 2147483648.0L * 120e6;
		check(std::fabs((double)(rate - 1)) < 1e-9, "nominal: a second a second, to 1 ppb");
		Base x;
		check(!baseFor(40000000, 50000000, &x), "HCLK below the update rate: refused");
		check(!baseFor(120000000, 1000000, &x), "increment over 255: refused");
		check(addendFor(b, 0) == b.addend, "0 ppb: the base");
		check(ppbOf(b, addendFor(b, 1000)) == 1000 && ppbOf(b, addendFor(b, -37000)) == -37000, "ppb both ways");
		check(ppbOf(b, b.addend + 1) == 1, "one addend step is about half a ppb (rounds to 1)");
		check(addendFor(b, 500000) > b.addend && addendFor(b, -500000) < b.addend, "+-500 ppm fit");
	}

	std::printf("error measurement\n");
	{
		const uint32_t S = 1791763200u;
		check(errorUs(S, 0, 1000003, S, 1000000) == -3, "read 3 us after the edge and showing the edge: 3 us behind");
		check(errorUs(S, fracFromSub(subFromUs(13)), 1000003, S, 1000000) == 10, "10 us ahead");
		check(errorUs(S - 1, 0xFFFFFFFFu, 2, S, 0xFFFFFFFFu) == -3, "across the timer's wrap");
		check(errorUs(S + 1, 0, 5, S, 5) == 1000000, "a second ahead");
	}

	std::printf("loop: a crystal 37 ppm fast, the clock set 0.3 s wrong\n");
	{
		SimPtp p;
		p.ppm = 37;
		p.increment = b.increment;
		p.addend = b.addend;
		Discipline d;
		uint32_t S = 1791763200u;
		p.set(S - 1, 0x4CCCCCCCu);             // 0.7 s behind (-0.3 s from S)
		uint32_t sets = 0, lockedAt = 0;
		int64_t worstAfter = 0;
		srand(7);
		for (uint32_t i = 0; i < 600; ++i, ++S) {
			// The PPS edge of second S. The ISR reads the clock 2..4 us later.
			const uint32_t lat = 2 + (uint32_t)(rand() % 3);
			p.run(lat * 1e-6);
			uint32_t sec, frac;
			p.read(&sec, &frac);
			// the timer reads in whole microseconds, on the same crystal
			const uint32_t ppsAt = 1000u * i;
			const uint32_t readAt = ppsAt + lat;
			const int64_t err = errorUs(sec, frac, readAt, S, ppsAt);
			const Discipline::Step st = d.update(S, err);
			if (st.set) {
				++sets;
				// main.cpp sets it some time after the edge, from the timer
				p.set(S, fracFromSub(subFromUs(lat)));
			}
			p.addend = addendFor(b, st.ppb);
			if (d.locked() && lockedAt == 0) lockedAt = i;
			if (i >= 120 && std::llabs(err) > worstAfter) worstAfter = std::llabs(err);
			p.run(1.0 - lat * 1e-6);
		}
		check(sets == 1, "set once, at the start");
		check(lockedAt > 0 && lockedAt < 120, "locked within two minutes");
		std::printf("     locked after %u s, worst error after 2 min %lld us, frequency %d ppb\n", lockedAt,
		            (long long) worstAfter, d.frequencyPpb());
		check(worstAfter <= 3, "within 3 us once settled (the 1 us timer and 2..4 us latency)");
		check(std::abs(d.frequencyPpb() - (-36999)) < 60, "frequency found: about -37 ppm");
		check(d.locked(), "still locked");

		// Holdover: no PPS for a minute, the clock runs at the learned rate
		uint32_t sec0, frac0;
		p.read(&sec0, &frac0);
		const int64_t before = errorUs(sec0, frac0, 0, S, 0);
		p.addend = addendFor(b, d.holdoverPpb());
		p.run(60.0);
		S += 60;
		uint32_t sec1, frac1;
		p.read(&sec1, &frac1);
		const int64_t after = errorUs(sec1, frac1, 0, S, 0);
		std::printf("     holdover: %lld us after a minute (from %lld)\n", (long long) after, (long long) before);
		check(std::llabs(after - before) < 10, "a minute of holdover adds under 10 us");

		// PPS back after 60 s: the gap restarts the phase history, not the frequency
		const int32_t f = d.frequencyPpb();
		Discipline::Step st = d.update(S, after);
		check(!st.set && !d.locked(), "after a gap: not locked, not set (error small)");
		check(std::abs(d.frequencyPpb() - f) < 50, "frequency kept across the gap");
	}

	std::printf("loop: the crystal warms by 2 ppm\n");
	{
		SimPtp p;
		p.ppm = -12;
		p.increment = b.increment;
		p.addend = b.addend;
		Discipline d;
		uint32_t S = 1791763200u;
		p.set(S, 0);
		int64_t worst = 0;
		for (uint32_t i = 0; i < 900; ++i, ++S) {
			if (i == 300) p.ppm = -10;
			uint32_t sec, frac;
			p.read(&sec, &frac);
			const int64_t err = errorUs(sec, frac, 1000u * i, S, 1000u * i);
			const Discipline::Step st = d.update(S, err);
			if (st.set) p.set(S, 0);
			p.addend = addendFor(b, st.ppb);
			if (i > 300 && std::llabs(err) > worst) worst = std::llabs(err);
			p.run(1.0);
		}
		std::printf("     worst error after the step %lld us, frequency %d ppb\n", (long long) worst, d.frequencyPpb());
		check(worst < 40, "a 2 ppm step costs under 40 us");
		check(std::abs(d.frequencyPpb() - 10000) < 60, "and the new frequency is found");
		check(d.locked(), "locked again");
	}

	std::printf("loop: limits\n");
	{
		Discipline d(300000);
		check(d.frequencyPpb() == Discipline::kMaxFreqPpb, "a stored frequency beyond 200 ppm is clamped");
		Discipline e;
		Discipline::Step st = e.update(100, 999);
		// frequency -999000/256 = -3902 ppb, phase term -999 x 125
		check(!st.set && st.ppb == -3902 - 124875, "999 us: slewed, not set");
		st = e.update(101, 1000);
		check(st.set, "1 ms: set");
		st = e.update(102, -400000);
		check(st.set, "-0.4 s: set");
		Discipline g;
		st = g.update(1, 900);
		st = g.update(2, 900);
		check(st.ppb >= -Discipline::kMaxSlewPpb, "the slew is limited to 500 ppm");
		Discipline h(-5000);
		h.restart();
		check(h.frequencyPpb() == -5000 && h.holdoverPpb() == -5000, "restart keeps the frequency");
		st = h.update(10, 0);
		check(st.ppb == -5000, "no error: runs at the frequency");
	}

	std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "all passed", g_failures, g_failures == 1 ? "" : "s");
	return g_failures ? 1 : 0;
}
