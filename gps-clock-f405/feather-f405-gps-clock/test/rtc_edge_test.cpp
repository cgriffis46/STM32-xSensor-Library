/*
 * rtc_edge_test.cpp
 *
 * Host test for Core/Inc/rtc_edge.h against a simulated RTC: its square
 * wave, seconds that tick on the falling edge or on the rising one (with a
 * delay), and probe reads every ~20 ms timed as the firmware times them.
 * Not part of the firmware build.
 *
 *   g++ -std=c++14 -Wall -Wextra -I../Core/Inc rtc_edge_test.cpp -o rtc_edge_test
 */

#include <cstdio>
#include <cstdint>
#include "rtc_edge.h"

using namespace rtcedge;

static int g_failures = 0;
static void check(bool ok, const char *what) {
	std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

struct Run {
	Analyzer a;
	int ticks = 0;
};

// t0: timer count at the start (to cross the wrap). Square wave falling at
// t0 + lag + k * period, rising half a period later; the seconds tick
// `delay` us after the chosen edge. Reads every `every` us, each START
// somewhere inside a window of `spread` us.
static Run simulate(bool tickOnRising, int32_t delayUs, uint32_t t0, uint32_t periodUs = 999988,
                    uint32_t lagUs = 237100, uint32_t everyUs = 20000, uint32_t spreadUs = 300, int seconds = 12) {
	Run r;
	uint64_t nextEdge = lagUs;   // relative to t0
	bool nextRising = false;
	const uint64_t end = (uint64_t)seconds * 1000000;
	uint64_t t = 13700;
	unsigned rnd = 12345;
	while (t < end) {
		while (nextEdge <= t) {
			r.a.edge((uint32_t)(t0 + nextEdge), nextRising);
			nextEdge += periodUs / 2;
			nextRising = !nextRising;
		}
		rnd = rnd * 1103515245u + 12345u;
		const uint64_t from = t, to = t + spreadUs;
		const uint64_t start = from + (rnd >> 8) % (spreadUs + 1);     // the real START
		// seconds at `start`: count the ticks before it
		uint64_t secs = 7;
		for (uint64_t k = 0;; ++k) {
			const uint64_t edge = lagUs + k * periodUs + (tickOnRising ? periodUs / 2 : 0);
			const int64_t tick = (int64_t)edge + delayUs;
			if (tick > (int64_t)start) break;
			++secs;
		}
		if (r.a.read((uint32_t)(t0 + from), (uint32_t)(t0 + to), (uint8_t)(secs % 60))) ++r.ticks;
		t += everyUs + (rnd >> 20) % 3000;
	}
	return r;
}

int main() {
	std::printf("seconds on the falling edge, 0.3 ms after it\n");
	{
		Run r = simulate(false, 300, 0);
		const Window &f = r.a.falling(), &ri = r.a.rising();
		std::printf("    falling %+d..%+d us (%u), rising %+d..%+d us (%u)\n", f.loUs, f.hiUs, f.n, ri.loUs, ri.hiUs, ri.n);
		check(r.ticks >= 11 && r.a.ticks() == r.ticks, "a tick bracketed each second");
		check(f.consistent() && f.loUs <= 300 && f.hiUs >= 300, "falling: the bracket holds the true 300 us");
		check(f.hiUs - f.loUs < 25000, "and narrows to within a read interval");
		check(!ri.consistent() || ri.loUs > 400000, "rising: only as half a second after it (a square wave can't say more)");
		check(r.a.verdict() == Polarity::Falling, "verdict: falling");
	}

	std::printf("seconds on the rising edge, 1 ms after it, across the timer's wrap\n");
	{
		Run r = simulate(true, 1000, 0xFFFFFFFFu - 5000000u);
		const Window &ri = r.a.rising();
		check(ri.consistent() && ri.loUs <= 1000 && ri.hiUs >= 1000, "rising: the bracket holds the true 1 ms");
		check(!r.a.falling().consistent() || r.a.falling().loUs > 400000, "falling: only as half a second after it");
		check(r.a.verdict() == Polarity::Rising, "verdict: rising");
	}

	std::printf("seconds just before the falling edge (-2 ms)\n");
	{
		Run r = simulate(false, -2000, 777);
		const Window &f = r.a.falling();
		check(f.consistent() && f.loUs <= -2000 && f.hiUs >= -2000 && r.a.verdict() == Polarity::Falling,
		      "a negative delay is found too");
	}

	std::printf("reads too slow to bracket\n");
	{
		Run r = simulate(false, 300, 0, 999988, 237100, 600000);
		check(r.a.ticks() == 0 && r.a.verdict() == Polarity::Unknown, "no brackets, no verdict");
	}

	std::printf("no square wave\n");
	{
		Analyzer a;
		a.read(0, 100, 5);
		a.read(1000000, 1000100, 5);
		a.read(1010000, 1010100, 6);
		check(a.ticks() == 1 && !a.falling().consistent() && a.verdict() == Polarity::Unknown,
		      "a tick, but no edge to put it against");
	}

	std::printf("widening a 16-bit capture\n");
	{
		check(widen(0xFFF0, 0x00010005u) == 0x0000FFF0u, "capture before the 32-bit count's low word wrapped");
		check(widen(0x1234, 0xABCD1300u) == 0xABCD1234u && widen(0x1400, 0xABCD1300u) == 0xABCC1400u,
		      "a capture 0.2 ms and 65.3 ms old");
		check(widen(0x0005, 0x00000010u) == 0x00000005u && widen(0xFFFF, 0x00000002u) == 0xFFFFFFFFu,
		      "right across the 32-bit wrap");
	}

	std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures, g_failures == 1 ? "" : "s");
	return g_failures ? 1 : 0;
}
