/*
 * rtc_edge.h
 *
 *  Where an RTC's seconds tick against its 1 Hz square wave (DS3231
 *  SQW/INT, DS1307 SQW/OUT), measured on the board: the probe reads the
 *  seconds register every few milliseconds, each read's I2C START timed by
 *  TIM2 either side of the call that issues it, and the square wave's edges
 *  are latched by TIM3 kept in step with TIM2. Two reads a second apart
 *  bracket the tick; each bracket is put against the nearest falling and
 *  the nearest rising edge, and the brackets are intersected. The edge whose
 *  intersection is not empty and lies nearest zero is the one the seconds
 *  follow. (A square wave can't say more: "0.3 ms after the falling edge"
 *  is also "500.3 ms after the rising one". The rule is that a counter
 *  ticks at, or just after, one of its own edges.)
 *
 *  The same method as Claude-iTransport's tools/saleae/saleae_log.py
 *  --rtc-edge, which does it from a logic analyser capture instead: the two
 *  should agree. No HAL here (test/rtc_edge_test.cpp).
 *
 *  Times are counts of a free-running 32-bit 1 MHz timer; differences are
 *  taken unsigned and then as signed 32-bit, right across its wrap.
 */

#ifndef RTC_EDGE_H_
#define RTC_EDGE_H_

#include <stdint.h>

namespace rtcedge {

// One polarity's result: the tick lies between loUs and hiUs of the edge
// (positive: after it), from n brackets. loUs > hiUs: they disagree.
struct Window {
	int32_t loUs = -1000000000;
	int32_t hiUs = 1000000000;
	uint16_t n = 0;
	bool consistent() const { return n != 0 && loUs <= hiUs; }
	int32_t worst() const { return (loUs < 0 ? -loUs : loUs) > (hiUs < 0 ? -hiUs : hiUs) ? (loUs < 0 ? -loUs : loUs)
	                                                                                     : (hiUs < 0 ? -hiUs : hiUs); }
};

enum class Polarity : uint8_t { Unknown, Falling, Rising };

class Analyzer {
public:
	static const int kEdges = 8;            // kept per polarity
	static const uint32_t kMaxBracketUs = 500000;
	static const uint32_t kMaxEdgeUs = 750000;

	void restart() {
		fall_ = rise_ = Window();
		nFall_ = nRise_ = 0;
		haveRead_ = false;
		ticks_ = 0;
	}

	// An edge of the square wave, at timer count `at`.
	void edge(uint32_t at, bool rising) {
		uint32_t *ring = rising ? rise_at_ : fall_at_;
		uint32_t &n = rising ? nRise_ : nFall_;
		ring[n % kEdges] = at;
		++n;
	}

	// A read of the seconds register whose START lay between timer counts
	// from and to. Brackets a tick when the seconds moved on by one.
	// Returns true when it did (bracket in *tickFrom, *tickTo).
	bool read(uint32_t from, uint32_t to, uint8_t seconds, uint32_t *tickFrom = nullptr, uint32_t *tickTo = nullptr) {
		bool ticked = false;
		if (haveRead_ && seconds == (uint8_t)((lastSec_ + 1) % 60) && to - lastFrom_ < kMaxBracketUs) {
			// latched after lastFrom (the earliest the last read could have
			// started) and by `to` (the latest this one could have)
			bracket(lastFrom_, to);
			if (tickFrom) *tickFrom = lastFrom_;
			if (tickTo) *tickTo = to;
			ticked = true;
		}
		haveRead_ = true;
		lastSec_ = seconds;
		lastFrom_ = from;
		return ticked;
	}

	const Window &falling() const { return fall_; }
	const Window &rising() const { return rise_; }
	uint16_t ticks() const { return ticks_; }

	// The edge the seconds follow: consistent, and nearest zero.
	Polarity verdict() const {
		const bool f = fall_.consistent(), r = rise_.consistent();
		if (f && (!r || fall_.worst() <= rise_.worst())) return Polarity::Falling;
		if (r) return Polarity::Rising;
		return Polarity::Unknown;
	}

private:
	void against(const uint32_t *ring, uint32_t n, uint32_t a, uint32_t b, Window *w) {
		const uint32_t mid = a + (b - a) / 2;
		bool have = false;
		uint32_t best = 0;
		int32_t bestD = 0;
		const uint32_t k = n < (uint32_t)kEdges ? n : (uint32_t)kEdges;
		for (uint32_t i = 0; i < k; ++i) {
			const uint32_t e = ring[(n - 1 - i) % kEdges];
			const int32_t d = (int32_t)(mid - e);
			const int32_t mag = d < 0 ? -d : d;
			if (!have || mag < (bestD < 0 ? -bestD : bestD)) { have = true; best = e; bestD = d; }
		}
		if (!have || (uint32_t)(bestD < 0 ? -bestD : bestD) > kMaxEdgeUs) return;
		const int32_t lo = (int32_t)(a - best), hi = (int32_t)(b - best);
		if (lo > w->loUs) w->loUs = lo;
		if (hi < w->hiUs) w->hiUs = hi;
		++w->n;
	}

	void bracket(uint32_t a, uint32_t b) {
		against(fall_at_, nFall_, a, b, &fall_);
		against(rise_at_, nRise_, a, b, &rise_);
		++ticks_;
	}

	uint32_t fall_at_[kEdges] = {0}, rise_at_[kEdges] = {0};
	uint32_t nFall_ = 0, nRise_ = 0;
	Window fall_, rise_;
	bool haveRead_ = false;
	uint8_t lastSec_ = 0;
	uint32_t lastFrom_ = 0;
	uint16_t ticks_ = 0;
};

// A 16-bit capture from a timer counting in step with a 32-bit one (same
// clock and prescaler, started together), widened with the 32-bit count
// read after it: right while that read is less than 65.5 ms late.
inline uint32_t widen(uint16_t capture, uint32_t now32) {
	return now32 - (uint16_t)((uint16_t)now32 - capture);
}

} // namespace rtcedge

#endif /* RTC_EDGE_H_ */
