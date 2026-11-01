/*
 * Hy3131FrontEnd.h
 *
 *  meter::Hy3131FrontEnd<TDmm>: an iMeterFrontEnd made of an hy3131_dmm (or
 *  xhy3131) and a board's route table. The table says, for each function and
 *  range, how to set up the chip and which of its results is the reading:
 *
 *      static void dcv5(hy3131::Config& c)  { ...switches, AD1 inputs, gain... }
 *      static void dcv50(hy3131::Config& c) { ... }
 *      static const meter::Hy3131Route kRoutes[] = {
 *        // function                 range  spec: full scale, V per count, OL, unit, decimals, label    build  source         settle
 *        { meter::Function::DcVolts, 0,     { 5.0f,  1e-4f, 0, meter::Unit::Volt, 4, "5V"  },          dcv5,  Source::Ad1,   1 },
 *        { meter::Function::DcVolts, 1,     { 50.0f, 1e-3f, 0, meter::Unit::Volt, 3, "50V" },          dcv50, Source::Ad1,   1 },
 *      };
 *      meter::Hy3131FrontEnd<xhy3131<...>> fe(dmm, kRoutes, sizeof kRoutes / sizeof kRoutes[0]);
 *
 *  The route tables for a real meter come from its schematic (the 121GW's,
 *  when we have it). Each build() starts from hy3131::defaultConfig(), with
 *  INTE cleared. If build() leaves INTE at 0, the front end turns on the
 *  interrupt that the source needs.
 *
 *  The front end is the dmm's iListener. It takes the dmm's results in the
 *  dmm's thread and hands them to the meter's thread through Latest. A
 *  result is tagged with the selection that produced it: the tag rides in
 *  hy3131::Config::userTag, so onRunning() knows which configuration the
 *  chip is in.
 *
 *  Every select() goes through start(), and so through a reset of the chip:
 *  a few tens of ms per range change. Good enough to begin with. A
 *  reconfigure without the reset is a later improvement to hy3131_dmm.
 */

#ifndef HY3131_FRONT_END_H_
#define HY3131_FRONT_END_H_

#include <math.h>
#include <atomic>
#include "iMeter.h"
#include "Latest.h"
#include "hy3131.h"

namespace meter {

enum class Hy3131Source : uint8_t {
	Ad1,        // AD1, 24 bits
	Ad2,        // AD2 (or AD3 with SDO23)
	Lpf,        // the LPF of AD2
	RmsRoot,    // the square root of RMS (sum of squares / N): an RMS in AD2 counts
	PeakMax, PeakMin,
	Frequency,  // the counter: Hz
	Duty,       // the counter: percent high
};

struct Hy3131Route {
	Function     function;
	uint8_t      range;
	RangeSpec    spec;
	void       (*build)(hy3131::Config& cfg);
	Hy3131Source source;
	uint8_t      settle;    // results thrown away after the change
};

template <typename TDmm>
class Hy3131FrontEnd : public iMeterFrontEnd, public hy3131::iListener {
public:
	Hy3131FrontEnd(TDmm& dmm, const Hy3131Route* routes, uint8_t count, uint32_t sysclkHz = hy3131::kXtalHz)
		: _dmm(dmm), _routes(routes), _count(count), _sysclkHz(sysclkHz) {
		_dmm.setListener(this);
	}

	// iMeterFrontEnd (the meter's thread)

	bool supports(Function f) const override { return rangeCount(f) != 0; }

	uint8_t rangeCount(Function f) const override {
		uint8_t n = 0;
		for (uint8_t i = 0; i < _count; i++) if (_routes[i].function == f) n++;
		return n;
	}

	const RangeSpec* rangeSpec(Function f, uint8_t r) const override {
		const Hy3131Route* rt = find(f, r);
		return rt ? &rt->spec : nullptr;
	}

	uint8_t settleResults(Function f, uint8_t r) const override {
		const Hy3131Route* rt = find(f, r);
		return rt ? rt->settle : 1;
	}

	bool select(Function f, uint8_t r, uint8_t tag) override {
		const Hy3131Route* rt = find(f, r);
		if (!rt) return false;
		hy3131::Config cfg = hy3131::defaultConfig();
		cfg.inte = 0;
		if (rt->build) rt->build(cfg);
		if (cfg.inte == 0) cfg.inte = interruptFor(rt->source, cfg);
		cfg.userTag = tag;
		_routeOf[tag & 15u] = rt;
		return _dmm.start(cfg);
	}

	bool result(uint32_t lastSeq, RawResult* out) override {
		RawResult r;
		if (!_latest.read(&r) || r.seq == lastSeq) return false;
		*out = r;
		return true;
	}

	bool faulted() const override { return _faulted.load(std::memory_order_acquire); }

	// hy3131::iListener (the dmm's thread)

	void onRunning() override {
		const uint8_t tag = (uint8_t)_dmm.config().userTag;
		_active = _routeOf[tag & 15u];
		_activeTag = tag;
		_faulted.store(false, std::memory_order_release);
	}

	void onFault() override {
		_active = nullptr;
		_faulted.store(true, std::memory_order_release);
	}

	void onSample(const hy3131::Sample& s) override {
		const Hy3131Route* rt = _active;
		if (!rt) return;
		float raw;
		switch (rt->source) {
		case Hy3131Source::Ad1:     if (!(s.flags & hy3131::kIntAd1)) return; raw = (float)s.ad1; break;
		case Hy3131Source::Ad2:     if (!(s.flags & hy3131::kIntAd2)) return; raw = (float)s.ad2; break;
		case Hy3131Source::Lpf:     if (!(s.flags & hy3131::kIntLpf)) return; raw = (float)s.lpf; break;
		case Hy3131Source::RmsRoot: if (!(s.flags & hy3131::kIntRms)) return; raw = s.rms > 0 ? sqrtf((float)s.rms) : 0.0f; break;
		case Hy3131Source::PeakMax: if (!s.peakValid) return; raw = (float)s.peakMax; break;
		case Hy3131Source::PeakMin: if (!s.peakValid) return; raw = (float)s.peakMin; break;
		default: return;
		}
		put(raw, false, s.atMs);
	}

	void onCount(const hy3131::Count& c) override {
		const Hy3131Route* rt = _active;
		if (!rt) return;
		if (rt->source == Hy3131Source::Frequency)  put((float)c.frequencyHz(_sysclkHz), c.overflow, c.atMs);
		else if (rt->source == Hy3131Source::Duty)  put((float)(c.duty() * 100.0), c.overflow, c.atMs);
	}

private:
	const Hy3131Route* find(Function f, uint8_t r) const {
		for (uint8_t i = 0; i < _count; i++) if (_routes[i].function == f && _routes[i].range == r) return &_routes[i];
		return nullptr;
	}

	static uint8_t interruptFor(Hy3131Source s, const hy3131::Config& cfg) {
		switch (s) {
		case Hy3131Source::Ad1:     return hy3131::kIntAd1;
		case Hy3131Source::Ad2:     return hy3131::kIntAd2;
		case Hy3131Source::Lpf:     return hy3131::kIntLpf;
		case Hy3131Source::RmsRoot: return hy3131::kIntRms;
		case Hy3131Source::PeakMax:
		case Hy3131Source::PeakMin: {
			// Peak hold is read with the results of the ADC it follows (PKHSEL).
			const uint8_t sel = cfg.r(hy3131::kR29) & hy3131::kR29PkhselMask;
			return sel == 0 ? hy3131::kIntAd2 : sel == 1 ? hy3131::kIntAd1 : hy3131::kIntLpf;
		}
		default:                    return hy3131::kIntCt;
		}
	}

	void put(float raw, bool overload, uint32_t atMs) {
		RawResult r;
		r.seq = ++_seq;
		r.atMs = atMs;
		r.raw = raw;
		r.overload = overload;
		r.rangeTag = _activeTag;
		_latest.publish(r);
	}

	TDmm&              _dmm;
	const Hy3131Route* _routes;
	uint8_t            _count;
	uint32_t           _sysclkHz;

	// Which route each tag selected. Written in select() before start(),
	// read in onRunning() after the dmm has taken that start().
	const Hy3131Route* volatile _routeOf[16] = {};   // by tag, mod 16: far more than are ever in flight
	const Hy3131Route* volatile _active = nullptr;   // the dmm's thread only
	uint8_t            _activeTag = 0;
	uint32_t           _seq = 0;
	std::atomic<bool>  _faulted{false};
	Latest<RawResult>  _latest;
};

} // namespace meter

#endif /* HY3131_FRONT_END_H_ */
