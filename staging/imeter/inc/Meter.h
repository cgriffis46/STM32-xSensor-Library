/*
 * Meter.h
 *
 *  meter::Meter: the iMeter of any iMeterFrontEnd. Hardware and OS agnostic,
 *  non-blocking: main(nowMs) does a little and returns.
 *
 *      Hy3131FrontEnd<...> fe(dmm, gw121Routes, ...);   // the board's front end
 *      meter::Meter m(fe, &calStore);
 *      m.setFunction(meter::Function::DcVolts);         // from the rotary switch
 *      for(;;){ m.main(nowMs); }                        // its own thread (xMeter), or
 *                                                       // the front end's, after its main()
 *      meter::Reading r; if (m.latest(&r)) { ...show r... }
 *
 *  What it does:
 *   * Function and range changes come through one-deep mailboxes: the newest
 *     request wins. A rotary switch swept across five positions costs one
 *     reconfiguration, not five.
 *   * Auto range: up on overload or past full scale; down when the value is
 *     below kDownFraction of the next range down's full scale. The gap is
 *     the hysteresis. It starts on the highest range, which is the safe one
 *     on an unknown input.
 *   * After every select(), the front end's settleResults() results are
 *     thrown away, and so is any result tagged with an older selection.
 *   * The front end's averageResults() results make one reading. An
 *     overload doesn't wait for the rest.
 *   * Calibration per function and range, from an iCalibrationStore. Gain 1,
 *     offset 0 and kUncal without one.
 *   * Hold, relative (the next valid value becomes the zero), min / max.
 *   * Never optimistic: the front end's faulted() shows as kFault until
 *     results come again.
 */

#ifndef METER_H_
#define METER_H_

#include <stdint.h>
#include <atomic>
#include "iMeter.h"
#include "Latest.h"

namespace meter {

class Meter : public iMeter {
public:
	static constexpr float    kDownFraction = 0.9f;
	static constexpr uint32_t kIdleSleepMs  = 20;   // longest sleep: also how often results are looked for

	explicit Meter(iMeterFrontEnd& fe, iCalibrationStore* cal = nullptr) : _fe(fe), _cal(cal) {}
	~Meter() override = default;

	void main(uint32_t nowMs);

	// iMeter
	bool setFunction(Function f) override;
	bool setRange(uint8_t range) override;
	void setAutoRange(bool on) override;
	void hold(bool on) override;
	void relative(bool on) override;
	void resetMinMax() override;
	bool latest(Reading* out) const override { return _latest.read(out); }
	bool supports(Function f) const override { return _fe.supports(f); }
	uint8_t rangeCount(Function f) const override { return _fe.rangeCount(f); }
	const RangeSpec* rangeSpec(Function f, uint8_t r) const override { return _fe.rangeSpec(f, r); }
	void setListener(iMeterListener* l) override { _listener = l; }

	// A front end calls this when it has a new result, so an OS version
	// wakes at once rather than at the next look.
	void notifyResult() { wake(); }

	Function function() const { return _fn; }
	uint8_t  range() const { return _range; }
	bool     autoRange() const { return _auto; }

protected:
	virtual void sleep(uint32_t ms) { (void)ms; }
	virtual void wake() {}
	bool workPending() const { return _req.load(std::memory_order_acquire) != 0 || _opt.load(std::memory_order_acquire) != 0; }

private:
	enum class State : uint8_t { Off, Selecting, Settling, Measuring };

	// The function / range mailbox, one 32 bit word: bit 0 a request is
	// there, bit 1 a manual range, bits 8..15 the function (None: the
	// current one), bits 16..23 the range.
	static constexpr uint32_t kReqValid = 1, kReqManual = 2;
	static uint32_t pack(Function f, bool manual, uint8_t r) {
		return kReqValid | (manual ? kReqManual : 0u) | ((uint32_t)f << 8) | ((uint32_t)r << 16);
	}

	// Option requests: a "change" bit and its value bit each.
	static constexpr uint32_t kOptHold = 1, kOptHoldOn = 2, kOptRel = 4, kOptRelOn = 8, kOptReset = 16, kOptAuto = 32, kOptAutoOn = 64;
	void setOpt(uint32_t change, uint32_t valueBit, bool on);

	void takeRequests(uint32_t nowMs);
	void beginSelect(uint32_t nowMs);
	void loadCalibration();
	void handleResult(const RawResult& r);
	void averageResult(const RawResult& r);
	void publishStatus(uint16_t status);
	void publish(Reading& rd);

	iMeterFrontEnd&    _fe;
	iCalibrationStore* _cal;
	iMeterListener*    _listener = nullptr;

	std::atomic<uint32_t> _req{0};
	std::atomic<uint32_t> _opt{0};

	State    _state = State::Off;
	Function _fn = Function::None;
	uint8_t  _range = 0;
	bool     _auto = true;
	uint8_t  _tag = 0;
	uint8_t  _settleLeft = 0;
	uint32_t _lastSeq = 0;
	uint32_t _nowMs = 0;
	Calibration _calib = {1.0f, 0.0f, false};
	uint8_t  _avgTarget = 1;
	uint8_t  _avgCount = 0;
	double   _avgSum = 0.0;      // double: 16 results of 3,000,000 counts lose digits in float

	bool     _hold = false;
	bool     _haveHeld = false;
	bool     _rel = false;
	bool     _relArmed = false;   // the next valid value becomes the reference
	float    _relRef = 0.0f;
	float    _min = 0.0f, _max = 0.0f;
	uint32_t _mmCount = 0;
	bool     _faultShown = false;

	uint32_t _readingSeq = 0;
	Reading  _last{};
	Latest<Reading> _latest;
};

} // namespace meter

#endif /* METER_H_ */
