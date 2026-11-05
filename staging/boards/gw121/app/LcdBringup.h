/*
 * LcdBringup.h
 *
 *  A first firmware for the 121GW itself: the LCD, keys, buzzer and rotary
 *  switch, without the HY3131 or anything analog. Four screens, SETUP
 *  steps through them:
 *
 *    1 ALL     every segment of the glass on (Gw121Glass.h's 160)
 *    2 DIGITS  both displays count 0..9 in every place, the point walking,
 *              the minus on odd digits; then 8.8.8.8.8 with every point
 *    3 WALK    one segment at a time, in Gw121Glass.h's kAll order, with its
 *              index on the sub display (on the main display while the
 *              segment is one of the sub display's own). RANGE next, REL
 *              back, long press: 10 at a time. Compare with the index list
 *              in the README.
 *    4 HMI     the real HMI (Gw121Lcd.h's LcdLayout) on our meter stack,
 *              with made-up readings (the BLE stand-in's SimFrontEnd). The
 *              rotary switch picks the function, RANGE/HOLD/REL/MODE work
 *              as they will on the meter, and the sub display shows the
 *              rotary switch's position (0 Low Z .. 7 mA/A).
 *
 *  On every screen: PEAK contrast up, MEM contrast down (0..7, 4 is the
 *  stock setting). A key beeps once if it did something, twice if not.
 *
 *  Hardware and OS agnostic, non-blocking: give it an iSegmentLcd, an
 *  iBeeper and something that sets the contrast, feed it key events and
 *  rotary switch positions, and call main(nowMs) often.
 */

#ifndef GW121_LCD_BRINGUP_H_
#define GW121_LCD_BRINGUP_H_

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "SegmentFrame.h"
#include "SevenSeg.h"
#include "xSegmentHmi.h"
#include "KeyScanner.h"
#include "Buzzer.h"
#include "Meter.h"
#include "MeterTags.h"
#include "Gw121Glass.h"
#include "Gw121Lcd.h"
#include "Gw121Keys.h"
#include "Gw121RangeSwitch.h"
#include "Gw121Simulator.h"

namespace gw121 {

class iLcdContrast {
public:
	virtual ~iLcdContrast() = default;
	virtual void setContrast(uint8_t cc) = 0;   // 0..7
};

class LcdBringup {
public:
	enum class Screen : uint8_t { All, Digits, Walk, Hmi, Count_ };
	static constexpr uint32_t kDigitStepMs = 700;
	static constexpr uint32_t kHmiRefreshMs = 100;

	LcdBringup(idisplay::iSegmentLcd& lcd, idisplay::iBeeper& beeper, iLcdContrast* contrast = nullptr, uint8_t cc = 4)
		: _lcd(lcd), _beep(beeper), _contrast(contrast), _cc(cc > 7 ? 7 : cc),
		  _meter(_fe), _tags(_meter), _chain(_sources, 2),
		  _hmi(lcd, _chain, _layout.fields(), _layout.count()), _ctl(_chain, &beeper) {
		_sources[0] = &_tags;
		_sources[1] = &_board;
		_meter.setFunction(firstFunction(Position::Volts));
		_ctl.setPosition(Position::Volts);
	}

	// Call once at start: the first screen, a beep, the contrast.
	void start() {
		if (_contrast) _contrast->setContrast(_cc);
		_beep.excited();
		enter(Screen::All);
	}

	void onKey(const idisplay::KeyEvent& e) {
		using A = idisplay::KeyEvent::Action;
		const bool shortPress = e.action == A::Pressed;
		const bool held = e.action == A::Held;
		if (e.keys == kKeySetup && shortPress) {
			enter((Screen)(((uint8_t)_screen + 1) % (uint8_t)Screen::Count_));
			_beep.ok();
			return;
		}
		if ((e.keys == kKeyPeak || e.keys == kKeyMem) && shortPress) {
			const bool up = e.keys == kKeyPeak;
			if ((up && _cc >= 7) || (!up && _cc == 0)) { _beep.no(); return; }
			_cc = (uint8_t)(up ? _cc + 1 : _cc - 1);
			if (_contrast) _contrast->setContrast(_cc);
			_beep.ok();
			return;
		}
		if (_screen == Screen::Walk && (shortPress || held) && (e.keys == kKeyRange || e.keys == kKeyRel)) {
			const int step = (e.keys == kKeyRange ? 1 : -1) * (held ? 10 : 1);
			const int n = (int)glass::kAllCount;
			_walk = (uint16_t)((((int)_walk + step) % n + n) % n);
			_dirty = true;
			_beep.ok();
			return;
		}
		if (_screen == Screen::Hmi) { _ctl.onKey(e); return; }
		if (shortPress || held) _beep.no();
	}

	// The rotary switch moved (RangeSwitch::poll()).
	void onPosition(Position p) {
		_board.position = p;
		_ctl.setPosition(p);
		const meter::Function f = firstFunction(p);
		if (f != meter::Function::None) _meter.setFunction(f);
	}

	void main(uint32_t nowMs) {
		_fe.tick(nowMs);
		_meter.main(nowMs);
		switch (_screen) {
		case Screen::All:
			if (_dirty) { _frame.clear(); for (unsigned i = 0; i < glass::kAllCount; i++) _frame.set(glass::kAll[i].seg, true); _dirty = false; _pending = true; }
			break;
		case Screen::Digits:
			if (_dirty || nowMs - _stepMs >= kDigitStepMs) {
				if (!_dirty) _digit = (uint8_t)((_digit + 1) % 11);
				_stepMs = nowMs;
				drawDigits();
				_dirty = false;
				_pending = true;
			}
			break;
		case Screen::Walk:
			if (_dirty) { drawWalk(); _dirty = false; _pending = true; }
			break;
		case Screen::Hmi:
			if (nowMs - _stepMs >= kHmiRefreshMs || _dirty) { _stepMs = nowMs; _dirty = false; _hmi.refresh(); }
			return;
		default:
			return;
		}
		if (_pending && _lcd.write(_frame)) _pending = false;   // a busy LCD: next time
	}

	Screen screen() const { return _screen; }
	uint16_t walkIndex() const { return _walk; }
	uint8_t contrast() const { return _cc; }
	meter::Meter& meter() { return _meter; }

	// The segments the walk shows its index on, for segment i: the sub
	// display's, or the main display's while i is one of the sub display's.
	static bool onSubDisplay(uint16_t i) { return strncmp(glass::kAll[i].name, "SS_", 3) == 0; }

private:
	// The board's own tags for the HMI: sub.value shows the rotary switch's position.
	struct BoardTags : idisplay::iTagSource {
		Position position = Position::None;
		int32_t find(const char* n) override {
			if (strcmp(n, "sub.value") == 0) return 0;
			if (strcmp(n, "sub.decimals") == 0) return 1;
			return -1;
		}
		bool read(int32_t h, idisplay::xTagValue& o) override {
			if (h == 0) { o = position == Position::None ? idisplay::xTagValue::num(NAN) : idisplay::xTagValue::num((double)(uint8_t)position); return true; }
			if (h == 1) { o = idisplay::xTagValue::num(0); return true; }
			return false;
		}
		bool write(int32_t, const idisplay::xTagValue&) override { return false; }
	};

	void enter(Screen s) {
		_screen = s;
		_dirty = true;
		_digit = 0;
		_stepMs = 0;
		if (s == Screen::Hmi) _hmi.invalidate();
	}

	static void drawNumber(idisplay::SegmentFrame& f, const idisplay::SevenSegDigit* d, uint16_t v) {
		// Right aligned, no leading zeros.
		for (int i = 4; i >= 0; i--) {
			idisplay::seven::draw(f, d[i], idisplay::seven::glyph((char)('0' + v % 10)));
			v /= 10;
			if (!v) break;
		}
	}

	void drawDigits() {
		using namespace idisplay;
		_frame.clear();
		const bool eights = _digit == 10;
		const uint8_t strokes = eights ? seven::glyph('8') : seven::glyph((char)('0' + _digit));
		for (uint8_t i = 0; i < 5; i++) { seven::draw(_frame, lcd::kMain[i], strokes); seven::draw(_frame, lcd::kSub[i], strokes); }
		for (uint8_t p = 0; p < 4; p++) {
			const bool on = eights || p == _digit % 4;
			_frame.set(lcd::kMainPoints[p], on);
			_frame.set(lcd::kSubPoints[p], on);
		}
		const bool minus = eights || (_digit & 1);
		_frame.set(glass::MS_NEGATIVE, minus);
		_frame.set(glass::SS_NEGATIVE, minus);
	}

	void drawWalk() {
		_frame.clear();
		drawNumber(_frame, onSubDisplay(_walk) ? lcd::kMain : lcd::kSub, _walk);
		_frame.set(glass::kAll[_walk].seg, true);
	}

	idisplay::iSegmentLcd& _lcd;
	idisplay::iBeeper& _beep;
	iLcdContrast* _contrast;
	uint8_t _cc;

	SimFrontEnd _fe;
	meter::Meter _meter;
	meter::MeterTags _tags;
	BoardTags _board;
	idisplay::iTagSource* _sources[2] = {};
	idisplay::xTagChain _chain;
	LcdLayout _layout;
	idisplay::xSegmentHmi _hmi;
	Controller _ctl;

	Screen _screen = Screen::All;
	idisplay::SegmentFrame _frame;
	bool _dirty = true, _pending = false;
	uint8_t _digit = 0;
	uint16_t _walk = 0;
	uint32_t _stepMs = 0;
};

} // namespace gw121

#endif /* GW121_LCD_BRINGUP_H_ */
