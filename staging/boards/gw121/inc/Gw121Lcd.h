/*
 * Gw121Lcd.h
 *
 *  The 121GW's LCD as an HMI: every part of the glass is a field bound to
 *  a tag, drawn by idisplay::xSegmentHmi. Change what a part shows by
 *  pointing it at another tag. The meter's tags come from meter::MeterTags
 *  ("meter.*"). The rest are the board's own, from an xTagTable, a PLC tag
 *  database, or both through an xTagChain:
 *
 *    Main display  meter.value, meter.decimals, meter.ol (5 digits, point, minus)
 *    Main units    meter.unit (V A ohm F Hz % C), meter.prefix (n u m k M),
 *                  meter.ac / meter.dc / meter.acdc
 *    Bargraph      meter.bar (-1..1 of full scale) when meter.valid;
 *                  labels from meter.barrange
 *    Icons         meter.auto, meter.hold, meter.rel,
 *                  meter.function (continuity, diode)
 *    Sub display   sub.value, sub.decimals, sub.ol, sub.unit, sub.prefix,
 *                  sub.ac, sub.dc, sub.db, sub.ns
 *    Board icons   bt.on, batt.low, apo.on, ui.min, ui.max, ui.avg, ui.mem,
 *                  ui.test, ui.autohold, meter.lowz, meter.lpf, meter.peak,
 *                  meter.danger, temp.f
 *
 *  A tag that doesn't exist (yet) leaves its part blank, or dashes on a
 *  number. So the sub display shows "-----" until something publishes
 *  sub.value.
 *
 *  The glass map is Gw121Glass.h (generated from the stock firmware's
 *  segment names). Digits and points are as the stock firmware's set_7seg()
 *  and scr_set_decimal_point() drive them; bars and range labels as
 *  scr_update_bargraph() and scr_update_bargraph_range().
 */

#ifndef GW121_LCD_H_
#define GW121_LCD_H_

#include "SegmentFields.h"
#include "xSegmentHmi.h"
#include "Gw121Glass.h"
#include "iMeter.h"

namespace gw121 {

using idisplay::SevenSegDigit;
using idisplay::SegRef;

namespace lcd {
using namespace glass;

#define GW121_DIGIT(p) SevenSegDigit{ p##_A, p##_B, p##_C, p##_D, p##_E, p##_F, p##_G }
inline constexpr SevenSegDigit kMain[5] = { GW121_DIGIT(MS_10000), GW121_DIGIT(MS_1000), GW121_DIGIT(MS_100),
                                            GW121_DIGIT(MS_10), GW121_DIGIT(MS_1) };
inline constexpr SevenSegDigit kSub[5]  = { GW121_DIGIT(SS_10000), GW121_DIGIT(SS_1000), GW121_DIGIT(SS_100),
                                            GW121_DIGIT(SS_10), GW121_DIGIT(SS_1) };
#undef GW121_DIGIT
// The point after digit i (NumberField's order): 4 decimals .. 1.
inline constexpr SegRef kMainPoints[4] = { MS_POINT_d0000, MS_POINT_d000, MS_POINT_d00, MS_POINT_d0 };
inline constexpr SegRef kSubPoints[4]  = { SS_POINT_d0000, SS_POINT_d000, SS_POINT_d00, SS_POINT_d0 };
inline constexpr SegRef kBars[25] = {
	BG_B1, BG_B2, BG_B3, BG_B4, BG_B5, BG_B6, BG_B7, BG_B8, BG_B9, BG_B10, BG_B11, BG_B12, BG_B13,
	BG_B14, BG_B15, BG_B16, BG_B17, BG_B18, BG_B19, BG_B20, BG_B21, BG_B22, BG_B23, BG_B24, BG_B25,
};
} // namespace lcd

// The layout: the fields, and the list xSegmentHmi draws.
class LcdLayout {
public:
	using IconField = idisplay::IconField;
	using U = meter::Unit;
	static constexpr uint16_t kFieldCount = 54;

	LcdLayout() {}

	idisplay::iSegmentField* const* fields() const { return _list; }
	uint16_t count() const { return kFieldCount; }

	// The main display's overload text ("  OL " by default).
	void setOverloadText(const char* t) { _main.setOverloadText(t); _sub.setOverloadText(t); }

private:
	static double u(U x) { return (double)(uint8_t)x; }
	static double fn(meter::Function f) { return (double)(uint8_t)f; }

	idisplay::NumberField _main{ lcd::kMain, 5, lcd::kMainPoints, glass::MS_NEGATIVE, "meter.value", "meter.decimals", 0, "meter.ol" };
	idisplay::NumberField _sub{ lcd::kSub, 5, lcd::kSubPoints, glass::SS_NEGATIVE, "sub.value", "sub.decimals", 0, "sub.ol" };
	idisplay::BarField    _bar{ lcd::kBars, 25, "meter.bar", 1.0, glass::BG_POSITIVE, glass::BG_NEGATIVE, "meter.valid" };

	// Main units and prefixes.
	IconField _volts{ glass::MS_VOLTS, "meter.unit", u(U::Volt) };
	IconField _amps{ glass::MS_AMPS, "meter.unit", u(U::Ampere) };
	IconField _ohms{ glass::MS_OHMS, "meter.unit", u(U::Ohm) };
	IconField _farads{ glass::MS_FARADS, "meter.unit", u(U::Farad) };
	IconField _hertz{ glass::MS_HERTZ, "meter.unit", u(U::Hertz) };
	IconField _percent{ glass::MS_DUTY_PERCENT, "meter.unit", u(U::Percent) };
	IconField _degC{ glass::MS_DEG_C, "meter.unit", u(U::Celsius) };
	IconField _degF{ glass::MS_DEG_F, "temp.f" };
	IconField _nano{ glass::MS_NANO, "meter.prefix", -9 };
	IconField _micro{ glass::MS_MICRO, "meter.prefix", -6 };
	IconField _milli{ glass::MS_MILLI, "meter.prefix", -3 };
	IconField _kilo{ glass::MS_KILO, "meter.prefix", 3 };
	IconField _mega{ glass::MS_MEGA, "meter.prefix", 6 };
	IconField _ac{ glass::MS_AC, "meter.ac" };
	IconField _dc{ glass::MS_DC, "meter.dc" };
	IconField _acdc{ glass::MS_PLUS_OF_AC_DC, "meter.acdc" };   // AC + DC: meter.ac and meter.dc are both true

	// Bargraph scale: 0: 5, 1: 50, 2: 500, 3: 1000 (shown while there's a reading).
	IconField _bgScale{ glass::BG_SCALE, "meter.valid" };
	IconField _bgScale5{ glass::BG_SCALE_5, "meter.valid" };
	IconField _bgScale10{ glass::BG_SCALE_10, "meter.valid" };
	IconField _bgR5{ glass::BG_RANGE_5, "meter.barrange", 0, 2 };
	IconField _bgR50{ glass::BG_RANGE_0_FOR_50, "meter.barrange", 1, 2 };
	IconField _bgR500{ glass::BG_RANGE_0_FOR_500, "meter.barrange", 2 };
	IconField _bgR1000{ glass::BG_RANGE_1000, "meter.barrange", 3 };

	// Icons.
	IconField _auto{ glass::ICON_AUTO_RANGE, "meter.auto" };
	IconField _hold{ glass::ICON_HOLD, "meter.hold" };
	IconField _rel{ glass::ICON_REL, "meter.rel" };
	IconField _cont{ glass::ICON_CONTINUITY, "meter.function", fn(meter::Function::Continuity) };
	IconField _diode{ glass::ICON_DIODE, "meter.function", fn(meter::Function::Diode) };
	IconField _bt{ glass::ICON_BT, "bt.on" };
	IconField _batt{ glass::ICON_BATT, "batt.low" };
	IconField _apo{ glass::ICON_APO, "apo.on" };
	IconField _min{ glass::ICON_MIN, "ui.min" };
	IconField _max{ glass::ICON_MAX, "ui.max" };
	IconField _avg{ glass::ICON_AVG, "ui.avg" };
	IconField _mem{ glass::ICON_MEM, "ui.mem" };
	IconField _test{ glass::ICON_TEST, "ui.test" };
	IconField _autoHold{ glass::ICON_AUTO_OF_HOLD, "ui.autohold" };
	IconField _lowz{ glass::ICON_LOWZ, "meter.lowz" };
	IconField _lpf{ glass::ICON_LPF, "meter.lpf" };
	IconField _peak{ glass::ICON_ONE_OF_1ms_PEAK, "meter.peak" };
	IconField _danger{ glass::ICON_DANGER, "meter.danger" };

	// Sub display units.
	IconField _sVolts{ glass::SS_VOLTS, "sub.unit", u(U::Volt) };
	IconField _sAmps{ glass::SS_AMPS, "sub.unit", u(U::Ampere) };
	IconField _sOhms{ glass::SS_OHMS, "sub.unit", u(U::Ohm) };
	IconField _sHertz{ glass::SS_HERTZ, "sub.unit", u(U::Hertz) };
	IconField _sMilli{ glass::SS_MILLI, "sub.prefix", -3 };
	IconField _sKilo{ glass::SS_KILO, "sub.prefix", 3 };
	IconField _sAc{ glass::SS_AC, "sub.ac" };
	IconField _sDc{ glass::SS_DC, "sub.dc" };
	IconField _sDb{ glass::SS_dB, "sub.db" };
	IconField _sNs{ glass::SS_nS, "sub.ns" };

	// A segment belongs to one field only: a field writes its segments off as well as on.
	idisplay::iSegmentField* const _list[kFieldCount] = {
		&_main, &_sub, &_bar,
		&_volts, &_amps, &_ohms, &_farads, &_hertz, &_percent, &_degC, &_degF,
		&_nano, &_micro, &_milli, &_kilo, &_mega, &_ac, &_dc, &_acdc,
		&_bgScale, &_bgScale5, &_bgScale10, &_bgR5, &_bgR50, &_bgR500, &_bgR1000,
		&_auto, &_hold, &_rel, &_cont, &_diode, &_bt, &_batt, &_apo, &_min, &_max, &_avg, &_mem,
		&_test, &_autoHold, &_lowz, &_lpf, &_peak, &_danger,
		&_sVolts, &_sAmps, &_sOhms, &_sHertz, &_sMilli, &_sKilo, &_sAc, &_sDc, &_sDb, &_sNs,
	};
};

} // namespace gw121

#endif /* GW121_LCD_H_ */
