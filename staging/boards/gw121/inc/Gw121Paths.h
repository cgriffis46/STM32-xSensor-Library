/*
 * Gw121Paths.h
 *
 *  The 121GW's analog switches outside the HY3131, set for each function:
 *  an iMeterPaths for Hy3131FrontEnd. The rules are the stock firmware's
 *  (121gw-re, decompiled and annotated: meas_hw_set_ac_rms_filter_ctl(),
 *  meas_hw_set_hv_diode(), meas_hw_set_va_ctl_useless(),
 *  meas_hw_set_temp_ctl(), meas_hw_set_fre_ctl()):
 *
 *    CTL_A  PD0   U16 X: the AD8436's RMS output to the HY's PB7       low in AC
 *    CTL_E  PF5   with CTL_D: AC to the RMS chip, or through the 1 kHz filter
 *    CTL_D  PF4     AC, no filter: D high, E low. AC, filter: D low, E high.
 *                   Not AC: E high, D left as it was.
 *    CTL_B  PC11  U16 Y: 1.5 nF from the HY's PB0 to input ground      low in ohms
 *    DCmV_CTL PD7 only to U14, not fitted                              low in AC volts
 *    LED_CK PE15  U9 Y and Z                                           high, but for the HV diode test
 *    ENB    PF6   U10, the 15 V diode test supply                      low, but for the HV diode test
 *    VA_CTL PA11  only to U14, not fitted                              low in AC VA
 *    TEMP_CTL PH2 U16 Z: VDD through 21 Mohm to the V jack, open lead detection  high in temperature
 *    FRE_CTL PA12 U9 X: the V jack to the HY's CNT input               low in Hz, ms, duty
 *
 *  pathLevels() is the rule, pure, for testing. Paths<TGpio> applies it;
 *  TGpio::set(Signal, bool high) is the board's (Gw121L1Gpio.h on the meter).
 */

#ifndef GW121_PATHS_H_
#define GW121_PATHS_H_

#include <stdint.h>
#include "iMeter.h"

namespace gw121 {

enum Signal : uint8_t { CTL_A, CTL_B, CTL_D, CTL_E, DCmV_CTL, LED_CK, ENB, VA_CTL, TEMP_CTL, FRE_CTL, kSignalCount };

struct Levels {
	uint16_t write;   // signals to set
	uint16_t high;    // of those, the ones high
	bool written(Signal s) const { return (write >> s) & 1u; }
	bool isHigh(Signal s) const { return (high >> s) & 1u; }
};

inline bool isAc(meter::Function f) {
	using F = meter::Function;
	return f == F::AcVolts || f == F::AcDcVolts || f == F::AcMilliVolts ||
	       f == F::AcAmps || f == F::AcMilliAmps || f == F::AcMicroAmps;
}

// lpf: the AC 1 kHz low pass filter is on (volts and amps only, as in the
// stock firmware). hvDiode: the 15 V diode test.
inline Levels pathLevels(meter::Function f, bool lpf = false, bool hvDiode = false) {
	using F = meter::Function;
	Levels l{0, 0};
	auto set = [&l](Signal s, bool high) { l.write |= (uint16_t)(1u << s); if (high) l.high |= (uint16_t)(1u << s); };
	if (!isAc(f)) {
		set(CTL_A, true);
		set(CTL_E, true);
	} else if (lpf) {
		set(CTL_A, false); set(CTL_D, false); set(CTL_E, true);
	} else {
		set(CTL_A, false); set(CTL_E, false); set(CTL_D, true);
	}
	set(CTL_B, f != F::Ohms);
	set(DCmV_CTL, !(f == F::AcVolts || f == F::AcDcVolts));
	const bool hv = (f == F::Diode) && hvDiode;
	set(LED_CK, !hv);
	set(ENB, hv);
	set(VA_CTL, true);   // AC VA isn't a function here yet
	set(TEMP_CTL, f == F::Temperature);
	set(FRE_CTL, !(f == F::Frequency || f == F::DutyCycle));
	return l;
}

template <typename TGpio>
class Paths : public meter::iMeterPaths {
public:
	template <typename... TArgs>
	explicit Paths(TArgs&&... gpioArgs) : _gpio(static_cast<TArgs&&>(gpioArgs)...) {}

	void select(meter::Function f, uint8_t range) override {
		(void)range;
		const Levels l = pathLevels(f, _lpf, _hvDiode);
		for (uint8_t s = 0; s < kSignalCount; s++)
			if (l.written((Signal)s)) _gpio.set((Signal)s, l.isHigh((Signal)s));
	}

	// Take effect at the next select().
	void setLowPass(bool on) { _lpf = on; }
	void setHighVoltageDiode(bool on) { _hvDiode = on; }
	TGpio& gpio() { return _gpio; }

private:
	TGpio _gpio;
	bool  _lpf = false;
	bool  _hvDiode = false;
};

} // namespace gw121

#endif /* GW121_PATHS_H_ */
