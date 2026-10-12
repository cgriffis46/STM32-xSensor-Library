/*
 * stm32_rtc_math.h
 *
 *  The arithmetic behind stm32_rtc.cpp, with no HAL, so it can be tested
 *  on a PC (test/stm32_rtc_math_test.cpp). Field positions are those of
 *  RTC_TR, RTC_DR and RTC_SHIFTR in RM0394; stm32_rtc.cpp checks them
 *  against the CMSIS device header with static_assert.
 */

#ifndef STM32_RTC_MATH_H_
#define STM32_RTC_MATH_H_

#include <stdint.h>
#include "ds1307.h"   // date helpers: ds1307_time_t, ds1307_to_unix / ds1307_from_unix / ds1307_valid

namespace rtcmath {

// RTC_TR: SU[3:0] ST[6:4] MNU[11:8] MNT[14:12] HU[19:16] HT[21:20] PM[22]
// RTC_DR: DU[3:0] DT[5:4] MU[11:8] MT[12] WDU[15:13] YU[19:16] YT[23:20]
const uint32_t kTrSecPos = 0, kTrMinPos = 8, kTrHourPos = 16;
const uint32_t kDrDayPos = 0, kDrMonthPos = 8, kDrWdPos = 13, kDrYearPos = 16;

inline uint32_t bcd2(uint32_t v) { return ((v >> 4) & 0x0F) * 10 + (v & 0x0F); }
inline uint32_t toBcd(uint32_t v) { return ((v / 10) << 4) | (v % 10); }

// TR/DR (24 hour) and SSR, as Unix seconds and a 2^-32 fraction. SS counts
// down from predivS; after a shift it can be above predivS, and then the
// time is one second less than TR/DR say. false: not a real date and time.
inline bool fromRegisters(uint32_t tr, uint32_t dr, uint32_t ssr, uint32_t predivS,
                          uint32_t *unixSeconds, uint32_t *fraction) {
	DS1307::ds1307_time_t t;
	t.year = (uint16_t)(2000 + bcd2((dr >> kDrYearPos) & 0xFF));
	t.month = (uint8_t)bcd2((dr >> kDrMonthPos) & 0x1F);
	t.day = (uint8_t)bcd2((dr >> kDrDayPos) & 0x3F);
	t.hour = (uint8_t)bcd2((tr >> kTrHourPos) & 0x3F);
	t.minute = (uint8_t)bcd2((tr >> kTrMinPos) & 0x7F);
	t.second = (uint8_t)bcd2((tr >> kTrSecPos) & 0x7F);
	t.weekday = 0;
	if (!DS1307::ds1307_valid(t)) return false;
	uint32_t secs = DS1307::ds1307_to_unix(t);
	uint32_t into;   // sub-second steps since the second began
	if (ssr <= predivS) {
		into = predivS - ssr;
	} else {
		secs -= 1;
		into = 2 * predivS + 1 - ssr;
		if (into > predivS) into = predivS;
	}
	*unixSeconds = secs;
	*fraction = (uint32_t)(((uint64_t)into << 32) / (predivS + 1));
	return true;
}

// Unix seconds as TR and DR values (24 hour; weekday Monday 1 .. Sunday 7).
// false: outside 2000..2099.
inline bool toRegisters(uint32_t unixSeconds, uint32_t *tr, uint32_t *dr) {
	DS1307::ds1307_time_t t;
	DS1307::ds1307_from_unix(unixSeconds, &t);
	if (!DS1307::ds1307_valid(t)) return false;
	const uint32_t wd = t.weekday == 0 ? 7 : t.weekday;
	*tr = (toBcd(t.hour) << kTrHourPos) | (toBcd(t.minute) << kTrMinPos) | (toBcd(t.second) << kTrSecPos);
	*dr = (toBcd(t.year - 2000) << kDrYearPos) | (wd << kDrWdPos) | (toBcd(t.month) << kDrMonthPos)
	      | (toBcd(t.day) << kDrDayPos);
	return true;
}

// To move the clock forward by `fraction` (2^-32 units) of a second with a
// shift: ADD1S and SUBFS = the returned value (1 - fraction, in sub-second
// steps). 0: no shift needed (less than half a step).
inline uint32_t shiftSubFs(uint32_t fraction, uint32_t predivS) {
	const uint32_t steps = predivS + 1;
	const uint32_t fwd = (uint32_t)(((uint64_t)fraction * steps + 0x80000000u) >> 32);   // steps forward, rounded
	if (fwd == 0 || fwd >= steps) return 0;
	return steps - fwd;   // 1 .. predivS
}

// -log2 of the step, rounded: 4096 steps a second is -12.
inline int8_t precisionLog2(uint32_t predivS) {
	uint32_t steps = predivS + 1;
	int8_t p = 0;
	while (steps > 1) { steps >>= 1; --p; }
	return p;
}

} // namespace rtcmath

#endif /* STM32_RTC_MATH_H_ */
