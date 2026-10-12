/*
 * stm32_rtc.cpp
 *
 *  The STM32 RTC as a UTC clock with sub-seconds. See stm32_rtc.h.
 *
 *  Register behaviour from RM0394 (STM32L41xxx/42xxx/43xxx/44xxx/45xxx/
 *  46xxx reference manual), RTC chapter:
 *  - Reading RTC_SSR locks RTC_TR and RTC_DR (the shadow registers) until
 *    RTC_DR is read, so SSR, TR, DR read in that order belong together
 *    (BYPSHAD = 0, the reset value).
 *  - SS counts down from PREDIV_S; the fraction of the second is
 *    (PREDIV_S - SS) / (PREDIV_S + 1). After a shift SS can exceed
 *    PREDIV_S, and then the time is one second less than TR/DR say.
 *  - Leaving init mode restarts the prescalers: the second starts then.
 *  - A shift with ADD1S set and SUBFS = s moves the clock forward by
 *    1 - s / (PREDIV_S + 1) seconds.
 */

#include "stm32_rtc.h"
#include "stm32_rtc_math.h"

// stm32_rtc_math.h's field positions, against the device header
static_assert(rtcmath::kTrSecPos == RTC_TR_SU_Pos && rtcmath::kTrMinPos == RTC_TR_MNU_Pos
              && rtcmath::kTrHourPos == RTC_TR_HU_Pos, "RTC_TR field positions");
static_assert(rtcmath::kDrDayPos == RTC_DR_DU_Pos && rtcmath::kDrMonthPos == RTC_DR_MU_Pos
              && rtcmath::kDrWdPos == RTC_DR_WDU_Pos && rtcmath::kDrYearPos == RTC_DR_YU_Pos, "RTC_DR field positions");
static_assert(RTC_TR_PM == (1u << 22) && RTC_SHIFTR_ADD1S == (1u << 31), "RTC_TR.PM and RTC_SHIFTR.ADD1S");

namespace {

RTC_HandleTypeDef *s_hrtc = nullptr;
osMutexId_t s_mutex = nullptr;

const uint32_t kBkpMagic = 0x6E74700u;   // "ntp", in BKP0R's upper bits with the source below
const uint32_t kSetTimeoutMs = 20;

uint32_t predivS() { return s_hrtc->Instance->PRER & RTC_PRER_PREDIV_S; }

bool lock(uint32_t timeoutMs) {
	return s_mutex != nullptr && osMutexAcquire(s_mutex, timeoutMs) == osOK;
}
void unlock() { osMutexRelease(s_mutex); }

} // namespace

void StmRtc_Begin(RTC_HandleTypeDef *hrtc, osMutexId_t mutex) {
	s_hrtc = hrtc;
	s_mutex = mutex;
}

bool StmRtc_IsSet(void) {
	return s_hrtc != nullptr && __HAL_RTC_IS_CALENDAR_INITIALIZED(s_hrtc);
}

int8_t StmRtc_PrecisionLog2(void) {
	return rtcmath::precisionLog2(predivS());
}

bool StmRtc_Read(uint32_t *unixSeconds, uint32_t *fraction, uint32_t timeoutMs) {
	if (!StmRtc_IsSet() || !lock(timeoutMs)) return false;
	const uint32_t ssr = s_hrtc->Instance->SSR;   // locks TR and DR...
	const uint32_t tr = s_hrtc->Instance->TR;
	const uint32_t dr = s_hrtc->Instance->DR;     // ...until DR is read
	unlock();
	return rtcmath::fromRegisters(tr, dr, ssr, predivS(), unixSeconds, fraction);
}

bool StmRtc_Set(uint32_t unixSeconds, uint32_t fraction, RtcSource source) {
	uint32_t tr, dr;
	if (!rtcmath::toRegisters(unixSeconds, &tr, &dr) || !lock(kSetTimeoutMs)) return false;

	// Date and time in one init session: the second starts as it ends.
	bool ok = true;
	__HAL_RTC_WRITEPROTECTION_DISABLE(s_hrtc);
	if (RTC_EnterInitMode(s_hrtc) != HAL_OK) {
		ok = false;
	} else {
		s_hrtc->Instance->TR = tr & RTC_TR_RESERVED_MASK;   // 24 hour: PM clear
		s_hrtc->Instance->DR = dr & RTC_DR_RESERVED_MASK;
		CLEAR_BIT(s_hrtc->Instance->CR, RTC_CR_BKP);        // no daylight saving
		ok = RTC_ExitInitMode(s_hrtc) == HAL_OK;
	}
	__HAL_RTC_WRITEPROTECTION_ENABLE(s_hrtc);
	s_hrtc->State = HAL_RTC_STATE_READY;

	// Then forward by the fraction.
	const uint32_t subfs = rtcmath::shiftSubFs(fraction, predivS());
	if (ok && subfs != 0)
		ok = HAL_RTCEx_SetSynchroShift(s_hrtc, RTC_SHIFTADD1S_SET, subfs) == HAL_OK;

	if (ok) {
		HAL_RTCEx_BKUPWrite(s_hrtc, RTC_BKP_DR0, (kBkpMagic << 4) | (uint32_t)source);
		HAL_RTCEx_BKUPWrite(s_hrtc, RTC_BKP_DR1, unixSeconds);
	}
	unlock();
	return ok;
}

bool StmRtc_LastSet(RtcSource *source, uint32_t *unixSeconds) {
	if (!StmRtc_IsSet()) return false;
	const uint32_t r0 = HAL_RTCEx_BKUPRead(s_hrtc, RTC_BKP_DR0);
	if ((r0 >> 4) != kBkpMagic) return false;
	*source = (RtcSource)(r0 & 0x0F);
	*unixSeconds = HAL_RTCEx_BKUPRead(s_hrtc, RTC_BKP_DR1);
	return *source != RtcSource::None;
}
