/*
 * f2_rtc.cpp
 *
 *  The F207's RTC as whole seconds across a reset. See f2_rtc.h.
 *
 *  RTC behaviour (general knowledge of the STM32 RTC; the F2's TR/DR and
 *  init mode are the F4's, as ST's two HALs show):
 *  - Reading RTC_TR locks RTC_DR (the shadow registers) until RTC_DR is
 *    read, so TR then DR belong together. The shadows follow the calendar
 *    every two RTCCLK cycles (RSF).
 *  - In init mode the calendar stops; leaving it loads TR/DR and restarts
 *    the prescalers, so the second starts then.
 */

#include "f2_rtc.h"
#include "cmsis_os.h"
#include "stm32_rtc_math.h"

static_assert(rtcmath::kTrSecPos == RTC_TR_SU_Pos && rtcmath::kTrMinPos == RTC_TR_MNU_Pos
              && rtcmath::kTrHourPos == RTC_TR_HU_Pos, "RTC_TR field positions");
static_assert(rtcmath::kDrDayPos == RTC_DR_DU_Pos && rtcmath::kDrMonthPos == RTC_DR_MU_Pos
              && rtcmath::kDrWdPos == RTC_DR_WDU_Pos && rtcmath::kDrYearPos == RTC_DR_YU_Pos, "RTC_DR field positions");

namespace {

RTC_HandleTypeDef *s_hrtc = nullptr;

const uint32_t kBkpMagic = 0x6E74700u;      // "ntp", in BKP0R's upper bits with the source below
const uint32_t kFreqCheck = 0xA5C3F00Fu;    // BKP3R = BKP2R ^ this: the frequency is valid

uint32_t predivS() { return s_hrtc->Instance->PRER & RTC_PRER_PREDIV_S; }

bool readRegisters(uint32_t *tr, uint32_t *dr)
{
  *tr = s_hrtc->Instance->TR;              // locks DR...
  *dr = s_hrtc->Instance->DR;              // ...until it is read
  return true;
}

bool toUnix(uint32_t tr, uint32_t dr, uint32_t *unixSeconds)
{
  uint32_t frac;
  const uint32_t p = predivS();
  return rtcmath::fromRegisters(tr, dr, p, p, unixSeconds, &frac);   // SS = PREDIV_S: no fraction
}

} // namespace

void F2Rtc_Begin(RTC_HandleTypeDef *hrtc)
{
  s_hrtc = hrtc;
  if (F2Rtc_IsSet())
    (void)HAL_RTC_WaitForSynchro(s_hrtc);   // the shadow registers, after the reset
}

bool F2Rtc_IsSet(void)
{
  return s_hrtc != nullptr && __HAL_RTC_IS_CALENDAR_INITIALIZED(s_hrtc);
}

bool F2Rtc_ReadSeconds(uint32_t *unixSeconds)
{
  uint32_t tr, dr;
  return F2Rtc_IsSet() && readRegisters(&tr, &dr) && toUnix(tr, dr, unixSeconds);
}

bool F2Rtc_WaitForTick(uint32_t *unixSeconds, volatile const uint32_t *counter, uint32_t *counterAt)
{
  if (!F2Rtc_IsSet())
    return false;
  uint32_t tr0, dr0;
  readRegisters(&tr0, &dr0);
  const uint32_t start = *counter;
  while (*counter - start < 1100000u) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t tr, dr;
    readRegisters(&tr, &dr);
    const uint32_t at = *counter;
    __set_PRIMASK(primask);
    if (tr != tr0) {
      *counterAt = at;
      return toUnix(tr, dr, unixSeconds);
    }
  }
  return false;
}

bool F2Rtc_StartSecondAt(uint32_t unixSecond, volatile const uint32_t *counter, uint32_t at)
{
  uint32_t tr, dr;
  if (s_hrtc == nullptr || !rtcmath::toRegisters(unixSecond, &tr, &dr))
    return false;
  const int32_t ahead = (int32_t)(at - *counter);
  if (ahead < 2000 || ahead > 1100000)
    return false;
  if (ahead > 3000)
    osDelay((uint32_t)(ahead - 2000) / 1000);   // 1 ms ticks: wakes 1..2 ms before

  bool ok = true;
  __HAL_RTC_WRITEPROTECTION_DISABLE(s_hrtc);
  if (RTC_EnterInitMode(s_hrtc) != HAL_OK) {   // INITF: a couple of LSE cycles
    ok = false;
  } else {
    s_hrtc->Instance->TR = tr & RTC_TR_RESERVED_MASK;   // 24 hour: PM clear
    s_hrtc->Instance->DR = dr & RTC_DR_RESERVED_MASK;
    CLEAR_BIT(s_hrtc->Instance->CR, RTC_CR_FMT);
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    while ((int32_t)(at - *counter) > 0) {
    }
    CLEAR_BIT(s_hrtc->Instance->ISR, RTC_ISR_INIT);    // the second starts
    __set_PRIMASK(primask);
  }
  __HAL_RTC_WRITEPROTECTION_ENABLE(s_hrtc);
  s_hrtc->State = HAL_RTC_STATE_READY;
  if (ok)
    ok = HAL_RTC_WaitForSynchro(s_hrtc) == HAL_OK;
  return ok;
}

void F2Rtc_NoteSource(ClockSource source, uint32_t unixSeconds)
{
  if (s_hrtc == nullptr)
    return;
  HAL_RTCEx_BKUPWrite(s_hrtc, RTC_BKP_DR0, (kBkpMagic << 4) | (uint32_t)source);
  HAL_RTCEx_BKUPWrite(s_hrtc, RTC_BKP_DR1, unixSeconds);
}

bool F2Rtc_LastSet(ClockSource *source, uint32_t *unixSeconds)
{
  if (s_hrtc == nullptr)
    return false;
  const uint32_t r0 = HAL_RTCEx_BKUPRead(s_hrtc, RTC_BKP_DR0);
  if ((r0 >> 4) != kBkpMagic)
    return false;
  *source = (ClockSource)(r0 & 0x0F);
  *unixSeconds = HAL_RTCEx_BKUPRead(s_hrtc, RTC_BKP_DR1);
  return true;
}

void F2Rtc_SaveFrequencyPpb(int32_t ppb)
{
  if (s_hrtc == nullptr)
    return;
  HAL_RTCEx_BKUPWrite(s_hrtc, RTC_BKP_DR2, (uint32_t)ppb);
  HAL_RTCEx_BKUPWrite(s_hrtc, RTC_BKP_DR3, (uint32_t)ppb ^ kFreqCheck);
}

bool F2Rtc_FrequencyPpb(int32_t *ppb)
{
  if (s_hrtc == nullptr)
    return false;
  const uint32_t v = HAL_RTCEx_BKUPRead(s_hrtc, RTC_BKP_DR2);
  if ((v ^ kFreqCheck) != HAL_RTCEx_BKUPRead(s_hrtc, RTC_BKP_DR3))
    return false;
  *ppb = (int32_t)v;
  return true;
}
