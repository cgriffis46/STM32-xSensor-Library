/*
 * f2_rtc.h
 *
 *  The STM32F207's RTC, on the LSE, as what carries the time across a
 *  reset: whole seconds only. The F2's RTC has no sub-second register, no
 *  shift and no smooth calibration (only a coarse one), so the clock that
 *  is served is the Ethernet MAC's (ptp_clock.h). This one is started at
 *  a second boundary of that clock, and read back at start-up by waiting
 *  for its next tick, so after a reset the clock is right to about a
 *  millisecond before GPS time comes back.
 *
 *  The backup registers (kept with the RTC while VDD or VBAT has power;
 *  the NUCLEO-F207ZG ties VBAT to VDD, so a reset keeps them and a power
 *  cycle doesn't: general knowledge of UM1974) hold where the time came
 *  from, when, and the PTP clock's frequency correction.
 *
 *  Called from clockTask only (no lock).
 */

#ifndef F2_RTC_H_
#define F2_RTC_H_

#include <stdint.h>
#include "main.h"

enum class ClockSource : uint8_t { None = 0, Gps = 1, GpsPps = 4 };   // the F405's RtcSource numbers

void F2Rtc_Begin(RTC_HandleTypeDef *hrtc);

// The calendar has been set since the RTC last lost power.
bool F2Rtc_IsSet(void);

// The Unix second the RTC is in. false: not set.
bool F2Rtc_ReadSeconds(uint32_t *unixSeconds);

// Waits, spinning, for the RTC's next second to begin (at most 1.1 s):
// *unixSeconds is that second and *counterAt the counter's value as it
// was seen to begin (to within the 61 us the shadow registers take to
// follow). false: not set, or no tick.
bool F2Rtc_WaitForTick(uint32_t *unixSeconds, volatile const uint32_t *counter, uint32_t *counterAt);

// Loads unixSecond into the calendar and lets it start when *counter
// reaches `at` (1 MHz counts, 2 ms to 1.1 s ahead): the RTC's second then
// begins about 4 LSE cycles later (122 us; general knowledge). Sleeps in
// osDelay until a millisecond before, then spins with interrupts masked.
bool F2Rtc_StartSecondAt(uint32_t unixSecond, volatile const uint32_t *counter, uint32_t at);

// Where the time came from, and its Unix second then (backup registers).
void F2Rtc_NoteSource(ClockSource source, uint32_t unixSeconds);
bool F2Rtc_LastSet(ClockSource *source, uint32_t *unixSeconds);

// The PTP clock's frequency correction, kept for the next start.
void F2Rtc_SaveFrequencyPpb(int32_t ppb);
bool F2Rtc_FrequencyPpb(int32_t *ppb);

#endif /* F2_RTC_H_ */
