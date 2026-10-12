/*
 * stm32_rtc.h
 *
 *  The STM32L432's own RTC as the board's clock: kept in UTC, on the LSE
 *  watch crystal, with PREDIV_S + 1 = 4096 sub-second steps (244 us).
 *  Set from GPS (or network) time, it is what the display shows and what
 *  the NTP server serves.
 *
 *  Every access holds rtc_mutex (created in main()). Reads are short; a
 *  set enters the RTC's init mode and applies a sub-second shift, a few
 *  hundred microseconds at most.
 */

#ifndef STM32_RTC_H_
#define STM32_RTC_H_

#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"

// Where the time in the RTC last came from, kept in a backup register so
// it survives a reset (not a power cycle: on the NUCLEO-L432KC VBAT is
// tied to VDD).
enum class RtcSource : uint8_t { None = 0, Gps = 1, Network = 2, Ds1307 = 3 };

void StmRtc_Begin(RTC_HandleTypeDef *hrtc, osMutexId_t mutex);

// The calendar has been set since the RTC last lost power.
bool StmRtc_IsSet(void);

// UTC now: Unix seconds and the fraction of a second in units of 2^-32
// (NTP's). false: not set, or the RTC is busy beyond timeoutMs.
bool StmRtc_Read(uint32_t *unixSeconds, uint32_t *fraction, uint32_t timeoutMs);

// Sets the RTC to unixSeconds + fraction (2^-32 units) as of now: the
// calendar is written at a whole second, then shifted forward by the
// fraction. Records the source and time in the backup registers.
bool StmRtc_Set(uint32_t unixSeconds, uint32_t fraction, RtcSource source);

// The last set recorded in the backup registers (survives a reset).
bool StmRtc_LastSet(RtcSource *source, uint32_t *unixSeconds);

// Resolution as a power of two in seconds (-12 for 1/4096 s), for NTP.
int8_t StmRtc_PrecisionLog2(void);

#endif /* STM32_RTC_H_ */
