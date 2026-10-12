/*
 * stm32_rtc.h
 *
 *  The STM32L432's own RTC as the board's clock: kept in UTC, on the LSE
 *  watch crystal, with PREDIV_S + 1 = 8192 sub-second steps (122 us).
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
enum class RtcSource : uint8_t { None = 0, Gps = 1, Network = 2, Ds1307 = 3, GpsPps = 4 };

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

// As StmRtc_Read, and *counterAt = *counter read in the same instant
// (interrupts masked across both): for measuring the RTC against a timer.
bool StmRtc_ReadWithCounter(uint32_t *unixSeconds, uint32_t *fraction, volatile const uint32_t *counter,
                            uint32_t *counterAt, uint32_t timeoutMs);

// Moves the RTC back by errUs microseconds (forward when negative; |errUs|
// under a second) with a sub-second shift, to the nearest step. false: too
// small to shift, too big, or the RTC busy.
bool StmRtc_ShiftUs(int64_t errUs);

// The RTC's smooth calibration (RTC_CALR, 32 s cycle) in parts per billion:
// positive runs it faster. Kept by the RTC across a reset.
bool StmRtc_SetCalibrationPpb(int32_t ppb);
int32_t StmRtc_CalibrationPpb(void);

// Records in the backup registers where the time now comes from, without
// setting it (the PPS loop keeps the RTC right with shifts).
void StmRtc_NoteSource(RtcSource source, uint32_t unixSeconds);

// The last set recorded in the backup registers (survives a reset).
bool StmRtc_LastSet(RtcSource *source, uint32_t *unixSeconds);

// Resolution as a power of two in seconds (-13 for 1/8192 s), for NTP.
int8_t StmRtc_PrecisionLog2(void);

#endif /* STM32_RTC_H_ */
