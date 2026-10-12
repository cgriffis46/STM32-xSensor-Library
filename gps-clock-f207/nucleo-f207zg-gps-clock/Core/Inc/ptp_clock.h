/*
 * ptp_clock.h
 *
 *  The board's clock: the Ethernet MAC's IEEE 1588 system time (seconds
 *  and 2^-31 s sub-seconds, counted from HCLK with a fine-update addend;
 *  Core/Inc/ptp_math.h has the arithmetic). It holds UTC as Unix seconds,
 *  which is what the NTP server serves (PTP itself would want TAI: a later
 *  step, with hardware timestamps). The F207's own RTC has no sub-seconds
 *  (RM0033: no RTC_SSR, RTC_SHIFTR or RTC_CALR, checked in ST's
 *  stm32f207xx.h), so it only carries whole seconds across a reset
 *  (Core/Inc/f2_rtc.h).
 *
 *  HAL_ETH_Init() resets the MAC, system time included: call
 *  PtpClock_Begin() after it, and never call HAL_ETH_Init() again
 *  (ethernetif.c follows the link without it).
 *
 *  Reads are lock-free and may come from any thread or interrupt. Set and
 *  frequency changes come from one thread (clockTask).
 */

#ifndef PTP_CLOCK_H_
#define PTP_CLOCK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the system time at 0, running ppb parts per billion fast.
   hclkHz: HCLK (the fine update needs it; 120 MHz here). */
bool PtpClock_Begin(uint32_t hclkHz, int32_t ppb);
bool PtpClock_Running(void);

/* Now, as Unix seconds and an NTP fraction (2^-32 s). false: not started. */
bool PtpClock_Read(uint32_t *unixSeconds, uint32_t *fraction);

/* As PtpClock_Read, and *counterAt = *counter read in the same instant
   (interrupts masked across both). */
bool PtpClock_ReadWithCounter(uint32_t *unixSeconds, uint32_t *fraction, volatile const uint32_t *counter,
                              uint32_t *counterAt);

/* Sets the clock to unixSeconds + fraction as of timer count `at` (TIM2,
   1 MHz): what is loaded is that plus the microseconds the counter has
   run since, worked out with interrupts masked right before the load. */
bool PtpClock_SetAt(uint32_t unixSeconds, uint32_t fraction, volatile const uint32_t *counter, uint32_t at);

/* Runs the clock ppb parts per billion fast (negative: slow), to about
   half a ppb. */
bool PtpClock_SetFrequencyPpb(int32_t ppb);
int32_t PtpClock_FrequencyPpb(void);

/* The clock's resolution as a power of two in seconds, for NTP: the
   sub-seconds step by the increment (43 x 2^-31 s, about 20 ns): -25. */
int8_t PtpClock_PrecisionLog2(void);

#ifdef __cplusplus
}
#endif

#endif /* PTP_CLOCK_H_ */
