/*
 * ptp_clock.cpp
 *
 *  The Ethernet MAC's system time as the clock (see ptp_clock.h). The
 *  start-up order is ST's AN3411 (ETH_PTPStart in its STM32F4x7 Ethernet
 *  library, the same MAC): mask the time stamp trigger interrupt, TSE,
 *  the sub-second increment, the addend with TSARU, fine update (TSFCU),
 *  then the time with TSSTI. Register and bit names are from ST's
 *  stm32f207xx.h; how they behave is from general knowledge and AN3411,
 *  not RM0033 (not read here). Not run on a board.
 */

#include "ptp_clock.h"
#include <atomic>
#include "main.h"
#include "ptp_math.h"

namespace {

ptpmath::Base g_base = { 0, 0 };
std::atomic<bool> g_running{false};
std::atomic<int32_t> g_ppb{0};

// TSSTI, TSSTU and TSARU clear themselves when the MAC has acted on them,
// within a few HCLK cycles; this many reads is far more than enough.
bool waitClear(uint32_t bits)
{
  for (uint32_t i = 0; i < 100000u; ++i)
    if ((ETH->PTPTSCR & bits) == 0)
      return true;
  return false;
}

void readRaw(uint32_t *sec, uint32_t *sub)
{
  uint32_t hi = ETH->PTPTSHR;
  uint32_t lo = ETH->PTPTSLR;
  const uint32_t hi2 = ETH->PTPTSHR;
  if (hi2 != hi) {          // the second rolled over between the reads
    lo = ETH->PTPTSLR;
    hi = hi2;
  }
  *sec = hi;
  *sub = lo & ETH_PTPTSLR_STSS;
}

} // namespace

bool PtpClock_Begin(uint32_t hclkHz, int32_t ppb)
{
  if (!ptpmath::baseFor(hclkHz, 50000000u, &g_base))
    return false;
  __HAL_RCC_ETHMACPTP_CLK_ENABLE();

  ETH->MACIMR |= ETH_MACIMR_TSTIM;               // no time stamp trigger interrupt
  ETH->PTPTSCR |= ETH_PTPTSCR_TSE;
  ETH->PTPSSIR = g_base.increment;
  ETH->PTPTSAR = ptpmath::addendFor(g_base, ppb);
  ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;
  if (!waitClear(ETH_PTPTSCR_TSARU))
    return false;
  ETH->PTPTSCR |= ETH_PTPTSCR_TSFCU;             // fine update: the addend sets the rate
  ETH->PTPTSHUR = 0;
  ETH->PTPTSLUR = 0;
  ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;
  if (!waitClear(ETH_PTPTSCR_TSSTI))
    return false;
  g_ppb.store(ppb);
  g_running.store(true, std::memory_order_release);
  return true;
}

bool PtpClock_Running(void)
{
  return g_running.load(std::memory_order_acquire);
}

bool PtpClock_Read(uint32_t *unixSeconds, uint32_t *fraction)
{
  if (!PtpClock_Running())
    return false;
  uint32_t sec, sub;
  readRaw(&sec, &sub);
  *unixSeconds = sec;
  *fraction = ptpmath::fracFromSub(sub);
  return true;
}

bool PtpClock_ReadWithCounter(uint32_t *unixSeconds, uint32_t *fraction, volatile const uint32_t *counter,
                              uint32_t *counterAt)
{
  if (!PtpClock_Running())
    return false;
  uint32_t sec, sub;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  readRaw(&sec, &sub);
  *counterAt = *counter;
  __set_PRIMASK(primask);
  *unixSeconds = sec;
  *fraction = ptpmath::fracFromSub(sub);
  return true;
}

bool PtpClock_SetAt(uint32_t unixSeconds, uint32_t fraction, volatile const uint32_t *counter, uint32_t at)
{
  if (!PtpClock_Running() || !waitClear(ETH_PTPTSCR_TSSTI | ETH_PTPTSCR_TSSTU))
    return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint32_t el = *counter - at;             // us since `at`
  uint64_t sub = (uint64_t)ptpmath::subFromFrac(fraction) + ((uint64_t)(el % 1000000u) << 31) / 1000000u;
  uint32_t sec = unixSeconds + el / 1000000u;
  if (sub >= ptpmath::kSubPerSecond) {
    sub -= ptpmath::kSubPerSecond;
    ++sec;
  }
  ETH->PTPTSHUR = sec;
  ETH->PTPTSLUR = (uint32_t)sub;                 // bit 31 clear: positive
  ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;
  __set_PRIMASK(primask);
  return waitClear(ETH_PTPTSCR_TSSTI);
}

bool PtpClock_SetFrequencyPpb(int32_t ppb)
{
  if (!PtpClock_Running() || !waitClear(ETH_PTPTSCR_TSARU))
    return false;
  ETH->PTPTSAR = ptpmath::addendFor(g_base, ppb);
  ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;
  if (!waitClear(ETH_PTPTSCR_TSARU))
    return false;
  g_ppb.store(ptpmath::ppbOf(g_base, ETH->PTPTSAR));
  return true;
}

int32_t PtpClock_FrequencyPpb(void)
{
  return g_ppb.load();
}

int8_t PtpClock_PrecisionLog2(void)
{
  // log2(increment x 2^-31), rounded up: 43 -> -25.4 -> -25
  int8_t p = -31;
  for (uint32_t step = 1; step < g_base.increment; step <<= 1)
    ++p;
  return p;
}
