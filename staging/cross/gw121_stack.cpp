// Compiled by staging/cross/Makefile only: the 121GW stack as it would be
// wired on the meter, built for its STM32L152ZD (Cortex-M3). It checks that
// everything compiles together and shows the size. It is not a firmware:
// no clocks, no GPIO set-up, no threads.
#include <new>
#include "BitBangSpiBlockTransport.h"
#include "Stm32L1GpioSpiPins.h"
#include "xhy3131.h"
#include "Hy3131FrontEnd.h"
#include "xMeter.h"
#include "Gw121Routes.h"
#include "Gw121Calibration.h"
#include "Gw121Paths.h"
#include "Gw121L1Gpio.h"
#include "Gw121RangeSwitch.h"

using HySpi = BitBangSpiBlockTransport<Stm32L1GpioSpiPins>;
using Dmm   = xhy3131<HySpi>;
using Fe    = meter::Hy3131FrontEnd<Dmm>;

alignas(Dmm) static unsigned char dmmMem[sizeof(Dmm)];
alignas(Fe) static unsigned char feMem[sizeof(Fe)];
alignas(meter::xMeter) static unsigned char meterMem[sizeof(meter::xMeter)];
static gw121::Paths<gw121::L1Gpio> paths;
static gw121::MappedEeprom eeprom;
static gw121::StockCalibration cal(eeprom);
static gw121::RangeSwitch rangeSwitch;
static Stm32L1GpioSpiPins doPins = gw121::hyPins();

void gw121Start() {
	hy3131_param_t p = hy3131_default_param();
	p.doLevel = &Stm32L1GpioSpiPins::doLevelOf;   // the stock firmware polls DO (PF3) too
	p.doCtx = &doPins;
	Dmm* dmm = new (dmmMem) Dmm(p, gw121::hyPins());
	Fe* fe = new (feMem) Fe(*dmm, gw121::kRoutes, gw121::kRouteCount, hy3131::kXtalHz, &paths);
	new (meterMem) meter::xMeter(*fe, &cal);
}
void gw121DmmThread(uint32_t now)   { reinterpret_cast<Dmm*>(dmmMem)->main(now); }
void gw121MeterThread(uint32_t now) { reinterpret_cast<meter::xMeter*>(meterMem)->main(now); }
void gw121Tick10ms() {
	rangeSwitch.sample(gw121::rangeSwitchIdr());
	gw121::Position pos;
	if (rangeSwitch.poll(&pos)) reinterpret_cast<meter::xMeter*>(meterMem)->setFunction(gw121::firstFunction(pos));
}
