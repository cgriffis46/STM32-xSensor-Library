// Compiled by staging/cross/Makefile only: the whole stack, instantiated
// for the Cortex-M3 (STM32L152xD) the way a 121GW-style meter would use it.
// It checks that the stack compiles and shows its size. It is not a firmware.
#include "Stm32L1SpiBlockTransport.h"
#include "FreeRtosTransport.h"
#include "xhy3131.h"
#include "Hy3131FrontEnd.h"
#include "xMeter.h"

using DmmSpi = Stm32L1SpiBlockTransport<FreeRtosTransport<SpiBlockTransport>>;
using Dmm    = xhy3131<DmmSpi>;

static void dcv(hy3131::Config& c) { c.ad1Input(1); }
static const meter::Hy3131Route kRoutes[] = {
	{ meter::Function::DcVolts, 0, { 5.0f, 1e-4f, 0, meter::Unit::Volt, 4, "5V" }, dcv, meter::Hy3131Source::Ad1, 1 },
};

static Stm32L1SpiPort spi2(SPI2);
alignas(Dmm) static unsigned char dmmMem[sizeof(Dmm)];
alignas(meter::Hy3131FrontEnd<Dmm>) static unsigned char feMem[sizeof(meter::Hy3131FrontEnd<Dmm>)];
alignas(meter::xMeter) static unsigned char meterMem[sizeof(meter::xMeter)];

extern "C" void SPI2_IRQHandler(void) { spi2.irq(); }

void gw121Start(osMutexId_t spi2Mutex) {
	hy3131_param_t p = hy3131_default_param();
	p.irq = true;
	Dmm* dmm = new (dmmMem) Dmm(p, spi2, GPIOB, (uint16_t)(1u << 12), spi2Mutex);
	auto* fe = new (feMem) meter::Hy3131FrontEnd<Dmm>(*dmm, kRoutes, 1);
	auto* m  = new (meterMem) meter::xMeter(*fe);
	m->setFunction(meter::Function::DcVolts);
}
void gw121DmmThread(uint32_t now)   { reinterpret_cast<Dmm*>(dmmMem)->main(now); }
void gw121MeterThread(uint32_t now) { reinterpret_cast<meter::xMeter*>(meterMem)->main(now); }
void gw121DoIrq()                   { reinterpret_cast<Dmm*>(dmmMem)->onIrq(); }
