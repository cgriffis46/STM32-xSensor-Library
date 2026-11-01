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
#include "Gw121Lcd.h"
#include "Gw121Bluetooth.h"
#include "MeterTags.h"
#include "Stm32L1Lcd.h"
#include "Gw121Keys.h"

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

// The LCD as an HMI: the meter's tags and the board's, on the glass.
static idisplay::Stm32L1Lcd glassLcd;
static gw121::LcdLayout layout;
static bool btOn = true, battLow = false;
alignas(meter::MeterTags) static unsigned char tagsMem[sizeof(meter::MeterTags)];
struct BoardTags : idisplay::iTagSource {
	int32_t find(const char* n) override { return n[0] == 'b' && n[1] == 't' ? 0 : (n[0] == 'b' ? 1 : -1); }
	bool read(int32_t h, idisplay::xTagValue& o) override { o = idisplay::xTagValue::boolean(h == 0 ? btOn : battLow); return true; }
};
static BoardTags boardTags;
static idisplay::iTagSource* tagSources[2];
alignas(idisplay::xTagChain) static unsigned char chainMem[sizeof(idisplay::xTagChain)];
alignas(idisplay::xSegmentHmi) static unsigned char hmiMem[sizeof(idisplay::xSegmentHmi)];

void gw121LcdStart() {
	glassLcd.start(idisplay::Stm32L1Lcd::Config{});
	tagSources[0] = new (tagsMem) meter::MeterTags(*reinterpret_cast<meter::xMeter*>(meterMem));
	tagSources[1] = &boardTags;
	auto* chain = new (chainMem) idisplay::xTagChain(tagSources, 2);
	new (hmiMem) idisplay::xSegmentHmi(glassLcd, *chain, layout.fields(), layout.count());
}
void gw121LcdTick() { reinterpret_cast<idisplay::xSegmentHmi*>(hmiMem)->refresh(); }

// A Bluetooth packet of the latest reading, for the UART.
size_t gw121BtPacket(uint8_t* out) {
	meter::Reading r;
	if (!reinterpret_cast<meter::xMeter*>(meterMem)->latest(&r)) return 0;
	return gw121::bt::encode(gw121::bt::frameOf(r, gw121::bt::MeterState{}), out);
}

// Keys and buzzer: sampled every 10 ms, the buzzer ticked every 1 ms;
// a key becomes tag writes, as a PLC's or the phone's would.
static idisplay::KeyScanner keys(gw121::keyConfig());
static idisplay::Buzzer<gw121::BuzzerPin> buzzer;
alignas(gw121::Controller) static unsigned char ctlMem[sizeof(gw121::Controller)];
static gw121::bt::Parser btParser;

void gw121KeysStart() {
	new (ctlMem) gw121::Controller(*reinterpret_cast<idisplay::xTagChain*>(chainMem), &buzzer);
}
void gw121Tick1ms(uint32_t now) { buzzer.tick(now); }
void gw121KeysTick10ms() { keys.sample(gw121::keysFromIdr(gw121::keysIdr())); }
void gw121KeysPoll() {
	idisplay::KeyEvent e;
	if (keys.poll(&e)) reinterpret_cast<gw121::Controller*>(ctlMem)->onKey(e);
}
void gw121BtByte(uint8_t b) {
	gw121::bt::Command c;
	if (btParser.feed(b, &c)) reinterpret_cast<gw121::Controller*>(ctlMem)->onBluetooth(c);
}
