// Host test of BitBangSpiBlockTransport: against a pin-level HY3131 (the
// simulated chip of isensor/hy3131/test, driven edge by edge as section 7 of
// the datasheet describes), and with hy3131_dmm on top, as on the 121GW.

#include <cstdio>
#include "BitBangSpiBlockTransport.h"
#include "hy3131.h"
#include "SimHy3131.h"

using namespace hy3131sim;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

// The chip's side of the four wires.
struct PinBus {
	SimChip* chip = nullptr;
	bool cs = true, ck = false, din = true;
	int  bit = 0;            // clocks since CS fell
	uint8_t cmd = 0, addr = 0, wbyte = 0, cur = 0;
	bool reading = false;
	int  csCycles = 0;
	int  edgesWhileCsHigh = 0;

	void rise() {
		if (cs) { edgesWhileCsHigh++; return; }
		if (bit < 8) {
			cmd = (uint8_t)((cmd << 1) | din);
			if (bit == 7) { addr = cmd >> 1; reading = cmd & 1; }
		} else if (!reading) {
			wbyte = (uint8_t)((wbyte << 1) | din);
			if ((bit - 8) % 8 == 7) chip->writeReg(addr++, wbyte);
		} else if (bit >= 9 && (bit - 9) % 8 == 0) {
			cur = chip->readReg(addr++, false);   // its first bit is going out
		}
	}
	bool dout() const {
		if (cs) return chip->doHigh();            // the IRQ output
		if (!reading || bit < 9) return false;    // the command, and the blank clock
		return (cur >> (7 - (bit - 9) % 8)) & 1;
	}
};

struct SimPins {
	PinBus* bus;
	void ckHigh() { if (!bus->ck) { bus->ck = true; bus->rise(); } }
	void ckLow()  { if (bus->ck) { bus->ck = false; if (!bus->cs) bus->bit++; } }
	void csLow()  { bus->cs = false; bus->bit = 0; bus->cmd = 0; bus->wbyte = 0; bus->reading = false; bus->csCycles++; }
	void csHigh() { bus->cs = true; }
	void di(bool b) { bus->din = b; }
	bool doLevel() const { return bus->dout(); }
	void delay() const {}
	static bool doLevelOf(void* self) { return static_cast<PinBus*>(self)->dout(); }
};

static void testTransport() {
	printf("bit-banged reads and writes, with the HY3131's blank clock\n");
	SimChip chip;
	PinBus bus; bus.chip = &chip;
	BitBangSpiBlockTransport<SimPins> t(SimPins{&bus});
	CHECK(bus.cs && !bus.ck);

	const uint8_t whdr = hy3131::cmdWrite(hy3131::kR22);
	const uint8_t v[3] = { 0x13, 0x8A, 0x05 };
	CHECK(t.beginWrite(&whdr, 1, v, 3));
	CHECK(!t.isBusy() && !t.lastOpFailed());
	CHECK(chip.reg[0x22] == 0x13 && chip.reg[0x23] == 0x8A && chip.reg[0x24] == 0x05);
	CHECK(bus.cs);

	const uint8_t rhdr = hy3131::cmdRead(hy3131::kR22);
	uint8_t raw[4] = {}, out[3] = {};
	CHECK(t.beginRead(&rhdr, 1, raw, 4));   // 3 wanted, + 1 for the blank clock
	hy3131::unshiftRead(raw, out, 3);
	CHECK(out[0] == 0x13 && out[1] == 0x8A && out[2] == 0x05);
	CHECK((raw[0] & 0x80) == 0);             // the blank bit

	uint8_t dummy;
	CHECK(!t.beginRead(nullptr, 1, &dummy, 1));
	CHECK(!t.beginWrite(&whdr, 0, nullptr, 0));
	CHECK(bus.edgesWhileCsHigh == 0);
}

static void testDriver() {
	printf("hy3131_dmm on the bit-banged transport, IRQ from DO's level\n");
	SimChip chip;
	PinBus bus; bus.chip = &chip;
	hy3131_param_t p = hy3131_default_param();
	p.doLevel = &SimPins::doLevelOf;
	p.doCtx = &bus;
	p.pollMs = 1000;                         // so the level does the work
	hy3131_dmm<BitBangSpiBlockTransport<SimPins>> dmm(p, SimPins{&bus});
	chip.ad1Value = -2468;
	dmm.start(hy3131::defaultConfig());
	for (uint32_t now = 0; now < 1500; now++) {
		chip.tick(now);
		for (int k = 0; k < 8; k++) dmm.main(now);
	}
	CHECK(dmm.running());
	CHECK(dmm.faults() == 0);
	hy3131::Sample s{};
	CHECK(dmm.latest(&s));
	CHECK(s.ad1 == -2468);
	CHECK(s.seq >= 6);                       // 5 Hz for over a second, found by the level, not the 1 s poll
	CHECK(chip.lostFlags == 0);
}

int main() {
	testTransport();
	testDriver();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
