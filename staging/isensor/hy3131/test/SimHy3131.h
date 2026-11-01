// A simulated HY3131 and an iBlockTransport to it, for host tests (the
// driver's own, and imeter's). See hy3131_test.cpp for what it models.
#pragma once
#include <cassert>
#include <cstring>
#include <functional>
#include <vector>
#include "iBlockTransport.h"
#include "Hy3131Regs.h"

namespace hy3131sim {
using namespace hy3131;


struct SimChip {
	uint8_t reg[0x38];
	bool present = true;
	uint32_t resets = 0;
	uint32_t nowMs = 0;
	uint32_t lastAd1Ms = 0;
	int32_t ad1Value = -12345;
	std::function<int32_t(const uint8_t* reg)> ad1Model;   // if set: AD1 from the configuration
	int32_t ad2Value = 0x3FFFF;
	int64_t rmsValue = 123456789012LL;
	// counter
	bool counting = false;
	uint32_t countStartMs = 0;
	uint32_t ctbResult = 1000;
	bool ctbovSet = false;
	// what went wrong
	uint32_t lostFlags = 0;        // INTF flags cleared by a read the driver didn't mean
	uint32_t lostCtbov = 0;
	bool eventEveryRead = false;   // an AD1 result lands as each data read starts
	std::vector<std::pair<uint8_t, std::vector<uint8_t>>> writes;

	SimChip() { reset(); }
	void reset() { memset(reg, 0, sizeof reg); resets++; counting = false; }

	uint32_t sysclk() const { return kXtalHz; }
	bool oscOn() const { return reg[kR33] & kR33Enosc; }

	void writeReg(uint8_t a, uint8_t v) {
		if (a >= sizeof reg) return;
		if (a == kR37) { if (v == kResetValue) reset(); return; }
		if (a == kIntf) { reg[kIntf] &= v; return; }   // 0s only
		if (a == kR20) {
			const bool was = reg[kR20] & kR20Enctr, now = v & kR20Enctr;
			if (was && !now) { reg[kCta] = 0; memset(&reg[kCtb], 0, 3); memset(&reg[kCtc], 0, 3); reg[kCtsta] &= ~kCtstaCtbov; counting = false; }
			if (!was && now) { counting = true; countStartMs = nowMs; }
		}
		reg[a] = v;
	}

	// The first bit of register a is going out. partial: the read ends inside it.
	uint8_t readReg(uint8_t a, bool partial) {
		if (a >= sizeof reg) return 0;
		const uint8_t v = reg[a];
		if (a == kIntf) {
			if (partial && (v & (kIntAll | kIntBor))) lostFlags++;
			reg[kIntf] = 0;
		}
		if (a == kCtsta) {
			if (partial && (v & kCtstaCtbov)) lostCtbov++;
			reg[kCtsta] &= ~kCtstaCtbov;
		}
		return v;
	}

	void write(uint8_t cmd, const uint8_t* d, size_t n) {
		uint8_t a = cmd >> 1;
		writes.push_back({a, std::vector<uint8_t>(d, d + n)});
		for (size_t i = 0; i < n; i++) writeReg(a++, d[i]);
	}

	// n bytes clocked after the command: 1 blank bit, then 8n - 1 bits of registers.
	void read(uint8_t cmd, uint8_t* out, size_t n) {
		uint8_t a = cmd >> 1;
		if (eventEveryRead && a != kIntf && (reg[kR23] & kR23Enad1)) reg[kIntf] |= kIntAd1;
		const size_t bits = 8 * n;
		std::vector<uint8_t> stream(bits, 0);
		size_t pos = 1;                       // the blank bit
		while (pos < bits) {
			const bool partial = pos + 8 > bits;
			const uint8_t v = readReg(a++, partial);
			for (int b = 7; b >= 0 && pos < bits; b--) stream[pos++] = (v >> b) & 1;
		}
		for (size_t i = 0; i < n; i++) {
			uint8_t x = 0;
			for (int b = 0; b < 8; b++) x = (uint8_t)((x << 1) | stream[8 * i + b]);
			out[i] = x;
		}
	}

	void set24(uint8_t a, uint32_t v) { reg[a] = v; reg[a + 1] = v >> 8; reg[a + 2] = v >> 16; }

	void tick(uint32_t ms) {
		nowMs = ms;
		if (!oscOn()) return;
		const uint8_t osr = reg[kR22] & 7;
		const uint32_t periodMs = 1000000u / ad1RateMilliHz(sysclk(), osr);
		if ((reg[kR23] & kR23Enad1) && ms - lastAd1Ms >= periodMs) {
			lastAd1Ms = ms;
			if (ad1Model) ad1Value = ad1Model(reg);
			set24(kAd1, (uint32_t)ad1Value & 0xFFFFFF);
			set24(kAd2, (uint32_t)ad2Value & 0xFFFFFF);
			for (int i = 0; i < 5; i++) reg[kRms + i] = (uint8_t)(rmsValue >> (8 * i));
			set24(kPkhMin, (uint32_t)-500 & 0xFFFFFF);
			set24(kPkhMax, 700);
			reg[kIntf] |= kIntAd1;
		}
		if (counting) {
			const uint32_t ctaInit = ((uint32_t)reg[kCta + 2] << 16) | ((uint32_t)reg[kCta + 1] << 8);
			const uint32_t gateMs = (uint32_t)((uint64_t)(0x1000000u - ctaInit) * 1000u / sysclk());
			if (ctbovSet && ms - countStartMs >= gateMs / 2) reg[kCtsta] |= kCtstaCtbov;
			if (ms - countStartMs >= gateMs + 1) {
				counting = false;
				set24(kCtb, ctbResult);
				set24(kCtc, ctbResult * 10);
				set24(kCta, 1234);
				reg[kIntf] |= kIntCt;
			}
		}
	}

	bool doHigh() const { return (reg[kIntf] & reg[kInte] & kIntAll) != 0; }
};

// ---- the transport ----

class SimTransport : public iBlockTransport {
public:
	SimChip* chip = nullptr;
	explicit SimTransport(SimChip* c) : chip(c) {}
	bool beginWrite(const uint8_t* h, uint8_t hl, const uint8_t* d, size_t n) override {
		assert(hl == 1 && !(h[0] & 1));
		if (chip->present) chip->write(h[0], d, n);
		return true;
	}
	bool beginRead(const uint8_t* h, uint8_t hl, uint8_t* d, size_t n) override {
		assert(hl == 1 && (h[0] & 1));
		if (chip->present) chip->read(h[0], d, n);
		else memset(d, 0xFF, n);
		return true;
	}
	bool isBusy() const override { return false; }
	bool lastOpFailed() const override { return false; }
};


static inline bool doLevel(void* ctx) { return static_cast<SimChip*>(ctx)->doHigh(); }

} // namespace hy3131sim
