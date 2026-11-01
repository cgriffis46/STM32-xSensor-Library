/*
 * Hy3131Regs.h
 *
 *  Registers, bits and SPI framing of the Hycon HY3131 (and HY3130), a
 *  50,000 / 5,000 count DMM analog front end. From Hycon's datasheet
 *  DS-HY3131-V10_EN (2014): register list (section 8), data registers (9),
 *  interrupts (10), clock (11), function network (12), ADCs (13) and the
 *  frequency counter (14). Not checked against a chip.
 *
 *  SPI (section 7): mode 0 (CK idles low, DI/DO change as CK falls and are
 *  latched as it rises), MSB first. CK high >= 20 ns, low >= 60 ns, DO valid
 *  60 ns after the falling edge: a few MHz is safe.
 *
 *    Write: [addr<6:0> 0] [data] [data] ...      byte aligned
 *    Read:  [addr<6:0> 1] then ONE blank clock (the "SPI read period"), then
 *           the data. Every data byte on DO is a bit late: it straddles two
 *           SPI bytes. Clock len + 1 bytes after the command and put each
 *           data byte back together with unshiftRead().
 *
 *  The address increments after every byte, reading or writing, until CS
 *  goes high.
 *
 *  While CS is high, DO is the interrupt output: high while any flag in INTF
 *  with its enable in INTE is set. So DO is driven all the time, and the
 *  HY3131 cannot share MISO with another SPI device without a buffer.
 */

#ifndef HY3131_REGS_H_
#define HY3131_REGS_H_

#include <stdint.h>

namespace hy3131 {

// ---- SPI framing ----

inline constexpr uint8_t cmdWrite(uint8_t addr) { return (uint8_t)((addr & 0x7F) << 1); }
inline constexpr uint8_t cmdRead(uint8_t addr)  { return (uint8_t)(((addr & 0x7F) << 1) | 1u); }

// rx: the len + 1 bytes clocked in after the command byte. The first bit of
// rx is the blank one; data byte i is the next eight. out may be rx.
inline void unshiftRead(const uint8_t* rx, uint8_t* out, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) out[i] = (uint8_t)((rx[i] << 1) | (rx[i + 1] >> 7));
}

// ---- Registers ----

enum Reg : uint8_t {
	kAd1      = 0x00,   // AD1<23:0>, 3 bytes, LSB first
	kAd2      = 0x03,   // AD2<23:0> (19 bits, sign extended to 24)
	kLpf      = 0x06,   // LPF<23:0> (19 bits, sign extended)
	kRms      = 0x09,   // RMS<39:0> (38 bits, sign extended to 40), 5 bytes
	kPkhMin   = 0x0E,   // PKHMIN<23:0>
	kPkhMax   = 0x11,   // PKHMAX<23:0>
	kCtsta    = 0x14,   // counter / comparator status. Reading it clears CTBOV.
	kCtc      = 0x15,   // CTC<23:0>: SYSCLKs the input was high
	kCtb      = 0x18,   // CTB<23:0>: whole input cycles counted
	kCta      = 0x1B,   // CTA<23:0>: the gate counter. CTA<23:8> is written for the gate time.
	kIntf     = 0x1E,   // flags. Reading it clears it.
	kInte     = 0x1F,   // interrupt enables
	kR20      = 0x20,   // first configuration register
	kR21 = 0x21, kR22 = 0x22, kR23 = 0x23, kR24 = 0x24, kR25 = 0x25, kR26 = 0x26, kR27 = 0x27,
	kR28 = 0x28, kR29 = 0x29, kR2A = 0x2A, kR2B = 0x2B, kR2C = 0x2C, kR2D = 0x2D, kR2E = 0x2E,
	kR2F = 0x2F, kR30 = 0x30, kR31 = 0x31, kR32 = 0x32, kR33 = 0x33,
	kR34      = 0x34,   // HY3131 only (AD3)
	kR35      = 0x35,   // HY3131 only (AD3 inputs)
	kR36      = 0x36,   // SPHACAL, HY3131 only
	kR37      = 0x37,   // test mode: write 0, or kResetValue to reset the chip
};

static constexpr uint8_t kResetValue  = 0x60;   // written to R37: the IC resets (section 11)
static constexpr uint8_t kCfgFirst    = kR20;
static constexpr uint8_t kCfgCount    = kR36 - kR20 + 1;   // 0x20..0x36: 23 registers
static constexpr uint8_t kDataLen     = kPkhMin - kAd1;    // 0x00..0x0D: AD1, AD2, LPF, RMS
static constexpr uint8_t kPeakLen     = kCtsta - kPkhMin;  // 0x0E..0x13
static constexpr uint8_t kCountLen    = kInte - kCtsta;    // 0x14..0x1E: CTSTA, CTC, CTB, CTA, INTF

inline constexpr uint8_t cfgIndex(uint8_t reg) { return (uint8_t)(reg - kCfgFirst); }

// Bits of each configuration register that the datasheet defines (the
// cells marked "-", "0" and "O" are left out). Only these are compared
// when the configuration is read back.
static constexpr uint8_t kCfgDefinedHy3131[kCfgCount] = {
	0xFE, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0xBF, 0xFF,   // 20..27
	0x77, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   // 28..2F
	0xFF, 0xFF, 0xFF, 0xFF, 0xBC, 0xFC, 0xFF,         // 30..36
};

// ---- INTF / INTE (0x1E / 0x1F) ----

static constexpr uint8_t kIntBor  = 0x80;   // INTF only: VDD fell below 1.9 V. No enable, no IRQ.
static constexpr uint8_t kIntRms  = 0x10;
static constexpr uint8_t kIntLpf  = 0x08;
static constexpr uint8_t kIntAd1  = 0x04;
static constexpr uint8_t kIntAd2  = 0x02;   // AD2 and AD3 (HY3131) share it
static constexpr uint8_t kIntCt   = 0x01;   // frequency counter finished
static constexpr uint8_t kIntAll  = kIntRms | kIntLpf | kIntAd1 | kIntAd2 | kIntCt;

// ---- CTSTA (0x14) ----

static constexpr uint8_t kCtstaPcnti = 0x80;   // CNT pin, through its buffer
static constexpr uint8_t kCtstaAcpo  = 0x40;   // comparator output after the latch
static constexpr uint8_t kCtstaCmpho = 0x20;
static constexpr uint8_t kCtstaCmplo = 0x10;
static constexpr uint8_t kCtstaCtbov = 0x01;   // CTB overflowed: the gate was too long, the count is no good

// ---- R20: comparator input, counter ----
static constexpr uint8_t kR20ScmpiShift = 5;   // SCMPI<2:0>
static constexpr uint8_t kR20Encmp      = 0x10;
static constexpr uint8_t kR20Encnti     = 0x08;   // CNT buffer on; counter counts PCNTI (else ACPO)
static constexpr uint8_t kR20Enpcmpo    = 0x04;
static constexpr uint8_t kR20Enctr      = 0x02;   // counter on. 0 clears CTA<7:0>, CTB, CTC and CTBOV.
// R21: SCMPRH<3:0> (7:4), SCMPRL<3:0> (3:0)

// ---- R22: AD1 offset, chopper, OSR ----
static constexpr uint8_t kR22Ad1osShift   = 5;   // AD1OS<2:0>: DC offset, see table in 13.1
static constexpr uint8_t kR22Ad1chopShift = 3;   // AD1CHOP<1:0>
static constexpr uint8_t kR22Ad1osrMask   = 0x07; // AD1OSR<2:0>: OSR1 = 256 << n

// ---- R23: AD1 enable, reference gain, buffers ----
static constexpr uint8_t kR23Enad1    = 0x80;
static constexpr uint8_t kR23Ad1rg    = 0x10;   // reference gain 0.333 (else 1.0)
static constexpr uint8_t kR23Ad1rhbuf = 0x08;
static constexpr uint8_t kR23Ad1rlbuf = 0x04;
static constexpr uint8_t kR23Ad1ipbuf = 0x02;
static constexpr uint8_t kR23Ad1inbuf = 0x01;

// ---- R24: AD1 pre-filter inputs ----
static constexpr uint8_t kR24Sad1fpShift = 4;   // SAD1FP<3:0>
static constexpr uint8_t kR24Sdio        = 0x08;
static constexpr uint8_t kR24Sad1fnMask  = 0x07; // SAD1FN<2:0>

// ---- R25: gains, ACM, OP inputs ----
static constexpr uint8_t kR25Ad2igShift = 6;   // AD2IG<1:0>: 0.5, 1.0, 1.5, 2.0
static constexpr uint8_t kR25Ad1igShift = 4;   // AD1IG<1:0>: 0.9, 1.8, 2.7, 3.6
static constexpr uint8_t kR25SacmShift  = 2;   // SACM<1:0>: ACM voltage (1.5 V recommended)
static constexpr uint8_t kR25Ops2       = 0x02;
static constexpr uint8_t kR25Ops1       = 0x01;

// ---- R26: AD2 ----
static constexpr uint8_t kR26Enad2      = 0x80;
static constexpr uint8_t kR26Enchopad2  = 0x20;
static constexpr uint8_t kR26Ad2rg      = 0x10;
static constexpr uint8_t kR26Sad2clk    = 0x08;   // AD2CLK = SYSCLK/4 (else /2)
static constexpr uint8_t kR26Ad2osrMask = 0x07;   // AD2OSR<2:0>: OSR2 = 32 << n, at most 1024

// R27: SAD2IP<1:0> (7:6), SAD2IN<1:0> (5:4), SAD2RH<1:0> (3:2), SAD2RL<1:0> (1:0)
// R28: SAD1RH<2:0> (6:4), SAD1RL<2:0> (2:0)

// ---- R29: LPF, RMS, peak hold ----
static constexpr uint8_t kR29Enrms        = 0x80;
static constexpr uint8_t kR29Enlpf        = 0x40;
static constexpr uint8_t kR29LpfbwShift   = 3;    // LPFBW<2:0>: OSR4 = 128 << n
static constexpr uint8_t kR29Enpkh        = 0x04;
static constexpr uint8_t kR29PkhselMask   = 0x03; // 00 AD2, 01 AD1<23:5>, 1x LPF

// R2A..R2E: PS/DS/FS/SS switches of PA<9:0>, two pins a register
// R2F: ENVS (7), SMODE<6:0>
static constexpr uint8_t kR2FEnvs  = 0x80;
// R30: SREFO (7), ACC<6:0>
static constexpr uint8_t kR30Srefo = 0x80;

// ---- R31: references ----
static constexpr uint8_t kR31Enrefo      = 0x80;
static constexpr uint8_t kR31Enbias      = 0x40;
static constexpr uint8_t kR31SagndShift  = 4;    // SAGND<1:0>: AGND at 0.5, 0.3, 0.1 of VDDA, or floating
static constexpr uint8_t kR31SfuvrMask   = 0x0F;

// R32: ENOP2 (7), SOP2P<2:0> (6:4), ENOP1 (3), SOP1P<2:0> (2:0)

// ---- R33: clock, pre-filter, AD1 input ----
static constexpr uint8_t kR33Op1chopShift = 6;
static constexpr uint8_t kR33Enosc        = 0x20;   // crystal oscillator on
static constexpr uint8_t kR33Enxi         = 0x10;   // SYSCLK from XIN (else the oscillator)
static constexpr uint8_t kR33Sft1Shift    = 2;
static constexpr uint8_t kR33Sad1iMask    = 0x03;   // 00 pre-filter, 01 FB/RLU, 10 TS1P/TS2N, 11 TS1N/TS2P

// ---- R34 (HY3131): AD3, output select ----
static constexpr uint8_t kR34Enad3     = 0x80;
static constexpr uint8_t kR34Enchopad3 = 0x20;
static constexpr uint8_t kR34Ad3rg     = 0x10;
static constexpr uint8_t kR34Svxi      = 0x08;   // RMS register gives sum(XY)/N (with SDO23 = 0)
static constexpr uint8_t kR34Sdo23     = 0x04;   // AD2, LPF and RMS registers show AD3 rather than AD2
// R35 (HY3131): SAD3IP<1:0> (7:6), SAD3IN<1:0> (5:4), AD3IG<1:0> (3:2)
// R36 (HY3131): SPHACAL<7:0>, AD2/AD3 phase. Set before ENAD2/ENAD3.

// ---- Clocks ----

static constexpr uint32_t kXtalHz = 4915200;   // the datasheet's crystal

inline constexpr uint32_t ad1RateMilliHz(uint32_t sysclkHz, uint8_t ad1osr) {
	return (uint32_t)(((uint64_t)sysclkHz * 1000u / 30u) >> (8 + (ad1osr & 7)));
}

// ---- Values ----

inline int32_t signExtend24(uint32_t v) { return (int32_t)(v << 8) >> 8; }

inline int32_t le24(const uint8_t* b) {
	return signExtend24((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16));
}

inline uint32_t le24u(const uint8_t* b) {
	return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

inline int64_t le40(const uint8_t* b) {
	uint64_t v = 0;
	for (int i = 4; i >= 0; i--) v = (v << 8) | b[i];
	return (int64_t)(v << 24) >> 24;
}

} // namespace hy3131

#endif /* HY3131_REGS_H_ */
