/*
 * Mcp2515Regs.h
 *
 *  The Microchip MCP2515's SPI instructions, registers and bits, and its
 *  bit timing, from Microchip's datasheet DS21801E (2007): the instruction
 *  set (table 12-1), the register map (tables 11-1 and 11-2), bit timing
 *  (section 5), errors (section 6), interrupts (section 7) and modes
 *  (section 10). Every value was also checked against three drivers, which
 *  agree with it: Linux's drivers/net/can/spi/mcp251x.c (v6.6), Zephyr's
 *  drivers/can/can_mcp2515.{c,h} (v4.2.0) and autowp's arduino-mcp2515
 *  (MIT). Where they differ it is said below.
 *
 *  SPI: up to 10 MHz, modes 0,0 and 1,1 (the datasheet's front page).
 */

#ifndef MCP2515_REGS_H_
#define MCP2515_REGS_H_

#include <stdint.h>

namespace mcp251x {

// ---- SPI instructions (table 12-1) ----
static constexpr uint8_t kInstrReset      = 0xC0;
static constexpr uint8_t kInstrRead       = 0x03;   // address, then reads (the address counts up)
static constexpr uint8_t kInstrWrite      = 0x02;   // address, then writes
static constexpr uint8_t kInstrBitModify  = 0x05;   // address, mask, data
static constexpr uint8_t kInstrReadStatus = 0xA0;
// LOAD TX BUFFER n from TXBnSIDH: 0x40, 0x42, 0x44 (Linux: 0x40 + 2n).
inline uint8_t instrLoadTx(uint8_t n) { return (uint8_t)(0x40 + 2 * n); }
// READ RX BUFFER n from RXBnSIDH: 0x90, 0x94. Ending the read (chip-select
// high) clears the buffer's RXnIF.
inline uint8_t instrReadRx(uint8_t n) { return n == 0 ? 0x90 : 0x94; }
// REQUEST TO SEND buffer n: 0x81, 0x82, 0x84.
inline uint8_t instrRts(uint8_t n) { return (uint8_t)(0x80 | (1u << n)); }

// ---- registers (table 11-1) ----
// BIT MODIFY works on BFPCTRL, TXRTSCTRL, CANCTRL, CNF1..3, CANINTE,
// CANINTF, EFLG, TXBnCTRL and RXBnCTRL (shaded in table 11-1); on any
// other register it writes the whole byte.
static constexpr uint8_t kRegBfpCtrl  = 0x0C;
static constexpr uint8_t kRegTxRtsCtrl = 0x0D;
static constexpr uint8_t kRegCanStat  = 0x0E;
static constexpr uint8_t kRegCanCtrl  = 0x0F;
static constexpr uint8_t kRegTec      = 0x1C;
static constexpr uint8_t kRegRec      = 0x1D;
static constexpr uint8_t kRegCnf3     = 0x28;   // CNF3, CNF2, CNF1, CANINTE, CANINTF, EFLG follow in order
static constexpr uint8_t kRegCnf2     = 0x29;
static constexpr uint8_t kRegCnf1     = 0x2A;
static constexpr uint8_t kRegCanIntE  = 0x2B;
static constexpr uint8_t kRegCanIntF  = 0x2C;
static constexpr uint8_t kRegEflg     = 0x2D;
inline uint8_t regTxCtrl(uint8_t n) { return (uint8_t)(0x30 + 0x10 * n); }   // TXBnCTRL, then SIDH..D7
static constexpr uint8_t kRegRxb0Ctrl = 0x60;
static constexpr uint8_t kRegRxb1Ctrl = 0x70;

// ---- CANSTAT / CANCTRL (registers 10-1, 10-2) ----
// A mode change waits until pending transmissions are done (section 10.0),
// so leaving the bus also sets ABAT. Configuration mode clears TEC and REC.
static constexpr uint8_t kOpModeMask    = 0xE0;   // CANSTAT.OPMOD, CANCTRL.REQOP
static constexpr uint8_t kOpNormal      = 0x00;
static constexpr uint8_t kOpSleep       = 0x20;
static constexpr uint8_t kOpLoopback    = 0x40;
static constexpr uint8_t kOpListenOnly  = 0x60;
static constexpr uint8_t kOpConfig      = 0x80;
static constexpr uint8_t kCanCtrlAbat   = 0x10;   // abort all pending transmissions
static constexpr uint8_t kCanCtrlOsm    = 0x08;   // one-shot mode
static constexpr uint8_t kCanCtrlClkEn  = 0x04;   // CLKOUT pin on
// CANCTRL after reset: register 10-1 gives 1000 0111 (0x87: configuration
// mode, CLKOUT on at /8), table 11-2 1110 0111, and the text says REQOP is
// 111 on power-up. Only the bits all three agree on are checked, as Linux's
// probe does: (CANCTRL & 0x17) == 0x07 (ABAT 0, CLKEN 1, CLKPRE 11). An
// absent chip reads 0x00 or 0xFF and fails it.
static constexpr uint8_t kCanCtrlResetMask  = 0x17;
static constexpr uint8_t kCanCtrlResetValue = 0x07;

// ---- CNF1..3 (bit timing) ----
// CNF1: SJW<7:6> (SJW - 1), BRP<5:0> (prescaler - 1).
// CNF2: BTLMODE<7> (PS2 from CNF3), SAM<6> (sample three times),
//       PHSEG1<5:3> (PS1 - 1), PRSEG<2:0> (propagation segment - 1).
// CNF3: SOF<7>, WAKFIL<6>, PHSEG2<2:0> (PS2 - 1). Zephyr and arduino-mcp2515
//       put SOF and WAKFIL at bits 7 and 6; Linux defines them as 0x08 and
//       0x04 but never sets them. This driver writes both 0.
static constexpr uint8_t kCnf2BtlMode = 0x80;
static constexpr uint8_t kCnf2Sam     = 0x40;

// ---- CANINTE / CANINTF ----
static constexpr uint8_t kIntRx0  = 0x01;
static constexpr uint8_t kIntRx1  = 0x02;
static constexpr uint8_t kIntTx0  = 0x04;
static constexpr uint8_t kIntTx1  = 0x08;
static constexpr uint8_t kIntTx2  = 0x10;
static constexpr uint8_t kIntErr  = 0x20;   // EFLG changed
static constexpr uint8_t kIntWak  = 0x40;
static constexpr uint8_t kIntMerr = 0x80;   // an error during a frame
inline uint8_t intTx(uint8_t n) { return (uint8_t)(kIntTx0 << n); }

// ---- EFLG (register 6-3) ----
// Bus-off: the chip goes back to error-active by itself after 128 x 11
// recessive bits (the note in section 6.7). This driver doesn't let it: it puts
// the chip in configuration mode.
static constexpr uint8_t kEflgEwarn  = 0x01;   // TEC or REC >= 96
static constexpr uint8_t kEflgRxWar  = 0x02;
static constexpr uint8_t kEflgTxWar  = 0x04;
static constexpr uint8_t kEflgRxEp   = 0x08;   // REC >= 128
static constexpr uint8_t kEflgTxEp   = 0x10;   // TEC >= 128
static constexpr uint8_t kEflgTxBo   = 0x20;   // bus-off
static constexpr uint8_t kEflgRx0Ovr = 0x40;   // RXB0 was full: a frame lost (the MCU clears it)
static constexpr uint8_t kEflgRx1Ovr = 0x80;

// ---- TXBnCTRL ----
// Buffers of equal priority (TXP) go highest number first (section 3.2).
static constexpr uint8_t kTxCtrlAbtf  = 0x40;
static constexpr uint8_t kTxCtrlMloa  = 0x20;
static constexpr uint8_t kTxCtrlTxErr = 0x10;
static constexpr uint8_t kTxCtrlTxReq = 0x08;

// ---- RXBnCTRL (registers 4-1, 4-2) ----
// RXM<6:5> = 11: every frame, masks and filters off (Linux's setting).
// BUKT<2> (RXB0 only): a frame RXB0 has no room for goes to RXB1. RXB0CTRL
// bit 1 reads back as a copy of BUKT.
static constexpr uint8_t kRxCtrlRxmAny = 0x60;
static constexpr uint8_t kRxCtrlBukt   = 0x04;

// ---- a frame in a TX or RX buffer, from SIDH: 13 bytes ----
// SIDH  SID<10:3>
// SIDL  SID<2:0> at <7:5>, SRR<4> (RX: a standard remote frame), IDE/EXIDE<3>,
//       EID<17:16> at <1:0>
// EID8  EID<15:8>
// EID0  EID<7:0>
// DLC   RTR<6> (TX: a remote frame; RX: an extended remote frame), DLC<3:0>
// D0..D7
static constexpr uint8_t kSidlSrr  = 0x10;
static constexpr uint8_t kSidlIde  = 0x08;
static constexpr uint8_t kDlcRtr   = 0x40;
static constexpr uint8_t kFrameLen = 13;

// After RESET the oscillator start-up timer holds the chip for 128 OSC1
// cycles, and no SPI access should be tried before (section 8.1: 16 us at
// 8 MHz). Linux waits 5 ms, arduino-mcp2515 10 ms. This driver waits
// kResetMs, then reads CANSTAT until it says configuration mode.

// ---- bit timing (section 5) ----
// A bit is SyncSeg (1 TQ) + PropSeg + PS1 + PS2, a time quantum TQ is
// 2 * prescaler / Fosc. The datasheet's limits: PropSeg 1..8, PS1 1..8, PS2
// 2..8 (at least the 2 TQ information processing time), SJW 1..4, prescaler
// 1..64; PropSeg + PS1 >= PS2, PS2 > SJW. Also PropSeg + PS1 >= 3, Linux's
// tseg1_min, a little stricter than the datasheet. The datasheet's example
// samples at 60-70 % of the bit; 87.5 % is CiA's usual choice (general
// knowledge), and the caller picks.
struct Timing {
	uint8_t prescaler;   // 1..64
	uint8_t propSeg;     // 1..8
	uint8_t ps1;         // 1..8
	uint8_t ps2;         // 2..8
	uint8_t sjw;         // 1..4
	uint16_t tq() const { return (uint16_t)(1 + propSeg + ps1 + ps2); }
	uint8_t cnf1() const { return (uint8_t)(((sjw - 1) << 6) | (prescaler - 1)); }
	uint8_t cnf2() const { return (uint8_t)(kCnf2BtlMode | ((ps1 - 1) << 3) | (propSeg - 1)); }
	uint8_t cnf3() const { return (uint8_t)(ps2 - 1); }
	// Sample point in permille.
	uint16_t samplePoint() const { return (uint16_t)(1000u * (1u + propSeg + ps1) / tq()); }
};

// The exact bit rate with the sample point nearest samplePointPermille:
// of the prescalers that give a whole number of TQ, 6 to 25, in a bit, the
// one whose sample point is nearest (on a tie, the most TQ). False if
// oscHz can't make the bit rate exactly, as an 8 MHz crystal can't make
// 1 Mbit/s (4 TQ a bit, under the 6 needed). SJW is 1 ("usually enough",
// section 5.3, and all three reference drivers' default).
inline bool computeTiming(uint32_t oscHz, uint32_t bitrate, uint16_t samplePointPermille, Timing* t) {
	if (bitrate == 0 || oscHz == 0 || samplePointPermille < 500 || samplePointPermille > 950) return false;
	bool found = false;
	uint32_t bestErr = 0;
	for (uint32_t brp = 1; brp <= 64; ++brp) {
		const uint64_t div = 2ull * brp * bitrate;
		if (oscHz % div != 0) continue;
		const uint32_t n = (uint32_t)(oscHz / div);
		if (n < 6 || n > 25) continue;
		// PS2 nearest (1000 - sp) of the bit, then held to its limits
		uint32_t ps2 = (n * (1000u - samplePointPermille) + 500u) / 1000u;
		if (ps2 < 2) ps2 = 2;
		if (ps2 > 8) ps2 = 8;
		const uint32_t tseg1 = n - 1 - ps2;   // PropSeg + PS1
		if (tseg1 < 3 || tseg1 > 16 || tseg1 < ps2) continue;
		uint32_t prop = tseg1 / 2;
		uint32_t ps1 = tseg1 - prop;
		if (ps1 > 8) { ps1 = 8; prop = tseg1 - 8; }
		if (prop > 8) continue;
		const uint32_t sp = 1000u * (1u + tseg1) / n;
		const uint32_t err = sp > samplePointPermille ? sp - samplePointPermille : samplePointPermille - sp;
		if (found && err >= bestErr) continue;   // smaller prescalers (more TQ) win ties
		found = true;
		bestErr = err;
		t->prescaler = (uint8_t)brp;
		t->propSeg = (uint8_t)prop;
		t->ps1 = (uint8_t)ps1;
		t->ps2 = (uint8_t)ps2;
		t->sjw = 1;
	}
	return found;
}

} // namespace mcp251x

#endif /* MCP2515_REGS_H_ */
