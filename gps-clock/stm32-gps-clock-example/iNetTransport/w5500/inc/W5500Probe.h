/*
 * W5500Probe.h
 *
 *  Blocking checks of a W5500 and its wiring, for hardware bring-up:
 *  run them on a bare iBlockTransport BEFORE the driver starts, and
 *  each failure says what to look at. Not for normal operation — the
 *  w5500 driver itself never blocks.
 *
 *      W5500::w5500_probe probe(spi, millis);
 *      uint8_t v;
 *      W5500::w5500_probe::Result r = probe.version(v);
 *      printf("%s\n", W5500::w5500_probe::describe(r));
 */

#ifndef W5500PROBE_H_
#define W5500PROBE_H_

#include <stddef.h>
#include <stdint.h>
#include "iBlockTransport.h"
#include "W5500Regs.h"

namespace W5500 {

class w5500_probe {
public:
	enum class Result : uint8_t {
		Ok,
		BusError,		// the transport refused, reported failure, or never finished
		ReadsZero,		// every byte reads 0x00
		ReadsOnes,		// every byte reads 0xFF
		WrongVersion,	// something answers, but not a W5500
		NoWrite,		// reads work, writes don't stick
		Corrupt,		// a long transfer came back different
	};

	// nowMs: a free-running ms counter (HAL_GetTick, say).
	w5500_probe(iBlockTransport &spi, uint32_t (*nowMs)(), uint32_t timeoutMs = 100)
		: _spi(spi), _now(nowMs), _timeout(timeoutMs) {}

	// VERSIONR, expected 0x04.
	Result version(uint8_t &v) {
		v = 0;
		if (!xfer(false, w5500_bsb_common(), w5500_VERSIONR, &v, 1)) return Result::BusError;
		if (v == 0x00) return Result::ReadsZero;
		if (v == 0xFF) return Result::ReadsOnes;
		return v == w5500_version ? Result::Ok : Result::WrongVersion;
	}

	// Writes two patterns to SHAR (the MAC) and reads each back, then
	// puts the original back.
	Result writeRead() {
		uint8_t orig[6], buf[6];
		if (!xfer(false, w5500_bsb_common(), w5500_SHAR, orig, 6)) return Result::BusError;
		static const uint8_t pat[2][6] = {{0xA5, 0x5A, 0xC3, 0x3C, 0x0F, 0xF0}, {0x5A, 0xA5, 0x3C, 0xC3, 0xF0, 0x0F}};
		Result r = Result::Ok;
		for (int p = 0; p < 2 && r == Result::Ok; ++p) {
			for (int i = 0; i < 6; ++i) buf[i] = pat[p][i];
			if (!xfer(true, w5500_bsb_common(), w5500_SHAR, buf, 6)) return Result::BusError;
			if (!xfer(false, w5500_bsb_common(), w5500_SHAR, buf, 6)) return Result::BusError;
			for (int i = 0; i < 6; ++i) {
				if (buf[i] != pat[p][i]) r = Result::NoWrite;
			}
		}
		if (!xfer(true, w5500_bsb_common(), w5500_SHAR, orig, 6)) return Result::BusError;
		return r;
	}

	// Fills socket 0's TX buffer memory with a pattern in one transfer
	// and reads it back in one: the long (DMA) transfers, at speed.
	// scratch: len bytes the probe may use; len up to 2048.
	Result bufferTest(uint8_t *scratch, size_t len) {
		if (len > 2048) len = 2048;
		for (size_t i = 0; i < len; ++i) scratch[i] = pattern(i);
		if (!xfer(true, w5500_bsb_sock_tx(0), 0, scratch, len)) return Result::BusError;
		for (size_t i = 0; i < len; ++i) scratch[i] = 0;
		if (!xfer(false, w5500_bsb_sock_tx(0), 0, scratch, len)) return Result::BusError;
		for (size_t i = 0; i < len; ++i) {
			if (scratch[i] != pattern(i)) {
				_badAt = i;
				return Result::Corrupt;
			}
		}
		return Result::Ok;
	}

	// PHYCFGR: bit 0 link, bit 1 100 Mbps, bit 2 full duplex.
	Result phy(uint8_t &phycfgr) {
		return xfer(false, w5500_bsb_common(), w5500_PHYCFGR, &phycfgr, 1) ? Result::Ok : Result::BusError;
	}

	// The first byte bufferTest() found wrong.
	size_t badAt() const { return _badAt; }

	static const char *describe(Result r) {
		switch (r) {
		case Result::Ok:           return "ok";
		case Result::BusError:     return "SPI transfer failed or never finished: check the SPI and DMA interrupts are enabled in the NVIC";
		case Result::ReadsZero:    return "reads 0x00: MISO stuck low, or the W5500 has no power or is held in reset (RSTn low)";
		case Result::ReadsOnes:    return "reads 0xFF: MISO floating — W5500 not selected (CS wiring/pin), not powered, or MISO not connected";
		case Result::WrongVersion: return "VERSIONR is not 0x04: wrong chip, or bits shifted (SPI mode must be 0: CPOL low, CPHA 1st edge)";
		case Result::NoWrite:      return "writes don't stick: MOSI not connected, or SCK/MOSI signal problems";
		case Result::Corrupt:      return "long transfer corrupted: SPI clock too fast for the wiring, or a DMA setting (byte width, normal mode)";
		}
		return "?";
	}

private:
	static uint8_t pattern(size_t i) { return static_cast<uint8_t>((i * 7) ^ (i >> 8) ^ 0x5A); }

	bool xfer(bool write, uint8_t bsb, uint16_t addr, uint8_t *buf, size_t len) {
		const uint8_t hdr[3] = {static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr),
		                        static_cast<uint8_t>((bsb << 3) | (write ? w5500_ctl_write : 0))};
		const uint32_t t0 = _now();
		bool started;
		while (!(started = write ? _spi.beginWrite(hdr, 3, buf, len) : _spi.beginRead(hdr, 3, buf, len))) {
			if (_now() - t0 > _timeout) return false;
		}
		while (_spi.isBusy()) {
			if (_now() - t0 > _timeout) return false;
		}
		return !_spi.lastOpFailed();
	}

	iBlockTransport &_spi;
	uint32_t (*_now)();
	uint32_t _timeout;
	size_t _badAt = 0;
};

} /* namespace W5500 */

#endif /* W5500PROBE_H_ */
