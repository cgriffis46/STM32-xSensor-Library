/*
 * Gw121Calibration.h
 *
 *  gw121::StockCalibration: an iCalibrationStore that reads the 121GW's
 *  factory calibration where the stock firmware keeps it, in the STM32L152's
 *  data EEPROM, so firmware of our own measures as well as the stock one.
 *  It only reads. Nothing here writes that EEPROM: the factory calibration
 *  is the one thing on the meter that can't be put back.
 *
 *  The layout, from the decompiled stock firmware (121gw-re: get_cal_factor_addr(),
 *  cal_load_factors_for_mode_and_range(), read_eeprom_4byte(); and
 *  routines/cal_decode.py):
 *
 *    address = cal_factor_eeloc_tbl[stock mode] + 16 * stock range
 *    +0  offset  int32, big endian, in AD1 counts
 *    +4  gain    uint32, big endian, G = gain / 1e6 (capacitance: / 1e5)
 *    stock firmware: digits = (AD1 - offset) / G
 *
 *  and the frequency reference at cal_factor_eeloc_tbl[MM_HZ] + 4: the
 *  HY3131's SYSCLK in tenths of a Hz (40,000,000..60,000,000, otherwise
 *  the stock firmware uses 49,152,050).
 *
 *  The data EEPROM is at 0x08080000 on the STM32L1, readable as memory.
 *  The reading goes through iByteReader, so a test (or a dump of the EEPROM
 *  on an SD card) can stand in for it.
 */

#ifndef GW121_CALIBRATION_H_
#define GW121_CALIBRATION_H_

#include <stdint.h>
#include <stddef.h>
#include "iMeter.h"
#include "Gw121Routes.h"

namespace gw121 {

class iByteReader {
public:
	virtual ~iByteReader() = default;
	// offset: from the start of the data EEPROM.
	virtual bool read(uint32_t offset, uint8_t* dst, size_t n) = 0;
};

// The data EEPROM, memory mapped. base: 0x08080000 on the STM32L1.
class MappedEeprom : public iByteReader {
public:
	explicit MappedEeprom(uintptr_t base = 0x08080000u, uint32_t size = 12u * 1024u) : _base(base), _size(size) {}
	bool read(uint32_t offset, uint8_t* dst, size_t n) override {
		if (offset + n > _size) return false;
		const volatile uint8_t* p = reinterpret_cast<const volatile uint8_t*>(_base + offset);
		for (size_t i = 0; i < n; i++) dst[i] = p[i];
		return true;
	}
private:
	uintptr_t _base;
	uint32_t  _size;
};

// cal_factor_eeloc_tbl[], by stock mode (MM_LOW_Z .. MM_DCVA, and 25: 1 ms peak).
inline constexpr uint16_t kCalBase[26] = {
	48, 112, 48, 176, 208, 240, 256, 256, 256, 272, 288, 384, 400,
	768, 832, 896, 496, 528, 560, 592, 624, 672, 960, 1024, 1088, 720,
};

class StockCalibration : public meter::iCalibrationStore {
public:
	// How far a factor may be from the nominal before it is taken for
	// garbage (an erased or foreign EEPROM) and ignored.
	static constexpr float kGainTolerance = 0.25f;
	static constexpr int32_t kMaxOffset = 200000;

	explicit StockCalibration(iByteReader& eeprom) : _eeprom(eeprom) {}

	bool load(meter::Function f, uint8_t range, meter::Calibration* out) override {
		const RangeCal* rc = find(f, range);
		if (!rc) return false;
		const uint32_t addr = (uint32_t)kCalBase[rc->stockMode] + 16u * rc->stockRange;
		uint8_t b[8];
		if (!_eeprom.read(addr, b, sizeof b)) return false;
		const int32_t  offset = (int32_t)be32(b);
		const uint32_t gainRaw = be32(b + 4);

		if (rc->stockMode == kMM_HZ) {
			// The frequency reference: no offset, gain = actual SYSCLK / the
			// 4.9152 MHz the front end assumes (Hy3131FrontEnd's sysclkHz).
			if (gainRaw < 40000000u || gainRaw > 60000000u) return false;
			*out = meter::Calibration{ (float)((double)gainRaw / (double)(hy3131::kXtalHz * 10u)), 0.0f, true };
			return true;
		}
		if (rc->nominalCountsPerDigit <= 0.0f || gainRaw == 0u || gainRaw == 0xFFFFFFFFu) return false;
		const float g = (float)gainRaw / 1e6f;
		const float ratio = rc->nominalCountsPerDigit / g;
		if (ratio < 1.0f - kGainTolerance || ratio > 1.0f + kGainTolerance) return false;
		if (offset > kMaxOffset || offset < -kMaxOffset) return false;
		*out = meter::Calibration{ ratio, (float)offset, true };
		return true;
	}

	// The factory calibration is never written from here.
	bool store(meter::Function, uint8_t, const meter::Calibration&) override { return false; }

	static const RangeCal* find(meter::Function f, uint8_t range) {
		for (uint8_t i = 0; i < kCalCount; i++)
			if (kCal[i].function == f && kCal[i].range == range) return &kCal[i];
		return nullptr;
	}

private:
	static uint32_t be32(const uint8_t* b) {
		return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
	}

	iByteReader& _eeprom;
};

} // namespace gw121

#endif /* GW121_CALIBRATION_H_ */
