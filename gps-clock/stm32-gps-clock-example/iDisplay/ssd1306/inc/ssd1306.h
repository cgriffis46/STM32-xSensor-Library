/*
 * ssd1306.h
 *
 *  SSD1306 128x64 / 128x32 monochrome OLED, non-blocking state machine.
 *
 *  The bus is chosen by the template argument. ssd1306<TTransport, H>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      ssd1306<Stm32HalI2CTransport, 64> oled(param, &hi2c1, SSD1306_I2C_ADDR, i2c1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  It is also a MonoCanvas (iDisplay/inc) with its frame buffer inside,
 *  and an iDisplayDevice, which is what the GUI (iDisplay/gui) drives:
 *
 *      oled.clear();
 *      oled.printAt(0, 0, "Temp: "); oled.print(t, 1);
 *      oled.requestFlush();
 *      while (!oled.idle() && !oled.failed()) oled.main(now());
 *
 *  Over I2C each transfer is a register write whose "register" is the
 *  SSD1306's control byte: 0x00 for commands, 0x40 for display data.
 *  writeRegs() sends at most ISensorTransport::kMaxWriteLen (32) bytes,
 *  so a 128 byte page goes out in four transfers. For the 4-wire SPI
 *  wiring, where a D/C pin takes the control byte's place, see
 *  ssd1306_spi below.
 *
 *  What it does: on the first flush (and after any failure), sends the
 *  power-up sequence in one transfer and turns the display on. Each
 *  flush sends only the pages that changed since they were last sent;
 *  a hash of every page that was sent is kept, so a screen that is
 *  cleared and redrawn the same costs no bus time. Contrast, inverse
 *  and display on/off are sent between flushes.
 *
 *  What it does not do: the hardware scroll, the reset pin (the
 *  FeatherWing and most I2C modules tie it to an RC reset; drive it in
 *  your board code before the first flush if yours has one), and the
 *  SH1106, which looks the same but has a 132 column RAM (set
 *  param.column_offset = 2 and it mostly works, untested).
 *
 *  The command values are from the Solomon Systech SSD1306 datasheet
 *  rev 1.1 and the power-up sequence from its application note. Not
 *  yet run against a chip.
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include <stdint.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"
#include "MonoCanvas.h"
#include "iDisplayDevice.h"

namespace SSD1306 {

#define SSD1306_I2C_ADDR 0x3C		// 7 bit. 0x3D with SA0 high
#define SSD1306_WIDTH 128

// Control bytes, I2C only
#define SSD1306_CONTROL_COMMAND 0x00
#define SSD1306_CONTROL_DATA 0x40

// Commands
#define SSD1306_SET_CONTRAST 0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_NORMAL_DISPLAY 0xA6
#define SSD1306_INVERT_DISPLAY 0xA7
#define SSD1306_DISPLAY_OFF 0xAE
#define SSD1306_DISPLAY_ON 0xAF
#define SSD1306_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_SET_COM_PINS 0xDA
#define SSD1306_SET_VCOM_DETECT 0xDB
#define SSD1306_SET_DISPLAY_CLOCK_DIV 0xD5
#define SSD1306_SET_PRECHARGE 0xD9
#define SSD1306_SET_MULTIPLEX 0xA8
#define SSD1306_SET_LOW_COLUMN 0x00
#define SSD1306_SET_HIGH_COLUMN 0x10
#define SSD1306_SET_START_LINE 0x40
#define SSD1306_MEMORY_MODE 0x20
#define SSD1306_PAGE_ADDRESSING 0x02
#define SSD1306_SET_PAGE 0xB0
#define SSD1306_COM_SCAN_INC 0xC0
#define SSD1306_COM_SCAN_DEC 0xC8
#define SSD1306_SEG_REMAP_0 0xA0
#define SSD1306_SEG_REMAP_127 0xA1
#define SSD1306_CHARGE_PUMP 0x8D
#define SSD1306_DEACTIVATE_SCROLL 0x2E

typedef enum ssd1306_state_t{
	ssd1306_init = 0,				// send the power-up sequence
	ssd1306_wait_init,
	ssd1306_idle,					// nothing to do until a flush or a setting is asked for
	ssd1306_command,				// send a pending setting (contrast, inverse, on/off)
	ssd1306_wait_command,
	ssd1306_next_page,				// find the next page that changed
	ssd1306_page_address,			// point the chip at it
	ssd1306_wait_page_address,
	ssd1306_page_data,				// send it, 32 bytes at a time
	ssd1306_wait_page_data,
	ssd1306_error
}ssd1306_state_t;

typedef struct ssd1306_param_t{
	uint8_t contrast;				// 0..255. 0xCF is the usual for 128x64, 0x8F for 128x32
	bool external_vcc;				// true: VCC from a 7-15 V supply. false: the internal charge pump (most modules)
	bool rotate_180;				// mount it the other way up
	uint8_t column_offset;			// 0 for SSD1306. 2 for an SH1106
}ssd1306_param_t;

// The usual settings for a module of the given height.
inline ssd1306_param_t ssd1306_default_param(uint8_t height) {
	ssd1306_param_t p = {};
	p.contrast = height == 32 ? 0x8F : 0xCF;
	p.external_vcc = false;
	p.rotate_180 = false;
	p.column_offset = 0;
	return p;
}

template <typename TTransport, uint8_t H = 64>
class ssd1306 : public SensorStateMachine<TTransport, ssd1306_state_t>,
				public idisplay::MonoCanvas,
				public idisplay::iDisplayDevice {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
		"ssd1306 needs an ISensorTransport (I2CTransport, SPITransport, ...)");
	static_assert(H == 32 || H == 64, "ssd1306 is 128x32 or 128x64");

	using Base = SensorStateMachine<TTransport, ssd1306_state_t>;

public:
	static constexpr uint8_t kPages = H / 8;
	static constexpr uint8_t kChunk = ISensorTransport::kMaxWriteLen;
	static_assert(SSD1306_WIDTH % kChunk == 0, "a page must split into whole transfers");
	static constexpr uint8_t kChunksPerPage = SSD1306_WIDTH / kChunk;
	static constexpr uint32_t kBusTimeoutMs = 100;
	static constexpr uint32_t kErrorBackoffMs = 500;

	template <typename... TArgs>
	explicit ssd1306(const ssd1306_param_t& param, TArgs&&... transportArgs);

	// iDisplayDevice
	idisplay::iTextSurface& text() override { return *this; }
	void requestFlush() override { flush_wanted = true; }
	void main(uint32_t nowMs) override;
	bool idle() const override;
	bool failed() const override { return this->_state == ssd1306_error; }

	// Settings, sent at the next main() between flushes. They survive a
	// failure: the power-up sequence is followed by them again.
	void setContrast(uint8_t contrast);
	void setInverted(bool inverted);	// light background, dark text
	void setDisplayOn(bool on);			// off keeps the RAM, and draws ~10 uA

	bool initialized() const { return is_initialized; }

protected:
	// How a run of commands or of display data is put on the bus. I2C:
	// one register write, the control byte standing in for the register.
	// ssd1306_spi overrides these for the D/C pin.
	virtual bool sendCommands(const uint8_t* cmds, uint8_t len) {
		return this->writeRegs(SSD1306_CONTROL_COMMAND, cmds, len);
	}
	virtual bool sendData(const uint8_t* data, uint8_t len) {
		return this->writeRegs(SSD1306_CONTROL_DATA, data, len);
	}

	void onFail() override;

private:
	uint8_t buildInitSequence(uint8_t* out) const;
	uint8_t buildPendingCommands(uint8_t* out);
	static uint32_t hashPage(const uint8_t* page);

	ssd1306_param_t param;
	uint8_t fb[SSD1306_WIDTH * kPages];

	bool is_initialized = false;
	bool flush_wanted = false;
	bool resend_all = true;				// after power-up or a failure, every page is sent

	// Settings waiting to be sent.
	bool contrast_pending = false, invert_pending = false, power_pending = false;
	bool inverted = false, display_on = true;

	uint8_t cmd_buf[ISensorTransport::kMaxWriteLen];
	uint8_t cmd_len = 0;

	uint8_t dirty = 0;					// bit p: page p is to be sent in this flush
	uint8_t page = 0, chunk = 0;
	uint32_t page_hash[kPages];			// of the page as drawn when this flush began
	uint32_t sent_hash[kPages];			// of the page as last sent
};

// 4-wire SPI: the D/C pin says whether a byte is a command (low) or
// display data (high), and there is no control byte. setDc is called
// with true for data, false for commands, just before each transfer is
// issued; the previous one has always landed by then, so the pin never
// changes under a transfer of this display's. The SPI transport drives
// chip-select.
//
//      static void oledDc(bool data) { HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, data ? GPIO_PIN_SET : GPIO_PIN_RESET); }
//      ssd1306_spi<Stm32HalSPITransport, 64> oled(param, oledDc, &hspi1, OLED_CS_GPIO_Port, OLED_CS_Pin, spi1Mutex);
template <typename TTransport, uint8_t H = 64>
class ssd1306_spi : public ssd1306<TTransport, H> {
public:
	typedef void (*dc_fn_t)(bool data);

	template <typename... TArgs>
	ssd1306_spi(const ssd1306_param_t& p, dc_fn_t setDc, TArgs&&... transportArgs)
		: ssd1306<TTransport, H>(p, std::forward<TArgs>(transportArgs)...), set_dc(setDc) {}

protected:
	bool sendCommands(const uint8_t* cmds, uint8_t len) override {
		if (this->isBusy()) return false;
		set_dc(false);
		return this->writeBytes(cmds, len);
	}
	bool sendData(const uint8_t* data, uint8_t len) override {
		if (this->isBusy()) return false;
		set_dc(true);
		return this->writeBytes(data, len);
	}

private:
	dc_fn_t set_dc;
};

} /* namespace SSD1306 */

#include "../src/ssd1306.tpp"

#endif /* SSD1306_H_ */
