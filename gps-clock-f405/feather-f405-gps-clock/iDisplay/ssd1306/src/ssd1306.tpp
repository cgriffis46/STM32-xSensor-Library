/*
 * ssd1306.tpp
 *
 *  Member definitions of ssd1306<TTransport, H>. Included at the end of
 *  inc/ssd1306.h; not compiled on its own.
 */

#include "../inc/ssd1306.h"

namespace SSD1306 {

template <typename TTransport, uint8_t H>
template <typename... TArgs>
ssd1306<TTransport, H>::ssd1306(const ssd1306_param_t& p, TArgs&&... transportArgs)
	: Base(ssd1306_init, ssd1306_error, kBusTimeoutMs, std::forward<TArgs>(transportArgs)...),
	  idisplay::MonoCanvas(fb, SSD1306_WIDTH, H),
	  param(p) {
	clear();
	for (uint8_t i = 0; i < kPages; ++i) {
		page_hash[i] = 0;
		sent_hash[i] = 0;
	}
}

template <typename TTransport, uint8_t H>
bool ssd1306<TTransport, H>::idle() const {
	return this->_state == ssd1306_idle && !flush_wanted
		&& !contrast_pending && !invert_pending && !power_pending;
}

template <typename TTransport, uint8_t H>
void ssd1306<TTransport, H>::setContrast(uint8_t contrast) {
	param.contrast = contrast;
	contrast_pending = true;
}

template <typename TTransport, uint8_t H>
void ssd1306<TTransport, H>::setInverted(bool inv) {
	inverted = inv;
	invert_pending = true;
}

template <typename TTransport, uint8_t H>
void ssd1306<TTransport, H>::setDisplayOn(bool on) {
	display_on = on;
	power_pending = true;
}

template <typename TTransport, uint8_t H>
void ssd1306<TTransport, H>::onFail() {
	// The chip may have lost power: start again from the power-up
	// sequence and send every page once it is back. Whatever was asked
	// for is still wanted.
	is_initialized = false;
	resend_all = true;
	if (dirty != 0) flush_wanted = true;
	dirty = 0;
}

// The power-up sequence from the datasheet's application note, in one
// transfer (26 bytes), ending with the display on.
template <typename TTransport, uint8_t H>
uint8_t ssd1306<TTransport, H>::buildInitSequence(uint8_t* out) const {
	uint8_t n = 0;
	out[n++] = SSD1306_DISPLAY_OFF;
	out[n++] = SSD1306_SET_DISPLAY_CLOCK_DIV;	out[n++] = 0x80;	// reset value: ratio 1, ~370 kHz
	out[n++] = SSD1306_SET_MULTIPLEX;			out[n++] = H - 1;
	out[n++] = SSD1306_SET_DISPLAY_OFFSET;		out[n++] = 0x00;
	out[n++] = SSD1306_SET_START_LINE | 0x00;
	out[n++] = SSD1306_CHARGE_PUMP;				out[n++] = param.external_vcc ? 0x10 : 0x14;
	out[n++] = SSD1306_MEMORY_MODE;				out[n++] = SSD1306_PAGE_ADDRESSING;
	out[n++] = param.rotate_180 ? SSD1306_SEG_REMAP_0 : SSD1306_SEG_REMAP_127;
	out[n++] = param.rotate_180 ? SSD1306_COM_SCAN_INC : SSD1306_COM_SCAN_DEC;
	out[n++] = SSD1306_SET_COM_PINS;			out[n++] = H == 32 ? 0x02 : 0x12;
	out[n++] = SSD1306_SET_CONTRAST;			out[n++] = param.contrast;
	out[n++] = SSD1306_SET_PRECHARGE;			out[n++] = param.external_vcc ? 0x22 : 0xF1;
	out[n++] = SSD1306_SET_VCOM_DETECT;			out[n++] = 0x40;
	out[n++] = SSD1306_DISPLAY_ALL_ON_RESUME;
	out[n++] = inverted ? SSD1306_INVERT_DISPLAY : SSD1306_NORMAL_DISPLAY;
	out[n++] = SSD1306_DEACTIVATE_SCROLL;
	out[n++] = display_on ? SSD1306_DISPLAY_ON : SSD1306_DISPLAY_OFF;
	return n;
}

// Every setting waiting to be sent, in one transfer. Clears the flags.
template <typename TTransport, uint8_t H>
uint8_t ssd1306<TTransport, H>::buildPendingCommands(uint8_t* out) {
	uint8_t n = 0;
	if (contrast_pending) {
		out[n++] = SSD1306_SET_CONTRAST;
		out[n++] = param.contrast;
		contrast_pending = false;
	}
	if (invert_pending) {
		out[n++] = inverted ? SSD1306_INVERT_DISPLAY : SSD1306_NORMAL_DISPLAY;
		invert_pending = false;
	}
	if (power_pending) {
		out[n++] = display_on ? SSD1306_DISPLAY_ON : SSD1306_DISPLAY_OFF;
		power_pending = false;
	}
	return n;
}

// FNV-1a over one page. A page whose hash has not changed since it was
// sent is not sent again; two different pages with the same 32 bit hash
// are not something a screen of text will meet.
template <typename TTransport, uint8_t H>
uint32_t ssd1306<TTransport, H>::hashPage(const uint8_t* p) {
	uint32_t h = 2166136261u;
	for (uint16_t i = 0; i < SSD1306_WIDTH; ++i) {
		h ^= p[i];
		h *= 16777619u;
	}
	return h;
}

template <typename TTransport, uint8_t H>
void ssd1306<TTransport, H>::main(uint32_t nowMs) {
	switch (this->_state) {
	case ssd1306_init:
		// The settings asked for so far are part of the sequence.
		contrast_pending = invert_pending = power_pending = false;
		cmd_len = buildInitSequence(cmd_buf);
		this->issued(sendCommands(cmd_buf, cmd_len), ssd1306_wait_init, nowMs);
		break;

	case ssd1306_wait_init:
		if (this->landed(nowMs)) {
			is_initialized = true;
			resend_all = true;
			this->enter(ssd1306_idle, nowMs);
		}
		break;

	case ssd1306_idle:
		if (contrast_pending || invert_pending || power_pending) {
			cmd_len = buildPendingCommands(cmd_buf);
			this->enter(ssd1306_command, nowMs);
		} else if (flush_wanted) {
			flush_wanted = false;
			dirty = 0;
			for (uint8_t p = 0; p < kPages; ++p) {
				page_hash[p] = hashPage(&fb[p * SSD1306_WIDTH]);
				if (resend_all || page_hash[p] != sent_hash[p]) dirty |= (uint8_t)(1u << p);
			}
			resend_all = false;
			page = 0;
			this->enter(ssd1306_next_page, nowMs);
		}
		break;

	case ssd1306_command:
		this->issued(sendCommands(cmd_buf, cmd_len), ssd1306_wait_command, nowMs);
		break;

	case ssd1306_wait_command:
		if (this->landed(nowMs)) this->enter(ssd1306_idle, nowMs);
		break;

	case ssd1306_next_page:
		while (page < kPages && !(dirty & (1u << page))) ++page;
		if (page >= kPages) {
			dirty = 0;
			this->enter(ssd1306_idle, nowMs);
		} else {
			// Page addressing mode: the page, then the column to start at.
			const uint8_t col = param.column_offset;
			cmd_buf[0] = (uint8_t)(SSD1306_SET_PAGE | page);
			cmd_buf[1] = (uint8_t)(SSD1306_SET_LOW_COLUMN | (col & 0x0F));
			cmd_buf[2] = (uint8_t)(SSD1306_SET_HIGH_COLUMN | (col >> 4));
			cmd_len = 3;
			this->enter(ssd1306_page_address, nowMs);
		}
		break;

	case ssd1306_page_address:
		this->issued(sendCommands(cmd_buf, cmd_len), ssd1306_wait_page_address, nowMs);
		break;

	case ssd1306_wait_page_address:
		if (this->landed(nowMs)) {
			chunk = 0;
			this->enter(ssd1306_page_data, nowMs);
		}
		break;

	case ssd1306_page_data:
		// The transport copies a write, so the frame buffer is free again
		// as soon as this returns.
		this->issued(sendData(&fb[page * SSD1306_WIDTH + chunk * kChunk], kChunk),
			ssd1306_wait_page_data, nowMs);
		break;

	case ssd1306_wait_page_data:
		if (this->landed(nowMs)) {
			if (++chunk < kChunksPerPage) {
				this->enter(ssd1306_page_data, nowMs);
			} else {
				sent_hash[page] = page_hash[page];
				dirty &= (uint8_t)~(1u << page);
				++page;
				this->enter(ssd1306_next_page, nowMs);
			}
		}
		break;

	case ssd1306_error:
		if (this->errorCleared(nowMs, kErrorBackoffMs)) this->enter(ssd1306_init, nowMs);
		break;

	default:
		this->enter(ssd1306_init, nowMs);
		break;
	}
}

} /* namespace SSD1306 */
