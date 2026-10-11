/*
 * SSD1306.cpp
 *
 *  I2C driver for SSD1306 monochrome OLED displays.
 */

#include "SSD1306.h"
#include "SSD1306_font.h"
#include <string.h>

// I2C control bytes
#define SSD1306_CONTROL_CMD   0x00
#define SSD1306_CONTROL_DATA  0x40

// Commands
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_CHARGEPUMP          0x8D
#define SSD1306_MEMORYMODE          0x20
#define SSD1306_SEGREMAP            0xA0
#define SSD1306_COMSCANDEC          0xC8
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETCONTRAST         0x81
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETVCOMDETECT       0xDB
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_DEACTIVATE_SCROLL   0x2E
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22

SSD1306::SSD1306(I2C_HandleTypeDef *_hi2c, uint8_t _devAddress, uint8_t _width, uint8_t _height) {
	this->hi2c = _hi2c;
	this->devAddress = _devAddress << 1;
	this->_width = (_width > SSD1306_MAX_WIDTH) ? SSD1306_MAX_WIDTH : _width;
	this->_height = (_height > 32) ? SSD1306_MAX_HEIGHT : 32;
	memset(buffer, 0, sizeof(buffer));
}

bool SSD1306::writeCommands(const uint8_t *cmds, uint16_t len) {
	return HAL_I2C_Mem_Write(this->hi2c, this->devAddress, SSD1306_CONTROL_CMD, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) cmds, len, SSD1306_I2C_TIMEOUT_MS) == HAL_OK;
}

bool SSD1306::writeCommand(uint8_t cmd) {
	return writeCommands(&cmd, 1);
}

bool SSD1306::begin() {
	if (HAL_I2C_IsDeviceReady(this->hi2c, this->devAddress, 3, SSD1306_I2C_TIMEOUT_MS) != HAL_OK)
		return false;

	const bool tall = (_height == 64);
	const uint8_t init[] = {
		SSD1306_DISPLAYOFF,
		SSD1306_SETDISPLAYCLOCKDIV, 0x80,
		SSD1306_SETMULTIPLEX, (uint8_t)(_height - 1),
		SSD1306_SETDISPLAYOFFSET, 0x00,
		SSD1306_SETSTARTLINE | 0x00,
		SSD1306_CHARGEPUMP, 0x14,              // internal charge pump
		SSD1306_MEMORYMODE, 0x00,              // horizontal addressing
		SSD1306_SEGREMAP | 0x01,               // column 127 -> SEG0
		SSD1306_COMSCANDEC,
		SSD1306_SETCOMPINS, (uint8_t)(tall ? 0x12 : 0x02),
		SSD1306_SETCONTRAST, (uint8_t)(tall ? 0xCF : 0x8F),
		SSD1306_SETPRECHARGE, 0xF1,
		SSD1306_SETVCOMDETECT, 0x40,
		SSD1306_DISPLAYALLON_RESUME,
		SSD1306_NORMALDISPLAY,
		SSD1306_DEACTIVATE_SCROLL,
		SSD1306_DISPLAYON
	};
	if (!writeCommands(init, sizeof(init)))
		return false;

	clear();
	return display();
}

bool SSD1306::display() {
	const uint8_t addr[] = {
		SSD1306_COLUMNADDR, 0, (uint8_t)(_width - 1),
		SSD1306_PAGEADDR, 0, (uint8_t)(_height / 8 - 1)
	};
	if (!writeCommands(addr, sizeof(addr)))
		return false;
	return HAL_I2C_Mem_Write(this->hi2c, this->devAddress, SSD1306_CONTROL_DATA, I2C_MEMADD_SIZE_8BIT,
			buffer, (uint16_t)(_width * _height / 8), SSD1306_I2C_TIMEOUT_MS) == HAL_OK;
}

void SSD1306::clear() {
	memset(buffer, 0, sizeof(buffer));
}

bool SSD1306::setContrast(uint8_t contrast) {
	const uint8_t cmd[] = { SSD1306_SETCONTRAST, contrast };
	return writeCommands(cmd, sizeof(cmd));
}

bool SSD1306::setInverted(bool inverted) {
	return writeCommand(inverted ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

bool SSD1306::setDisplayOn(bool on) {
	return writeCommand(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}

void SSD1306::drawPixel(int16_t x, int16_t y, SSD1306_Color color) {
	if (x < 0 || y < 0 || x >= _width || y >= _height)
		return;
	uint8_t *b = &buffer[x + (y / 8) * _width];
	uint8_t mask = 1 << (y & 7);
	switch (color) {
	case SSD1306_WHITE:   *b |= mask;  break;
	case SSD1306_BLACK:   *b &= ~mask; break;
	case SSD1306_INVERSE: *b ^= mask;  break;
	}
}

void SSD1306::drawHLine(int16_t x, int16_t y, int16_t w, SSD1306_Color color) {
	for (int16_t i = 0; i < w; i++)
		drawPixel(x + i, y, color);
}

void SSD1306::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, SSD1306_Color color) {
	for (int16_t j = 0; j < h; j++)
		drawHLine(x, y + j, w, color);
}

void SSD1306::drawChar(int16_t x, int16_t y, char c, uint8_t scale, SSD1306_Color color) {
	if (c < SSD1306_FONT_FIRST_CHAR || c > SSD1306_FONT_LAST_CHAR)
		c = '?';
	if (scale == 0)
		scale = 1;
	const uint8_t *glyph = ssd1306_font5x7[c - SSD1306_FONT_FIRST_CHAR];
	for (uint8_t col = 0; col < SSD1306_FONT_WIDTH; col++) {
		uint8_t bits = glyph[col];
		for (uint8_t row = 0; row < SSD1306_FONT_HEIGHT; row++) {
			if (bits & (1 << row)) {
				if (scale == 1)
					drawPixel(x + col, y + row, color);
				else
					fillRect(x + col * scale, y + row * scale, scale, scale, color);
			}
		}
	}
}

int16_t SSD1306::drawString(int16_t x, int16_t y, const char *str, uint8_t scale, SSD1306_Color color) {
	if (scale == 0)
		scale = 1;
	while (*str) {
		drawChar(x, y, *str++, scale, color);
		x += (SSD1306_FONT_WIDTH + SSD1306_CHAR_SPACING) * scale;
	}
	return x;
}

void SSD1306::drawStringCentered(int16_t y, const char *str, uint8_t scale, SSD1306_Color color) {
	int16_t x = (_width - stringWidth(str, scale)) / 2;
	drawString(x < 0 ? 0 : x, y, str, scale, color);
}

int16_t SSD1306::stringWidth(const char *str, uint8_t scale) {
	if (scale == 0)
		scale = 1;
	int16_t n = (int16_t) strlen(str);
	if (n == 0)
		return 0;
	// the trailing spacing column after the last glyph is not counted
	return (int16_t)(n * (SSD1306_FONT_WIDTH + SSD1306_CHAR_SPACING) - SSD1306_CHAR_SPACING) * scale;
}
