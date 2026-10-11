/*
 * SSD1306.h
 *
 *  I2C driver for SSD1306 monochrome OLED displays (128x64 and 128x32).
 *
 *  The driver keeps a frame buffer in RAM. Drawing functions only change the
 *  frame buffer; call display() to push it to the panel.
 *
 *  The driver does no locking of its own. If the I2C bus is shared with other
 *  devices across FreeRTOS tasks, take the bus mutex around display() and
 *  any other call that talks to the panel.
 */

#ifndef SSD1306_SSD1306_H_
#define SSD1306_SSD1306_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define SSD1306_I2C_ADDR        0x3C   ///< 7-bit address (0x3D when SA0 is high)
#define SSD1306_MAX_WIDTH       128
#define SSD1306_MAX_HEIGHT      64
#define SSD1306_I2C_TIMEOUT_MS  100

#define SSD1306_FONT_WIDTH      5      ///< glyph width in pixels
#define SSD1306_FONT_HEIGHT     7      ///< glyph height in pixels
#define SSD1306_CHAR_SPACING    1      ///< blank columns after each glyph

enum SSD1306_Color {
	SSD1306_BLACK = 0,
	SSD1306_WHITE = 1,
	SSD1306_INVERSE = 2
};

class SSD1306 {
public:
	SSD1306(I2C_HandleTypeDef *_hi2c, uint8_t _devAddress = SSD1306_I2C_ADDR,
			uint8_t _width = 128, uint8_t _height = 64);

	bool begin();                       // probe and initialise the panel
	bool display();                     // push the frame buffer to the panel
	void clear();                       // clear the frame buffer

	bool setContrast(uint8_t contrast);
	bool setInverted(bool inverted);
	bool setDisplayOn(bool on);

	void drawPixel(int16_t x, int16_t y, SSD1306_Color color = SSD1306_WHITE);
	void drawHLine(int16_t x, int16_t y, int16_t w, SSD1306_Color color = SSD1306_WHITE);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, SSD1306_Color color = SSD1306_WHITE);

	// Text uses the built-in 5x7 font. scale multiplies each pixel (1 = 6x8 cell).
	void drawChar(int16_t x, int16_t y, char c, uint8_t scale = 1, SSD1306_Color color = SSD1306_WHITE);
	int16_t drawString(int16_t x, int16_t y, const char *str, uint8_t scale = 1, SSD1306_Color color = SSD1306_WHITE);
	void drawStringCentered(int16_t y, const char *str, uint8_t scale = 1, SSD1306_Color color = SSD1306_WHITE);
	static int16_t stringWidth(const char *str, uint8_t scale = 1);

	uint8_t width() const { return _width; }
	uint8_t height() const { return _height; }
	uint8_t *getBuffer() { return buffer; }

protected:
	bool writeCommands(const uint8_t *cmds, uint16_t len);
	bool writeCommand(uint8_t cmd);

	I2C_HandleTypeDef *hi2c;
	uint8_t devAddress;   // 8-bit (shifted) address used by the HAL
	uint8_t _width;
	uint8_t _height;
	uint8_t buffer[SSD1306_MAX_WIDTH * SSD1306_MAX_HEIGHT / 8];
};

#endif /* SSD1306_SSD1306_H_ */
