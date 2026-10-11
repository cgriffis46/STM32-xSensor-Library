/*
 * MTK3333.h
 *
 *  UART driver for MediaTek MT3333 based GPS/GNSS modules
 *  (e.g. Adafruit Ultimate GPS / Mini GPS PA1010D UART, Quectel L80/L86).
 *
 *  Bytes are received one at a time by the HAL UART interrupt into a ring
 *  buffer. A task calls update() to assemble and parse NMEA sentences
 *  (RMC and GGA, any talker ID: GP/GN/GL/...).
 *
 *  Hook-up from the application:
 *
 *    MTK3333 gps(&huart1);
 *
 *    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { gps.rxCpltCallback(huart); }
 *    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)  { gps.errorCallback(huart); }
 *
 *    gps.begin();
 *    for (;;) { gps.update(); if (gps.newData()) gps.getData(&data); osDelay(10); }
 */

#ifndef MTK3333_MTK3333_H_
#define MTK3333_MTK3333_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define MTK3333_DEFAULT_BAUD     9600
#define MTK3333_RX_BUFFER_SIZE   256    ///< ring buffer, must be a power of two
#define MTK3333_MAX_SENTENCE     100    ///< NMEA max is 82 chars, PMTK replies can be longer
#define MTK3333_TX_TIMEOUT_MS    100

#if (MTK3333_RX_BUFFER_SIZE & (MTK3333_RX_BUFFER_SIZE - 1)) != 0
#error "MTK3333_RX_BUFFER_SIZE must be a power of two"
#endif

typedef struct {
	// UTC time / date (from RMC, time also from GGA)
	uint8_t hour, minute, second;
	uint16_t millisecond;
	uint8_t day, month;
	uint16_t year;
	bool timeValid;          ///< a time field has been received
	bool dateValid;          ///< RMC reported a valid fix, so date and time are trustworthy

	// Position
	bool fix;                ///< RMC status 'A'
	uint8_t fixQuality;      ///< GGA: 0 = none, 1 = GPS, 2 = DGPS, ...
	uint8_t satellites;      ///< GGA: satellites used in the solution
	int32_t latitudeE6;      ///< micro-degrees, north positive
	int32_t longitudeE6;     ///< micro-degrees, east positive
	float latitude;          ///< degrees, north positive
	float longitude;         ///< degrees, east positive
	float altitude;          ///< metres above mean sea level
	float hdop;
	float speedKnots;
	float course;            ///< degrees true
} GPS_Data;

class MTK3333 {
public:
	MTK3333(UART_HandleTypeDef *_huart);

	bool begin();                                  // start interrupt reception
	bool update();                                 // parse received bytes; true if a sentence was parsed
	bool newData() const { return _newData; }      // a new RMC (once per fix epoch) has been parsed
	void getData(GPS_Data *_data);                 // copy the latest data and clear newData()

	// Call these from HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback
	void rxCpltCallback(UART_HandleTypeDef *_huart);
	void errorCallback(UART_HandleTypeDef *_huart);

	// PMTK configuration. The '$', checksum and CR/LF are added automatically,
	// e.g. sendCommand("PMTK220,1000")
	bool sendCommand(const char *cmd);
	bool setUpdateRate(uint16_t periodMs);         // PMTK220, 100 - 10000 ms
	bool setNmeaOutput(bool rmc, bool gga, bool gsa = false, bool gsv = false, bool vtg = false);
	bool setBaudRate(uint32_t baud);               // PMTK251, also reconfigures the UART

	// Parse one complete sentence ("$....*hh"). Exposed for unit testing.
	static bool parseSentence(const char *sentence, GPS_Data *data);
	static bool checksumOk(const char *sentence);

	uint32_t checksumErrors() const { return _checksumErrors; }
	uint32_t rxErrors() const { return _rxErrors; }   // ring overflows + UART errors

protected:
	bool handleChar(char c);
	bool startReceive();

	UART_HandleTypeDef *huart;
	uint8_t rxByte;
	volatile uint8_t rxRing[MTK3333_RX_BUFFER_SIZE];
	volatile uint16_t rxHead;   // written by the ISR
	volatile uint16_t rxTail;   // written by update()

	char sentence[MTK3333_MAX_SENTENCE + 1];
	uint8_t sentenceLen;
	bool inSentence;

	GPS_Data data;
	bool _newData;
	uint32_t _checksumErrors;
	volatile uint32_t _rxErrors;
};

#endif /* MTK3333_MTK3333_H_ */
