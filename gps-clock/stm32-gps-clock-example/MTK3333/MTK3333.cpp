/*
 * MTK3333.cpp
 *
 *  UART / NMEA driver for MediaTek MT3333 based GPS modules.
 */

#include "MTK3333.h"
#include <stdio.h>
#include <string.h>

#define NMEA_MAX_FIELDS 24

/* ---------------------------------------------------------------------------
 * Small parsing helpers (no strtod / sscanf so that newlib-nano stays small)
 * ------------------------------------------------------------------------- */

static int hexValue(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}

static bool isDigit(char c) {
	return c >= '0' && c <= '9';
}

// two decimal digits starting at s
static bool parse2(const char *s, uint8_t *out) {
	if (!isDigit(s[0]) || !isDigit(s[1]))
		return false;
	*out = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
	return true;
}

// Parse "[-]123.456" into an integer scaled by 10^decimals (extra digits truncated).
static bool parseScaled(const char *s, uint8_t decimals, int32_t *out) {
	if (s == NULL || *s == '\0')
		return false;
	bool neg = false;
	if (*s == '-') { neg = true; s++; }
	else if (*s == '+') { s++; }
	if (!isDigit(*s) && *s != '.')
		return false;
	int32_t v = 0;
	while (isDigit(*s))
		v = v * 10 + (*s++ - '0');
	uint8_t d = 0;
	if (*s == '.') {
		s++;
		while (isDigit(*s)) {
			if (d < decimals) {
				v = v * 10 + (*s - '0');
				d++;
			}
			s++;
		}
	}
	for (; d < decimals; d++)
		v *= 10;
	*out = neg ? -v : v;
	return true;
}

static bool parseFloat(const char *s, float *out) {
	int32_t v;
	if (!parseScaled(s, 3, &v))
		return false;
	*out = (float) v / 1000.0f;
	return true;
}

// NMEA "ddmm.mmmmm" / "dddmm.mmmmm" + hemisphere -> micro-degrees
static bool parseCoordinate(const char *s, const char *hemi, int32_t *microDeg) {
	int32_t v;
	if (!parseScaled(s, 5, &v) || v < 0 || hemi == NULL)
		return false;
	int32_t degrees = v / 10000000;      // 100 (minutes) * 1e5
	int32_t minutesE5 = v % 10000000;
	int32_t result = degrees * 1000000 + (minutesE5 * 10) / 60;
	if (*hemi == 'S' || *hemi == 'W')
		result = -result;
	else if (*hemi != 'N' && *hemi != 'E')
		return false;
	*microDeg = result;
	return true;
}

// "hhmmss[.sss]"
static bool parseTime(const char *s, GPS_Data *d) {
	uint8_t h, m, sec;
	if (!parse2(s, &h) || !parse2(s + 2, &m) || !parse2(s + 4, &sec))
		return false;
	if (h > 23 || m > 59 || sec > 60)
		return false;
	uint16_t ms = 0;
	if (s[6] == '.') {
		uint16_t scale = 100;
		for (const char *p = s + 7; isDigit(*p) && scale > 0; p++, scale /= 10)
			ms += (uint16_t)(*p - '0') * scale;
	}
	d->hour = h;
	d->minute = m;
	d->second = sec;
	d->millisecond = ms;
	d->timeValid = true;
	return true;
}

// "ddmmyy"
static bool parseDate(const char *s, uint8_t *day, uint8_t *month, uint16_t *year) {
	uint8_t dd, mm, yy;
	if (!parse2(s, &dd) || !parse2(s + 2, &mm) || !parse2(s + 4, &yy))
		return false;
	if (dd < 1 || dd > 31 || mm < 1 || mm > 12)
		return false;
	*day = dd;
	*month = mm;
	*year = 2000 + yy;
	return true;
}

/* ---------------------------------------------------------------------------
 * MTK3333
 * ------------------------------------------------------------------------- */

MTK3333::MTK3333(UART_HandleTypeDef *_huart) {
	huart = _huart;
	rxByte = 0;
	rxHead = 0;
	rxTail = 0;
	sentenceLen = 0;
	inSentence = false;
	memset(&data, 0, sizeof(data));
	_newData = false;
	_checksumErrors = 0;
	_rxErrors = 0;
}

bool MTK3333::startReceive() {
	return HAL_UART_Receive_IT(huart, &rxByte, 1) == HAL_OK;
}

bool MTK3333::begin() {
	rxHead = rxTail = 0;
	sentenceLen = 0;
	inSentence = false;
	_newData = false;
	return startReceive();
}

void MTK3333::rxCpltCallback(UART_HandleTypeDef *_huart) {
	if (_huart != huart)
		return;
	uint16_t next = (rxHead + 1) & (MTK3333_RX_BUFFER_SIZE - 1);
	if (next != rxTail) {
		rxRing[rxHead] = rxByte;
		rxHead = next;
	} else {
		_rxErrors++;
	}
	startReceive();
}

void MTK3333::errorCallback(UART_HandleTypeDef *_huart) {
	if (_huart != huart)
		return;
	// Overrun / framing / noise errors abort the reception: clear and re-arm.
	__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
	_rxErrors++;
	startReceive();
}

bool MTK3333::update() {
	bool parsed = false;
	while (rxTail != rxHead) {
		char c = (char) rxRing[rxTail];
		rxTail = (rxTail + 1) & (MTK3333_RX_BUFFER_SIZE - 1);
		if (handleChar(c))
			parsed = true;
	}
	return parsed;
}

bool MTK3333::handleChar(char c) {
	if (c == '$') {
		inSentence = true;
		sentenceLen = 0;
		sentence[sentenceLen++] = c;
		return false;
	}
	if (!inSentence)
		return false;

	if (c == '\r' || c == '\n') {
		inSentence = false;
		sentence[sentenceLen] = '\0';
		if (!checksumOk(sentence)) {
			_checksumErrors++;
			return false;
		}
		bool isRmc = (sentenceLen > 6 && strncmp(&sentence[3], "RMC", 3) == 0);
		if (parseSentence(sentence, &data)) {
			if (isRmc)
				_newData = true;
			return true;
		}
		return false;
	}

	if (sentenceLen < MTK3333_MAX_SENTENCE)
		sentence[sentenceLen++] = c;
	else
		inSentence = false;   // too long, drop it
	return false;
}

void MTK3333::getData(GPS_Data *_data) {
	memcpy(_data, &data, sizeof(GPS_Data));
	_newData = false;
}

bool MTK3333::checksumOk(const char *s) {
	if (s == NULL || s[0] != '$')
		return false;
	uint8_t sum = 0;
	const char *p = s + 1;
	while (*p && *p != '*')
		sum ^= (uint8_t) *p++;
	if (*p != '*')
		return false;
	int hi = hexValue(p[1]);
	int lo = hexValue(p[2]);
	if (hi < 0 || lo < 0)
		return false;
	return sum == (uint8_t)((hi << 4) | lo);
}

bool MTK3333::parseSentence(const char *s, GPS_Data *d) {
	if (!checksumOk(s))
		return false;

	// Copy the body between '$' and '*' and split it on commas.
	char buf[MTK3333_MAX_SENTENCE + 1];
	size_t n = 0;
	for (const char *p = s + 1; *p && *p != '*' && n < MTK3333_MAX_SENTENCE; p++)
		buf[n++] = *p;
	buf[n] = '\0';

	char *field[NMEA_MAX_FIELDS];
	uint8_t count = 0;
	field[count++] = buf;
	for (char *p = buf; *p && count < NMEA_MAX_FIELDS; p++) {
		if (*p == ',') {
			*p = '\0';
			field[count++] = p + 1;
		}
	}

	// field[0] is the address, e.g. "GPRMC" / "GNGGA"
	if (strlen(field[0]) != 5)
		return false;
	const char *type = field[0] + 2;

	if (strcmp(type, "RMC") == 0) {
		// time,status,lat,N,lon,E,speed,course,date,magvar,magdir[,mode]
		if (count < 10)
			return false;
		parseTime(field[1], d);
		d->fix = (field[2][0] == 'A');
		int32_t lat, lon;
		if (parseCoordinate(field[3], field[4], &lat) && parseCoordinate(field[5], field[6], &lon)) {
			d->latitudeE6 = lat;
			d->longitudeE6 = lon;
			d->latitude = (float) lat / 1000000.0f;
			d->longitude = (float) lon / 1000000.0f;
		}
		parseFloat(field[7], &d->speedKnots);
		parseFloat(field[8], &d->course);
		uint8_t day, month;
		uint16_t year;
		if (parseDate(field[9], &day, &month, &year)) {
			d->day = day;
			d->month = month;
			d->year = year;
			d->dateValid = d->fix && d->timeValid;
		} else {
			d->dateValid = false;
		}
		return true;
	}

	if (strcmp(type, "GGA") == 0) {
		// time,lat,N,lon,E,quality,numSV,hdop,alt,M,sep,M,diffAge,diffStation
		if (count < 10)
			return false;
		parseTime(field[1], d);
		int32_t v;
		d->fixQuality = parseScaled(field[6], 0, &v) ? (uint8_t) v : 0;
		d->satellites = parseScaled(field[7], 0, &v) ? (uint8_t) v : 0;
		parseFloat(field[8], &d->hdop);
		parseFloat(field[9], &d->altitude);
		int32_t lat, lon;
		if (d->fixQuality > 0 && parseCoordinate(field[2], field[3], &lat)
				&& parseCoordinate(field[4], field[5], &lon)) {
			d->latitudeE6 = lat;
			d->longitudeE6 = lon;
			d->latitude = (float) lat / 1000000.0f;
			d->longitude = (float) lon / 1000000.0f;
		}
		return true;
	}

	return false;   // valid but unhandled sentence (GSA, GSV, PMTK001, ...)
}

bool MTK3333::sendCommand(const char *cmd) {
	char out[MTK3333_MAX_SENTENCE + 8];
	uint8_t sum = 0;
	for (const char *p = cmd; *p; p++)
		sum ^= (uint8_t) *p;
	int len = snprintf(out, sizeof(out), "$%s*%02X\r\n", cmd, sum);
	if (len <= 0 || len >= (int) sizeof(out))
		return false;
	return HAL_UART_Transmit(huart, (uint8_t*) out, (uint16_t) len, MTK3333_TX_TIMEOUT_MS) == HAL_OK;
}

bool MTK3333::setUpdateRate(uint16_t periodMs) {
	if (periodMs < 100 || periodMs > 10000)
		return false;
	char cmd[16];
	snprintf(cmd, sizeof(cmd), "PMTK220,%u", (unsigned) periodMs);
	return sendCommand(cmd);
}

bool MTK3333::setNmeaOutput(bool rmc, bool gga, bool gsa, bool gsv, bool vtg) {
	// GLL,RMC,VTG,GGA,GSA,GSV, then 13 reserved/unused fields
	char cmd[64];
	snprintf(cmd, sizeof(cmd), "PMTK314,0,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,0,0",
			rmc ? 1 : 0, vtg ? 1 : 0, gga ? 1 : 0, gsa ? 1 : 0, gsv ? 1 : 0);
	return sendCommand(cmd);
}

bool MTK3333::setBaudRate(uint32_t baud) {
	char cmd[24];
	snprintf(cmd, sizeof(cmd), "PMTK251,%lu", (unsigned long) baud);
	if (!sendCommand(cmd))
		return false;
	HAL_Delay(20);   // let the module switch over

	HAL_UART_AbortReceive_IT(huart);
	huart->Init.BaudRate = baud;
	if (HAL_UART_Init(huart) != HAL_OK)
		return false;
	return begin();
}
