/*
 * RtcDate.h
 *
 *  What the real-time clock drivers (ds1307, ds3231) share: a date and
 *  time, checks and conversions for it, and the chips' BCD. Pure
 *  functions, no state; usable on their own.
 *
 *  Dates are 2000..2099, as the chips count them (two BCD year digits).
 *  The day and Unix conversions are Howard Hinnant's days_from_civil and
 *  civil_from_days (general knowledge, checked by ds1307_test against
 *  days counted one by one).
 */

#ifndef RTC_DATE_H_
#define RTC_DATE_H_

#include <stdint.h>

namespace rtc {

// A date and time, 24 hour. year 2000..2099. weekday 0 Sunday .. 6
// Saturday (worked out from the date: a time passed in to be set has its
// weekday ignored).
struct rtc_time_t {
	uint16_t year;
	uint8_t month, day;
	uint8_t hour, minute, second;
	uint8_t weekday;
};

// Days since 1970-01-01 of a date in the proleptic Gregorian calendar.
inline int32_t daysFromCivil(int32_t y, uint32_t m, uint32_t d) {
	y -= m <= 2;
	const int32_t era = (y >= 0 ? y : y - 399) / 400;
	const uint32_t yoe = (uint32_t)(y - era * 400);
	const uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
	const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097 + (int32_t)doe - 719468;
}

inline uint8_t daysInMonth(uint16_t year, uint8_t month) {
	static const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	if (month == 2 && (year % 4 == 0) && (year % 100 != 0 || year % 400 == 0)) return 29;
	return days[month - 1];
}

// 0 Sunday .. 6 Saturday, for 2000-01-01 onwards.
inline uint8_t weekday(uint16_t year, uint8_t month, uint8_t day) {
	// 1970-01-01 was a Thursday (4).
	return (uint8_t)((daysFromCivil(year, month, day) + 4) % 7);
}

// The date and time are real (2000..2099, day within its month, 24 hour).
inline bool valid(const rtc_time_t& t) {
	return t.year >= 2000 && t.year <= 2099
		&& t.month >= 1 && t.month <= 12
		&& t.day >= 1 && t.day <= daysInMonth(t.year, t.month)
		&& t.hour <= 23 && t.minute <= 59 && t.second <= 59;
}

// Seconds since 1970-01-01 00:00:00, the time taken as UTC. t must be valid.
inline uint32_t toUnix(const rtc_time_t& t) {
	const uint32_t days = (uint32_t)daysFromCivil(t.year, t.month, t.day);
	return days * 86400u + t.hour * 3600u + t.minute * 60u + t.second;
}

// The reverse, weekday included. Times before 2000 or after 2099 give a
// time valid() refuses.
inline void fromUnix(uint32_t s, rtc_time_t* t) {
	const uint32_t days = s / 86400u;
	const uint32_t secs = s % 86400u;
	t->hour = (uint8_t)(secs / 3600u);
	t->minute = (uint8_t)(secs / 60u % 60u);
	t->second = (uint8_t)(secs % 60u);
	t->weekday = (uint8_t)((days + 4) % 7);
	// civil_from_days, for days >= 0
	const uint32_t z = days + 719468;
	const uint32_t era = z / 146097;
	const uint32_t doe = z - era * 146097;
	const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	const uint32_t mp = (5 * doy + 2) / 153;
	const uint32_t d = doy - (153 * mp + 2) / 5 + 1;
	const uint32_t m = mp < 10 ? mp + 3 : mp - 9;
	const uint32_t y = yoe + era * 400 + (m <= 2);
	t->year = (uint16_t)(y > 0xFFFF ? 0xFFFF : y);
	t->month = (uint8_t)m;
	t->day = (uint8_t)d;
}

// BCD, refusing a nibble above 9 (a chip that was never set, or a bad read).
inline bool fromBcd(uint8_t v, uint8_t* out) {
	if ((v & 0x0F) > 9 || (v >> 4) > 9) return false;
	*out = (uint8_t)((v >> 4) * 10 + (v & 0x0F));
	return true;
}

inline uint8_t toBcd(uint8_t v) {
	return (uint8_t)(((v / 10) << 4) | (v % 10));
}

// The hours register both chips share: bit 6 set is 12 hour mode, then
// bit 5 is PM and the hour 1..12 (12 AM is 0, 12 PM is 12). False if it
// isn't a real hour.
inline bool hoursFromReg(uint8_t v, uint8_t* hour) {
	uint8_t h;
	if (v & 0x40) {
		if (!fromBcd(v & 0x1F, &h) || h < 1 || h > 12) return false;
		*hour = (uint8_t)(h % 12 + ((v & 0x20) ? 12 : 0));
		return true;
	}
	if (!fromBcd(v & 0x3F, &h) || h > 23) return false;
	*hour = h;
	return true;
}

} /* namespace rtc */

#endif /* RTC_DATE_H_ */
