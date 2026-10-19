/*
 * RainGauge.h
 *
 *  Created on: Mar 22, 2026
 *      Author: coryg
 */

#ifndef RAINGAUGE_H_
#define RAINGAUGE_H_

#include <stm32l4xx_hal_rtc.h>
#include <cstdint>
#include <queue>
#include <chrono>
#include <stdint.h>
#include <stdlib.h>

namespace wxutil {

struct RG_Param{
	RTC_HandleTypeDef *_rtc;
};

struct rain_tip{
	RTC_TimeTypeDef _current_tip_time;
	RTC_DateTypeDef _current_tip_date;
	RTC_TimeTypeDef _last_tip_time;
	RTC_DateTypeDef _last_tip_date;
}rain_tip;

typedef enum rg_state{
	rg_init_state,
	rg_running_state,
	rg_error_state
};

class RainGauge {
public:
	RainGauge();
	virtual ~RainGauge();
	void irq();
	void resetCounters();
	uint16_t HourlyRainfall();
	uint16_t DailyRainfall();

private:
	rg_state _state;
	volatile long RainCounter;
	void IncrementCounter();
	rain_tip _last_tip;
	std::queue<rain_tip> _rg_fifo;
	void loadCounters();
	void _resetCounters();
};

} /* namespace wxutil */

#endif /* RAINGAUGE_H_ */
