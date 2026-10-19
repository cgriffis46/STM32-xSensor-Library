/*
 * RainGauge.cpp
 *
 *  Created on: Mar 22, 2026
 *      Author: coryg
 */

#include <RainGauge.h>

namespace wxutil {

RainGauge::RainGauge() {
	// TODO Auto-generated constructor stub
	RainCounter=0;
	LastRainTip=HAL_GetTick();
	RainRate=0;
}

RainGauge::~RainGauge() {
	// TODO Auto-generated destructor stub
	while(not(_rg_fifo.empty())){
		_rg_fifo.pop();
	}
	~_rg_fifo();
	delete this;
}

void RainGauge::IncrementCounter(){
	RainCounter++;
	CurrentRainTipTime=HAL_GetTick();
	if(CurrentRainTipTime>LastRainTipTime){

	} else {

	}
	LastRainTipTime=CurrentRainTipTime;
}

void RainGauge::irq(){
	if(HAL_RTC_GetState(_param->_rtc)==HAL_RTC_STATE_READY){
		rain_tip _new_tip;
		HAL_RTC_GetDate(_param._rtc, &_new_tip._current_tip_date, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(_param._rtc, &_new_tip._current_tip_time, RTC_FORMAT_BIN);
		//_new_tip._last_tip_date=
		IncrementCounter();
		_last_tip._tip_date = _new_tip._tip_date;
		_last_tip._tip_time = _new_tip._tip_time;
		if(_rg_fifo.size()<1200){
			_rg_fifo.push(*_new_tip);
		}
	}
}

uint16_t RainGauge::HourlyRainfall(){
}

uint16_t RainGauge::DailyRainfall(){
}

void RainGauge::_resetCounters(){
}

void RainGauge::loadCounters(){
}

} /* namespace wxutil */
