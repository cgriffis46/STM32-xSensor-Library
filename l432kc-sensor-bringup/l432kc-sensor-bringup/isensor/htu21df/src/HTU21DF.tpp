/*
 * HTU21DF.tpp
 *
 *  HTU21DF's member definitions. HTU21DF.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include HTU21DF.h, not this file.
 */

#ifndef HTU21DF_TPP_
#define HTU21DF_TPP_

#include "../inc/HTU21DF.h"

template <typename TTransport>
template <typename... TArgs>
HTU21DF<TTransport>::HTU21DF(TArgs&&... transportArgs)
	: base(htu21df_init_state, htu21df_error_state, htu21df_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
  /* Assign default values to internal tracking variables. */
    _newData=false;
    _start_measure=false;
  _humidity = NAN;
  _temp = NAN;
}

template <typename TTransport>
void HTU21DF<TTransport>::startTempMeasurement(){
	_start_measure=true;
}

template <typename TTransport>
void HTU21DF<TTransport>::startHumidityMeasurement(){
	_start_measure=true;
}

template <typename TTransport>
void HTU21DF<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case htu21df_init_state:
		if(this->checkDevice()){ // check if device exists
			enter(htu21df_send_reset_state,nowMs);
		} else{
			fail(nowMs);
		}
		break;
	case htu21df_send_reset_state:
		{
			const uint8_t cmd = HTU21DF_RESET;
			issued(this->writeBytes(&cmd,1),htu21df_wait_send_reset_state,nowMs); // if device exists send reset.
		}
		break;
	case htu21df_wait_send_reset_state:
		if(landed(nowMs)){
			enter(htu21df_reset_state,nowMs);
		}
		break;
	case htu21df_reset_state:
		if(elapsed(nowMs,htu21df_reset_ms)){ // wait 15ms for soft reset.
			enter(htu21df_sleep_state,nowMs);
		} else {
			sleepRemaining(nowMs,htu21df_reset_ms);
		}
		break;
	case htu21df_sleep_state:
		if(_start_measure||elapsed(nowMs,htu21df_period_ms)){
			_start_measure=false;
			enter(htu21df_measure_temp_state,nowMs);
		} else {
			// No longer than htu21df_request_poll_ms at a time, so a
			// request is seen without waiting out the whole period.
			uint32_t left=htu21df_period_ms-(nowMs-last_update);
			sleep(left<htu21df_request_poll_ms?left:htu21df_request_poll_ms);
		}
		break;
	case htu21df_measure_temp_state:
		{
			const uint8_t cmd = HTU21DF_READTEMP;
			issued(this->writeBytes(&cmd,1),htu21df_wait_measure_temp_state,nowMs);
		}
		break;
	case htu21df_wait_measure_temp_state:
		if(landed(nowMs)){
			enter(htu21df_temp_conversion_state,nowMs);
		}
		break;
	case htu21df_temp_conversion_state:
		if(elapsed(nowMs,htu21df_conversion_ms)){
			enter(htu21df_read_temp_state,nowMs);
		} else {
			sleepRemaining(nowMs,htu21df_conversion_ms);
		}
		break;
	case htu21df_read_temp_state:
		issued(this->readBytes(dataReg,3),htu21df_wait_read_temp_state,nowMs);
		break;
	case htu21df_wait_read_temp_state:
		if(landed(nowMs)){
			if(measurementValid()){
				float temp = measurementCode();
				temp *= 175.72f;
				temp /= 65536.0f;
				temp -= 46.85f;
				_temp = temp;
				enter(htu21df_measure_humidity_state,nowMs);
			} else {
				fail(nowMs); // 3rd byte is the CRC
			}
		}
		break;
	case htu21df_measure_humidity_state:
		{
			const uint8_t cmd = HTU21DF_READHUM;
			issued(this->writeBytes(&cmd,1),htu21df_wait_measure_humidity_state,nowMs);
		}
		break;
	case htu21df_wait_measure_humidity_state:
		if(landed(nowMs)){
			enter(htu21df_humidity_conversion_state,nowMs);
		}
		break;
	case htu21df_humidity_conversion_state:
		if(elapsed(nowMs,htu21df_conversion_ms)){
			enter(htu21df_read_humidity_state,nowMs);
		} else {
			sleepRemaining(nowMs,htu21df_conversion_ms);
		}
		break;
	case htu21df_read_humidity_state:
		issued(this->readBytes(dataReg,3),htu21df_wait_read_humidity_state,nowMs);
		break;
	case htu21df_wait_read_humidity_state:
		if(landed(nowMs)){
			if(measurementValid()){
				float hum = measurementCode();
				hum *= 125.0f;
				hum /= 65536.0f;
				hum -= 6.0f;
				_humidity = hum;
				_newData=true;
				enter(htu21df_sleep_state,nowMs);
			} else {
				fail(nowMs); // 3rd byte is the CRC
			}
		}
		break;
	case htu21df_error_state:
		if(errorCleared(nowMs,htu21df_error_backoff_ms)){
			enter(htu21df_init_state,nowMs);
		}
		break;
	default:
		enter(htu21df_init_state,nowMs);
		break;
	}
}

template <typename TTransport>
bool HTU21DF<TTransport>::measurementValid() const{
	return crc8(dataReg,2)==dataReg[2];
}

/* 16 bits of data, dropping the last two status bits. */
template <typename TTransport>
uint16_t HTU21DF<TTransport>::measurementCode() const{
	uint16_t code = dataReg[0];
	code <<= 8;
	code |= dataReg[1] & 0b11111100;
	return code;
}

// CRC-8, polynomial 0x31 (x8 + x5 + x4 + 1), initial value 0x00.
template <typename TTransport>
uint8_t HTU21DF<TTransport>::crc8(const uint8_t *data, int len) {
  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0x00);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

template <typename TTransport>
bool HTU21DF<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
bool HTU21DF<TTransport>::readTemperature(float *temperature){
	bool old = _newData;
	*temperature=_temp;
	_newData=false;
	return old;
}

template <typename TTransport>
bool HTU21DF<TTransport>::readHumidity(float *humidity){
	bool old = _newData;
	*humidity = _humidity;
	_newData=false;
	return old;
}

#endif /* HTU21DF_TPP_ */
