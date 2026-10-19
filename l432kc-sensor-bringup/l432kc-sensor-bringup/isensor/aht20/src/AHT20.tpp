/*
 * AHT20.tpp
 *
 *  AHT20's member definitions. AHT20.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include AHT20.h, not this file.
 */

#ifndef AHT20_TPP_
#define AHT20_TPP_

#include "../inc/AHT20.h"

template <typename TTransport>
template <typename... TArgs>
AHT20<TTransport>::AHT20(TArgs&&... transportArgs)
	: base(aht20_init, aht_error, aht20_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	status_word=0;
	phase_start=0;
	_start_measure = false;
	_newData=false;
	_temp=NAN;
	_humidity=NAN;
}

template <typename TTransport>
void AHT20<TTransport>::StartMeasurement(){
	if(aht_sleep==_state){
		_start_measure=true;
	}
}

template <typename TTransport>
void AHT20<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case aht20_init:
		_start_measure=false;
		if(this->checkDevice()){ // check if device exists
			enter(aht_soft_reset,nowMs);
		}
		else { // device doesn't exist. go to error
			fail(nowMs);
		}
		break;
	case aht_soft_reset:
		issued(this->writeBytes(&AHT20_SoftResetCmd,1),aht_wait_soft_reset,nowMs);
		break;
	case aht_wait_soft_reset:
		if(landed(nowMs)){
			phase_start=nowMs;
			enter(aht_reset,nowMs);
		}
		break;
	case aht_reset:
		// wait for AHT20 to complete reset.
		if(!elapsed(nowMs,aht20_reset_ms)){
			sleepRemaining(nowMs,aht20_reset_ms);
		} else if(phaseExpired(nowMs)){
			fail(nowMs); // re-initialised over and over and still not calibrated
		} else {
			enter(aht_status_cmd,nowMs);
		}
		break;
	case aht_status_cmd:
		issued(this->writeBytes(&AHT20_Status_Reg,1),aht_wait_status_cmd,nowMs);
		break;
	case aht_wait_status_cmd:
		if(landed(nowMs)){
			enter(aht_read_status,nowMs);
		}
		break;
	case aht_read_status:
		issued(this->readBytes(&status_word,1),aht_wait_status,nowMs);
		break;
	case aht_wait_status:
		if(landed(nowMs)){
			// if the calibration bits are not both set, reinitialize the AHT20 sensor
			if((status_word&AHT20_Status_Calibrated)!=AHT20_Status_Calibrated){
				enter(aht_reinit,nowMs);
			}
			else {
				enter(aht_sleep,nowMs);
			}
		}
		break;
	case aht_reinit:
		{
			// The transport takes its own copy, so a local is fine here.
			const uint8_t buf[3]={AHT20_Init_Cmd,AHT20_Init_High,AHT20_Init_Low};
			issued(this->writeBytes(buf,3),aht_wait_reinit,nowMs);
		}
		break;
	case aht_wait_reinit:
		if(landed(nowMs)){
			enter(aht_reset,nowMs);
		}
		break;
	case aht_sleep:
		if((_start_measure==true)||elapsed(nowMs,aht20_period_ms)){
			_start_measure=false;
			phase_start=nowMs;
			enter(aht_start_measurement,nowMs);
		} else {
			// No longer than aht20_request_poll_ms at a time, so a
			// StartMeasurement() request is seen without waiting out
			// the whole period.
			uint32_t left=aht20_period_ms-(nowMs-last_update);
			sleep(left<aht20_request_poll_ms?left:aht20_request_poll_ms);
		}
		break;
	case aht_start_measurement:
		{
			const uint8_t buf[3]={AHT20_MeasurementCmd,AHT20_MeasurementCmdHigh,AHT20_MeasurementCmdLow};
			issued(this->writeBytes(buf,3),aht_wait_start_measurement,nowMs);
		}
		break;
	case aht_wait_start_measurement:
		if(landed(nowMs)){
			enter(aht_measuring,nowMs);
		}
		break;
	case aht_measuring:
		// wait for conversion to complete
		if(elapsed(nowMs,aht20_conversion_ms)){
			enter(aht_read_measurement,nowMs);
		} else {
			sleepRemaining(nowMs,aht20_conversion_ms);
		}
		break;
	case aht_read_measurement:
		issued(this->readBytes(dataReg,7),aht_wait_measurement,nowMs);
		break;
	case aht_wait_measurement:
		if(landed(nowMs)){
			if((dataReg[0]&AHT20_Status_Busy)>0){ // sensor is busy
				if(phaseExpired(nowMs)){
					fail(nowMs);
				} else {
					enter(aht_busy_pause,nowMs);
				}
			} else if(crc8(dataReg,6)!=dataReg[6]){ // check crc
				fail(nowMs);
			} else {
				convert();
				_newData=true;
				enter(aht_sleep,nowMs);
			}
		}
		break;
	case aht_busy_pause:
		if(elapsed(nowMs,aht20_busy_poll_ms)){
			enter(aht_read_measurement,nowMs);
		} else {
			sleepRemaining(nowMs,aht20_busy_poll_ms);
		}
		break;
	case aht_error:
		if(errorCleared(nowMs,aht20_error_backoff_ms)){
			enter(aht20_init,nowMs);
		}
		break;
	default:
		enter(aht20_init,nowMs);
		break;
	}
}

template <typename TTransport>
bool AHT20<TTransport>::phaseExpired(uint32_t nowMs) const{
	return (nowMs-phase_start)>aht20_phase_timeout_ms;
}

template <typename TTransport>
void AHT20<TTransport>::convert(){
	uint32_t t,h;
	uint8_t scratch;

	// convert humidity value
	h = dataReg[1];
	h = h<<8;
	scratch = dataReg[2];
	h|=scratch;
	h = h<<4;
	scratch = dataReg[3]&0b11110000;
	scratch=scratch>>4;
	h|=scratch;

	_humidity = (float(h)/float(0x100000))*100;

	// convert temp value
	t=0;
	scratch = dataReg[3]&0b00001111;
	t|=scratch;
	t<<=8;
	scratch = dataReg[4];
	t|=scratch;
	t<<=8;
	scratch=dataReg[5];
	t|=scratch;

	_temp = (float(t)/float(0x100000))*200 - 50;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
template <typename TTransport>
uint8_t AHT20<TTransport>::crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

template <typename TTransport>
void AHT20<TTransport>::getTempHumidity(float *temp, float *humidity){
	*temp = _temp;
	*humidity = _humidity;
}

template <typename TTransport>
bool AHT20<TTransport>::newData(){
	bool old = _newData;
	_newData=false;
	return old;
}

#endif /* AHT20_TPP_ */
