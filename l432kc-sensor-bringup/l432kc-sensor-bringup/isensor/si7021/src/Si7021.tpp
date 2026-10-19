/*
 * Si7021.tpp
 *
 *  Si7021's member definitions. Si7021.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include Si7021.h, not this file.
 */

#ifndef SI7021_TPP_
#define SI7021_TPP_

#include "../inc/Si7021.h"

template <typename TTransport>
template <typename... TArgs>
Si7021<TTransport>::Si7021(const Si7021_param_t &param, TArgs&&... transportArgs)
	: base(Si7021_init_state, Si7021_err_state, Si7021_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_new_data = false;
	_updateTemp=false;
	_updateHumidity=false;
	_updateHeater=false;
	_enable_heater=param.enable;
	_heater_current=param.heater_current;
	_res=param.res;
	_reg1=0;
	_htr=0;
	temp=NAN;
	humidity=NAN;
}

template <typename TTransport>
void Si7021<TTransport>::main(uint32_t nowMs){
switch (_state){
case Si7021_init_state:
	if(this->checkDevice()){
		enter(Si7021_send_reset_state,nowMs);
	} else {
		fail(nowMs);
	}
	break;
case Si7021_send_reset_state:
	{
		const uint8_t cmd=Si7021_Cmd_Reset;
		issued(this->writeBytes(&cmd,1),Si7021_wait_send_reset_state,nowMs);
	}
	break;
case Si7021_wait_send_reset_state:
	if(landed(nowMs)){
		enter(Si7021_reset_state,nowMs);
	}
	break;
case Si7021_reset_state:
	if(elapsed(nowMs,Si7021_reset_ms)){
		enter(Si7021_initializing,nowMs);
	} else {
		sleepRemaining(nowMs,Si7021_reset_ms);
	}
	break;
case Si7021_initializing:
	// setup user register 1
	// datasheet recommends reading register before writing back to it.
	_updateHeater=false;
	issued(this->readRegs(Si7021_Cmd_Read_RHT_User_Register_1,&_reg1,1),Si7021_wait_read_reg1,nowMs);
	break;
case Si7021_wait_read_reg1:
	if(landed(nowMs)){
		_reg1 = ((_reg1&Si7021_reg1_mask_rsv)|(_res&Si7021_reg1_mask)|(_enable_heater&Si7021_reg1_mask));
		enter(Si7021_write_reg1,nowMs);
	}
	break;
case Si7021_write_reg1:
	issued(this->writeReg(Si7021_Cmd_Write_RHT_User_Register_1,_reg1),Si7021_wait_write_reg1,nowMs);
	break;
case Si7021_wait_write_reg1:
	if(landed(nowMs)){
		enter(Si7021_read_heater_reg,nowMs);
	}
	break;
case Si7021_read_heater_reg:
	issued(this->readRegs(Si7021_Cmd_Read_Heater_Control_Reg,&_htr,1),Si7021_wait_read_heater_reg,nowMs);
	break;
case Si7021_wait_read_heater_reg:
	if(landed(nowMs)){
		_htr=((_htr&Si7021_htr_mask_rsv)|(_heater_current&Si7021_htr_mask));
		enter(Si7021_write_heater_reg,nowMs);
	}
	break;
case Si7021_write_heater_reg:
	issued(this->writeReg(Si7021_Cmd_Write_Heater_Control_Reg,_htr),Si7021_wait_write_heater_reg,nowMs);
	break;
case Si7021_wait_write_heater_reg:
	if(landed(nowMs)){
		enter(Si7021_done_state,nowMs);
	}
	break;
case Si7021_done_state:
	if(_updateHeater){
		// Writes both settings registers again, then comes back here.
		// The wait for the next measurement starts over.
		enter(Si7021_initializing,nowMs);
	} else if(_updateTemp){
		enter(Si7021_start_temp_measurement,nowMs);
	} else if (_updateHumidity){
		enter(Si7021_start_humidity_measurement,nowMs);
	} else if (elapsed(nowMs,Si7021_period_ms)){
		enter(Si7021_start_humidity_measurement,nowMs);
	} else {
		// No longer than Si7021_request_poll_ms at a time, so a
		// request is seen without waiting out the whole period.
		uint32_t left=Si7021_period_ms-(nowMs-last_update);
		sleep(left<Si7021_request_poll_ms?left:Si7021_request_poll_ms);
	}
	break;
case Si7021_start_humidity_measurement:
	{
		const uint8_t cmd=Si7021_Cmd_Measure_RH_No_Hold;
		// send measurement command
		issued(this->writeBytes(&cmd,1),Si7021_wait_start_humidity_measurement,nowMs);
	}
	break;
case Si7021_wait_start_humidity_measurement:
	if(landed(nowMs)){
		enter(Si7021_humidity_measuring,nowMs);
	}
	break;
case Si7021_humidity_measuring:
	if(elapsed(nowMs,Si7021_Measurement_Time)){
		enter(Si7021_read_humidity_measurement,nowMs);
	} else {
		sleepRemaining(nowMs,Si7021_Measurement_Time);
	}
	break;
case Si7021_read_humidity_measurement:
	// read humidity measurement
	issued(this->readBytes(dataReg,3),Si7021_wait_humidity_measurement,nowMs);
	break;
case Si7021_wait_humidity_measurement:
	if(landed(nowMs)){
		if(measurementValid()){
			humidity = (float(measurementCode())*125)/65536-6;
			_updateHumidity=false;
			enter(Si7021_start_temp_measurement,nowMs);
		} else {
			fail(nowMs); // check crc
		}
	}
	break;
case Si7021_start_temp_measurement:
	{
		const uint8_t cmd=Si7021_Cmd_Measure_T_No_Hold;
		// send measurement command
		issued(this->writeBytes(&cmd,1),Si7021_wait_start_temp_measurement,nowMs);
	}
	break;
case Si7021_wait_start_temp_measurement:
	if(landed(nowMs)){
		enter(Si7021_temp_measuring,nowMs);
	}
	break;
case Si7021_temp_measuring:
	if(elapsed(nowMs,Si7021_Measurement_Time)){
		enter(Si7021_read_temp_measurement,nowMs);
	} else {
		sleepRemaining(nowMs,Si7021_Measurement_Time);
	}
	break;
case Si7021_read_temp_measurement:
	issued(this->readBytes(dataReg,3),Si7021_wait_temp_measurement,nowMs);
	break;
case Si7021_wait_temp_measurement:
	if(landed(nowMs)){
		if(measurementValid()){
			temp = (175.72f*(float)measurementCode())/65536-46.85f;
			_new_data = true;
			_updateTemp=false;
			enter(Si7021_done_state,nowMs);
		} else {
			fail(nowMs); // check crc
		}
	}
	break;
case Si7021_err_state:
	if(errorCleared(nowMs,Si7021_error_backoff_ms)){
		enter(Si7021_init_state,nowMs);
	}
	break;
default:
	enter(Si7021_init_state,nowMs);
	break;
}

}

template <typename TTransport>
bool Si7021<TTransport>::measurementValid() const{
	return crc8(dataReg,2)==dataReg[2];
}

template <typename TTransport>
uint16_t Si7021<TTransport>::measurementCode() const{
	uint16_t code;
	code = dataReg[0];
	code<<=8;
	code|= dataReg[1];
	return code;
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
uint8_t Si7021<TTransport>::crc8(const uint8_t *data, int len) {
  /*
   * Initialization data 0x00
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

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
void Si7021<TTransport>::startTempMeasurement(){
	_updateTemp=true;
}

template <typename TTransport>
void Si7021<TTransport>::startHumidityMeasurement(){
	_updateHumidity=true;
}

template <typename TTransport>
void Si7021<TTransport>::setHeater(Si7021_heater_state_t htr_enable,Si7021_heater_current_t htr_current){
	_enable_heater=htr_enable?Si7021_heater_enable:Si7021_heater_disable;
	_heater_current=htr_current;
	_updateHeater=true;
}

template <typename TTransport>
void Si7021<TTransport>::readTempHumidity(float *t, float *h){
	*t=temp;
	*h=humidity;
	_new_data=false;
}

template <typename TTransport>
bool Si7021<TTransport>::newData(){
	return _new_data;
}

#endif /* SI7021_TPP_ */
