/*
 * Si7021.cpp
 *
 *  Created on: Feb 15, 2025
 *      Author: coryg
 */

#include <Si7021.h>
#include <cstring>
#include <stdio.h>
extern UART_HandleTypeDef huart2;
uint8_t error_buf[255];

Si7021::Si7021(Si7021_param_t param) {
	// TODO Auto-generated constructor stub
	_hi2c = param.hi2c;
	_DevAddress = SI7021_I2C_ADDR<<1;

	_new_data = false;
	_updateTemp=false;
	_updateHumidity=false;
	_enable_heater=param.enable;
	_heater_current=param.heater_current;
	_res=param.res;

}

Si7021::~Si7021() {
	// TODO Auto-generated destructor stub
	delete this;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len) {
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
  uint8_t crc(0x00);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

bool Si7021::readHumidity(float *h){
	uint8_t buf[3];
	uint16_t hm;
	memset(buf,0,3);
	// read humidity measurement
	if(HAL_OK!=HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,3,HAL_MAX_DELAY)){
		*h=NAN;
		return false;
	}

	if(crc8(buf,2)!=buf[2]) {
		*h = NAN;
		sprintf((char*)error_buf,"Failed CRC:");
		HAL_UART_Transmit(&huart2,error_buf,strlen((char*)error_buf),HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2,buf,3,HAL_MAX_DELAY);
		return false; // check crc

	}
	hm = buf[0];
	hm<<=8;
	hm|= buf[1];

	*h = (float(hm)*125)/65536-6;
	_updateHumidity=false;
	return true;
}

bool Si7021::readTemp(float *t){
	uint8_t buf[3];
	uint16_t temp_code;

	if(HAL_OK!=HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,3,HAL_MAX_DELAY)){
		*t=NAN;
		return false;
	}

	if(crc8(buf,2)!=buf[2]) {
		//sprintf((char*)error,"Did not pass CRC check");
		//  HAL_UART_Transmit(&huart2,error,strlen((char*)error),HAL_MAX_DELAY);
		//  HAL_Delay(100);
		*t = NAN;
		return false; // check crc
	}
	temp_code = buf[0];
	temp_code<<=8;
	temp_code|= buf[1];
	*t = (175.72*(float)temp_code)/65536-46.85f;
	_new_data = true;
	_updateTemp=false;
	return true;
}

bool Si7021::_readTempAfterRH(float *t){
	uint8_t buf[3];
	uint16_t temp_code;
	buf[0]=Si7021_Cmd_Read_T_From_RH;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,2,HAL_MAX_DELAY);
	// no CRC8 available
	temp_code = buf[0];
	temp_code<<=8;
	temp_code|= buf[1];
	*t = (175.72*(float)temp_code)/65536-46.85;
	_new_data = true;
	_updateTemp = false;
	return true;
}

void Si7021::main(){
switch (_state){
case Si7021_init_state:
	if(checkDevice()){
		reset();
		_state=Si7021_reset_state;
		last_update=HAL_GetTick();
	} else {
		_state=Si7021_err_state;
	}
	break;
case Si7021_reset_state:
	if(HAL_GetTick()>(last_update+100)){
		_state=Si7021_initializing;
		last_update=HAL_GetTick();
	}
	break;
case Si7021_initializing:
	_write_reg1(_enable_heater,_res);
	_write_heater_reg(_heater_current);
	last_update=HAL_GetTick();
	_state=Si7021_done_state;
	break;
case Si7021_start_temp_measurement:
	_startTempMeasurement();
	last_update=HAL_GetTick();
	_state=Si7021_read_temp_measurement;
	break;
case Si7021_read_temp_measurement:
	 if (HAL_GetTick()>(last_update+Si7021_Measurement_Time)){
		 if(readTemp(&temp)){
			 _state=Si7021_done_state;
			 last_update=HAL_GetTick();
		 } else {
			 _state=Si7021_init_state;
		 }
	 }
	break;
case Si7021_start_humidity_measurement:
	_startHumidityMeasurement();
	last_update=HAL_GetTick();
	_state=Si7021_read_humidity_measurement;
	break;
case Si7021_read_humidity_measurement:
	if(HAL_GetTick()>(last_update+Si7021_Measurement_Time)){
		if(readHumidity(&humidity)){
			last_update=HAL_GetTick();
			_state=Si7021_start_temp_measurement;
		} else {
			_state=Si7021_init_state;
		}
	}
	break;
case Si7021_read_temp_measurement_RH:
	_readTempAfterRH(&temp);
	last_update=HAL_GetTick();
	_state=Si7021_done_state;
	break;
case Si7021_done_state:
	if(_updateTemp){
		_state=Si7021_start_temp_measurement;
	} else if (_updateHumidity){
		_state=Si7021_start_humidity_measurement;
	} else if (HAL_GetTick()>(last_update+2000)){
		_state=Si7021_start_humidity_measurement;
	}
	break;
case Si7021_err_state:
	_state=Si7021_init_state;
	break;
}

}

bool Si7021::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_hi2c,_DevAddress,1,HAL_MAX_DELAY)){
		return true;
	}
	return false;
}

void Si7021::reset(){
	uint8_t buf[1];
	buf[0] = Si7021_Cmd_Reset;
	// send measurement command
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
}

void Si7021::startTempMeasurement(){
	_updateTemp=true;
}

void Si7021::startHumidityMeasurement(){
	_updateHumidity=true;
}

void Si7021::_startTempMeasurement(){
	uint8_t buf[1];
	buf[0] = Si7021_Cmd_Measure_T_No_Hold;
	// send measurement command
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
}

void Si7021::_startHumidityMeasurement(){
	uint8_t buf[1];
	buf[0] = Si7021_Cmd_Measure_RH_No_Hold;
	// send measurement command
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
}

void Si7021::_read_reg1(){
	uint8_t buf[1];
	buf[0]= Si7021_Cmd_Read_RHT_User_Register_1;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	_reg1=buf[0];
}

void Si7021::_write_reg1(Si7021_heater_enable_t enable,Si7021_measure_res res){
	uint8_t buf[2];
	// setup user register 1
	_read_reg1();// datasheet recommends reading register before writing back to it.
	_reg1 = ((_reg1&Si7021_reg1_mask_rsv)|(res&Si7021_reg1_mask)|(enable&Si7021_reg1_mask));
	// write the
	buf[0]= Si7021_Cmd_Write_RHT_User_Register_1;
	buf[1]=_reg1;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,2,HAL_MAX_DELAY);
}

void Si7021::_read_heater_reg(){
	uint8_t buf[2];
	buf[0]= Si7021_Cmd_Read_Heater_Control_Reg;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	_htr=buf[0];
}

void Si7021::_write_heater_reg(Si7021_heater_current_t heater_current){
	uint8_t buf[2];
	_read_heater_reg();
	_htr=((_htr&Si7021_htr_mask_rsv)|(heater_current&Si7021_htr_mask));
	buf[0]= Si7021_Cmd_Write_Heater_Control_Reg;
	buf[1]=_htr;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,2,HAL_MAX_DELAY);
}

void Si7021::readTempHumidity(float *t, float *h){
	*t=temp;
	*h=humidity;
	_new_data=false;
}

bool Si7021::newData(){
	return _new_data;
}
