/*
 * AHT20.cpp
 *
 *  Created on: May 31, 2025
 *      Author: coryg
 */

#include "AHT20.h"
#include <stm32l4xx_hal.h>

extern "C" {

extern UART_HandleTypeDef huart2;
uint8_t error[255];

//namespace AHT20 {

static uint8_t crc8(const uint8_t *data, int len);

AHT20::AHT20(I2C_HandleTypeDef *hi2c) {
	// TODO Auto-generated constructor stub
	_DevAddress = AHT20_I2C_Addr<<1;
	_hi2c = hi2c;
	state = aht20_init;
	_start_measure = false;
	_newData=false;
}

AHT20::~AHT20() {
	// TODO Auto-generated destructor stub
	delete this;
}

/*
 * Performs a Soft Reset
 * @param none
 * @return none
 */
void AHT20::softReset(){
	uint8_t buf[1];
	buf[0]=AHT20_SoftResetCmd;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
}

/*
 * Initializes the AHT20
 * @param none
 * @return none
 */
bool AHT20::_Init(){
	uint8_t buf[3];
	HAL_StatusTypeDef I2CStatus; // HAL I2C Status
	buf[0]=AHT20_Init_Cmd;
	buf[1]=AHT20_Init_High;
	buf[2]=AHT20_Init_Low;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,3,HAL_MAX_DELAY);
	return (HAL_OK==I2CStatus);
}

/*
 * Starts an AHT20 Measurement.
 * Wait 80ms after calling StartMeasurement() before calling ReadTempHumidity()
 * @param none
 * @return none
 */
bool AHT20::_startmeasurement(){
	uint8_t buf[3];
	HAL_StatusTypeDef I2CStatus; // HAL I2C Status
	_newData=false;
	buf[0]=AHT20_MeasurementCmd;
	buf[1]=AHT20_MeasurementCmdHigh;
	buf[2]=AHT20_MeasurementCmdLow;
	I2CStatus=HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,3,HAL_MAX_DELAY);
	return (HAL_OK==I2CStatus);
}

void AHT20::StartMeasurement(){
	if(aht_sleep==state){
		_start_measure=true;
	}
}

/*
 * Reads the sensor measurement.
 * Wait 80ms after calling StartMeasurement() before calling ReadTempHumidity().
 * @param *temp	pointer to the temperature
 * @param *humidity pointer to the humidity
 * @return sensor reading completed successfully
 */
bool AHT20::ReadTempHumidity(float *temp, float *humidity){
	uint8_t buf[8];
	uint32_t t,h;
	uint8_t scratch;

	HAL_StatusTypeDef I2CStatus; // HAL I2C Status

	*temp = NAN;
	*humidity = NAN;

	I2CStatus=HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,7,HAL_MAX_DELAY);

	if(I2CStatus != HAL_OK) return false; // return false if sensor did not ACK

	if((buf[0]&0b10000000)>0) return false; // return false if sensor is busy

	if(crc8(buf,6)!=buf[6]) {
			sprintf((char*)error,"Did not pass CRC check");
			HAL_UART_Transmit(&huart2,error,strlen((char*)error),HAL_MAX_DELAY);
			HAL_Delay(100);
			return false; // check crc
	}
	// convert humidity value
	h = buf[1];
	h = h<<8;
	scratch = buf[2];
	h|=scratch;
	h = h<<4;
	scratch = buf[3]&0b11110000;
	scratch=scratch>>4;
	h|=scratch;

	*humidity = (float(h)/float(0x100000))*100;

	// convert temp value
	t=0;
	scratch = buf[3]&0b00001111;
	t|=scratch;
	t<<=8;
	scratch = buf[4];
	t|=scratch;
	t<<=8;
	scratch=buf[5];
	t|=scratch;

	*temp = (float(t)/float(0x100000))*200 - 50;

	_newData=true;

	return true;
}

bool AHT20::readStatusReg(){
	uint8_t buf[1];
	HAL_StatusTypeDef I2CStatus; // HAL I2C Status
	// Read Status Bit
	buf[0]=AHT20_Status_Reg;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	I2CStatus=HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	status_word=buf[0];
	return (HAL_OK==I2CStatus);
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
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

void AHT20::main(){
	switch(state){
	case aht20_init:
		if(checkDevice()){ // check if device exists
			softReset(); // if device exists perform soft reset
			state=aht_reset;
			last_update=HAL_GetTick();
		}
		else { // device doesn't exist. go to error
			state=aht_error;
			last_update=HAL_GetTick();
		}
		break;
	case aht_reset:
		if(HAL_GetTick()>(last_update+100)){ // wait for AHT20 to complete reset.
			// if status byte is not 0x18, reinitialize the AHT20 sensor
			if(readStatusReg()){
				if(status_word!=0x18){
					state=aht_reinit;
				}
				else {
					state=aht_sleep;
				}
				last_update=HAL_GetTick();
			}
			else {
				state=aht_error;
				last_update=HAL_GetTick();
			}
		}
		break;
	case aht_reinit:
		if(_Init()){ // reinit the AHT20
			last_update=HAL_GetTick();
			state=aht_reset;
		}
		else{
			last_update=HAL_GetTick();
			state=aht_error;
		}
		break;
	case aht_sleep:
		if((_start_measure==true)||(HAL_GetTick()>(last_update+1000))){
			if(_startmeasurement()){// try to start measurement
				state=aht_measuring; // wait for conversion to complete
			}
			else{
				state=aht_error; // could not communicate with sensor
			}
			last_update=HAL_GetTick();
		}
		break;
	case aht_measuring:
		if(HAL_GetTick()>(last_update+100)){ // wait for conversion to complete
			if(ReadTempHumidity(&_temp,&_humidity)){ // if conversion completed, read temp and humidity
				state=aht_sleep;
				last_update=HAL_GetTick();
			}else{
				state=aht_error;
				last_update=HAL_GetTick();
			}
		}
		break;
	case aht_error:
		if(HAL_GetTick()>(last_update+100)){
			state=aht20_init;
		}
		break;
	default:
		state=aht20_init;
		break;
	}
}

// check if device exists
bool AHT20::checkDevice(){
	HAL_StatusTypeDef I2CStatus = HAL_I2C_IsDeviceReady(_hi2c,_DevAddress,1,HAL_MAX_DELAY);
	return (HAL_OK==I2CStatus);
}

void AHT20::getTempHumidity(float *temp, float *humidity){
	*temp = _temp;
	*humidity = _humidity;
}

bool AHT20::newData(){
	bool old = _newData;
	_newData=false;
	return old;
}
//} /* namespace AHT20 */

} // extern C
