/**
  ******************************************************************************
  * @file     SHT31.cpp
  * @author   cgriffis46
  * @version  V1.0
  * @date     18/03/2024 13:49:10
  * @brief    Default under dev library file.
  ******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "SHT31.h"

//#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c3;

/** Functions ----------------------------------------------------------------*/

extern "C"{

/*!
 * @brief  SHT31 constructor using i2c
 * @param  _hi2c i2c bus handle
 */
SHT31::SHT31(SHT31_param_t sht31_param) {
  state=sht31_init_state;
  this->_hi2c = sht31_param.hi2c;
  this->_devAddress=sht31_param.i2c_addr<<1;
  this->_mps=sht31_param.sample_rate;
  _PeriodicMode=sht31_param.periodic_mode;
  humidity = NAN;
  temp = NAN;
  TempHighAlert = false; TempLowAlert = false; HumidityHighAlert = false; HumidityLowAlert = false;
  _newdata=false;
  _force_measurement=false;
  _update_periodic_mode=false;
}

/**
 * Destructor to free memory in use.
 */
SHT31::~SHT31() {
	delete this;
}

/**
 * Gets the current status register contents.
 *
 * @return The 16-bit status register.
 */
uint16_t SHT31::readStatus(void) {
  writeCommand(SHT31_READSTATUS);

  uint8_t data[3];
  HAL_I2C_Master_Receive(this->_hi2c,this->_devAddress,data,3,HAL_MAX_DELAY);

  uint16_t stat = data[0];
  stat <<= 8;
  stat |= data[1];
  // Serial.println(stat, HEX);
  return stat;
}

/**
 * Performs a reset of the sensor to put it into a known state.
 */
HAL_StatusTypeDef SHT31::reset(void) {
	return writeCommand(SHT31_SOFTRESET);
}

/**
 * Enables or disabled the heating element.
 *
 * @param h True to enable the heater, False to disable it.
 */
void SHT31::heater(bool h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
  //HAL_Delay(1);
}

/*!
 *  @brief  Return sensor heater state
 *  @return heater state (TRUE = enabled, FALSE = disabled)
 */
bool SHT31::isHeaterEnabled() {
  uint16_t regValue = readStatus();
  return false;
//  return (bool)bitRead(regValue, SHT31_REG_HEATER_BIT);
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

/**
 * Internal function to perform and I2C write.
 *
 * @param cmd   The 16-bit command ID to send.
 */
HAL_StatusTypeDef SHT31::writeCommand(uint16_t command) {

	uint8_t cmd[2];

	//SendBreak();

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,2,HAL_MAX_DELAY);
}

bool SHT31::FetchData(float *t, float *h){
  uint8_t cmd[2];
  uint8_t readbuffer[6];

  cmd[0] = SHT31_FETCH_DATA_MSB;
  cmd[1] = SHT31_FETCH_DATA_LSB;

  	if(!(HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,2,HAL_MAX_DELAY)==HAL_OK)) return false;
    if(!(HAL_I2C_Master_Receive(this->_hi2c,this->_devAddress,readbuffer,6,HAL_MAX_DELAY)==HAL_OK)) return false;

    if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) + readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  this->temp = (float)stemp / 100.0f;
  *t = temp;
  uint32_t shum = ((uint32_t)readbuffer[3] << 8) + readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  this->humidity = (float)shum / 100.0f;
  *h = this->humidity;

  if(this->humidity>=this->HighAlert.SetHumidity){this->HumidityHighAlert = true;}
  if(this->humidity<=this->HighAlert.ClearHumidity){this->HumidityHighAlert = false;}
  if(this->temp>=this->HighAlert.SetTemp){this->TempHighAlert = true;}
  if(this->temp<=this->HighAlert.ClearTemp){this->TempHighAlert = false;}

  if(this->humidity<=this->LowAlert.SetHumidity){this->HumidityLowAlert = true;}
  if(this->humidity>=this->LowAlert.ClearHumidity){this->HumidityLowAlert = false;}
  if(this->temp<=this->LowAlert.SetTemp){this->TempLowAlert = true;}
  if(this->temp>=this->LowAlert.ClearTemp){this->TempLowAlert = false;}

  _newdata=true;
  return true;
}

void SHT31::setHighAlert(const SHT31_Alert_t* alert){
  uint16_t TempSet,TempClear, HumiditySet, HumidityClear;
  uint16_t rhtSet,rhtClear;
  uint8_t rhtSetMSB,rhtSetLSB,rhtClearMSB,rhtClearLSB;

  uint8_t cmd[5];

  memcpy(&this->HighAlert,alert,sizeof(SHT31_Alert_t));

  switch (this->unit) {
    case F:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+49)/315)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+49)/315)*65535.0f);
      break;
    }
    default:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+45)/175)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+45)/175)*65535.0f);
      break;
    }
  }

  HumiditySet = static_cast<uint16_t>((alert->SetHumidity*65535.0f)/100);
  HumidityClear = static_cast<uint16_t>((alert->ClearHumidity*65535.0f)/100);

  rhtSet = (HumiditySet&0xFE00);
  rhtSet |= (TempSet>>7);
  rhtSetMSB = (uint8_t)(rhtSet>>8);
  rhtSetLSB = (uint8_t)(rhtSet&0xFF);

  rhtClear =(HumidityClear&0xFE00);
  rhtClear |= (TempClear>>7);
  rhtClearMSB = (uint8_t)(rhtClear>>8);
  rhtClearLSB = (uint8_t)(rhtClear&0xFF);

  cmd[0] = 0x61; // MSB
  cmd[1] = 0x1D;
  cmd[2] = rhtSetMSB;
  cmd[3] = rhtSetLSB;
  cmd[4] = crc8(&cmd[2],2);
  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,5,HAL_MAX_DELAY);

  //printf("Set Byte %x %x \n",cmd[2],cmd[3]);
  cmd[0] = 0x61; // MSB
  cmd[1] = 16;
  cmd[2] = rhtClearMSB;
  cmd[3] = rhtClearLSB;
  cmd[4] = crc8(&cmd[2],2);
  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,5,HAL_MAX_DELAY);

  //printf("clear  Byte %x %x \n",cmd[2],cmd[3]);
}

void SHT31::setLowAlert(const SHT31_Alert_t* alert){
  uint16_t TempSet,TempClear, HumiditySet, HumidityClear;
  uint16_t rhtSet,rhtClear;
  uint8_t rhtSetMSB,rhtSetLSB,rhtClearMSB,rhtClearLSB;

  uint8_t cmd[5];

    memcpy(&this->LowAlert,alert,sizeof(SHT31_Alert_t));

  switch (this->unit) {
    case F:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+49)/315)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+49)/315)*65535.0f);
      break;
    }
    default:
    {
      TempSet = static_cast<uint16_t>(((alert->SetTemp+45)/175)*65535.0f);
      TempClear = static_cast<uint16_t>(((alert->ClearTemp+45)/175)*65535.0f);
      break;
    }
  }

  HumiditySet = static_cast<uint16_t>((alert->SetHumidity*65535.0f)/100);
  HumidityClear = static_cast<uint16_t>((alert->ClearHumidity*65535.0f)/100);

  rhtSet = (HumiditySet&0xFE00);
  rhtSet |= (TempSet>>7);
  rhtSetMSB = (uint8_t)(rhtSet>>8);
  rhtSetLSB = (uint8_t)(rhtSet&0xFF);

  rhtClear =(HumidityClear&0xFE00);
  rhtClear |= (TempClear>>7);
  rhtClearMSB = (uint8_t)(rhtClear>>8);
  rhtClearLSB = (uint8_t)(rhtClear&0xFF);

  cmd[0] = 0x61; // MSB
  cmd[1] = 0x0B;
  cmd[2] = rhtSetMSB;
  cmd[3] = rhtSetLSB;
  cmd[4] = crc8(&cmd[2],2);

  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,5,HAL_MAX_DELAY);
  //printf("Set Byte %x %x \n",cmd[2],cmd[3]);
  cmd[0] = 0x61; // MSB
  cmd[1] = 00;
  cmd[2] = rhtClearMSB;
  cmd[3] = rhtClearLSB;
  cmd[4] = crc8(&cmd[2],2);

  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,5,HAL_MAX_DELAY);
  //printf("clear  Byte %x %x \n",cmd[2],cmd[3]);

}

void SHT31::ReadLowAlert(SHT31_Alert_t* alert){
  memcpy(&this->LowAlert,alert,sizeof(SHT31_Alert_t));
}

void SHT31::ReadHighAlert(SHT31_Alert_t* alert){
  memcpy(&this->HighAlert,alert,sizeof(SHT31_Alert_t));
}

void SHT31::_setperiodicmode(SHT31_Sample_Rate_t mps){
  uint8_t cmd[2];
//  clearStatus();
  switch (mps) {
    case _05mps_high_Res: {
      cmd[0] = SHT31_PERIODIC_05mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_05mps_HIGHREP_LSB;
    }
    case _05_med_Res :{
      cmd[0] = SHT31_PERIODIC_05mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_05mps_MEDREP_LSB;
    }
    case _05_low_Res:{
      cmd[0] = SHT31_PERIODIC_05mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_05mps_LOWREP_LSB;
    }
    case _1mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_1mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_HIGHREP_LSB;
    }
    case _1mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_1mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_MEDREP_LSB;
    }
    case _1mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_1mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_LOWREP_LSB;
    }
    case _2mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_2mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_2mps_HIGHREP_LSB;
    }
    case _2mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_2mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_2mps_MEDREP_LSB;
    }
    case _2mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_2mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_2mps_LOWREP_LSB;
    }
    case _4mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_4mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_4mps_HIGHREP_LSB;
    }
    case _4mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_4mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_4mps_MEDREP_LSB;
    }
    case _4mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_4mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_4mps_LOWREP_LSB;
    }
    case _10mps_high_Res:{
      cmd[0] = SHT31_PERIODIC_10mps_HIGHREP_MSB;
      cmd[1] = SHT31_PERIODIC_10mps_HIGHREP_LSB;
    }
    case _10mps_med_Res:{
      cmd[0] = SHT31_PERIODIC_10mps_MEDREP_MSB;
      cmd[1] = SHT31_PERIODIC_10mps_MEDREP_LSB;
    }
    case _10mps_low_Res:{
      cmd[0] = SHT31_PERIODIC_10mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_10mps_LOWREP_LSB;
    }
    default: {
      cmd[0] = SHT31_PERIODIC_1mps_LOWREP_MSB;
      cmd[1] = SHT31_PERIODIC_1mps_LOWREP_LSB;
    }
  }

  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,2,HAL_MAX_DELAY);

  _update_periodic_mode=false;
}

  bool SHT31::HighTempActive(){
    return TempHighAlert;
  }

  bool SHT31::LowTempActive(){
    return TempLowAlert;
  }

  bool SHT31::HighHumidityActive(){
    return HumidityHighAlert;
  }

  bool SHT31::LowHumidityActive(){
    return HumidityLowAlert;
  }

void SHT31::clearStatus(){
  uint8_t cmd[2];
  cmd[0] = SHT31_CLEARSTATUS_MSB;
  cmd[1] = SHT31_CLEARSTATUS_LSB;
  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,2,HAL_MAX_DELAY);
}

/*
 *
 *
 */
void SHT31::main(){
	switch(state){
	case sht31_init_state:
		if(checkDevice()){ // if device exists, go to reset.
			reset();
			last_update=HAL_GetTick();
			state=sht31_reset_state;
		}
		else { // if device doesn't exist go to error state.
			state=sht31_error_state;
		}
		break;
	case sht31_reset_state:
		if(HAL_GetTick()>(last_update+100)){// wait 100ms for device to reset. then begin init.
			clearStatus(); // clear status register
			last_update=HAL_GetTick();
			state=sht31_clearstatus_state;
		}
		break;
	case sht31_clearstatus_state:
		if(HAL_GetTick()>(last_update+100)){ // wait for status register to clear
			if(readStatus() != 0xFFFF){
				last_update=HAL_GetTick();
				if(this->_PeriodicMode){ // if the sensor should be in periodic mode, set it up.
					last_update=HAL_GetTick();
					state=sht31_set_periodic_mode_state;
				} else { // if
					last_update=HAL_GetTick();
					state=sht31_force_measurement_state; // perform a forced measurement so the data is in the register.
				}
			}
		}
		break;
	case sht31_set_periodic_mode_state:
		_setperiodicmode(this->_mps);
		last_update=HAL_GetTick();
		state=sht31_sleeping_state;
		break;
	case sht31_sleeping_state: // should be in periodic mode.
		if(_force_measurement){ // break from
			SendBreak();
			last_update=HAL_GetTick();
			state=sht31_force_measurement_state;
		}
		else if(HAL_GetTick()>(last_update+2000)){
			if(FetchData(&temp,&humidity)){
				last_update=HAL_GetTick();
				}
			}
		break;
	case sht31_done_state:
		if(_force_measurement){
			if(_PeriodicMode){
				SendBreak();
				last_update=HAL_GetTick();
				state=sht31_force_measurement_state;
			}
			else {
				_SHT31_force_measurement();
				last_update=HAL_GetTick();
				state=sht31_measuring_state;
			}
		}
		break;
	case sht31_measuring_state:
		if(HAL_GetTick()>(last_update+2000)){
			FetchData(&temp, &humidity);
			last_update=HAL_GetTick();
			state=sht31_done_state;
			}
		break;
	case sht31_force_measurement_state: // wait 2ms for a break before calling _SHT31_force_measurement();
		if(HAL_GetTick()>(last_update+2)){
			_SHT31_force_measurement();
			last_update=HAL_GetTick();
			state=sht31_measuring_state;
		}
		break;
	case sht31_error_state:
		if(HAL_GetTick()>(last_update+100)){
			last_update=HAL_GetTick();
			state=sht31_init_state;
		}
		break;
	default:
		 state=sht31_init_state;
		break;
	}

}

bool SHT31::checkDevice(){
	HAL_StatusTypeDef I2CStatus = HAL_I2C_IsDeviceReady(_hi2c,_devAddress,1,HAL_MAX_DELAY);
	return (HAL_OK==I2CStatus);
}

/*
 * The datasheet recommends sending a Break command to the SHT31
 * before issuing commands.
 * Especially if the sensor is in Periodic mode or in a Measurement.
 * do not call Break() before calling FetchData().
 */
void SHT31::SendBreak(){
	uint8_t cmd[2];
	  cmd[0]=SHT31_BREAK_MSB;
	  cmd[1]=SHT31_BREAK_LSB;
	  HAL_I2C_Master_Transmit(this->_hi2c,this->_devAddress,cmd,2,HAL_MAX_DELAY);
	  _PeriodicMode=false;
}

/*
 * If the SHT31 is in Periodic mode, we need to issue a Break() first and wait 1ms
 * when the SHT31 completes measurement in Periodic mode, it will go into sleep state
 * when the SHT31 completes measurement in Foced mode, it will go into done state.
 *
 */
bool SHT31::ForceMeasurement(){

	switch(state){
	case sht31_sleeping_state:
		_force_measurement=true;
		break;
	case sht31_done_state:
		_force_measurement=true;
		break;
	case sht31_measuring_state:
		_force_measurement=true;
		break;
	default:
		return false;
		break;
	}

	return false;
}

void SHT31::_SHT31_force_measurement(){
	writeCommand(SHT31_MEAS_HIGHREP);
	_force_measurement=false;
}

void SHT31::SetPeriodicMode(SHT31_Sample_Rate_t mps){
	if(!_update_periodic_mode){
		switch(state){
			case sht31_init_state:
				this->_mps=mps;
				_update_periodic_mode=true;
			break;
			case sht31_sleeping_state:
				this->_mps=mps;
				_update_periodic_mode=true;
				break;
			case sht31_done_state:
				this->_mps=mps;
				_update_periodic_mode=true;
				break;
			default:
				break;
			}
	}
}

bool SHT31::GetTemperature(float *t){
	bool old = _newdata;
	_newdata=false;
	*t=this->temp;
	return old;
}

bool SHT31::GetHumidity(float *h){
	bool old = _newdata;
	_newdata=false;
	*h=this->humidity;
	return old;
}

bool SHT31::newData(){
	return _newdata;
}

}
