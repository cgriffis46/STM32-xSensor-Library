/*
 * bmp280.cpp
 *
 *  Created on: Aug 9, 2025
 *      Author: coryg
 */

#include "bmp280.h"
#include <stm32l4xx_hal.h>

//extern UART_HandleTypeDef huart2;

namespace BMP280 {

uint8_t charBuffer[255];

bmp280::bmp280() {
	// TODO Auto-generated constructor stub
	bmp280_state=bmp280_init;
	last_update=HAL_GetTick();
}

bmp280::~bmp280() {
	// TODO Auto-generated destructor stub
	delete this;
}

bool bmp280::init(){return false;}

bool bmp280i2c::init(){
	HAL_StatusTypeDef I2CStatus;
	// check if device exists
	I2CStatus = HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY);
	return (I2CStatus==HAL_OK);
}

void bmp280::main(){
	switch (bmp280_state){
		case bmp280_init:
			if(checkDevice()){
				reset();
				_newData=false;
				last_update=HAL_GetTick(); // get system time &  wait 100ms before continuing
				bmp280_state = bmp280_reset;
			} else
			{
				bmp280_state=bmp280_error;
			}
			break;
		case bmp280_reset:
			// wait 200ms after issuing Rest
			if(HAL_GetTick()>(last_update+1)){
				last_update=HAL_GetTick();
				bmp280_state=bmp280_wait_im_update;
			}
			break;
		case bmp280_wait_im_update:
			// im_update clears when BMP280 completes copying calibration trim values from eeprom into the registers.
			if(HAL_GetTick()>(last_update+10)){
				readStatusReg();
				if(status.bit.im_update!=1){
					bmp280_state=bmp280_initializing;
				} else {
					last_update=HAL_GetTick();
				}
			}
			break;
		case bmp280_initializing:
			if(readIDReg()) {
				readCal();
				setConfigReg();
				setCtrlMeasReg();
				last_update=HAL_GetTick();
				bmp280_state=bmp280_sleeping;
			} else{
           	    bmp280_state = bmp280_error;
			}
			break;
		case bmp280_start_measurement:
			if(startMeasurement()){
				last_update=HAL_GetTick();
				bmp280_state=bmp280_measuring;
			} else {
			}
			break;
		case bmp280_measuring:
			if(HAL_GetTick()>(last_update+200)){
				if(readTempPressure(&_temp,&_pressure)){
					last_update=HAL_GetTick();
					bmp280_state=bmp280_sleeping;}
			}
			break;
		case bmp280_sleeping:
			if(HAL_GetTick()>(last_update+1000)){
					bmp280_state=bmp280_start_measurement;
				}
			break;
		case bmp280_error:
			if(HAL_GetTick()>(last_update+100)){
						bmp280_state=bmp280_init;
					}
			break;
		default: // if we don't know what state we're in, re-init
			bmp280_state=bmp280_init;
			break;
	}
}

/*
 * virtual functions overridden in bmp280i2c and bmp280spi child classes
 * */
uint8_t bmp280::readReg8(bmp280_reg_addr_t _addr,uint8_t *buf){
	return 0;
}

uint8_t bmp280::writeReg8(bmp280_reg_addr_t _addr,uint8_t *buf){
	return 0;
}

void bmp280::readRegMulti(bmp280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
}

void bmp280::writeRegMulti(){
}

void bmp280::reset(){
	bmp280_reg8_t bmp280_reset_reg = bmp280_reset_value;
	writeReg8(bmp280_reg_reset_addr,&bmp280_reset_reg);
}

/* Taken from BMP280 Datasheet */
double bmp280::bmp280_compensate_T_double(BMP280_S32_t adc_T){
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) *((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)*(((double)adc_T)/13172.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1+var2);
	T=(var1+var2)/5120.0;
	return T;
}

/* Taken from BMP280 Datasheet */
double bmp280::bmp280_compensate_P_double(BMP280_S32_t adc_P){
	double var1, var2, P;
	var1 = ((double)t_fine/2.0)-64000.0;
	var2 = var1*var1*((double)dig_P6)/32768.0;
	var2 = var2+var1 * ((double)dig_P5)*2.0;
	var2 = (var2/4.0)+(((double)dig_P4)*65536.0);
	var1 = (((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1 = (1.0+var1/32768.0)*((double)dig_P1);
	if(var1 == 0.0){
		return 0;
	}
	P=1048576.0 - (double)adc_P;
	P=(P-(var2/4096.0))*6250.0/var1;
	var1 = ((double)dig_P9)*P*P/2147483648.0;
	return P;
}

bool bmp280::readIDReg(){
	readReg8(bmp280_reg_id_addr,&_id);
	if(_id==BMP280_ID){
		return true;
	}
	return false;
}

void bmp280::readConfigReg(){
	readReg8(bmp280_reg_config_addr,&config.reg);
}

void bmp280::readCtrlMeas(){
	readReg8(bmp280_reg_ctrl_meas_addr,&ctrl_meas.reg);
}

void bmp280::readStatusReg(){
	readReg8(bmp280_reg_status_addr,&status.reg);
}

void bmp280::readCal(){
	uint8_t calReg[24];
	readRegMulti(bmp280_reg_cal_addr,calReg,24);
	dig_T1=calReg[1]<<8;
	dig_T1|=calReg[0];
	dig_T2=calReg[3]<<8;
	dig_T2|=calReg[2];
	dig_T3=calReg[5]<<8;
	dig_T3|=calReg[4];

	dig_P1=calReg[7]<<8;
	dig_P1|=calReg[6];
	dig_P2=calReg[9]<<8;
	dig_P2|=calReg[8];
	dig_P3=calReg[11]<<8;
	dig_P3|=calReg[10];
	dig_P4=calReg[13]<<8;
	dig_P4|=calReg[12];
	dig_P5=calReg[15]<<8;
	dig_P5|=calReg[14];
	dig_P6=calReg[17]<<8;
	dig_P6|=calReg[16];
	dig_P7=calReg[19]<<8;
	dig_P7|=calReg[18];
	dig_P8=calReg[21]<<8;
	dig_P8|=calReg[20];
	dig_P9=calReg[23]<<8;
	dig_P9|=calReg[22];
}

void bmp280::setCtrlMeasReg(){
	//ctrl_meas.addr=bmp280_reg_ctrl_meas_addr;
	//ctrl_meas.reg_value=(osrs_t_t|osrs_p_t|mode_t);

	ctrl_meas.bit.mode=_param._mode_t;
	ctrl_meas.bit.osrs_p=_param._osrs_p_t;
	ctrl_meas.bit.osrs_t=_param._osrs_t_t;

	writeReg8(bmp280_reg_ctrl_meas_addr,&ctrl_meas.reg);
}

void bmp280::setConfigReg(){
	config.bit.t_sb=_param._t_sb;
	config.bit.spi3w_en=_spi_en;
	config.bit.filter=_param._filter_t;
	writeReg8(bmp280_reg_config_addr,&config.reg);
}

bool bmp280::startMeasurement(){
	readStatusReg();
	if(status.bit.measuring==1){ // bmp80 is already measuring
		return false;
	}
	else {// start forced measurement
		_param._mode_t=bmp280_forced_mode;
		setCtrlMeasReg();
		return true;
	}
}

bool bmp280::readTempPressure(double *t, double *p){
	uint8_t buf[6];
	readStatusReg();
	if(status.bit.measuring!=1){ // bmp80 is already measuring
		readRegMulti(bmp280_reg_press_msb_addr,buf,6); // burst read temp/pressure data

		this->adc_p = 0;
		this->adc_p = buf[0];
		this->adc_p=this->adc_p<<8;
		this->adc_p |= buf[1];
		this->adc_p=this->adc_p<<8;
		this->adc_p |= buf[2];
		this->adc_p=this->adc_p>>4;

		this->adc_t = 0;
		this->adc_t = buf[3];
		this->adc_t=this->adc_t<<8;
		this->adc_t |= buf[4];
		this->adc_t=this->adc_t<<8;
		this->adc_t |= buf[5];
		this->adc_t=this->adc_t>>4;

		*t = bmp280_compensate_T_double(this->adc_t);
		*p = bmp280_compensate_P_double(this->adc_p);

		_newData = true;

		return true;
	}
	return false;
}

bool bmp280::newData(){
	return _newData;
}


bool bmp280::GetTemperature(double *t){
	*t = _temp;
	_newData = false;
	return true;
}

bool bmp280::GetPressure(double *p){
	*p = _pressure;
	_newData = false;
	return true;
}

// I2C BMP280

// bmp280 constructor
bmp280i2c::bmp280i2c(bmp280_param_t param){
	memcpy(&this->_param,&param,sizeof(bmp280_param_t));
	_param._DevAddress = param._DevAddress<<1;
	last_update=HAL_GetTick();
	_spi_en=bmp280_spi_en_off;
	bmp280_state=bmp280_init;
}

bmp280i2c::~bmp280i2c() {
	// TODO Auto-generated destructor stub
	delete this;
}


uint8_t bmp280i2c::readReg8(bmp280_reg_addr_t _addr,uint8_t *buf){
	HAL_I2C_Mem_Read(_param._hi2c,_param._DevAddress,_addr,(uint16_t)1, buf,(uint16_t)1, 100);
	return 0;
}

uint8_t bmp280i2c::writeReg8(bmp280_reg_addr_t _addr,uint8_t *buf){
	HAL_I2C_Mem_Write(_param._hi2c,_param._DevAddress, _addr, 1, buf, 1, 100000);
	return 0;
}

void bmp280i2c::readRegMulti(bmp280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
	HAL_I2C_Mem_Read(_param._hi2c,_param._DevAddress, addr_t, 1, buf, (uint16_t)numRegs, 100000);
}

void bmp280i2c::writeRegMulti(){
}

bool bmp280::checkDevice(){
	return false;
}

bool bmp280i2c::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY)){return true;}
	return false;
}

} /* namespace BMP280 */




