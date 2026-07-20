/*
 * BME280.cpp
 *
 *  Created on: Nov 16, 2025
 *      Author: coryg
 */

#include "BME280.h"

namespace BME280 {

uint8_t charBuffer[255];

bme280::bme280() {
	// TODO Auto-generated constructor stub
	bme280_state=bme280_init;
}

bme280::~bme280() {
	// TODO Auto-generated destructor stub
	delete this;
}

bool bme280::init(){return false;}

bool bme280i2c::init(){
	HAL_StatusTypeDef I2CStatus;
	// check if device exists
	I2CStatus = HAL_I2C_IsDeviceReady(this->_hi2c,this->_DevAddress,1,HAL_MAX_DELAY);
	return (HAL_OK==I2CStatus);
}

void bme280::main(){
	switch (bme280_state){
		case bme280_init:
			reset();
			// get system time. wait 100ms before continuing
			_newData=false;
			last_update=HAL_GetTick();
			bme280_state = bme280_reset;
			break;
		case bme280_reset:
			// wait 100ms after issuing Rest
			if(HAL_GetTick()>(last_update+100)){
				if(readIDReg()){
					bme280_state=bme280_initializing;
				}
			}
			break;
		case bme280_initializing:
			readStatusReg();
			if((status.reg_value&bme_status_im_update)!=bme_status_im_update) { // wait for cal values to load into register
				readCal();
				readHumCal();
				_osrs_p_t = bme280_osrs_p_1x;
				_osrs_t_t = bme280_osrs_t_1x;
				_osrs_h_t = bme280_osrs_h_1x;
				mode_t = bme280_forced_mode;
				setCtrlHumReg(_osrs_h_t);
				setConfigReg(bme280_sb_500,bme280_filter_off,bme280_spi_en_off);
				setCtrlMeasReg(_osrs_t_t,_osrs_p_t,mode_t);
				last_update=HAL_GetTick();
				bme280_state=bme280_sleeping;
			} else{
			}
			break;
		case bme280_start_measurement:
			if(HAL_GetTick()>last_update+10){
				if(startMeasurement()){
					bme280_state=bme280_measuring;
				} else {
					}}
			last_update=HAL_GetTick();
			break;
		case bme280_measuring:
			if(HAL_GetTick()>(last_update+100)){
				if(readTempPressureHum(&_temp,&_pressure,&_humidity)){
					last_update=HAL_GetTick();
					bme280_state=bme280_sleeping;}
			}
			break;
		case bme280_sleeping:
			if(HAL_GetTick()>(last_update+100)){
					bme280_state=bme280_start_measurement;
				}
			break;
		case bme280_error:
			if(HAL_GetTick()>(last_update+100)){
						bme280_state=bme280_init;
					}
			break;
		default: // if we don't know what state we're in, re-init
			bme280_state=bme280_init;
			break;
	}
}

/*
 * virtual functions overridden in bme280i2c and bme280spi child classes
 * */
uint8_t bme280::readReg8(bme280_reg8_t *bme280_reg_addr){
	return 0;
}

uint8_t bme280::writeReg8(bme280_reg8_t *bme280_reg_addr){
	return 0;
}

void bme280::readRegMulti(bme280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
}

void bme280::writeRegMulti(){
}

void bme280::reset(){
	bme280_reg8_t bme280_reset_reg = {bme280_reg_reset_addr,bme280_reset_value};
	writeReg8(&bme280_reset_reg);
}

/* Taken from bme280 Datasheet */
double bme280::bme280_compensate_T_double(bme280_S32_t adc_T){
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) *((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)*(((double)adc_T)/13172.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	t_fine = (bme280_S32_t)(var1+var2);
	T=(var1+var2)/5120.0;
	return T;
}

/* Taken from bme280 Datasheet */
double bme280::bme280_compensate_P_double(bme280_S32_t adc_P){
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

double bme280::bme280_compensate_H_double(bme280_S32_t adc_H){
	double var_H;
	var_H = (((double)t_fine) - 76800.0);
	var_H = (adc_H-(((double)dig_H4)*64.0+((double)dig_H5)/16384.0*var_H))*(((double)dig_H2)/65536.0*(1.0+((double)dig_H6)/67108864.0*var_H*(1.0+((double)dig_H3)/67108864.0*var_H)));
	var_H = var_H*(1.0-((double)dig_H1)*var_H/524288.0);
	if(var_H>100.0) {
		var_H = 100.0;
	} else if (var_H<0.0){
		var_H = 0.0;
	}
	return var_H;
}

bool bme280::readIDReg(){
	bme280_reg8_t bme280_id_reg = {bme280_reg_id_addr,0};
	readReg8(&bme280_id_reg);
	if(bme280_id_reg.reg_value==bme280_id){
		this->id.reg_value=bme280_id_reg.reg_value;
		return true;
	}
	return false;
}

void bme280::readConfigReg(){
	config.addr=bme280_reg_config_addr;
	readReg8(&config);
}

void bme280::readCtrlMeas(){
	ctrl_meas.addr=bme280_reg_ctrl_meas_addr;
	readReg8(&ctrl_meas);
}

void bme280::readStatusReg(){
	status.addr=bme280_reg_status_addr;
	readReg8(&status);
}

void bme280::readCal(){
	uint8_t calReg[24];
	readRegMulti(bme280_reg_cal_addr,calReg,24);
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

void bme280::readHumCal(){
	uint8_t calReg[7];
	uint8_t temp;
	bme280_reg8_t _dig_H1_reg = {bme280_reg_dig_H1_addr,0};
	readReg8(&_dig_H1_reg);
	dig_H1 = _dig_H1_reg.reg_value;

	readRegMulti(bme280_reg_dig_H2_addr,calReg,7);

	dig_H2 = calReg[1];
	dig_H2 = dig_H2<<8;
	dig_H2 |= calReg[0];

	dig_H3 = calReg[2];

	dig_H4 = calReg[3];
	dig_H4 = dig_H4<<4;
	dig_H4 |= (calReg[4]&0b00001111);

	dig_H5 = 0;
	temp = calReg[4]&0b11110000;
	temp = temp>>4;
	dig_H5 = calReg[5];//0xE6
	dig_H5 = dig_H5<<4;
	dig_H5 |= temp;

	dig_H6 = calReg[6];
}

void bme280::setCtrlMeasReg(bme280_osrs_t_t osrs_t_t, bme280_osrs_p_t osrs_p_t, bme280_mode_t mode_t){
	ctrl_meas.addr=bme280_reg_ctrl_meas_addr;
	ctrl_meas.reg_value=(osrs_t_t|osrs_p_t|mode_t);
	writeReg8(&ctrl_meas);
}

void bme280::setConfigReg(bme280_sb_t sb_t, bme280_filter_t filter_t, bme280_spi_en spi_en){
	this->_sb_t=sb_t;this->_filter_t=filter_t;this->_spi_en=spi_en;
	config.addr=bme280_reg_config_addr;
	config.reg_value=(sb_t|filter_t|spi_en);
	writeReg8(&config);
}

void bme280::setCtrlHumReg(bme280_osrs_h_t osrs_h_t){
	this->_osrs_h_t = osrs_h_t;
	ctrl_hum.addr = bme280_reg_ctrl_hum_addr;
	ctrl_hum.reg_value = this->_osrs_h_t;
	writeReg8(&ctrl_hum);
}

bool bme280::startMeasurement(){
	readStatusReg();
	if((status.reg_value&bme_status_measuring)!=bme_status_measuring){ // bme80 is not already measuring
		// start forced measurement
		mode_t=bme280_forced_mode;
		setCtrlMeasReg(_osrs_t_t,_osrs_p_t, mode_t);
		return true;
	}
	else {
		return false;
	}
}

bool bme280::readTempPressureHum(double *t, double *p, double *h){
	uint8_t buf[8];
	readStatusReg();
	if((status.reg_value&bme_status_measuring)!=bme_status_measuring){ // bme80 is already measuring
		readRegMulti(bme280_reg_press_msb_addr,buf,8); // burst read temp/pressure data

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

		this->adc_h = 0;
		this->adc_h = buf[6];
		this->adc_h = this->adc_h<< 8;
		this->adc_h |= buf[7];

		*t = bme280_compensate_T_double(this->adc_t);
		*p = bme280_compensate_P_double(this->adc_p);
		*h = bme280_compensate_H_double(this->adc_h);

		_newData = true;

		return true;
	}
	return false;
}

bool bme280::newData(){
	bool _new = _newData;
	_newData = false;
	return _new;
}


bool bme280::GetTemperature(double *t){
	*t = _temp;
	_newData = false;
	return true;
}
bool bme280::GetPressure(double *p){
	*p = _pressure;
	_newData=false;
	return true;
}

bool bme280::GetHumidity(double *h){
	*h = _humidity;
	_newData=false;
	return true;
}

// I2C BME280

// bme280 constructor
bme280i2c::bme280i2c(I2C_HandleTypeDef *hi2c, bme280_i2c_addr_t dev_addr){
	this->_DevAddress = dev_addr<<1;
	this->_hi2c = hi2c;
	bme280_state=bme280_init;
}

bme280i2c::~bme280i2c() {
	// TODO Auto-generated destructor stub
	delete this;
}

uint8_t bme280i2c::readReg8(bme280_reg8_t *bme280_reg){
	uint8_t buf[1];
	buf[0]=bme280_reg->addr;
	buf[1]=0;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,&buf[1],1,HAL_MAX_DELAY);
	bme280_reg->reg_value=buf[1];
	return 0;
}

uint8_t bme280i2c::writeReg8(bme280_reg8_t *bme280_reg){
	uint8_t buf[2];
	buf[0]=bme280_reg->addr;
	buf[1]=bme280_reg->reg_value;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,2,HAL_MAX_DELAY);
	return 0;
}

void bme280i2c::readRegMulti(bme280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
	buf[0]=addr_t;
	HAL_I2C_Master_Transmit(this->_hi2c,this->_DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(this->_hi2c,this->_DevAddress,&buf[0],numRegs,HAL_MAX_DELAY);
}

void bme280i2c::writeRegMulti(){
}

} /* namespace BME280 */
