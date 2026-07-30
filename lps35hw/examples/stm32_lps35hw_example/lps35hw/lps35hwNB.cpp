/*
 * lps35hwNB.cpp
 *
 *  Created on: Dec 31, 2025
 *      Author: coryg
 */

#include "lps35hwNB.h"

#include <cstring>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

namespace LPS35HWNB {

uint8_t charBuffer[256];

lps35hwNB::lps35hwNB() {
}

lps35hwNB::~lps35hwNB() {
	// TODO Auto-generated destructor stub
	while(not(_lps35hw_fifo.empty())){
		_lps35hw_fifo.pop();
	}
	delete this;
}

lps35hwNB_i2c::lps35hwNB_i2c(lps35hw_param_t param){
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(lps35hw_param_t));
	_state=lps35hw_init_state;
	_force_measurement=false;
	_newData=false;
	_param._DevAddress=param._DevAddress<<1;
	_pressure=NAN;
	_temp=NAN;
}

lps35hwNB_spi::lps35hwNB_spi(lps35hw_param_t param){
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(lps35hw_param_t));
	_state=lps35hw_init_state;
	_force_measurement=false;
	_newData=false;
	_pressure=NAN;
	_temp=NAN;
}

lps35hwNB_i2c::~lps35hwNB_i2c(){
	delete this;
}

lps35hwNB_spi::~lps35hwNB_spi(){
	delete this;
}

void lps35hwNB::main(){
	osStatus status;
	switch(_state){
	case lps35hw_init_state:
		if(checkDevice()){
			if(whoAmI()){
				reset();
				_state=lps35hw_reset_state;
				last_update=HAL_GetTick();}
			else {
				_state=lps35hw_err_state;
			}
		} else {
			_state=lps35hw_err_state;
		}
		break;
	case lps35hw_reset_state:
		if(HAL_GetTick()>(last_update+100)){
			if(!boot()){
				last_update=HAL_GetTick();
				_state=lps35hw_initializing_state;}
		}
		break;
	case lps35hw_initializing_state:

		write_ctrl_reg1();
		write_ctrl_reg2();
		write_ctrl_reg3();
		write_fifo_ctrl();

		if(fifoEnabled()){
			sprintf((char*)charBuffer,"Fifo Enabled");
			HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
			last_update=HAL_GetTick();
			_state=lps35hw_sleep_state;
		} else {
			sprintf((char*)charBuffer,"Fifo NOT Enabled");
			HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
			last_update=HAL_GetTick();
			_state=lps35hw_done_state;
		}

		break;
	case lps35hw_sleep_state:
		break;
	case lps35hw_done_state:
		if(HAL_GetTick()>(last_update+1000)){
			status = osMutexWait(*_param.i2cMutex, 0);
			if(status==osOK){
				_startMeasurement();
				last_update=HAL_GetTick();
				_state=lps35hw_wait_forced_measurement;
			}
		} else if(_force_measurement&&(HAL_GetTick()>(last_update+100))){

			status = osMutexWait(*_param.i2cMutex, 0);
			if(status==osOK){
					_startMeasurement();
					last_update=HAL_GetTick();
					_state=lps35hw_wait_forced_measurement;
				}
		}
		break;
	case lps35hw_wait_forced_measurement:
		if(HAL_GetTick()>(last_update+100)){
			read_ctrl_reg2();
			if(not(_ctrl_reg2.bit.one_shot)){
				readMeasurement();
				osMutexRelease(*_param.i2cMutex);
				last_update=HAL_GetTick();
				_state=lps35hw_done_state;
			}
		}
		break;
	case lps35hw_err_state:
		_state=lps35hw_init_state;
		break;
	default:
		_state=lps35hw_init_state;
		break;
	}
}

bool lps35hwNB::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY)){
		return true;
	}
	return false;
}

bool lps35hwNB::whoAmI(){
	uint8_t buf[1];
	buf[0]= who_am_i_addr;

	if(HAL_OK==HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY)){
		if(HAL_OK==HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY)){
			if(buf[0]==LPS35HW_WHO_AM_I){
				return true;}
		}
	}
	return false;
}

uint8_t lps35hwNB::readReg8(lps35hw_reg_addr_t RegAddress){
return 0;
}

void lps35hwNB::writeReg8(lps35hw_reg_addr_t RegAddress, uint8_t regValue){
}

uint8_t lps35hwNB_i2c::readReg8(lps35hw_reg_addr_t RegAddress){
	uint8_t buf[1] = {RegAddress};
		if(HAL_OK==HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY)){
			if(HAL_OK==HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY)){
				return buf[0];
			}
		}
		return 0;
}

void lps35hwNB_i2c::writeReg8(lps35hw_reg_addr_t RegAddress, uint8_t regValue){
	uint8_t buf[2] = {RegAddress,regValue};
	HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,2,HAL_MAX_DELAY);
}

void lps35hwNB::read_ctrl_reg1(){
	_ctrl_reg1.reg=readReg8(ctrl_reg1_addr);
}

void lps35hwNB::read_ctrl_reg2(){
	_ctrl_reg2.reg=readReg8(ctrl_reg2_addr);
}

void lps35hwNB::write_ctrl_reg1(){
	_ctrl_reg1.reg=readReg8(ctrl_reg1_addr);
	_ctrl_reg1.bit.odr=_param.odr; // output data rate
	_ctrl_reg1.bit.en_lpfp=_param.en_lpfp;
	_ctrl_reg1.bit.lpfp_cfg=_param.lpfp_cfg;
	_ctrl_reg1.bit.bdu=true;
	_ctrl_reg1.bit.sim=_param.spi_mode; // spi 3 wire or 4 wire mode
	writeReg8(ctrl_reg1_addr, _ctrl_reg1.reg);
}

/*
 * write ctrl_reg2
 * use separate methods for
 * I2C and SPI to set i2c_dis correctly
 *
 */
void lps35hwNB::write_ctrl_reg2(){}

void lps35hwNB_i2c::write_ctrl_reg2(){
	_ctrl_reg2.bit.i2c_dis=false;
	writeReg8(ctrl_reg2_addr, _ctrl_reg2.reg);
}

void lps35hwNB_spi::write_ctrl_reg2(){
	_ctrl_reg2.bit.i2c_dis=true;
	writeReg8(ctrl_reg2_addr, _ctrl_reg2.reg);
}

void lps35hwNB::write_ctrl_reg3(){
	_ctrl_reg3.bit.int_h_l=_param.int_h_l;
	_ctrl_reg3.bit.pp_od=_param.pp_od;
	_ctrl_reg3.bit.f_fss5=_param.f_fss5;
	_ctrl_reg3.bit.f_fth=_param.f_fth;
	_ctrl_reg3.bit.f_ovr=_param.f_ovr;
	_ctrl_reg3.bit.drdy=_param.drdy;
	_ctrl_reg3.bit.int_s2=_param.int_s2;
	_ctrl_reg3.bit.int_s1=_param.int_s1;
	writeReg8(ctrl_reg3_addr,_ctrl_reg3.reg);
}

void lps35hwNB::reset(){
	read_ctrl_reg2();
	_ctrl_reg2.bit.swreset=1;
	write_ctrl_reg2();
}

void lps35hwNB::write_interrupt_cfg(){

}

void lps35hwNB::write_fifo_ctrl(){
	_fifo_ctrl.bit.fifo_mode=_param.fifo_mode;
	_fifo_ctrl.bit.watermark=_param.fifo_threshold;
	writeReg8(fifo_ctrl_addr, _fifo_ctrl.reg);
}

void lps35hwNB::readIntSource(){
	_int_source.reg=readReg8(int_source_addr);
}

uint8_t lps35hwNB::readFifoStatus(){
	return 0;
}

bool lps35hwNB::boot(){
	readIntSource();
	if(_int_source.bit.boot_status==1) return true;
	return false;
}

void lps35hwNB::startMeasurement(){
	_force_measurement=true;
}

void lps35hwNB::_startMeasurement(){
	read_ctrl_reg2();
	_ctrl_reg2.bit.one_shot=true;
	write_ctrl_reg2();
}

bool lps35hwNB::fifoEnabled(){
	read_ctrl_reg2();
	if(_ctrl_reg2.bit.fifo_en==true) return true;
	return false;
}

void lps35hwNB::readMeasurement(){
	uint8_t buf[5];
	int32_t _p,_t;
	buf[0]=press_out_xl_addr;
	if(HAL_OK==HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY)){
				if(HAL_OK==HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,buf,5,HAL_MAX_DELAY)){

					_p=buf[2];
					_p<<=8;
					_p|=buf[1];
					_p<<=8;
					_p|=buf[0];

					_t=buf[4];
					_t<<=8;
					_t|=buf[3];

					if (_p & 0x800000) {
						_p = (0xff000000 | _p);
					  }
					_pressure=(float)_p/4096.0;
					_temp=(float)_t/100.0;

					_newData=true;
				}
	}
}

void lps35hwNB::tempPressure(float *t,float *p){
	*t=_temp;
	*p=_pressure;
	_newData=false;
}

void lps35hwNB::queueData(lps35hw_fifo_entry *_TempPressure){
	if(_lps35hw_fifo.size()<32){
		_lps35hw_fifo.push(*_TempPressure);
	}
}

void lps35hwNB::readFifo(){

}

bool lps35hwNB::newData(){
return _newData;
}

} /* namespace lps35hwNB */
