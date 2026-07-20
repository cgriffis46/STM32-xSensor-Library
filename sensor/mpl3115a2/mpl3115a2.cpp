/*
 * mpl3115a2.cpp
 *
 *  Created on: Dec 29, 2025
 *      Author: coryg
 */

#include "mpl3115a2.h"
#include <cstring>
#include <stdio.h>

namespace mpl3115a2 {

mpl3115a2::mpl3115a2(mpl3115a2_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(mpl3115a2_param_t));
	_state=mpl3115a2_init_state;
	_param._DevAddress = param._DevAddress<<1;
	last_update=HAL_GetTick();
	_new_data=false;
	_temperature=NAN;
	_pressure=NAN;
	_altitude=NAN;
	irq_cfg.reg=0;
	_EnableIRQ=false;
}

mpl3115a2::~mpl3115a2() {
	// TODO Auto-generated destructor stub
}

void mpl3115a2::main(){
	switch(_state){
	case mpl3115a2_init_state:
		if(HAL_GetTick()>(last_update+100)){
			if(checkDevice()){_state=mpl3115a2_whoami_state;} else {_state=mpl3115a2_error_state;}
		}
		break;
	case mpl3115a2_whoami_state:
		if(who_am_i()){
			reset();
			last_update=HAL_GetTick();
			_state=mpl3115a2_reset_state;
		}
		break;
	case mpl3115a2_reset_state:
		if(HAL_GetTick()>(last_update+10)){
			readCtrlReg1();
			if(ctrl_reg1.bit.RST==1){
				break;
			} else {
				_state=mpl3115a2_initializing_state;
				last_update=HAL_GetTick();
			}}
		break;
	case mpl3115a2_initializing_state:
		setupCtrlReg1();
		setupPtDataCfg();

		last_update=HAL_GetTick();
		_state=mpl3115a2_done_state;
		break;
	case mpl3115a2_sleep_state:
		break;
	case mpl3115a2_done_state:
		if(HAL_GetTick()>(last_update+5000)){
			_state=mpl3115a2_initiate_one_shot_measurement;
		}
		break;
	case mpl3115a2_initiate_one_shot_measurement:
		startMeasurement();
		last_update=HAL_GetTick();
		_state=mpl3115a2_wait_one_shot_measurement;
		break;
	case mpl3115a2_wait_one_shot_measurement:
		if(HAL_GetTick()>(last_update+100)){
			if(conversionReady()){
				if(_getConversion()) {
				_new_data=true;
				last_update=HAL_GetTick();
				_state=mpl3115a2_done_state;
				} else {
					_state=mpl3115a2_error_state;
				}
			} else {
				last_update=HAL_GetTick();
			}
		}
		break;
	case mpl3115a2_error_state:
		if(HAL_GetTick()>(last_update+100)){
			_state=mpl3115a2_init_state;
				}
		break;
	default:
		_state=mpl3115a2_init_state;
		break;
	}
}

bool mpl3115a2::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY)){return true;}
	return false;
}

bool mpl3115a2::_readReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t *reg_value){
	uint8_t buf[1];
	buf[0]=(uint8_t)reg_addr;
	//HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	//HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(_param._hi2c,_param._DevAddress,reg_addr,(uint16_t)1, reg_value,(uint16_t)1, 100);
	return true;
}

bool mpl3115a2::_writeReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t reg_value){
	uint8_t buf[2];
	buf[0]=(uint8_t)reg_addr;
	buf[1]=reg_value;

	HAL_I2C_Mem_Write(_param._hi2c,_param._DevAddress, reg_addr, 1, &reg_value, 1, 100000);

	//if(HAL_OK!=HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,2,HAL_MAX_DELAY)){
	//		return false;
	//	}
	return false;
}

bool mpl3115a2::who_am_i(){
	uint8_t who_am_i;
	uint8_t buf[1] = {mpl3115a2_who_am_i_addr};

	HAL_I2C_Mem_Read(_param._hi2c,_param._DevAddress,mpl3115a2_who_am_i_addr,(uint16_t)1, &who_am_i,(uint16_t)1, 100);

	//_readReg8(mpl3115a2_who_am_i_addr,&who_am_i);

	if(who_am_i==mpl3115a2_who_am_i){
		return true;
	}
	return false;
}

bool mpl3115a2::reset(){
	//readCtrlReg1();
	ctrl_reg1.reg=0;
	ctrl_reg1.bit.RST=1;
	writeCtrlReg1();
	return true;
}

bool mpl3115a2::readCtrlReg1(){
		return _readReg8(mpl3115a2_ctrl_reg1_addr,&ctrl_reg1.reg);
}

bool mpl3115a2::readCtrlReg2(){
	if(_readReg8(mpl3115a2_ctrl_reg2_addr,&ctrl_reg2.reg)){
			return true;
		}
	return false;
}

bool mpl3115a2::readCtrlReg3(){
	if(_readReg8(mpl3115a2_ctrl_reg3_addr,&ctrl_reg3.reg)){
			return true;
		}
	return false;
}

bool mpl3115a2::readCtrlReg4(){
	if(_readReg8(mpl3115a2_ctrl_reg4_addr,&ctrl_reg4.reg)){
				return true;
			}
	return false;
}

bool mpl3115a2::readCtrlReg5(){
	if(_readReg8(mpl3115a2_ctrl_reg5_addr,&ctrl_reg5.reg)){
				return true;
			}
	return false;
}

bool mpl3115a2::writeCtrlReg1(){
	//ctrl_reg1.bit.SBYB=1;
	//ctrl_reg1.bit.OS=_param.oversample_ratio;
	return _writeReg8(mpl3115a2_ctrl_reg1_addr,ctrl_reg1.reg);
}

bool mpl3115a2::writeCtrlReg2(){
	if(_writeReg8(mpl3115a2_ctrl_reg2_addr,ctrl_reg2.reg)){
		return true;
	}
	return false;
}

bool mpl3115a2::writeCtrlReg3(){
	if(_writeReg8(mpl3115a2_ctrl_reg3_addr,ctrl_reg3.reg)){
		return true;
	}
	return false;
}

bool mpl3115a2::writeCtrlReg4(){
	if(_writeReg8(mpl3115a2_ctrl_reg4_addr,ctrl_reg4.reg)){
		return true;
	}
	return false;
}

bool mpl3115a2::writeCtrlReg5(){
	if(_writeReg8(mpl3115a2_ctrl_reg5_addr,ctrl_reg5.reg)){
		return true;
	}
	return false;
}

bool mpl3115a2::setupCtrlReg1(){
	readCtrlReg1();
	ctrl_reg1.bit.ALT=_param.mode;
	//ctrl_reg1.bit.SBYB=1;
	//ctrl_reg1.bit.OST=1;
	ctrl_reg1.bit.OS=_param.oversample_ratio;
	writeCtrlReg1();
	return false;
}

bool mpl3115a2::setupCtrlReg2(){
	readCtrlReg2();
	ctrl_reg2.bit.ST=_param.acquisition_time;
	writeCtrlReg2();
	return false;
}

bool mpl3115a2::setupCtrlReg3(){
	readCtrlReg3();
	writeCtrlReg3();
	return false;
}

bool mpl3115a2::setupCtrlReg4(){
	readCtrlReg4();
	writeCtrlReg4();
	return false;
}

bool mpl3115a2::setupCtrlReg5(){
	readCtrlReg5();
	writeCtrlReg5();
	return false;
}

bool mpl3115a2::readIntSource(){
	if(_readReg8(mpl3115a2_int_source_addr,&int_source.reg)){
				return true;
		}
	return false;
}

bool mpl3115a2::startMeasurement(){
	if(readCtrlReg1()) {
		if(ctrl_reg1.bit.OST==1)
			{
				return true;
			}
			else {
				ctrl_reg1.bit.OST=1;
				//ctrl_reg1.bit.OS=_param.oversample_ratio;
				//ctrl_reg1.bit.SBYB=1;
				return writeCtrlReg1();
			}
	}
	return false;
}

bool mpl3115a2::conversionReady(){
	_readReg8(mpl3115a2_status_addr,&dr_status.reg);
	return (dr_status.bit.PTDR!=0);
}

bool mpl3115a2::newData(){
	return _new_data;
}

bool mpl3115a2::readTemp(){
	_readReg8(mpl3115a2_out_t_msb_addr,&OUT_T_MSB);
	_readReg8(mpl3115a2_out_t_lsb_addr,&OUT_T_LSB);
	return false;
}

bool mpl3115a2::_getConversion(){
	uint8_t buf[5]={0,0,0,0,0};
	uint32_t pressure;
	int16_t t;
	HAL_I2C_Mem_Read(_param._hi2c,_param._DevAddress,mpl3115a2_out_p_msb_addr,(uint16_t)1, buf,(uint16_t)5, 100);
		pressure=0;
		pressure |= (int32_t(buf[0])<<16);
		pressure|= (uint32_t(buf[1])<<8);
		pressure|= (uint32_t(buf[2]));

		_pressure=(float(pressure) / 6400.0);
		t = ((uint16_t(buf[3]) << 8) | (uint16_t(buf[4])));
		_temperature=(float(t)/256.0);
	return true;
}

bool mpl3115a2::readDrStatus(){
	return _readReg8(mpl3115a2_dr_status_addr,&dr_status.reg);
}

float mpl3115a2::getTemp(){
	_new_data=false;
	//int16_t t;
	//t = OUT_T_MSB;
	//t<<=8;
	//t|=OUT_T_LSB;
	//t = ((uint16_t(OUT_T_MSB) << 8) | (uint16_t(OUT_T_LSB)));
	//return (float(t)/256.0);
	return _temperature;
}

float mpl3115a2::getPressure(){
	_new_data=false;
	 //uint32_t pressure;
	 //pressure = OUT_P_MSB;
	 //pressure <<= 16;
	 //pressure |= OUT_P_CSB;
	 //pressure <<= 8;
	 //pressure |= OUT_P_LSB;
	 //pressure = ((uint32_t(OUT_P_MSB) << 16) | (uint32_t(OUT_P_CSB) << 8) | uint32_t(OUT_P_LSB));
	 //return (float(pressure) / 6400.0);
	return _pressure;
}

float mpl3115a2::getAltitude(){
	_new_data=false;
	int32_t alt;
	    alt = uint32_t(OUT_P_MSB) << 24 | uint32_t(OUT_P_CSB) << 16 |
	          uint32_t(OUT_P_LSB) << 8;
	    return float(alt) / 65536.0;
}

void mpl3115a2::SetIRQ(irq_en_param_t irq_en, irq_cfg_param_t irq_cfg){
	ctrl_reg4.reg=irq_en.reg;
	ctrl_reg5.reg=irq_cfg.reg;
	_EnableIRQ=true;
}

bool mpl3115a2::readPtDataCfg(){
	return _readReg8(mpl3115a2_pt_data_cfg_addr, &pt_data_cfg.reg);
}

bool mpl3115a2::writePtDataCfg(){
	return _writeReg8(mpl3115a2_pt_data_cfg_addr, pt_data_cfg.reg);
}

bool mpl3115a2::setupPtDataCfg(){
	//readPtDataCfg();
	pt_data_cfg.reg=0;
	pt_data_cfg.bit.DREM=1;
	pt_data_cfg.bit.PDEFE=1;
	pt_data_cfg.bit.TDEFE=1;
	writePtDataCfg();
	return true;
}

bool mpl3115a2::setActive(){
	readCtrlReg1();
	ctrl_reg1.bit.SBYB=1;
	writeCtrlReg1();
	return true;
}

} /* namespace mpl3115a2 */
