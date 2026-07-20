/*
 * bmp390.cpp
 *
 *  Created on: Dec 4, 2025
 *      Author: coryg
 */

#include "bmp390.h"

namespace bmp390 {

bmp390::bmp390(bmp390_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(bmp390_param_t));
	_state=bmp390_init_state;
	temp = NAN; pressure = NAN;
	_newData=false;
}

bmp390::~bmp390() {
	// TODO Auto-generated destructor stub
}

bool bmp390::newData(){
	return _newData;
}

bool bmp390::ReadReg8(bmp390_reg_t *_reg, bmp390_reg_addr_t addr){
	uint8_t buf[1];
	buf[0]=addr;
	buf[1]=0;
	HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	*_reg=buf[0];
	return true;
}

bool bmp390::WriteReg8(bmp390_reg_t *_reg,bmp390_reg_addr_t _addr){
	uint8_t buf[2];
	buf[0]=_addr;
	buf[1]=*_reg;
	HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,2,HAL_MAX_DELAY);
	return true;
}

void bmp390::ReadRegMulti(bmp390_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs){
	buf[0]=addr_t;
	HAL_I2C_Master_Transmit(_param._hi2c,_param._DevAddress,buf,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(_param._hi2c,_param._DevAddress,&buf[0],numRegs,HAL_MAX_DELAY);
}

static float BMP390_compensate_temperature(uint32_t uncomp_temp, BMP390_calib_data *calib_data)
{
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp-calib_data->par_t1);
	partial_data2 = (float)(partial_data1 * calib_data->par_t2);
	/* Update the compensated temperature in calib structure since this is
	 * needed for pressure calculation */
	calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
	/* Returns compensated temperature */
	return calib_data->t_lin;
}

static float BMP390_compensate_pressure(uint32_t uncomp_press, BMP390_calib_data *calib_data)
{
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	/* Calibration data */
	partial_data1 = calib_data->par_p6 * calib_data->t_lin;
	partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data->par_p2 * calib_data->t_lin;
	partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;
	return comp_press;
}

bool bmp390::ReadCalData(){
	uint8_t buf[21];
	ReadRegMulti(bmp390_reg_NVM_PAR_T1_addr,buf,21);

	NVM_PAR_T1=(buf[1]<<8);
	NVM_PAR_T1|=buf[0];
	NVM_PAR_T2=buf[3]<<8;
	NVM_PAR_T2|=buf[2];
	NVM_PAR_T3=buf[4];
	NVM_PAR_P1=buf[6]<<8;
	NVM_PAR_P1|=buf[5];
	NVM_PAR_P2=buf[8]<<8;
	NVM_PAR_P2|=buf[7];
	NVM_PAR_P3=buf[9];
	NVM_PAR_P4=buf[10];
	NVM_PAR_P5=buf[12]<<8;
	NVM_PAR_P5|=buf[11];
	NVM_PAR_P6=buf[14]<<8;
	NVM_PAR_P6|=buf[13];
	NVM_PAR_P7=buf[15];
	NVM_PAR_P8=buf[16];
	NVM_PAR_P9=buf[18]<<8;
	NVM_PAR_P9|=buf[17];
	NVM_PAR_P10=buf[19];
	NVM_PAR_P11=buf[20];

	_calib_data.par_t1=NVM_PAR_T1/(2^-8);
	_calib_data.par_t2=NVM_PAR_T2/(1<<30);
	_calib_data.par_t3=NVM_PAR_T3/((uint64_t)1<<48);
	_calib_data.par_p1=(NVM_PAR_P1-(1<<14))/(1<<20);
	_calib_data.par_p2=(NVM_PAR_P2-(1<<14))/(1<<29);
	_calib_data.par_p3=(NVM_PAR_P3)/(1<<32);
	_calib_data.par_p4=(NVM_PAR_P4)/(1<<37);
	_calib_data.par_p5=(NVM_PAR_P5)/(2^-3);
	_calib_data.par_p6=(NVM_PAR_P6)/(1<<6);
	_calib_data.par_p7=(NVM_PAR_P7)/(1<<8);
	_calib_data.par_p8=(NVM_PAR_P8)/(1<<15);
	_calib_data.par_p9=(NVM_PAR_P9)/(1<<48);
	_calib_data.par_p10=(NVM_PAR_P10)/(1<<48);
	_calib_data.par_p11=(NVM_PAR_P11)/((2.0^65));

	return true;
}

bool bmp390::Reset(){

}

bool bmp390::ReadStatusReg(){
	return ReadReg8(&status_reg.reg,bmp390_reg_status_addr);
}

bool bmp390::ReadEventReg(){

}

bool bmp390::WriteFifoConfig1Reg(){

}

bool bmp390::WriteIntCtrlReg(){

}

bool bmp390::WriteIfConfReg(){

}

bool bmp390::WritePwrCtrlReg(){

}

bool bmp390::WriteOsrReg(){
//	osr_reg.reg=_param.
}

bool bmp390::WriteOdrReg(){

}

bool bmp390::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY)){
		return true;
	}
	return false;
}

bool ReadTempPressure(){
	return false;
}

void bmp390::main(){
	switch(_state){
		case bmp390_init_state:
			break;
		case bmp390_reset_state:
			break;
		case bmp390_err_state:
			break;

		default:
			_state=bmp390_init_state;
			break;
	}
}

} /* namespace bmp390 */
