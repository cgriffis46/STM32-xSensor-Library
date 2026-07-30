/*
 * bmp390.cpp
 *
 *  Created on: Dec 4, 2025
 *      Author: coryg
 */

#include "bmp390.h"

namespace bmp390 {

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static void delay_ms(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);


bmp390::bmp390(bmp390_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(bmp390_param_t));
	_state=bmp390_init_state;
	temp = NAN; pressure = NAN;
	_newData=false;
	_dev.intf_ptr=_param.hi2c;
	//_dev.chip_id = addr;
	_dev.intf = BMP3_I2C_INTF;
	_dev.read = &i2c_read;
	_dev.write = &i2c_write;
	_dev.dummy_byte = 0;
}

bmp390::~bmp390() {
	// TODO Auto-generated destructor stub
}

bool bmp390::newData(){
	return _newData;
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
	int8_t bmp_res;

	switch(_state){
		case bmp390_init_state:
			if(checkDevice()){
				bmp_res=bmp3_soft_reset(&_dev);
				_state=bmp390_reset_state;
			}
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
