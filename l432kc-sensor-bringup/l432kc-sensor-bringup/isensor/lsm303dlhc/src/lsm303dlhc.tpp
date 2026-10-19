/*
 * lsm303dlhc.tpp
 *
 *  lsm303dlhc_accel's and lsm303dlhc_mag's member definitions.
 *  lsm303dlhc.h includes this file at its end, since templates have
 *  to be visible wherever they are used: include lsm303dlhc.h, not
 *  this file.
 */

#ifndef LSM303DLHC_TPP_
#define LSM303DLHC_TPP_

#include "../inc/lsm303dlhc.h"

namespace LSM303DLHC {

// ---- accelerometer ----

template <typename TTransport>
template <typename... TArgs>
lsm303dlhc_accel<TTransport>::lsm303dlhc_accel(const lsm303dlhc_accel_param_t &param, TArgs&&... transportArgs)
	: base(lsm303dlhc_accel_init_state, lsm303dlhc_accel_error_state, lsm303dlhc_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	phase_start=0;
	_start_measurement=false;
	_newData=false;
	_x=NAN; _y=NAN; _z=NAN;
	_check=0;
	_status=0;
}

template <typename TTransport>
void lsm303dlhc_accel<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case lsm303dlhc_accel_init_state:
		_start_measurement=false;
		if(this->checkDevice()){
			enter(lsm303dlhc_accel_ctrl1_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;
	case lsm303dlhc_accel_ctrl1_state:
		issued(this->writeReg(lsm303dlhc_ctrl_reg1_a,ctrl1Value()),lsm303dlhc_accel_wait_ctrl1_state,nowMs);
		break;
	case lsm303dlhc_accel_wait_ctrl1_state:
		if(landed(nowMs)){
			enter(lsm303dlhc_accel_check_state,nowMs);
		}
		break;
	// The accelerometer has no ID register. Reading back what was just
	// written is the check that it is the chip, and that it took it.
	case lsm303dlhc_accel_check_state:
		issued(this->readRegs(lsm303dlhc_ctrl_reg1_a,&_check,1),lsm303dlhc_accel_wait_check_state,nowMs);
		break;
	case lsm303dlhc_accel_wait_check_state:
		if(landed(nowMs)){
			if(_check==ctrl1Value()){
				enter(lsm303dlhc_accel_ctrl4_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case lsm303dlhc_accel_ctrl4_state:
		{
			const uint8_t ctrl4=LSM303DLHC_CTRL_REG4_A_BDU|(uint8_t)((_param.scale&0x03)<<4)|LSM303DLHC_CTRL_REG4_A_HR;
			issued(this->writeReg(lsm303dlhc_ctrl_reg4_a,ctrl4),lsm303dlhc_accel_wait_ctrl4_state,nowMs);
		}
		break;
	case lsm303dlhc_accel_wait_ctrl4_state:
		if(landed(nowMs)){
			enter(lsm303dlhc_accel_done_state,nowMs);
		}
		break;
	case lsm303dlhc_accel_done_state:
		if(elapsed(nowMs,_param.period_ms)||_start_measurement){
			_start_measurement=false;
			phase_start=nowMs;
			enter(lsm303dlhc_accel_status_state,nowMs);
		} else {
			// No longer than lsm303dlhc_request_poll_ms at a time, so a
			// startMeasurement() request is seen without waiting out
			// the whole period.
			const uint32_t left=_param.period_ms-(nowMs-last_update);
			sleep(left<lsm303dlhc_request_poll_ms?left:lsm303dlhc_request_poll_ms);
		}
		break;
	case lsm303dlhc_accel_status_state:
		issued(this->readRegs(lsm303dlhc_status_reg_a,&_status,1),lsm303dlhc_accel_wait_status_state,nowMs);
		break;
	case lsm303dlhc_accel_wait_status_state:
		if(landed(nowMs)){
			if(_status&LSM303DLHC_STATUS_REG_A_ZYXDA){
				enter(lsm303dlhc_accel_read_state,nowMs);
			} else if((nowMs-phase_start)>lsm303dlhc_sample_timeout_ms){
				fail(nowMs); // the chip has stopped sampling
			} else { // no new sample yet. ask again
				sleep(1);
				enter(lsm303dlhc_accel_status_state,nowMs);
			}
		}
		break;
	case lsm303dlhc_accel_read_state:
		issued(this->readRegs(lsm303dlhc_out_x_l_a|LSM303DLHC_AUTO_INCREMENT,dataReg,6),lsm303dlhc_accel_wait_read_state,nowMs);
		break;
	case lsm303dlhc_accel_wait_read_state:
		if(landed(nowMs)){
			// 12 bits, left justified in 16, low byte first.
			const float scale=mgPerLsb()*0.001f*lsm303dlhc_gravity;
			_x=(float)((int16_t)(((uint16_t)dataReg[1]<<8)|dataReg[0])>>4)*scale;
			_y=(float)((int16_t)(((uint16_t)dataReg[3]<<8)|dataReg[2])>>4)*scale;
			_z=(float)((int16_t)(((uint16_t)dataReg[5]<<8)|dataReg[4])>>4)*scale;
			_newData=true;
			enter(lsm303dlhc_accel_done_state,nowMs);
		}
		break;
	case lsm303dlhc_accel_error_state:
		if(errorCleared(nowMs,lsm303dlhc_error_backoff_ms)){
			enter(lsm303dlhc_accel_init_state,nowMs);
		}
		break;
	default:
		enter(lsm303dlhc_accel_init_state,nowMs);
		break;
	}
}

// The data rate in bits 7:4, normal mode (not low power), and the
// x, y and z axes on.
template <typename TTransport>
uint8_t lsm303dlhc_accel<TTransport>::ctrl1Value() const{
	return (uint8_t)(((_param.odr&0x0F)<<4)|LSM303DLHC_CTRL_REG1_A_XYZ_EN);
}

// Sensitivity at 12 bits, from the datasheet.
template <typename TTransport>
float lsm303dlhc_accel<TTransport>::mgPerLsb() const{
	switch(_param.scale){
	case lsm303dlhc_accel_4g:  return 2.0f;
	case lsm303dlhc_accel_8g:  return 4.0f;
	case lsm303dlhc_accel_16g: return 12.0f;
	default:                   return 1.0f;
	}
}

template <typename TTransport>
void lsm303dlhc_accel<TTransport>::startMeasurement(){
	_start_measurement=true;
}

template <typename TTransport>
bool lsm303dlhc_accel<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void lsm303dlhc_accel<TTransport>::getAccel(float *x, float *y, float *z){
	*x=_x;
	*y=_y;
	*z=_z;
	_newData=false;
}

// ---- magnetometer ----

template <typename TTransport>
template <typename... TArgs>
lsm303dlhc_mag<TTransport>::lsm303dlhc_mag(const lsm303dlhc_mag_param_t &param, TArgs&&... transportArgs)
	: base(lsm303dlhc_mag_init_state, lsm303dlhc_mag_error_state, lsm303dlhc_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	_start_measurement=false;
	_newData=false;
	_x=NAN; _y=NAN; _z=NAN;
	_id[0]=_id[1]=_id[2]=0;
}

template <typename TTransport>
void lsm303dlhc_mag<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case lsm303dlhc_mag_init_state:
		_start_measurement=false;
		if(this->checkDevice()){
			enter(lsm303dlhc_mag_id_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;
	case lsm303dlhc_mag_id_state:
		issued(this->readRegs(lsm303dlhc_ira_reg_m,_id,3),lsm303dlhc_mag_wait_id_state,nowMs);
		break;
	case lsm303dlhc_mag_wait_id_state:
		if(landed(nowMs)){
			if(_id[0]=='H'&&_id[1]=='4'&&_id[2]=='3'){
				enter(lsm303dlhc_mag_cra_state,nowMs);
			} else {
				fail(nowMs); // something else is at this address
			}
		}
		break;
	case lsm303dlhc_mag_cra_state:
		issued(this->writeReg(lsm303dlhc_cra_reg_m,(uint8_t)((_param.rate&0x07)<<2)),lsm303dlhc_mag_wait_cra_state,nowMs);
		break;
	case lsm303dlhc_mag_wait_cra_state:
		if(landed(nowMs)){
			enter(lsm303dlhc_mag_crb_state,nowMs);
		}
		break;
	case lsm303dlhc_mag_crb_state:
		issued(this->writeReg(lsm303dlhc_crb_reg_m,(uint8_t)((_param.gain&0x07)<<5)),lsm303dlhc_mag_wait_crb_state,nowMs);
		break;
	case lsm303dlhc_mag_wait_crb_state:
		if(landed(nowMs)){
			enter(lsm303dlhc_mag_mr_state,nowMs);
		}
		break;
	case lsm303dlhc_mag_mr_state:
		issued(this->writeReg(lsm303dlhc_mr_reg_m,LSM303DLHC_MR_REG_M_CONTINUOUS),lsm303dlhc_mag_wait_mr_state,nowMs);
		break;
	case lsm303dlhc_mag_wait_mr_state:
		if(landed(nowMs)){
			enter(lsm303dlhc_mag_done_state,nowMs);
		}
		break;
	case lsm303dlhc_mag_done_state:
		if(elapsed(nowMs,_param.period_ms)||_start_measurement){
			_start_measurement=false;
			enter(lsm303dlhc_mag_read_state,nowMs);
		} else {
			const uint32_t left=_param.period_ms-(nowMs-last_update);
			sleep(left<lsm303dlhc_request_poll_ms?left:lsm303dlhc_request_poll_ms);
		}
		break;
	// The chip converts continuously, so this just reads the latest
	// sample. The data ready flag is left alone: with the period longer
	// than a conversion there is always a sample to read.
	case lsm303dlhc_mag_read_state:
		issued(this->readRegs(lsm303dlhc_out_x_h_m,dataReg,6),lsm303dlhc_mag_wait_read_state,nowMs);
		break;
	case lsm303dlhc_mag_wait_read_state:
		if(landed(nowMs)){
			// x, z, y in that order, high byte first.
			const int16_t rx=(int16_t)(((uint16_t)dataReg[0]<<8)|dataReg[1]);
			const int16_t rz=(int16_t)(((uint16_t)dataReg[2]<<8)|dataReg[3]);
			const int16_t ry=(int16_t)(((uint16_t)dataReg[4]<<8)|dataReg[5]);
			_x=toMicroTesla(rx,lsbPerGaussXY());
			_y=toMicroTesla(ry,lsbPerGaussXY());
			_z=toMicroTesla(rz,lsbPerGaussZ());
			_newData=true;
			enter(lsm303dlhc_mag_done_state,nowMs);
		}
		break;
	case lsm303dlhc_mag_error_state:
		if(errorCleared(nowMs,lsm303dlhc_error_backoff_ms)){
			enter(lsm303dlhc_mag_init_state,nowMs);
		}
		break;
	default:
		enter(lsm303dlhc_mag_init_state,nowMs);
		break;
	}
}

template <typename TTransport>
float lsm303dlhc_mag<TTransport>::toMicroTesla(int16_t raw, float lsbPerGauss) const{
	if(raw==LSM303DLHC_MAG_OVERFLOW) return NAN;
	return (float)raw/lsbPerGauss*lsm303dlhc_ut_per_gauss;
}

// Sensitivity for each gain, from the datasheet. z differs from x and y.
template <typename TTransport>
float lsm303dlhc_mag<TTransport>::lsbPerGaussXY() const{
	switch(_param.gain){
	case lsm303dlhc_mag_1_9g: return 855.0f;
	case lsm303dlhc_mag_2_5g: return 670.0f;
	case lsm303dlhc_mag_4_0g: return 450.0f;
	case lsm303dlhc_mag_4_7g: return 400.0f;
	case lsm303dlhc_mag_5_6g: return 330.0f;
	case lsm303dlhc_mag_8_1g: return 230.0f;
	default:                  return 1100.0f;
	}
}

template <typename TTransport>
float lsm303dlhc_mag<TTransport>::lsbPerGaussZ() const{
	switch(_param.gain){
	case lsm303dlhc_mag_1_9g: return 760.0f;
	case lsm303dlhc_mag_2_5g: return 600.0f;
	case lsm303dlhc_mag_4_0g: return 400.0f;
	case lsm303dlhc_mag_4_7g: return 355.0f;
	case lsm303dlhc_mag_5_6g: return 295.0f;
	case lsm303dlhc_mag_8_1g: return 205.0f;
	default:                  return 980.0f;
	}
}

template <typename TTransport>
void lsm303dlhc_mag<TTransport>::startMeasurement(){
	_start_measurement=true;
}

template <typename TTransport>
bool lsm303dlhc_mag<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void lsm303dlhc_mag<TTransport>::getMag(float *x, float *y, float *z){
	*x=_x;
	*y=_y;
	*z=_z;
	_newData=false;
}

} /* namespace LSM303DLHC */

#endif /* LSM303DLHC_TPP_ */
