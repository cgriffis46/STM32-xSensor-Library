/*
 * bmp280.tpp
 *
 *  bmp280's member definitions. bmp280.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include bmp280.h, not this file.
 */

#ifndef BMP280_TPP_
#define BMP280_TPP_

#include "../inc/bmp280.h"

namespace BMP280 {

template <typename TTransport>
template <typename... TArgs>
bmp280<TTransport>::bmp280(const bmp280_param_t &param, TArgs&&... transportArgs)
	: base(bmp280_init, bmp280_error, bmp280_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	config.reg=0;
	ctrl_meas.reg=0;
	status.reg=0;
	_id=0;
	t_fine=0;
	adc_t=0;
	adc_p=0;
	_newData=false;
	_temp=NAN;
	_pressure=NAN;
	phase_start=0;
}

template <typename TTransport>
void bmp280<TTransport>::main(uint32_t nowMs){
	switch (_state){
		case bmp280_init:
			_newData=false;
			if(this->checkDevice()){
				enter(bmp280_reset,nowMs);
			} else {
				fail(nowMs);
			}
			break;
		case bmp280_reset:
			issued(this->writeReg(bmp280_reg_reset_addr,bmp280_reset_value),bmp280_wait_reset,nowMs);
			break;
		case bmp280_wait_reset:
			if(landed(nowMs)){
				phase_start=nowMs;
				enter(bmp280_reset_settle,nowMs);
			}
			break;
		case bmp280_reset_settle:
			if(!elapsed(nowMs,bmp280_im_update_poll_ms)){
				sleepRemaining(nowMs,bmp280_im_update_poll_ms);
			} else if(phaseExpired(nowMs)){
				fail(nowMs); // im_update never cleared
			} else {
				enter(bmp280_read_im_update,nowMs);
			}
			break;
		case bmp280_read_im_update:
			issued(this->readRegs(bmp280_reg_status_addr,&status.reg,1),bmp280_wait_im_update,nowMs);
			break;
		case bmp280_wait_im_update:
			// im_update clears when BMP280 completes copying calibration trim values from eeprom into the registers.
			if(landed(nowMs)){
				if(status.bit.im_update!=1){
					enter(bmp280_read_id,nowMs);
				} else {
					enter(bmp280_reset_settle,nowMs);
				}
			}
			break;
		case bmp280_read_id:
			issued(this->readRegs(bmp280_reg_id_addr,&_id,1),bmp280_wait_id,nowMs);
			break;
		case bmp280_wait_id:
			if(landed(nowMs)){
				if(_id==BMP280_ID){
					enter(bmp280_read_cal,nowMs);
				} else {
					fail(nowMs);
				}
			}
			break;
		case bmp280_read_cal:
			issued(this->readRegs(bmp280_reg_cal_addr,calReg,24),bmp280_wait_cal,nowMs);
			break;
		case bmp280_wait_cal:
			if(landed(nowMs)){
				parseCal();
				enter(bmp280_write_config,nowMs);
			}
			break;
		case bmp280_write_config:
			config.bit.t_sb=_param._t_sb;
			config.bit.spi3w_en=bmp280_spi_en_off; // itransport's SPI is 4 wire
			config.bit.filter=_param._filter_t;
			issued(this->writeReg(bmp280_reg_config_addr,config.reg),bmp280_wait_config,nowMs);
			break;
		case bmp280_wait_config:
			if(landed(nowMs)){
				enter(bmp280_write_ctrl_meas,nowMs);
			}
			break;
		case bmp280_write_ctrl_meas:
			ctrl_meas.bit.mode=_param._mode_t;
			ctrl_meas.bit.osrs_p=_param._osrs_p_t;
			ctrl_meas.bit.osrs_t=_param._osrs_t_t;
			issued(this->writeReg(bmp280_reg_ctrl_meas_addr,ctrl_meas.reg),bmp280_wait_ctrl_meas,nowMs);
			break;
		case bmp280_wait_ctrl_meas:
			if(landed(nowMs)){
				enter(bmp280_sleeping,nowMs);
			}
			break;
		case bmp280_sleeping:
			if(elapsed(nowMs,bmp280_period_ms)){
				phase_start=nowMs;
				enter(bmp280_start_measurement,nowMs);
			} else {
				sleepRemaining(nowMs,bmp280_period_ms);
			}
			break;
		case bmp280_start_measurement:
			issued(this->readRegs(bmp280_reg_status_addr,&status.reg,1),bmp280_wait_idle,nowMs);
			break;
		case bmp280_wait_idle:
			if(landed(nowMs)){
				if(status.bit.measuring!=1){
					enter(bmp280_force_measurement,nowMs);
				} else if(phaseExpired(nowMs)){
					fail(nowMs);
				} else { // bmp280 is already measuring. ask again
					sleep(1);
					enter(bmp280_start_measurement,nowMs);
				}
			}
			break;
		case bmp280_force_measurement:
			_param._mode_t=bmp280_forced_mode;
			ctrl_meas.bit.mode=_param._mode_t;
			ctrl_meas.bit.osrs_p=_param._osrs_p_t;
			ctrl_meas.bit.osrs_t=_param._osrs_t_t;
			issued(this->writeReg(bmp280_reg_ctrl_meas_addr,ctrl_meas.reg),bmp280_wait_force,nowMs);
			break;
		case bmp280_wait_force:
			if(landed(nowMs)){
				enter(bmp280_measuring,nowMs);
			}
			break;
		case bmp280_measuring:
			if(elapsed(nowMs,bmp280_conversion_ms)){
				enter(bmp280_read_measuring,nowMs);
			} else {
				sleepRemaining(nowMs,bmp280_conversion_ms);
			}
			break;
		case bmp280_read_measuring:
			issued(this->readRegs(bmp280_reg_status_addr,&status.reg,1),bmp280_wait_measuring,nowMs);
			break;
		case bmp280_wait_measuring:
			if(landed(nowMs)){
				if(status.bit.measuring!=1){
					enter(bmp280_read_data,nowMs);
				} else if(phaseExpired(nowMs)){
					fail(nowMs);
				} else { // still converting. ask again
					sleep(1);
					enter(bmp280_read_measuring,nowMs);
				}
			}
			break;
		case bmp280_read_data:
			issued(this->readRegs(bmp280_reg_press_msb_addr,dataReg,6),bmp280_wait_data,nowMs); // burst read temp/pressure data
			break;
		case bmp280_wait_data:
			if(landed(nowMs)){
				enter(bmp280_convert_measurement,nowMs);
			}
			break;
		case bmp280_convert_measurement:
			adc_p = dataReg[0];
			adc_p = adc_p<<8;
			adc_p |= dataReg[1];
			adc_p = adc_p<<8;
			adc_p |= dataReg[2];
			adc_p = adc_p>>4;

			adc_t = dataReg[3];
			adc_t = adc_t<<8;
			adc_t |= dataReg[4];
			adc_t = adc_t<<8;
			adc_t |= dataReg[5];
			adc_t = adc_t>>4;

			_temp = bmp280_compensate_T_double(adc_t); // must run first. sets t_fine
			_pressure = bmp280_compensate_P_double(adc_p);
			_newData = true;
			enter(bmp280_sleeping,nowMs);
			break;
		case bmp280_error:
			if(errorCleared(nowMs,bmp280_error_backoff_ms)){
				enter(bmp280_init,nowMs);
			}
			break;
		default: // if we don't know what state we're in, re-init
			enter(bmp280_init,nowMs);
			break;
	}
}

template <typename TTransport>
bool bmp280<TTransport>::phaseExpired(uint32_t nowMs) const{
	return (nowMs-phase_start)>bmp280_phase_timeout_ms;
}

template <typename TTransport>
void bmp280<TTransport>::parseCal(){
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

/* Taken from BMP280 Datasheet */
template <typename TTransport>
double bmp280<TTransport>::bmp280_compensate_T_double(BMP280_S32_t adc_T){
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) *((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)*(((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1+var2);
	T=(var1+var2)/5120.0;
	return T;
}

/* Taken from BMP280 Datasheet */
template <typename TTransport>
double bmp280<TTransport>::bmp280_compensate_P_double(BMP280_S32_t adc_P){
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
	var2 = P*((double)dig_P8)/32768.0;
	P = P+(var1+var2+((double)dig_P7))/16.0;
	return P;
}

template <typename TTransport>
bool bmp280<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
bool bmp280<TTransport>::GetTemperature(double *t){
	*t = _temp;
	_newData = false;
	return !isnan(_temp);
}

template <typename TTransport>
bool bmp280<TTransport>::GetPressure(double *p){
	*p = _pressure;
	_newData = false;
	return !isnan(_pressure);
}

} /* namespace BMP280 */

#endif /* BMP280_TPP_ */
