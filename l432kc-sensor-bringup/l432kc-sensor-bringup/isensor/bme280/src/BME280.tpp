/*
 * BME280.tpp
 *
 *  BME280's member definitions. BME280.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include BME280.h, not this file.
 */

#ifndef BME280_TPP_
#define BME280_TPP_

#include "../inc/BME280.h"

namespace BME280 {

template <typename TTransport>
template <typename... TArgs>
bme280<TTransport>::bme280(const bme280_param_t &param, TArgs&&... transportArgs)
	: base(bme280_init, bme280_error, bme280_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	_config.reg=0;
	_ctrl_meas.reg=0;
	_status.reg=0;
	_ctrl_hum.reg=0;
	_id=0;
	t_fine=0;
	adc_t=0;
	adc_p=0;
	adc_h=0;
	_newData=false;
	_temp=NAN;
	_pressure=NAN;
	_humidity=NAN;
	phase_start=0;
}

template <typename TTransport>
void bme280<TTransport>::main(uint32_t nowMs){
	switch (_state){
		case bme280_init:
			_newData=false;
			enter(bme280_reset,nowMs);
			break;
		case bme280_reset:
			issued(this->writeReg(bme280_reg_reset_addr,bme280_reset_value),bme280_wait_reset,nowMs);
			break;
		case bme280_wait_reset:
			if(landed(nowMs)){
				phase_start=nowMs;
				enter(bme280_reset_settle,nowMs);
			}
			break;
		case bme280_reset_settle:
			// wait 100ms after issuing Reset
			if(elapsed(nowMs,bme280_reset_settle_ms)){
				enter(bme280_read_id,nowMs);
			} else {
				sleepRemaining(nowMs,bme280_reset_settle_ms);
			}
			break;
		case bme280_read_id:
			issued(this->readRegs(bme280_reg_id_addr,&_id,1),bme280_wait_id,nowMs);
			break;
		case bme280_wait_id:
			if(landed(nowMs)){
				if(_id==bme280_id){
					enter(bme280_im_update_pause,nowMs);
				} else {
					fail(nowMs);
				}
			}
			break;
		case bme280_im_update_pause:
			if(!elapsed(nowMs,bme280_im_update_poll_ms)){
				sleepRemaining(nowMs,bme280_im_update_poll_ms);
			} else if(phaseExpired(nowMs)){
				fail(nowMs); // im_update never cleared
			} else {
				enter(bme280_read_im_update,nowMs);
			}
			break;
		case bme280_read_im_update:
			issued(this->readRegs(bme280_reg_status_addr,&_status.reg,1),bme280_wait_im_update,nowMs);
			break;
		case bme280_wait_im_update:
			// im_update clears when BME280 completes copying calibration trim values from eeprom into the registers.
			if(landed(nowMs)){
				if(_status.bit.im_update!=1){
					enter(bme280_read_cal,nowMs);
				} else {
					enter(bme280_im_update_pause,nowMs);
				}
			}
			break;
		case bme280_read_cal:
			issued(this->readRegs(bme280_reg_cal_addr,calReg,24),bme280_wait_cal,nowMs);
			break;
		case bme280_wait_cal:
			if(landed(nowMs)){
				parseCal();
				enter(bme280_read_cal_h1,nowMs);
			}
			break;
		case bme280_read_cal_h1:
			issued(this->readRegs(bme280_reg_dig_H1_addr,&calH1Reg,1),bme280_wait_cal_h1,nowMs);
			break;
		case bme280_wait_cal_h1:
			if(landed(nowMs)){
				enter(bme280_read_cal_h2,nowMs);
			}
			break;
		case bme280_read_cal_h2:
			issued(this->readRegs(bme280_reg_dig_H2_addr,calHReg,7),bme280_wait_cal_h2,nowMs);
			break;
		case bme280_wait_cal_h2:
			if(landed(nowMs)){
				parseHumCal();
				enter(bme280_write_ctrl_hum,nowMs);
			}
			break;
		case bme280_write_ctrl_hum:
			// Must come before ctrl_meas: the chip only takes up a new
			// ctrl_hum when ctrl_meas is next written.
			_ctrl_hum.bit.osrs_h=_param._osrs_h_t;
			issued(this->writeReg(bme280_reg_ctrl_hum_addr,_ctrl_hum.reg),bme280_wait_ctrl_hum,nowMs);
			break;
		case bme280_wait_ctrl_hum:
			if(landed(nowMs)){
				enter(bme280_write_config,nowMs);
			}
			break;
		case bme280_write_config:
			_config.bit.t_sb=_param._sb_t;
			_config.bit.spi3w_en=bme280_spi_en_off; // itransport's SPI is 4 wire
			_config.bit.filter=_param._filter_t;
			issued(this->writeReg(bme280_reg_config_addr,_config.reg),bme280_wait_config,nowMs);
			break;
		case bme280_wait_config:
			if(landed(nowMs)){
				enter(bme280_write_ctrl_meas,nowMs);
			}
			break;
		case bme280_write_ctrl_meas:
			_ctrl_meas.bit.mode=_param._mode_t;
			_ctrl_meas.bit.osrs_p=_param._osrs_p_t;
			_ctrl_meas.bit.osrs_t=_param._osrs_t_t;
			issued(this->writeReg(bme280_reg_ctrl_meas_addr,_ctrl_meas.reg),bme280_wait_ctrl_meas,nowMs);
			break;
		case bme280_wait_ctrl_meas:
			if(landed(nowMs)){
				enter(bme280_sleeping,nowMs);
			}
			break;
		case bme280_sleeping:
			if(elapsed(nowMs,bme280_sleep_ms)){
				phase_start=nowMs;
				enter(bme280_start_measurement,nowMs);
			} else {
				sleepRemaining(nowMs,bme280_sleep_ms);
			}
			break;
		case bme280_start_measurement:
			issued(this->readRegs(bme280_reg_status_addr,&_status.reg,1),bme280_wait_idle,nowMs);
			break;
		case bme280_wait_idle:
			if(landed(nowMs)){
				if(_status.bit.measuring!=1){ // bme280 is not already measuring
					enter(bme280_force_measurement,nowMs);
				} else if(phaseExpired(nowMs)){
					fail(nowMs);
				} else { // ask again
					sleep(1);
					enter(bme280_start_measurement,nowMs);
				}
			}
			break;
		case bme280_force_measurement:
			// start forced measurement
			_param._mode_t=bme280_forced_mode;
			_ctrl_meas.bit.mode=_param._mode_t;
			_ctrl_meas.bit.osrs_p=_param._osrs_p_t;
			_ctrl_meas.bit.osrs_t=_param._osrs_t_t;
			issued(this->writeReg(bme280_reg_ctrl_meas_addr,_ctrl_meas.reg),bme280_wait_force,nowMs);
			break;
		case bme280_wait_force:
			if(landed(nowMs)){
				enter(bme280_measuring,nowMs);
			}
			break;
		case bme280_measuring:
			if(elapsed(nowMs,bme280_conversion_ms)){
				enter(bme280_read_measuring,nowMs);
			} else {
				sleepRemaining(nowMs,bme280_conversion_ms);
			}
			break;
		case bme280_read_measuring:
			issued(this->readRegs(bme280_reg_status_addr,&_status.reg,1),bme280_wait_measuring,nowMs);
			break;
		case bme280_wait_measuring:
			if(landed(nowMs)){
				if(_status.bit.measuring!=1){
					enter(bme280_read_data,nowMs);
				} else if(phaseExpired(nowMs)){
					fail(nowMs);
				} else { // still converting. ask again
					sleep(1);
					enter(bme280_read_measuring,nowMs);
				}
			}
			break;
		case bme280_read_data:
			issued(this->readRegs(bme280_reg_press_msb_addr,dataReg,8),bme280_wait_data,nowMs); // burst read temp/pressure/humidity data
			break;
		case bme280_wait_data:
			if(landed(nowMs)){
				enter(bme280_convert_measurement,nowMs);
			}
			break;
		case bme280_convert_measurement:
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

			adc_h = dataReg[6];
			adc_h = adc_h<<8;
			adc_h |= dataReg[7];

			_temp = bme280_compensate_T_double(adc_t); // must run first. sets t_fine
			_pressure = bme280_compensate_P_double(adc_p);
			_humidity = bme280_compensate_H_double(adc_h);
			_newData = true;
			enter(bme280_sleeping,nowMs);
			break;
		case bme280_error:
			if(errorCleared(nowMs,bme280_error_backoff_ms)){
				enter(bme280_init,nowMs);
			}
			break;
		default: // if we don't know what state we're in, re-init
			enter(bme280_init,nowMs);
			break;
	}
}

template <typename TTransport>
bool bme280<TTransport>::phaseExpired(uint32_t nowMs) const{
	return (nowMs-phase_start)>bme280_phase_timeout_ms;
}

template <typename TTransport>
void bme280<TTransport>::parseCal(){
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

template <typename TTransport>
void bme280<TTransport>::parseHumCal(){
	uint8_t temp;

	dig_H1 = calH1Reg;

	dig_H2 = calHReg[1];
	dig_H2 = dig_H2<<8;
	dig_H2 |= calHReg[0];

	dig_H3 = calHReg[2];

	// dig_H4 and dig_H5 are signed 12 bit values. The byte holding
	// the top 8 bits carries the sign, so it is read as signed.
	dig_H4 = (signed char)calHReg[3];
	dig_H4 = dig_H4*16;
	dig_H4 |= (calHReg[4]&0b00001111);

	temp = calHReg[4]&0b11110000;
	temp = temp>>4;
	dig_H5 = (signed char)calHReg[5];//0xE6
	dig_H5 = dig_H5*16;
	dig_H5 |= temp;

	dig_H6 = (signed char)calHReg[6];
}

/* Taken from bme280 Datasheet */
template <typename TTransport>
double bme280<TTransport>::bme280_compensate_T_double(bme280_S32_t adc_T){
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) *((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)*(((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	t_fine = (bme280_S32_t)(var1+var2);
	T=(var1+var2)/5120.0;
	return T;
}

/* Taken from bme280 Datasheet */
template <typename TTransport>
double bme280<TTransport>::bme280_compensate_P_double(bme280_S32_t adc_P){
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

/* Taken from bme280 Datasheet */
template <typename TTransport>
double bme280<TTransport>::bme280_compensate_H_double(bme280_S32_t adc_H){
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

template <typename TTransport>
bool bme280<TTransport>::newData(){
	bool _new = _newData;
	_newData = false;
	return _new;
}

template <typename TTransport>
bool bme280<TTransport>::GetTemperature(double *t){
	*t = _temp;
	_newData = false;
	return !isnan(_temp);
}

template <typename TTransport>
bool bme280<TTransport>::GetPressure(double *p){
	*p = _pressure;
	_newData=false;
	return !isnan(_pressure);
}

template <typename TTransport>
bool bme280<TTransport>::GetHumidity(double *h){
	*h = _humidity;
	_newData=false;
	return !isnan(_humidity);
}

} /* namespace BME280 */

#endif /* BME280_TPP_ */
