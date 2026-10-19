/*
 * lps35hw.tpp
 *
 *  lps35hw's member definitions. lps35hw.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include lps35hw.h, not this file.
 */

#ifndef LPS35HW_TPP_
#define LPS35HW_TPP_

#include "../inc/lps35hw.h"

namespace LPS35HW {

template <typename TTransport>
template <typename... TArgs>
lps35hw<TTransport>::lps35hw(const lps35hw_param_t &param, TArgs&&... transportArgs)
	: base(lps35hw_init_state, lps35hw_err_state, lps35hw_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	_force_measurement=false;
	_newData=false;
	_pressure=NAN;
	_temp=NAN;
	_who_am_i=0;
	phase_start=0;
	_ctrl_reg1.reg=0;
	_ctrl_reg2.reg=0;
	_ctrl_reg3.reg=0;
	_fifo_ctrl.reg=0;
	_int_source.reg=0;
}

template <typename TTransport>
void lps35hw<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case lps35hw_init_state:
		if(this->checkDevice()){
			enter(lps35hw_who_am_i_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;
	case lps35hw_who_am_i_state:
		issued(this->readRegs(who_am_i_addr,&_who_am_i,1),lps35hw_wait_who_am_i_state,nowMs);
		break;
	case lps35hw_wait_who_am_i_state:
		if(landed(nowMs)){
			if(_who_am_i==LPS35HW_WHO_AM_I){
				enter(lps35hw_reset_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case lps35hw_reset_state:
		issued(this->writeReg(ctrl_reg2_addr,ctrlReg2(true,false)),lps35hw_wait_reset_state,nowMs);
		break;
	case lps35hw_wait_reset_state:
		if(landed(nowMs)){
			phase_start=nowMs;
			enter(lps35hw_reset_settle_state,nowMs);
		}
		break;
	case lps35hw_reset_settle_state:
		if(!elapsed(nowMs,lps35hw_reset_settle_ms)){
			sleepRemaining(nowMs,lps35hw_reset_settle_ms);
		} else if(phaseExpired(nowMs)){
			fail(nowMs); // boot_status never cleared
		} else {
			enter(lps35hw_boot_state,nowMs);
		}
		break;
	case lps35hw_boot_state:
		issued(this->readRegs(int_source_addr,&_int_source.reg,1),lps35hw_wait_boot_state,nowMs);
		break;
	case lps35hw_wait_boot_state:
		if(landed(nowMs)){
			if(_int_source.bit.boot_status!=1){
				enter(lps35hw_initializing_state,nowMs);
			} else {
				enter(lps35hw_reset_settle_state,nowMs);
			}
		}
		break;
	case lps35hw_initializing_state:
		_ctrl_reg1.reg=0;
		_ctrl_reg1.bit.odr=_param.odr; // output data rate
		_ctrl_reg1.bit.en_lpfp=_param.en_lpfp;
		_ctrl_reg1.bit.lpfp_cfg=_param.lpfp_cfg;
		_ctrl_reg1.bit.bdu=true;
		_ctrl_reg1.bit.sim=false; // 4 wire. itransport's SPI is 4 wire
		issued(this->writeReg(ctrl_reg1_addr,_ctrl_reg1.reg),lps35hw_wait_ctrl_reg1_state,nowMs);
		break;
	case lps35hw_wait_ctrl_reg1_state:
		if(landed(nowMs)){
			enter(lps35hw_ctrl_reg2_state,nowMs);
		}
		break;
	case lps35hw_ctrl_reg2_state:
		issued(this->writeReg(ctrl_reg2_addr,ctrlReg2(false,false)),lps35hw_wait_ctrl_reg2_state,nowMs);
		break;
	case lps35hw_wait_ctrl_reg2_state:
		if(landed(nowMs)){
			enter(lps35hw_ctrl_reg3_state,nowMs);
		}
		break;
	case lps35hw_ctrl_reg3_state:
		_ctrl_reg3.bit.int_h_l=_param.int_h_l;
		_ctrl_reg3.bit.pp_od=_param.pp_od;
		_ctrl_reg3.bit.f_fss5=_param.f_fss5;
		_ctrl_reg3.bit.f_fth=_param.f_fth;
		_ctrl_reg3.bit.f_ovr=_param.f_ovr;
		_ctrl_reg3.bit.drdy=_param.drdy;
		_ctrl_reg3.bit.int_s2=_param.int_s2;
		_ctrl_reg3.bit.int_s1=_param.int_s1;
		issued(this->writeReg(ctrl_reg3_addr,_ctrl_reg3.reg),lps35hw_wait_ctrl_reg3_state,nowMs);
		break;
	case lps35hw_wait_ctrl_reg3_state:
		if(landed(nowMs)){
			enter(lps35hw_fifo_ctrl_state,nowMs);
		}
		break;
	case lps35hw_fifo_ctrl_state:
		_fifo_ctrl.bit.fifo_mode=_param.fifo_mode;
		_fifo_ctrl.bit.watermark=_param.fifo_threshold;
		issued(this->writeReg(fifo_ctrl_addr,_fifo_ctrl.reg),lps35hw_wait_fifo_ctrl_state,nowMs);
		break;
	case lps35hw_wait_fifo_ctrl_state:
		if(landed(nowMs)){
			enter(lps35hw_done_state,nowMs);
		}
		break;
	case lps35hw_done_state:
		if(elapsed(nowMs,lps35hw_period_ms)||(_force_measurement&&elapsed(nowMs,lps35hw_forced_ms))){
			_force_measurement=false;
			phase_start=nowMs;
			if(_param.odr==lps35hw_odr_0hz){
				enter(lps35hw_start_measurement_state,nowMs);
			} else { // the chip is measuring by itself. read the latest
				enter(lps35hw_read_measurement_state,nowMs);
			}
		} else {
			// No longer than lps35hw_forced_ms at a time, so a
			// startMeasurement() request is seen without waiting out
			// the whole period.
			uint32_t waited=nowMs-last_update;
			uint32_t left=lps35hw_period_ms-waited;
			sleep(left<lps35hw_forced_ms?left:lps35hw_forced_ms);
		}
		break;
	case lps35hw_start_measurement_state:
		issued(this->writeReg(ctrl_reg2_addr,ctrlReg2(false,true)),lps35hw_wait_start_measurement_state,nowMs);
		break;
	case lps35hw_wait_start_measurement_state:
		if(landed(nowMs)){
			enter(lps35hw_wait_forced_measurement,nowMs);
		}
		break;
	case lps35hw_wait_forced_measurement:
		if(elapsed(nowMs,lps35hw_conversion_ms)){
			enter(lps35hw_one_shot_state,nowMs);
		} else {
			sleepRemaining(nowMs,lps35hw_conversion_ms);
		}
		break;
	case lps35hw_one_shot_state:
		issued(this->readRegs(ctrl_reg2_addr,&_ctrl_reg2.reg,1),lps35hw_wait_one_shot_state,nowMs);
		break;
	case lps35hw_wait_one_shot_state:
		if(landed(nowMs)){
			if(!_ctrl_reg2.bit.one_shot){ // the chip clears one_shot when the measurement is done
				enter(lps35hw_read_measurement_state,nowMs);
			} else if(phaseExpired(nowMs)){
				fail(nowMs);
			} else { // still measuring. ask again
				sleep(1);
				enter(lps35hw_one_shot_state,nowMs);
			}
		}
		break;
	case lps35hw_read_measurement_state:
		issued(this->readRegs(press_out_xl_addr,dataReg,5),lps35hw_wait_measurement_state,nowMs);
		break;
	case lps35hw_wait_measurement_state:
		if(landed(nowMs)){
			int32_t _p;
			int16_t _t;

			_p=dataReg[2];
			_p<<=8;
			_p|=dataReg[1];
			_p<<=8;
			_p|=dataReg[0];

			if (_p & 0x800000) { // 24 bit two's complement
				_p -= 0x1000000;
			}

			_t=(int16_t)(((uint16_t)dataReg[4]<<8)|dataReg[3]); // 16 bit two's complement

			_pressure=(float)_p/4096.0f;
			_temp=(float)_t/100.0f;

			_newData=true;
			enter(lps35hw_done_state,nowMs);
		}
		break;
	case lps35hw_err_state:
		if(errorCleared(nowMs,lps35hw_error_backoff_ms)){
			enter(lps35hw_init_state,nowMs);
		}
		break;
	default:
		enter(lps35hw_init_state,nowMs);
		break;
	}
}

template <typename TTransport>
bool lps35hw<TTransport>::phaseExpired(uint32_t nowMs) const{
	return (nowMs-phase_start)>lps35hw_phase_timeout_ms;
}

// ctrl_reg2 is built fresh for every write, so a one-off bit such as
// swreset or one_shot cannot be left set and sent again later.
template <typename TTransport>
uint8_t lps35hw<TTransport>::ctrlReg2(bool swreset, bool one_shot){
	_ctrl_reg2.reg=0;
	_ctrl_reg2.bit.if_add_inc=true; // the 5 byte measurement read relies on it
	// The chip's I2C interface is switched off when it is on SPI,
	// which is what lps35hw_spi used to do.
	_ctrl_reg2.bit.i2c_dis=std::is_base_of<SPITransport, TTransport>::value;
	_ctrl_reg2.bit.swreset=swreset;
	_ctrl_reg2.bit.one_shot=one_shot;
	return _ctrl_reg2.reg;
}

template <typename TTransport>
void lps35hw<TTransport>::startMeasurement(){
	_force_measurement=true;
}

template <typename TTransport>
void lps35hw<TTransport>::tempPressure(float *t,float *p){
	*t=_temp;
	*p=_pressure;
	_newData=false;
}

template <typename TTransport>
bool lps35hw<TTransport>::newData(){
	return _newData;
}

} /* namespace LPS35HW */

#endif /* LPS35HW_TPP_ */
