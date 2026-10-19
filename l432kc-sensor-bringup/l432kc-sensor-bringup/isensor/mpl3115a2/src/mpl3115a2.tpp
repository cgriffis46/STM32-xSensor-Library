/*
 * mpl3115a2.tpp
 *
 *  mpl3115a2's member definitions. mpl3115a2.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include mpl3115a2.h, not this file.
 */

#ifndef MPL3115A2_TPP_
#define MPL3115A2_TPP_

#include "../inc/mpl3115a2.h"

namespace mpl3115a2 {

template <typename TTransport>
template <typename... TArgs>
mpl3115a2<TTransport>::mpl3115a2(const mpl3115a2_param_t &param, TArgs&&... transportArgs)
	: base(mpl3115a2_init_state, mpl3115a2_error_state, mpl3115a2_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	_new_data=false;
	_temperature=NAN;
	_pressure=NAN;
	_altitude=NAN;
	_EnableIRQ=false;
	_irq_changed=false;
	_who_am_i=0;
	phase_start=0;
	ctrl_reg1.reg=0;
	ctrl_reg4.reg=0;
	ctrl_reg5.reg=0;
	dr_status.reg=0;
	pt_data_cfg.reg=0;
}

template <typename TTransport>
void mpl3115a2<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case mpl3115a2_init_state:
		if(elapsed(nowMs,mpl3115a2_startup_ms)){
			if(this->checkDevice()){
				enter(mpl3115a2_whoami_state,nowMs);
			} else {
				fail(nowMs);
			}
		} else {
			sleepRemaining(nowMs,mpl3115a2_startup_ms);
		}
		break;
	case mpl3115a2_whoami_state:
		issued(this->readRegs(mpl3115a2_who_am_i_addr,&_who_am_i,1),mpl3115a2_wait_whoami_state,nowMs);
		break;
	case mpl3115a2_wait_whoami_state:
		if(landed(nowMs)){
			if(_who_am_i==mpl3115a2_who_am_i){
				enter(mpl3115a2_reset_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case mpl3115a2_reset_state:
		ctrl_reg1.reg=0;
		ctrl_reg1.bit.RST=1;
		issued(this->writeReg(mpl3115a2_ctrl_reg1_addr,ctrl_reg1.reg),mpl3115a2_wait_reset_state,nowMs);
		break;
	case mpl3115a2_wait_reset_state:
		// The chip can reset before it acknowledges this write, so an
		// error here is not a fault. Whether the reset happened is
		// settled by the RST polls that follow.
		if(finished(nowMs)){
			phase_start=nowMs;
			enter(mpl3115a2_reset_settle_state,nowMs);
		}
		break;
	case mpl3115a2_reset_settle_state:
		if(!elapsed(nowMs,mpl3115a2_reset_poll_ms)){
			sleepRemaining(nowMs,mpl3115a2_reset_poll_ms);
		} else if(phaseExpired(nowMs)){
			fail(nowMs); // RST never cleared
		} else {
			enter(mpl3115a2_read_reset_state,nowMs);
		}
		break;
	case mpl3115a2_read_reset_state:
		issued(this->readRegs(mpl3115a2_ctrl_reg1_addr,&ctrl_reg1.reg,1),mpl3115a2_wait_read_reset_state,nowMs);
		break;
	case mpl3115a2_wait_read_reset_state:
		// The chip does not answer while it is resetting, so a failed
		// read means the same as RST still being set: ask again.
		if(finished(nowMs)){
			if(this->lastOpFailed()||ctrl_reg1.bit.RST==1){
				enter(mpl3115a2_reset_settle_state,nowMs);
			} else {
				enter(mpl3115a2_initializing_state,nowMs);
			}
		}
		break;
	case mpl3115a2_initializing_state:
		ctrl_reg1.reg=0;
		ctrl_reg1.bit.ALT=_param.mode;
		ctrl_reg1.bit.OS=_param.oversample_ratio;
		issued(this->writeReg(mpl3115a2_ctrl_reg1_addr,ctrl_reg1.reg),mpl3115a2_wait_ctrl_reg1_state,nowMs);
		break;
	case mpl3115a2_wait_ctrl_reg1_state:
		if(landed(nowMs)){
			enter(mpl3115a2_pt_data_cfg_state,nowMs);
		}
		break;
	case mpl3115a2_pt_data_cfg_state:
		pt_data_cfg.reg=0;
		pt_data_cfg.bit.DREM=1;
		pt_data_cfg.bit.PDEFE=1;
		pt_data_cfg.bit.TDEFE=1;
		issued(this->writeReg(mpl3115a2_pt_data_cfg_addr,pt_data_cfg.reg),mpl3115a2_wait_pt_data_cfg_state,nowMs);
		break;
	case mpl3115a2_wait_pt_data_cfg_state:
		if(landed(nowMs)){
			if(_EnableIRQ){
				enter(mpl3115a2_ctrl_reg5_state,nowMs);
			} else {
				enter(mpl3115a2_done_state,nowMs);
			}
		}
		break;
	case mpl3115a2_ctrl_reg5_state:
		// Routing first, so an interrupt is never enabled on the wrong pin.
		_irq_changed=false;
		issued(this->writeReg(mpl3115a2_ctrl_reg5_addr,ctrl_reg5.reg),mpl3115a2_wait_ctrl_reg5_state,nowMs);
		break;
	case mpl3115a2_wait_ctrl_reg5_state:
		if(landed(nowMs)){
			enter(mpl3115a2_ctrl_reg4_state,nowMs);
		}
		break;
	case mpl3115a2_ctrl_reg4_state:
		issued(this->writeReg(mpl3115a2_ctrl_reg4_addr,ctrl_reg4.reg),mpl3115a2_wait_ctrl_reg4_state,nowMs);
		break;
	case mpl3115a2_wait_ctrl_reg4_state:
		if(landed(nowMs)){
			enter(mpl3115a2_done_state,nowMs);
		}
		break;
	case mpl3115a2_done_state:
		if(_irq_changed){ // SetIRQ() was called while running. The wait starts again afterwards
			enter(mpl3115a2_ctrl_reg5_state,nowMs);
		} else if(elapsed(nowMs,mpl3115a2_period_ms)){
			phase_start=nowMs;
			enter(mpl3115a2_initiate_one_shot_measurement,nowMs);
		} else {
			sleepRemaining(nowMs,mpl3115a2_period_ms);
		}
		break;
	case mpl3115a2_initiate_one_shot_measurement:
		issued(this->readRegs(mpl3115a2_ctrl_reg1_addr,&ctrl_reg1.reg,1),mpl3115a2_wait_ost_state,nowMs);
		break;
	case mpl3115a2_wait_ost_state:
		if(landed(nowMs)){
			if(ctrl_reg1.bit.OST==1){ // a measurement is already running
				enter(mpl3115a2_wait_one_shot_measurement,nowMs);
			} else {
				enter(mpl3115a2_write_ost_state,nowMs);
			}
		}
		break;
	case mpl3115a2_write_ost_state:
		ctrl_reg1.bit.OST=1;
		issued(this->writeReg(mpl3115a2_ctrl_reg1_addr,ctrl_reg1.reg),mpl3115a2_wait_write_ost_state,nowMs);
		break;
	case mpl3115a2_wait_write_ost_state:
		if(landed(nowMs)){
			enter(mpl3115a2_wait_one_shot_measurement,nowMs);
		}
		break;
	case mpl3115a2_wait_one_shot_measurement:
		if(!elapsed(nowMs,mpl3115a2_conversion_poll_ms)){
			sleepRemaining(nowMs,mpl3115a2_conversion_poll_ms);
		} else if(phaseExpired(nowMs)){
			fail(nowMs); // the conversion never finished
		} else {
			enter(mpl3115a2_status_state,nowMs);
		}
		break;
	case mpl3115a2_status_state:
		issued(this->readRegs(mpl3115a2_status_addr,&dr_status.reg,1),mpl3115a2_wait_status_state,nowMs);
		break;
	case mpl3115a2_wait_status_state:
		if(landed(nowMs)){
			if(dr_status.bit.PTDR!=0){
				enter(mpl3115a2_conversion_state,nowMs);
			} else {
				enter(mpl3115a2_wait_one_shot_measurement,nowMs);
			}
		}
		break;
	case mpl3115a2_conversion_state:
		issued(this->readRegs(mpl3115a2_out_p_msb_addr,dataReg,5),mpl3115a2_wait_conversion_state,nowMs);
		break;
	case mpl3115a2_wait_conversion_state:
		if(landed(nowMs)){
			_getConversion();
			_new_data=true;
			enter(mpl3115a2_done_state,nowMs);
		}
		break;
	case mpl3115a2_error_state:
		if(errorCleared(nowMs,mpl3115a2_error_backoff_ms)){
			enter(mpl3115a2_init_state,nowMs);
		}
		break;
	default:
		enter(mpl3115a2_init_state,nowMs);
		break;
	}
}

template <typename TTransport>
bool mpl3115a2<TTransport>::phaseExpired(uint32_t nowMs) const{
	return (nowMs-phase_start)>mpl3115a2_phase_timeout_ms;
}

// The same three bytes hold pressure in barometer mode and altitude
// in altimeter mode, so only one of the two is ever valid.
template <typename TTransport>
void mpl3115a2<TTransport>::_getConversion(){
	int16_t t;

	if(_param.mode==mpl3115a2_altimeter_mode){
		// signed, metres, in 1/65536ths once left justified to 32 bits
		int32_t alt;
		alt = (int32_t)(uint32_t(dataReg[0]) << 24 | uint32_t(dataReg[1]) << 16 |
			  uint32_t(dataReg[2]) << 8);
		_altitude=float(alt) / 65536.0f;
		_pressure=NAN;
	} else {
		uint32_t pressure;
		pressure=0;
		pressure|= (uint32_t(dataReg[0])<<16);
		pressure|= (uint32_t(dataReg[1])<<8);
		pressure|= (uint32_t(dataReg[2]));
		_pressure=(float(pressure) / 6400.0f);
		_altitude=NAN;
	}

	t = (int16_t)((uint16_t(dataReg[3]) << 8) | (uint16_t(dataReg[4])));
	_temperature=(float(t)/256.0f);
}

template <typename TTransport>
bool mpl3115a2<TTransport>::newData(){
	return _new_data;
}

template <typename TTransport>
float mpl3115a2<TTransport>::getTemp(){
	_new_data=false;
	return _temperature;
}

template <typename TTransport>
float mpl3115a2<TTransport>::getPressure(){
	_new_data=false;
	return _pressure;
}

template <typename TTransport>
float mpl3115a2<TTransport>::getAltitude(){
	_new_data=false;
	return _altitude;
}

template <typename TTransport>
void mpl3115a2<TTransport>::SetIRQ(irq_en_param_t irq_en, irq_cfg_param_t irq_cfg){
	ctrl_reg4.reg=irq_en.reg;
	ctrl_reg5.reg=irq_cfg.reg;
	_EnableIRQ=true;
	_irq_changed=true;
}

} /* namespace mpl3115a2 */

#endif /* MPL3115A2_TPP_ */
