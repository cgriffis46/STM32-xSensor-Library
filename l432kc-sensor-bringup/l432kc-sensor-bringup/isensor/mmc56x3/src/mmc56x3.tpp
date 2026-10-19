/*
 * mmc56x3.tpp
 *
 *  mmc56x3's member definitions. mmc56x3.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include mmc56x3.h, not this file.
 */

#ifndef MMC56X3_TPP_
#define MMC56X3_TPP_

#include "../inc/mmc56x3.h"

namespace MMC56X3 {

template <typename TTransport>
template <typename... TArgs>
mmc56x3<TTransport>::mmc56x3(const mmc56x3_param_t &param, TArgs&&... transportArgs)
	: base(mmc56x3_init_state, mmc56x3_error_state, mmc56x3_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	phase_start=0;
	_start_measurement=false;
	_newData=false;
	_x=NAN; _y=NAN; _z=NAN; _temp=NAN;
	_product_id=0;
	_status=0;
}

template <typename TTransport>
void mmc56x3<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case mmc56x3_init_state:
		_start_measurement=false;
		if(this->checkDevice()){
			enter(mmc56x3_product_id_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;
	case mmc56x3_product_id_state:
		issued(this->readRegs(mmc56x3_product_id,&_product_id,1),mmc56x3_wait_product_id_state,nowMs);
		break;
	case mmc56x3_wait_product_id_state:
		if(landed(nowMs)){
			if(_product_id==MMC56X3_PRODUCT_ID){
				enter(mmc56x3_sw_reset_state,nowMs);
			} else {
				fail(nowMs); // something else is at this address
			}
		}
		break;
	case mmc56x3_sw_reset_state:
		issued(this->writeReg(mmc56x3_ctrl1,MMC56X3_CTRL1_SW_RESET),mmc56x3_wait_sw_reset_state,nowMs);
		break;
	case mmc56x3_wait_sw_reset_state:
		if(landed(nowMs)){
			enter(mmc56x3_reset_settle_state,nowMs);
		}
		break;
	case mmc56x3_reset_settle_state:
		settled(nowMs,mmc56x3_reset_settle_ms,mmc56x3_set_state);
		break;
	// A large current through the set coil and then the reset coil
	// clears any offset the sensor has picked up.
	case mmc56x3_set_state:
		issued(this->writeReg(mmc56x3_ctrl0,MMC56X3_CTRL0_DO_SET),mmc56x3_wait_set_state,nowMs);
		break;
	case mmc56x3_wait_set_state:
		if(landed(nowMs)){
			enter(mmc56x3_set_settle_state,nowMs);
		}
		break;
	case mmc56x3_set_settle_state:
		settled(nowMs,mmc56x3_set_reset_ms,mmc56x3_reset_state);
		break;
	case mmc56x3_reset_state:
		issued(this->writeReg(mmc56x3_ctrl0,MMC56X3_CTRL0_DO_RESET),mmc56x3_wait_reset_state,nowMs);
		break;
	case mmc56x3_wait_reset_state:
		if(landed(nowMs)){
			enter(mmc56x3_reset_pulse_settle_state,nowMs);
		}
		break;
	case mmc56x3_reset_pulse_settle_state:
		settled(nowMs,mmc56x3_set_reset_ms,mmc56x3_bandwidth_state);
		break;
	case mmc56x3_bandwidth_state:
		issued(this->writeReg(mmc56x3_ctrl1,(uint8_t)(_param.bandwidth&0x03)),mmc56x3_wait_bandwidth_state,nowMs);
		break;
	case mmc56x3_wait_bandwidth_state:
		if(landed(nowMs)){
			enter(_param.mode==mmc56x3_continuous?mmc56x3_odr_state:mmc56x3_done_state,nowMs);
		}
		break;
	// Continuous mode, in the order the datasheet gives: the data rate,
	// then cmm_freq_en so the chip works out its measurement period
	// from it, then cmm_en to start.
	case mmc56x3_odr_state:
		issued(this->writeReg(mmc56x3_odr,odrValue()),mmc56x3_wait_odr_state,nowMs);
		break;
	case mmc56x3_wait_odr_state:
		if(landed(nowMs)){
			enter(mmc56x3_cmm_freq_state,nowMs);
		}
		break;
	case mmc56x3_cmm_freq_state:
		issued(this->writeReg(mmc56x3_ctrl0,ctrl0Value(MMC56X3_CTRL0_CMM_FREQ_EN)),mmc56x3_wait_cmm_freq_state,nowMs);
		break;
	case mmc56x3_wait_cmm_freq_state:
		if(landed(nowMs)){
			enter(mmc56x3_cmm_en_state,nowMs);
		}
		break;
	case mmc56x3_cmm_en_state:
		{
			// 1000 a second needs the high power mode, with the data rate register at 255.
			const uint8_t ctrl2=MMC56X3_CTRL2_CMM_EN|(_param.odr>255?MMC56X3_CTRL2_HPOWER:0);
			issued(this->writeReg(mmc56x3_ctrl2,ctrl2),mmc56x3_wait_cmm_en_state,nowMs);
		}
		break;
	case mmc56x3_wait_cmm_en_state:
		if(landed(nowMs)){
			enter(mmc56x3_done_state,nowMs);
		}
		break;
	case mmc56x3_done_state:
		if(elapsed(nowMs,_param.period_ms)||_start_measurement){
			_start_measurement=false;
			phase_start=nowMs;
			if(_param.mode==mmc56x3_continuous){ // the chip is already measuring. read the latest
				enter(mmc56x3_read_field_state,nowMs);
			} else {
				enter(mmc56x3_trigger_m_state,nowMs);
			}
		} else {
			// No longer than mmc56x3_request_poll_ms at a time, so a
			// startMeasurement() request is seen without waiting out
			// the whole period.
			const uint32_t left=_param.period_ms-(nowMs-last_update);
			sleep(left<mmc56x3_request_poll_ms?left:mmc56x3_request_poll_ms);
		}
		break;

	// One-shot field measurement.
	case mmc56x3_trigger_m_state:
		issued(this->writeReg(mmc56x3_ctrl0,ctrl0Value(MMC56X3_CTRL0_TAKE_MEAS_M)),mmc56x3_wait_trigger_m_state,nowMs);
		break;
	case mmc56x3_wait_trigger_m_state:
		if(landed(nowMs)){
			enter(mmc56x3_measuring_m_state,nowMs);
		}
		break;
	case mmc56x3_measuring_m_state:
		settled(nowMs,conversionMs(),mmc56x3_status_m_state);
		break;
	case mmc56x3_status_m_state:
		issued(this->readRegs(mmc56x3_status1,&_status,1),mmc56x3_wait_status_m_state,nowMs);
		break;
	case mmc56x3_wait_status_m_state:
		if(landed(nowMs)){
			if(_status&MMC56X3_STATUS1_MEAS_M_DONE){
				enter(mmc56x3_read_field_state,nowMs);
			} else if(phaseExpired(nowMs)){
				fail(nowMs); // the measurement never finished
			} else { // still measuring. ask again
				sleep(1);
				enter(mmc56x3_status_m_state,nowMs);
			}
		}
		break;
	case mmc56x3_read_field_state:
		issued(this->readRegs(mmc56x3_xout0,dataReg,9),mmc56x3_wait_read_field_state,nowMs);
		break;
	case mmc56x3_wait_read_field_state:
		if(landed(nowMs)){
			// 20 bits an axis: bits 19:12 in xout0, 11:4 in xout1 and
			// 3:0 in the top of xout2, for x, y and z in turn. Zero
			// field reads as the middle of the range.
			uint32_t raw[3];
			for(int i=0;i<3;++i){
				raw[i]=((uint32_t)dataReg[2*i]<<12)|((uint32_t)dataReg[2*i+1]<<4)|((uint32_t)dataReg[6+i]>>4);
			}
			_x=(float)((int32_t)raw[0]-(1<<19))*mmc56x3_ut_per_lsb;
			_y=(float)((int32_t)raw[1]-(1<<19))*mmc56x3_ut_per_lsb;
			_z=(float)((int32_t)raw[2]-(1<<19))*mmc56x3_ut_per_lsb;
			if(_param.mode==mmc56x3_one_shot&&_param.read_temperature){
				enter(mmc56x3_trigger_t_state,nowMs);
			} else {
				_newData=true;
				enter(mmc56x3_done_state,nowMs);
			}
		}
		break;

	// One-shot temperature measurement, after the field.
	case mmc56x3_trigger_t_state:
		issued(this->writeReg(mmc56x3_ctrl0,ctrl0Value(MMC56X3_CTRL0_TAKE_MEAS_T)),mmc56x3_wait_trigger_t_state,nowMs);
		break;
	case mmc56x3_wait_trigger_t_state:
		if(landed(nowMs)){
			enter(mmc56x3_measuring_t_state,nowMs);
		}
		break;
	case mmc56x3_measuring_t_state:
		settled(nowMs,mmc56x3_temp_conversion_ms,mmc56x3_status_t_state);
		break;
	case mmc56x3_status_t_state:
		issued(this->readRegs(mmc56x3_status1,&_status,1),mmc56x3_wait_status_t_state,nowMs);
		break;
	case mmc56x3_wait_status_t_state:
		if(landed(nowMs)){
			if(_status&MMC56X3_STATUS1_MEAS_T_DONE){
				enter(mmc56x3_read_temp_state,nowMs);
			} else if(phaseExpired(nowMs)){
				fail(nowMs);
			} else {
				sleep(1);
				enter(mmc56x3_status_t_state,nowMs);
			}
		}
		break;
	case mmc56x3_read_temp_state:
		issued(this->readRegs(mmc56x3_tout,dataReg,1),mmc56x3_wait_read_temp_state,nowMs);
		break;
	case mmc56x3_wait_read_temp_state:
		if(landed(nowMs)){
			_temp=(float)dataReg[0]*mmc56x3_c_per_lsb+mmc56x3_c_offset;
			_newData=true;
			enter(mmc56x3_done_state,nowMs);
		}
		break;

	case mmc56x3_error_state:
		if(errorCleared(nowMs,mmc56x3_error_backoff_ms)){
			enter(mmc56x3_init_state,nowMs);
		}
		break;
	default:
		enter(mmc56x3_init_state,nowMs);
		break;
	}
}

// Moves on to next once ms have passed since the current state was
// entered, and sleeps for what is left until then.
template <typename TTransport>
bool mmc56x3<TTransport>::settled(uint32_t nowMs, uint32_t ms, mmc56x3_state_t next){
	if(elapsed(nowMs,ms)){
		enter(next,nowMs);
		return true;
	}
	sleepRemaining(nowMs,ms);
	return false;
}

template <typename TTransport>
bool mmc56x3<TTransport>::phaseExpired(uint32_t nowMs) const{
	return (nowMs-phase_start)>mmc56x3_phase_timeout_ms;
}

// How long one field measurement takes at the chosen bandwidth,
// rounded up. Status is polled after this.
template <typename TTransport>
uint32_t mmc56x3<TTransport>::conversionMs() const{
	switch(_param.bandwidth){
	case mmc56x3_bw_3_5ms: return 4;
	case mmc56x3_bw_2_0ms: return 2;
	case mmc56x3_bw_1_2ms: return 2;
	default:               return 7;
	}
}

// The data rate register takes 1 to 255 measurements a second. 1000
// a second is 255 here with hpower set in internal control 2.
template <typename TTransport>
uint8_t mmc56x3<TTransport>::odrValue() const{
	if(_param.odr>255) return 255;
	if(_param.odr==0) return 1;
	return (uint8_t)_param.odr;
}

// Internal control 0 is write-only and each write acts on every bit,
// so it is built fresh each time: the command, plus auto set/reset if
// it is wanted.
template <typename TTransport>
uint8_t mmc56x3<TTransport>::ctrl0Value(uint8_t command) const{
	return (uint8_t)(command|(_param.auto_sr?MMC56X3_CTRL0_AUTO_SR_EN:0));
}

template <typename TTransport>
void mmc56x3<TTransport>::startMeasurement(){
	_start_measurement=true;
}

template <typename TTransport>
bool mmc56x3<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void mmc56x3<TTransport>::getField(float *x, float *y, float *z){
	*x=_x;
	*y=_y;
	*z=_z;
	_newData=false;
}

template <typename TTransport>
void mmc56x3<TTransport>::getTemperature(float *t){
	*t=_temp;
}

} /* namespace MMC56X3 */

#endif /* MMC56X3_TPP_ */
