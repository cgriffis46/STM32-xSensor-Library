/*
 * hmc6352.tpp
 *
 *  hmc6352's member definitions. hmc6352.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include hmc6352.h, not this file.
 */

#ifndef HMC6352_TPP_
#define HMC6352_TPP_

#include "../inc/hmc6352.h"

namespace HMC6352 {

template <typename TTransport>
template <typename... TArgs>
hmc6352<TTransport>::hmc6352(const hmc6352_param_t &param, TArgs&&... transportArgs)
	: base(hmc6352_init_state, hmc6352_error_state, hmc6352_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	_start_measurement=false;
	_newData=false;
	_heading=NAN;
	_raw=0;
	_op_mode.reg=0;
	_output_data.reg=0;
}

template <typename TTransport>
void hmc6352<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case hmc6352_init_state:
		_start_measurement=false;
		if(this->checkDevice()){
			enter(hmc6352_op_mode_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;
	case hmc6352_op_mode_state:
		_op_mode.reg=0;
		_op_mode.bit.op_mode=_param._op_mode;
		_op_mode.bit.per_sr=_param.per;
		_op_mode.bit.measurement_rate=_param.rate;
		issued(writeRAM(hmc6352_operational_mode_ram,_op_mode.reg),hmc6352_wait_op_mode_state,nowMs);
		break;
	case hmc6352_wait_op_mode_state:
		if(landed(nowMs)){
			enter(hmc6352_output_mode_state,nowMs);
		}
		break;
	case hmc6352_output_mode_state:
		if(!elapsed(nowMs,hmc6352_command_gap_ms)){ // give the chip time to act on the last command
			sleepRemaining(nowMs,hmc6352_command_gap_ms);
		} else {
			_output_data.reg=0;
			_output_data.bit.mode=_param.output_mode;
			issued(writeRAM(hmc6352_output_mode,_output_data.reg),hmc6352_wait_output_mode_state,nowMs);
		}
		break;
	case hmc6352_wait_output_mode_state:
		if(landed(nowMs)){
			enter(hmc6352_done_state,nowMs);
		}
		break;
	case hmc6352_done_state:
		if(elapsed(nowMs,hmc6352_period_ms)||_start_measurement){
			_start_measurement=false;
			switch(_op_mode.bit.op_mode){
				case hmc6352_continuous: // the chip is already measuring. read the latest
					enter(hmc6352_read_data_state,nowMs);
					break;
				default: // standby and query: ask for a measurement
					enter(hmc6352_get_data_state,nowMs);
					break;
			}
		} else {
			// No longer than hmc6352_request_poll_ms at a time, so a
			// startMeasurement() request is seen without waiting out
			// the whole period.
			uint32_t left=hmc6352_period_ms-(nowMs-last_update);
			sleep(left<hmc6352_request_poll_ms?left:hmc6352_request_poll_ms);
		}
		break;
	case hmc6352_get_data_state:
		{
			const uint8_t cmd=hmc6352_get_data;
			issued(this->writeBytes(&cmd,1),hmc6352_wait_get_data_state,nowMs);
		}
		break;
	case hmc6352_wait_get_data_state:
		if(landed(nowMs)){
			enter(hmc6352_measuring_state,nowMs);
		}
		break;
	case hmc6352_measuring_state:
		if(elapsed(nowMs,hmc6352_conversion_ms)){
			enter(hmc6352_read_data_state,nowMs);
		} else {
			sleepRemaining(nowMs,hmc6352_conversion_ms);
		}
		break;
	case hmc6352_read_data_state:
		issued(this->readBytes(dataReg,2),hmc6352_wait_read_data_state,nowMs);
		break;
	case hmc6352_wait_read_data_state:
		if(landed(nowMs)){
			// most significant byte first
			_raw=(int16_t)(((uint16_t)dataReg[0]<<8)|dataReg[1]);
			if(_output_data.bit.mode==hmc6352_heading_mode){
				if(_raw<0||_raw>3599){ // a heading is 0 to 3599 tenths of a degree
					fail(nowMs);
					break;
				}
				_heading=(float)_raw/10.0f;
			} else {
				_heading=NAN;
			}
			_newData=true;
			enter(hmc6352_done_state,nowMs);
		}
		break;
	case hmc6352_error_state:
		if(errorCleared(nowMs,hmc6352_error_backoff_ms)){
			enter(hmc6352_init_state,nowMs);
		}
		break;
	default :
		enter(hmc6352_init_state,nowMs);
		break;
	}
}

// Write one byte to the chip's RAM: the command, the address, the value.
template <typename TTransport>
bool hmc6352<TTransport>::writeRAM(hmc6352_address_t address, uint8_t byte){
	const uint8_t buf[3]={hmc6352_write_ram,(uint8_t)address,byte};
	return this->writeBytes(buf,3); // the transport takes its own copy
}

template <typename TTransport>
void hmc6352<TTransport>::startMeasurement(){
	_start_measurement = true;
}

template <typename TTransport>
bool hmc6352<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void hmc6352<TTransport>::getHeading(float *heading){
	*heading=_heading;
	_newData=false;
}

template <typename TTransport>
void hmc6352<TTransport>::getRaw(int16_t *raw){
	*raw=_raw;
	_newData=false;
}

} /* namespace HMC6352 */

#endif /* HMC6352_TPP_ */
