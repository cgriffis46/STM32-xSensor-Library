/*
 * iaqcore.tpp
 *
 *  iaqcore's member definitions. iaqcore.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include iaqcore.h, not this file.
 */

#ifndef IAQCORE_TPP_
#define IAQCORE_TPP_

#include "../inc/iaqcore.h"

namespace IAQCORE {

template <typename TTransport>
template <typename... TArgs>
iaqcore<TTransport>::iaqcore(const iaqcore_param_t &param, TArgs&&... transportArgs)
	: base(iaqcore_init_state, iaqcore_error_state, iaqcore_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_period_ms=param.period_ms;
	if(_period_ms==0){
		_period_ms=(param.variant==iaqcore_pulsed)?iaqcore_pulsed_period_ms:iaqcore_continuous_period_ms;
	}
	_periodStart=0;
	_start_measurement=false;
	_newData=false;
	_statusChanged=false;
	_status=iaqcore_status_none;
	_errorStreak=0;
	_busyCount=0;
	_co2=NAN;
	_tvoc=NAN;
	_resistance=NAN;
	for(uint8_t i=0;i<iaqcore_frame_len;++i){ frame[i]=0; }
}

template <typename TTransport>
void iaqcore<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case iaqcore_init_state:
		// No checkDevice() here: it would address the module with the
		// write bit. The first read finds out whether it is there.
		_start_measurement=false;
		_busyCount=0;
		_periodStart=nowMs;
		enter(iaqcore_read_state,nowMs);
		break;
	case iaqcore_read_state:
		issued(this->readBytes(frame,iaqcore_frame_len),iaqcore_wait_read_state,nowMs);
		break;
	case iaqcore_wait_read_state:
		if(landed(nowMs)){
			uint16_t co2=0, tvoc=0; int32_t res=0; iaqcore_status_t st=iaqcore_status_none;
			if(!decode(frame,&co2,&st,&res,&tvoc)){ // none of the four status values: not an iAQ-Core's frame
				fail(nowMs);
				break;
			}
			if(st==iaqcore_status_busy){
				// The values may be half updated. The last good reading
				// and status stay as they are until a read says otherwise.
				if(_busyCount>=iaqcore_busy_retries){
					fail(nowMs);
				} else {
					++_busyCount;
					enter(iaqcore_busy_retry_state,nowMs);
				}
				break;
			}
			_busyCount=0;
			setStatus(st);
			if(st==iaqcore_status_ok){
				_errorStreak=0;
				_co2=(float)co2;
				_tvoc=(float)tvoc;
				_resistance=(float)res;
				_newData=true;
			} else {
				if(st==iaqcore_status_error){
					if(_errorStreak!=UINT32_MAX){ ++_errorStreak; }
				} else {
					_errorStreak=0; // RUNIN
				}
				invalidate();
			}
			enter(iaqcore_done_state,nowMs);
		}
		break;
	case iaqcore_busy_retry_state:
		if(elapsed(nowMs,iaqcore_busy_retry_ms)){
			enter(iaqcore_read_state,nowMs);
		} else {
			sleepRemaining(nowMs,iaqcore_busy_retry_ms);
		}
		break;
	case iaqcore_done_state:
		if((uint32_t)(nowMs-_periodStart)>=_period_ms||_start_measurement){
			_start_measurement=false;
			nextPeriod(_periodStart,nowMs,_period_ms); // start to start, so a late wake-up doesn't add up
			enter(iaqcore_read_state,nowMs);
		} else {
			// No longer than iaqcore_request_poll_ms at a time, so a
			// startMeasurement() request is seen without waiting out
			// the whole period.
			const uint32_t left=_period_ms-(nowMs-_periodStart);
			sleep(left<iaqcore_request_poll_ms?left:iaqcore_request_poll_ms);
		}
		break;
	case iaqcore_error_state:
		if(errorCleared(nowMs,iaqcore_error_backoff_ms)){
			enter(iaqcore_init_state,nowMs);
		}
		break;
	default :
		enter(iaqcore_init_state,nowMs);
		break;
	}
}

// Bytes 0-1 prediction (CO2 eq. ppm), 2 status, 3-6 resistance (Ohm,
// byte 3 always 0), 7-8 TVOC eq. ppb. Most significant byte first.
template <typename TTransport>
bool iaqcore<TTransport>::decode(const uint8_t *f, uint16_t *co2, iaqcore_status_t *status, int32_t *resistance, uint16_t *tvoc){
	switch(f[2]){
	case iaqcore_status_ok:
	case iaqcore_status_busy:
	case iaqcore_status_runin:
	case iaqcore_status_error:
		break;
	default:
		return false;
	}
	*co2=(uint16_t)(((uint16_t)f[0]<<8)|f[1]);
	*status=(iaqcore_status_t)f[2];
	*resistance=(int32_t)(((uint32_t)f[3]<<24)|((uint32_t)f[4]<<16)|((uint32_t)f[5]<<8)|(uint32_t)f[6]);
	*tvoc=(uint16_t)(((uint16_t)f[7]<<8)|f[8]);
	return true;
}

template <typename TTransport>
void iaqcore<TTransport>::setStatus(iaqcore_status_t s){
	if(s!=_status){
		_status=s;
		_statusChanged=true;
	}
}

template <typename TTransport>
void iaqcore<TTransport>::invalidate(){
	_co2=NAN;
	_tvoc=NAN;
	_resistance=NAN;
	_newData=false;
}

// A read that failed, timed out or made no sense: the module may be
// gone, so nothing known about it stands.
template <typename TTransport>
void iaqcore<TTransport>::onFail(){
	invalidate();
	setStatus(iaqcore_status_none);
	_errorStreak=0;
}

template <typename TTransport>
void iaqcore<TTransport>::startMeasurement(){
	_start_measurement = true;
}

template <typename TTransport>
bool iaqcore<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void iaqcore<TTransport>::getCO2(float *ppm){
	*ppm=_co2;
	_newData=false;
}

template <typename TTransport>
void iaqcore<TTransport>::getTVOC(float *ppb){
	*ppb=_tvoc;
	_newData=false;
}

template <typename TTransport>
void iaqcore<TTransport>::getResistance(float *ohm){
	*ohm=_resistance;
	_newData=false;
}

template <typename TTransport>
bool iaqcore<TTransport>::statusChanged(){
	return _statusChanged;
}

template <typename TTransport>
iaqcore_status_t iaqcore<TTransport>::getStatus(){
	_statusChanged=false;
	return _status;
}

} /* namespace IAQCORE */

#endif /* IAQCORE_TPP_ */
