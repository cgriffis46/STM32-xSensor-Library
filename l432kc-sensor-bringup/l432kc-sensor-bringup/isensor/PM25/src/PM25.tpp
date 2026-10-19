/*
 * PM25.tpp
 *
 *  PM25's member definitions. PM25.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include PM25.h, not this file.
 */

#ifndef PM25_TPP_
#define PM25_TPP_

#include "../inc/PM25.h"

template <typename TTransport>
template <typename... TArgs>
PM25<TTransport>::PM25(TArgs&&... transportArgs)
	: base(pm25_init_state, pm25_error_state, 0, std::forward<TArgs>(transportArgs)...) {
	_newdata=false;
	_havedata=false;
	_frameReady=false;
	rxCount=0;
	memset(&data,0,sizeof(data));
	memset(rxBuffer,0,sizeof(rxBuffer));
	memset(frame,0,sizeof(frame));
}

template <typename TTransport>
void PM25<TTransport>::main(uint32_t nowMs){
	switch(this->_state){
	case pm25_init_state:
		// From here on the transport calls onByteReceived(). This also
		// starts its receiver if it is not already running.
		this->setRxSink(*this);
		enter(pm25_listening_state,nowMs);
		break;
	case pm25_listening_state:
		if(_frameReady){
			unpack();
			enter(pm25_listening_state,nowMs); // the silence timer starts again from this frame
		} else if(elapsed(nowMs,pm25_silence_ms)){
			fail(nowMs);
		} else {
			sleepRemaining(nowMs,pm25_silence_ms);
		}
		break;
	case pm25_error_state:
		if(_frameReady){ // the sensor is back
			enter(pm25_listening_state,nowMs);
		} else if(elapsed(nowMs,pm25_retry_ms)){
			this->setRxSink(*this); // in case it is the receiver that has stopped, not the sensor
			enter(pm25_error_state,nowMs);
		} else {
			sleepRemaining(nowMs,pm25_retry_ms);
		}
		break;
	default:
		enter(pm25_init_state,nowMs);
		break;
	}
}

// Interrupt context. One byte at a time: wait for 0x42, then 0x4d,
// then collect the rest. Anything that is not where a start byte
// should be is skipped, which is what lines the receiver up again
// after starting part way through a frame or losing a byte.
template <typename TTransport>
void PM25<TTransport>::onByteReceived(uint8_t byte){
	if(rxCount==0){
		if(byte==pm25_start_1){
			rxBuffer[0]=byte;
			rxCount=1;
		}
		return;
	}
	if(rxCount==1){
		if(byte==pm25_start_2){
			rxBuffer[1]=byte;
			rxCount=2;
		} else if(byte!=pm25_start_1){ // a second 0x42 may itself be the real start
			rxCount=0;
		}
		return;
	}

	rxBuffer[rxCount]=byte;
	rxCount++;

	if(rxCount==4){
		// The length field. If it is wrong this was not a real start:
		// go back to looking now, not 28 bytes later.
		uint16_t len=(uint16_t)((rxBuffer[2]<<8)|rxBuffer[3]);
		if(len!=pm25_frame_len){
			rxCount=0;
		}
		return;
	}

	if(rxCount==pm25_frame_size){
		rxCount=0;
		if(checksumOk(rxBuffer)&&!_frameReady){ // if main() has not taken the last one yet, this one is dropped
			memcpy(frame,rxBuffer,pm25_frame_size);
			_frameReady=true;
			wake();
		}
	}
}

// The last two bytes are the sum of the 30 before them.
template <typename TTransport>
bool PM25<TTransport>::checksumOk(const uint8_t *f){
	  uint16_t csum = 0;
	  for (uint8_t i = 0; i < 30; i++) {
	        csum += f[i];
	      }
	  uint16_t sent = (uint16_t)((f[30] << 8) | f[31]);
	  return csum==sent;
}

// main() side. Only called with _frameReady true, so the interrupt
// is leaving frame[] alone.
template <typename TTransport>
void PM25<TTransport>::unpack(){
	  // The data comes in endian'd, this solves it so it works on all platforms
	      uint16_t buffer_u16[15];
	      for (uint8_t i = 0; i < 15; i++) {
	        buffer_u16[i] = frame[2 + i * 2 + 1];
	        buffer_u16[i] += (frame[2 + i * 2] << 8);
	      }
	      // put it into a nice struct :)
	      memcpy((void *)&data, (void *)buffer_u16, sizeof(data));
	      _newdata=true;
	      _havedata=true;
	      _frameReady=false; // the interrupt may use frame[] again
}

template <typename TTransport>
bool PM25<TTransport>::getData(PM25_AQI_Data *_data){
		memcpy((void*)_data,&data, sizeof(data));
		_newdata=false;
		return _havedata;
}

template <typename TTransport>
bool PM25<TTransport>::newData(){
	return _newdata;
}

#endif /* PM25_TPP_ */
