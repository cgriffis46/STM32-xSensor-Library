/*
 * DS18B20.tpp
 *
 *  DS18B20's member definitions. DS18B20.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include DS18B20.h, not this file.
 */

#ifndef DS18B20_TPP_
#define DS18B20_TPP_

#include "../inc/DS18B20.h"

namespace DS18B20 {

template <typename TTransport>
template <typename... TArgs>
ds18b20<TTransport>::ds18b20(TArgs&&... transportArgs)
	: base(DS18B20_init_state, DS18B20_err_state, DS18B20_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_newData=false;
	_temp=NAN;
	cycle_start=0;
	for(uint8_t i=0;i<DS18B20_scratchpad_len;i++){
		scratchpad[i]=0;
	}
}

template <typename TTransport>
void ds18b20<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case DS18B20_init_state:
		cycle_start=nowMs;
		enter(DS18B20_done_state,nowMs);
		break;
	case DS18B20_done_state:
		if((nowMs-cycle_start)>=DS18B20_period_ms){
			cycle_start=nowMs;
			enter(DS18B20_reset_state,nowMs);
		} else {
			sleepRemaining(cycle_start,nowMs,DS18B20_period_ms);
		}
		break;
	case DS18B20_reset_state:
		issued(this->reset(),DS18B20_presence_state,nowMs);
		break;
	case DS18B20_presence_state:
		if(landed(nowMs)){
			if(this->presence()){
				enter(DS18B20_convert_state,nowMs);
			} else {
				fail(nowMs); // No presence pulse detected
			}
		}
		break;
	case DS18B20_convert_state:
		{
			// The transport takes its own copy, so a local is fine here.
			const uint8_t cmd[2]={DS18B20_cmd_skip_rom,DS18B20_cmd_convert_t};
			issued(this->writeBytes(cmd,2),DS18B20_wait_convert_state,nowMs);
		}
		break;
	case DS18B20_wait_convert_state:
		if(landed(nowMs)){
			enter(DS18B20_converting_state,nowMs);
		}
		break;
	case DS18B20_converting_state:
		// Read any sooner and the scratchpad still holds the previous
		// conversion, or 85 C straight after power up.
		if(elapsed(nowMs,DS18B20_conversion_ms)){
			enter(DS18B20_read_reset_state,nowMs);
		} else {
			sleepRemaining(nowMs,DS18B20_conversion_ms);
		}
		break;
	case DS18B20_read_reset_state:
		issued(this->reset(),DS18B20_read_presence_state,nowMs);
		break;
	case DS18B20_read_presence_state:
		if(landed(nowMs)){
			if(this->presence()){
				enter(DS18B20_read_cmd_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case DS18B20_read_cmd_state:
		{
			const uint8_t cmd[2]={DS18B20_cmd_skip_rom,DS18B20_cmd_read_scratchpad};
			issued(this->writeBytes(cmd,2),DS18B20_wait_read_cmd_state,nowMs);
		}
		break;
	case DS18B20_wait_read_cmd_state:
		if(landed(nowMs)){
			enter(DS18B20_read_temp_state,nowMs);
		}
		break;
	case DS18B20_read_temp_state:
		issued(this->readBytes(scratchpad,DS18B20_scratchpad_len),DS18B20_wait_read_temp_state,nowMs);
		break;
	case DS18B20_wait_read_temp_state:
		if(landed(nowMs)){
			if(scratchpadValid()){
				_temp=scratchpadTemp();
				_newData=true;
				enter(DS18B20_done_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case DS18B20_err_state:
		if(errorCleared(nowMs,DS18B20_error_backoff_ms)){
			enter(DS18B20_init_state,nowMs);
		}
		break;
	default: // if we don't know what state we're in, re-init
		enter(DS18B20_init_state,nowMs);
		break;
	}
}

// The CRC catches a corrupted read and a device that has gone away
// (the line then reads all 1s). All 0s has to be caught separately:
// the CRC of nine zero bytes is itself zero.
template <typename TTransport>
bool ds18b20<TTransport>::scratchpadValid() const{
	bool allZero=true;
	for(uint8_t i=0;i<DS18B20_scratchpad_len;i++){
		if(scratchpad[i]!=0){
			allZero=false;
		}
	}
	if(allZero){
		return false;
	}
	return crc8(scratchpad,DS18B20_scratchpad_crc)==scratchpad[DS18B20_scratchpad_crc];
}

template <typename TTransport>
float ds18b20<TTransport>::scratchpadTemp() const{
	// Signed: temperatures below zero are two's complement.
	int16_t Temp = (int16_t)(((uint16_t)scratchpad[DS18B20_scratchpad_temp_msb]<<8)|scratchpad[DS18B20_scratchpad_temp_lsb]);

	// Below 12 bit resolution the lowest bits are undefined.
	switch((scratchpad[DS18B20_scratchpad_config]>>5)&0x03){
	case 0: // 9 bit
		Temp &= ~0x07;
		break;
	case 1: // 10 bit
		Temp &= ~0x03;
		break;
	case 2: // 11 bit
		Temp &= ~0x01;
		break;
	default: // 12 bit
		break;
	}
	return (float)Temp/16.0f;
}

// Dallas/Maxim CRC-8, x^8 + x^5 + x^4 + 1, as the DS18B20 puts in
// the last byte of its scratchpad.
template <typename TTransport>
uint8_t ds18b20<TTransport>::crc8(const uint8_t *data, uint8_t len){
	uint8_t crc=0;
	for(uint8_t i=0;i<len;i++){
		uint8_t inbyte=data[i];
		for(uint8_t j=0;j<8;j++){
			uint8_t mix=(crc^inbyte)&0x01;
			crc>>=1;
			if(mix){
				crc^=0x8C;
			}
			inbyte>>=1;
		}
	}
	return crc;
}

template <typename TTransport>
bool ds18b20<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void ds18b20<TTransport>::getTemp(float *temp){
	*temp=_temp;
	_newData=false;
}

} /* namespace DS18B20 */

#endif /* DS18B20_TPP_ */
