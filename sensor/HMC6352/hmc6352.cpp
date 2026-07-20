/*
 * hmc6352.cpp
 *
 *  Created on: Jul 16, 2026
 *      Author: coryg
 */

#include <hmc6352.h>

namespace HMC6352 {

hmc6352::hmc6352(hmc6352_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(hmc6352_param_t));
	_state=hmc6352_init_state;
	last_update=HAL_GetTick();
	_start_measurement=false;
}

hmc6352::~hmc6352() {
	// TODO Auto-generated destructor stub
}

void hmc6352::main(){
	if(last_update>HAL_GetTick()) {last_update=HAL_GetTick();} // prevent problems caused by rollovers
	switch(_state){
	case hmc6352_init_state:
		break;
	case hmc6352_reset_state:
		break;
	case hmc6352_done_state:
		switch(_op_mode.bit.op_mode){
			case hmc6352_standby:
				if((HAL_GetTick()>(last_update+1000))||_start_measurement){

				}
				break;
			case hmc6352_query:
				if((HAL_GetTick()>(last_update+1000))||_start_measurement){
					_getData();
				}
				break;
			case hmc6352_continuous: // should never get here.
				_state=hmc6352_sleep_state;
				break;
			default:
				break;
		}
		break;
	case hmc6352_sleep_state:
		if((HAL_GetTick()>(last_update+1000))){

		}
		break;
	case hmc6352_error_state:
		if(HAL_GetTick()>(last_update+100)){
			_state=hmc6352_init_state;
		}
		break;
	default :
		break;
	}

}

bool hmc6352::writeOpMode(){
	return false;
}

bool hmc6352::readRAM(hmc6352_address_t address, uint8_t *byte){
	return false;
}

bool hmc6352::writeRAM(hmc6352_address_t address, uint8_t byte){
	return false;
}

bool hmc6352::readEE(hmc6352_address_t address, uint8_t *byte){
	return false;
}

bool hmc6352::writeEE(hmc6352_address_t address, uint8_t byte){
	return false;
}

bool hmc6352::_enterSleep(){
	return false;
}
bool hmc6352::_exitSleep(){
	return false;
}
bool hmc6352::_updateBridgeOffsets(){
	return false;
}
bool hmc6352::_enterUserCal(){
	return false;
}
bool hmc6352::_exitUserCal(){
	return false;
}
bool hmc6352::_saveOpModetoEeprom(){
	return false;
}
bool hmc6352::_getData(){
	return false;
}

void hmc6352::startMeasurement(){
	_start_measurement = true;
}

bool hmc6352::_startMeasurement(){

	return false;
}

} /* namespace HMC6352 */
