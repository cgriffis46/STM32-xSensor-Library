/*
 * vl53l0x.cpp
 *
 *  Created on: Jul 15, 2026
 *      Author: coryg
 */

#include <vl53l0x.h>

namespace VL53L0X {

vl53l0x::vl53l0x(vl53l0x_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(vl53l0x_param_t));
	_state=vl53l0x_init_state;
	last_update=HAL_GetTick();
}

vl53l0x::~vl53l0x() {
	// TODO Auto-generated destructor stub
}

void vl53l0x::main(){
switch(_state){

case vl53l0x_init_state :
	break;
case vl53l0x_reset_state :
	break;
case vl53l0x_done_state :
	break;
case vl53l0x_sleep_state :
	break;
case vl53l0x_error_state :
	break;
default:
	break;
}

}

} /* namespace VL53L0X */
