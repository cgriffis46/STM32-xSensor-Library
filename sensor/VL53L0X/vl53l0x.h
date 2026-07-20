/*
 * vl53l0x.h
 *
 *  Created on: Jul 15, 2026
 *      Author: coryg
 */

#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "stm32l4xx_hal.h"
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

namespace VL53L0X {

typedef enum vl53l0x_state_t{
	vl53l0x_init_state,
	vl53l0x_reset_state,
	vl53l0x_done_state,
	vl53l0x_sleep_state,
	vl53l0x_error_state
}vl53l0x_state_t;

typedef struct vl53l0x_param_t{
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
}vl53l0x_param_t;

class vl53l0x {
public:
	vl53l0x(vl53l0x_param_t param);
	virtual ~vl53l0x();
	void main();
private:
	vl53l0x_param_t _param;
	vl53l0x_state_t _state;
	uint32_t last_update;

	//bool _readReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t *reg_value);
	//bool _writeReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t reg_value);
};

} /* namespace VL53L0X */

#endif /* VL53L0X_H_ */
