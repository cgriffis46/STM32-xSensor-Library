/*
 * DS18B20.h
 *
 *  Created on: Feb 1, 2026
 *      Author: coryg
 */

#ifndef DS18B20_H_
#define DS18B20_H_

#include <stm32l4xx_hal.h>
#include "stm32l4xx_ll_usart.h"
#include <cstdint>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <cstring>
#include <stdio.h>

namespace DS18B20 {

typedef struct DS18B20_param_t{
	UART_HandleTypeDef *huart1;
}DS18B20_param_t;

enum DS18B20_state_t{
	DS18B20_init_state,
	DS18B20_read_temp_state,
	DS18B20_done_state,
	DS18B20_err_state
};

class ds18b20 {
public:
	ds18b20(DS18B20_param_t param);
	virtual ~ds18b20();
	void main();
	bool newData();
	void getTemp(float *temp);
protected:
	bool _newData;
	uint32_t last_update;
	uint8_t DS18B20_Init(void);
	uint8_t DS18B20_ReadBit(void);
	uint8_t DS18B20_ReadByte(void);
	void DS18B20_WriteByte(uint8_t);
	void DS18B20_SampleTemp(void);
	float DS18B20_ReadTemp(void);
	float _temp;
	DS18B20_param_t _param;
	DS18B20_state_t _state;
};

} /* namespace DS18B20 */

#endif /* DS18B20_H_ */
