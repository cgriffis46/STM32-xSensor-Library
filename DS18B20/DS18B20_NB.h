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

namespace DS18B20_NB {

typedef struct DS18B20_param_t{
	UART_HandleTypeDef *huart1;
}DS18B20_param_t;

enum DS18B20_state_t{
	DS18B20_init_state,
	DS18B20_reset_state,
	DS18B20_presence_state,
	DS18B20_read_temp_state,
	DS18B20_done_state,
	DS18B20_err_state
};

static void ds18b20_UART_SampleTemp_RxCpltCallback(UART_HandleTypeDef *huart);
static void ds18b20_UART_SampleTemp_TxCpltCallback(UART_HandleTypeDef *huart);

enum cb_state_t{
	sample_temp_init,
	sample_temp_presence,
	sample_temp_write_1,
	sample_temp_write_2,
	read_temp_init,
	read_temp_write_1,
	read_temp_write_2,
	read_temp_read_LSB,
	read_temp_read_MSB,
	read_temp_read_lsb_bit,
	read_temp_read_msb_bit
};

cb_state_t cb_state=sample_temp_init;

class ds18b20 {
public:
	ds18b20(DS18B20_param_t param);
	virtual ~ds18b20();
	void main();
	bool newData();
	void getTemp(float *temp);
	bool presence = false;

	DS18B20_param_t _param;
	DS18B20_state_t _state;

protected:
	bool _newData;

	uint32_t last_update;
	uint8_t DS18B20_Init(void);
	uint8_t DS18B20_ReadBit(void);
	uint8_t DS18B20_ReadByte(void);
	//void DS18B20_WriteByte(uint8_t);
	void DS18B20_SampleTemp(void);
	float DS18B20_ReadTemp(void);
	float _temp;

};

static ds18b20 *ds18b20_selfptr;

} /* namespace DS18B20 */

#endif /* DS18B20_H_ */
