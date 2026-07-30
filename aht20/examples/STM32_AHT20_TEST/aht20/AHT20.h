/*
 * AHT20.h
 *
 *  Created on: May 31, 2025
 *      Author: coryg
 */

#ifndef AHT20_H_
#define AHT20_H_

#include <stdint.h>
#include <stm32l4xx_hal.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//namespace AHT20 {
extern "C"{

const uint8_t AHT20_I2C_Addr = 0x38;
const uint8_t AHT20_MeasurementCmd = 0xAC;
const uint8_t AHT20_Status_Reg = 0x71;
const uint8_t AHT20_Init_Cmd = 0xBE;
const uint8_t AHT20_Init_High = 0x08;
const uint8_t AHT20_Init_Low = 0x00;
const uint8_t AHT20_MeasurementCmdHigh = 0x33;
const uint8_t AHT20_MeasurementCmdLow = 0x00;
const uint8_t AHT20_SoftResetCmd = 0xBA;

typedef enum AHT20_state_t{
	aht20_init,
	aht_reset,
	aht_reinit,
	aht_sleep,
	aht_measuring,
	aht_error
}AHT20_state_t;


class AHT20 {
public:
	AHT20(I2C_HandleTypeDef *hi2c);
	virtual ~AHT20();
	void StartMeasurement(); //
	void main();
	bool newData();
	void getTempHumidity(float *temp, float *humidity); // returns the values already read from the sensor.
private:
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	uint8_t status_word;
	AHT20_state_t state;
	uint32_t last_update;
	bool _start_measure;
	bool _startmeasurement();
	bool ReadTempHumidity(float *temp, float *humidity);
	bool _newData;
	void softReset();
	bool _Init();
	float _temp,_humidity;
	bool checkDevice();
	bool readStatusReg();
};

//} /* namespace AHT20 */
} // extern C
#endif /* AHT20_H_ */
