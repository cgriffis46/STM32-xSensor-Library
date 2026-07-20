/*
 * Si7021.h
 *
 *  Created on: Feb 15, 2025
 *      Author: coryg
 */

#ifndef SI7021_H_
#define SI7021_H_

#include <stm32l4xx_hal.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <ranges>

#define SI7021_I2C_ADDR 0x40

#define Si7021_reg1_mask 0b10000101
#define Si7021_reg1_mask_rsv 0b01111010
#define Si7021_htr_mask 0b00001111
#define Si7021_htr_mask_rsv 0b11110000
#define Si7021_Measurement_Time 100

typedef uint8_t Si7021_reg_t;
typedef bool Si7021_heater_state_t;
typedef uint8_t Si7021_heater_current_t;

typedef enum Si7021_Cmd_t {
	Si7021_Cmd_Measure_RH_Hold = 0xE5,
	Si7021_Cmd_Measure_RH_No_Hold = 0xF5,
	Si7021_Cmd_Measure_T_Hold = 0xE3,
	Si7021_Cmd_Measure_T_No_Hold = 0xF3,
	Si7021_Cmd_Read_T_From_RH = 0xE0,
	Si7021_Cmd_Reset = 0xFE,
	Si7021_Cmd_Write_RHT_User_Register_1 = 0xE6,
	Si7021_Cmd_Read_RHT_User_Register_1 = 0xE7,
	Si7021_Cmd_Write_Heater_Control_Reg = 0x51,
	Si7021_Cmd_Read_Heater_Control_Reg = 0x11
}Si7021_Cmd_t;

typedef enum Si7021_measure_res{
	Si7021_RH12_T14 = 0b00000000,
	Si7021_RH8_T12 = 0b00000001,
	Si7021_RH10_T13 = 0b10000000,
	Si7021_RH11_T11 = 0b10000000
}Si7021_measure_res;

typedef enum Si7021_heater_enable_t{
	Si7021_heater_enable = 0b00000100,
	Si7021_heater_disable = 0b00000000
}Si7021_heater_enable_t;

typedef enum Si7021_state_t{
	Si7021_init_state,
	Si7021_reset_state,
	Si7021_initializing,
	Si7021_start_temp_measurement,
	//Si7021_temp_measuring,
	Si7021_start_humidity_measurement,
	//Si7021_humidity_measuring,
	Si7021_read_temp_measurement,
	Si7021_read_humidity_measurement,
	Si7021_read_temp_measurement_RH,
	Si7021_done_state,
	Si7021_err_state
}Si7021_state_t;

typedef struct Si7021_param_t{
	I2C_HandleTypeDef *hi2c;
	uint16_t _DevAddress;
	Si7021_heater_enable_t enable;
	Si7021_heater_current_t heater_current;
	Si7021_measure_res res;
}Si7021_param_t;



class Si7021 {
public:
	Si7021(Si7021_param_t param);
	virtual ~Si7021();
	void readTempHumidity(float *t, float *h);
	void main();
	void startTempMeasurement();
	void startHumidityMeasurement();
	bool checkDevice();
	bool newData();
	void setHeater(Si7021_heater_state_t htr_enable,Si7021_heater_current_t htr_current);
private:
	// Si7021 data fields
	I2C_HandleTypeDef *_hi2c; // i2c bus
	uint16_t _DevAddress; // i2c address
	Si7021_state_t _state; // device state
	Si7021_heater_enable_t _enable_heater; // heater enable setting
	Si7021_heater_current_t _heater_current; // heater current setting
	Si7021_measure_res _res; // measurement resolution
	Si7021_reg_t _reg1; // user register 1
	Si7021_reg_t _htr; // heater register
	uint32_t last_update;
	bool _new_data; // new data available
	float temp,humidity; // temperature and humidity
	bool _updateTemp,_updateHumidity; // flags to force a measurement
	// Si7021 Methods
	bool readTemp(float *t);
	bool readHumidity(float *h);
	void reset();
	void _startTempMeasurement();
	void _startHumidityMeasurement();
	bool _readTempAfterRH(float *t);
	void _read_reg1();
	void _write_reg1(Si7021_heater_enable_t enable,Si7021_measure_res res);
	void _read_heater_reg();
	void _write_heater_reg(Si7021_heater_current_t heater_current);
};

#endif /* SI7021_H_ */
