/*
 * hmc6352.h
 *
 *  Created on: Jul 16, 2026
 *      Author: coryg
 */

#ifndef HMC6352_H_
#define HMC6352_H_

#include "stm32l4xx_hal.h"
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

namespace HMC6352 {

typedef enum hmc6352_state_t{
	hmc6352_init_state,
	hmc6352_reset_state,
	hmc6352_done_state,
	hmc6352_sleep_state,
	hmc6352_error_state
}hmc6352_state_t;

typedef enum hmc6352_command_t{
	hmc6352_write_eeprom = 0x77,
	hmc6352_read_eeprom = 0x72,
	hmc6352_write_ram = 0x47,
	hmc6352_read_ram = 0x67,
	hmc6352_enter_sleep = 0x53,
	hmc6352_exit_sleep = 0x57,
	hmc6352_upd_bridge_offsets = 0x4F,
	hmc6352_enter_user_cal_mode = 0x43,
	hmc6352_exit_user_cal_mode = 0x45,
	hmc6352_save_op_mode = 0x4C,
	hmc6352_get_data = 0x41
}hmc6352_command_t;

typedef enum hmc6352_address_t{
	hmc6352_address = 0x00,
	hmc6352_magnetometer_x_offset_msb = 0x01,
	hmc6352_magnetometer_x_offset_lsb = 0x02,
	hmc6352_magnetometer_y_offset_msb = 0x03,
	hmc6352_magnetometer_y_offset_lsb = 0x04,
	hmc6352_time_delay = 0x05,
	hmc6352_number_measurements = 0x06,
	hmc6352_operational_mode_ee = 0x08,
	hmc6352_operational_mode_ram = 0x74,
	hmc6352_version = 0x07,
	hmc6352_output_mode = 0x4E
}hmc6352_address_t;

typedef enum hmc6352_measurement_rate_t{
	hmc6352_1hz = 0b00,
	hmc6352_5hz = 0b01,
	hmc6352_10hz = 0b10,
	hmc6352_20hz = 0b11
}hmc6352_measurement_rate_t;

typedef enum hmc6352_op_mode_t{
	hmc6352_standby = 0b00,
	hmc6352_query = 0b01,
	hmc6352_continuous = 0b10
}hmc6352_mode_t;

typedef enum hmc6352_output_data_mode{
	hmc6352_heading_mode = 0b000,
	hmc6352_raw_magnetometer_x_mode = 0b001,
	hmc6352_raw_magnetometer_y_mode = 0b010,
	hmc6352_magnetometer_x_mode = 0b011,
	hmc6352_magnetometer_y_mode = 0b100
}hmc6352_output_data_mode;

// periodic set/reset enable/disable
typedef enum hmc6352_per_mode_t{
	hmc6352_per_en = 0b1,
	hmc6352_per_dis = 0b0
}hmc6352_sr_mode_t;

typedef struct hmc6352_param_t{
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	hmc6352_mode_t _op_mode;
	hmc6352_measurement_rate_t rate;
	hmc6352_per_mode_t per;
	hmc6352_output_data_mode output_mode;
}hmc6352_param_t;

class hmc6352 {
public:
	hmc6352(hmc6352_param_t param);
	virtual ~hmc6352();
	void main();
	void startMeasurement();
private:
	hmc6352_state_t _state;
	hmc6352_param_t _param;
	uint32_t last_update;
	bool _start_measurement;

	typedef union {
		struct {
			uint8_t op_mode : 2;
			uint8_t : 2;
			uint8_t per_sr : 1;
			uint8_t measurement_rate : 2;

		}bit;
		uint8_t reg;
	}hmc6352_op_mode_t;
	hmc6352_op_mode_t _op_mode;

	typedef union{
		struct{
			uint8_t mode : 3;
			uint8_t : 5;
		}bit;
		uint8_t reg;
	}hmc6352_output_data_mode_reg_t;
	hmc6352_output_data_mode_reg_t _output_data;

	bool writeOpMode();

	bool readRAM(hmc6352_address_t address,uint8_t *byte);
	bool writeRAM(hmc6352_address_t address, uint8_t byte);
	bool readEE(hmc6352_address_t address,uint8_t *byte);
	bool writeEE(hmc6352_address_t address, uint8_t byte);

	bool _enterSleep();
	bool _exitSleep();
	bool _updateBridgeOffsets();
	bool _enterUserCal();
	bool _exitUserCal();
	bool _saveOpModetoEeprom();
	bool _getData();

	bool _startMeasurement();

	//bool _readReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t *reg_value);
	//bool _writeReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t reg_value);
};

} /* namespace HMC6352 */

#endif /* HMC6352_H_ */
