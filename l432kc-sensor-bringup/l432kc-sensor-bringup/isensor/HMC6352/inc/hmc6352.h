/*
 * hmc6352.h
 *
 *  Created on: Jul 16, 2026
 *      Author: coryg
 *
 *  HMC6352 compass, non-blocking state machine.
 *
 *  The bus is chosen by the template argument. hmc6352<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      hmc6352<Stm32HalI2CTransport>  sensor(param, &hi2c1, HMC6352_I2C_ADDR, i2c1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  The HMC6352 takes one letter commands, some followed by an address
 *  and a value, and is read back as a plain run of bytes, so this
 *  driver uses the transport's writeBytes() and readBytes().
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each command and each read is one state that issues it and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xhmc6352.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 *
 *  What it does: writes the operational mode and the output mode to
 *  the chip's RAM, then takes a reading every hmc6352_period_ms, and
 *  at once when startMeasurement() asks.
 *    standby and query mode   sends the get data command, waits out
 *                             the conversion, reads the two bytes
 *    continuous mode          the chip measures by itself; reads the
 *                             two bytes
 *
 *  What it does not do: sleep and wake, the EEPROM, user calibration
 *  and the bridge offset update. The commands are in
 *  hmc6352_command_t for when they are wanted.
 *
 *  The original of this file had the states and the settings but no
 *  bus code. The command sequences and timings below are from the
 *  Honeywell datasheet and have not been run against a chip.
 */

#ifndef HMC6352_H_
#define HMC6352_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace HMC6352 {

#define HMC6352_I2C_ADDR 0x21

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum hmc6352_state_t{
	hmc6352_init_state,					// check a device answers
	hmc6352_op_mode_state,				// issue the operational mode write
	hmc6352_wait_op_mode_state,
	hmc6352_output_mode_state,			// issue the output mode write
	hmc6352_wait_output_mode_state,
	hmc6352_done_state,					// between readings
	hmc6352_get_data_state,				// issue the get data command
	hmc6352_wait_get_data_state,
	hmc6352_measuring_state,			// conversion time
	hmc6352_read_data_state,			// issue the 2 byte read
	hmc6352_wait_read_data_state,
	hmc6352_error_state					// pause, then start again from hmc6352_init_state
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

// All times in ms.
const uint32_t hmc6352_bus_timeout_ms = 100;	// one bus operation, to issue or to land
const uint32_t hmc6352_command_gap_ms = 1;		// the chip needs up to 125 us after a command before the next
const uint32_t hmc6352_conversion_ms = 10;		// get data takes 6 ms
const uint32_t hmc6352_period_ms = 1000;		// between readings
const uint32_t hmc6352_request_poll_ms = 100;	// longest sleep while a startMeasurement() could arrive
const uint32_t hmc6352_error_backoff_ms = 100;	// in hmc6352_error_state before starting again

// The bus handle and device address are no longer here. They belong
// to the transport and go to its constructor.
typedef struct hmc6352_param_t{
	hmc6352_mode_t _op_mode;
	hmc6352_measurement_rate_t rate;	// continuous mode only
	hmc6352_per_mode_t per;
	hmc6352_output_data_mode output_mode;
}hmc6352_param_t;

template <typename TTransport>
class hmc6352 : public SensorStateMachine<TTransport, hmc6352_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"hmc6352<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, hmc6352_state_t> base;
protected:
	// Names from a template base class have to be brought in before
	// they can be used unqualified.
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::issued;
	using base::landed;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit hmc6352(const hmc6352_param_t &param, TArgs&&... transportArgs);
	virtual ~hmc6352() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a reading now
	bool newData();
	void getHeading(float *heading);	// degrees, 0 to 359.9, in heading mode. NAN otherwise, and until the first reading
	void getRaw(int16_t *raw);			// the two bytes as the chip sent them: tenths of a degree, or magnetometer counts
private:
	hmc6352_param_t _param;
	bool _start_measurement;
	bool _newData;
	float _heading;
	int16_t _raw;

	typedef union {
		struct {
			uint8_t op_mode : 2;
			uint8_t : 2;
			uint8_t per_sr : 1;
			uint8_t measurement_rate : 2;
			uint8_t : 1;
		}bit;
		uint8_t reg;
	}op_mode_reg_t;
	op_mode_reg_t _op_mode;
	typedef union{
		struct{
			uint8_t mode : 3;
			uint8_t : 5;
		}bit;
		uint8_t reg;
	}output_data_mode_reg_t;
	output_data_mode_reg_t _output_data;

	// The transport fills this in after main() has returned, so it
	// has to be a member and not a local.
	uint8_t dataReg[2];

	bool writeRAM(hmc6352_address_t address, uint8_t byte);
};

} /* namespace HMC6352 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/hmc6352.tpp"

#endif /* HMC6352_H_ */
