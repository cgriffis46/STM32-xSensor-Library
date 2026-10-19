/*
 * Si7021.h
 *
 *  Created on: Feb 15, 2025
 *      Author: coryg
 *
 *  Si7021 temperature / humidity sensor, non-blocking state machine.
 *
 *  The bus is chosen by the template argument. Si7021<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      Si7021<Stm32HalI2CTransport>  sensor(param, &hi2c1, SI7021_I2C_ADDR, i2c1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  Measurements are commands: one byte starts a conversion and the
 *  result is read back later as a plain run of bytes, so they use the
 *  transport's writeBytes() and readBytes(). The two settings
 *  registers are a command byte followed by a value, which is exactly
 *  a register access, so they use readRegs() and writeReg().
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each command and each read is one state that issues it and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xSi7021.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 */

#ifndef SI7021_H_
#define SI7021_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

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

// measurement resolution: bits 7 and 0 of user register 1
typedef enum Si7021_measure_res{
	Si7021_RH12_T14 = 0b00000000,
	Si7021_RH8_T12 = 0b00000001,
	Si7021_RH10_T13 = 0b10000000,
	Si7021_RH11_T11 = 0b10000001
}Si7021_measure_res;

typedef enum Si7021_heater_enable_t{
	Si7021_heater_enable = 0b00000100,
	Si7021_heater_disable = 0b00000000
}Si7021_heater_enable_t;

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum Si7021_state_t{
	Si7021_init_state,						// check a device answers
	Si7021_send_reset_state,				// issue the reset command
	Si7021_wait_send_reset_state,
	Si7021_reset_state,						// pause while the reset takes effect
	Si7021_initializing,					// issue the user register 1 read
	Si7021_wait_read_reg1,
	Si7021_write_reg1,
	Si7021_wait_write_reg1,
	Si7021_read_heater_reg,
	Si7021_wait_read_heater_reg,
	Si7021_write_heater_reg,
	Si7021_wait_write_heater_reg,
	Si7021_done_state,						// between measurements
	Si7021_start_humidity_measurement,		// issue the humidity measurement command
	Si7021_wait_start_humidity_measurement,
	Si7021_humidity_measuring,				// conversion time
	Si7021_read_humidity_measurement,		// issue the 3 byte read
	Si7021_wait_humidity_measurement,
	Si7021_start_temp_measurement,			// issue the temperature measurement command
	Si7021_wait_start_temp_measurement,
	Si7021_temp_measuring,					// conversion time
	Si7021_read_temp_measurement,			// issue the 3 byte read
	Si7021_wait_temp_measurement,
	Si7021_err_state						// pause, then start again from Si7021_init_state
}Si7021_state_t;

// All times in ms. Si7021_Measurement_Time, above, is the conversion wait.
const uint32_t Si7021_bus_timeout_ms = 100;		// one bus operation, to issue or to land
const uint32_t Si7021_reset_ms = 100;			// after the reset command
const uint32_t Si7021_period_ms = 2000;			// between measurements
const uint32_t Si7021_request_poll_ms = 100;	// longest sleep while a request could arrive
const uint32_t Si7021_error_backoff_ms = 100;	// in Si7021_err_state before starting again

// The bus handle and device address are no longer here. They belong
// to the transport and go to its constructor.
typedef struct Si7021_param_t{
	Si7021_heater_enable_t enable;
	Si7021_heater_current_t heater_current;
	Si7021_measure_res res;
}Si7021_param_t;

template <typename TTransport>
class Si7021 : public SensorStateMachine<TTransport, Si7021_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"Si7021<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, Si7021_state_t> base;
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
	explicit Si7021(const Si7021_param_t &param, TArgs&&... transportArgs);
	virtual ~Si7021() {}

	void readTempHumidity(float *t, float *h);	// degrees C, %RH. NAN until the first reading, and after a failure
	void main(uint32_t nowMs);
	void startTempMeasurement();		// ask for a temperature measurement now
	void startHumidityMeasurement();	// ask for humidity then temperature now
	bool newData();
	void setHeater(Si7021_heater_state_t htr_enable,Si7021_heater_current_t htr_current);	// written before the next measurement
private:
	// Si7021 data fields
	Si7021_heater_enable_t _enable_heater; // heater enable setting
	Si7021_heater_current_t _heater_current; // heater current setting
	Si7021_measure_res _res; // measurement resolution
	Si7021_reg_t _reg1; // user register 1
	Si7021_reg_t _htr; // heater register
	bool _new_data; // new data available
	float temp,humidity; // temperature and humidity
	bool _updateTemp,_updateHumidity; // flags to force a measurement
	bool _updateHeater; // flag to write the heater settings again

	// The transport fills this in after main() has returned, so it
	// has to be a member and not a local.
	uint8_t dataReg[3];

	// A failure leaves no valid reading, as it did when a failed read
	// came back as NAN.
	void onFail() override { temp=NAN; humidity=NAN; }

	// Si7021 Methods
	bool measurementValid() const;
	uint16_t measurementCode() const;
	static uint8_t crc8(const uint8_t *data, int len);
};

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/Si7021.tpp"

#endif /* SI7021_H_ */
