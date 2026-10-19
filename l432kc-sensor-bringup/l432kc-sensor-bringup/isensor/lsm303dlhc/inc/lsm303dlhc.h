/*
 * lsm303dlhc.h
 *
 *  Created on: Apr 19, 2024
 *      Author: coryg
 *
 *  ST LSM303DLHC accelerometer and magnetometer, non-blocking state
 *  machines.
 *
 *  The two halves of the chip are separate devices on the I2C bus,
 *  at their own addresses, so each has its own driver and its own
 *  transport:
 *
 *      lsm303dlhc_accel<Stm32HalI2CTransport>  accel(aparam, &hi2c1, LSM303DLHC_ACCEL_ADDR, i2c1Mutex);
 *      lsm303dlhc_mag<Stm32HalI2CTransport>    mag(mparam, &hi2c1, LSM303DLHC_MAG_ADDR, i2c1Mutex);
 *
 *  The two can share the bus and its mutex, and run from the same
 *  loop or thread. Each inherits from its TTransport, which must be
 *  an ISensorTransport (itransport/inc/ISensorTransport.h); everything
 *  after the param goes to the transport's own constructor. The chip
 *  is I2C only. This file includes no HAL and no RTOS header.
 *
 *  Call main(now) on each every pass of the loop. It never waits on
 *  the bus: each register access is one state that issues it and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xlsm303dlhc.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 *
 *  Accelerometer: writes the data rate with all three axes on, reads
 *  it back to check the chip took it (the accelerometer has no ID
 *  register), then sets block data update, the full scale and high
 *  resolution (12 bit). Every param.period_ms, or at once when
 *  startMeasurement() asks, polls the status register until a new
 *  sample is ready and reads it.
 *
 *  Magnetometer: checks the identification registers ("H43"), writes
 *  the data rate and the gain, and starts continuous conversion. Every
 *  param.period_ms, or at once when startMeasurement() asks, reads the
 *  latest sample. An axis that has overflowed (the chip reports -4096)
 *  is NAN.
 *
 *  What they do not do: the accelerometer's interrupts, click
 *  detection, FIFO, high-pass filter and low power mode; the
 *  magnetometer's temperature sensor, which reads relative to an
 *  unknown offset.
 *
 *  The register map, sensitivities and data rates are from the ST
 *  LSM303DLHC datasheet (DocID018771) and have not yet been run
 *  against a chip.
 */

#ifndef LSM303DLHC_H_
#define LSM303DLHC_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace LSM303DLHC {

#define LSM303DLHC_ACCEL_ADDR 0x19	// 7 bit
#define LSM303DLHC_MAG_ADDR   0x1E	// 7 bit

// ---- accelerometer ----

typedef enum lsm303dlhc_accel_register_t{
	lsm303dlhc_ctrl_reg1_a = 0x20,
	lsm303dlhc_ctrl_reg4_a = 0x23,
	lsm303dlhc_status_reg_a = 0x27,
	lsm303dlhc_out_x_l_a = 0x28		// 0x28 to 0x2D: x, y, z, low byte first
}lsm303dlhc_accel_register_t;

// Set in the register address of a multi-byte read, so the
// accelerometer moves on to the next register after each byte.
#define LSM303DLHC_AUTO_INCREMENT 0x80

#define LSM303DLHC_CTRL_REG1_A_XYZ_EN 0x07
#define LSM303DLHC_CTRL_REG4_A_BDU    0x80	// block data update: the high and low bytes are from the same sample
#define LSM303DLHC_CTRL_REG4_A_HR     0x08	// high resolution, 12 bit
#define LSM303DLHC_STATUS_REG_A_ZYXDA 0x08	// a new x, y and z sample is ready

// CTRL_REG1_A bits 7:4, in normal mode.
typedef enum lsm303dlhc_accel_odr_t{
	lsm303dlhc_accel_1hz = 0x1,
	lsm303dlhc_accel_10hz = 0x2,
	lsm303dlhc_accel_25hz = 0x3,
	lsm303dlhc_accel_50hz = 0x4,
	lsm303dlhc_accel_100hz = 0x5,
	lsm303dlhc_accel_200hz = 0x6,
	lsm303dlhc_accel_400hz = 0x7,
	lsm303dlhc_accel_1344hz = 0x9
}lsm303dlhc_accel_odr_t;

// CTRL_REG4_A bits 5:4.
typedef enum lsm303dlhc_accel_scale_t{
	lsm303dlhc_accel_2g = 0b00,
	lsm303dlhc_accel_4g = 0b01,
	lsm303dlhc_accel_8g = 0b10,
	lsm303dlhc_accel_16g = 0b11
}lsm303dlhc_accel_scale_t;

typedef enum lsm303dlhc_accel_state_t{
	lsm303dlhc_accel_init_state,			// check a device answers
	lsm303dlhc_accel_ctrl1_state,			// issue the CTRL_REG1_A write: data rate, axes on
	lsm303dlhc_accel_wait_ctrl1_state,
	lsm303dlhc_accel_check_state,			// issue the CTRL_REG1_A read back
	lsm303dlhc_accel_wait_check_state,
	lsm303dlhc_accel_ctrl4_state,			// issue the CTRL_REG4_A write: block update, scale, 12 bit
	lsm303dlhc_accel_wait_ctrl4_state,
	lsm303dlhc_accel_done_state,			// between readings
	lsm303dlhc_accel_status_state,			// issue the status read
	lsm303dlhc_accel_wait_status_state,
	lsm303dlhc_accel_read_state,			// issue the 6 byte read
	lsm303dlhc_accel_wait_read_state,
	lsm303dlhc_accel_error_state			// pause, then start again from the init state
}lsm303dlhc_accel_state_t;

typedef struct lsm303dlhc_accel_param_t{
	lsm303dlhc_accel_odr_t odr;			// the chip samples at this rate by itself
	lsm303dlhc_accel_scale_t scale;
	uint32_t period_ms;					// between readings
}lsm303dlhc_accel_param_t;

// ---- magnetometer ----

typedef enum lsm303dlhc_mag_register_t{
	lsm303dlhc_cra_reg_m = 0x00,
	lsm303dlhc_crb_reg_m = 0x01,
	lsm303dlhc_mr_reg_m = 0x02,
	lsm303dlhc_out_x_h_m = 0x03,		// 0x03 to 0x08: x, z, y, high byte first
	lsm303dlhc_ira_reg_m = 0x0A			// 0x0A to 0x0C: 'H', '4', '3'
}lsm303dlhc_mag_register_t;

#define LSM303DLHC_MR_REG_M_CONTINUOUS 0x00
#define LSM303DLHC_MAG_OVERFLOW (-4096)

// CRA_REG_M bits 4:2.
typedef enum lsm303dlhc_mag_rate_t{
	lsm303dlhc_mag_0_75hz = 0,
	lsm303dlhc_mag_1_5hz = 1,
	lsm303dlhc_mag_3hz = 2,
	lsm303dlhc_mag_7_5hz = 3,
	lsm303dlhc_mag_15hz = 4,
	lsm303dlhc_mag_30hz = 5,
	lsm303dlhc_mag_75hz = 6,
	lsm303dlhc_mag_220hz = 7
}lsm303dlhc_mag_rate_t;

// CRB_REG_M bits 7:5. The range in gauss.
typedef enum lsm303dlhc_mag_gain_t{
	lsm303dlhc_mag_1_3g = 1,
	lsm303dlhc_mag_1_9g = 2,
	lsm303dlhc_mag_2_5g = 3,
	lsm303dlhc_mag_4_0g = 4,
	lsm303dlhc_mag_4_7g = 5,
	lsm303dlhc_mag_5_6g = 6,
	lsm303dlhc_mag_8_1g = 7
}lsm303dlhc_mag_gain_t;

typedef enum lsm303dlhc_mag_state_t{
	lsm303dlhc_mag_init_state,				// check a device answers
	lsm303dlhc_mag_id_state,				// issue the identification read
	lsm303dlhc_mag_wait_id_state,
	lsm303dlhc_mag_cra_state,				// issue the CRA_REG_M write: data rate
	lsm303dlhc_mag_wait_cra_state,
	lsm303dlhc_mag_crb_state,				// issue the CRB_REG_M write: gain
	lsm303dlhc_mag_wait_crb_state,
	lsm303dlhc_mag_mr_state,				// issue the MR_REG_M write: continuous conversion
	lsm303dlhc_mag_wait_mr_state,
	lsm303dlhc_mag_done_state,				// between readings
	lsm303dlhc_mag_read_state,				// issue the 6 byte read
	lsm303dlhc_mag_wait_read_state,
	lsm303dlhc_mag_error_state				// pause, then start again from the init state
}lsm303dlhc_mag_state_t;

typedef struct lsm303dlhc_mag_param_t{
	lsm303dlhc_mag_rate_t rate;			// the chip converts at this rate by itself
	lsm303dlhc_mag_gain_t gain;
	uint32_t period_ms;					// between readings
}lsm303dlhc_mag_param_t;

// All times in ms.
const uint32_t lsm303dlhc_bus_timeout_ms = 100;		// one bus operation, to issue or to land
const uint32_t lsm303dlhc_sample_timeout_ms = 2000;	// for a new accelerometer sample: twice the slowest rate
const uint32_t lsm303dlhc_request_poll_ms = 100;	// longest sleep while a startMeasurement() could arrive
const uint32_t lsm303dlhc_error_backoff_ms = 100;	// in an error state before starting again

const float lsm303dlhc_gravity = 9.80665f;			// m/s^2 in 1 g
const float lsm303dlhc_ut_per_gauss = 100.0f;

template <typename TTransport>
class lsm303dlhc_accel : public SensorStateMachine<TTransport, lsm303dlhc_accel_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"lsm303dlhc_accel<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, lsm303dlhc_accel_state_t> base;
protected:
	// Names from a template base class have to be brought in before
	// they can be used unqualified.
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::issued;
	using base::landed;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit lsm303dlhc_accel(const lsm303dlhc_accel_param_t &param, TArgs&&... transportArgs);
	virtual ~lsm303dlhc_accel() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a reading now
	bool newData();
	void getAccel(float *x, float *y, float *z);	// m/s^2. NAN until the first reading, and after a failure

protected:
	void onFail() override { _x=NAN; _y=NAN; _z=NAN; }

private:
	lsm303dlhc_accel_param_t _param;
	uint32_t phase_start;	// when the current reading began
	bool _start_measurement;
	bool _newData;
	float _x, _y, _z;

	// The transport fills these in after main() has returned, so they
	// have to be members and not locals.
	uint8_t _check;
	uint8_t _status;
	uint8_t dataReg[6];

	uint8_t ctrl1Value() const;
	float mgPerLsb() const;
};

template <typename TTransport>
class lsm303dlhc_mag : public SensorStateMachine<TTransport, lsm303dlhc_mag_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"lsm303dlhc_mag<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, lsm303dlhc_mag_state_t> base;
protected:
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::issued;
	using base::landed;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit lsm303dlhc_mag(const lsm303dlhc_mag_param_t &param, TArgs&&... transportArgs);
	virtual ~lsm303dlhc_mag() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a reading now
	bool newData();
	void getMag(float *x, float *y, float *z);	// uT. NAN until the first reading, after a failure, and on an axis that overflowed

protected:
	void onFail() override { _x=NAN; _y=NAN; _z=NAN; }

private:
	lsm303dlhc_mag_param_t _param;
	bool _start_measurement;
	bool _newData;
	float _x, _y, _z;

	// The transport fills these in after main() has returned, so they
	// have to be members and not locals.
	uint8_t _id[3];
	uint8_t dataReg[6];

	float toMicroTesla(int16_t raw, float lsbPerGauss) const;
	float lsbPerGaussXY() const;
	float lsbPerGaussZ() const;
};

} /* namespace LSM303DLHC */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/lsm303dlhc.tpp"

#endif /* LSM303DLHC_H_ */
