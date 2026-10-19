/*
 * mmc56x3.h
 *
 *  Created on: Apr 24, 2024
 *      Author: coryg
 *
 *  MEMSIC MMC5603 / MMC5633 3 axis magnetometer, non-blocking state
 *  machine. Both chips have the same registers and product ID.
 *
 *  The bus is chosen by the template argument. mmc56x3<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      mmc56x3<Stm32HalI2CTransport>  sensor(param, &hi2c1, MMC56X3_I2C_ADDR, i2c1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each register access is one state that issues it and one state
 *  that waits for it to land. When a state has nothing to do it calls
 *  sleep(), an empty stub here. Override sleep() for an OS (see
 *  xmmc56x3.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 *
 *  What it does: checks the product ID, resets the chip, pulses the
 *  set and reset coils to clear any offset, writes the bandwidth and,
 *  for continuous mode, the data rate. Then every param.period_ms, or
 *  at once when startMeasurement() asks:
 *    one-shot mode     triggers a measurement, waits it out, polls the
 *                      status register until it is done, and reads the
 *                      field. Then the same for the temperature, if
 *                      param.read_temperature.
 *    continuous mode   the chip measures by itself at param.odr; reads
 *                      the latest field. The chip does not measure
 *                      temperature in this mode.
 *
 *  The control registers are write-only, so the values written are
 *  built here each time rather than read back and changed.
 *
 *  What it does not do: the self test, the axis inhibit bits and the
 *  periodic set (Prd_set); auto set/reset (param.auto_sr) is the
 *  usual way to keep the offset cleared.
 *
 *  The register map and timings are from the MEMSIC MMC5603NJ
 *  datasheet and have not yet been run against a chip.
 */

#ifndef MMC56X3_H_
#define MMC56X3_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace MMC56X3 {

#define MMC56X3_I2C_ADDR 0x30		// 7 bit
#define MMC56X3_PRODUCT_ID 0x10		// read from the product ID register

typedef enum mmc56x3_register_t{
	mmc56x3_xout0 = 0x00,			// 0x00 to 0x08: the field, see readField
	mmc56x3_tout = 0x09,
	mmc56x3_status1 = 0x18,
	mmc56x3_odr = 0x1A,
	mmc56x3_ctrl0 = 0x1B,			// internal control 0. write-only
	mmc56x3_ctrl1 = 0x1C,			// internal control 1. write-only
	mmc56x3_ctrl2 = 0x1D,			// internal control 2. write-only
	mmc56x3_product_id = 0x39
}mmc56x3_register_t;

// internal control 0
#define MMC56X3_CTRL0_TAKE_MEAS_M  0x01
#define MMC56X3_CTRL0_TAKE_MEAS_T  0x02
#define MMC56X3_CTRL0_DO_SET       0x08
#define MMC56X3_CTRL0_DO_RESET     0x10
#define MMC56X3_CTRL0_AUTO_SR_EN   0x20
#define MMC56X3_CTRL0_CMM_FREQ_EN  0x80
// internal control 1
#define MMC56X3_CTRL1_SW_RESET     0x80
// internal control 2
#define MMC56X3_CTRL2_CMM_EN       0x10
#define MMC56X3_CTRL2_HPOWER       0x80
// status 1
#define MMC56X3_STATUS1_MEAS_M_DONE 0x40
#define MMC56X3_STATUS1_MEAS_T_DONE 0x80

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum mmc56x3_state_t{
	mmc56x3_init_state,					// check a device answers
	mmc56x3_product_id_state,			// issue the product ID read
	mmc56x3_wait_product_id_state,
	mmc56x3_sw_reset_state,				// issue the software reset
	mmc56x3_wait_sw_reset_state,
	mmc56x3_reset_settle_state,			// the chip restarts
	mmc56x3_set_state,					// issue the set coil pulse
	mmc56x3_wait_set_state,
	mmc56x3_set_settle_state,
	mmc56x3_reset_state,				// issue the reset coil pulse
	mmc56x3_wait_reset_state,
	mmc56x3_reset_pulse_settle_state,
	mmc56x3_bandwidth_state,			// issue the internal control 1 write
	mmc56x3_wait_bandwidth_state,
	mmc56x3_odr_state,					// continuous mode: issue the data rate write
	mmc56x3_wait_odr_state,
	mmc56x3_cmm_freq_state,				// continuous mode: internal control 0, to work out the period
	mmc56x3_wait_cmm_freq_state,
	mmc56x3_cmm_en_state,				// continuous mode: internal control 2, to start
	mmc56x3_wait_cmm_en_state,
	mmc56x3_done_state,					// between readings
	mmc56x3_trigger_m_state,			// one-shot: issue take_meas_m
	mmc56x3_wait_trigger_m_state,
	mmc56x3_measuring_m_state,			// conversion time
	mmc56x3_status_m_state,				// issue the status read
	mmc56x3_wait_status_m_state,
	mmc56x3_read_field_state,			// issue the 9 byte field read
	mmc56x3_wait_read_field_state,
	mmc56x3_trigger_t_state,			// one-shot: issue take_meas_t
	mmc56x3_wait_trigger_t_state,
	mmc56x3_measuring_t_state,
	mmc56x3_status_t_state,
	mmc56x3_wait_status_t_state,
	mmc56x3_read_temp_state,			// issue the temperature read
	mmc56x3_wait_read_temp_state,
	mmc56x3_error_state					// pause, then start again from mmc56x3_init_state
}mmc56x3_state_t;

// Internal control 1 bits 1:0. A narrower bandwidth is quieter and
// slower to measure.
typedef enum mmc56x3_bandwidth_t{
	mmc56x3_bw_6_6ms = 0b00,			// measurement time 6.6 ms
	mmc56x3_bw_3_5ms = 0b01,
	mmc56x3_bw_2_0ms = 0b10,
	mmc56x3_bw_1_2ms = 0b11
}mmc56x3_bandwidth_t;

typedef enum mmc56x3_mode_t{
	mmc56x3_one_shot,					// a measurement each period, triggered here
	mmc56x3_continuous					// the chip measures at param.odr by itself
}mmc56x3_mode_t;

// All times in ms.
const uint32_t mmc56x3_bus_timeout_ms = 100;	// one bus operation, to issue or to land
const uint32_t mmc56x3_reset_settle_ms = 20;	// after the software reset
const uint32_t mmc56x3_set_reset_ms = 1;		// after each set or reset coil pulse
const uint32_t mmc56x3_temp_conversion_ms = 2;	// before first checking a temperature
const uint32_t mmc56x3_phase_timeout_ms = 100;	// one whole measurement, polls included
const uint32_t mmc56x3_request_poll_ms = 100;	// longest sleep while a startMeasurement() could arrive
const uint32_t mmc56x3_error_backoff_ms = 100;	// in mmc56x3_error_state before starting again

const float mmc56x3_ut_per_lsb = 0.00625f;		// 20 bit field, 0.0625 mG per count
const float mmc56x3_c_per_lsb = 0.8f;			// temperature
const float mmc56x3_c_offset = -75.0f;			// temperature at a reading of 0

// The bus handle and device address belong to the transport and go
// to its constructor.
typedef struct mmc56x3_param_t{
	mmc56x3_mode_t mode;
	mmc56x3_bandwidth_t bandwidth;
	uint16_t odr;					// continuous mode: measurements a second, 1 to 255, or 1000
	bool auto_sr;					// set/reset the coils automatically before each measurement
	bool read_temperature;			// one-shot mode: measure the temperature too
	uint32_t period_ms;				// between readings
}mmc56x3_param_t;

template <typename TTransport>
class mmc56x3 : public SensorStateMachine<TTransport, mmc56x3_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"mmc56x3<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, mmc56x3_state_t> base;
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
	explicit mmc56x3(const mmc56x3_param_t &param, TArgs&&... transportArgs);
	virtual ~mmc56x3() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a reading now
	bool newData();
	void getField(float *x, float *y, float *z);	// uT. NAN until the first reading, and after a failure
	void getTemperature(float *t);					// degrees C. NAN unless one-shot mode with read_temperature

protected:
	void onFail() override { _x=NAN; _y=NAN; _z=NAN; _temp=NAN; }

private:
	mmc56x3_param_t _param;
	uint32_t phase_start;	// when the current measurement began
	bool _start_measurement;
	bool _newData;
	float _x, _y, _z, _temp;

	// The transport fills these in after main() has returned, so they
	// have to be members and not locals.
	uint8_t _product_id;
	uint8_t _status;
	uint8_t dataReg[9];

	bool phaseExpired(uint32_t nowMs) const;
	uint32_t conversionMs() const;
	uint8_t odrValue() const;
	uint8_t ctrl0Value(uint8_t command) const;
	bool settled(uint32_t nowMs, uint32_t ms, mmc56x3_state_t next);
};

} /* namespace MMC56X3 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/mmc56x3.tpp"

#endif /* MMC56X3_H_ */
