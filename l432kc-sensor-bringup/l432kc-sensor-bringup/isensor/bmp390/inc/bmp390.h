/*
 * bmp390.h
 *
 *  Bosch BMP390 (and the BMP388, which takes the same registers)
 *  pressure and temperature sensor, non-blocking state machine.
 *
 *  The bus is chosen by the template argument. bmp390<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      bmp390<Stm32HalI2CTransport>  sensor(param, &hi2c1, BMP390_I2C_ADDR_HIGH, i2c1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  I2C only: over SPI the chip sends a dummy byte before each read's
 *  data, which SPITransport doesn't strip.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each register access is one state that issues it and one that waits
 *  for it to land. When a state has nothing to do it calls sleep(), an
 *  empty stub here; xbmp390.h gives up the CPU instead.
 *
 *  What it does: reads the chip ID (0x60 BMP390, 0x50 BMP388) before
 *  writing anything, waits for cmd_rdy, soft resets the chip, reads the
 *  ID again and the error register (a command error fails), reads the
 *  21 bytes of calibration, and writes the oversampling and the IIR
 *  filter. Then every param.period_ms (the first at once), or when
 *  startMeasurement() asks: a forced measurement of both pressure and
 *  temperature (PWR_CTRL written whole, never read and modified), a
 *  sleep for the measurement time Bosch's API gives, and the status
 *  register polled until both data-ready bits are set, then the six
 *  data bytes read in one transfer.
 *
 *  Registers, bit positions, the calibration layout, the measurement
 *  time and the floating point compensation follow Bosch Sensortec's
 *  BMP3 Sensor API v2.0.6 (github.com/boschsensortec/BMP3_SensorAPI,
 *  BSD-3-Clause, Copyright (c) 2022 Bosch Sensortec GmbH), not the
 *  datasheet (not read here). One difference: Bosch's pow_bmp3()
 *  returns a float, so its pressure loses a little to rounding; here
 *  every product is a double (test/bmp390_ref_test.cpp shows how close
 *  the two are). Bosch's limits are kept: -40..85 C, 30000..125000 Pa,
 *  a reading outside clamped, as its API does.
 *
 *  Not done: normal mode, the FIFO, interrupts, SPI.
 *  Not yet run against a chip.
 */

#ifndef BMP390_H_
#define BMP390_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"
#include "Barometer.h"
#include "Thermometer.h"

namespace BMP390 {

#define BMP390_I2C_ADDR_LOW   0x76		// SDO to GND
#define BMP390_I2C_ADDR_HIGH  0x77		// SDO to VDDIO (most breakouts, Adafruit's included)
#define BMP390_CHIP_ID        0x60
#define BMP388_CHIP_ID        0x50
#define BMP390_SOFT_RESET     0xB6
#define BMP390_LEN_CALIB      21
#define BMP390_LEN_DATA       6

typedef enum bmp390_reg_t{
	bmp390_reg_chip_id = 0x00,
	bmp390_reg_err = 0x02,				// bit 0 fatal_err, bit 1 cmd_err, bit 2 conf_err
	bmp390_reg_status = 0x03,			// bit 4 cmd_rdy, bit 5 drdy_press, bit 6 drdy_temp
	bmp390_reg_data = 0x04,				// pressure xlsb, lsb, msb, then temperature
	bmp390_reg_pwr_ctrl = 0x1B,			// bit 0 press_en, bit 1 temp_en, bits 5:4 mode
	bmp390_reg_osr = 0x1C,				// bits 2:0 osr_p, bits 5:3 osr_t
	bmp390_reg_odr = 0x1D,
	bmp390_reg_config = 0x1F,			// bits 3:1 iir_filter
	bmp390_reg_calib = 0x31,
	bmp390_reg_cmd = 0x7E
}bmp390_reg_t;

const uint8_t bmp390_err_cmd = 0x02;
const uint8_t bmp390_status_cmd_rdy = 0x10;
const uint8_t bmp390_status_drdy_press = 0x20;
const uint8_t bmp390_status_drdy_temp = 0x40;
const uint8_t bmp390_pwr_press_en = 0x01;
const uint8_t bmp390_pwr_temp_en = 0x02;
const uint8_t bmp390_mode_forced = 0x01;	// bits 5:4

// Oversampling, for pressure and temperature alike
typedef enum bmp390_osr_t{
	bmp390_osr_1x = 0,
	bmp390_osr_2x = 1,
	bmp390_osr_4x = 2,
	bmp390_osr_8x = 3,
	bmp390_osr_16x = 4,
	bmp390_osr_32x = 5
}bmp390_osr_t;

// IIR filter coefficient
typedef enum bmp390_iir_t{
	bmp390_iir_off = 0,
	bmp390_iir_1 = 1,
	bmp390_iir_3 = 2,
	bmp390_iir_7 = 3,
	bmp390_iir_15 = 4,
	bmp390_iir_31 = 5,
	bmp390_iir_63 = 6,
	bmp390_iir_127 = 7
}bmp390_iir_t;

typedef enum bmp390_variant_t{
	bmp390_variant_unknown = 0,
	bmp390_variant_bmp390 = BMP390_CHIP_ID,
	bmp390_variant_bmp388 = BMP388_CHIP_ID
}bmp390_variant_t;

// Every bus access is two states: one that issues it and a wait_ state
// it lands in.
typedef enum bmp390_state_t{
	bmp390_init_state,
	bmp390_id_state,				// the chip ID, before anything is written
	bmp390_wait_id_state,
	bmp390_cmd_rdy_state,			// the status register: ready for a command
	bmp390_wait_cmd_rdy_state,
	bmp390_cmd_rdy_poll_state,		// pause between cmd_rdy polls
	bmp390_reset_state,				// issue the soft reset
	bmp390_wait_reset_state,
	bmp390_reset_settle_state,		// pause after the reset
	bmp390_id2_state,				// the ID again: back from the reset
	bmp390_wait_id2_state,
	bmp390_err_state,				// the error register: did the reset command fail
	bmp390_wait_err_state,
	bmp390_calib_state,				// the 21 calibration bytes
	bmp390_wait_calib_state,
	bmp390_osr_state,				// write the oversampling
	bmp390_wait_osr_state,
	bmp390_config_state,			// write the IIR filter
	bmp390_wait_config_state,
	bmp390_done_state,				// between measurements
	bmp390_force_state,				// issue the forced measurement
	bmp390_wait_force_state,
	bmp390_measuring_state,			// the measurement time
	bmp390_status_state,			// issue the status read: data ready?
	bmp390_wait_status_state,
	bmp390_status_poll_state,		// pause between status polls
	bmp390_data_state,				// issue the six byte data read
	bmp390_wait_data_state,
	bmp390_error_state				// pause, then start again from init
}bmp390_state_t;

// All times in ms.
const uint32_t bmp390_bus_timeout_ms = 100;		// one bus operation, to issue or to land
const uint32_t bmp390_reset_settle_ms = 10;		// after the soft reset (Bosch's API waits 2)
const uint32_t bmp390_cmd_rdy_timeout_ms = 100;	// cmd_rdy must be set within this, at start up
const uint32_t bmp390_poll_ms = 2;				// between status polls
const uint32_t bmp390_phase_margin_ms = 100;	// a measurement fails this long after its expected end
const uint32_t bmp390_request_poll_ms = 100;	// longest sleep while a startMeasurement() could arrive
const uint32_t bmp390_error_backoff_ms = 100;	// in bmp390_error_state before starting again

typedef struct bmp390_param_t{
	bmp390_osr_t osr_pressure;
	bmp390_osr_t osr_temperature;
	bmp390_iir_t iir;
	uint32_t period_ms;				// between measurement starts
}bmp390_param_t;

// The calibration, as Bosch's quantized (floating point) form.
typedef struct bmp390_calib_t{
	double par_t1, par_t2, par_t3;
	double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
}bmp390_calib_t;

// The 21 calibration bytes (from register 0x31) as Bosch's
// parse_calib_data() (floating point build) quantizes them.
inline bmp390_calib_t bmp390_parse_calib(const uint8_t *r);

// Bosch's compensation, in doubles. temperature: degrees C; pressure:
// Pa. Both clamped to Bosch's limits.
inline double bmp390_compensate_temperature(const bmp390_calib_t &c, uint32_t adcT);
inline double bmp390_compensate_pressure(const bmp390_calib_t &c, double temperature, uint32_t adcP);

// The measurement time Bosch's API gives for forced mode, in us:
// 234 + 392 + 2^osr_p x 2000 + 313 + 2^osr_t x 2000.
inline uint32_t bmp390_measurement_us(bmp390_osr_t osrP, bmp390_osr_t osrT);

template <typename TTransport>
class bmp390 : public SensorStateMachine<TTransport, bmp390_state_t>, public Barometer, public Thermometer {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"bmp390<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, bmp390_state_t> base;
protected:
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::issued;
	using base::landed;
	using base::finished;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit bmp390(const bmp390_param_t &param, TArgs&&... transportArgs);
	virtual ~bmp390() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a measurement now
	bool newData();				// true once per measurement

	bool GetTemperature(double *t);	// degrees C. false (NAN) until the first reading, and after a failure
	bool GetPressure(double *p);	// Pa. likewise

	// Barometer and Thermometer (isensor/inc): the same readings as
	// floats. They leave newData() alone.
	float getPressure() const override { return (float)_pressure; }
	float getTemperature() const override { return (float)_temperature; }

	bmp390_variant_t variant() const { return _variant; }
	// One measurement's time with this configuration, rounded up to the ms
	uint32_t measurementMs() const;
	const bmp390_calib_t& calibration() const { return _calib; }

protected:
	void onFail() override;

private:
	bmp390_param_t _param;
	bmp390_calib_t _calib;
	bmp390_variant_t _variant;
	double _temperature, _pressure;
	bool _newData;
	bool _start_measurement;
	uint32_t phase_start;		// when start up's cmd_rdy wait, or the current measurement, began

	// The transport fills these in after main() has returned, so they
	// have to be members and not locals.
	uint8_t _id;
	uint8_t _status;
	uint8_t _err;
	uint8_t _calibReg[BMP390_LEN_CALIB];
	uint8_t _data[BMP390_LEN_DATA];

	bool settled(uint32_t nowMs, uint32_t ms, bmp390_state_t next);
	void convert();
};

} /* namespace BMP390 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/bmp390.tpp"

#endif /* BMP390_H_ */
