/*
 * bme680.h
 *
 *  Bosch BME680 gas, pressure, temperature and humidity sensor (and
 *  the BME688 in the same forced mode), non-blocking state machine.
 *
 *  The bus is chosen by the template argument. bme680<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      bme680<Stm32HalI2CTransport>  sensor(param, &hi2c1, BME680_I2C_ADDR_HIGH, i2c1Mutex);
 *      bme680<Stm32HalSPITransport>  sensor(param, &hspi1, GPIOB, GPIO_PIN_6, spi1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  param.bus must say which bus it is: over SPI the chip shows its
 *  registers in two pages of 128, and the driver switches pages
 *  (status register, spi_mem_page). Register numbers are the I2C ones
 *  either way; SPITransport puts the read bit on itself.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each register access is one state that issues it and one state
 *  that waits for it to land. When a state has nothing to do it calls
 *  sleep(), an empty stub here; xbme680.h gives up the CPU instead.
 *
 *  What it does: reads the chip ID, soft resets the chip, reads the ID
 *  again, the variant (BME680 or BME688) and the 42 bytes of
 *  calibration, reads the control registers once (to keep the bits it
 *  does not set as the chip has them, as Bosch's API does), and writes
 *  the oversampling, the IIR filter and heater set-point 0. Then every
 *  param.period_ms (the first at once), or when startMeasurement() asks:
 *  starts a forced measurement (ctrl_meas), waits the measurement
 *  time Bosch's API gives plus the heater time, and reads field 0
 *  (17 bytes) until new_data is set.
 *
 *  The heater's resistance value is worked out once, for
 *  param.ambient_c, as Bosch's API does with its amb_temp (25 C in its
 *  examples). Its integer formula hardly uses it: over -40 to 85 C the
 *  value moves by one step at most with typical calibration, so it
 *  isn't worked out again from the temperature measured.
 *
 *  Temperature is always measured (oversampling skip is raised to 1x):
 *  pressure, humidity and the heater are all compensated with it.
 *
 *  The gas resistance is reported only when the chip says the gas
 *  reading is valid and the heater reached its temperature
 *  (gas_valid_r and heat_stab_r); NAN otherwise, with both flags in
 *  the reading. Bosch's BSEC library (IAQ, CO2 equivalent) is not
 *  part of this: it is closed source, and takes these raw readings.
 *
 *  Not done: the BME688's parallel and sequential modes (heater
 *  profiles of up to 10 steps), the 3-wire SPI mode and the self test.
 *
 *  Registers, bit positions, the calibration layout, the measurement
 *  time and the integer compensation formulas (temperature, pressure,
 *  humidity, both gas formulas, the heater resistance and gas_wait
 *  encodings) follow Bosch Sensortec's BME68x Sensor API v4.4.8
 *  (github.com/boschsensortec/BME68x_SensorAPI, BSD-3-Clause,
 *  Copyright (c) 2023 Bosch Sensortec GmbH), not the datasheet (not
 *  downloaded here). test/bme680_ref_test.cpp runs that API and this
 *  driver against the same simulated chip and compares them. Not yet
 *  run against a chip.
 */

#ifndef BME680_H_
#define BME680_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace BME680 {

#define BME680_I2C_ADDR_LOW  0x76		// SDO to GND
#define BME680_I2C_ADDR_HIGH 0x77		// SDO to VDDIO
#define BME680_CHIP_ID       0x61		// BME680 and BME688 alike
#define BME680_SOFT_RESET    0xB6

typedef enum bme680_register_t{
	bme680_reg_coeff3 = 0x00,			// calibration, 5 bytes
	bme680_reg_field0 = 0x1D,			// meas_status_0, then 16 bytes of results
	bme680_reg_res_heat0 = 0x5A,		// heater set-point 0: resistance
	bme680_reg_gas_wait0 = 0x64,		// heater set-point 0: time
	bme680_reg_ctrl_gas0 = 0x70,
	bme680_reg_ctrl_gas1 = 0x71,
	bme680_reg_ctrl_hum = 0x72,
	bme680_reg_mem_page = 0xF3,			// SPI only: status, spi_mem_page. 0x73 on the wire, in both pages
	bme680_reg_ctrl_meas = 0x74,
	bme680_reg_config = 0x75,
	bme680_reg_coeff1 = 0x8A,			// calibration, 23 bytes
	bme680_reg_chip_id = 0xD0,
	bme680_reg_soft_reset = 0xE0,
	bme680_reg_coeff2 = 0xE1,			// calibration, 14 bytes
	bme680_reg_variant_id = 0xF0
}bme680_register_t;

#define BME680_LEN_COEFF1 23
#define BME680_LEN_COEFF2 14
#define BME680_LEN_COEFF3 5
#define BME680_LEN_COEFF  (BME680_LEN_COEFF1+BME680_LEN_COEFF2+BME680_LEN_COEFF3)
#define BME680_LEN_FIELD  17
#define BME680_LEN_CTRL   6				// 0x70 to 0x75

// Bits, from Bosch's bme68x_defs.h.
#define BME680_NEW_DATA        0x80		// meas_status_0
#define BME680_GAS_VALID       0x20		// gas_r_lsb
#define BME680_HEAT_STAB       0x10		// gas_r_lsb
#define BME680_GAS_RANGE_MSK   0x0F		// gas_r_lsb
#define BME680_MEM_PAGE_BIT    0x10		// status: set for 0x00-0x7F, clear for 0x80-0xFF
#define BME680_HEAT_OFF        0x08		// ctrl_gas_0
#define BME680_ODR3            0x80		// ctrl_gas_1: set means no standby (Bosch's ODR_NONE)
#define BME680_RUN_GAS_MSK     0x30		// ctrl_gas_1
#define BME680_NB_CONV_MSK     0x0F		// ctrl_gas_1
#define BME680_OSRS_H_MSK      0x07		// ctrl_hum
#define BME680_FILTER_MSK      0x1C		// config
#define BME680_ODR20_MSK       0xE0		// config
#define BME680_MODE_FORCED     0x01		// ctrl_meas

typedef enum bme680_bus_t{
	bme680_i2c = 0,
	bme680_spi = 1						// 4-wire
}bme680_bus_t;

typedef enum bme680_variant_t{
	bme680_variant_bme680 = 0x00,		// Bosch: VARIANT_GAS_LOW
	bme680_variant_bme688 = 0x01,		// Bosch: VARIANT_GAS_HIGH
	bme680_variant_unknown = 0xFF		// not read yet
}bme680_variant_t;

// Oversampling: osrs_t, osrs_p, osrs_h.
typedef enum bme680_os_t{
	bme680_os_skip = 0,					// not measured; temperature is raised to 1x
	bme680_os_1x = 1,
	bme680_os_2x = 2,
	bme680_os_4x = 3,
	bme680_os_8x = 4,
	bme680_os_16x = 5
}bme680_os_t;

// IIR filter on temperature and pressure: config bits 4:2.
typedef enum bme680_filter_t{
	bme680_filter_off = 0,
	bme680_filter_1 = 1,
	bme680_filter_3 = 2,
	bme680_filter_7 = 3,
	bme680_filter_15 = 4,
	bme680_filter_31 = 5,
	bme680_filter_63 = 6,
	bme680_filter_127 = 7
}bme680_filter_t;

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum bme680_state_t{
	bme680_init_state,					// check a device answers
	bme680_page_read_state,				// SPI: issue the status read, to change page
	bme680_wait_page_read_state,
	bme680_page_write_state,			// SPI: issue the status write with the new page
	bme680_wait_page_write_state,
	bme680_id_state,					// issue the chip ID read
	bme680_wait_id_state,
	bme680_reset_state,					// issue the soft reset
	bme680_wait_reset_state,
	bme680_reset_settle_state,			// the chip restarts
	bme680_id2_state,					// the chip ID again, after the reset
	bme680_wait_id2_state,
	bme680_variant_state,				// issue the variant ID read
	bme680_wait_variant_state,
	bme680_cal1_state,					// calibration, 0x8A
	bme680_wait_cal1_state,
	bme680_cal2_state,					// calibration, 0xE1
	bme680_wait_cal2_state,
	bme680_cal3_state,					// calibration, 0x00
	bme680_wait_cal3_state,
	bme680_ctrl_read_state,				// control registers 0x70 to 0x75, once
	bme680_wait_ctrl_read_state,
	bme680_config_state,				// issue the next write of the configuration
	bme680_wait_config_state,
	bme680_done_state,					// between measurements
	bme680_force_state,					// issue ctrl_meas with forced mode
	bme680_wait_force_state,
	bme680_measuring_state,				// measurement and heater time
	bme680_field_state,					// issue the field 0 read
	bme680_wait_field_state,
	bme680_field_poll_state,			// new_data not set yet: wait, then read again
	bme680_error_state					// pause, then start again from bme680_init_state
}bme680_state_t;

// All times in ms.
const uint32_t bme680_bus_timeout_ms = 100;		// one bus operation, to issue or to land
const uint32_t bme680_reset_settle_ms = 10;		// after the soft reset (Bosch's BME68X_PERIOD_RESET)
const uint32_t bme680_field_poll_ms = 5;		// between field reads while new_data is clear
const uint32_t bme680_phase_margin_ms = 100;	// a measurement fails this long after its expected end
const uint32_t bme680_request_poll_ms = 100;	// longest sleep while a startMeasurement() could arrive
const uint32_t bme680_error_backoff_ms = 100;	// in bme680_error_state before starting again

const uint16_t bme680_heater_max_c = 400;		// Bosch caps the set-point here
const uint16_t bme680_heater_max_ms = 4032;		// gas_wait's longest (63 x 64)

// The bus handle and device address belong to the transport and go
// to its constructor.
typedef struct bme680_param_t{
	bme680_bus_t bus;
	bme680_os_t os_temperature;			// skip is raised to 1x
	bme680_os_t os_pressure;			// skip: pressure NAN
	bme680_os_t os_humidity;			// skip: humidity NAN
	bme680_filter_t filter;
	uint16_t heater_c;					// heater set-point, up to 400 C. 0: no gas measurement, heater off
	uint16_t heater_ms;					// how long it is held there, 1 to 4032 ms (Bosch's examples: 300 C, 100 ms)
	int8_t ambient_c;					// for the heater value (Bosch's examples: 25)
	uint32_t period_ms;					// between measurements
}bme680_param_t;

typedef struct bme680_reading_t{
	float temperature;					// degrees C, 0.01 steps
	float pressure;						// Pa
	float humidity;						// %RH, 0.001 steps
	float gas_resistance;				// ohms. NAN unless gas_valid and heat_stable
	bool gas_valid;						// gas_valid_r: a gas conversion was made
	bool heat_stable;					// heat_stab_r: the heater reached its temperature in time
}bme680_reading_t;

// The calibration, as Bosch's struct bme68x_calib_data holds it.
typedef struct bme680_calib_t{
	uint16_t par_h1, par_h2;
	int8_t par_h3, par_h4, par_h5;
	uint8_t par_h6;
	int8_t par_h7;
	int8_t par_gh1;
	int16_t par_gh2;
	int8_t par_gh3;
	uint16_t par_t1;
	int16_t par_t2;
	int8_t par_t3;
	uint16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int16_t par_p4, par_p5;
	int8_t par_p6, par_p7;
	int16_t par_p8, par_p9;
	uint8_t par_p10;
	uint8_t res_heat_range;
	int8_t res_heat_val;
	int8_t range_sw_err;
}bme680_calib_t;

// Pure functions, Bosch's integer ones (BME68X_DO_NOT_USE_FPU).
// Public so the reference test can call them.
inline bme680_calib_t bme680_parse_calib(const uint8_t coeff[BME680_LEN_COEFF]);
inline int16_t bme680_calc_temperature(uint32_t adc, const bme680_calib_t &c, int32_t *t_fine);	// 0.01 C
inline uint32_t bme680_calc_pressure(uint32_t adc, const bme680_calib_t &c, int32_t t_fine);		// Pa
inline uint32_t bme680_calc_humidity(uint16_t adc, const bme680_calib_t &c, int32_t t_fine);		// 0.001 %RH
inline uint32_t bme680_calc_gas_low(uint16_t adc, uint8_t range, const bme680_calib_t &c);		// BME680, ohms
inline uint32_t bme680_calc_gas_high(uint16_t adc, uint8_t range);								// BME688, ohms
inline uint8_t bme680_calc_res_heat(uint16_t temp_c, int8_t ambient_c, const bme680_calib_t &c);
inline uint8_t bme680_calc_gas_wait(uint16_t ms);
inline uint32_t bme680_gas_wait_ms(uint8_t gas_wait);		// what the chip waits for that encoding
inline uint32_t bme680_meas_us(uint8_t os_t, uint8_t os_p, uint8_t os_h);	// Bosch's bme68x_get_meas_dur, forced mode

template <typename TTransport>
class bme680 : public SensorStateMachine<TTransport, bme680_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"bme680<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, bme680_state_t> base;
protected:
	// Names from a template base class have to be brought in before
	// they can be used unqualified.
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
	explicit bme680(const bme680_param_t &param, TArgs&&... transportArgs);
	virtual ~bme680() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a measurement now
	bool newData();				// true once per measurement, until getReading()

	// The last measurement; every value NAN until the first one, and
	// after a failure. Clears newData().
	void getReading(bme680_reading_t *r);

	bme680_variant_t variant() const { return _variant; }

	// How long one measurement takes with this configuration, heater
	// included, rounded up to the ms.
	uint32_t measurementMs() const;

protected:
	void onFail() override;

private:
	bme680_param_t _param;
	bme680_calib_t _calib;
	bme680_variant_t _variant;
	bme680_reading_t _reading;
	bool _newData;
	bool _start_measurement;
	uint32_t phase_start;		// when the current measurement began
	uint8_t _page;				// SPI: BME680_MEM_PAGE_BIT or 0, 0xFF when not known
	uint8_t _pageWant;
	bme680_state_t _pageReturn;
	uint8_t _cfgIndex;			// which configuration write is next
	uint8_t _cfgReg[7];
	uint8_t _cfgVal[7];
	uint8_t _cfgCount;

	// The transport fills these in after main() has returned, so they
	// have to be members and not locals.
	uint8_t _id;
	uint8_t _variantReg;
	uint8_t _status;
	uint8_t _coeff[BME680_LEN_COEFF];
	uint8_t _ctrl[BME680_LEN_CTRL];
	uint8_t _field[BME680_LEN_FIELD];

	bool pageFor(uint8_t reg, uint32_t nowMs);
	bool settled(uint32_t nowMs, uint32_t ms, bme680_state_t next);
	void buildConfig();
	uint8_t ctrlMeas(uint8_t mode) const;
	uint8_t osTemperature() const;
	bool gasOn() const { return _param.heater_c!=0; }
	void convert();
};

} /* namespace BME680 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/bme680.tpp"

#endif /* BME680_H_ */
