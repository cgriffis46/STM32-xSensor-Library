/*
 * BME280.h
 *
 *  Created on: Nov 16, 2025
 *      Author: coryg
 *
 *  BME280 pressure / temperature / humidity sensor, non-blocking
 *  state machine.
 *
 *  The bus is chosen by the template argument. bme280<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      bme280<Stm32HalI2CTransport>  sensor(param, &hi2c1, bme280_i2c_addr_2, i2c1Mutex);
 *      bme280<Stm32HalSPITransport>  sensor(param, &hspi1, GPIOB, GPIO_PIN_6, spi1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header, so it builds for any
 *  target the transport builds for.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each register access is one state that issues the transfer and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xBME280.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace BME280 {

#define bme280_id 0x60

typedef enum bme280_i2c_addr_t{
	bme280_i2c_addr_1 = 0x76,
	bme280_i2c_addr_2 = 0x77,
}bme280_i2c_addr_t;

// defines for register addresses

typedef enum bme280_reg_addr_t{
	bme280_reg_id_addr = 0xD0,
	bme280_reg_reset_addr = 0xE0,
	bme280_reg_ctrl_hum_addr = 0xF2,
	bme280_reg_status_addr = 0xF3,
	bme280_reg_ctrl_meas_addr = 0xF4,
	bme280_reg_config_addr = 0xF5,
	bme280_reg_press_msb_addr = 0xF7,
	bme280_reg_press_lsb_addr = 0xF8,
	bme280_reg_press_xlsb_addr = 0xF9,
	bme280_reg_temp_msb_addr = 0xFA,
	bme280_reg_temp_lsb_addr = 0xFB,
	bme280_reg_temp_xlsb_addr = 0xFC,
	bme280_reg_hum_msb_addr = 0xFD,
	bme280_reg_hum_lsb_addr = 0xFE,
	bme280_reg_cal_addr = 0x88,
	bme280_reg_dig_H1_addr = 0xA1,
	bme280_reg_dig_H2_addr = 0xE1
}bme280_reg_addr_t;

typedef uint8_t bme280_reg8_val_t;
typedef int32_t bme280_S32_t;
typedef uint32_t bme280_U32_t;
const bme280_reg8_val_t bme280_reset_value = 0xB6;

// pressure oversampling register settings
typedef enum bme280_osrs_p_t{
 bme280_osrs_p_skip = 0b000,
 bme280_osrs_p_1x = 0b001,
 bme280_osrs_p_2x = 0b010,
 bme280_osrs_p_4x = 0b011,
 bme280_osrs_p_8x = 0b100,
 bme280_osrs_p_16x = 0b101
}bme280_osrs_p_t;

// temperature oversampling register settings
typedef enum bme280_osrs_t_t{
 bme280_osrs_t_skip = 0b000,
 bme280_osrs_t_1x = 0b001,
 bme280_osrs_t_2x = 0b010,
 bme280_osrs_t_4x = 0b011,
 bme280_osrs_t_8x = 0b100,
 bme280_osrs_t_16x = 0b101
}bme280_osrs_t_t;

typedef enum bme280_osrs_h_t{
	 bme280_osrs_h_skip = 0b000,
	 bme280_osrs_h_1x = 0b001,
	 bme280_osrs_h_2x = 0b010,
	 bme280_osrs_h_4x = 0b011,
	 bme280_osrs_h_8x = 0b100,
	 bme280_osrs_h_16x = 0b101
}bme280_osrs_h_t;

typedef enum bme280_mode_t{
	bme280_sleep_mode = 0b00,
	bme280_forced_mode = 0b10,
	bme280_normal_mode = 0b11
}bme280_mode_t;

// standby time in normal mode. The last two differ from the BMP280,
// where the same settings mean 2000 ms and 4000 ms.
typedef enum bme280_sb_t{
	bme280_sb_05 = 0b000,
	bme280_sb_62 = 0b001,
	bme280_sb_125 = 0b010,
	bme280_sb_250 = 0b011,
	bme280_sb_500 = 0b100,
	bme280_sb_1000 = 0b101,
	bme280_sb_10 = 0b110,
	bme280_sb_20 = 0b111
}bme280_sb_t;

// IIR filter coefficient
typedef enum bme280_filter_t{
	bme280_filter_off = 0b000,
	bme280_filter_x2 = 0b001,
	bme280_filter_x4 = 0b010,
	bme280_filter_x8 = 0b011,
	bme280_filter_x16 = 0b100
}bme280_filter_t;

typedef enum bme280_spi_en{
	bme280_spi_en_off = 0b0,
	bme280_spi_en_on = 0b1
}bme280_spi_en;

// Every bus access is two states: one that issues the transfer and a
// wait_ state that it lands in. The transport never blocks, so the
// state machine has to remember which half it is in.
typedef enum bme280_state_t{
	bme280_init,
	bme280_reset,					// issue the soft reset
	bme280_wait_reset,
	bme280_reset_settle,			// pause after reset
	bme280_read_id,
	bme280_wait_id,
	bme280_im_update_pause,			// pause between im_update polls
	bme280_read_im_update,			// issue the status read
	bme280_wait_im_update,			// im_update clears once the trim values are copied
	bme280_read_cal,				// temperature and pressure trim, 0x88
	bme280_wait_cal,
	bme280_read_cal_h1,				// humidity trim, first byte, 0xA1
	bme280_wait_cal_h1,
	bme280_read_cal_h2,				// humidity trim, the rest, 0xE1
	bme280_wait_cal_h2,
	bme280_write_ctrl_hum,
	bme280_wait_ctrl_hum,
	bme280_write_config,
	bme280_wait_config,
	bme280_write_ctrl_meas,
	bme280_wait_ctrl_meas,
	bme280_sleeping,				// between measurements
	bme280_start_measurement,		// issue the status read: is the chip idle
	bme280_wait_idle,
	bme280_force_measurement,		// issue the forced mode write
	bme280_wait_force,
	bme280_measuring,				// conversion time
	bme280_read_measuring,			// issue the status read: has it finished
	bme280_wait_measuring,
	bme280_read_data,
	bme280_wait_data,
	bme280_convert_measurement,
	bme280_error					// pause, then start again from bme280_init
}bme280_state_t;

// All times in ms.
const uint32_t bme280_bus_timeout_ms = 100;		// one transfer, to issue or to land
const uint32_t bme280_reset_settle_ms = 100;	// after the soft reset, before asking for the ID
const uint32_t bme280_im_update_poll_ms = 10;	// between im_update polls
const uint32_t bme280_phase_timeout_ms = 1000;	// start up, or one whole measurement
const uint32_t bme280_sleep_ms = 100;			// between measurements
const uint32_t bme280_conversion_ms = 100;		// before first checking the measuring bit
const uint32_t bme280_error_backoff_ms = 100;	// in bme280_error before starting again

// The bus handle and device address are no longer here. They belong
// to the transport and go to its constructor.
typedef struct bme280_param_t{
	bme280_osrs_p_t _osrs_p_t;
	bme280_osrs_t_t _osrs_t_t;
	bme280_osrs_h_t _osrs_h_t;
	bme280_mode_t _mode_t;
	bme280_sb_t _sb_t;
	bme280_filter_t _filter_t;
}bme280_param_t;

template <typename TTransport>
class bme280 : public SensorStateMachine<TTransport, bme280_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"bme280<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, bme280_state_t> base;
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
	explicit bme280(const bme280_param_t &param, TArgs&&... transportArgs);
	virtual ~bme280() {}

	void main(uint32_t nowMs);

	bool newData();	// true once per measurement

	bool GetTemperature(double *t);	// degrees C. false until the first reading
	bool GetPressure(double *p);	// Pa. false until the first reading
	bool GetHumidity(double *h);	// %RH. false until the first reading

protected:

	uint8_t _id;
	bme280_S32_t t_fine;

	unsigned char dig_H1;
	signed short dig_H2;
	unsigned char dig_H3;
	signed short dig_H4;
	signed short dig_H5;
	signed char dig_H6;

	unsigned short dig_T1;
	signed short dig_T2, dig_T3;
	unsigned short dig_P1;
	signed short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

	uint32_t phase_start;	// when start up, or the current measurement, began

	bool _newData;
	bme280_param_t _param;

	typedef union{
		struct{
			uint8_t spi3w_en : 1;
			uint8_t : 1;
			uint8_t filter : 3;
			uint8_t t_sb : 3;
		}bit;
		uint8_t reg;
	}config_reg_t;
	config_reg_t _config;

	typedef union{
		struct{
			uint8_t mode : 2;   // data aquisition mode
			uint8_t osrs_p : 3; // pressure oversampling rate
			uint8_t osrs_t : 3; // temperature oversamping rate
		}bit;
		uint8_t reg;
	}ctrl_meas_reg_t;
	ctrl_meas_reg_t _ctrl_meas;

	typedef union{
		struct{
			uint8_t im_update : 1; // true when the sensor is loading calibration values.
			uint8_t : 2;
			uint8_t measuring : 1; // true when the sensor is measuring
			uint8_t : 4;
		}bit;
		uint8_t reg;
	}status_reg_t;
	status_reg_t _status;

	typedef union{
		struct{
			uint8_t osrs_h : 3; // humidity oversampling rate
			uint8_t : 5;
		}bit;
		uint8_t reg;
	}ctrl_hum_reg_t;
	ctrl_hum_reg_t _ctrl_hum;

	double _temp,_pressure,_humidity;
	bme280_S32_t adc_t, adc_p, adc_h;

private:

	// The transport fills these in after main() has returned, so they
	// have to be members and not locals.
	uint8_t calReg[24];
	uint8_t calH1Reg;
	uint8_t calHReg[7];
	uint8_t dataReg[8];

	bool phaseExpired(uint32_t nowMs) const;

	void parseCal();
	void parseHumCal();
	double bme280_compensate_T_double(bme280_S32_t adc_T);
	double bme280_compensate_P_double(bme280_S32_t adc_P);
	double bme280_compensate_H_double(bme280_S32_t adc_H);

};

} /* namespace BME280 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/BME280.tpp"

#endif /* BME280_H_ */
