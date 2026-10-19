/*
 * lps35hw.h
 *
 *  Created on: Dec 31, 2025
 *      Author: coryg
 *
 *  LPS35HW pressure / temperature sensor, non-blocking state machine.
 *  Replaces lps35hw.cpp and lps35hwNB.h/.cpp.
 *
 *  The bus is chosen by the template argument. lps35hw<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      lps35hw<Stm32HalI2CTransport>  sensor(param, &hi2c1, LPS35HW_ADDR, i2c1Mutex);
 *      lps35hw<Stm32HalSPITransport>  sensor(param, &hspi1, GPIOB, GPIO_PIN_6, spi1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  The lps35hw_i2c / lps35hw_spi pair is gone: the one thing they
 *  differed in, whether the chip's I2C interface is switched off, is
 *  now worked out from the transport type.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each register access is one state that issues the transfer and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xlps35hw.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 *
 *  Measures once every lps35hw_period_ms, and sooner when
 *  startMeasurement() asks. The chip's FIFO is not used.
 */

#ifndef LPS35HW_H_
#define LPS35HW_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SPITransport.h"
#include "SensorStateMachine.h"

namespace LPS35HW {

#define LPS35HW_WHO_AM_I 0xB1
#define LPS35HW_ADDR 0x5D
// B8 = 10111000
// 5C = 01011100

// Every bus access is two states: one that issues the transfer and a
// wait_ state that it lands in.
typedef enum lps35hw_state_t{
	lps35hw_init_state,					// check a device answers
	lps35hw_who_am_i_state,				// issue the who_am_i read
	lps35hw_wait_who_am_i_state,
	lps35hw_reset_state,				// issue the software reset
	lps35hw_wait_reset_state,
	lps35hw_reset_settle_state,			// pause after reset and between boot polls
	lps35hw_boot_state,					// issue the int_source read
	lps35hw_wait_boot_state,			// boot_status clears once the trim values are loaded
	lps35hw_initializing_state,			// issue the ctrl_reg1 write
	lps35hw_wait_ctrl_reg1_state,
	lps35hw_ctrl_reg2_state,
	lps35hw_wait_ctrl_reg2_state,
	lps35hw_ctrl_reg3_state,
	lps35hw_wait_ctrl_reg3_state,
	lps35hw_fifo_ctrl_state,
	lps35hw_wait_fifo_ctrl_state,
	lps35hw_done_state,					// between measurements
	lps35hw_start_measurement_state,	// issue the one_shot write
	lps35hw_wait_start_measurement_state,
	lps35hw_wait_forced_measurement,	// conversion time
	lps35hw_one_shot_state,				// issue the ctrl_reg2 read: has one_shot cleared
	lps35hw_wait_one_shot_state,
	lps35hw_read_measurement_state,
	lps35hw_wait_measurement_state,
	lps35hw_err_state					// pause, then start again from lps35hw_init_state
}lps35hw_state_t;

typedef enum lps35hw_reg_addr_t{
	interrupt_cfg_addr = 0x0B,
	ths_p_l_addr = 0x0C,
	ths_p_h_addr = 0x0D,
	who_am_i_addr = 0x0F,
	ctrl_reg1_addr = 0x10,
	ctrl_reg2_addr = 0x11,
	ctrl_reg3_addr = 0x12,
	fifo_ctrl_addr = 0x14,
	ref_p_xl_addr = 0x15,
	ref_p_l_addr = 0x16,
	ref_p_h_addr = 0x17,
	rpds_l_addr = 0x18,
	rpds_h_addr = 0x19,
	res_conf_addr = 0x1A,
	int_source_addr = 0x25,
	fifo_status_addr = 0x26,
	status_addr = 0x27,
	press_out_xl_addr = 0x28,
	press_out_l_addr = 0x29,
	press_out_h_addr = 0x2A,
	temp_out_l_addr = 0x2B,
	temp_out_h_addr = 0x2C,
	lpfp_res_addr = 0x33
}lps35hw_reg_addr_t;

// data types for ctrl_reg1

// lps35hw Output Data Rate. lps35hw_odr_0hz is one-shot mode: the
// chip measures only when this driver asks. At any other rate the
// chip measures by itself and the driver reads the latest result.
typedef enum lps35hw_odr_t{
	lps35hw_odr_0hz = 0b000,
	lps35hw_odr_1hz = 0b001,
	lps35hw_odr_10hz = 0b010,
	lps35hw_odr_25hz = 0b011,
	lps35hw_odr_50hz = 0b100,
	lps35hw_odr_75hz = 0b101
}lps35hw_odr_t;

typedef bool lps35hw_en_lpfp_t;
typedef bool lps35hw_lpfp_cfg_t;

// data types for fifo_ctrl
typedef enum fifo_mode_t{
	fifo_bypass_mode = 0b000,
	fifo_mode = 0b001,
	stream_mode = 0b010,
	stream_to_fifo_mode = 0b011,
	bypass_to_stream_mode = 0b100,
	dynamic_stream_mode = 0b110,
	bypass_to_fifo_mode = 0b111
}fifo_mode_t;

typedef uint8_t lps35hw_reg_value;

// All times in ms.
const uint32_t lps35hw_bus_timeout_ms = 100;	// one transfer, to issue or to land
const uint32_t lps35hw_reset_settle_ms = 100;	// after reset, and between boot_status polls
const uint32_t lps35hw_phase_timeout_ms = 1000;	// start up, or one whole measurement
const uint32_t lps35hw_period_ms = 1000;		// between measurements
const uint32_t lps35hw_forced_ms = 100;			// soonest a startMeasurement() request is taken up
const uint32_t lps35hw_conversion_ms = 100;		// before first checking one_shot
const uint32_t lps35hw_error_backoff_ms = 100;	// in lps35hw_err_state before starting again

// The bus handle, device address and mutex are no longer here. They
// belong to the transport and go to its constructor. So does 3 or 4
// wire SPI: itransport's SPI is 4 wire.
typedef struct lps35hw_param_t{
	lps35hw_odr_t odr; // data rate
	lps35hw_en_lpfp_t en_lpfp; // enable lowpass filter
	lps35hw_lpfp_cfg_t lpfp_cfg; //

	// interrupt pin, ctrl_reg3
	bool int_h_l;
	bool pp_od;
	bool f_fss5;
	bool f_fth;
	bool f_ovr;
	bool drdy;
	bool int_s2;
	bool int_s1;

	fifo_mode_t fifo_mode;
	uint8_t fifo_threshold;
}lps35hw_param_t;

template <typename TTransport>
class lps35hw : public SensorStateMachine<TTransport, lps35hw_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"lps35hw<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, lps35hw_state_t> base;
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
	explicit lps35hw(const lps35hw_param_t &param, TArgs&&... transportArgs);
	virtual ~lps35hw() {}

	void main(uint32_t nowMs);
	void startMeasurement();	// ask for a measurement ahead of the next period
	void tempPressure(float *t,float *p);	// degrees C, hPa. NAN until the first reading
	bool newData();

protected:
	lps35hw_param_t _param;
	uint32_t phase_start;	// when start up, or the current measurement, began
	bool _force_measurement;
	float _temp,_pressure;
	bool _newData;
	uint8_t _who_am_i;

	// ctrl_reg_1 bit fields
	typedef union {
		struct {
			uint8_t sim : 1; // 3 or 4-wire SPI
			uint8_t bdu : 1; // Block Data Update
			uint8_t lpfp_cfg : 1; // Low-pass Filter Config
			uint8_t en_lpfp : 1; // Enable Low-Pass Filter
			uint8_t odr : 3; // Output Data Rate
			uint8_t : 1;
		}bit;
		uint8_t reg;
	}ctrl_reg1_value_t;
	ctrl_reg1_value_t _ctrl_reg1;

	// ctrl_reg_2 bit fields
	typedef union{
	struct {
		uint8_t one_shot : 1; // One-Shot Measurement
		uint8_t : 1;
		uint8_t swreset : 1; // Software Reset
		uint8_t i2c_dis : 1; // Disable I2C Interface
		uint8_t if_add_inc : 1; // Auto Increment Register address during reads
		uint8_t stop_on_fth : 1; // Stop on Fifo Threshold
		uint8_t fifo_en : 1; // Fifo Enable
		uint8_t boot : 1; // reloads trim parameters
	}bit;
	uint8_t reg;
	}ctrl_reg2_value_t;
	ctrl_reg2_value_t _ctrl_reg2;

	// ctrl_reg_3 bit fields
	typedef union{
		struct{
			uint8_t int_s1 : 1;
			uint8_t int_s2 : 1;
			uint8_t drdy : 1;
			uint8_t f_ovr : 1;
			uint8_t f_fth : 1;
			uint8_t f_fss5 : 1;
			uint8_t pp_od : 1;
			uint8_t int_h_l : 1;
		}bit;
		uint8_t reg;
	}ctrl_reg3_value_t;
	ctrl_reg3_value_t _ctrl_reg3;

	typedef union{
		struct {
			lps35hw_reg_value watermark : 5;
			lps35hw_reg_value fifo_mode : 3;
		}bit;
		uint8_t reg;
	}fifo_ctrl_value_t;
	fifo_ctrl_value_t _fifo_ctrl;

	typedef union{
		struct{
			uint8_t ph : 1;
			uint8_t pl : 1;
			uint8_t ia : 1;
			uint8_t : 4;
			uint8_t boot_status : 1;
		}bit;
		uint8_t reg;
	}int_source;
	int_source _int_source;

private:

	// The transport fills this in after main() has returned, so it
	// has to be a member and not a local.
	uint8_t dataReg[5];

	bool phaseExpired(uint32_t nowMs) const;
	uint8_t ctrlReg2(bool swreset, bool one_shot);
};

} /* namespace LPS35HW */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/lps35hw.tpp"

#endif /* LPS35HW_H_ */
