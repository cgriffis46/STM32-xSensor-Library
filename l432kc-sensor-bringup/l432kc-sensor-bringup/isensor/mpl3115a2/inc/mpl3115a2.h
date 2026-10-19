/*
 * mpl3115a2.h
 *
 *  Created on: Dec 29, 2025
 *      Author: coryg
 *
 *  MPL3115A2 pressure / altitude / temperature sensor, non-blocking
 *  state machine.
 *
 *  The bus is chosen by the template argument. mpl3115a2<TTransport>
 *  inherits from TTransport, which must be an ISensorTransport
 *  (itransport/inc/ISensorTransport.h):
 *
 *      mpl3115a2::mpl3115a2<Stm32HalI2CTransport>  sensor(param, &hi2c1, mpl3115a2::MPL3115A2_DEV_ADDRESS, i2c1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  Call main(now) every pass of the loop. It never waits on the bus:
 *  each register access is one state that issues the transfer and one
 *  state that waits for it to land. When a state has nothing to do it
 *  calls sleep(), an empty stub here. Override sleep() for an OS (see
 *  xmpl3115a2.h) to give up the CPU for that long.
 *
 *  The state bookkeeping, sleep() and the issue/wait helpers are
 *  inherited from SensorStateMachine (isensor/inc), shared with the
 *  other drivers.
 */

#ifndef MPL3115A2_H_
#define MPL3115A2_H_

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"

namespace mpl3115a2 {

#define mpl3115a2_who_am_i 0xC4
const uint16_t MPL3115A2_DEV_ADDRESS = 0x60;

typedef enum mpl3115a2_reg_addr_t{
	mpl3115a2_status_addr = 0x00,
	mpl3115a2_out_p_msb_addr = 0x01,
	mpl3115a2_out_p_csb_addr = 0x02,
	mpl3115a2_out_p_lsb_addr = 0x03,
	mpl3115a2_out_t_msb_addr = 0x04,
	mpl3115a2_out_t_lsb_addr = 0x05,
	mpl3115a2_dr_status_addr = 0x06,
	mpl3115a2_out_p_delta_msb_addr = 0x07,
	mpl3115a2_out_p_delta_csb_addr = 0x08,
	mpl3115a2_out_p_delta_lsb_addr = 0x09,
	mpl3115a2_out_t_delta_msb_addr = 0x0A,
	mpl3115a2_out_t_delta_lsb_addr = 0x0B,
	mpl3115a2_who_am_i_addr = 0x0C,
	mpl3115a2_f_status_addr = 0x0D,
	mpl3115a2_f_data_addr = 0x0E,
	mpl3115a2_f_setup = 0x0F,
	mpl3115a2_time_dly_addr = 0x10,
	mpl3115a2_sysmod_addr = 0x11,
	mpl3115a2_int_source_addr = 0x12,
	mpl3115a2_pt_data_cfg_addr = 0x13,
	mpl3115a2_bar_in_msb_addr = 0x14,
	mpl3115a2_bar_in_lsb_addr = 0x15,
	mpl3115a2_p_tgt_msb_addr = 0x16,
	mpl3115a2_p_tgt_lsb_addr = 0x17,
	mpl3115a2_t_tgt_addr = 0x18,
	mpl3115a2_p_wnd_msb_addr = 0x19,
	mpl3115a2_p_wnd_lsb_addr = 0x1A,
	mpl3115a2_t_wnd_addr = 0x1B,
	mpl3115a2_p_min_msb_addr = 0x1C,
	mpl3115a2_p_min_csb_addr = 0x1D,
	mpl3115a2_p_min_lsb_addr = 0x1E,
	mpl3115a2_t_min_msb_addr = 0x1F,
	mpl3115a2_t_min_lsb_addr = 0x20,
	mpl3115a2_p_max_msb_addr = 0x21,
	mpl3115a2_p_max_csb_addr = 0x22,
	mpl3115a2_p_max_lsb_addr = 0x23,
	mpl3115a2_t_max_msb_addr = 0x24,
	mpl3115a2_t_max_lsb_addr = 0x25,
	mpl3115a2_ctrl_reg1_addr = 0x26,
	mpl3115a2_ctrl_reg2_addr = 0x27,
	mpl3115a2_ctrl_reg3_addr = 0x28,
	mpl3115a2_ctrl_reg4_addr = 0x29,
	mpl3115a2_ctrl_reg5_addr = 0x2A,
	mpl3115a2_off_p_addr = 0x2B,
	mpl3115a2_off_t_addr = 0x2C,
	mpl3115a2_off_h_addr = 0x2D
}mpl3115a2_reg_addr_t;

// Every bus access is two states: one that issues the transfer and a
// wait_ state that it lands in.
typedef enum mpl3115a2_state_t{
	mpl3115a2_init_state,						// pause, then check a device answers
	mpl3115a2_whoami_state,						// issue the who_am_i read
	mpl3115a2_wait_whoami_state,
	mpl3115a2_reset_state,						// issue the software reset
	mpl3115a2_wait_reset_state,
	mpl3115a2_reset_settle_state,				// pause between RST polls
	mpl3115a2_read_reset_state,					// issue the ctrl_reg1 read: has RST cleared
	mpl3115a2_wait_read_reset_state,
	mpl3115a2_initializing_state,				// issue the ctrl_reg1 write
	mpl3115a2_wait_ctrl_reg1_state,
	mpl3115a2_pt_data_cfg_state,
	mpl3115a2_wait_pt_data_cfg_state,
	mpl3115a2_ctrl_reg5_state,					// interrupt routing, after SetIRQ()
	mpl3115a2_wait_ctrl_reg5_state,
	mpl3115a2_ctrl_reg4_state,					// interrupt enables, after SetIRQ()
	mpl3115a2_wait_ctrl_reg4_state,
	mpl3115a2_done_state,						// between measurements
	mpl3115a2_initiate_one_shot_measurement,	// issue the ctrl_reg1 read: is one already running
	mpl3115a2_wait_ost_state,
	mpl3115a2_write_ost_state,					// issue the ctrl_reg1 write with OST set
	mpl3115a2_wait_write_ost_state,
	mpl3115a2_wait_one_shot_measurement,		// pause between status polls
	mpl3115a2_status_state,						// issue the status read
	mpl3115a2_wait_status_state,
	mpl3115a2_conversion_state,					// issue the 5 byte data read
	mpl3115a2_wait_conversion_state,
	mpl3115a2_error_state						// pause, then start again from mpl3115a2_init_state
}mpl3115a2_state_t;

typedef enum mpl3115a2_mode_t{
	mpl3115a2_altimeter_mode=0b1,
	mpl3115a2_barometer_mode=0b0
}mpl3115a2_mode_t;

/** MPL3115A2 oversample values **/
enum {
  MPL3115A2_CTRL_REG1_OS1 = 0b000,
  MPL3115A2_CTRL_REG1_OS2 = 0b001,
  MPL3115A2_CTRL_REG1_OS4 = 0b010,
  MPL3115A2_CTRL_REG1_OS8 = 0b011,
  MPL3115A2_CTRL_REG1_OS16 = 0b100,
  MPL3115A2_CTRL_REG1_OS32 = 0b101,
  MPL3115A2_CTRL_REG1_OS64 = 0b110,
  MPL3115A2_CTRL_REG1_OS128 = 0b111,
};

typedef union {
	struct{
		uint8_t INT_EN_TCHG : 1;
		uint8_t INT_EN_PCHG : 1;
		uint8_t INT_EN_TTH : 1;
		uint8_t INT_EN_PTH : 1;
		uint8_t INT_EN_TW : 1;
		uint8_t INT_EN_PW : 1;
		uint8_t INT_EN_FIFO : 1;
		uint8_t INT_EN_DRDY : 1;
	}bit;
	uint8_t reg;
}irq_en_param_t;

typedef union {
	struct{
		uint8_t INT_CFG_TCHG : 1;
		uint8_t INT_CFG_PCHG : 1;
		uint8_t INT_CFG_TTH : 1;
		uint8_t INT_CFG_PTH : 1;
		uint8_t INT_CFG_TW : 1;
		uint8_t INT_CFG_PW : 1;
		uint8_t INT_CFG_FIFO : 1;
		uint8_t INT_CFG_DRDY : 1;
	}bit;
	uint8_t reg;
}irq_cfg_param_t;

// All times in ms.
const uint32_t mpl3115a2_bus_timeout_ms = 100;		// one transfer, to issue or to land
const uint32_t mpl3115a2_startup_ms = 100;			// in mpl3115a2_init_state before the first access
const uint32_t mpl3115a2_reset_poll_ms = 10;		// after reset, and between RST polls
const uint32_t mpl3115a2_phase_timeout_ms = 1000;	// start up, or one whole measurement
const uint32_t mpl3115a2_period_ms = 5000;			// between measurements
const uint32_t mpl3115a2_conversion_poll_ms = 100;	// before each look at the status register
const uint32_t mpl3115a2_error_backoff_ms = 100;	// in mpl3115a2_error_state before starting again

// The bus handle and device address are no longer here. They belong
// to the transport and go to its constructor.
typedef struct mpl3115a2_param_t{
	mpl3115a2_mode_t mode;
	uint8_t oversample_ratio;	// one of MPL3115A2_CTRL_REG1_OSx
}mpl3115a2_param_t;

template <typename TTransport>
class mpl3115a2 : public SensorStateMachine<TTransport, mpl3115a2_state_t> {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"mpl3115a2<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, mpl3115a2_state_t> base;
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
	explicit mpl3115a2(const mpl3115a2_param_t &param, TArgs&&... transportArgs);
	virtual ~mpl3115a2() {}

	void main(uint32_t nowMs);
	bool newData();
	float getTemp();		// degrees C. NAN until the first reading
	float getPressure();	// hPa, in barometer mode. NAN in altimeter mode
	float getAltitude();	// m, in altimeter mode. NAN in barometer mode

	// Sets the interrupt enables (ctrl_reg4) and which pin each goes
	// to (ctrl_reg5). Written to the chip during start up, or before
	// the next measurement if the chip is already running.
	void SetIRQ(irq_en_param_t irq_en, irq_cfg_param_t irq_cfg);

private:

	mpl3115a2_param_t _param;
	uint32_t phase_start;	// when start up, or the current measurement, began
	bool _new_data,_EnableIRQ,_irq_changed;

	float _temperature;
	float _pressure;
	float _altitude;

	uint8_t _who_am_i;

	// The transport fills this in after main() has returned, so it
	// has to be a member and not a local.
	uint8_t dataReg[5];

	typedef union{
		struct{
			uint8_t SBYB : 1;
			uint8_t OST : 1;
			uint8_t RST : 1;
			uint8_t OS : 3;
			uint8_t : 1;
			uint8_t ALT : 1;
		}bit;
		uint8_t reg;
	}ctrl_reg1_t;
	ctrl_reg1_t ctrl_reg1;

	// ctrl_reg4 - interrupt enable register
	irq_en_param_t ctrl_reg4;

	// ctrl_reg5 - interrupt configuration register
	irq_cfg_param_t ctrl_reg5;

	typedef union{
			struct{
				uint8_t :1;
				uint8_t TDR :1;
				uint8_t PDR :1;
				uint8_t PTDR : 1;
				uint8_t : 1;
				uint8_t TOW : 1;
				uint8_t POW : 1;
				uint8_t PTOW : 1;
			}bit;
			uint8_t reg;
	}dr_status_reg_t;
	dr_status_reg_t dr_status;

	typedef union{
				struct{
					uint8_t TDEFE : 1;
					uint8_t PDEFE : 1;
					uint8_t DREM : 1;
					uint8_t : 5;
				}bit;
				uint8_t reg;
	}pt_data_cfg_reg_t;
	pt_data_cfg_reg_t pt_data_cfg;

	bool phaseExpired(uint32_t nowMs) const;
	void _getConversion();

};

} /* namespace mpl3115a2 */

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/mpl3115a2.tpp"

#endif /* MPL3115A2_H_ */
