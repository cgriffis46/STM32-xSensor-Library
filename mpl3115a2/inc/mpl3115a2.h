/*
 * mpl3115a2.h
 *
 *  Created on: Dec 29, 2025
 *      Author: coryg
 */

#ifndef MPL3115A2_H_
#define MPL3115A2_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

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

//typedef uint8_t mpl3115a2_reg_value_t;

typedef enum mpl3115a2_state_t{
	mpl3115a2_init_state,
	mpl3115a2_reset_state,
	mpl3115a2_initializing_state,
	mpl3115a2_whoami_state,
	mpl3115a2_initiate_one_shot_measurement,
	mpl3115a2_wait_one_shot_measurement,
	mpl3115a2_done_state,
	mpl3115a2_sleep_state,
	mpl3115a2_error_state
}mpl3115a2_state_t;

typedef enum mpl3115a2_mode_t{
	mpl3115a2_altimeter_mode=0b1,
	mpl3115a2_barometer_mode=0b0
}mpl3115a2_mode_t;

typedef enum mpl4115a2_fifo_mode_t{
	mpl3115a2_fifo_mode_disabled = 0b00,
	mpl3115a2_fifo_mode_ring_buffer = 0b01,
	mpl3115a2_fifo_mode_full_stop = 0b10
}mpl4115a2_fifo_mode_t;

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
	struct irq{
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
	struct irq{
		struct{
							uint8_t INT_CFG_TCHG : 1;
							uint8_t INT_CFG_PCHG : 1;
							uint8_t INT_CFG_TTH : 1;
							uint8_t INT_CFG_PTH : 1;
							uint8_t INT_CFG_TW : 1;
							uint8_t INT_CFG_PW : 1;
							uint8_t INT_CFG_FIFO : 1;
							uint8_t INT_EN_DRDY : 1;
						}bit;
	}bit;
	uint8_t reg;
}irq_cfg_param_t;

typedef struct mpl3115a2_param_t{
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;

	mpl3115a2_mode_t mode;
	uint8_t oversample_ratio;
	uint8_t acquisition_time;

	mpl4115a2_fifo_mode_t fifo_mode;
	uint8_t fifo_watermark;

}mpl3115a2_param_t;

class mpl3115a2 {
public:
	mpl3115a2(mpl3115a2_param_t param);
	virtual ~mpl3115a2();
	void main();
	bool newData();
	float getTemp();
	float getPressure();
	float getAltitude();

	void SetIRQ(irq_en_param_t irq_en, irq_cfg_param_t irq_cfg);

protected:

private:
	//typedef uint8_t mpl3115a2_reg_value_t;

	mpl3115a2_state_t _state;
	mpl3115a2_param_t _param;
	uint32_t last_update;
	bool checkDevice();
	bool who_am_i();
	bool reset();
	bool _readReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t *reg_value);
	bool _writeReg8(mpl3115a2_reg_addr_t reg_addr, uint8_t reg_value);
	bool _new_data,_EnableIRQ;

	float _temperature;
	float _pressure;
	float _altitude;

	uint8_t OUT_P_MSB,OUT_P_CSB,OUT_P_LSB;
	uint8_t OUT_T_MSB,OUT_T_LSB;

	irq_cfg_param_t irq_cfg;

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

	typedef union{
			struct{
				uint8_t ST : 4;
				uint8_t ALARM_SEL : 1;
				uint8_t LOAD_OUTPUT : 1;
				uint8_t : 2;
			}bit;
			uint8_t reg;
	}ctrl_reg2_t;
	ctrl_reg2_t ctrl_reg2;

	typedef union{
			struct{
				uint8_t PP_OD2 : 1;
				uint8_t IPOL2 : 1;
				uint8_t : 2;
				uint8_t PP_OD1 : 1;
				uint8_t IPOL1 : 1;
				uint8_t : 2;
			}bit;
			uint8_t reg;
		}ctrl_reg3_t;
	ctrl_reg3_t ctrl_reg3;

	// ctrl_reg4 - interrupt enable register
	typedef union{
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
	}ctrl_reg4_t;
	ctrl_reg4_t ctrl_reg4;

	// ctrl_reg5 - interrupt configuration register
	typedef union{
				struct{
					uint8_t INT_CFG_TCHG : 1;
					uint8_t INT_CFG_PCHG : 1;
					uint8_t INT_CFG_TTH : 1;
					uint8_t INT_CFG_PTH : 1;
					uint8_t INT_CFG_TW : 1;
					uint8_t INT_CFG_PW : 1;
					uint8_t INT_CFG_FIFO : 1;
					uint8_t INT_EN_DRDY : 1;
				}bit;
				uint8_t reg;
	}ctrl_reg5_t;
	ctrl_reg5_t ctrl_reg5;

	typedef union{
		struct{
			uint8_t SRC_TCHG : 1;
			uint8_t SRC_PCHG : 1;
			uint8_t SRC_TTH : 1;
			uint8_t SRC_PTH : 1;
			uint8_t SRC_TW : 1;
			uint8_t SRC_PW : 1;
			uint8_t SRC_FIFO : 1;
			uint8_t SRC_DRDY : 1;
		}bit;
		uint8_t reg;
	}int_src_reg_t;
	int_src_reg_t int_source;

	typedef union{
		struct{
			uint8_t F_WMRK : 6;
			uint8_t F_MODE : 2;
		}bit;
		uint8_t reg;
	}f_setup_reg_t;
	f_setup_reg_t f_setup;

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

	bool readCtrlReg1();
	bool readCtrlReg2();
	bool readCtrlReg3();
	bool readCtrlReg4();
	bool readCtrlReg5();
	bool readDrStatus();
	bool readIntSource();
	bool readPtDataCfg();

	bool writeCtrlReg1();
	bool writeCtrlReg2();
	bool writeCtrlReg3();
	bool writeCtrlReg4();
	bool writeCtrlReg5();
	bool writePtDataCfg();

	bool setupCtrlReg1();
	bool setupCtrlReg2();
	bool setupCtrlReg3();
	bool setupCtrlReg4();
	bool setupCtrlReg5();
	bool setupPtDataCfg();

	bool setActive();
	bool startMeasurement();
	bool conversionReady();

	bool readTemp();
	bool _getConversion();

};

} /* namespace mpl3115a2 */

#endif /* MPL3115A2_H_ */
