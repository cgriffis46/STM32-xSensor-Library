/*
 * bmp390.h
 *
 *  Created on: Dec 4, 2025
 *      Author: coryg
 */

#ifndef BMP390_BMP390_H_
#define BMP390_BMP390_H_

#include <stm32l4xx_hal.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <math.h>
#include <cstdint>
#include <iostream>
#include <queue>
#include <cstring>
#include <stdio.h>
#include "bmp3.h"
#include "bmp3_defs.h"

namespace bmp390 {

enum bmp390_state_t{
	bmp390_init_state,
	bmp390_reset_state,
	bmp390_setup_state,
	bmp390_sleep_state,
	bmp390_done_state,
	bmp390_err_state
};

enum bmp390_reg_addr_t{
	bmp390_reg_cmd_addr=0x7E,

	bmp390_reg_NVM_PAR_T1_addr=0x31,
	bmp390_reg_NVM_PAR_T2_addr=0x33,
	bmp390_reg_NVM_PAR_T3_addr=0x35,
	bmp390_reg_NVM_PAR_P1_addr=0x37,
	bmp390_reg_NVM_PAR_P2_addr=0x38,
	bmp390_reg_NVM_PAR_P3_addr=0x3A,
	bmp390_reg_NVM_PAR_P4_addr=0x3B,
	bmp390_reg_NVM_PAR_P5_addr=0x3C,
	bmp390_reg_NVM_PAR_P6_addr=0x3E,
	bmp390_reg_NVM_PAR_P7_addr=0x40,
	bmp390_reg_NVM_PAR_P8_addr=0x41,
	bmp390_reg_NVM_PAR_P9_addr=0x42,
	bmp390_reg_NVM_PAR_P10_addr=0x44,
	bmp390_reg_NVM_PAR_P11_addr=0x45,
	bmp390_reg_config_addr=0x1F,
	bmp390_reg_ODR_addr=0x1D,
	bmp390_reg_OSR_addr=0x1C,
	bmp390_reg_pwr_ctrl_addr=0x1B,
	bmp390_reg_if_conf_addr=0x1A,
	bmp390_reg_int_cntrl_addr=0x19,
	bmp390_reg_fifo_config_2_addr=0x18,
	bmp390_reg_fifo_config_1_addr=0x17,
	bmp390_reg_fifo_wtm_1_addr=0x16,
	bmp390_reg_fifo_wtm_0_addr=0x15,
	bmp390_reg_fifo_data_addr=0x14,
	bmp390_reg_fifo_length_1_addr=0x13,
	bmp390_reg_fifo_length_0_addr=0x12,
	bmp390_reg_int_status_addr=0x11,
	bmp390_reg_event_addr=0x10,
	bmp390_reg_sensortime_2_addr=0x0E,
	bmp390_reg_sensortime_1_addr=0x0D,
	bmp390_reg_sensortime_0_addr=0x0C,
	bmp390_reg_data_5=0x09,
	bmp390_reg_data_4=0x08,
	bmp390_reg_data_3=0x07,
	bmp390_reg_data_2=0x06,
	bmp390_reg_data_1=0x05,
	bmp390_reg_data_0=0x04,
	bmp390_reg_status_addr=0x03,
	bmp390_reg_err_reg_addr=0x02,
	bmp390_reg_rev_id_addr=0x01,
	bmp390_reg_chip_id_addr=0x00
};

enum bmp390_odr_sel_t{
	odr_200 = 0x00,
	odr_100 = 0x01,
	odr_50 = 0x02,
	odr_25 = 0x03,
	odr_12p5 = 0x04,
	odr_6p25 = 0x05,
	odr_3p1 = 0x06,
	odr_1p5 = 0x07,
	odr_0p78 = 0x08,
	odr_0p39 = 0x09,
	odr_0p2 = 0x0A,
	odr_0p1 = 0x0B,
	odr_0p05 = 0x0C,
	odr_0p02 = 0x0D,
	odr_0p01 = 0x0E,
	odr_0p006 = 0x0F,
	odr_0p003 = 0x10,
	odr_0p0015 = 0x11
};

enum bmp390_cmd_t{
	bmp390_nop = 0x00,
	bmp390_fifo_flush = 0xB0,
	bmp390_soft_reset = 0xB6
};

struct BMP390_calib_data{
	float par_t1;
	float par_t2;
	float par_t3;
	float t_lin;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
};

typedef struct {
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	bmp390_odr_sel_t odr;

}bmp390_param_t;

typedef uint8_t bmp390_reg_t;

class bmp390 {
public:
	bmp390(bmp390_param_t param);
	virtual ~bmp390();
	void main();
	bool newData();
protected:
	bmp390_param_t _param;
	bmp390_state_t _state;
	bmp3_dev _dev;
	bool ReadCalData();
	bool Reset();
	bool ReadStatusReg();
	bool ReadEventReg();
	bool WriteFifoConfig1Reg();
	bool WriteIntCtrlReg();
	bool WriteIfConfReg();
	bool WritePwrCtrlReg();
	bool WriteOsrReg();
	bool WriteOdrReg();
	bool checkDevice();
	bool ReadTempPressure();
	uint8_t rev_id;
	bool _newData;
	BMP390_calib_data _calib_data;
	union{
		struct {
			uint8_t fatal_err : 1;
			uint8_t cmd_err : 1;
			uint8_t conf_err : 1;
		}bit;
		bmp390_reg_t reg;
	}err_reg;

	union{
		struct {
			uint8_t : 4;
			uint8_t cmd_rdy : 1;
			uint8_t drdy_press : 1;
			uint8_t drdy_temp : 1;
		}bit;
		bmp390_reg_t reg;
	}status_reg;
	union{
		struct {
			uint8_t por_detected : 1;
			uint8_t itf_act_pt : 1;
		}bit;
		bmp390_reg_t reg;
	}event_reg;
	union{
		struct {
			uint8_t fwm_int : 1;
			uint8_t ffull_int : 1;
			uint8_t : 1;
			uint8_t drdy : 1;
		}bit;
		bmp390_reg_t reg;
	}int_status_reg;
	union{
		struct {
			uint8_t fifo_mode : 1;
			uint8_t fifo_stop_on_full : 1;
			uint8_t fifo_time_en : 1;
			uint8_t fifo_press_en : 1;
			uint8_t fifo_temp_en : 1;
		}bit;
		bmp390_reg_t reg;
	}fifo_config_1_reg;

	union{
		struct{
			uint8_t fifo_subsampling : 3;
			uint8_t data_select : 2;
		}bit;
		bmp390_reg_t reg;
	}fifo_config_2_reg;

	union{
		struct{
			uint8_t int_od : 1;
			uint8_t int_level : 1;
			uint8_t int_latch : 1;
			uint8_t fwtm_en : 1;
			uint8_t ffull_en : 1;
			uint8_t int_ds : 1;
			uint8_t drdy_en : 1;
		}bit;
		bmp390_reg_t reg;
	}int_ctrl_reg;

	union{
		struct{
			uint8_t spi3 : 1 ;
			uint8_t i2c_wdt_en : 1;
			uint8_t i2c_wdt_sel : 1;
		}bit;
		bmp390_reg_t reg;
	}if_conf_reg;

	union{
		struct{
			uint8_t press_en : 1 ;
			uint8_t temp_en : 1;
			uint8_t : 2;
			uint8_t mode : 2;
		}bit;
		bmp390_reg_t reg;
	}pwr_ctrl_reg;

	union {
		struct{
			uint8_t osr_p : 3;
			uint8_t osr4_t : 3;
		}bit;
		bmp390_reg_t reg;
	}osr_reg;

	union {
		struct {
			uint8_t odr_sel : 5;
		}bit;
		bmp390_reg_t reg;
	}odr_reg;

	union{
		struct{
			uint8_t short_in : 1;
			uint8_t iir_filter : 3;
			uint8_t : 4;
		}bit;
		bmp390_reg_t reg;
	}bmp3x_config_reg_t;

	bmp3x_config_reg_t _config;
	uint32_t last_update;
private:
	float temp,pressure;
	int8_t NVM_PAR_P11;
	int8_t NVM_PAR_P10;
	int16_t NVM_PAR_P9;
	int8_t NVM_PAR_P8;
	int8_t NVM_PAR_P7;
	uint16_t NVM_PAR_P6;
	uint16_t NVM_PAR_P5;
	int8_t NVM_PAR_P4;
	int8_t NVM_PAR_P3;
	int16_t NVM_PAR_P2;
	int16_t NVM_PAR_P1;
	int8_t NVM_PAR_T3;
	uint16_t NVM_PAR_T2;
	uint16_t NVM_PAR_T1;
};

} /* namespace bmp390 */

#endif /* BMP390_BMP390_H_ */
