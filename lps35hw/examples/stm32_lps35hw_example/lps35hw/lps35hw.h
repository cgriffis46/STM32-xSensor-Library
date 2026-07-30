/*
 * lps35hw.h
 *
 *  Created on: Dec 31, 2025
 *      Author: coryg
 */

#ifndef LPS35HW_H_
#define LPS35HW_H_

#include <stm32l4xx_hal.h>
//#include <stm32l4xx_hal_i2c.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
//#include <ranges>
//#include <bit>
#include <cstdint>
#include <iostream>
//#include <mutex>
#include <queue>

namespace LPS35HW {

using namespace std;

#define LPS35HW_WHO_AM_I 0xB1
#define LPS35HW_ADDR 0x5D
// B8 = 10111000
// 5C = 01011100

typedef enum lps35hw_state_t{
	lps35hw_init_state,
	lps35hw_reset_state,
	lps35hw_initializing_state,
	lps35hw_sleep_state,
	lps35hw_done_state,
	lps35hw_wait_forced_measurement,
	lps35hw_err_state
};

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
};

// data types for ctrl_reg1

// lps35hw Output Data Rate
typedef enum lps35hw_odr_t{
	lps35hw_odr_0hz = 0b000,
	lps35hw_odr_1hz = 0b001,
	lps35hw_odr_10hz = 0b010,
	lps35hw_odr_25hz = 0b011,
	lps35hw_odr_50hz = 0b100,
	lps35hw_odr_75hz = 0b101
};

typedef bool lps35hw_en_lpfp_t;
typedef bool lps35hw_lpfp_cfg_t;
typedef bool lps35hw_bdu_t;
typedef bool lps35hw_spi_mode_t;

// data types for ctrl_reg2
//typedef bool lps35hw_boot;
//typedef bool lps35hw_fifo_en;
//typedef bool lps35hw_stop_on_fth;
//typedef bool lps35hw_if_add_inc;
//typedef bool lps35hw_i2c_dis;
//typedef bool lps35hw_swreset;
//typedef bool lps35hw_one_shot;

// data types for ctrl_reg3

// data types for fifo_ctrl
enum fifo_mode_t{
	fifo_bypass_mode = 0b000,
	fifo_mode = 0b001,
	stream_mode = 0b010,
	stream_to_fifo_mode = 0b011,
	bypass_to_stream_mode = 0b100,
	dynamic_stream_mode = 0b110,
	bypass_to_fifo_mode = 0b111
};

typedef uint8_t lps35hw_reg_value;

struct lps35hw_fifo_entry{uint8_t entry[5];};

//lps35hw_fifo_entry _fifo[32];

typedef struct lps35hw_param_t{
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	//HAL_LockTypeDef *i2c_mutex;

	lps35hw_odr_t odr; // data rate
	lps35hw_en_lpfp_t en_lpfp; // enable lowpass filter
	lps35hw_lpfp_cfg_t lpfp_cfg; //
	//lps35hw_bdu_t bdu; // block data update
	lps35hw_spi_mode_t spi_mode; // 3-wire or 4-wire mode

	bool int_h_l;
	bool pp_od;
	bool f_fss5;
	bool f_fth;
	bool f_ovr;
	bool drdy;
	bool int_s2;
	bool int_s1;

	bool fifo_en;
	bool stop_on_fth;
	bool if_add_inc;

	bool autorifp;

	fifo_mode_t fifo_mode;
	uint8_t fifo_threshold;
}lps35hw_param_t;

class lps35hw {
public:
	lps35hw();
	virtual ~lps35hw();
	void main();
	void startMeasurement();
	void tempPressure(float *t,float *p);
	bool newData();
protected:
	lps35hw_state_t _state;
	lps35hw_param_t _param;
	bool checkDevice();
	bool whoAmI();
	void reset();
	virtual uint8_t readReg8(lps35hw_reg_addr_t RegAddress);
	virtual void writeReg8(lps35hw_reg_addr_t RegAddress, uint8_t regValue);
	//uint8_t readStatus();
	void read_ctrl_reg1();
	void write_ctrl_reg1();
	void read_ctrl_reg2();
	virtual void write_ctrl_reg2();
	void write_ctrl_reg3();
	void readIntSource();
	uint8_t readFifoStatus();
	void write_interrupt_cfg();
	void write_fifo_ctrl();
	void readMeasurement();
	uint32_t last_update;
	bool boot();
	void _startMeasurement();
	bool fifoEnabled(); // check if fifo is enabled
	void queueData(lps35hw_fifo_entry *_TempPressure);
	void readFifo();
	bool intActive;
	bool _force_measurement;
	float _temp,_pressure;
	bool _newData;
	std::queue<lps35hw_fifo_entry> _lps35hw_fifo;

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
		struct ctrl_reg3_value_t{
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
			lps35hw_reg_value fifo_mode : 3;
			lps35hw_reg_value watermark : 5;
		}bit;
		uint8_t reg;
	}fifo_ctrl_value_t;
	fifo_ctrl_value_t _fifo_ctrl;

	typedef union{
		struct{
			uint8_t p_da:1;
			uint8_t t_da:1;
			uint8_t p_or:1;
			uint8_t t_or:1;
			uint8_t :2;
			uint8_t :2;
		}bit;
		uint8_t reg;
	}status_reg_t;

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

	typedef union{
		struct{
			uint8_t phe : 1;
			uint8_t ple : 1;
			uint8_t lir : 1;
			uint8_t diff_en : 1;
			uint8_t reset_az : 1;
			uint8_t autozero : 1;
			uint8_t reset_arp : 1;
			uint8_t autorifp : 1;
		}bit;
		uint8_t reg;
	}interrupt_cfg;
	interrupt_cfg _interrupt_cfg;

private:

};

class lps35hw_i2c: public lps35hw{
public:
	lps35hw_i2c(lps35hw_param_t param);
	~lps35hw_i2c();
protected:
	uint8_t readReg8(lps35hw_reg_addr_t RegAddress);
	void writeReg8(lps35hw_reg_addr_t RegAddress,uint8_t regValue);
	void write_ctrl_reg2();
private:
};

class lps35hw_spi : public lps35hw{
public:
	lps35hw_spi(lps35hw_param_t param);
	~lps35hw_spi();
protected:
	uint8_t readReg8(lps35hw_reg_addr_t RegAddress);
	void writeReg8(lps35hw_reg_addr_t RegAddress,uint8_t regValue);
	void write_ctrl_reg2();
private:

};

} /* namespace LPS35HW */

#endif /* LPS35HW_H_ */
