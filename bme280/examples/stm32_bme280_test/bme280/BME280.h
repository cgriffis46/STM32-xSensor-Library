/*
 * BME280.h
 *
 *  Created on: Nov 16, 2025
 *      Author: coryg
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include <stm32l4xx_hal.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

namespace BME280 {

#define bme280_id 0x60

//i2c addresses
//#define bme280_i2c_addr_1
//#define bme280_i2c_addr_2 0x77

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
typedef long signed int bme280_S32_t;
typedef long unsigned int bme280_U32_t;
const bme280_reg8_val_t bme280_reset_value = 0xB6;

typedef struct bme280_reg8_t{
	bme280_reg_addr_t addr;
	bme280_reg8_val_t reg_value;
}bme280_reg8_t;

// defines for register masks
#define bme280_t_sb_mask 0b11100000
#define bme280_filter_mask 0b00011100
#define bme_osrs_t_mask 0b11100000
#define bme_osrs_p_mask 0b00011100
#define bme_spi3w_en_mask 0b00000001
#define bme_status_measuring 0b00001000
#define bme_status_im_update 0b00000001

// pressure oversampling register settings
typedef enum bme280_osrs_p_t{
 bme280_osrs_p_skip = 0b00000000,
 bme280_osrs_p_1x = 0b00000100,
 bme280_osrs_p_2x = 0b00001000,
 bme280_osrs_p_4x = 0b00001100,
 bme280_osrs_p_8x = 0b00010000,
 bme280_osrs_p_16x = 0b00010100
}bme280_osrs_p_t;

// temperature oversampling register settings
typedef enum bme280_osrs_t_t {
 bme280_osrs_t_skip = 0b00000000,
 bme280_osrs_t_1x = 0b00100000,
 bme280_osrs_t_2x = 0b01000000,
 bme280_osrs_t_4x = 0b01100000,
 bme280_osrs_t_8x = 0b10000000,
 bme280_osrs_t_16x = 0b10100000
}bme280_osrs_t_t;


typedef enum bme280_osrs_h_t{
	 bme280_osrs_h_skip = 0b00000000,
	 bme280_osrs_h_1x = 0b00000001,
	 bme280_osrs_h_2x = 0b00000010,
	 bme280_osrs_h_4x = 0b00000011,
	 bme280_osrs_h_8x = 0b00000100,
	 bme280_osrs_h_16x = 0b00000101
}bme280_osrs_h_t;

typedef enum bme280_mode_t{
	bme280_sleep_mode = 0b00000000,
	bme280_forced_mode = 0b00000010,
	bme280_normal_mode = 0b00000011
}bme280_mode_t;

typedef enum bme280_sb_t{
	bme280_sb_05 = 0b00000000,
	bme280_sb_62 = 0b00100000,
	bme280_sb_125 = 0b01000000,
	bme280_sb_250 = 0b01100000,
	bme280_sb_500 = 0b10000000,
	bme280_sb_1000 = 0b10100000,
	bme280_sb_2000 = 0b11000000,
	bme280_sb_4000 = 0b11100000
}bme280_sb_t;

typedef enum bme280_filter_t{
	bme280_filter_off = 0b00000000,
	bme280_filter_x2 = 0b00000100,
	bme280_filter_x4 = 0b00001000,
	bme280_filter_x8 = 0b00001100,
	bme280_filter_x16 = 0b00010000
}bme280_filter_t;

typedef enum bme280_spi_en{
	bme280_spi_en_off = 0b00000000,
	bme280_spi_en_on = 0b00000001
}bme280_spi_en;

typedef enum bme280_err_t{
	bme280_err_busy,
	bme280_err_done,
	bme280_err_not_init,
	bme280_err_bus
}bme280_err_t;

typedef enum bme280_state_t{
	bme280_init,
	bme280_reset,
	bme280_initializing,
	bme280_sleeping,
	bme280_error,
	bme280_start_measurement,
	bme280_measuring,
	bme280_convert_measurement,
	bme280_done
}bme280_state_t;

class bme280 {
public:
	bme280();
	virtual ~bme280();
	virtual bool init();
	void reset();
	void main();
	bool startMeasurement();
	bool readTempPressureHum(double *t, double *p, double *h);

	bool newData();

	bool GetTemperature(double *t);
	bool GetPressure(double *p);
	bool GetHumidity(double *h);

protected:
	bme280_reg8_t temp_xlsb;
	bme280_reg8_t temp_lsb;
	bme280_reg8_t temp_msb;
	bme280_reg8_t press_xlsb;
	bme280_reg8_t press_lsb;
	bme280_reg8_t press_msb;

	bme280_reg8_t config;
	bme280_reg8_t ctrl_meas;
	bme280_reg8_t ctrl_hum;
	bme280_reg8_t status;
	bme280_reg8_t id;
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
	bme280_state_t bme280_state;

	uint32_t last_update;

	bool _newData;

	virtual uint8_t readReg8(bme280_reg8_t *bme280_reg_addr);
	virtual uint8_t writeReg8(bme280_reg8_t *bme280_reg_addr);
	virtual void readRegMulti(bme280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs);
	virtual void writeRegMulti();

	void readConfigReg();
	void readCtrlMeas();
	void readStatusReg();
	bool readIDReg();
	void readCal();
	void readHumCal();

	void setCtrlMeasReg(bme280_osrs_t_t osrs_t_t, bme280_osrs_p_t osrs_p_t, bme280_mode_t mode_t);
	void setConfigReg(bme280_sb_t sb_t, bme280_filter_t filter_t, bme280_spi_en spi_en);
	void setCtrlHumReg(bme280_osrs_h_t osrs_h_t);

	double _temp,_pressure,_humidity;
	bme280_osrs_p_t _osrs_p_t;
	bme280_osrs_t_t _osrs_t_t;
	bme280_osrs_h_t _osrs_h_t;
	bme280_mode_t mode_t;
	bme280_sb_t _sb_t;
	bme280_filter_t _filter_t;
	bme280_spi_en _spi_en;
	bme280_S32_t adc_t, adc_p, adc_h;

private:

	double bme280_compensate_T_double(bme280_S32_t adc_T);
	double bme280_compensate_P_double(bme280_S32_t adc_P);
	double bme280_compensate_H_double(bme280_S32_t adc_H);

};

class bme280i2c : public bme280{
public:
	bme280i2c(I2C_HandleTypeDef *hi2c, bme280_i2c_addr_t dev_addr);
	~bme280i2c();
	bool init();
private:
	I2C_HandleTypeDef *_hi2c;
	uint16_t _DevAddress;
	uint8_t readReg8(bme280_reg8_t *bme280_reg_addr);
	uint8_t writeReg8(bme280_reg8_t *bme280_reg_addr);

	void readRegMulti(bme280_reg_addr_t addr_t,uint8_t *buf,uint8_t numRegs);
	void writeRegMulti();
};

} /* namespace BME280 */

#endif /* BME280_H_ */
