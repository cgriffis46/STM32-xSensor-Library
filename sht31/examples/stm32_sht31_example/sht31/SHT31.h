/**
  ******************************************************************************
  * @file     SHT31.h
  * @author   cgriffis46
  * @version  V1.0
  * @date     18/03/2024 13:49:10
  * @brief    Default under dev library file.
  ******************************************************************************
*/

#ifndef SHT31_H
#define SHT31_H

//#include "main.h"
//#include "stm32f4xx_hal.h"
#include <stm32l4xx_hal.h>

static const uint16_t SHT31_DEFAULT_ADDR = 0x44; /**< SHT31 Default Address */
static const uint16_t SHT31_DEFAULT_ADDR2 = 0x45; /**< SHT31 Default Address */

#define SHT31_MEAS_HIGHREP_STRETCH                                             \
  0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_MEDREP_STRETCH                                              \
  0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH                                              \
  0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP                                                     \
  0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP                                                      \
  0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP                                                      \
  0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */
#define SHT31_REG_HEATER_BIT 0x0d /**< Status Register Heater Bit */

#define SHT31_FETCH_DATA 0xE000
#define SHT31_FETCH_DATA_MSB 0xE0
#define SHT31_FETCH_DATA_LSB 0x00

#define SHT31_BREAK_MSB 0x30
#define SHT31_BREAK_LSB 0x93

#define SHT31_PERIODIC_05mps_HIGHREP_MSB 0x20
#define SHT31_PERIODIC_05mps_HIGHREP_LSB 0x32

#define SHT31_PERIODIC_05mps_MEDREP_MSB 0x20
#define SHT31_PERIODIC_05mps_MEDREP_LSB 0x24

#define SHT31_PERIODIC_05mps_LOWREP_MSB 0x20
#define SHT31_PERIODIC_05mps_LOWREP_LSB 0x2F

#define SHT31_PERIODIC_1mps_HIGHREP_MSB 0x21
#define SHT31_PERIODIC_1mps_HIGHREP_LSB 0x30

#define SHT31_PERIODIC_1mps_MEDREP_MSB 0x21
#define SHT31_PERIODIC_1mps_MEDREP_LSB 0x26

#define SHT31_PERIODIC_1mps_LOWREP_MSB 0x21
#define SHT31_PERIODIC_1mps_LOWREP_LSB 0x2D

#define SHT31_PERIODIC_2mps_HIGHREP_MSB 0x22
#define SHT31_PERIODIC_2mps_HIGHREP_LSB 0x36

#define SHT31_PERIODIC_2mps_MEDREP_MSB 0x22
#define SHT31_PERIODIC_2mps_MEDREP_LSB 0x20

#define SHT31_PERIODIC_2mps_LOWREP_MSB 0x22
#define SHT31_PERIODIC_2mps_LOWREP_LSB 0x2B

#define SHT31_PERIODIC_4mps_HIGHREP_MSB 0x23
#define SHT31_PERIODIC_4mps_HIGHREP_LSB 0x34

#define SHT31_PERIODIC_4mps_MEDREP_MSB 0x23
#define SHT31_PERIODIC_4mps_MEDREP_LSB 0x22

#define SHT31_PERIODIC_4mps_LOWREP_MSB 0x23
#define SHT31_PERIODIC_4mps_LOWREP_LSB 0x29

#define SHT31_PERIODIC_10mps_HIGHREP_MSB 0x27
#define SHT31_PERIODIC_10mps_HIGHREP_LSB 0x37

#define SHT31_PERIODIC_10mps_MEDREP_MSB 0x27
#define SHT31_PERIODIC_10mps_MEDREP_LSB 0x21

#define SHT31_PERIODIC_10mps_LOWREP_MSB 0x27
#define SHT31_PERIODIC_10mps_LOWREP_LSB 0x2A

#define SHT31_CLEARSTATUS_MSB 0x30  /**< Clear Status */
#define SHT31_CLEARSTATUS_LSB 0x41  /**< Clear Status */

typedef enum SHT31_addr_t{
	sht31_i2c_addr1=SHT31_DEFAULT_ADDR,
	sht31_i2c_addr2=SHT31_DEFAULT_ADDR2
}SHT31_addr_t;

typedef enum sht31_state_t{
	sht31_init_state,
	sht31_reset_state,
	sht31_clearstatus_state,
	sht31_set_periodic_mode_state,
	sht31_sleeping_state,
	sht31_done_state,
	sht31_force_measurement_state,
	sht31_measuring_state,
	sht31_error_state
}sht31_state_t;

struct SHT31_Alert_t{
  float SetTemp, ClearTemp;
  float SetHumidity, ClearHumidity;
};

typedef enum {
  C = 0,
  F = 1
}temp_unit;

typedef enum{
  _05mps_high_Res = 0,
  _05_med_Res = 1,
  _05_low_Res = 2,
  _1mps_high_Res = 3,
  _1mps_med_Res = 4,
  _1mps_low_Res = 5,
  _2mps_high_Res = 6,
  _2mps_med_Res = 7,
  _2mps_low_Res = 8,
  _4mps_high_Res = 9,
  _4mps_med_Res = 10,
  _4mps_low_Res = 11,
  _10mps_high_Res = 12,
  _10mps_med_Res = 13,
  _10mps_low_Res = 14,
}SHT31_Sample_Rate_t;

typedef struct SHT31_param_t{
	I2C_HandleTypeDef* hi2c;
	SHT31_addr_t i2c_addr;
	bool periodic_mode;
	bool heater_enable;
	SHT31_Sample_Rate_t sample_rate;
}SHT31_param_t;

typedef struct {
	bool ok = false;
	// error type here
}SHT31_Ret_t;

class SHT31 {
public:
  SHT31(SHT31_param_t sht31_param);
  ~SHT31();

  //bool begin(uint16_t i2caddr = SHT31_DEFAULT_ADDR);
  uint16_t readStatus(void);
  HAL_StatusTypeDef reset(void);
  void heater(bool h);
  bool isHeaterEnabled();
  void SetPeriodicMode(SHT31_Sample_Rate_t mps);
  bool FetchData(float *t, float *h);
  bool GetTemperature(float *t);
  bool GetHumidity(float *h);
  void setHighAlert(const SHT31_Alert_t* alert);
  void setLowAlert(const SHT31_Alert_t* alert);
  void ReadLowAlert(SHT31_Alert_t* alert);
  void ReadHighAlert(SHT31_Alert_t* alert);
  void SendBreak();
  void clearStatus();
  bool HighTempActive();
  bool LowTempActive();
  bool HighHumidityActive();
  bool LowHumidityActive();
  void main();
  bool ForceMeasurement(); // start a forced measurement
  bool newData();
private:
  sht31_state_t state;
  uint32_t last_update;
  bool _newdata;
  bool _force_measurement;
  bool _update_periodic_mode;
  /**
   * Placeholder to track humidity internally.
   */
  float humidity;
  /**
   * Placeholder to track temperature internally.
   */
  float temp;

  bool checkDevice();// check if device exists
  void _SHT31_force_measurement();
  void _setperiodicmode(SHT31_Sample_Rate_t mps);
  HAL_StatusTypeDef writeCommand(uint16_t cmd);
  temp_unit unit = C;
  I2C_HandleTypeDef * _hi2c;

  uint16_t _devAddress;

  bool _PeriodicMode;
  SHT31_Sample_Rate_t _mps;
  SHT31_Alert_t HighAlert;
  SHT31_Alert_t LowAlert;

  bool TempHighAlert, TempLowAlert, HumidityHighAlert, HumidityLowAlert;

};

#endif
