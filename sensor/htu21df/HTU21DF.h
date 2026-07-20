/*
 * HTU21DF.h
 *
 *  Created on: Mar 20, 2024
 *      Author: coryg
 */

#ifndef HTU21DF_HTU21DF_H_
#define HTU21DF_HTU21DF_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/** Default I2C address for the HTU21D. */
static const uint16_t HTU21DF_I2CADDR = (0x40);

/** Read temperature register. */
#define HTU21DF_READTEMP (0xE3)

/** Read humidity register. */
#define HTU21DF_READHUM (0xE5)

/** Write register command. */
#define HTU21DF_WRITEREG (0xE6)

/** Read register command. */
#define HTU21DF_READREG (0xE7)

/** Reset command. */
#define HTU21DF_RESET (0xFE)

typedef enum HTU21DF_state_t{
	htu21df_init_state,
	htu21df_reset_state,
	htu21df_sleep_state,
	htu21df_measure_temp_state,
	htu21df_measure_humidity_state,
	htu21df_read_temp_state,
	htu21df_read_humidity_state,
	htu21df_error_state
}HTU21DF_state_t;

/**
 * Driver for the Adafruit HTU21DF breakout board.
 */
class HTU21DF {
public:
  HTU21DF(I2C_HandleTypeDef* _hi2c,uint8_t devAddress = HTU21DF_I2CADDR );
  bool readTemperature(float *temperature);
  bool readHumidity(float *humidity);
  void reset(void);
  void main();
  void startTempMeasurement();
  void startHumidityMeasurement();
  bool checkDevice();
  bool newData();
private:
  HTU21DF_state_t state;
  //I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  float _humidity, _temp;
  float _last_humidity, _last_temp;
  bool _update_temp;
  uint32_t last_update;
  bool _newData;
  void _startTempMeasurement();
  void _startHumidityMeasurement();

  bool _readTemperature(float *temperature);
  bool _readHumidity(float *humidity);

protected:
  //i2c_inst_t i2c;
  uint8_t _devAddress = HTU21DF_I2CADDR;
  I2C_HandleTypeDef *_hi2c;
};



#endif /* HTU21DF_HTU21DF_H_ */
