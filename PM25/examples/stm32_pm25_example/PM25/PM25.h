/*
 * PM25.h
 *
 *  Created on: Mar 26, 2024
 *      Author: coryg
 *
 *
 */

#ifndef PMS5003_PM25_PM25_H_
#define PMS5003_PM25_PM25_H_


#include "stm32l4xx_hal.h"
#include <stdint.h>

#if (USE_HAL_UART_REGISTER_CALLBACKS!=1U)
	#error "USE_HAL_UART_REGISTER_CALLBACKS Must be set to 1U in stm32l4xx_hal_conf.h"
#endif

extern "C"{

void pm25_UART_RxCpltCallback(UART_HandleTypeDef *huart); // Interrupt callback for HAL UART

/**! Structure holding Plantower's standard packet **/
typedef struct PMSAQIdata {
  uint16_t framelen;       ///< How long this data chunk is
  uint16_t pm10_standard,  ///< Standard PM1.0
      pm25_standard,       ///< Standard PM2.5
      pm100_standard;      ///< Standard PM10.0
  uint16_t pm10_env,       ///< Environmental PM1.0
      pm25_env,            ///< Environmental PM2.5
      pm100_env;           ///< Environmental PM10.0
  uint16_t particles_03um, ///< 0.3um Particle Count
      particles_05um,      ///< 0.5um Particle Count
      particles_10um,      ///< 1.0um Particle Count
      particles_25um,      ///< 2.5um Particle Count
      particles_50um,      ///< 5.0um Particle Count
      particles_100um;     ///< 10.0um Particle Count
  uint16_t unused;         ///< Unused
  uint16_t checksum;       ///< Packet checksum
} PM25_AQI_Data;

/*
 * Class definition for pms5003 PM25 Air Quality Sensor/Dust Particle Counter
 */
class PM25{

public:
	PM25(UART_HandleTypeDef *_huart);
	bool begin();
	bool getData(PM25_AQI_Data *_data);
	uint8_t rxBuffer[32];// buffer for the UART
	bool newData();
	UART_HandleTypeDef* getPm25Uart(); // for interrupt function to get the UART definition
	PM25_AQI_Data data,*dataPtr;
	bool crc8();
protected:
	UART_HandleTypeDef *huart; // pointer to the UART
    bool _newdata=false;
};

static PM25 *pm25SelfPointer;

}
#endif /* PMS5003_PM25_PM25_H_ */
