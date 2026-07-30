/*
 * PM25.cpp
 *
 *  Created on: Mar 26, 2024
 *      Author: coryg
 */
#include "PM25.h"
//#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
extern UART_HandleTypeDef huart2;

extern "C"{

PM25::PM25(UART_HandleTypeDef *_huart) {
	huart=_huart;
	pm25SelfPointer = this;
	_newdata=false;
	dataPtr=&data;}

bool PM25::getData(PM25_AQI_Data *_data){
		memcpy((void*)_data,dataPtr, sizeof(data));
		_newdata=false;
		return true;
}

UART_HandleTypeDef* PM25::getPm25Uart(){
	return this->huart;
}

bool PM25::newData(){
	return _newdata;
}

bool PM25::crc8(){
	  uint16_t sum = 0;
	  uint16_t csum = 0;
	  // check CRC
	  for (uint8_t i = 0; i < 30; i++) {
	        csum += pm25SelfPointer->rxBuffer[i];
	      }
	  // The data comes in endian'd, this solves it so it works on all platforms
	      uint16_t buffer_u16[15];
	      for (uint8_t i = 0; i < 15; i++) {
	        buffer_u16[i] = pm25SelfPointer->rxBuffer[2 + i * 2 + 1];
	        buffer_u16[i] += (pm25SelfPointer->rxBuffer[2 + i * 2] << 8);
	      }
	      // put it into a nice struct :)
	      memcpy((void *)pm25SelfPointer->dataPtr, (void *)buffer_u16, 32);

		 if(csum==buffer_u16[14]) {
			 _newdata=true;
			 return true;
		 }
		 return false;
}

uint8_t charbuffer[255];

void pm25_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  // PM25 sensor starts each transmission with 0x42 0x4d
  if(huart==pm25SelfPointer->getPm25Uart()&&pm25SelfPointer->rxBuffer[0]==0x42&&pm25SelfPointer->rxBuffer[1]==0x4d){
     pm25SelfPointer->crc8();
  }
  HAL_UART_Receive_IT(pm25SelfPointer->getPm25Uart(), pm25SelfPointer->rxBuffer, sizeof(pm25SelfPointer->rxBuffer));
}

bool PM25::begin(){
	_newdata=false;

	HAL_UART_RegisterCallback(this->huart,HAL_UART_RX_COMPLETE_CB_ID, pm25_UART_RxCpltCallback);
	if(HAL_UART_Receive_IT(this->huart, this->rxBuffer, sizeof(rxBuffer))==HAL_OK) return true;
	return false;
}

}
