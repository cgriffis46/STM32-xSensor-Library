/*
 * DS18B20.cpp
 *
 *  Created on: Feb 1, 2026
 *      Author: coryg
 */

#include <DS18B20_NB.h>

namespace DS18B20_NB {

uint8_t RxByte = 0;
uint8_t ResetByte = 0xF0, PresenceByte;

ds18b20::ds18b20(DS18B20_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(DS18B20_param_t));
	ds18b20_selfptr=this;
	RxByte = 0;
	_newData=false;
	_temp=NAN;
	last_update=HAL_GetTick();
	_state=DS18B20_init_state;
}

ds18b20::~ds18b20() {
	// TODO Auto-generated destructor stub
}

uint8_t ds18b20::DS18B20_Init(void)
{
	cb_state=sample_temp_init;
    HAL_UART_RegisterCallback(_param.huart1,HAL_UART_TX_COMPLETE_CB_ID, ds18b20_UART_SampleTemp_TxCpltCallback);
    HAL_UART_RegisterCallback(_param.huart1,HAL_UART_RX_COMPLETE_CB_ID, ds18b20_UART_SampleTemp_RxCpltCallback);
    LL_USART_SetBaudRate(_param.huart1->Instance, HAL_RCC_GetPCLK2Freq(),LL_USART_OVERSAMPLING_16, 9600);
    // Send reset pulse (0xF0)
    HAL_UART_Transmit_IT(_param.huart1, &ResetByte, 1);
    return 0;
}

uint8_t ds18b20::DS18B20_ReadBit(void)
{
    uint8_t ReadBitCMD = 0xFF;
    uint8_t RxBit;

    // Send Read Bit CMD
    HAL_UART_Transmit(_param.huart1, &ReadBitCMD, 1, 1);
    // Receive The Bit
    HAL_UART_Receive(_param.huart1, &RxBit, 1, 1);

    return (RxBit & 0x01);
}

uint8_t ds18b20::DS18B20_ReadByte(void)
{
    uint8_t RxByte = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        RxByte >>= 1;
        if (DS18B20_ReadBit())
        {
            RxByte |= 0x80;
        }
    }
    return RxByte;
}

void DS18B20_WriteByte(uint8_t data)
{
    uint8_t TxBuffer[8];
    for (int i=0; i<8; i++)
    {
      if ((data & (1<<i)) != 0){
          TxBuffer[i] = 0xFF;
      }
      else{
          TxBuffer[i] = 0;
      }
    }
    HAL_UART_Transmit_IT(ds18b20_selfptr->_param.huart1, TxBuffer, 8);
}

void ds18b20::DS18B20_SampleTemp(void)
{
    DS18B20_Init();

    DS18B20_WriteByte(0xCC);  // Skip ROM   (ROM-CMD)
    DS18B20_WriteByte(0x44);  // Convert T  (F-CMD)
}

float ds18b20::DS18B20_ReadTemp(void)
{
    uint8_t Temp_LSB, Temp_MSB;
    uint16_t Temp;
    float Temperature;

    DS18B20_Init();

    DS18B20_WriteByte(0xCC);  // Skip ROM         (ROM-CMD)
    DS18B20_WriteByte(0xBE);  // Read Scratchpad  (F-CMD)

    Temp_LSB = DS18B20_ReadByte();
    Temp_MSB = DS18B20_ReadByte();
    Temp = ((Temp_MSB<<8))|Temp_LSB;
    Temperature = (float)Temp/16.0;

    _newData=true;
    return Temperature;
}

void ds18b20::main(){
	switch(_state){
	case DS18B20_init_state:
		last_update=HAL_GetTick();
		_state=DS18B20_done_state;
		break;
	case DS18B20_reset_state:
		break;
	case DS18B20_presence_state:
		break;
	case DS18B20_read_temp_state:
		if(HAL_GetTick()>(last_update+1)){
		_temp=DS18B20_ReadTemp();
		last_update=HAL_GetTick();
		_state=DS18B20_done_state;}
		break;
	case DS18B20_done_state:
		if(HAL_GetTick()>(last_update+1000)){
			DS18B20_SampleTemp();
			last_update=HAL_GetTick();
			_state=DS18B20_read_temp_state;}
		break;
	case DS18B20_err_state:
		break;
		_state=DS18B20_init_state;
	default:
		_state=DS18B20_init_state;
		break;
	}
}

bool ds18b20::newData(){
	return _newData;
}

void ds18b20::getTemp(float *temp){
	*temp=_temp;
	_newData=false;
}

void ds18b20_UART_SampleTemp_RxCpltCallback(UART_HandleTypeDef *huart)
{

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */

  switch (cb_state){
   	  case sample_temp_init: // check presence pulse
   		  if (PresenceByte != ResetByte){
   			cb_state=sample_temp_write_1;
   			LL_USART_SetBaudRate(ds18b20_selfptr->_param.huart1->Instance, HAL_RCC_GetPCLK2Freq(),LL_USART_OVERSAMPLING_16, 115200);
   			DS18B20_WriteByte(0xCC);  // Skip ROM   (ROM-CMD)
   		  }
   		  else{
   			ds18b20_selfptr->presence = false; // No presence pulse detected
   		  }
   	  case read_temp_init:
   		  break;
   	  default:
   		  break;
   }

  //HAL_UART_Receive_IT(pm25SelfPointer->getPm25Uart(), pm25SelfPointer->rxBuffer, sizeof(pm25SelfPointer->rxBuffer));
}

void ds18b20_UART_SampleTemp_TxCpltCallback(UART_HandleTypeDef *huart)
{

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  switch (cb_state){
  	  case sample_temp_init:
  		  // Wait for the presence pulse
  		  HAL_UART_Receive_IT(ds18b20_selfptr->_param.huart1, &PresenceByte, 1);
  	  case sample_temp_write_1:
  		cb_state=sample_temp_write_2;
  		DS18B20_WriteByte(0x44);  // Convert T  (F-CMD)
  	  case read_temp_write_1:
  		  break;
  	  case read_temp_write_2:
  		  break;
  	  default:
  		  break;
  }

}


} /* namespace DS18B20 */
