/*
 * DS18B20.cpp
 *
 *  Created on: Feb 1, 2026
 *      Author: coryg
 */

#include <DS18B20.h>

namespace DS18B20 {

ds18b20::ds18b20(DS18B20_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(DS18B20_param_t));
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
    uint8_t ResetByte = 0xF0, PresenceByte;
    LL_USART_SetBaudRate(_param.huart1->Instance, HAL_RCC_GetPCLK2Freq(),LL_USART_OVERSAMPLING_16, 9600);
    // Send reset pulse (0xF0)
    HAL_UART_Transmit(_param.huart1, &ResetByte, 1, 1);
    // Wait for the presence pulse
    HAL_UART_Receive(_param.huart1, &PresenceByte, 1, 1);
    LL_USART_SetBaudRate(_param.huart1->Instance, HAL_RCC_GetPCLK2Freq(),LL_USART_OVERSAMPLING_16, 115200);
    // Check presence pulse
    if (PresenceByte != ResetByte){
        return 1; // Presence pulse detected
    }
    else{
        return 0; // No presence pulse detected
    }
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

void ds18b20::DS18B20_WriteByte(uint8_t data)
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
    HAL_UART_Transmit(_param.huart1, TxBuffer, 8, 10);
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

enum tx_cb_state_t {
	init_wait,
	write_bit_wait
};

enum rx_cb_state_t{
	read_bit_wait,
	read_bit_done
};

} /* namespace DS18B20 */
