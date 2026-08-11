/*
 * bmp390.cpp
 *
 *  Created on: Dec 4, 2025
 *      Author: coryg
 */

#include "bmp390.h"

#include "cmsis_os.h"

namespace bmp390 {

bmp3_dev _dev;
bmp390 *_selfpointer;

static int8_t i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);

static void delay_ms(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);

bmp390::bmp390(bmp390_param_t param) {
	// TODO Auto-generated constructor stub
	memcpy(&this->_param,&param,sizeof(bmp390_param_t));
	 _param._DevAddress = param._DevAddress<<1;
	_state=bmp390_init_state;
	_temp = NAN; _pressure = NAN;
	_newData=false;
	_dev.intf_ptr=_param._hi2c;
	_dev.chip_id = _param._DevAddress;
	_dev.delay_ms = &delay_ms;
	_dev.intf = BMP3_I2C_INTF;
	_dev.read = &i2c_read;
	_dev.write = &i2c_write;
	_dev.dummy_byte = 0;
}

bmp390::~bmp390() {
	// TODO Auto-generated destructor stub
}


bool bmp390::newData(){
	return _newData;
}

bool bmp390::Reset(){
	if(BMP3_OK==bmp3_soft_reset(&_dev)) return true;
	return false;
}

bool bmp390::ReadStatusReg(){
//	return ReadReg8(&status_reg.reg,bmp390_reg_status_addr);
	return false;
}

bool bmp390::ReadEventReg(){
	return false;
}

bool bmp390::WriteFifoConfig1Reg(){
	return false;
}

bool bmp390::WriteIntCtrlReg(){
	return false;
}

bool bmp390::WriteIfConfReg(){
	return false;
}

bool bmp390::WritePwrCtrlReg(){
	return false;
}

bool bmp390::WriteOsrReg(){
//	osr_reg.reg=_param.
	return false;
}

bool bmp390::WriteOdrReg(){
	return false;
}

bool bmp390::checkDevice(){
	if(HAL_OK==HAL_I2C_IsDeviceReady(_param._hi2c,_param._DevAddress,1,HAL_MAX_DELAY)){
		return true;
	}
	return false;
}

bool ReadTempPressure(){
	return false;
}

void bmp390::main(){
//	int8_t bmp_res;
	switch(_state){
		case bmp390_init_state:
			if(_param._hi2c->State == HAL_I2C_STATE_READY){
				if(checkDevice()){
					if(Reset()){
						last_update=HAL_GetTick();
						_state=bmp390_reset_state;
					}
					else{
						last_update=HAL_GetTick();
						_state=bmp390_err_state;
					}
				}
			}
			break;
		case bmp390_reset_state:
			if(HAL_GetTick()>(last_update+10)){
				_state=bmp390_setup_state;
			}
			break;
		case bmp390_setup_state:
			if(bmp3_init(&_dev)!=BMP3_OK){
				last_update=HAL_GetTick();
				_state=bmp390_err_state;
				break;
			}
			if(validate_trimming_param(&_dev)!=BMP3_OK){
				last_update=HAL_GetTick();
				_state=bmp390_err_state;
				break;
			}
			_dev.settings.odr_filter.temp_os = _param.temp_os;
			_dev.settings.odr_filter.press_os = _param.press_os;
			_dev.settings.odr_filter.odr = _param.odr;
			_dev.settings.odr_filter.iir_filter = _param.iir_filter;
			_dev.settings.op_mode=_param.mode;
			_dev.settings.press_en=BMP3_ENABLE;
			_dev.settings.temp_en=BMP3_ENABLE;


			bmp3_set_op_mode(&_dev);
			last_update=HAL_GetTick();
			switch (_param.mode){
				case pwr_normal:
					//_state=
					break;
				case pwr_forced:
					_state=bmp390_sleep_state;
					break;
				case pwr_sleep:
					_state=bmp390_sleep_state;
					break;
				default:
					break;
			}
			break;
		case bmp390_err_state:
			break;
		default:
			_state=bmp390_init_state;
			break;
	}
}

static int8_t i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
		HAL_I2C_Mem_Write((I2C_HandleTypeDef*)_dev.intf_ptr,_dev.chip_id,reg_addr, 1, reg_data, len, 100);
		return 0;
}

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
	HAL_I2C_Mem_Read((I2C_HandleTypeDef*)_dev.intf_ptr,_dev.chip_id,reg_addr, 1, reg_data, len, 100);
	return 0;
}

static void delay_ms(uint32_t us, void *intf_ptr){
	osDelay(us);
}

static int8_t validate_trimming_param(struct bmp3_dev *dev) {
  int8_t rslt;
  uint8_t crc = 0xFF;
  uint8_t stored_crc;
  uint8_t trim_param[21];
  uint8_t i;

  rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
  if (rslt == BMP3_OK) {
    for (i = 0; i < 21; i++) {
      crc = (uint8_t)cal_crc(crc, trim_param[i]);
    }

    crc = (crc ^ 0xFF);
    rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
    if (stored_crc != crc) {
      rslt = -1;
    }
  }
  return rslt;
}

/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data) {
  int8_t poly = 0x1D;
  int8_t var2;
  uint8_t i;
  for (i = 0; i < 8; i++) {
    if ((seed & 0x80) ^ (data & 0x80)) {
      var2 = 1;
    } else {
      var2 = 0;
    }

    seed = (seed & 0x7F) << 1;
    data = (data & 0x7F) << 1;
    seed = seed ^ (uint8_t)(poly * var2);
  }
  return (int8_t)seed;
}

bool startMeasurement(){
	return false;
}

} /* namespace bmp390 */
