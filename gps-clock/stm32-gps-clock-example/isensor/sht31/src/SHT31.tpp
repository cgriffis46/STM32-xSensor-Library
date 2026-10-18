/*
 * SHT31.tpp
 *
 *  SHT31's member definitions. SHT31.h includes this file at
 *  its end, since templates have to be visible wherever they are
 *  used: include SHT31.h, not this file.
 */

#ifndef SHT31_TPP_
#define SHT31_TPP_

#include "../inc/SHT31.h"

/*!
 * @brief  SHT31 constructor
 * @param  sht31_param settings
 * @param  transportArgs passed on to the transport
 */
template <typename TTransport>
template <typename... TArgs>
SHT31<TTransport>::SHT31(const SHT31_param_t &sht31_param, TArgs&&... transportArgs)
	: base(sht31_init_state, sht31_error_state, sht31_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
  this->_mps=sht31_param.sample_rate;
  _PeriodicMode=sht31_param.periodic_mode;
  _heater=sht31_param.heater_enable;
  humidity = NAN;
  temp = NAN;
  TempHighAlert = false; TempLowAlert = false; HumidityHighAlert = false; HumidityLowAlert = false;
  // No limits until setHighAlert()/setLowAlert(): nothing compares true against NAN.
  HighAlert.SetTemp=NAN; HighAlert.ClearTemp=NAN; HighAlert.SetHumidity=NAN; HighAlert.ClearHumidity=NAN;
  LowAlert=HighAlert;
  _newdata=false;
  _force_measurement=false;
  _update_periodic_mode=false;
  _update_heater=false;
  _update_high_alert=false;
  _update_low_alert=false;
  _break_requested=false;
  _chip_periodic=false;
  _status=0;
}

template <typename TTransport>
void SHT31<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case sht31_init_state:
		if(this->checkDevice()){ // if device exists, go to reset.
			enter(sht31_reset_state,nowMs);
		}
		else { // if device doesn't exist go to error state.
			fail(nowMs);
		}
		break;
	case sht31_reset_state:
		issued(writeCommand(SHT31_SOFTRESET),sht31_wait_reset_state,nowMs);
		break;
	case sht31_wait_reset_state:
		if(landed(nowMs)){
			// The reset leaves the chip idle with the heater off.
			_chip_periodic=false;
			_update_heater=_heater;
			if(!_PeriodicMode){
				_force_measurement=true; // perform a forced measurement so there is a reading from the start.
			}
			enter(sht31_reset_settle_state,nowMs);
		}
		break;
	case sht31_reset_settle_state:
		if(elapsed(nowMs,sht31_reset_ms)){// wait 100ms for device to reset. then begin init.
			enter(sht31_clearstatus_state,nowMs);
		} else {
			sleepRemaining(nowMs,sht31_reset_ms);
		}
		break;
	case sht31_clearstatus_state:
		issued(writeCommand(SHT31_CLEARSTATUS),sht31_wait_clearstatus_state,nowMs); // clear status register
		break;
	case sht31_wait_clearstatus_state:
		if(landed(nowMs)){
			enter(sht31_clearstatus_settle_state,nowMs);
		}
		break;
	case sht31_clearstatus_settle_state:
		if(elapsed(nowMs,sht31_clearstatus_ms)){ // wait for status register to clear
			enter(sht31_status_cmd_state,nowMs);
		} else {
			sleepRemaining(nowMs,sht31_clearstatus_ms);
		}
		break;
	case sht31_status_cmd_state:
		issued(writeCommand(SHT31_READSTATUS),sht31_wait_status_cmd_state,nowMs);
		break;
	case sht31_wait_status_cmd_state:
		if(landed(nowMs)){
			enter(sht31_read_status_state,nowMs);
		}
		break;
	case sht31_read_status_state:
		issued(this->readBytes(dataReg,3),sht31_wait_status_state,nowMs);
		break;
	case sht31_wait_status_state:
		if(landed(nowMs)){
			if(crc8(dataReg,2)!=dataReg[2]){
				fail(nowMs);
			} else {
				_status = dataReg[0];
				_status <<= 8;
				_status |= dataReg[1];
				enter(sht31_done_state,nowMs);
			}
		}
		break;
	case sht31_done_state:
		// The chip is idle. Carry out whatever has been asked for, one
		// thing per pass, then go back to periodic mode if that is wanted.
		_break_requested=false; // already out of periodic mode
		if(_update_heater){
			enter(sht31_heater_state,nowMs);
		} else if(_update_high_alert){
			enter(sht31_high_alert_set_state,nowMs);
		} else if(_update_low_alert){
			enter(sht31_low_alert_set_state,nowMs);
		} else if(_force_measurement){
			enter(sht31_force_measurement_state,nowMs);
		} else if(_PeriodicMode){ // if the sensor should be in periodic mode, set it up.
			enter(sht31_set_periodic_mode_state,nowMs);
		} else {
			sleep(sht31_request_poll_ms);
		}
		break;
	case sht31_sleeping_state: // should be in periodic mode.
		if(requestPending()){ // break from periodic mode before any other command
			enter(sht31_break_state,nowMs);
		}
		else if(elapsed(nowMs,sht31_fetch_period_ms)){
			enter(sht31_fetch_cmd_state,nowMs);
		} else {
			// No longer than sht31_request_poll_ms at a time, so a
			// request is seen without waiting out the whole period.
			uint32_t left=sht31_fetch_period_ms-(nowMs-last_update);
			sleep(left<sht31_request_poll_ms?left:sht31_request_poll_ms);
		}
		break;
	case sht31_break_state:
		issued(writeCommand(SHT31_BREAK),sht31_wait_break_state,nowMs);
		break;
	case sht31_wait_break_state:
		if(landed(nowMs)){
			_chip_periodic=false;
			enter(sht31_break_settle_state,nowMs);
		}
		break;
	case sht31_break_settle_state: // wait 2ms after a break before the next command
		if(elapsed(nowMs,sht31_break_ms)){
			enter(sht31_done_state,nowMs);
		} else {
			sleepRemaining(nowMs,sht31_break_ms);
		}
		break;
	case sht31_heater_state:
		_update_heater=false;
		issued(writeCommand(_heater?SHT31_HEATEREN:SHT31_HEATERDIS),sht31_wait_heater_state,nowMs);
		break;
	case sht31_wait_heater_state:
		if(landed(nowMs)){
			enter(sht31_status_cmd_state,nowMs); // read the status back, so isHeaterEnabled() is the chip's answer
		}
		break;
	case sht31_high_alert_set_state:
		_update_high_alert=false;
		issued(writeAlert(SHT31_WRITE_HIGH_ALERT_SET,HighAlert.SetTemp,HighAlert.SetHumidity),sht31_wait_high_alert_set_state,nowMs);
		break;
	case sht31_wait_high_alert_set_state:
		if(landed(nowMs)){
			enter(sht31_high_alert_clear_state,nowMs);
		}
		break;
	case sht31_high_alert_clear_state:
		issued(writeAlert(SHT31_WRITE_HIGH_ALERT_CLEAR,HighAlert.ClearTemp,HighAlert.ClearHumidity),sht31_wait_high_alert_clear_state,nowMs);
		break;
	case sht31_wait_high_alert_clear_state:
		if(landed(nowMs)){
			enter(sht31_done_state,nowMs);
		}
		break;
	case sht31_low_alert_set_state:
		_update_low_alert=false;
		issued(writeAlert(SHT31_WRITE_LOW_ALERT_SET,LowAlert.SetTemp,LowAlert.SetHumidity),sht31_wait_low_alert_set_state,nowMs);
		break;
	case sht31_wait_low_alert_set_state:
		if(landed(nowMs)){
			enter(sht31_low_alert_clear_state,nowMs);
		}
		break;
	case sht31_low_alert_clear_state:
		issued(writeAlert(SHT31_WRITE_LOW_ALERT_CLEAR,LowAlert.ClearTemp,LowAlert.ClearHumidity),sht31_wait_low_alert_clear_state,nowMs);
		break;
	case sht31_wait_low_alert_clear_state:
		if(landed(nowMs)){
			enter(sht31_done_state,nowMs);
		}
		break;
	case sht31_set_periodic_mode_state:
		_update_periodic_mode=false;
		issued(writeCommand(periodicCommand(_mps)),sht31_wait_set_periodic_mode_state,nowMs);
		break;
	case sht31_wait_set_periodic_mode_state:
		if(landed(nowMs)){
			_chip_periodic=true;
			enter(sht31_sleeping_state,nowMs);
		}
		break;
	case sht31_force_measurement_state:
		_force_measurement=false;
		issued(writeCommand(SHT31_MEAS_HIGHREP),sht31_wait_force_measurement_state,nowMs);
		break;
	case sht31_wait_force_measurement_state:
		if(landed(nowMs)){
			enter(sht31_measuring_state,nowMs);
		}
		break;
	case sht31_measuring_state:
		if(elapsed(nowMs,sht31_conversion_ms)){
			enter(sht31_read_measurement_state,nowMs);
		} else {
			sleepRemaining(nowMs,sht31_conversion_ms);
		}
		break;
	case sht31_read_measurement_state:
		// A single measurement is read straight back. The fetch data
		// command is for periodic mode only.
		issued(this->readBytes(dataReg,6),sht31_wait_measurement_state,nowMs);
		break;
	case sht31_wait_measurement_state:
		if(landed(nowMs)){
			if(convert()){
				enter(sht31_done_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case sht31_fetch_cmd_state:
		issued(writeCommand(SHT31_FETCH_DATA),sht31_wait_fetch_cmd_state,nowMs);
		break;
	case sht31_wait_fetch_cmd_state:
		if(landed(nowMs)){
			enter(sht31_fetch_read_state,nowMs);
		}
		break;
	case sht31_fetch_read_state:
		issued(this->readBytes(dataReg,6),sht31_wait_fetch_state,nowMs);
		break;
	case sht31_wait_fetch_state:
		// The chip does not acknowledge the read when it has nothing
		// new, so an error here only means try again next time. A chip
		// that has gone away fails the fetch command before this.
		if(finished(nowMs)){
			if(this->lastOpFailed()){
				enter(sht31_sleeping_state,nowMs);
			} else if(convert()){
				enter(sht31_sleeping_state,nowMs);
			} else {
				fail(nowMs);
			}
		}
		break;
	case sht31_error_state:
		if(errorCleared(nowMs,sht31_error_backoff_ms)){
			enter(sht31_init_state,nowMs);
		}
		break;
	default:
		enter(sht31_init_state,nowMs);
		break;
	}

}

template <typename TTransport>
bool SHT31<TTransport>::requestPending() const{
	return _force_measurement||_update_heater||_update_high_alert||_update_low_alert
			||_update_periodic_mode||_break_requested;
}

/**
 * Internal function to perform and I2C write.
 *
 * @param cmd   The 16-bit command ID to send.
 */
template <typename TTransport>
bool SHT31<TTransport>::writeCommand(uint16_t command) {

	uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return this->writeBytes(cmd,2); // the transport takes its own copy
}

// One alert limit: the command, the limit packed as the top 7 bits of
// humidity over the top 9 bits of temperature, and its CRC.
template <typename TTransport>
bool SHT31<TTransport>::writeAlert(uint16_t command, float t, float h){
  uint16_t Temp, Humidity;
  uint16_t rht;

  uint8_t cmd[5];

  switch (this->unit) {
    case F:
    {
      Temp = static_cast<uint16_t>(((t+49)/315)*65535.0f);
      break;
    }
    default:
    {
      Temp = static_cast<uint16_t>(((t+45)/175)*65535.0f);
      break;
    }
  }

  Humidity = static_cast<uint16_t>((h*65535.0f)/100);

  rht = (Humidity&0xFE00);
  rht |= (Temp>>7);

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;
  cmd[2] = (uint8_t)(rht>>8);
  cmd[3] = (uint8_t)(rht&0xFF);
  cmd[4] = crc8(&cmd[2],2);
  return this->writeBytes(cmd,5);
}

template <typename TTransport>
uint16_t SHT31<TTransport>::periodicCommand(SHT31_Sample_Rate_t mps) const{
  switch (mps) {
    case _05mps_high_Res: return SHT31_PERIODIC_05mps_HIGHREP;
    case _05_med_Res: return SHT31_PERIODIC_05mps_MEDREP;
    case _05_low_Res: return SHT31_PERIODIC_05mps_LOWREP;
    case _1mps_high_Res: return SHT31_PERIODIC_1mps_HIGHREP;
    case _1mps_med_Res: return SHT31_PERIODIC_1mps_MEDREP;
    case _1mps_low_Res: return SHT31_PERIODIC_1mps_LOWREP;
    case _2mps_high_Res: return SHT31_PERIODIC_2mps_HIGHREP;
    case _2mps_med_Res: return SHT31_PERIODIC_2mps_MEDREP;
    case _2mps_low_Res: return SHT31_PERIODIC_2mps_LOWREP;
    case _4mps_high_Res: return SHT31_PERIODIC_4mps_HIGHREP;
    case _4mps_med_Res: return SHT31_PERIODIC_4mps_MEDREP;
    case _4mps_low_Res: return SHT31_PERIODIC_4mps_LOWREP;
    case _10mps_high_Res: return SHT31_PERIODIC_10mps_HIGHREP;
    case _10mps_med_Res: return SHT31_PERIODIC_10mps_MEDREP;
    case _10mps_low_Res: return SHT31_PERIODIC_10mps_LOWREP;
    default: return SHT31_PERIODIC_1mps_LOWREP;
  }
}

// Turns the 6 bytes in dataReg into temp and humidity. false if either
// CRC is wrong.
template <typename TTransport>
bool SHT31<TTransport>::convert(){
    if (dataReg[2] != crc8(dataReg, 2) ||
      dataReg[5] != crc8(dataReg + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)dataReg[0] << 8) + dataReg[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  this->temp = (float)stemp / 100.0f;
  uint32_t shum = ((uint32_t)dataReg[3] << 8) + dataReg[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  this->humidity = (float)shum / 100.0f;

  if(this->humidity>=this->HighAlert.SetHumidity){this->HumidityHighAlert = true;}
  if(this->humidity<=this->HighAlert.ClearHumidity){this->HumidityHighAlert = false;}
  if(this->temp>=this->HighAlert.SetTemp){this->TempHighAlert = true;}
  if(this->temp<=this->HighAlert.ClearTemp){this->TempHighAlert = false;}

  if(this->humidity<=this->LowAlert.SetHumidity){this->HumidityLowAlert = true;}
  if(this->humidity>=this->LowAlert.ClearHumidity){this->HumidityLowAlert = false;}
  if(this->temp<=this->LowAlert.SetTemp){this->TempLowAlert = true;}
  if(this->temp>=this->LowAlert.ClearTemp){this->TempLowAlert = false;}

  _newdata=true;
  return true;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
template <typename TTransport>
uint8_t SHT31<TTransport>::crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

/**
 * Gets the status register contents as last read from the chip.
 *
 * @return The 16-bit status register.
 */
template <typename TTransport>
uint16_t SHT31<TTransport>::readStatus(void) {
  return _status;
}

/**
 * Enables or disabled the heating element.
 *
 * @param h True to enable the heater, False to disable it.
 */
template <typename TTransport>
void SHT31<TTransport>::heater(bool h) {
  _heater=h;
  _update_heater=true;
}

/*!
 *  @brief  Return sensor heater state
 *  @return heater state (TRUE = enabled, FALSE = disabled)
 */
template <typename TTransport>
bool SHT31<TTransport>::isHeaterEnabled() {
  return ((_status>>SHT31_REG_HEATER_BIT)&0x01)!=0;
}

template <typename TTransport>
void SHT31<TTransport>::setHighAlert(const SHT31_Alert_t* alert){
  this->HighAlert=*alert;
  _update_high_alert=true;
}

template <typename TTransport>
void SHT31<TTransport>::setLowAlert(const SHT31_Alert_t* alert){
  this->LowAlert=*alert;
  _update_low_alert=true;
}

template <typename TTransport>
void SHT31<TTransport>::ReadLowAlert(SHT31_Alert_t* alert){
  *alert=this->LowAlert;
}

template <typename TTransport>
void SHT31<TTransport>::ReadHighAlert(SHT31_Alert_t* alert){
  *alert=this->HighAlert;
}

template <typename TTransport>
bool SHT31<TTransport>::HighTempActive(){
  return TempHighAlert;
}

template <typename TTransport>
bool SHT31<TTransport>::LowTempActive(){
  return TempLowAlert;
}

template <typename TTransport>
bool SHT31<TTransport>::HighHumidityActive(){
  return HumidityHighAlert;
}

template <typename TTransport>
bool SHT31<TTransport>::LowHumidityActive(){
  return HumidityLowAlert;
}

/*
 * The datasheet recommends sending a Break command to the SHT31
 * before issuing commands.
 * Especially if the sensor is in Periodic mode or in a Measurement.
 * main() does that itself before every request. Calling this leaves
 * periodic mode for good, until SetPeriodicMode() is called again.
 */
template <typename TTransport>
void SHT31<TTransport>::SendBreak(){
	_PeriodicMode=false;
	_break_requested=true;
}

/*
 * If the SHT31 is in Periodic mode, a Break is issued first.
 * When the measurement completes the SHT31 goes back to Periodic mode
 * if that is how it is set up, and otherwise waits to be asked again.
 */
template <typename TTransport>
bool SHT31<TTransport>::ForceMeasurement(){
	_force_measurement=true;
	return true;
}

template <typename TTransport>
void SHT31<TTransport>::SetPeriodicMode(SHT31_Sample_Rate_t mps){
	this->_mps=mps;
	_PeriodicMode=true;
	_update_periodic_mode=true;
}

template <typename TTransport>
bool SHT31<TTransport>::GetTemperature(float *t){
	bool old = _newdata;
	_newdata=false;
	*t=this->temp;
	return old;
}

template <typename TTransport>
bool SHT31<TTransport>::GetHumidity(float *h){
	bool old = _newdata;
	_newdata=false;
	*h=this->humidity;
	return old;
}

template <typename TTransport>
bool SHT31<TTransport>::newData(){
	return _newdata;
}

#endif /* SHT31_TPP_ */
