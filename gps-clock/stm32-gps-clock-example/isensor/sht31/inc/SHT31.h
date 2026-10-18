/**
  ******************************************************************************
  * @file     SHT31.h
  * @author   cgriffis46
  * @version  V1.0
  * @date     18/03/2024 13:49:10
  * @brief    Default under dev library file.
  ******************************************************************************
  *
  *  SHT31 temperature / humidity sensor, non-blocking state machine.
  *
  *  The bus is chosen by the template argument. SHT31<TTransport>
  *  inherits from TTransport, which must be an ISensorTransport
  *  (itransport/inc/ISensorTransport.h):
  *
  *      SHT31<Stm32HalI2CTransport>  sensor(param, &hi2c1, sht31_i2c_addr1, i2c1Mutex);
  *
  *  Everything after param goes to the transport's own constructor.
  *  This file includes no HAL and no RTOS header.
  *
  *  The SHT31 has no registers. It takes 16 bit commands and is read
  *  back as a plain run of bytes, so this driver uses the transport's
  *  writeBytes() and readBytes().
  *
  *  Call main(now) every pass of the loop. It never waits on the bus:
  *  each command and each read is one state that issues it and one
  *  state that waits for it to land. When a state has nothing to do it
  *  calls sleep(), an empty stub here. Override sleep() for an OS (see
  *  xSHT31.h) to give up the CPU for that long.
  *
  *  Nothing here talks to the chip directly any more. heater(),
  *  setHighAlert(), setLowAlert(), SetPeriodicMode(), SendBreak() and
  *  ForceMeasurement() each record a request and return at once; main()
  *  carries the requests out between measurements. If the chip is in
  *  periodic mode it is stopped first, as the datasheet asks, and
  *  started again afterwards.
  *
  *  The state bookkeeping, sleep() and the issue/wait helpers are
  *  inherited from SensorStateMachine (isensor/inc), shared with the
  *  other drivers.
*/

#ifndef SHT31_H
#define SHT31_H

#include <stdint.h>
#include <math.h>
#include <type_traits>
#include <utility>
#include "ISensorTransport.h"
#include "SensorStateMachine.h"
#include "Thermometer.h"
#include "Hygrometer.h"

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
#define SHT31_BREAK 0x3093

#define SHT31_PERIODIC_05mps_HIGHREP 0x2032
#define SHT31_PERIODIC_05mps_MEDREP 0x2024
#define SHT31_PERIODIC_05mps_LOWREP 0x202F
#define SHT31_PERIODIC_1mps_HIGHREP 0x2130
#define SHT31_PERIODIC_1mps_MEDREP 0x2126
#define SHT31_PERIODIC_1mps_LOWREP 0x212D
#define SHT31_PERIODIC_2mps_HIGHREP 0x2236
#define SHT31_PERIODIC_2mps_MEDREP 0x2220
#define SHT31_PERIODIC_2mps_LOWREP 0x222B
#define SHT31_PERIODIC_4mps_HIGHREP 0x2334
#define SHT31_PERIODIC_4mps_MEDREP 0x2322
#define SHT31_PERIODIC_4mps_LOWREP 0x2329
#define SHT31_PERIODIC_10mps_HIGHREP 0x2737
#define SHT31_PERIODIC_10mps_MEDREP 0x2721
#define SHT31_PERIODIC_10mps_LOWREP 0x272A

#define SHT31_WRITE_HIGH_ALERT_SET 0x611D
#define SHT31_WRITE_HIGH_ALERT_CLEAR 0x6116
#define SHT31_WRITE_LOW_ALERT_CLEAR 0x610B
#define SHT31_WRITE_LOW_ALERT_SET 0x6100

typedef enum SHT31_addr_t{
	sht31_i2c_addr1=SHT31_DEFAULT_ADDR,
	sht31_i2c_addr2=SHT31_DEFAULT_ADDR2
}SHT31_addr_t;

// Every bus access is two states: one that issues it and a wait
// state that it lands in.
typedef enum sht31_state_t{
	sht31_init_state,					// check a device answers
	sht31_reset_state,					// issue the soft reset
	sht31_wait_reset_state,
	sht31_reset_settle_state,			// pause while the reset takes effect
	sht31_clearstatus_state,			// issue the clear status command
	sht31_wait_clearstatus_state,
	sht31_clearstatus_settle_state,		// pause while the status register clears
	sht31_status_cmd_state,				// issue the read status command
	sht31_wait_status_cmd_state,
	sht31_read_status_state,			// issue the 3 byte read
	sht31_wait_status_state,
	sht31_done_state,					// idle, chip measuring only when asked. requests are carried out here
	sht31_sleeping_state,				// idle, chip in periodic mode
	sht31_break_state,					// issue the break that stops periodic mode
	sht31_wait_break_state,
	sht31_break_settle_state,			// pause before the next command
	sht31_heater_state,					// issue heater on or off
	sht31_wait_heater_state,
	sht31_high_alert_set_state,
	sht31_wait_high_alert_set_state,
	sht31_high_alert_clear_state,
	sht31_wait_high_alert_clear_state,
	sht31_low_alert_set_state,
	sht31_wait_low_alert_set_state,
	sht31_low_alert_clear_state,
	sht31_wait_low_alert_clear_state,
	sht31_set_periodic_mode_state,		// issue the periodic mode command
	sht31_wait_set_periodic_mode_state,
	sht31_force_measurement_state,		// issue a single measurement
	sht31_wait_force_measurement_state,
	sht31_measuring_state,				// conversion time
	sht31_read_measurement_state,		// issue the 6 byte read
	sht31_wait_measurement_state,
	sht31_fetch_cmd_state,				// periodic mode: issue the fetch data command
	sht31_wait_fetch_cmd_state,
	sht31_fetch_read_state,				// issue the 6 byte read
	sht31_wait_fetch_state,
	sht31_error_state					// pause, then start again from sht31_init_state
}sht31_state_t;

// All times in ms.
const uint32_t sht31_bus_timeout_ms = 100;		// one bus operation, to issue or to land
const uint32_t sht31_reset_ms = 100;			// after the soft reset
const uint32_t sht31_clearstatus_ms = 100;		// after clearing the status register
const uint32_t sht31_break_ms = 2;				// after a break, before the next command
const uint32_t sht31_conversion_ms = 20;		// a single high repeatability measurement takes 15.5 at most
const uint32_t sht31_fetch_period_ms = 2000;	// between reads in periodic mode
const uint32_t sht31_request_poll_ms = 100;		// longest sleep while a request could arrive
const uint32_t sht31_error_backoff_ms = 100;	// in sht31_error_state before starting again

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

// The bus handle and device address are no longer here. They belong
// to the transport and go to its constructor.
typedef struct SHT31_param_t{
	bool periodic_mode;
	bool heater_enable;
	SHT31_Sample_Rate_t sample_rate;
}SHT31_param_t;

template <typename TTransport>
class SHT31 : public SensorStateMachine<TTransport, sht31_state_t>, public Thermometer, public Hygrometer {
	static_assert(std::is_base_of<ISensorTransport, TTransport>::value,
			"SHT31<TTransport>: TTransport must derive from ISensorTransport");
	typedef SensorStateMachine<TTransport, sht31_state_t> base;
protected:
	// Names from a template base class have to be brought in before
	// they can be used unqualified.
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::issued;
	using base::landed;
	using base::finished;
	using base::errorCleared;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
  template <typename... TArgs>
  explicit SHT31(const SHT31_param_t &sht31_param, TArgs&&... transportArgs);
  virtual ~SHT31() {}

  void main(uint32_t nowMs);
  bool newData();
  bool GetTemperature(float *t);	// degrees C. returns whether the value is new
  bool GetHumidity(float *h);		// %RH. returns whether the value is new

  // The same readings through isensor's interfaces (isensor/inc). NAN
  // until the first reading; they leave newData() alone.
  float getTemperature() const override { return temp; }		// degrees C (whatever the alert unit)
  float getHumidity() const override { return humidity; }		// %RH

  // Each of these records a request and returns at once. main()
  // carries it out the next time the driver is between measurements.
  bool ForceMeasurement(); // start a forced measurement
  void heater(bool h);
  void SetPeriodicMode(SHT31_Sample_Rate_t mps);	// switch to periodic mode at this rate
  void SendBreak();									// leave periodic mode
  void setHighAlert(const SHT31_Alert_t* alert);
  void setLowAlert(const SHT31_Alert_t* alert);

  uint16_t readStatus(void);	// the status register as last read: at start up, and after each heater change
  bool isHeaterEnabled();		// from that same status register
  void ReadLowAlert(SHT31_Alert_t* alert);
  void ReadHighAlert(SHT31_Alert_t* alert);
  bool HighTempActive();
  bool LowTempActive();
  bool HighHumidityActive();
  bool LowHumidityActive();
private:
  bool _newdata;
  bool _force_measurement;
  bool _update_periodic_mode;
  bool _update_heater;
  bool _update_high_alert;
  bool _update_low_alert;
  bool _break_requested;
  bool _heater;
  bool _chip_periodic;		// the chip itself is in periodic mode right now
  /**
   * Placeholder to track humidity internally.
   */
  float humidity;
  /**
   * Placeholder to track temperature internally.
   */
  float temp;

  temp_unit unit = C;

  bool _PeriodicMode;		// periodic mode is wanted
  SHT31_Sample_Rate_t _mps;
  SHT31_Alert_t HighAlert;
  SHT31_Alert_t LowAlert;

  bool TempHighAlert, TempLowAlert, HumidityHighAlert, HumidityLowAlert;

  uint16_t _status;

  // The transport fills this in after main() has returned, so it
  // has to be a member and not a local.
  uint8_t dataReg[6];

  bool requestPending() const;
  bool writeCommand(uint16_t cmd);
  bool writeAlert(uint16_t cmd, float t, float h);
  uint16_t periodicCommand(SHT31_Sample_Rate_t mps) const;
  bool convert();
  static uint8_t crc8(const uint8_t *data, int len);
};

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/SHT31.tpp"

#endif
