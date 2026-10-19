/*
 * PM25.h
 *
 *  Created on: Mar 26, 2024
 *      Author: coryg
 *
 *  PMS5003 PM2.5 air quality sensor / dust particle counter.
 *
 *  The sensor sends a 32 byte frame about once a second, unasked, so
 *  it sits on a stream transport. PM25<TTransport> inherits from
 *  TTransport, which must be an iTransport (itransport/inc/iTransport.h):
 *
 *      PM25<Stm32HalUartTransport>  sensor(&huart1);
 *
 *  The constructor arguments all go to the transport's own
 *  constructor. This file includes no HAL and no RTOS header, and
 *  does not need USE_HAL_UART_REGISTER_CALLBACKS.
 *
 *  The work is in two halves.
 *
 *  onByteReceived() is called by the transport for every byte, from
 *  its interrupt. It finds the start of a frame, collects the 32
 *  bytes and checks the length and checksum. It looks for the start
 *  byte by byte, so it does not matter where in a frame reception
 *  begins, or if a byte goes missing: it lines up again by itself, at
 *  the cost of at most the two frames around the gap.
 *
 *  main(now), called every pass of the loop, takes each good frame
 *  over from the interrupt, makes it available through getData(), and
 *  notices if the sensor has gone quiet. When there is nothing to do
 *  it calls sleep(), an empty stub here. Override sleep() and wake()
 *  for an OS (see xPM25.h) so the thread sleeps until the interrupt
 *  has a frame for it.
 *
 *  The state bookkeeping and sleep() are inherited from
 *  SensorStateMachine (isensor/inc), shared with the other drivers.
 *
 *  Nothing is ever sent to the sensor. It is left in the active mode
 *  it powers up in.
 */

#ifndef PMS5003_PM25_PM25_H_
#define PMS5003_PM25_PM25_H_

#include <stdint.h>
#include <string.h>
#include <type_traits>
#include <utility>
#include "iTransport.h"
#include "SensorStateMachine.h"

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

// PM25 sensor starts each transmission with 0x42 0x4d
const uint8_t pm25_start_1 = 0x42;
const uint8_t pm25_start_2 = 0x4d;
const uint8_t pm25_frame_size = 32;		// the whole frame
const uint16_t pm25_frame_len = 28;		// what the frame's own length field says: the bytes after it

typedef enum pm25_state_t{
	pm25_init_state,		// attach to the transport and start listening
	pm25_listening_state,	// frames arriving
	pm25_error_state		// nothing good for pm25_silence_ms. still listening
}pm25_state_t;

// All times in ms.
const uint32_t pm25_silence_ms = 5000;	// no good frame for this long is an error. the sensor sends about one a second
const uint32_t pm25_retry_ms = 1000;	// in pm25_error_state, between attempts to get the receiver going again

/*
 * Class definition for pms5003 PM25 Air Quality Sensor/Dust Particle Counter
 */
template <typename TTransport>
class PM25 : public SensorStateMachine<TTransport, pm25_state_t>, public iTransportRxSink {
	static_assert(std::is_base_of<iTransport, TTransport>::value,
			"PM25<TTransport>: TTransport must derive from iTransport");
	static_assert(sizeof(PM25_AQI_Data)==30, "PM25_AQI_Data must be 15 values of 2 bytes with no padding");
	typedef SensorStateMachine<TTransport, pm25_state_t> base;
protected:
	// Names from a template base class have to be brought in before
	// they can be used unqualified.
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	template <typename... TArgs>
	explicit PM25(TArgs&&... transportArgs);
	virtual ~PM25() {}

	void main(uint32_t nowMs);
	bool getData(PM25_AQI_Data *_data);	// false until the first good frame
	bool newData();

	// Called by the transport for every byte received. Interrupt context.
	void onByteReceived(uint8_t byte) override;

protected:
	// Called from the interrupt when a good frame is waiting for
	// main(). Empty here: main() finds it the next time round. An OS
	// version wakes the thread that is in sleep().
	virtual void wake() {}

	// True while a good frame is waiting for main(). For an OS
	// version's sleep(), which must not go to sleep with one waiting.
	bool framePending() const { return _frameReady; }

	PM25_AQI_Data data;
	bool _newdata;
	bool _havedata;

private:
	// Interrupt side: the frame being put together.
	uint8_t rxBuffer[pm25_frame_size];
	uint8_t rxCount;

	// Handed from the interrupt to main(). The interrupt writes
	// frame[] only while _frameReady is false, and main() reads it
	// only while it is true, so neither ever sees it half written.
	uint8_t frame[pm25_frame_size];
	volatile bool _frameReady;

	static bool checksumOk(const uint8_t *f);
	void unpack();
};

// Member definitions. Templates have to be visible wherever they
// are used, so they are included here rather than compiled apart.
#include "../src/PM25.tpp"

#endif /* PMS5003_PM25_PM25_H_ */
