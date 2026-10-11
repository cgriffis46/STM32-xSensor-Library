/*
 * mtk3339.h
 *
 *  A MediaTek MT3339 GNSS module on a UART (Adafruit Ultimate GPS,
 *  GlobalTop PA6H/PA1616S, CDTop modules and the like). NMEA for the
 *  data, MediaTek's PMTK sentences to set it up. Non-blocking, on
 *  SensorStateMachine like the other drivers; mtk3339<TTransport>
 *  inherits its transport, an iTransport:
 *
 *      mtk3339_param_t param(true);   // configure it
 *      param.updateMs = 200;          // 5 Hz
 *      mtk3339<Stm32HalUartTransport> gps(param, &huart1);
 *
 *      for(;;){ gps.main(HAL_GetTick()); ... }
 *      if(gps.newData()){ GnssData d; gps.getData(&d); ... }
 *
 *  Setup, unless param.configure is false (listen only): the sentences
 *  (PMTK314), the output interval (PMTK220) and the fix interval
 *  (PMTK300, never under 200 ms: the chip fixes at most 5 times a
 *  second, and repeats a fix to go faster). Each waits for its
 *  "$PMTK001,cmd,3" (3 tries, ack_timeout each). A flag other than 3,
 *  or no answer, sets configStatus() and the driver carries on with
 *  what the module sends anyway. param.antennaStatus also turns on
 *  GlobalTop's antenna report ("$PGCMD,33,1"; "$PGTOP,11,x" or CDTop's
 *  "$PCD,11,x" come back), read by antenna().
 *
 *  The baud rate is not changed (PMTK251): the module starts at 9600
 *  unless told otherwise, and 9600 carries about 960 characters a
 *  second, so RMC+GGA at 5 Hz, or RMC alone at 10 Hz, is about the
 *  limit there.
 *
 *  "$PMTK010,001" is the module saying it has just started (power on
 *  or reset): without a backup battery its settings are back to the
 *  defaults, so the driver configures it again (restarts() counts
 *  these).
 *
 *  The work is in two halves, as in ublox_gps: the UART interrupt only
 *  puts bytes in a ByteRing; main() parses them with NmeaParser (both in
 *  isensor/nmea). Nothing is valid until the module says so, and
 *  everything goes back to "nothing known" when it falls silent for
 *  silenceMs (the error state, which starts again after
 *  mtk3339_retry_ms).
 *
 *  Debug log (ITRANSPORT_DEBUG, tag "mtk3339"): cfg-send/cfg-ack/
 *  cfg-reject/cfg-noanswer/cfg-done with the command, module-start,
 *  antenna, rx-overflow, silent, and at level 3 every sentence.
 *
 *  Not run against a module: see Pmtk.h for where the commands come
 *  from.
 */

#ifndef MTK3339_H_
#define MTK3339_H_

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <type_traits>
#include <utility>
#include "iTransport.h"
#include "SensorStateMachine.h"
#include "ByteRing.h"
#include "NmeaParser.h"
#include "Pmtk.h"

// One bit per pmtk::Nmea sentence, by its PMTK314 field number.
inline uint32_t mtk3339_sentence_bit(pmtk::Nmea s) { return 1u << (uint8_t)s; }

struct mtk3339_param_t {
	bool     configure;      // false: listen only, send nothing
	uint16_t updateMs;       // ms between outputs: 1000 is 1 Hz, 100 is 10 Hz
	uint32_t sentences;      // mtk3339_sentence_bit()s of the sentences wanted
	bool     antennaStatus;  // ask for $PGTOP antenna reports
	uint32_t silenceMs;      // nothing good for this long is a failure

	explicit mtk3339_param_t(bool configureIt)
		: configure(configureIt), updateMs(1000),
		  sentences(mtk3339_sentence_bit(pmtk::Nmea::RMC) | mtk3339_sentence_bit(pmtk::Nmea::GGA) |
		            mtk3339_sentence_bit(pmtk::Nmea::GSA) | mtk3339_sentence_bit(pmtk::Nmea::GSV)),
		  antennaStatus(false), silenceMs(3000) {}
};

typedef enum mtk3339_state_t {
	mtk3339_init_state,        // attach to the transport
	mtk3339_cfg_send_state,    // send the next PMTK command
	mtk3339_cfg_wait_state,    // wait for its PMTK001
	mtk3339_listening_state,   // NMEA arriving
	mtk3339_error_state        // silent for silenceMs; data invalid; starts again
} mtk3339_state_t;

enum class mtk3339_config_status_t : uint8_t {
	NotDone,    // not configured (yet, or configure false)
	Done,       // every command acknowledged with flag 3
	Rejected,   // a PMTK001 flag other than 3: see configFlag()
	NoAnswer    // no PMTK001 after cfg_tries (module's RX not connected? baud?)
};

enum class mtk3339_antenna_t : uint8_t {
	Unknown,    // no report (antennaStatus off, or not a GlobalTop/CDTop module)
	Shorted,    // the external antenna's feed is shorted
	Internal,   // using the patch antenna on the module
	External    // an external active antenna is connected
};

// All times in ms.
const uint32_t mtk3339_ack_timeout_ms = 1000;   // the MT3339 can take a while
const uint8_t  mtk3339_cfg_tries = 3;
const uint32_t mtk3339_write_timeout_ms = 200;
const uint32_t mtk3339_retry_ms = 1000;
const uint32_t mtk3339_max_sleep_ms = 100;

template <typename TTransport>
class mtk3339 : public SensorStateMachine<TTransport, mtk3339_state_t>, public iTransportRxSink {
	static_assert(std::is_base_of<iTransport, TTransport>::value,
			"mtk3339<TTransport>: TTransport must derive from iTransport");
	typedef SensorStateMachine<TTransport, mtk3339_state_t> base;
protected:
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	static const uint16_t kRxRingSize = 512;   // power of two; 0.5 s at 9600 baud

	struct Stats {
		uint32_t rxOverflows;   // bytes lost because main() fell behind
		uint32_t cfgResends;
		uint32_t restarts;      // $PMTK010,001 seen
	};

	template <typename... TArgs>
	explicit mtk3339(const mtk3339_param_t& param, TArgs&&... transportArgs);
	virtual ~mtk3339() {}

	void main(uint32_t nowMs);

	// False until the module has sent a good GGA or RMC (and again after
	// it falls silent). Clears newData().
	bool getData(GnssData* out);
	// True when a GGA or RMC has arrived since the last getData().
	bool newData() const { return _newData; }
	// The module's own view: it has a fix. 1e-7 degrees. False without one.
	bool position(int32_t* latE7, int32_t* lonE7) const;

	mtk3339_config_status_t configStatus() const { return _configStatus; }
	// The PMTK001 flag that rejected the configuration, and its command.
	pmtk::AckFlag configFlag() const { return _rejectFlag; }
	uint16_t      configCommand() const { return _waitCmd; }
	mtk3339_antenna_t antenna() const { return _antenna; }
	const NmeaParser::Stats& nmeaStats() const { return _nmea.stats(); }
	Stats stats() const;

	// Called by the transport for every byte received. Interrupt context.
	void onByteReceived(uint8_t byte) override;

protected:
	// Called from the interrupt at the end of each line, and every 64
	// bytes. An OS version wakes the thread that is in sleep().
	virtual void wake() {}
	bool wakePending() const { return _wakePending.load(std::memory_order_acquire); }

	void onFail() override;

private:
	void drain(uint32_t nowMs);
	void proprietary(uint32_t nowMs);   // $PMTK001, $PMTK010, $PGTOP, $PCD
	bool nextCommand();                 // builds the next one into _tx; false when done
	void startConfig(uint32_t nowMs);

	mtk3339_param_t _param;
	NmeaParser      _nmea;

	ByteRing<kRxRingSize> _rx;
	std::atomic<bool>     _wakePending;
	uint8_t               _sinceWake;   // interrupt only

	// iTransport may send from these after write() returns. The antenna
	// command has its own, since nothing answers it: the next command is
	// built while it may still be going out.
	char     _tx[64];
	char     _txAntenna[16];
	size_t   _txLen;
	uint8_t  _cfgStep;
	uint8_t  _cfgTries;
	uint16_t _waitCmd;
	enum class Ack : uint8_t { Waiting, Acked, Refused } _ack;
	pmtk::AckFlag _rejectFlag;
	mtk3339_config_status_t _configStatus;
	uint32_t _cfgResends;
	uint32_t _restarts;
	bool     _restartSeen;
	mtk3339_antenna_t _antenna;

	uint32_t _lastGoodMs;
	bool     _haveData;
	bool     _newData;
#if ITRANSPORT_DEBUG
	uint32_t _dbgOverflows = 0;
#endif
};

#include "../src/mtk3339.tpp"

#endif /* MTK3339_H_ */
