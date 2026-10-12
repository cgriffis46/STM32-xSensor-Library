/*
 * teseo.h
 *
 *  ST's Teseo GNSS modules on a UART: the Teseo-VIC3D/VIC3DA (Teseo III
 *  with a 6-axis IMU and dead reckoning; ST's X-NUCLEO-GNSS2A1 board),
 *  and the Teseo-LIV3F, which take the same commands. NMEA for the data,
 *  ST's $PSTM sentences to set it up (Pstm.h). Non-blocking, on
 *  SensorStateMachine like the other drivers; teseo<TTransport> inherits
 *  its transport, an iTransport:
 *
 *      teseo_param_t param(true);              // configure it
 *      teseo<Stm32HalUartTransport> gps(param, &huart6);
 *
 *      for(;;){ gps.main(HAL_GetTick()); ... }
 *      if(gps.newData()){ GnssData d; gps.getData(&d); ... }
 *
 *  Setup, unless param.configure is false (listen only):
 *    1. $PSTMGETSWVER: the firmware version (version()); no answer in
 *       3 tries means the module doesn't hear us: NoAnswer, and the
 *       driver listens anyway.
 *    2. $PSTMGETPAR,1201 and ,1228: NMEA message list 0 (which sentences
 *       come out). If both already hold param.messagesLow/High, nothing
 *       is written: Done.
 *    3. Otherwise $PSTMCFGMSGL,0,1,<low>,<high>, $PSTMSAVEPAR (to the
 *       module's flash), $PSTMSRR (it reboots), the same as ST's own
 *       GetPos application does; then, once NMEA flows again, the two
 *       read back: Done if they now match, Unverified if not (at most
 *       one save per start, so a reply this driver misreads can't wear
 *       the module's flash out in a loop).
 *    Each command waits for its answer (3 tries, teseo_ack_timeout_ms);
 *    an ...ERROR answer sets configStatus() Rejected. While the status is
 *    NoAnswer and NMEA is arriving (a module that was rebooting, say),
 *    the setup runs again every teseo_noanswer_retry_ms.
 *
 *  The settings live in the module's flash, so a module that restarts on
 *  its own keeps them; the driver configures again only when it attaches
 *  (at start, and after the module has been silent for silenceMs).
 *
 *  The baud rate is the module's: 115200 for the Teseo-VIC3DA, 9600 for
 *  the Teseo-LIV3F (ST's X-CUBE-GNSS1 readme). No fix rate, PPS, dead
 *  reckoning or odometer settings yet.
 *
 *  As in mtk3339: the UART interrupt only puts bytes in a ByteRing;
 *  main() parses them with NmeaParser (isensor/nmea), which takes any
 *  talker ($GN.. from a multi-constellation fix). Nothing is valid until
 *  the module says so, and everything goes back to "nothing known" when
 *  it falls silent for silenceMs.
 *
 *  Debug log (ITRANSPORT_DEBUG, tag "teseo"): cfg-send (step, try),
 *  cfg-ok, cfg-error, cfg-noanswer, cfg-match/cfg-differ (CDB, word),
 *  cfg-reset, cfg-done (status), rx-overflow, silent, and at level 3
 *  every sentence.
 *
 *  Not run against a module. The commands and replies are from ST's
 *  X-CUBE-GNSS1 (see Pstm.h); the format of $PSTMSETPAR's value isn't,
 *  so it is read both as hex and as decimal (pstm::valueIs).
 */

#ifndef TESEO_H_
#define TESEO_H_

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <type_traits>
#include <utility>
#include "iTransport.h"
#include "SensorStateMachine.h"
#include "ByteRing.h"
#include "NmeaParser.h"
#include "Pstm.h"

struct teseo_param_t {
	bool     configure;       // false: listen only, send nothing
	uint32_t messagesLow;     // CDB-ID 201: pstm::kMsg* bits
	uint32_t messagesHigh;    // CDB-ID 228
	bool     checksum;        // add "*hh" to commands (ST's library doesn't)
	uint32_t silenceMs;       // nothing good for this long is a failure

	explicit teseo_param_t(bool configureIt)
		: configure(configureIt),
		  messagesLow(pstm::kMsgGGA | pstm::kMsgGSA | pstm::kMsgRMC | pstm::kMsgGSV),
		  messagesHigh(0), checksum(false), silenceMs(3000) {}
};

typedef enum teseo_state_t {
	teseo_init_state,        // attach to the transport
	teseo_cfg_send_state,    // send the next $PSTM command
	teseo_cfg_wait_state,    // wait for its answer
	teseo_boot_wait_state,   // after $PSTMSRR: wait for NMEA again
	teseo_listening_state,   // NMEA arriving
	teseo_error_state        // silent for silenceMs; data invalid; starts again
} teseo_state_t;

enum class teseo_config_status_t : uint8_t {
	NotDone,      // not configured (yet, or configure false)
	Done,         // the message list holds what was asked (read back)
	Rejected,     // an ...ERROR answer: see configStep()
	NoAnswer,     // no answer after teseo_cfg_tries (RX not wired? baud?)
	Unverified    // written and saved, but the read-back doesn't show it
};

// The setup's steps, for configStep() and the debug log.
enum class teseo_step_t : uint8_t {
	Version, ReadLow, ReadHigh, WriteList, Save, Reset, VerifyLow, VerifyHigh, Finished
};

// All times in ms.
const uint32_t teseo_ack_timeout_ms = 1000;
const uint8_t  teseo_cfg_tries = 3;
const uint32_t teseo_write_timeout_ms = 200;
const uint32_t teseo_boot_min_ms = 500;     // after $PSTMSRR, before NMEA counts as "back"
const uint32_t teseo_boot_max_ms = 10000;   // and the most it may take
const uint32_t teseo_retry_ms = 1000;
const uint32_t teseo_noanswer_retry_ms = 30000;   // set up again while it talks but hasn't answered
const uint32_t teseo_max_sleep_ms = 100;

template <typename TTransport>
class teseo : public SensorStateMachine<TTransport, teseo_state_t>, public iTransportRxSink {
	static_assert(std::is_base_of<iTransport, TTransport>::value,
			"teseo<TTransport>: TTransport must derive from iTransport");
	typedef SensorStateMachine<TTransport, teseo_state_t> base;
protected:
	using base::enter;
	using base::fail;
	using base::elapsed;
	using base::sleepRemaining;
	using base::sleep;
	using base::_state;
	using base::last_update;
public:
	static const uint16_t kRxRingSize = 1024;   // power of two; 89 ms at 115200 baud
	static const size_t kVersionLen = 48;

	struct Stats {
		uint32_t rxOverflows;   // bytes lost because main() fell behind
		uint32_t cfgResends;
		uint32_t saves;         // $PSTMSAVEPAR sent (writes to the module's flash)
	};

	template <typename... TArgs>
	explicit teseo(const teseo_param_t& param, TArgs&&... transportArgs);
	virtual ~teseo() {}

	void main(uint32_t nowMs);

	// False until the module has sent a good GGA or RMC (and again after
	// it falls silent). Clears newData().
	bool getData(GnssData* out);
	bool newData() const { return _newData; }
	// The module's own view: it has a fix. 1e-7 degrees.
	bool position(int32_t* latE7, int32_t* lonE7) const;

	teseo_config_status_t configStatus() const { return _configStatus; }
	// The step a Rejected, NoAnswer or Unverified status stopped at.
	teseo_step_t configStep() const { return _step; }
	// $PSTMVER's text ("" until it has answered).
	const char* version() const { return _version; }
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
	enum class Ack : uint8_t { Waiting, Ok, Error };

	void drain(uint32_t nowMs);
	void proprietary();
	bool buildCommand();             // the command for _step into _tx; false: none to send
	void startConfig(uint32_t nowMs);
	void advance(uint32_t nowMs);    // _step done: on to the next
	void finish(teseo_config_status_t s, uint32_t nowMs);
	uint16_t waitCdb() const;

	teseo_param_t _param;
	NmeaParser    _nmea;

	ByteRing<kRxRingSize> _rx;
	std::atomic<bool>     _wakePending;
	uint8_t               _sinceWake;   // interrupt only

	char     _tx[64];                   // iTransport may send from it after write() returns
	size_t   _txLen;
	teseo_step_t _step;
	uint8_t  _tries;
	Ack      _ack;
	bool     _match;                    // the $PSTMSETPAR just read holds what is wanted
	bool     _lowOk, _highOk;
	bool     _savedThisStart;           // since attaching: retries never save twice
	teseo_config_status_t _configStatus;
	char     _version[kVersionLen];
	uint32_t _cfgResends;
	uint32_t _saves;

	uint32_t _lastGoodMs;
	bool     _haveData;
	bool     _newData;
#if ITRANSPORT_DEBUG
	uint32_t _dbgOverflows = 0;
#endif
};

#include "../src/teseo.tpp"

#endif /* TESEO_H_ */
