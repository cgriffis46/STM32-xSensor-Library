/*
 * mcp2515.h
 *
 *  The Microchip MCP2515 stand-alone CAN controller, over SPI: an iCan.
 *  Non-blocking, on SensorStateMachine like the sensor and radio drivers.
 *  mcp2515<TTransport> inherits its transport, an iBlockTransport (every
 *  access is an instruction and an address, then data):
 *
 *      mcp2515_param_t p = mcp2515_default_param(8000000);   // the module's crystal
 *      p.intPin = true;                                       // INT wired to an EXTI line
 *      xmcp2515<Stm32HalSpiBlockTransport> can(p, &hspi1, CAN_CS_GPIO_Port, CAN_CS_Pin, spi1Mutex);
 *      // in the EXTI callback (INT's falling edge): can.onInt(HAL_GetTick());
 *
 *      can.start(can::defaultConfig(500000));
 *      for(;;){ can.main(nowMs); }                // in its own thread
 *
 *      can::Frame f = can::frame(0x123, false, data, 8);
 *      can.send(f);                                // from any one thread
 *      while (can.receive(&f)) { ... }             // from any one thread
 *
 *  Startup (after start(), and again after a fault): RESET, kResetMs, then
 *  CANSTAT must say configuration mode and CANCTRL hold its reset value
 *  (Linux's probe: an absent chip reads 0x00 or 0xFF and fails it). Then
 *  CNF3..1 and CANINTE in one write, CANINTF and EFLG cleared, both RX
 *  buffers to take every frame (masks and filters off) with RXB0 rolling
 *  over into RXB1, the RXnBF and TXnRTS pins off; all read back. Last,
 *  CANCTRL asks for the mode (CLKOUT off, one-shot off), and CANSTAT is
 *  read until the chip is in it (kModeTimeoutMs).
 *
 *  Running: on INT (onInt()) or every param.pollMs, CANINTF and EFLG are
 *  read in one transfer, then each full RX buffer is read whole (READ RX
 *  BUFFER, which also clears its flag), RXB0 first (with rollover it is
 *  the older), and the TX, error and overflow flags seen are cleared with
 *  BIT MODIFY, so a flag raised meanwhile is never lost. This repeats until
 *  nothing is pending, which leaves INT high, so an edge-triggered
 *  interrupt sees the next frame. A frame to send is loaded into a free TX
 *  buffer (LOAD TX BUFFER) and sent (RTS): TXB0 only with cfg.inOrder, all
 *  three otherwise.
 *
 *  Never optimistic: every kHealthMs, and after an error interrupt,
 *  CANSTAT, CANCTRL, TEC and REC are read; a chip that is no longer in the
 *  mode it was put in (it reset, or lost power) is a fault and is
 *  configured again. Bus-off takes the chip off the bus: configuration
 *  mode, with ABAT, since a mode change otherwise waits for pending
 *  transmissions. Left alone the MCP2515 would go back on the bus by itself
 *  after 128 x 11 recessive bits (the datasheet's note in section 6.7);
 *  here it goes back only with start(), or after cfg.busOffRestartMs with
 *  cfg.autoRestart. Frames still queued are dropped. stop() does the same.
 *
 *  Two RX buffers are all the chip has. Each frame costs a READ of CANINTF
 *  and EFLG and a READ RX BUFFER (and a BIT MODIFY when TX or error flags
 *  are up), so what keeps up depends on the SPI clock and the thread: in
 *  the simulation, at 10 us a transfer step, 8 byte frames back to back at
 *  1 Mbit/s are all received with INT wired, and a third of 0 byte frames
 *  (one every 47 us) are lost. Losses are counted (stats().rxOverflows
 *  counts the times the chip's overflow flags went up, not frames).
 *
 *  Registers, instructions and timing from Microchip's datasheet DS21801E,
 *  checked against Linux's, Zephyr's and arduino-mcp2515's drivers (see
 *  Mcp2515Regs.h). No acceptance filters, one-shot mode, TX abort, sleep or
 *  wake-up yet. Not run against a chip.
 */

#ifndef MCP2515_H_
#define MCP2515_H_

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <type_traits>
#include <utility>
#include "iBlockTransport.h"
#include "SensorStateMachine.h"
#include "iClock.h"
#include "iCan.h"
#include "Mcp2515Regs.h"

typedef enum mcp2515_state_t {
	mcp2515_stopped = 0,     // not started (or stop()ped, or off after a bus-off): waiting for start()
	mcp2515_init,            // queue RESET
	mcp2515_ops,             // the queued transfers, one each
	mcp2515_wait_ops,
	mcp2515_reset_settle,    // the oscillator starts again
	mcp2515_check_reset,     // CANSTAT and CANCTRL have landed
	mcp2515_check_config,    // the read-back has landed
	mcp2515_mode_poll,       // CANSTAT has landed: in the mode yet?
	mcp2515_running,         // on the bus: service the chip, load frames
	mcp2515_check_flags,     // CANINTF and EFLG have landed
	mcp2515_serviced,        // the RX buffers have landed and the flags are cleared
	mcp2515_check_health,    // CANSTAT, CANCTRL, TEC and REC have landed
	mcp2515_bus_off,         // off the bus: wait for start() or the restart
	mcp2515_error
} mcp2515_state_t;

typedef struct mcp2515_param_t {
	uint32_t oscHz;    // the crystal on the module: 8, 16 or 20 MHz usually
	bool     intPin;   // onInt() is called from INT's falling edge
	uint8_t  pollMs;   // how often the chip is read without an interrupt (also a backstop with one)
} mcp2515_param_t;

inline mcp2515_param_t mcp2515_default_param(uint32_t oscHz) {
	mcp2515_param_t p;
	p.oscHz = oscHz;
	p.intPin = false;
	p.pollMs = 1;
	return p;
}

template <typename TTransport>
class mcp2515 : public SensorStateMachine<TTransport, mcp2515_state_t>, public can::iCan {
	static_assert(std::is_base_of<iBlockTransport, TTransport>::value,
		"mcp2515 needs an iBlockTransport (SpiBlockTransport, ...)");
	using Base = SensorStateMachine<TTransport, mcp2515_state_t>;

public:
	static constexpr uint32_t kBusTimeoutMs = 100;
	static constexpr uint32_t kErrorBackoffMs = 1000;
	static constexpr uint32_t kResetMs = 5;          // after RESET (the datasheet asks 128 OSC1 cycles; Linux waits 5 ms)
	static constexpr uint32_t kResetTimeoutMs = 50;  // configuration mode by then
	static constexpr uint32_t kModeTimeoutMs = 100;  // the requested mode by then
	static constexpr uint32_t kHealthMs = 1000;
	static constexpr uint32_t kIdleSleepMs = 100;    // longest sleep while running or stopped
	static constexpr uint8_t  kRxQueue = 16;
	static constexpr uint8_t  kTxQueue = 8;
	static constexpr uint8_t  kInte = mcp251x::kIntRx0 | mcp251x::kIntRx1 | mcp251x::kIntTx0 | mcp251x::kIntTx1 |
	                                  mcp251x::kIntTx2 | mcp251x::kIntErr | mcp251x::kIntMerr;

	template <typename... TArgs>
	explicit mcp2515(const mcp2515_param_t& param, TArgs&&... transportArgs);

	void main(uint32_t nowMs);

	// iCan. start() and stop() take effect in main(); start() checks the
	// bit timing at once.
	bool start(const can::Config& cfg) override;
	void stop() override;
	bool send(const can::Frame& f) override;
	bool receive(can::Frame* f) override;
	can::State state() const override { return _canState.load(std::memory_order_relaxed); }
	uint8_t txErrorCount() const override { return _tec; }
	uint8_t rxErrorCount() const override { return _rec; }
	const can::Stats& stats() const override { return _st; }
	uint8_t txPending() const override;

	// From INT's falling edge. Safe in an interrupt (iClock::now() is).
	void onInt(uint32_t nowMs) {
		_intTicks = _clock ? _clock->now() : nowMs;
		_intPending.store(true, std::memory_order_release);
		wake();
	}

	void setClock(iClock* clock) { _clock = clock; }
	// The bit timing in use (valid once started).
	const mcp251x::Timing& timing() const { return _timing; }
	mcp2515_state_t driverState() const { return this->_state; }

protected:
	void onFail() override;
	// Called from onInt(), send(), start() and stop(). An OS version wakes the thread.
	virtual void wake() {}
	bool workPending() const {
		return _intPending.load(std::memory_order_acquire) || _req.load(std::memory_order_acquire) != Req::None;
	}

private:
	enum class OpKind : uint8_t { Write, Read };
	struct Op {
		OpKind   kind;
		uint8_t  hdr[4];
		uint8_t  hdrLen;
		uint8_t  v[mcp251x::kFrameLen];   // Write: the data
		uint8_t  len;
		uint8_t* dst;                      // Read: where it lands
	};
	enum class Req : uint8_t { None, Start, Stop };

	void opsClear() { _opCount = 0; _opPos = 0; }
	Op* add(OpKind k);
	void write(uint8_t addr, const uint8_t* v, uint8_t n);
	void write1(uint8_t addr, uint8_t v) { write(addr, &v, 1); }
	void read(uint8_t addr, uint8_t* dst, uint8_t n);
	void bitModify(uint8_t addr, uint8_t mask, uint8_t value);
	void instr(uint8_t instruction);                 // an instruction alone (RESET, RTS)
	void readRx(uint8_t n);
	void loadTx(uint8_t n, const can::Frame& f);
	void runOps(mcp2515_state_t after, uint32_t nowMs);
	bool issueOp(const Op& o);

	void queueConfig();
	void decode(const uint8_t* b, can::Frame* f) const;
	void setState(can::State s) { _canState.store(s, std::memory_order_relaxed); }
	void updateState(uint8_t eflg);
	void serviceFlags(uint32_t nowMs);
	bool loadNext(uint32_t nowMs);
	void goBusOff(uint32_t nowMs);
	bool handleRequest(uint32_t nowMs, bool chipOk);
	void dropTx();
	uint8_t freeTxBuffer() const;   // 0..2, or 3 for none
	uint32_t nowTicks(uint32_t nowMs) const { return _clock ? _clock->now() : nowMs; }

	mcp2515_param_t _param;
	can::Config     _cfg;
	can::Config     _reqCfg;
	mcp251x::Timing _timing = {1, 1, 1, 2, 1};
	std::atomic<Req> _req{Req::None};
	bool _wanted = false;        // on the bus wanted (start() and no stop() since)
	std::atomic<can::State> _canState{can::State::Stopped};

	Op      _ops[12];
	uint8_t _opCount = 0, _opPos = 0;
	mcp2515_state_t _afterOps = mcp2515_running;

	uint32_t _phaseMs = 0;       // when the current wait (reset, mode change, bus-off) began
	uint32_t _lastPollMs = 0;
	uint32_t _lastHealthMs = 0;
	bool     _healthDue = false;
	uint8_t  _expectMode = mcp251x::kOpNormal;

	// What the chip said. Members, as the transport fills them after main() returns.
	uint8_t _stat[2];            // CANSTAT, CANCTRL after reset
	uint8_t _cnfBack[4];         // CNF3, CNF2, CNF1, CANINTE
	uint8_t _rxCtrlBack[2];      // RXB0CTRL, RXB1CTRL
	uint8_t _flags[2];           // CANINTF, EFLG
	uint8_t _health[16];         // CANSTAT (0x0E) .. REC (0x1D)
	uint8_t _rxb[2][mcp251x::kFrameLen];
	uint8_t _rxRead = 0;         // which RX buffers this service read (bits 0, 1)
	uint32_t _serviceTicks = 0;
	uint8_t _tec = 0, _rec = 0;

	// TX buffers in the chip that hold a frame (bits 0..2).
	uint8_t _txBusy = 0;

	// Single producer, single consumer rings.
	can::Frame _rxq[kRxQueue];
	std::atomic<uint8_t> _rxHead{0}, _rxTail{0};   // main() writes head, receive() tail
	can::Frame _txq[kTxQueue];
	std::atomic<uint8_t> _txHead{0}, _txTail{0};   // send() writes head, main() tail

	std::atomic<bool> _intPending{false};
	volatile uint32_t _intTicks = 0;
	iClock* _clock = nullptr;

	can::Stats _st = {};
};

#include "../src/mcp2515.tpp"

#endif /* MCP2515_H_ */
