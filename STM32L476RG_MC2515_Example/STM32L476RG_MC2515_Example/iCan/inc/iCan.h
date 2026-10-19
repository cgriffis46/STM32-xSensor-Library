/*
 * iCan.h
 *
 *  A CAN controller as the application sees it: classic CAN 2.0B frames
 *  (11 or 29 bit identifiers, 0..8 data bytes, remote frames), a bit rate,
 *  a mode, and the controller's error state. The MCP2515 over SPI is one
 *  implementation (mcp2515<TTransport>); an STM32's own bxCAN or FDCAN, or
 *  Zephyr's CAN API, can be others, and whatever runs above (CANopen,
 *  J1939, the zone controller's own messages) doesn't care which.
 *
 *  As everywhere in this repository, the work is polled: the owner calls
 *  the implementation's main(nowMs), and send()/receive() only queue. The
 *  queues are single producer, single consumer: one thread may call
 *  send() and one (the same or another) receive(), while main() runs in
 *  the driver's own thread.
 *
 *  No CAN FD. No dynamic allocation.
 */

#ifndef ICAN_H_
#define ICAN_H_

#include <stdint.h>
#include <string.h>

namespace can {

static constexpr uint32_t kStdIdMask = 0x7FFu;        // 11 bit identifier
static constexpr uint32_t kExtIdMask = 0x1FFFFFFFu;   // 29 bit identifier

struct Frame {
	uint32_t id;        // 11 bits, or 29 with extended
	bool     extended;  // a 29 bit identifier (CAN 2.0B)
	bool     rtr;       // a remote frame: dlc bytes asked for, none carried
	uint8_t  dlc;       // 0..8 data bytes
	uint8_t  data[8];
	uint32_t ticks;     // received: when (the interrupt's time, or the poll's)
};

inline Frame frame(uint32_t id, bool extended, const uint8_t* data, uint8_t len) {
	Frame f;
	memset(&f, 0, sizeof f);
	f.id = id;
	f.extended = extended;
	f.dlc = len;
	if (data != nullptr && len <= 8) memcpy(f.data, data, len);
	return f;
}

// The identifier fits its kind and dlc is 0..8.
inline bool valid(const Frame& f) {
	return f.dlc <= 8 && (f.id & ~(f.extended ? kExtIdMask : kStdIdMask)) == 0;
}

enum class Mode : uint8_t {
	Normal,      // on the bus: sends, receives and acknowledges
	ListenOnly,  // receives, never drives the bus (no ACK, no error frames)
	Loopback     // its own frames come back to it; nothing goes on the bus
};

struct Config {
	uint32_t bitrate;              // bit/s: 125000, 250000, 500000, 1000000...
	uint16_t samplePointPermille;  // where in the bit it is sampled; 875 (87.5 %) is CiA's usual choice
	Mode     mode;
	// Bus-off: false stays off the bus until start() is called again (the
	// application decides); true starts again after busOffRestartMs.
	bool     autoRestart;
	uint32_t busOffRestartMs;
	// true: frames go on the bus in the order sent (one waits for the one
	// before). false: several can wait in the controller, which sends
	// them in its own order (not necessarily the order sent).
	bool     inOrder;
};

inline Config defaultConfig(uint32_t bitrate) {
	Config c;
	c.bitrate = bitrate;
	c.samplePointPermille = 875;
	c.mode = Mode::Normal;
	c.autoRestart = false;
	c.busOffRestartMs = 1000;
	c.inOrder = true;
	return c;
}

// The controller's state, worst first after Stopped and Fault.
enum class State : uint8_t {
	Stopped,       // not started, or stop()ped; or starting
	Fault,         // the controller doesn't answer or misbehaves: restarting
	BusOff,        // too many transmit errors (TEC > 255): off the bus
	ErrorPassive,  // TEC or REC over 127
	ErrorWarning,  // TEC or REC over 95
	ErrorActive    // all well
};

struct Stats {
	uint32_t txFrames;     // sent (acknowledged)
	uint32_t rxFrames;     // received and queued
	uint32_t rxDropped;    // received, but the receive queue was full
	uint32_t rxOverflows;  // times the controller lost one or more frames (its buffers were full)
	uint32_t busErrors;    // frames that met an error while being sent or received (the MCP2515's MERRF)
	uint32_t busOffs;
	uint32_t restarts;     // configured again after a bus-off or a fault
	uint32_t faults;
};

class iCan {
public:
	virtual ~iCan() {}

	// Configure and go on the bus (again). False if the controller can't
	// make the bit rate and sample point, or the mode isn't supported.
	virtual bool start(const Config& cfg) = 0;
	// Off the bus. Queued frames are dropped.
	virtual void stop() = 0;

	// Queue a frame. False if it isn't valid, the controller isn't on the
	// bus (or listening only), or the queue is full.
	virtual bool send(const Frame& f) = 0;
	// The oldest frame received. False if there is none.
	virtual bool receive(Frame* f) = 0;

	virtual State state() const = 0;
	virtual uint8_t txErrorCount() const = 0;
	virtual uint8_t rxErrorCount() const = 0;
	virtual const Stats& stats() const = 0;
	// Frames queued and not yet sent.
	virtual uint8_t txPending() const = 0;
};

inline const char* stateName(State s) {
	switch (s) {
	case State::Stopped:      return "stopped";
	case State::Fault:        return "fault";
	case State::BusOff:       return "bus-off";
	case State::ErrorPassive: return "error-passive";
	case State::ErrorWarning: return "error-warning";
	case State::ErrorActive:  return "error-active";
	}
	return "?";
}

} // namespace can

#endif /* ICAN_H_ */
