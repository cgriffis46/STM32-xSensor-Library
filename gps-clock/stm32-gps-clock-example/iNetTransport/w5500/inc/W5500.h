/*
 * W5500.h
 *
 *  WIZnet W5500 Ethernet controller, non-blocking state machine.
 *
 *  The bus is chosen by the template argument, as for the sensor
 *  drivers. w5500<TTransport> inherits from TTransport, which must be
 *  an iBlockTransport (itransport/inc/iBlockTransport.h):
 *
 *      w5500<Stm32HalSpiBlockTransport> chip(param, &hspi1, W5500_CS_GPIO_Port, W5500_CS_Pin, spi1Mutex);
 *
 *  Everything after param goes to the transport's own constructor.
 *  This file includes no HAL and no RTOS header.
 *
 *  It is an iEthernetDevice (iNetTransport/inc/iNetDevice.h): hand it to an
 *  xEthernet, whose driver thread calls poll() and relays your
 *  threads' connect()/read()/write() to it. It can also be driven
 *  from a bare-metal loop by anything that implements iNetDeviceHost.
 *
 *  poll(now) never waits on the bus. Each register access is one
 *  state that issues the transfer and one that waits for it to land
 *  (xfer_issue / xfer_wait, shared by every access). It returns how
 *  long it can be left — the driver thread sleeps on its queue for
 *  that long, rather than in an osDelay(), so a request from a user
 *  thread or the INT pin wakes it at once.
 *
 *  TCP sockets for the interface; a static address or DHCP. The W5500
 *  has 8 sockets, each one connection (a listening socket becomes the
 *  connection when a peer arrives). With w5500_param_t::serviceSocket
 *  set (the default) the last one, socket 7, is kept back as a UDP
 *  socket on port 68 for DHCP, DNS and SNTP (iNetTransport/dhcp,
 *  dns, sntp: pure-logic clients this driver runs), told apart by the
 *  peer's port (67, 53, 123), and the interface gets sockets 0..6.
 */

#ifndef W5500_H_
#define W5500_H_

#include <stddef.h>
#include <stdint.h>
#include <atomic>
#include <type_traits>
#include <utility>
#include "iBlockTransport.h"
#include "iNetDevice.h"
#include "W5500Regs.h"
#include "DhcpClient.h"
#include "DnsClient.h"
#include "SntpClient.h"

namespace W5500 {

// All times in ms.
const uint32_t w5500_bus_timeout_ms		= 100;		// one transfer, to issue or to land
const uint32_t w5500_reset_timeout_ms	= 100;		// MR.RST to clear after a soft reset
const uint32_t w5500_error_backoff_ms	= 1000;		// in w5500_state_t::error before starting again
const uint32_t w5500_connect_timeout_ms	= 60000;	// backstop: the chip's own RTR/RCR timeout comes first
const uint32_t w5500_close_timeout_ms	= 2000;		// graceful close before forcing CLOSE
const uint32_t w5500_send_timeout_ms	= 60000;	// SEND to SEND_OK, backstop as above

// Largest piece moved between the chip and the host in one SPI
// transfer. RAM: the driver keeps one buffer this size.
const uint16_t w5500_chunk_bytes = 1024;

// The UDP service socket (DHCP, DNS, SNTP), when
// w5500_param_t::serviceSocket is set.
const uint8_t w5500_service_socket = 7;

typedef struct w5500_param_t {
	// Per-socket buffer sizes in KB: 0, 1, 2, 4, 8 or 16. Each
	// direction totals at most 16. A socket given 0 can't be opened.
	// Only the sockets the interface uses (xNetInterface::Config::
	// maxSockets) need any: e.g. {4,4,4,4,0,0,0,0} for four.
	uint8_t  rxBufKb[w5500_sockets] = {2, 2, 2, 2, 2, 2, 2, 2};
	uint8_t  txBufKb[w5500_sockets] = {2, 2, 2, 2, 2, 2, 2, 2};
	uint16_t retryTime100us = 2000;		// RTR: 200 ms
	uint8_t  retryCount = 8;			// RCR
	// How often SIR (which sockets have news) is read when nothing
	// else prompts it. With the INT pin wired to
	// xNetInterface::interruptFromIsr() this is only a backstop and can
	// be long (100 ms); without it, it is the receive latency.
	uint32_t pollMs = 5;
	// Sockets mid-connect, mid-close or waiting for SEND_OK are polled
	// this often whatever pollMs is: the chip raises no interrupt for
	// some of those steps.
	uint32_t busyPollMs = 2;
	uint32_t linkPollMs = 500;
	// Keep socket 7 back as the UDP service socket, for DHCP
	// (NetConfig::dhcp), DNS (resolve()) and SNTP (requestTime()). It
	// needs at least 1 KB each way in rxBufKb/txBufKb. Turn it off to
	// give the interface all 8 sockets, with a static address and no
	// name lookups or time.
	bool     serviceSocket = true;
	// Mixed into DHCP transaction IDs, DNS query IDs and SNTP nonces,
	// so devices booting together don't collide and answers are hard
	// to forge: e.g. HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2().
	uint32_t seed = 0;
} w5500_param_t;

// Counters, for bring-up and field diagnostics. stats() may be called
// from any thread.
typedef struct w5500_stats_t {
	uint32_t transfers;		// SPI transfers issued
	uint32_t failures;		// times the driver gave up on the chip and started again
	uint32_t inits;			// successful start-ups
	uint32_t interrupts;	// INT pin events relayed by the host
	uint32_t rxBytes;		// TCP payload, chip -> host
	uint32_t txBytes;		// TCP payload, host -> chip
} w5500_stats_t;

enum class w5500_state_t : uint8_t {
	unconfigured,		// until configure()
	reset,				// MR.RST
	reset_poll,
	reset_check,
	read_version,
	check_version,
	write_addr,			// GAR, SUBR, SHAR, SIPR in one burst
	write_retry,		// RTR, RCR
	write_bufsize,		// Sn_RXBUF_SIZE/TXBUF_SIZE, one socket per pass
	bufsize_next,
	write_simr,
	addr_applied,		// SIPR/GAR/SUBR rewritten after a DHCP change: report it
	init_phy,
	init_done,
	idle,				// decide what to do next
	read_phy,
	check_phy,
	check_sir,
	svc_check,			// a socket's Sn_IR/Sn_SR: clear what was read
	svc_eval,			// and act on it
	svc_next,
	open_mode,			// connect/listen: Sn_MR
	open_ir,			// Sn_IR cleared
	open_addr,			// Sn_PORT..Sn_DPORT
	open_cmd,			// OPEN
	open_read_sr,
	open_check,			// SOCK_INIT? then CONNECT or LISTEN
	open_done,
	rx_again,			// Sn_RX_RSR read a second time: it must read the same twice
	rx_check,
	rx_deliver,
	rx_recv,
	tx_again,			// Sn_TX_FSR likewise
	tx_check,
	tx_fill,			// copy host bytes into the chip's TX buffer
	tx_wrote,
	tx_send,
	tx_sent,
	svc_open_port,		// the service socket: UDP, port 68
	svc_open_cmd,
	svc_open_sr,
	svc_open_check,
	svc_rx_again,		// a datagram: Sn_RX_RSR twice, then its 8-byte header, then the data
	svc_rx_check,
	svc_rx_hdr,
	svc_rx_data,
	svc_rx_recv,
	svc_tx_ptr,			// a datagram: Sn_DIPR/DPORT written, now Sn_TX_WR
	svc_tx_data,
	svc_tx_wr,
	cmd_poll,			// Sn_CR, until the chip has taken the command
	cmd_check,
	xfer_issue,			// every register access passes through these two
	xfer_wait,
	error				// back off, then start again from reset
};

template <typename TTransport>
class w5500 : protected TTransport, public iEthernetDevice {
	static_assert(std::is_base_of<iBlockTransport, TTransport>::value,
		"w5500<TTransport> needs an iBlockTransport (SpiBlockTransport, Stm32HalSpiBlockTransport, ...)");

public:
	template <typename... TArgs>
	explicit w5500(const w5500_param_t &param, TArgs&&... transportArgs);

	// iNetDevice
	void attach(iNetDeviceHost &host) override { _host = &host; }
	uint8_t socketCount() const override { return _param.serviceSocket ? w5500_service_socket : w5500_sockets; }
	void configure(const NetConfig &cfg) override;
	uint32_t poll(uint32_t nowMs) override;
	bool connect(uint8_t s, const IpAddress &ip, uint16_t port, uint16_t localPort) override;
	bool listen(uint8_t s, uint16_t port) override;
	void close(uint8_t s) override;
	void interrupt() override { _irq = true; _stInterrupts.fetch_add(1, std::memory_order_relaxed); }
	bool resolve(const char *name) override;
	bool requestTime(const char *server) override;

	// iEthernetDevice
	uint16_t speedMbps() const override;
	bool fullDuplex() const override;

	w5500_state_t state() const { return _state; }
	bool ready() const { return _ready; }
	bool linkUp() const { return _link; }
	// The address in use (all zeros without one). Driver thread only.
	const NetConfig &address() const { return _net; }
	const DhcpClient &dhcp() const { return _dhcp; }

	w5500_stats_t stats() const {
		w5500_stats_t s;
		s.transfers = _stTransfers.load(std::memory_order_relaxed);
		s.failures = _stFailures.load(std::memory_order_relaxed);
		s.inits = _stInits.load(std::memory_order_relaxed);
		s.interrupts = _stInterrupts.load(std::memory_order_relaxed);
		s.rxBytes = _stRx.load(std::memory_order_relaxed);
		s.txBytes = _stTx.load(std::memory_order_relaxed);
		return s;
	}

private:
	struct Sock {
		enum Mode : uint8_t { Closed, Opening, Connecting, Listening, Established, Closing };
		enum Req : uint8_t { NoReq, ReqConnect, ReqListen, ReqClose };
		Mode      mode = Closed;
		Req       req = NoReq;
		IpAddress reqIp;			// what connect()/listen() asked for
		uint16_t  reqPort = 0;
		uint16_t  reqLocalPort = 0;
		bool      opening_listen = false;
		bool      rxPending = false;	// the chip may hold received bytes
		bool      peerClosed = false;	// FIN seen: drain, then close our side
		bool      sendInFlight = false;	// SEND issued, SEND_OK not yet seen
		bool      txBlocked = false;	// chip TX buffer was full
		bool      forceClose = false;	// given up on (address lost): CLOSE it on the chip, quietly
		uint32_t  since = 0;			// entered Connecting/Closing
		uint32_t  sendSince = 0;
		uint32_t  txBlockedAt = 0;
		uint32_t  lastSvc = 0;			// Sn_IR/Sn_SR last read
	};

	uint32_t step(uint32_t nowMs);
	uint32_t schedule(uint32_t nowMs);
	uint32_t startRequest(uint8_t s, uint32_t nowMs);
	uint32_t evaluate(uint8_t s, uint8_t ir, uint8_t sr, uint32_t nowMs);
	uint32_t service(uint8_t s, bool fromSir, uint32_t nowMs);
	uint32_t rxBegin(uint8_t s, uint32_t nowMs);
	uint32_t txBegin(uint8_t s, uint32_t nowMs);
	uint32_t readSir(uint32_t nowMs);
	uint32_t idleWait(uint32_t nowMs) const;

	// Queue one register access; it runs through xfer_issue/xfer_wait
	// and carries on at `then`. buf must be a member: it is used after
	// this returns.
	uint32_t xfer(bool write, uint8_t bsb, uint16_t addr, uint8_t *buf, uint16_t len,
	              w5500_state_t then, uint32_t nowMs);
	// Write Sn_CR, wait for the chip to clear it, carry on at `then`.
	uint32_t command(uint8_t s, uint8_t cmd, w5500_state_t then, uint32_t nowMs);
	uint32_t fail(uint32_t nowMs);
	void dropAll(SocketEvent ev, bool closeOnChip);
	void applyPhy(uint32_t nowMs);
	void fillAddr();			// _w[0..18): GAR, SUBR, SHAR, SIPR from _net
	void setAddress(const NetConfig &net);
	void dhcpEvent(DhcpClient::Event ev);
	void dnsEvent(DnsClient::Event ev);
	void ntpDnsEvent(DnsClient::Event ev, uint32_t nowMs);
	void sntpEvent(SntpClient::Event ev);
	void failQueries();			// answer any DNS/time request in progress: failed
	bool svcUsable() const {	// the service socket is kept, and has buffers
		return _param.serviceSocket && _param.rxBufKb[w5500_service_socket] != 0
		       && _param.txBufKb[w5500_service_socket] != 0;
	}
	uint32_t serviceStep(uint32_t nowMs);
	uint32_t sendService(const IpAddress &dst, uint16_t port, size_t len, uint32_t nowMs);
	uint32_t nextRand() { _rand = _rand * 1664525u + 1013904223u; return _rand; }
	bool bufConfigValid() const;

	void enter(w5500_state_t next, uint32_t nowMs) { _state = next; _since = nowMs; }
	bool elapsed(uint32_t nowMs, uint32_t ms) const { return (nowMs - _since) >= ms; }
	static bool after(uint32_t nowMs, uint32_t t0, uint32_t ms) { return (nowMs - t0) >= ms; }
	static uint32_t remaining(uint32_t nowMs, uint32_t t0, uint32_t ms) {
		const uint32_t gone = nowMs - t0;
		return gone >= ms ? 0 : ms - gone;
	}
	static uint16_t be16(const uint8_t *p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
	static void putBe16(uint8_t *p, uint16_t v) { p[0] = static_cast<uint8_t>(v >> 8); p[1] = static_cast<uint8_t>(v); }

	void emit(uint8_t s, SocketEvent ev) { if (_host) _host->socketEvent(s, ev); }
	void emitDev(DeviceEvent ev) { if (_host) _host->deviceEvent(ev); }

	w5500_param_t   _param;
	NetConfig       _cfg;				// what configure() asked for
	NetConfig       _net;				// the address in use: _cfg's, or a lease, or none
	IpAddress       _prevIp;			// last address held, to ask DHCP for again
	DhcpClient      _dhcp;
	bool            _dhcpOn = false;	// this configuration uses DHCP
	bool            _svcOpen = false;	// socket 7 open in UDP mode
	bool            _addrDirty = false;	// _net changed: write it to the chip and report it

	// DNS and SNTP, over the service socket.
	DnsClient       _dns;				// resolve()
	DnsClient       _ntpDns;			// the time server's name, for requestTime()
	SntpClient      _sntp;
	bool            _dnsReq = false;	// resolve() asked, not yet started
	bool            _ntpReq = false;	// requestTime() asked, not yet started
	bool            _ntpBusy = false;	// a time request in progress (DNS or SNTP)
	char            _dnsName[DnsClient::kMaxName + 2] = {0};
	char            _ntpServer[DnsClient::kMaxName + 2] = {0};
	uint32_t        _rand = 0;
	iNetDeviceHost *_host = nullptr;
	Sock            _sock[w5500_sockets];

	w5500_state_t _state = w5500_state_t::unconfigured;
	uint32_t      _since = 0;			// when _state was entered
	uint32_t      _phaseStart = 0;
	bool          _configured = false;
	bool          _reconfigure = false;
	bool          _ready = false;
	bool          _link = false;
	bool          _irq = false;			// INT pin fired since SIR was last read
	bool          _sirRecheck = false;	// read SIR again once there is nothing better to do
	uint32_t      _lastSir = 0;
	uint32_t      _lastPhy = 0;
	uint8_t       _rr = 0;				// round robin start for RX/TX
	uint16_t      _nextPort = 49152;

	// the access in flight (xfer)
	bool          _xWrite = false;
	uint8_t       _xBsb = 0;
	uint16_t      _xAddr = 0;
	uint8_t      *_xBuf = nullptr;
	uint16_t      _xLen = 0;
	w5500_state_t _xThen = w5500_state_t::idle;

	// command()
	uint8_t       _cmdSock = 0;
	uint8_t       _cmdByte = 0;
	uint8_t       _cmdRead = 0;
	uint8_t       _cmdTries = 0;
	w5500_state_t _cmdThen = w5500_state_t::idle;

	uint8_t  _cur = 0;					// socket the current sequence is about
	bool     _svcFromSir = false;
	uint8_t  _sirMask = 0;
	uint8_t  _rd = 0;					// one-byte reads
	uint8_t  _sir = 0;
	uint8_t  _phy = 0;
	uint8_t  _bufIdx = 0;
	uint8_t  _svc[2] = {0, 0};			// Sn_IR, Sn_SR
	uint8_t  _udpHdr[8] = {0};			// a received datagram's: source IP, port, length
	uint16_t _udpLen = 0;				// its length
	bool     _udpRead = false;			// it fitted in _chunk and was read
	uint8_t  _a[6] = {0};				// first read of RSR/RD or FSR/RD/WR
	uint8_t  _b[2] = {0};				// second read of RSR or FSR
	uint8_t  _tries = 0;
	uint16_t _fsr = 0;
	uint16_t _ptr = 0;					// Sn_RX_RD or Sn_TX_WR being advanced
	uint16_t _moved = 0;				// bytes in _chunk
	uint16_t _txWrote = 0;
	uint8_t  _w[18] = {0};				// register write data
	uint8_t  _chunk[w5500_chunk_bytes];	// payload to or from the chip's buffers

	std::atomic<uint32_t> _stTransfers{0}, _stFailures{0}, _stInits{0}, _stInterrupts{0}, _stRx{0}, _stTx{0};
};

} /* namespace W5500 */

#include "../src/W5500.tpp"

#endif /* W5500_H_ */
