/*
 * W5500Regs.h
 *
 *  WIZnet W5500 register map and SPI frame format, from the W5500
 *  datasheet v1.1, sections 2.2 and 4. Shared by the driver and its
 *  host test's simulated chip.
 */

#ifndef W5500REGS_H_
#define W5500REGS_H_

#include <stdint.h>

namespace W5500 {

const uint8_t w5500_sockets = 8;
const uint8_t w5500_version = 0x04;		// VERSIONR

// Every SPI access is a 3-byte header and then the data:
//   [ address 15:8 ][ address 7:0 ][ BSB 4:0 | RWB | OM 1:0 ]
// BSB picks the block, RWB is 1 for a write, OM 00 is variable
// length (chip-select framed), the only mode used here.
const uint8_t w5500_ctl_write = 0x04;

inline uint8_t w5500_bsb_common()           { return 0x00; }
inline uint8_t w5500_bsb_sock_reg(uint8_t s){ return static_cast<uint8_t>(s * 4 + 1); }
inline uint8_t w5500_bsb_sock_tx(uint8_t s) { return static_cast<uint8_t>(s * 4 + 2); }
inline uint8_t w5500_bsb_sock_rx(uint8_t s) { return static_cast<uint8_t>(s * 4 + 3); }

// Common registers (BSB 0)
typedef enum w5500_common_reg_t {
	w5500_MR		= 0x0000,
	w5500_GAR		= 0x0001,	// gateway, 4
	w5500_SUBR		= 0x0005,	// subnet mask, 4
	w5500_SHAR		= 0x0009,	// MAC, 6
	w5500_SIPR		= 0x000F,	// own IP, 4
	w5500_IR		= 0x0015,
	w5500_IMR		= 0x0016,
	w5500_SIR		= 0x0017,	// one bit per socket with an interrupt pending
	w5500_SIMR		= 0x0018,
	w5500_RTR		= 0x0019,	// retry time, 2, units of 100 us
	w5500_RCR		= 0x001B,	// retry count
	w5500_PHYCFGR	= 0x002E,
	w5500_VERSIONR	= 0x0039
} w5500_common_reg_t;

// Socket registers (BSB = socket * 4 + 1)
typedef enum w5500_sock_reg_t {
	w5500_Sn_MR			= 0x00,
	w5500_Sn_CR			= 0x01,
	w5500_Sn_IR			= 0x02,
	w5500_Sn_SR			= 0x03,
	w5500_Sn_PORT		= 0x04,	// 2
	w5500_Sn_DHAR		= 0x06,	// 6
	w5500_Sn_DIPR		= 0x0C,	// 4
	w5500_Sn_DPORT		= 0x10,	// 2
	w5500_Sn_RXBUF_SIZE	= 0x1E,	// KB
	w5500_Sn_TXBUF_SIZE	= 0x1F,	// KB
	w5500_Sn_TX_FSR		= 0x20,	// 2, free space in the TX buffer
	w5500_Sn_TX_RD		= 0x22,	// 2
	w5500_Sn_TX_WR		= 0x24,	// 2
	w5500_Sn_RX_RSR		= 0x26,	// 2, bytes received and not yet read
	w5500_Sn_RX_RD		= 0x28,	// 2
	w5500_Sn_RX_WR		= 0x2A,	// 2
	w5500_Sn_IMR		= 0x2C
} w5500_sock_reg_t;

const uint8_t w5500_MR_RST = 0x80;

const uint8_t w5500_PHY_LNK = 0x01;		// PHYCFGR
const uint8_t w5500_PHY_SPD = 0x02;		// 1: 100 Mbps
const uint8_t w5500_PHY_DPX = 0x04;		// 1: full duplex

const uint8_t w5500_Sn_MR_TCP = 0x01;
const uint8_t w5500_Sn_MR_UDP = 0x02;
const uint8_t w5500_Sn_MR_ND  = 0x20;	// TCP: ACK at once, no delayed ACK

typedef enum w5500_cmd_t {
	w5500_CR_OPEN		= 0x01,
	w5500_CR_LISTEN		= 0x02,
	w5500_CR_CONNECT	= 0x04,
	w5500_CR_DISCON		= 0x08,
	w5500_CR_CLOSE		= 0x10,
	w5500_CR_SEND		= 0x20,
	w5500_CR_RECV		= 0x40
} w5500_cmd_t;

const uint8_t w5500_IR_CON		= 0x01;
const uint8_t w5500_IR_DISCON	= 0x02;
const uint8_t w5500_IR_RECV		= 0x04;
const uint8_t w5500_IR_TIMEOUT	= 0x08;
const uint8_t w5500_IR_SENDOK	= 0x10;

typedef enum w5500_sock_status_t {
	w5500_SOCK_CLOSED		= 0x00,
	w5500_SOCK_INIT			= 0x13,
	w5500_SOCK_LISTEN		= 0x14,
	w5500_SOCK_SYNSENT		= 0x15,
	w5500_SOCK_SYNRECV		= 0x16,
	w5500_SOCK_ESTABLISHED	= 0x17,
	w5500_SOCK_FIN_WAIT		= 0x18,
	w5500_SOCK_CLOSING		= 0x1A,
	w5500_SOCK_TIME_WAIT	= 0x1B,
	w5500_SOCK_CLOSE_WAIT	= 0x1C,
	w5500_SOCK_LAST_ACK		= 0x1D,
	w5500_SOCK_UDP			= 0x22
} w5500_sock_status_t;

} /* namespace W5500 */

#endif /* W5500REGS_H_ */
