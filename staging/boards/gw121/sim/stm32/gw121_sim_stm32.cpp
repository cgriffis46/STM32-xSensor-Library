/*
 * gw121_sim_stm32.cpp
 *
 *  See gw121_sim_stm32.h. Two rings between the UART interrupts and the
 *  loop: bytes received go in one, one at a time (HAL_UART_Receive_IT of 1
 *  byte, re-armed in the callback). Bytes to send go in the other, sent in
 *  contiguous pieces by HAL_UART_Transmit_IT, the next piece started from
 *  the transmit-complete callback. The loop feeds the simulator and runs it.
 */
#include <new>
#include "gw121_sim_stm32.h"
#include "Gw121Simulator.h"

namespace {

template <uint16_t N>
struct Ring {   // one writer, one reader, N a power of two
	static_assert((N & (N - 1)) == 0, "N must be a power of two");
	uint8_t buf[N];
	volatile uint16_t head = 0, tail = 0;
	uint16_t used() const { return (uint16_t)(head - tail); }
	bool put(uint8_t b) { if (used() >= N) return false; buf[head & (N - 1)] = b; head = (uint16_t)(head + 1); return true; }
	bool get(uint8_t* b) { if (head == tail) return false; *b = buf[tail & (N - 1)]; tail = (uint16_t)(tail + 1); return true; }
};

UART_HandleTypeDef* g_uart = nullptr;
Ring<256> g_rx;
Ring<1024> g_tx;
uint8_t g_rxByte;
volatile uint16_t g_txLen = 0;     // bytes of g_tx in flight (0: idle)
volatile uint32_t g_dropped = 0;

// Start the next piece: from the tail to the end of the buffer at most.
void kickTx() {
	if (g_txLen || !g_uart) return;
	const uint16_t n = g_tx.used();
	if (!n) return;
	const uint16_t at = g_tx.tail & (1024 - 1);
	const uint16_t len = (uint16_t)(n < 1024 - at ? n : 1024 - at);
	g_txLen = len;
	if (HAL_UART_Transmit_IT(g_uart, &g_tx.buf[at], len) != HAL_OK) g_txLen = 0;
}

struct UartSink : gw121::iByteSink {
	void write(const uint8_t* b, size_t n) override {
		for (size_t i = 0; i < n; i++) if (!g_tx.put(b[i])) g_dropped = g_dropped + 1;
		__disable_irq();
		kickTx();
		__enable_irq();
	}
};

UartSink g_sink;
alignas(gw121::Simulator) unsigned char g_simMem[sizeof(gw121::Simulator)];
gw121::Simulator* g_sim = nullptr;

} // namespace

extern "C" void gw121SimStart(UART_HandleTypeDef* huart) {
	g_uart = huart;
	g_sim = new (g_simMem) gw121::Simulator(g_sink);
	HAL_UART_Receive_IT(g_uart, &g_rxByte, 1);
}

extern "C" void gw121SimLoop(void) {
	if (!g_sim) return;
	uint8_t b;
	while (g_rx.get(&b)) g_sim->feed(b);
	g_sim->main(HAL_GetTick());
}

extern "C" void gw121SimUartRxDone(UART_HandleTypeDef* huart) {
	if (huart != g_uart) return;
	g_rx.put(g_rxByte);
	HAL_UART_Receive_IT(g_uart, &g_rxByte, 1);
}

extern "C" void gw121SimUartTxDone(UART_HandleTypeDef* huart) {
	if (huart != g_uart) return;
	g_tx.tail = (uint16_t)(g_tx.tail + g_txLen);
	g_txLen = 0;
	kickTx();
}

extern "C" void gw121SimUartError(UART_HandleTypeDef* huart) {
	if (huart != g_uart) return;
	// An overrun or noise: drop what was in flight and listen again.
	if (g_txLen) { g_tx.tail = (uint16_t)(g_tx.tail + g_txLen); g_txLen = 0; }
	HAL_UART_Receive_IT(g_uart, &g_rxByte, 1);
	kickTx();
}

#ifndef GW121_SIM_NO_HAL_CALLBACKS
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) { gw121SimUartRxDone(huart); }
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) { gw121SimUartTxDone(huart); }
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) { gw121SimUartError(huart); }
#endif
