/*
 * ByteRing.h
 *
 *  A byte ring from one interrupt to one thread: the UART interrupt
 *  push()es, the driver's main() pop()s. Single producer, single
 *  consumer, so the two indices are each written by one side only and
 *  need nothing but atomic loads and stores (no read-modify-write, so
 *  it suits a Cortex-M0 too). A push into a full ring drops the byte
 *  and counts it.
 *
 *  Used by the GNSS drivers (ublox_gps, mtk3339), which parse in the
 *  thread, not in the interrupt.
 */

#ifndef BYTE_RING_H_
#define BYTE_RING_H_

#include <stdint.h>
#include <atomic>

template <uint16_t N>
class ByteRing {
	static_assert(N >= 2 && (N & (N - 1)) == 0, "ByteRing<N>: N must be a power of two");
public:
	ByteRing() : _head(0), _tail(0), _overflows(0) {}

	// Interrupt side. False (and counted) if the ring is full.
	bool push(uint8_t b) {
		const uint32_t head = _head.load(std::memory_order_relaxed);
		const uint32_t tail = _tail.load(std::memory_order_acquire);
		if (head - tail >= N) {
			_overflows.store(_overflows.load(std::memory_order_relaxed) + 1, std::memory_order_release);
			return false;
		}
		_buf[head & (N - 1)] = b;
		_head.store(head + 1, std::memory_order_release);
		return true;
	}

	// Thread side. False if there is nothing waiting.
	bool pop(uint8_t* b) {
		const uint32_t tail = _tail.load(std::memory_order_relaxed);
		if (tail == _head.load(std::memory_order_acquire)) return false;
		*b = _buf[tail & (N - 1)];
		_tail.store(tail + 1, std::memory_order_release);
		return true;
	}

	// Bytes dropped because the ring was full, since construction.
	uint32_t overflows() const { return _overflows.load(std::memory_order_acquire); }

private:
	uint8_t               _buf[N] = {0};
	std::atomic<uint32_t> _head;        // written by the interrupt only
	std::atomic<uint32_t> _tail;        // written by the thread only
	std::atomic<uint32_t> _overflows;   // written by the interrupt only
};

#endif /* BYTE_RING_H_ */
