/*
 * Latest.h
 *
 *  meter::Latest<T>: the most recent value of T, written by one thread and
 *  read by any number, without a lock (a sequence lock). A reader that
 *  overlaps a write tries again. For small T: a Reading, a RawResult.
 */

#ifndef METER_LATEST_H_
#define METER_LATEST_H_

#include <stdint.h>
#include <atomic>

namespace meter {

template <typename T>
class Latest {
public:
	// The single writer.
	void publish(const T& v) {
		_seq.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
		std::atomic_thread_fence(std::memory_order_release);
		_v = v;
		std::atomic_thread_fence(std::memory_order_release);
		_seq.fetch_add(1, std::memory_order_release);   // even: done
	}

	// False until the first publish().
	bool read(T* out) const {
		for (;;) {
			const uint32_t a = _seq.load(std::memory_order_acquire);
			if (a == 0) return false;
			if (a & 1u) continue;
			std::atomic_thread_fence(std::memory_order_acquire);
			*out = _v;
			std::atomic_thread_fence(std::memory_order_acquire);
			if (_seq.load(std::memory_order_relaxed) == a) return true;
		}
	}

private:
	std::atomic<uint32_t> _seq{0};
	T _v{};
};

} // namespace meter

#endif /* METER_LATEST_H_ */
