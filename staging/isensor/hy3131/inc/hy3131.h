/*
 * hy3131.h
 *
 *  The Hycon HY3131 (or HY3130) DMM analog front end, over SPI. Non-blocking,
 *  on SensorStateMachine like the other isensor drivers. hy3131_dmm<TTransport>
 *  inherits its transport, an iBlockTransport: every access is one command
 *  byte (the register address and the read bit), then data.
 *
 *      hy3131_param_t p = hy3131_default_param();
 *      p.irq = true;                                  // DO's rising edge wired to an EXTI line
 *      xhy3131<Stm32HalSpiBlockTransport> dmm(p, &hspi2, DMM_CS_GPIO_Port, DMM_CS_Pin, spi2Mutex);
 *      // in the EXTI callback (DO rising): dmm.onIrq();
 *
 *      hy3131::Config cfg = hy3131::defaultConfig();  // references on, AD1 at 5 Hz on the temperature sensor
 *      ...set the switch network, inputs and gains for the function wanted...
 *      dmm.start(cfg);
 *      for(;;){ dmm.main(HAL_GetTick()); }            // its own thread
 *
 *      hy3131::Sample s;
 *      if (dmm.latest(&s)) { ...s.ad1... }            // from any thread
 *      // or dmm.setListener(&myListener): onSample()/onCount() in the driver's thread
 *
 *  What the chip measures is all in its configuration registers (0x20..0x36):
 *  the switch network on PA<9:0>, the pre-filter, the ADC input and reference
 *  multiplexers, gains, OSRs, the LPF, the RMS converter, peak hold, the
 *  comparators and the counter. hy3131::Config holds that image whole, as the
 *  chip should end up; this driver loads it, keeps it there, and reads results.
 *  The register settings for each meter function (DCV, ACV, ohms, ...) depend
 *  on the external network around the chip and belong to the application.
 *
 *  Startup (after start(), and again after a fault): R37 = 0x60 resets the
 *  chip; kResetMs later INTE and INTF are cleared and the configuration
 *  written with the ADC and counter enables held off (the oscillator and
 *  references come on here; SPHACAL goes in before AD2/AD3 are enabled, as
 *  13.2 asks). kOscSettleMs later AD1, AD2, AD3 and the counter are enabled
 *  together (footnote 7: don't switch one ADC on or off while another runs),
 *  the counter's gate is loaded, and the configuration is read back and
 *  compared (only the bits the datasheet defines). Then INTE.
 *
 *  There is no ID register. The read-back is the presence check: an absent
 *  chip reads all 0s or all 1s and fails it.
 *
 *  Running: on an IRQ (onIrq()), when param.doLevel says DO is high, or every
 *  param.pollMs, INTF is read (which clears it). Then, for what it flags:
 *    AD1, AD2, LPF or RMS  -> 0x00..0x0D (AD1, AD2, LPF, RMS) in one transfer,
 *                             and 0x0E..0x13 (peak hold) with ENPKH
 *    the counter           -> 0x14..0x1E (CTSTA, CTC, CTB, CTA, INTF), and the
 *                             gate is reloaded and the counter started again
 *  and INTF is read again, until it reads 0. A Sample is published for the
 *  first group, a Count for the second.
 *
 *  A read ends seven clocks into the register after the last one wanted (the
 *  data is a bit late, see Hy3131Regs.h). The datasheet doesn't say whether
 *  that counts as reading it; this driver assumes it might, and never ends a
 *  read just before INTF or CTSTA, whose reads clear something. With the
 *  counter on, peak hold is read with the counter's results (0x0E..0x1E), not
 *  on its own.
 *
 *  Never optimistic: every param.healthMs the configuration is read back
 *  again, and a brown-out (BORF in INTF) is a fault. Either way the chip is
 *  reset and configured again.
 *
 *  DO is the IRQ line while CS is high, and toggles with data while CS is
 *  low: edges during the driver's own transfers are ignored. An event that
 *  lands in the last INTF read raises DO as CS goes up, which is ignored too;
 *  param.doLevel (a read of the DO pin) catches it at once, otherwise the poll
 *  does.
 *
 *  The HY3131 drives DO whenever it is powered: it needs an SPI bus of its
 *  own, or a buffer on DO enabled by its CS.
 *
 *  From Hycon's datasheet DS-HY3131-V10_EN. Not run against a chip.
 */

#ifndef HY3131_H_
#define HY3131_H_

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <type_traits>
#include <utility>
#include "iBlockTransport.h"
#include "SensorStateMachine.h"
#include "Hy3131Regs.h"

namespace hy3131 {

enum class Variant : uint8_t { Hy3131, Hy3130 };

// The configuration registers as they should end up, enables and all.
struct Config {
	uint8_t  reg[kCfgCount];       // 0x20..0x36 (0x20..0x33 on the HY3130)
	uint8_t  inte;                 // INTE: which flags raise DO
	uint32_t gateCycles;           // the counter's gate in SYSCLK cycles (with ENCTR), 256..16,776,960
	uint32_t userTag;              // not the chip's: carried along, so config() says which request is running

	uint8_t& r(uint8_t addr) { return reg[cfgIndex(addr)]; }
	uint8_t  r(uint8_t addr) const { return reg[cfgIndex(addr)]; }
	void field(uint8_t addr, uint8_t mask, uint8_t shift, uint8_t value) {
		uint8_t& x = r(addr);
		x = (uint8_t)((x & ~(mask << shift)) | ((value & mask) << shift));
	}
	void bit(uint8_t addr, uint8_t m, bool on) { r(addr) = on ? (uint8_t)(r(addr) | m) : (uint8_t)(r(addr) & ~m); }

	// The few fields everyone sets. The rest by r() and the kRxx constants.
	void ad1(bool on, uint8_t osr, uint8_t inputGain, bool refGainThird) {
		bit(kR23, kR23Enad1, on);
		field(kR22, 0x07, 0, osr);
		field(kR25, 0x03, kR25Ad1igShift, inputGain);
		bit(kR23, kR23Ad1rg, refGainThird);
	}
	void ad1Input(uint8_t sad1i) { field(kR33, 0x03, 0, sad1i); }
	void ad1Reference(uint8_t sad1rh, uint8_t sad1rl) { field(kR28, 0x07, 4, sad1rh); field(kR28, 0x07, 0, sad1rl); }
	void ad2(bool on, uint8_t osr, uint8_t inputGain, bool clkQuarter) {
		bit(kR26, kR26Enad2, on);
		field(kR26, 0x07, 0, osr);
		field(kR25, 0x03, kR25Ad2igShift, inputGain);
		bit(kR26, kR26Sad2clk, clkQuarter);
	}
	void counter(bool on, bool fromCntPin, uint32_t gate) {
		bit(kR20, kR20Enctr, on);
		bit(kR20, kR20Encnti, fromCntPin);
		gateCycles = gate;
	}
	bool ad1On() const   { return (r(kR23) & kR23Enad1) != 0; }
	bool ad2On() const   { return (r(kR26) & kR26Enad2) != 0; }
	bool peakOn() const  { return (r(kR29) & kR29Enpkh) != 0; }
	bool counterOn() const { return (r(kR20) & kR20Enctr) != 0; }
};

// Oscillator (crystal, SYSCLK 4.9152 MHz), bias, both references (REFO from
// the band gap), AGND at half VDDA, ACM at 1.5 V. AD1 at OSR 32768 (5 Hz),
// input gain 3.6, reference gain 0.333, reference REFO/AGND (the datasheet's
// "ADC set default value", 12.4), on the temperature sensor TS1P/TS2N, with
// its IRQ. Everything else off and every switch open.
inline Config defaultConfig() {
	Config c;
	memset(&c, 0, sizeof c);
	c.r(kR33) = kR33Enosc;
	c.r(kR31) = kR31Enrefo | kR31Enbias | (0u << kR31SagndShift);
	c.r(kR2F) = kR2FEnvs;
	c.field(kR25, 0x03, kR25SacmShift, 2);   // SACM 10: 1.5 V
	c.ad1(true, 7, 3, true);
	c.ad1Reference(1, 1);                    // REFO, AGND
	c.ad1Input(2);                           // TS1P / TS2N
	c.inte = kIntAd1;
	c.gateCycles = kXtalHz;                  // 1 s, should the counter be turned on
	return c;
}

// One update of the measurement registers. flags: the INTF bits that made it.
// Values as the chip gives them, sign extended: AD1 24 bits (HY3130: <23:8>
// valid), AD2 / LPF / peak 19 bits, RMS 38 bits (sum of squares / N: take the
// square root yourself).
struct Sample {
	uint32_t seq;
	uint32_t atMs;
	uint8_t  flags;
	int32_t  ad1;
	int32_t  ad2;
	int32_t  lpf;
	int64_t  rms;
	int32_t  peakMin;
	int32_t  peakMax;
	bool     peakValid;
};

// One finished frequency count. Frequency = ctb / T, T = cycles() / SYSCLK.
struct Count {
	uint32_t seq;
	uint32_t atMs;
	uint32_t ctaInit;    // the gate as loaded (CTA<23:8>, low byte 0)
	uint32_t ctaFinal;
	uint32_t ctb;        // whole input cycles
	uint32_t ctc;        // SYSCLKs the input was high
	bool     overflow;   // CTBOV: no good, use a shorter gate
	uint8_t  ctsta;

	uint32_t cycles() const { return (0x1000000u - ctaInit) + ctaFinal; }
	double   frequencyHz(uint32_t sysclkHz) const {
		const uint32_t n = cycles();
		return n ? (double)ctb * (double)sysclkHz / (double)n : 0.0;
	}
	double   duty() const { const uint32_t n = cycles(); return n ? (double)ctc / (double)n : 0.0; }
};

// Told in the driver's thread. Keep it short: the driver waits.
class iListener {
public:
	virtual ~iListener() = default;
	virtual void onSample(const Sample& s) { (void)s; }
	virtual void onCount(const Count& c) { (void)c; }
	virtual void onFault() {}
	virtual void onRunning() {}
};

} // namespace hy3131

typedef enum hy3131_state_t {
	hy3131_stopped = 0,     // waiting for start()
	hy3131_init,            // queue the reset
	hy3131_ops,             // the queued transfers, one each
	hy3131_wait_ops,
	hy3131_reset_settle,    // after R37 = 0x60
	hy3131_osc_settle,      // configured, ADCs off: the oscillator and references start
	hy3131_check_config,    // the read-back has landed
	hy3131_running,         // wait for an IRQ, the poll or the health check
	hy3131_check_flags,     // INTF has landed
	hy3131_serviced,        // the data has landed
	hy3131_check_health,    // the periodic read-back has landed
	hy3131_error
} hy3131_state_t;

typedef struct hy3131_param_t {
	hy3131::Variant variant;
	uint32_t sysclkHz;        // 4,915,200 with the datasheet's crystal
	bool     irq;             // onIrq() is called from DO's rising edge
	uint16_t pollMs;          // how often INTF is read without an IRQ (a backstop with one)
	uint16_t healthMs;        // how often the configuration is read back; 0: never
	bool     verifyConfig;    // compare the read-back (false: read it, don't judge it)
	// Optional: the level of the DO pin, read while CS is high. High means an
	// enabled flag is set. Called from the driver's thread.
	bool   (*doLevel)(void* ctx);
	void*    doCtx;
} hy3131_param_t;

inline hy3131_param_t hy3131_default_param() {
	hy3131_param_t p;
	p.variant = hy3131::Variant::Hy3131;
	p.sysclkHz = hy3131::kXtalHz;
	p.irq = false;
	p.pollMs = 50;
	p.healthMs = 1000;
	p.verifyConfig = true;
	p.doLevel = nullptr;
	p.doCtx = nullptr;
	return p;
}

template <typename TTransport>
class hy3131_dmm : public SensorStateMachine<TTransport, hy3131_state_t> {
	static_assert(std::is_base_of<iBlockTransport, TTransport>::value,
		"hy3131 needs an iBlockTransport (SpiBlockTransport, ...)");
	using Base = SensorStateMachine<TTransport, hy3131_state_t>;

public:
	static constexpr uint32_t kBusTimeoutMs   = 100;
	static constexpr uint32_t kErrorBackoffMs = 1000;
	static constexpr uint32_t kResetMs        = 10;   // the datasheet gives none
	static constexpr uint32_t kOscSettleMs    = 20;   // nor this: a 4.9 MHz crystal starts in a few ms
	static constexpr uint32_t kIdleSleepMs    = 100;

	template <typename... TArgs>
	explicit hy3131_dmm(const hy3131_param_t& param, TArgs&&... transportArgs);

	void main(uint32_t nowMs);

	// Take effect in main(). start() again with a new Config reconfigures
	// (through a reset). stop() resets the chip and leaves it.
	bool start(const hy3131::Config& cfg);
	void stop();

	// From DO's rising edge. Safe in an interrupt.
	void onIrq() {
		if (_inXfer.load(std::memory_order_acquire)) return;   // DO is data, not the IRQ
		_irqPending.store(true, std::memory_order_release);
		wake();
	}

	// The most recent Sample / Count. False until there is one. Any thread.
	bool latest(hy3131::Sample* out) const { return readLocked(_sample, out); }
	bool latestCount(hy3131::Count* out) const { return readLocked(_count, out); }

	void setListener(hy3131::iListener* l) { _listener = l; }
	bool running() const { return this->_state >= hy3131_running && this->_state <= hy3131_check_health; }
	hy3131_state_t driverState() const { return this->_state; }
	uint32_t faults() const { return _faults; }
	const hy3131::Config& config() const { return _cfg; }

protected:
	void onFail() override;
	// Called from onIrq(), start() and stop(). An OS version wakes the thread.
	virtual void wake() {}
	bool workPending() const {
		return _irqPending.load(std::memory_order_acquire) || _req.load(std::memory_order_acquire) != Req::None;
	}

private:
	enum class OpKind : uint8_t { Write, Read };
	struct Op {
		OpKind   kind;
		uint8_t  addr;
		uint8_t  len;
		uint8_t  v[hy3131::kCfgCount];   // Write: the data
		uint8_t* dst;                     // Read: where it lands, len bytes
	};
	enum class Req : uint8_t { None, Start, Stop };

	// A single writer (main()) and any number of readers: a sequence lock.
	template <typename T>
	struct Locked {
		std::atomic<uint32_t> seq{0};   // odd while being written; 0: never written
		T v;
	};
	template <typename T>
	void publish(Locked<T>& l, const T& v) {
		l.seq.fetch_add(1, std::memory_order_acq_rel);
		std::atomic_thread_fence(std::memory_order_release);
		l.v = v;
		std::atomic_thread_fence(std::memory_order_release);
		l.seq.fetch_add(1, std::memory_order_release);
	}
	template <typename T>
	static bool readLocked(const Locked<T>& l, T* out) {
		for (;;) {
			const uint32_t a = l.seq.load(std::memory_order_acquire);
			if (a == 0) return false;
			if (a & 1) continue;
			std::atomic_thread_fence(std::memory_order_acquire);
			*out = l.v;
			std::atomic_thread_fence(std::memory_order_acquire);
			if (l.seq.load(std::memory_order_relaxed) == a) return true;
		}
	}

	void opsClear() { _opCount = 0; _opPos = 0; }
	Op* add(OpKind k);
	void write(uint8_t addr, const uint8_t* v, uint8_t n);
	void write1(uint8_t addr, uint8_t v) { write(addr, &v, 1); }
	void read(uint8_t addr, uint8_t* dst, uint8_t n);
	void runOps(hy3131_state_t after, uint32_t nowMs);
	bool issueOp(const Op& o);

	uint8_t cfgLen() const;   // configuration registers this variant has
	uint8_t backLen() const;  // of those, how many are read back
	void queueGate();
	void queueReadBack();
	bool configMatches() const;
	bool handleRequest(uint32_t nowMs);
	void beginService(uint32_t nowMs);
	void publishResults(uint32_t nowMs);
	static uint32_t clampGate(uint32_t g);

	hy3131_param_t _param;
	hy3131::Config _cfg;
	hy3131::Config _reqCfg;
	std::atomic<Req> _req{Req::None};

	Op      _ops[8];
	uint8_t _opCount = 0, _opPos = 0;
	hy3131_state_t _afterOps = hy3131_running;
	uint8_t _rx[32];              // a read's raw bytes, before unshiftRead()

	uint32_t _lastPollMs = 0;
	uint32_t _lastHealthMs = 0;
	uint32_t _ctaInit = 0;        // the gate last loaded
	uint32_t _countGate = 0;      // the gate of the count being read
	uint8_t  _carry = 0;          // INTF flags read (and so cleared) along with the counter

	// What the chip said. Members, as the transport fills them after main() returns.
	uint8_t _intf = 0;
	uint8_t _flags = 0;           // what this service is reading for
	uint8_t _data[hy3131::kDataLen];
	uint8_t _peak[hy3131::kPeakLen];
	uint8_t _countRegs[hy3131::kPeakLen + hy3131::kCountLen];   // 0x0E..0x1E
	bool    _readData = false, _readPeak = false, _readCount = false, _peakWithCount = false;
	uint8_t _back[hy3131::kCfgCount];

	std::atomic<bool> _irqPending{false};
	std::atomic<bool> _inXfer{false};
	uint32_t _faults = 0;

	Locked<hy3131::Sample> _sample;
	Locked<hy3131::Count>  _count;
	uint32_t _sampleSeq = 0, _countSeq = 0;
	hy3131::Sample _lastSample;
	hy3131::iListener* _listener = nullptr;
};

#include "../src/hy3131.tpp"

#endif /* HY3131_H_ */
