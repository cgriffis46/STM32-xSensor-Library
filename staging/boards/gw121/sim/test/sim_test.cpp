// Host test of the simulator: our firmware's meter stack talking the
// 121GW's Bluetooth protocol. Optionally writes the packets it sent, in
// hex, one per line, for the web page's decoder test:  ./sim_test packets.txt
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "Gw121Simulator.h"

using namespace gw121;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

struct Capture : iByteSink {
	std::vector<uint8_t> bytes;
	void write(const uint8_t* b, size_t n) override { bytes.insert(bytes.end(), b, b + n); }
};

// The packets in what was sent, and the echoes.
static std::vector<bt::Frame> frames(const std::vector<uint8_t>& s, std::vector<std::vector<uint8_t>>* raw, int* echoes) {
	std::vector<bt::Frame> out;
	for (size_t i = 0; i < s.size();) {
		if (s[i] == bt::kStart && i + bt::kPacketLen <= s.size()) {
			bt::Frame f;
			if (bt::decode(&s[i], &f)) { out.push_back(f); if (raw) raw->push_back(std::vector<uint8_t>(&s[i], &s[i] + bt::kPacketLen)); }
			i += bt::kPacketLen;
		} else if (s[i] == bt::kKeyStart) { if (echoes) (*echoes)++; i += bt::kKeyLen; }
		else if (s[i] == bt::kTimeStart) { if (echoes) (*echoes)++; i += bt::kTimeLen; }
		else i++;
	}
	return out;
}

int main(int argc, char** argv) {
	printf("simulator: packets, keys, the rotary switch by SETUP\n");
	Capture cap;
	Simulator sim(cap);
	CHECK(sim.packetInterval() == 1000);                   // the default: one a second
	sim.setPacketInterval(250);                            // faster, so the test runs quickly
	sim.setSequenceSerial(false);                          // the fixed serial, checked below
	uint32_t now = 0;
	std::vector<std::vector<uint8_t>> raw;
	auto run = [&](uint32_t ms) { for (uint32_t i = 0; i < ms; i++, now++) sim.main(now); };
	auto key = [&](bt::Key k, bool lp = false) { uint8_t m[5]; bt::encodeKey(k, lp, m); for (uint8_t b : m) sim.feed(b); };
	auto last = [&]() { cap.bytes.clear(); run(600); auto f = frames(cap.bytes, &raw, nullptr); return f.empty() ? bt::Frame{} : f.back(); };

	run(2000);
	int echoes = 0;
	auto f = frames(cap.bytes, &raw, &echoes);
	CHECK(f.size() >= 6);                                  // every 250 ms
	bt::Frame x = f.back();
	CHECK(x.mode == 1 && x.range == 0 && x.serial == 12345 && x.calYearMonth == 2611);
	CHECK(x.digits > 11000 && x.digits < 13000);           // 1.2345 V +- 0.05, in 100 uV
	CHECK(x.status1 & bt::kS1Auto);

	cap.bytes.clear();
	key(bt::Key::Mode);
	echoes = 0; frames(cap.bytes, nullptr, &echoes);
	CHECK(echoes == 1);                                    // echoed
	x = last();
	CHECK(x.mode == 2 && (x.status1 & bt::kS1Ac));          // AC V
	printf("  ACV: range %d digits %d\n", x.range, x.digits);
	CHECK(x.digits > 22000 && x.digits < 24000 && x.range == 2);   // 230 V on 500.00 V: 23000 digits of 10 mV
	key(bt::Key::Mode);
	x = last();
	CHECK(x.mode == 1);                                    // AC+DC skipped: back to DC V
	key(bt::Key::Hold);
	x = last();
	CHECK(((x.status3 >> 2) & 3) == 1);
	key(bt::Key::Hold);
	key(bt::Key::Range);                                   // to manual
	x = last();
	CHECK(!(x.status1 & bt::kS1Auto) && x.range == 0);
	key(bt::Key::Range);                                   // 50 V
	x = last();
	CHECK(x.range == 1 && x.digits > 1100 && x.digits < 1300);
	key(bt::Key::Range, true);                             // auto again
	x = last();
	CHECK((x.status1 & bt::kS1Auto) && x.range == 0);
	key(bt::Key::Setup);                                   // the switch to mV
	x = last();
	CHECK(x.mode == 3 && x.range == 0);                    // DC mV, 50.000 mV: 12.3 mV
	CHECK(x.digits > 11000 && x.digits < 13000);
	key(bt::Key::Setup); key(bt::Key::Setup);              // Hz, then ohms
	x = last();
	printf("  ohms: mode %d range %d digits %d\n", x.mode, x.range, x.digits);
	CHECK(x.mode == 9 && x.range == 2 && x.digits > 32000 && x.digits < 34000);   // 3.3000 k
	key(bt::Key::Rel);
	x = last();
	CHECK(x.status2 & bt::kS2Rel);
	// the clock
	uint8_t t[15]; t[0] = 0xF8; uint8_t ck = 0; const uint8_t v[6] = { 26, 11, 1, 15, 42, 7 };
	for (int i = 0; i < 6; i++) { snprintf((char*)t + 1 + 2 * i, 3, "%02X", v[i]); ck ^= v[i]; }
	char cs[3]; snprintf(cs, 3, "%02X", ck); t[13] = cs[0]; t[14] = cs[1];
	for (uint8_t b : t) sim.feed(b);
	bt::Command c;
	CHECK(sim.clock(&c) && c.year == 26 && c.minute == 42);

	// The packet counter in the serial field: consecutive, one per packet.
	sim.setSequenceSerial(true);
	cap.bytes.clear();
	run(2000);
	auto seq = frames(cap.bytes, nullptr, nullptr);
	CHECK(seq.size() >= 7);
	bool consecutive = seq.size() > 1;
	for (size_t i = 1; i < seq.size(); i++) if (seq[i].serial != seq[i - 1].serial + 1) consecutive = false;
	CHECK(consecutive);
	sim.setSequenceSerial(false);

	// A slow link: while the last packet is still waiting to go, no new one is
	// made. When it has gone, the next is the latest reading, and the counter
	// carries on with no gap (nothing was dropped, only not made).
	{
		struct SlowSink : iByteSink {
			std::vector<uint8_t> bytes;
			size_t queued = 0;
			void write(const uint8_t* b, size_t n) override { bytes.insert(bytes.end(), b, b + n); queued += n; }
			size_t pending() const override { return queued; }
		} slow;
		Simulator s2(slow);
		s2.setPacketInterval(250);
		uint32_t t = 0;
		for (; t < 2000; t += 10) s2.main(t);
		CHECK(s2.packets() == 1);                          // held back after the first
		slow.queued = 0;                                   // it has gone
		for (; t < 2100; t += 10) s2.main(t);
		CHECK(s2.packets() == 2);                          // the next goes at once
		auto sf = frames(slow.bytes, nullptr, nullptr);
		CHECK(sf.size() == 2 && sf[1].serial == sf[0].serial + 1);
	}

	if (argc > 1) {
		FILE* fp = fopen(argv[1], "w");
		for (auto& p : raw) { for (uint8_t b : p) fprintf(fp, "%02X", b); fprintf(fp, "\n"); }
		fclose(fp);
		printf("  wrote %zu packets to %s\n", raw.size(), argv[1]);
	}
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
