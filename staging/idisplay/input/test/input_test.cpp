// Host tests of KeyScanner and Buzzer.
#include <cstdio>
#include <vector>
#include "KeyScanner.h"
#include "Buzzer.h"

using namespace idisplay;
using A = KeyEvent::Action;

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

static std::vector<KeyEvent> run(KeyScanner& k, uint16_t down, int samples) {
	std::vector<KeyEvent> ev;
	for (int i = 0; i < samples; i++) { k.sample(down); KeyEvent e; if (k.poll(&e)) ev.push_back(e); }
	return ev;
}

static void testKeys() {
	printf("KeyScanner\n");
	static const uint16_t chords[] = { 0x28 };
	KeyScanner::Config c; c.chords = chords; c.chordCount = 1; c.repeatMask = 0x40;
	KeyScanner k(c);
	// a short press: nothing while down, Pressed on release
	CHECK(run(k, 0x10, 10).empty());
	auto e = run(k, 0, 5);
	CHECK(e.size() == 1 && e[0].keys == 0x10 && e[0].action == A::Pressed);
	// bounce shorter than the debounce: nothing
	k.sample(0x10); k.sample(0);
	CHECK(run(k, 0, 5).empty());
	// a long press: Held at 50 samples, nothing on release
	e = run(k, 0x01, 60);
	CHECK(e.size() == 1 && e[0].action == A::Held && e[0].keys == 0x01);
	CHECK(run(k, 0, 5).empty());
	// two keys that aren't a chord: nothing
	CHECK(run(k, 0x11, 80).empty() && run(k, 0, 5).empty());
	// the chord
	e = run(k, 0x28, 60);
	CHECK(e.size() == 1 && e[0].keys == 0x28 && e[0].action == A::Held);
	run(k, 0, 5);
	// a repeating key: Pressed at once, Repeat at 50, then every 31
	std::vector<KeyEvent> all;
	for (int i = 0; i < 2 + 50 + 31 * 2 + 1; i++) { k.sample(0x40); KeyEvent x; if (k.poll(&x)) all.push_back(x); }
	CHECK(all.size() == 4 && all[0].action == A::Pressed && all[1].action == A::Repeat && all[3].action == A::Repeat);
	CHECK(run(k, 0, 5).empty());
	// one-deep mailbox: two events, the newest kept
	run(k, 0x02, 3); k.sample(0); k.sample(0); k.sample(0);  // press 0x02 (not polled)
	k.sample(0x04); k.sample(0x04); k.sample(0x04); k.sample(0); k.sample(0); k.sample(0);
	KeyEvent x;
	CHECK(k.poll(&x) && x.keys == 0x04 && !k.poll(&x));
}

struct Pin { std::vector<std::pair<uint32_t, bool>>* log; uint32_t* now; void set(bool on) { log->push_back({*now, on}); } };

static void testBuzzer() {
	printf("Buzzer\n");
	std::vector<std::pair<uint32_t, bool>> log;
	uint32_t now = 0;
	Buzzer<Pin> b(Pin{&log, &now});
	log.clear();
	b.no();                                  // 2 x 50 ms, 50 ms apart
	for (now = 1000; now < 1400; now++) b.tick(now);
	CHECK(log.size() == 4);
	CHECK(log[0].first == 1000 && log[0].second && log[1].first == 1050 && !log[1].second);
	CHECK(log[2].first == 1100 && log[2].second && log[3].first == 1150 && !log[3].second);
	CHECK(!b.beeping());
	log.clear();
	b.ok();
	for (now = 2000; now < 2050; now++) b.tick(now);
	b.stop();
	CHECK(log.size() == 2 && log[1].second == false && !b.beeping());
}

int main() {
	testKeys();
	testBuzzer();
	if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
	printf("all passed\n");
	return 0;
}
