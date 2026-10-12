#pragma once
#include <cstdint>

// Plain value types shared by every network interface and chip
// driver. No RTOS, no HAL.

struct IpAddress {
    uint8_t b[4] = {0, 0, 0, 0};

    constexpr IpAddress() = default;
    constexpr IpAddress(uint8_t a, uint8_t b1, uint8_t c, uint8_t d) : b{a, b1, c, d} {}

    constexpr bool isZero() const { return b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0; }

    // "a.b.c.d" exactly (nothing before or after, each part 0..255).
    // Leaves out alone and returns false for anything else, such as a
    // host name.
    static bool parse(const char* s, IpAddress& out) {
        if (s == nullptr) return false;
        IpAddress a;
        for (int i = 0; i < 4; ++i) {
            if (*s < '0' || *s > '9') return false;
            unsigned v = 0;
            for (int d = 0; *s >= '0' && *s <= '9'; ++d, ++s) {
                if (d == 3) return false;
                v = v * 10 + static_cast<unsigned>(*s - '0');
            }
            if (v > 255) return false;
            a.b[i] = static_cast<uint8_t>(v);
            if (i < 3 && *s++ != '.') return false;
        }
        if (*s != 0) return false;
        out = a;
        return true;
    }

    // "a.b.c.d" into buf, which needs 16 bytes. Returns buf.
    char* format(char* buf) const {
        char* p = buf;
        for (int i = 0; i < 4; ++i) {
            const unsigned v = b[i];
            if (v >= 100) *p++ = static_cast<char>('0' + v / 100);
            if (v >= 10) *p++ = static_cast<char>('0' + v / 10 % 10);
            *p++ = static_cast<char>('0' + v % 10);
            if (i < 3) *p++ = '.';
        }
        *p = 0;
        return buf;
    }
    constexpr bool operator==(const IpAddress& o) const {
        return b[0] == o.b[0] && b[1] == o.b[1] && b[2] == o.b[2] && b[3] == o.b[3];
    }
    constexpr bool operator!=(const IpAddress& o) const { return !(*this == o); }
};

struct MacAddress {
    uint8_t b[6] = {0, 0, 0, 0, 0, 0};

    constexpr MacAddress() = default;
    constexpr MacAddress(uint8_t a, uint8_t b1, uint8_t c, uint8_t d, uint8_t e, uint8_t f)
        : b{a, b1, c, d, e, f} {}
};

// The interface's own address. With dhcp set, ip/subnet/gateway/dns/ntp
// are ignored here and come from a DHCP server instead; the address
// actually in use is reported back through
// iNetDeviceHost::addressChanged() either way. mac is ignored by chips
// that carry their own (most Wi-Fi modules). ntp: a time server on
// this network (DHCP option 42), all zeros for none.
struct NetConfig {
    MacAddress mac;
    bool       dhcp = false;
    IpAddress  ip;
    IpAddress  subnet;
    IpAddress  gateway;
    IpAddress  dns;
    IpAddress  ntp;
};
