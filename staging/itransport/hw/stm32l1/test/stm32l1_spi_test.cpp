// Host test of Stm32L1SpiPort and Stm32L1SpiBlockTransport against a
// simulated SPI peripheral (fake/stm32l1xx.h): each byte the port writes
// to DR is "shifted", the device's answer put in DR with RXNE, and the
// interrupt called, as the hardware would.

#include <cstdio>
#include <vector>
#include "Stm32L1SpiBlockTransport.h"

static int g_failures = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_failures++; } } while (0)

static SPI_TypeDef  g_spi;
static GPIO_TypeDef g_gpio;
static constexpr uint16_t kCsPin = 1u << 4;

struct Device {
    std::vector<uint8_t> mosi;
    uint8_t next = 0x10;            // MISO counts up from here
    bool csLowSeen = false;
    int overrunAt = -1;             // inject OVR at this byte
};
static Device g_dev;

static bool csLow() {
    // The last BSRR store says: reset half (bit 16+) low, set half high.
    return (g_gpio.BSRR & ((uint32_t)kCsPin << 16)) != 0;
}

// Plays the peripheral until the port has nothing in flight.
static void pump(Stm32L1SpiPort& port) {
    int guard = 0;
    while (port.active() && guard++ < 100000) {
        CHECK(csLow());
        g_dev.mosi.push_back((uint8_t)g_spi.DR);
        g_spi.DR = g_dev.next++;
        g_spi.SR |= SPI_SR_RXNE;
        if (g_dev.overrunAt >= 0 && (int)g_dev.mosi.size() == g_dev.overrunAt + 1) g_spi.SR |= SPI_SR_OVR;
        port.irq();
        g_spi.SR &= ~(SPI_SR_RXNE | SPI_SR_OVR);   // a read of DR (then SR) clears them
    }
}

using Xport = Stm32L1SpiBlockTransport<SpiBlockTransport>;   // bare metal, polling

static void testConfigure() {
    printf("configure\n");
    Stm32L1SpiPort port(&g_spi);
    port.configure(Stm32L1SpiPort::Mode::Mode3, 3);
    CHECK(g_spi.CR1 & SPI_CR1_SPE);
    CHECK(g_spi.CR1 & SPI_CR1_MSTR);
    CHECK(g_spi.CR1 & SPI_CR1_CPOL);
    CHECK(g_spi.CR1 & SPI_CR1_CPHA);
    CHECK(((g_spi.CR1 >> SPI_CR1_BR_Pos) & 7u) == 3u);
    port.configure(Stm32L1SpiPort::Mode::Mode0, 1);
    CHECK(!(g_spi.CR1 & (SPI_CR1_CPOL | SPI_CR1_CPHA)));
}

static void testReadWrite() {
    printf("header + read, header + write, under one chip-select each\n");
    Stm32L1SpiPort port(&g_spi);
    port.configure(Stm32L1SpiPort::Mode::Mode0, 3);
    Xport t(port, &g_gpio, kCsPin, nullptr);
    CHECK(!csLow());

    g_dev = Device();
    const uint8_t hdr[1] = { 0x3D };
    uint8_t rx[5] = {};
    CHECK(t.beginRead(hdr, 1, rx, 5));
    pump(port);
    CHECK(!t.isBusy());
    CHECK(!t.lastOpFailed());
    CHECK(!csLow());
    CHECK(g_dev.mosi.size() == 6);
    CHECK(g_dev.mosi[0] == 0x3D);
    for (int i = 1; i < 6; i++) CHECK(g_dev.mosi[i] == 0xFF);   // the fill byte
    for (int i = 0; i < 5; i++) CHECK(rx[i] == 0x11 + i);       // 0x10 came in with the header

    g_dev = Device();
    const uint8_t whdr[1] = { 0x40 };
    const uint8_t data[3] = { 0xA1, 0xB2, 0xC3 };
    CHECK(t.beginWrite(whdr, 1, data, 3));
    pump(port);
    CHECK(!t.isBusy());
    CHECK(!t.lastOpFailed());
    CHECK(g_dev.mosi.size() == 4);
    CHECK(g_dev.mosi[1] == 0xA1 && g_dev.mosi[3] == 0xC3);
    CHECK(!(g_spi.CR2 & SPI_CR2_RXNEIE));   // interrupts off when idle
}

static void testBusyAndOverrun() {
    printf("a second transfer is refused while one is in flight; OVR fails it\n");
    Stm32L1SpiPort port(&g_spi);
    port.configure(Stm32L1SpiPort::Mode::Mode0, 3);
    Xport t(port, &g_gpio, kCsPin, nullptr);
    g_dev = Device();
    const uint8_t hdr[1] = { 0x01 };
    uint8_t rx[8];
    CHECK(t.beginRead(hdr, 1, rx, 8));
    CHECK(!port.start(nullptr, rx, 1));
    g_dev.overrunAt = 3;
    pump(port);
    CHECK(!t.isBusy());
    CHECK(t.lastOpFailed());
    CHECK(port.errors() == 1);
    CHECK(!csLow());
    // and the next one works
    g_dev = Device();
    CHECK(t.beginRead(hdr, 1, rx, 2));
    pump(port);
    CHECK(!t.isBusy());
    CHECK(!t.lastOpFailed());
}

static void testStaleRx() {
    printf("a byte and OVR left over from before are cleared at the start\n");
    Stm32L1SpiPort port(&g_spi);
    port.configure(Stm32L1SpiPort::Mode::Mode0, 3);
    Xport t(port, &g_gpio, kCsPin, nullptr);
    g_spi.SR |= SPI_SR_RXNE | SPI_SR_OVR;   // the fake can't clear them on a read; start() bounds its loop
    g_dev = Device();
    const uint8_t hdr[1] = { 0x02 };
    uint8_t rx[2];
    CHECK(t.beginRead(hdr, 1, rx, 2));
    g_spi.SR &= ~(SPI_SR_RXNE | SPI_SR_OVR);
    pump(port);
    CHECK(!t.isBusy());
    CHECK(!t.lastOpFailed());
    CHECK(rx[0] == 0x11);
}

int main() {
    testConfigure();
    testReadWrite();
    testBusyAndOverrun();
    testStaleRx();
    if (g_failures) { printf("%d FAILED\n", g_failures); return 1; }
    printf("all passed\n");
    return 0;
}
