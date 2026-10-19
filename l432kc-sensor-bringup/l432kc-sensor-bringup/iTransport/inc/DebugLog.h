#pragma once
// Debug log and debug pins for testing on hardware, read with a logic
// analyser (a Saleae: its async serial decoder for the log, digital
// channels for the pins) or any serial terminal.
//
// The log is text, one line per event:
//
//     <seq> <ms> <tag> <what>[ <value>...]\r\n
//     1042 53187 mtk3339 st 1 2
//
// seq counts every line made, so a gap in it means lines were dropped
// (the ring was full); ms is the port's clock (HAL_GetTick on STM32);
// tag says who (a driver, "bus", "dcl"...); what says what happened;
// the values are signed decimal numbers.
//
// Nothing here blocks. A log call formats the line on the stack (no
// printf), copies it into a ring under the port's lock (a few
// microseconds with interrupts masked, so interrupts may log too) and
// returns. poll(), called from a thread (the idle loop, a low priority
// task), hands what is queued to the debug UART's iTransport, two
// buffers in turn, since write() may send from the buffer after it
// returns. A line that does not fit in the ring is dropped and counted.
//
// The pins are for timing a UART line cannot show (sub-microsecond):
// DBG_PIN(id, level) calls the port's pin function, which on STM32
// writes the GPIO's BSRR. Which ids exist is up to the application's
// port; the hooks in this repository use DbgPin below.
//
// Everything is compiled in only when ITRANSPORT_DEBUG is defined above
// 0 (in the library build and in the application, since drivers are
// templates compiled in the application):
//   1  faults only: DBG_FAULT, DBG_PIN
//   2  + events: state changes, configuration steps (DBG_EVENT)
//   3  + per-transfer trace (DBG_TRACE): chatty, mind the UART's speed
// At 0 (the default) every macro is empty and its arguments are not
// evaluated. dbg::begin()/poll() still exist, so an application's setup
// code builds either way.
//
//     static dbg::Port port = Stm32DebugPort::make(pins, 4);   // hw/stm32
//     static Stm32HalUartTransport debugUart(&huart2);
//     dbg::begin(&debugUart, port);
//     ... in the idle loop or a low priority task:  dbg::poll();

#include <stddef.h>
#include <stdint.h>
#include <initializer_list>
#include "iTransport.h"

#ifndef ITRANSPORT_DEBUG
#define ITRANSPORT_DEBUG 0
#endif

// The ring's size, in bytes (a power of two).
#ifndef ITRANSPORT_DEBUG_RING
#define ITRANSPORT_DEBUG_RING 1024
#endif

namespace dbg {

// What the platform provides. Any of these may be null: no clock gives
// 0 ms, no lock is right only where nothing logs from an interrupt (a
// host test), no pin function ignores DBG_PIN.
struct Port {
    uint32_t (*nowMs)();                  // a millisecond clock, callable from interrupts
    uint32_t (*lock)();                   // masks interrupts, returns what to restore
    void     (*unlock)(uint32_t saved);
    void     (*pin)(uint8_t id, bool level);
};

// The debug pins the hooks here drive. 4 and up are the application's.
enum DbgPin : uint8_t {
    kPinBusIrq   = 0,   // pulsed in the bus interrupt that ends a transfer
    kPinTransfer = 1,   // high while a bus transfer is in flight
    kPinState    = 2,   // pulsed on every driver state change
    kPinFault    = 3    // pulsed on every fault
};

struct Stats {
    uint32_t lines;      // lines made (the last seq)
    uint32_t dropped;    // of those, dropped for want of room
    uint32_t bytesSent;  // handed to the UART
};

// out may be null: pins only. Both must outlive the logger. Calling it
// again replaces them (and keeps what is queued).
void begin(iTransport* out, const Port& port);

// One line. what and tag must be string literals (or outlive the call).
// At most kMaxValues values are written. Any context.
const uint8_t kMaxValues = 6;
void line(const char* tag, const char* what, const int32_t* values, size_t count);
inline void event(const char* tag, const char* what, std::initializer_list<int32_t> values) {
    line(tag, what, values.begin(), values.size());
}

// Sets a debug pin. Any context.
void pin(uint8_t id, bool level);
inline void pulse(uint8_t id) { pin(id, true); pin(id, false); }

// Sends what is queued, as far as the UART will take it. Thread only.
void poll();

Stats stats();

// For tests: forget everything (the ring, the counters, the port).
void reset();

} // namespace dbg

#if ITRANSPORT_DEBUG >= 1
#define DBG_FAULT(tag, what, ...) ::dbg::event((tag), (what), {__VA_ARGS__})
#define DBG_PIN(id, level)        ::dbg::pin((id), (level))
#define DBG_PULSE(id)             ::dbg::pulse((id))
#else
#define DBG_FAULT(tag, what, ...) do {} while (0)
#define DBG_PIN(id, level)        do {} while (0)
#define DBG_PULSE(id)             do {} while (0)
#endif

#if ITRANSPORT_DEBUG >= 2
#define DBG_EVENT(tag, what, ...) ::dbg::event((tag), (what), {__VA_ARGS__})
#else
#define DBG_EVENT(tag, what, ...) do {} while (0)
#endif

#if ITRANSPORT_DEBUG >= 3
#define DBG_TRACE(tag, what, ...) ::dbg::event((tag), (what), {__VA_ARGS__})
#else
#define DBG_TRACE(tag, what, ...) do {} while (0)
#endif
