/**
  ******************************************************************************
  * @file           : app.cpp
  * @brief          : The SparkFun CAN-BUS shield's joystick, sent over CAN
  *
  *  A NUCLEO-L476RG with a SparkFun CAN-BUS shield (v13: an MCP2515 with a
  *  16 MHz crystal and an MCP2551 transceiver) reads the shield's joystick
  *  and sends its position on the bus, for a second node (an L432KC with an
  *  SSD1306) to show. The CAN controller is Claude-iTransport's non-blocking
  *  MCP2515 driver (iCan/mcp2515, see iTransport/SOURCE.txt) in a thread of
  *  its own; this file only reads the joystick and talks to it through the
  *  iCan interface.
  *
  *  On the bus, 500 kbit/s, 87.5 % sample point:
  *    ID 0x100 (standard), 1 byte: bit 0 up, bit 1 down, bit 2 left,
  *    bit 3 right, bit 4 click (1 = pressed; the rest 0).
  *    Sent when the (debounced) position changes, and every second as a
  *    heartbeat, so a receiver that hears nothing for 2 s can say so.
  *    Only one frame is ever waiting to go: a change while one is still
  *    waiting is sent after it, so the newest position goes next.
  *
  *  The console (USART2, the ST-LINK's virtual COM port, 115200 8N1) shows
  *  the bit timing at start, each change of position, each frame received
  *  (none, unless another node sends or APP_CAN_LOOPBACK is set) and the
  *  controller's state every 5 s. Keys: s (status now), h (help).
  *
  *  LEDs (the shield's): LED1 (D7, PA9) toggles on each frame acknowledged
  *  by another node; LED2 (D8, PA8) is lit while the controller isn't
  *  error-active. Alone on the bus nobody acknowledges: the MCP2515 sends
  *  each frame again and again and goes error-passive (TEC 128), LED2 on.
  *  That is expected until the second node is connected.
  *
  *  APP_CAN_LOOPBACK (set it to 1 in the project's defines) puts the
  *  MCP2515 in loopback mode: its own frames come back to it and nothing
  *  goes on the bus. A test of the shield and the SPI alone.
  *
  *  freertos.c is CubeMX's: its USER CODE block only calls
  *  app_default_task() (app.h), so regenerating code keeps it.
  *
  *  Wiring (the shield on the Nucleo's Arduino headers):
  *    SPI1   SCK D13 (PA5), MISO D12 (PA6), MOSI D11 (PA7)
  *    CAN_CS D10 (PB6)  CAN_INT D2 (PA10), EXTI falling edge
  *    JOY_UP A1 (PA1), JOY_LEFT A2 (PA4), JOY_DOWN A3 (PB0),
  *    JOY_CLICK A4 (PC1), JOY_RIGHT A5 (PC0): to ground when pressed,
  *    the MCU's pull-ups (the shield has none)
  *    CAN_H, CAN_L on the shield's screw terminal (or DB9), 120 ohm at
  *    each end of the bus
  *  The shield runs the MCP2515 on 5 V: its inputs want 3.5 V for a high,
  *  so the L476's 3.3 V SCK/MOSI/CS are out of its specification (they
  *  often work), and MISO and INT come back at 5 V (PA6 and PA10 must be
  *  5 V tolerant pins).
  *
  *  Not run on the board yet.
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"
#include "app.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <atomic>
#include "Stm32HalSpiBlockTransport.h"
#include "Stm32HalUartTransport.h"
#include "xmcp2515.h"

// Defined by CubeMX
extern "C" {
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
}

#ifndef APP_CAN_LOOPBACK
#define APP_CAN_LOOPBACK        0
#endif

#define CAN_BITRATE             500000UL
#define MCP2515_OSC_HZ          16000000UL // the shield's crystal
#define JOY_CAN_ID              0x100u

#define JOY_UP                  0x01u
#define JOY_DOWN                0x02u
#define JOY_LEFT                0x04u
#define JOY_RIGHT               0x08u
#define JOY_CLICK               0x10u

#define JOY_SAMPLE_MS           10UL       // the joystick is read this often
#define JOY_STABLE_SAMPLES      2          // and taken once it reads the same this many times
#define HEARTBEAT_MS            1000UL
#define STATUS_MS               5000UL

#define CAN_STACK_WORDS         384
#define CONSOLE_RING            1024

typedef xmcp2515<Stm32HalSpiBlockTransport> Can;

// Set once the driver exists; INT's interrupt may come before that.
static std::atomic<Can *> g_can{nullptr};

// ---- the console ----------------------------------------------------------
// Written by the default task only, and handed to USART2 by it too, a piece
// at a time, never waiting for the UART: no lock is needed.
static char g_console[CONSOLE_RING];
static uint32_t g_conHead = 0, g_conTail = 0;
static uint32_t g_conDropped = 0;
static uint8_t g_conChunk[64];             // sent from here by interrupt: static
static uint32_t g_conInFlight = 0;         // bytes of g_conChunk being sent
static Stm32HalUartTransport *g_conPort = nullptr;

static void consolePrint(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// printf to the console; a line that doesn't fit is dropped and counted
static void consolePrint(const char *fmt, ...)
{
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n <= 0)
    return;
  if (n >= (int) sizeof(line))
    n = sizeof(line) - 1;
  if (g_conHead - g_conTail + (uint32_t) n > CONSOLE_RING) {
    ++g_conDropped;
    return;
  }
  for (int i = 0; i < n; ++i)
    g_console[(g_conHead + i) % CONSOLE_RING] = line[i];
  g_conHead += n;
}

// Starts the next piece once the last one is out. Never waits.
static void consolePump(void)
{
  if (g_conPort == nullptr || huart2.gState != HAL_UART_STATE_READY)
    return;
  g_conTail += g_conInFlight;              // the last piece is out
  g_conInFlight = 0;
  const uint32_t avail = g_conHead - g_conTail;
  if (avail == 0)
    return;
  const uint32_t len = avail < sizeof(g_conChunk) ? avail : sizeof(g_conChunk);
  for (uint32_t i = 0; i < len; ++i)
    g_conChunk[i] = (uint8_t) g_console[(g_conTail + i) % CONSOLE_RING];
  if (g_conPort->write(g_conChunk, len))
    g_conInFlight = len;
}

// The last key typed (interrupt context); 0 once taken
static std::atomic<uint8_t> g_key{0};
struct ConsoleKeys : iTransportRxSink {
  void onByteReceived(uint8_t b) override { g_key.store(b, std::memory_order_release); }
};
static ConsoleKeys consoleKeys;

// The HAL SPI and UART interrupt callbacks are in iTransport/hw/stm32/src
// (Stm32SpiItCallbacks.cpp, Stm32UartItCallbacks.cpp): don't define them here.

// INT's falling edge (CAN_INT, PA10, EXTI15_10)
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == CAN_INT_Pin) {
    Can *can = g_can.load(std::memory_order_acquire);
    if (can != nullptr)
      can->onInt(HAL_GetTick());
  }
}

// The driver's own thread: it sleeps inside main() until INT, a request or
// its next poll.
static void canTask(void *argument)
{
  Can *can = static_cast<Can *>(argument);
  for (;;)
    can->main(HAL_GetTick());
}

// ---- the joystick ---------------------------------------------------------
static bool pressed(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;   // to ground when pressed
}

static uint8_t readJoystick(void)
{
  uint8_t v = 0;
  if (pressed(JOY_UP_GPIO_Port, JOY_UP_Pin))       v |= JOY_UP;
  if (pressed(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin))   v |= JOY_DOWN;
  if (pressed(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin))   v |= JOY_LEFT;
  if (pressed(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin)) v |= JOY_RIGHT;
  if (pressed(JOY_CLICK_GPIO_Port, JOY_CLICK_Pin)) v |= JOY_CLICK;
  return v;
}

// "up+left", "click", "centre"
static const char *joyName(uint8_t v, char *buf, size_t len)
{
  static const char *const names[] = {"up", "down", "left", "right", "click"};
  size_t n = 0;
  buf[0] = 0;
  for (int i = 0; i < 5; ++i) {
    if (v & (1u << i))
      n += (size_t) snprintf(buf + n, n < len ? len - n : 0, "%s%s", n ? "+" : "", names[i]);
  }
  if (n == 0)
    snprintf(buf, len, "centre");
  return buf;
}

// ---- the application ------------------------------------------------------
static void printHelp(void)
{
  consolePrint("\r\nNUCLEO-L476RG + SparkFun CAN-BUS shield: joystick on CAN ID 0x%03X, %lu kbit/s%s\r\n"
               "keys: s status, h help\r\n\r\n",
               JOY_CAN_ID, (unsigned long) (CAN_BITRATE / 1000), APP_CAN_LOOPBACK ? " (loopback)" : "");
}

static void printStatus(const Can &can)
{
  const can::Stats &st = can.stats();
  consolePrint("can %s  tec %u rec %u  tx %lu rx %lu  pending %u  dropped %lu overflows %lu  "
               "bus-off %lu restarts %lu faults %lu%s\r\n",
               can::stateName(can.state()), can.txErrorCount(), can.rxErrorCount(),
               (unsigned long) st.txFrames, (unsigned long) st.rxFrames, can.txPending(),
               (unsigned long) st.rxDropped, (unsigned long) st.rxOverflows,
               (unsigned long) st.busOffs, (unsigned long) st.restarts, (unsigned long) st.faults,
               g_conDropped ? "  (console lines dropped)" : "");
}

/**
  * @brief  The default task's body (freertos.c's StartDefaultTask calls it):
  *         starts the CAN thread, then reads the joystick every 10 ms and
  *         sends it, prints what happens, and runs the LEDs.
  */
void app_default_task(void *argument)
{
  (void) argument;
  static Stm32HalUartTransport port(&huart2);
  port.setRxSink(consoleKeys);
  g_conPort = &port;
  printHelp();

  // SPI1's mutex: the transport takes it for each transfer
  osMutexId_t spi1Mutex = osMutexNew(nullptr);

  mcp2515_param_t param = mcp2515_default_param(MCP2515_OSC_HZ);
  param.intPin = true;
  param.pollMs = 10;                       // a backstop: INT does the work
  static Can can(param, &hspi1, CAN_CS_GPIO_Port, CAN_CS_Pin, spi1Mutex);
  g_can.store(&can, std::memory_order_release);

  osThreadAttr_t canAttr;
  memset(&canAttr, 0, sizeof(canAttr));
  canAttr.name = "can";
  canAttr.stack_size = CAN_STACK_WORDS * 4;
  canAttr.priority = (osPriority_t) osPriorityAboveNormal;
  if (spi1Mutex == nullptr || osThreadNew(canTask, &can, &canAttr) == nullptr)
    consolePrint("can: no memory for its thread or mutex (FreeRTOS heap)\r\n");

  can::Config cfg = can::defaultConfig(CAN_BITRATE);
  cfg.autoRestart = true;                  // after a bus-off, try again in a second
  cfg.busOffRestartMs = 1000;
#if APP_CAN_LOOPBACK
  cfg.mode = can::Mode::Loopback;
#endif
  if (can.start(cfg)) {
    const mcp251x::Timing &t = can.timing();
    consolePrint("can: %lu MHz crystal, prescaler %u, %u TQ (prop %u ps1 %u ps2 %u sjw %u), sample point %u.%u %%\r\n",
                 (unsigned long) (MCP2515_OSC_HZ / 1000000), t.prescaler, t.tq(), t.propSeg, t.ps1, t.ps2,
                 t.sjw, t.samplePoint() / 10, t.samplePoint() % 10);
  } else {
    consolePrint("can: the MCP2515 can't make %lu bit/s from its crystal\r\n", (unsigned long) CAN_BITRATE);
  }

  uint8_t raw = 0xFF, rawCount = 0;        // the last sample, and how often in a row
  uint8_t joy = 0;                         // the debounced position
  bool joyKnown = false;
  bool dirty = true;                       // a position not sent yet
  uint32_t lastSentMs = 0;
  uint32_t lastStatusMs = HAL_GetTick();
  uint32_t lastTx = 0;
  can::State lastState = can::State::Stopped;
  uint32_t wake = osKernelGetTickCount();

  for (;;)
  {
    const uint32_t now = HAL_GetTick();

    // The joystick, debounced
    const uint8_t r = readJoystick();
    if (r == raw) {
      if (rawCount < JOY_STABLE_SAMPLES)
        ++rawCount;
    } else {
      raw = r;
      rawCount = 1;
    }
    if (rawCount >= JOY_STABLE_SAMPLES && (!joyKnown || raw != joy)) {
      char name[40];
      joy = raw;
      joyKnown = true;
      dirty = true;
      consolePrint("joystick %s (0x%02X)\r\n", joyName(joy, name, sizeof(name)), joy);
    }

    // Sent on a change and as a heartbeat, one frame waiting at most
    if (joyKnown && (dirty || now - lastSentMs >= HEARTBEAT_MS) && can.txPending() == 0) {
      const can::Frame f = can::frame(JOY_CAN_ID, false, &joy, 1);
      if (can.send(f)) {
        dirty = false;
        lastSentMs = now;
      }
    }

    // LED1: a frame acknowledged; LED2: not error-active
    const uint32_t tx = can.stats().txFrames;
    if (tx != lastTx) {
      lastTx = tx;
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    const can::State s = can.state();
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s == can::State::ErrorActive ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (s != lastState) {
      consolePrint("can: %s -> %s\r\n", can::stateName(lastState), can::stateName(s));
      lastState = s;
    }

    // Frames from the bus (or our own, in loopback)
    can::Frame f;
    while (can.receive(&f)) {
      char data[3 * 8 + 1];
      size_t n = 0;
      data[0] = 0;
      for (uint8_t i = 0; i < f.dlc && !f.rtr; ++i)
        n += (size_t) snprintf(data + n, sizeof(data) - n, "%s%02X", i ? " " : "", f.data[i]);
      consolePrint("rx %s0x%0*lX%s [%u] %s\r\n", f.extended ? "x" : "", f.extended ? 8 : 3,
                   (unsigned long) f.id, f.rtr ? " rtr" : "", f.dlc, data);
    }

    // The console
    const uint8_t key = g_key.exchange(0, std::memory_order_acq_rel);
    if (key == 's' || key == 'S')
      printStatus(can);
    else if (key == 'h' || key == 'H' || key == '?')
      printHelp();
    if (now - lastStatusMs >= STATUS_MS) {
      lastStatusMs = now;
      printStatus(can);
    }
    consolePump();

    wake += JOY_SAMPLE_MS * osKernelGetTickFreq() / 1000u;
    if ((int32_t) (wake - osKernelGetTickCount()) > 0)
      osDelayUntil(wake);
    else
      wake = osKernelGetTickCount();       // fell behind: don't try to catch up
  }
}
