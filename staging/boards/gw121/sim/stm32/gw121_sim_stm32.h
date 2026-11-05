/*
 * gw121_sim_stm32.h
 *
 *  The 121GW simulator on an STM32 with a BLE UART module (Adafruit
 *  Bluefruit LE UART Friend): the C API main.c calls. Bare metal, HAL UART
 *  in interrupt mode, nothing waits.
 *
 *    main.c, after MX_USARTx_UART_Init():   gw121SimStart(&huart1);
 *    in while (1):                          gw121SimLoop();
 *
 *  This file defines HAL_UART_RxCpltCallback, HAL_UART_TxCpltCallback and
 *  HAL_UART_ErrorCallback. If the project has its own, call
 *  gw121SimUartRxDone / TxDone / Error from them instead and build with
 *  GW121_SIM_NO_HAL_CALLBACKS.
 */
#ifndef GW121_SIM_STM32_H_
#define GW121_SIM_STM32_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void gw121SimStart(UART_HandleTypeDef* huart);
void gw121SimLoop(void);

/* How often a packet goes out, in ms (default 1000, at least 100). */
void gw121SimSetPacketInterval(uint32_t ms);
/* Packets and echoes dropped because the transmit ring was full: worth a
   look in the debugger (Live Expressions) when packets go missing. */
uint32_t gw121SimDropped(void);

void gw121SimUartRxDone(UART_HandleTypeDef* huart);
void gw121SimUartTxDone(UART_HandleTypeDef* huart);
void gw121SimUartError(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* GW121_SIM_STM32_H_ */
