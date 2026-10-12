/**
  ******************************************************************************
  * @file    usbd_cdc_if.h
  * @brief   The USB virtual COM port as the clock's console.
  ******************************************************************************
  */

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  256

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* Starts sending len bytes (copied; at most APP_TX_DATA_SIZE). USBD_BUSY
   while the last is still going, USBD_FAIL when no host has the port open. */
uint8_t CDC_Transmit_FS(const uint8_t *Buf, uint16_t Len);

/* A host has configured the device and set the line coding at least once. */
uint8_t CDC_Ready_FS(void);

/* Called from the USB interrupt with what the host typed. Weak: define it
   in the application. */
void CDC_RxFromIsr(const uint8_t *Buf, uint32_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
