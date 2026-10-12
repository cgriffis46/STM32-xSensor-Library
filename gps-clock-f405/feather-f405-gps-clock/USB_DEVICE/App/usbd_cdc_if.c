/**
  ******************************************************************************
  * @file    usbd_cdc_if.c
  * @brief   The CDC interface: line coding stored and echoed back, received
  *          bytes handed to CDC_RxFromIsr(), sends copied into one buffer.
  *          In the shape STM32CubeMX generates.
  ******************************************************************************
  */

#include "usbd_cdc_if.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
static uint8_t lineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};   /* 115200 8N1 */
static volatile uint8_t s_ready = 0;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

__attribute__((weak)) void CDC_RxFromIsr(const uint8_t *Buf, uint32_t Len)
{
  (void)Buf;
  (void)Len;
}

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  s_ready = 0;
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
  case CDC_SET_LINE_CODING:
    if (length >= sizeof(lineCoding))
      memcpy(lineCoding, pbuf, sizeof(lineCoding));
    s_ready = 1;
    break;
  case CDC_GET_LINE_CODING:
    memcpy(pbuf, lineCoding, sizeof(lineCoding));
    break;
  case CDC_SET_CONTROL_LINE_STATE:
    s_ready = 1;
    break;
  default:
    break;
  }
  return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  CDC_RxFromIsr(Buf, *Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  (void)Buf;
  (void)Len;
  (void)epnum;
  return (USBD_OK);
}

uint8_t CDC_Ready_FS(void)
{
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && s_ready;
}

uint8_t CDC_Transmit_FS(const uint8_t *Buf, uint16_t Len)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL || !CDC_Ready_FS())
    return USBD_FAIL;
  if (hcdc->TxState != 0)
    return USBD_BUSY;
  if (Len > APP_TX_DATA_SIZE)
    Len = APP_TX_DATA_SIZE;
  memcpy(UserTxBufferFS, Buf, Len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
