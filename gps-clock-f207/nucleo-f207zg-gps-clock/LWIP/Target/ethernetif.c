/*
 * ethernetif.c
 *
 *  lwIP's interface to the F207's Ethernet MAC, adapted from ST's
 *  NUCLEO-F207ZG LwIP_HTTP_Server_Netconn_RTOS example (STM32CubeF2,
 *  Copyright (c) 2017 STMicroelectronics, BSD-3) for CMSIS-RTOS2 and this
 *  application. The frame copying (low_level_output, low_level_input) is
 *  ST's, on the F2 HAL's legacy ETH API (chained DMA descriptors, the
 *  receive interrupt waking the input thread). What changed:
 *
 *  - CMSIS-RTOS2 (osSemaphoreNew, osThreadNew) instead of CMSIS-RTOS v1.
 *  - The MAC address comes from the application (the MCU's unique ID).
 *  - HAL_ETH_Init() is called once. ST's waits up to 10 s (link, then
 *    autonegotiation) with the HAL's busy delays, and a later cable would
 *    need it again, which would reset the MAC and with it the IEEE 1588
 *    system time that is this board's clock (ptp_clock.h). So it is
 *    called with autonegotiation off (it forces 100M full duplex, at
 *    once), autonegotiation is turned back on in the PHY, and
 *    ethernetif_link_poll() follows the link: speed and duplex into the
 *    MAC with HAL_ETH_ConfigMAC(), which leaves the DMA and the system
 *    time alone, and netif_set_link_up/down().
 *  - The HAL's ETH calls share one handle (and its __HAL_LOCK) between
 *    transmission (the tcpip thread), reception (the input thread) and the
 *    PHY reads (the link poll); a call that meets the lock held fails with
 *    HAL_BUSY, which ST's example ignores. Here all three run with lwIP's
 *    core lock held (the tcpip thread always holds it), so they never
 *    meet.
 *  - The PTP clock is enabled with the MAC's (ETHMACPTPEN).
 */

#include "ethernetif.h"
#include <string.h>
#include "cmsis_os.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"

#define IFNAME0 'e'
#define IFNAME1 'n'
#define INPUT_THREAD_STACK_BYTES  1024

/* The LAN8742A's PHY address on the NUCLEO-F207ZG (ST's example) */
#ifndef LAN8742A_PHY_ADDRESS
#define LAN8742A_PHY_ADDRESS 0x00U
#endif

ETH_DMADescTypeDef DMARxDscrTab[ETH_RXBUFNB] __attribute__((aligned(4)));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TXBUFNB] __attribute__((aligned(4)));
static uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((aligned(4)));

ETH_HandleTypeDef heth;

static osSemaphoreId_t s_rxSem = NULL;
static uint8_t s_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };
static bool s_macOk = false;
static volatile uint32_t s_mbps = 0;
static volatile bool s_fullDuplex = false;

static void ethernetif_input(void *argument);

/* ---- MSP: pins, clocks, interrupt (ST's) ---- */

void HAL_ETH_MspInit(ETH_HandleTypeDef *h)
{
  GPIO_InitTypeDef gpio = {0};
  (void)h;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* RMII on the NUCLEO-F207ZG (ST's example):
       REF_CLK PA1, MDIO PA2, MDC PC1, CRS_DV PA7, RXD0 PC4, RXD1 PC5,
       RXER PG2, TX_EN PG11, TXD0 PG13, TXD1 PB13 */
  gpio.Speed = GPIO_SPEED_HIGH;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Alternate = GPIO_AF11_ETH;
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOG, &gpio);

  /* 7: below configMAX_SYSCALL_INTERRUPT_PRIORITY (5), as the callback
     releases a semaphore */
  HAL_NVIC_SetPriority(ETH_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);

  __HAL_RCC_ETH_CLK_ENABLE();
  __HAL_RCC_ETHMACPTP_CLK_ENABLE();
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *h)
{
  (void)h;
  if (s_rxSem != NULL)
    osSemaphoreRelease(s_rxSem);
}

/* ---- lwIP -> MAC ---- */

void ethernetif_set_mac(const uint8_t mac[6])
{
  memcpy(s_mac, mac, 6);
}

bool ethernetif_mac_ok(void)
{
  return s_macOk;
}

static void low_level_init(struct netif *netif)
{
  heth.Instance = ETH;
  heth.Init.MACAddr = s_mac;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;   /* see the top of the file */
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
  heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;

  s_macOk = HAL_ETH_Init(&heth) == HAL_OK;
  /* Autonegotiation back on; ethernetif_link_poll() waits for it */
  (void)HAL_ETH_WritePHYRegister(&heth, PHY_BCR, PHY_AUTONEGOTIATION | PHY_RESTART_AUTONEGOTIATION);

  HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

  netif->hwaddr_len = ETH_HWADDR_LEN;
  memcpy(netif->hwaddr, s_mac, 6);
  netif->mtu = 1500;
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  s_rxSem = osSemaphoreNew(1, 0, NULL);
  const osThreadAttr_t attr = {
    .name = "ethif",
    .stack_size = INPUT_THREAD_STACK_BYTES,
    .priority = osPriorityRealtime,
  };
  osThreadNew(ethernetif_input, netif, &attr);

  HAL_ETH_Start(&heth);
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  err_t errval;
  struct pbuf *q;
  uint8_t *buffer = (uint8_t *)(heth.TxDesc->Buffer1Addr);
  __IO ETH_DMADescTypeDef *DmaTxDesc;
  uint32_t framelength = 0;
  uint32_t bufferoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t payloadoffset = 0;
  (void)netif;

  DmaTxDesc = heth.TxDesc;
  bufferoffset = 0;

  /* copy frame from pbufs to driver buffers */
  for (q = p; q != NULL; q = q->next)
  {
    /* Is this buffer available? If not, goto error */
    if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
    {
      errval = ERR_USE;
      goto error;
    }

    byteslefttocopy = q->len;
    payloadoffset = 0;

    /* Check if the length of data to copy is bigger than Tx buffer size */
    while ((byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE)
    {
      memcpy((uint8_t *)((uint8_t *)buffer + bufferoffset), (uint8_t *)((uint8_t *)q->payload + payloadoffset),
             (ETH_TX_BUF_SIZE - bufferoffset));

      DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);
      if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
      {
        errval = ERR_USE;
        goto error;
      }

      buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);
      byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
      payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
      framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
      bufferoffset = 0;
    }

    memcpy((uint8_t *)((uint8_t *)buffer + bufferoffset), (uint8_t *)((uint8_t *)q->payload + payloadoffset),
           byteslefttocopy);
    bufferoffset = bufferoffset + byteslefttocopy;
    framelength = framelength + byteslefttocopy;
  }

  /* Prepare transmit descriptors to give to DMA */
  HAL_ETH_TransmitFrame(&heth, framelength);
  errval = ERR_OK;

error:
  /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
  if ((heth.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
  {
    heth.Instance->DMASR = ETH_DMASR_TUS;
    heth.Instance->DMATPDR = 0;
  }
  return errval;
}

/* ---- MAC -> lwIP ---- */

static struct pbuf *low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL, *q = NULL;
  uint16_t len = 0;
  uint8_t *buffer;
  __IO ETH_DMADescTypeDef *dmarxdesc;
  uint32_t bufferoffset = 0;
  uint32_t payloadoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t i = 0;
  (void)netif;

  if (HAL_ETH_GetReceivedFrame_IT(&heth) != HAL_OK)
    return NULL;

  len = heth.RxFrameInfos.length;
  buffer = (uint8_t *)heth.RxFrameInfos.buffer;

  if (len > 0)
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

  if (p != NULL)
  {
    dmarxdesc = heth.RxFrameInfos.FSRxDesc;
    bufferoffset = 0;

    for (q = p; q != NULL; q = q->next)
    {
      byteslefttocopy = q->len;
      payloadoffset = 0;

      while ((byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE)
      {
        memcpy((uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)buffer + bufferoffset),
               (ETH_RX_BUF_SIZE - bufferoffset));

        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
        buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);

        byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
        payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
        bufferoffset = 0;
      }

      memcpy((uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)buffer + bufferoffset),
             byteslefttocopy);
      bufferoffset = bufferoffset + byteslefttocopy;
    }
  }

  /* Release descriptors to DMA: set the Own bit in each */
  dmarxdesc = heth.RxFrameInfos.FSRxDesc;
  for (i = 0; i < heth.RxFrameInfos.SegCount; i++)
  {
    dmarxdesc->Status |= ETH_DMARXDESC_OWN;
    dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
  }
  heth.RxFrameInfos.SegCount = 0;

  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
  {
    heth.Instance->DMASR = ETH_DMASR_RBUS;
    heth.Instance->DMARPDR = 0;
  }
  return p;
}

/* Woken by the receive interrupt; hands every waiting frame to lwIP
   (tcpip_input() queues it for the tcpip thread). */
static void ethernetif_input(void *argument)
{
  struct netif *netif = (struct netif *)argument;
  struct pbuf *p;

  for (;;)
  {
    if (osSemaphoreAcquire(s_rxSem, osWaitForever) != osOK)
      continue;
    do
    {
      LOCK_TCPIP_CORE();
      p = low_level_input(netif);
      UNLOCK_TCPIP_CORE();
      if (p != NULL && netif->input(p, netif) != ERR_OK)
        pbuf_free(p);
    } while (p != NULL);
  }
}

err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
#if LWIP_NETIF_HOSTNAME
  netif->hostname = "gps-ntp";
#endif
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;
  low_level_init(netif);
  return ERR_OK;
}

/* ---- the link ---- */

void ethernetif_link_poll(struct netif *netif)
{
  uint32_t bsr = 0, sr = 0;
  LOCK_TCPIP_CORE();
  /* BSR's link bit latches low: the second read is the link now */
  bool ok = HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &bsr) == HAL_OK
            && HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &bsr) == HAL_OK;
  const bool up = ok && (bsr & PHY_LINKED_STATUS) != 0 && (bsr & PHY_AUTONEGO_COMPLETE) != 0;
  if (up && !netif_is_link_up(netif))
  {
    if (HAL_ETH_ReadPHYRegister(&heth, PHY_SR, &sr) == HAL_OK)
    {
      /* The LAN8742A's special status register (31): what was negotiated */
      heth.Init.DuplexMode = (sr & PHY_DUPLEX_STATUS) ? ETH_MODE_FULLDUPLEX : ETH_MODE_HALFDUPLEX;
      heth.Init.Speed = (sr & PHY_SPEED_STATUS) ? ETH_SPEED_10M : ETH_SPEED_100M;
      (void)HAL_ETH_ConfigMAC(&heth, NULL);
      s_mbps = (sr & PHY_SPEED_STATUS) ? 10 : 100;
      s_fullDuplex = (sr & PHY_DUPLEX_STATUS) != 0;
      netif_set_link_up(netif);
    }
  }
  else if (!up && netif_is_link_up(netif))
  {
    s_mbps = 0;
    netif_set_link_down(netif);
  }
  UNLOCK_TCPIP_CORE();
}

void ethernetif_link_mode(uint32_t *mbps, bool *fullDuplex)
{
  *mbps = s_mbps;
  *fullDuplex = s_fullDuplex;
}

/* ---- lwIP's port hooks (cc.h) ---- */

uint32_t lwip_port_rand(void)
{
  /* xorshift32, seeded from the unique ID and the tick: DHCP's
     transaction IDs need only differ between boards and boots */
  static uint32_t s = 0;
  if (s == 0)
    s = (HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2() ^ (SysTick->VAL << 16) ^ HAL_GetTick()) | 1u;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

volatile const char *lwip_assert_msg = NULL;
volatile const char *lwip_assert_file = NULL;
volatile int lwip_assert_line = 0;

void lwip_port_assert(const char *msg, int line, const char *file)
{
  lwip_assert_msg = msg;
  lwip_assert_file = file;
  lwip_assert_line = line;
  __disable_irq();
  for (;;)
  {
  }
}
