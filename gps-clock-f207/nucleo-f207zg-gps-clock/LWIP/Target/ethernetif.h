/*
 * ethernetif.h
 *
 *  lwIP's interface to the F207's Ethernet MAC and the NUCLEO-F207ZG's
 *  LAN8742A PHY (RMII, PHY address 0). See ethernetif.c.
 */

#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include <stdbool.h>
#include <stdint.h>
#include "lwip/err.h"
#include "lwip/netif.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ETH_HandleTypeDef heth;

/* netif_add()'s init function. The MAC address is mac[6]. */
err_t ethernetif_init(struct netif *netif);
void ethernetif_set_mac(const uint8_t mac[6]);

/* HAL_ETH_Init()'s result: false if the MAC didn't come out of its
   software reset (no 50 MHz reference clock from the PHY?). */
bool ethernetif_mac_ok(void);

/* Reads the PHY and brings the link up (speed and duplex from the
   LAN8742A's autonegotiation) or down. Every 250 ms or so, from one
   thread, without the core lock held (it takes it). */
void ethernetif_link_poll(struct netif *netif);

/* The link as last configured: 10 or 100 Mbit/s, full duplex or not. */
void ethernetif_link_mode(uint32_t *mbps, bool *fullDuplex);

#ifdef __cplusplus
}
#endif

#endif /* ETHERNETIF_H */
