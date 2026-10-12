/*
 * lwipopts.h
 *
 *  lwIP 2.2.1 for the GPS clock: UDP (the NTP server), DHCP and ICMP over
 *  the F207's Ethernet MAC, with FreeRTOS (lwIP's own FreeRTOS port,
 *  contrib/ports/freertos). No TCP yet (the web server is a later step),
 *  no netconn or socket API: the NTP server answers in the tcpip thread
 *  through the raw UDP API, as soon as the request is handed up.
 *
 *  Starting point: ST's NUCLEO-F207ZG LwIP_HTTP_Server_Netconn_RTOS
 *  example's lwipopts.h (STM32CubeF2), cut down.
 */

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#include "cmsis_os2.h"

/* ---- system ---- */
#define NO_SYS                          0
#define SYS_LIGHTWEIGHT_PROT            1
/* The raw API is called from other threads (the link poll, DHCP start)
   with LOCK_TCPIP_CORE(); ethernetif.c also holds it around the HAL's
   ETH calls, which share one handle and its lock with transmission. */
#define LWIP_TCPIP_CORE_LOCKING         1

/* ---- memory ---- */
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (6 * 1024)
#define MEMP_NUM_PBUF                   10
#define MEMP_NUM_UDP_PCB                4
#define PBUF_POOL_SIZE                  8
#define PBUF_POOL_BUFSIZE               1524

/* ---- protocols ---- */
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_UDP                        1
#define UDP_TTL                         64
#define LWIP_TCP                        0
#define LWIP_DHCP                       1
#define LWIP_DNS                        0
#define LWIP_IGMP                       0
#define LWIP_IPV6                       0
#define LWIP_STATS                      0

/* ---- the interface ---- */
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_STATUS_CALLBACK      1

/* ---- checksums: the MAC computes and checks IP, UDP, TCP and ICMP
   checksums (HAL ChecksumMode = ETH_CHECKSUM_BY_HARDWARE), so the NTP
   reply's checksum covers its transmit time stamp as sent ---- */
#define CHECKSUM_GEN_IP                 0
#define CHECKSUM_GEN_UDP                0
#define CHECKSUM_GEN_TCP                0
#define CHECKSUM_GEN_ICMP               0
#define CHECKSUM_CHECK_IP               0
#define CHECKSUM_CHECK_UDP              0
#define CHECKSUM_CHECK_TCP              0
#define CHECKSUM_CHECK_ICMP             0

/* ---- APIs ---- */
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

/* ---- the tcpip thread (lwIP's FreeRTOS port: stack sizes in bytes,
   priorities are FreeRTOS's, which CMSIS-RTOS2's equal) ---- */
#define TCPIP_THREAD_NAME               "tcpip"
#define TCPIP_THREAD_STACKSIZE          3072
#define TCPIP_THREAD_PRIO               osPriorityHigh
#define TCPIP_MBOX_SIZE                 8
#define DEFAULT_THREAD_STACKSIZE        1024

#endif /* LWIPOPTS_H */
