// Minimal lwIP configuration for Pico W HTTP server (NO_SYS mode).
#ifndef _LWIPOPTS_H_
#define _LWIPOPTS_H_

#define NO_SYS 1
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0

#define MEM_LIBC_MALLOC 0
#define MEM_ALIGNMENT 4
#define MEM_SIZE 16384

#define TCP_MSS 1460
#define TCP_SND_BUF (4 * TCP_MSS)
#define TCP_WND (4 * TCP_MSS)

// The dashboard uses short HTTP connections.  Keep enough PCB and segment
// entries for normal browser polling and brief reconnect bursts without
// rejecting otherwise valid connections while earlier sockets are retiring.
#define MEMP_NUM_TCP_PCB 12
#define MEMP_NUM_TCP_PCB_LISTEN 4
#define MEMP_NUM_TCP_SEG 32

#define LWIP_DHCP 1
#define LWIP_DNS 1
#define LWIP_NUM_NETIF_CLIENT_DATA 1
#define MEMP_NUM_SYS_TIMEOUT 16

#define LWIP_NETIF_HOSTNAME 1
#define LWIP_HTTPD 0

#define LWIP_STATS 1
#define MEM_STATS 1
#define MEMP_STATS 1
#define LINK_STATS 0

#endif
