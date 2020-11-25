#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define NO_SYS                         0

#define SYS_LIGHTWEIGHT_PROT           1

/* ---------- TCP ---------- */
#define LWIP_TCP                       1
#define TCP_TTL                        255

#define TCP_QUEUE_OOSEQ                1

#define TCP_MSS                        (1500 - 40)
#define TCP_SND_BUF                    (4*TCP_MSS)
#define TCP_SND_QUEUELEN               (2*TCP_SND_BUF/TCP_MSS)
#define TCP_WND                        (2*TCP_MSS)

/* ---------- Memory options ---------- */
#define MEM_ALIGNMENT                  4
#define MEM_SIZE                       (20*1024)
#define MEMP_NUM_PBUF                  20
#define MEMP_NUM_RAW_PCB               4
#define MEMP_NUM_UDP_PCB               4
#define MEMP_NUM_TCP_PCB               (TCP_WND + TCP_SND_BUF)/TCP_MSS
#define MEMP_NUM_TCP_PCB_LISTEN        1
#define MEMP_NUM_TCP_SEG               20
#define MEMP_NUM_SYS_TIMEOUT           8
#define MEMP_NUM_NETBUF                10
#define MEMP_NUM_NETCONN               10

/* ---------- Pbuf ---------- */
#define PBUF_POOL_SIZE                 20
#define PBUF_POOL_BUFSIZE              512


/* ---------- ICMP ---------- */
#define LWIP_ICMP                      1

/* ---------- DNS ----------- */
#define LWIP_DNS                       1

/* ---------- DHCP ---------- */
#define LWIP_DHCP                      1
#define LWIP_NETIF_HOSTNAME            1

/* ---------- UDP ----------- */
#define LWIP_UDP                       1
#define UDP_TTL                        255
#define LWIP_UDPLITE                   1

/* ---------- Statistics options ---------- */
#define LWIP_STATS                     0
#define LWIP_PROVIDE_ERRNO             1

/* ---------- link callback options ---------- */
#define LWIP_NETIF_LINK_CALLBACK       1

/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
*/
#define CHECKSUM_BY_HARDWARE

#ifdef CHECKSUM_BY_HARDWARE
#define CHECKSUM_GEN_IP                0
#define CHECKSUM_GEN_UDP               0
#define CHECKSUM_GEN_TCP               0
#define CHECKSUM_CHECK_IP              0
#define CHECKSUM_CHECK_UDP             0
#define CHECKSUM_CHECK_TCP             0
#define CHECKSUM_GEN_ICMP              0
#else
#define CHECKSUM_GEN_IP                1
#define CHECKSUM_GEN_UDP               1
#define CHECKSUM_GEN_TCP               1
#define CHECKSUM_CHECK_IP              1
#define CHECKSUM_CHECK_UDP             1
#define CHECKSUM_CHECK_TCP             1
#define CHECKSUM_GEN_ICMP              1
#endif

/*
   -------------------------------------------
   ---------- SequentialAPI----------
   -------------------------------------------
*/
#define LWIP_NETCONN                   1

/*
   ----------------------------------
   ---------- Socket API----------
   ----------------------------------
*/
#define LWIP_SOCKET                    1

/*
   ----------------------------------
   -------- RCV/SND Time Out --------
   ----------------------------------
*/
#define LWIP_SO_RCVTIMEO               1
#define LWIP_SO_SNDTIMEO               1

#define LWIP_TCP_KEEPALIVE             1
#define TCP_KEEPIDLE_DEFAULT           5000UL
#define TCP_KEEPINTVL_DEFAULT          1000UL
#define TCP_KEEPCNT_DEFAULT            5UL

/*
   ---------------------------------
   ---------- OS options ----------
   ---------------------------------
*/
#define TCPIP_THREAD_NAME              "TCP/IP"
#define TCPIP_THREAD_STACKSIZE         3*1024
#define TCPIP_MBOX_SIZE                20
#define DEFAULT_UDP_RECVMBOX_SIZE      20
#define DEFAULT_TCP_RECVMBOX_SIZE      20
#define DEFAULT_ACCEPTMBOX_SIZE        20
#define DEFAULT_THREAD_STACKSIZE       1024
#define TCPIP_THREAD_PRIO              osPriorityAboveNormal

#define TASK_DHCP_STACK                1024

/*
   --------------------------------------
   ---------- Lwip DEBUG----------
   --------------------------------------
*/
//#define LWIP_DEBUG

#define TASK_USERTCP_STACK             3*1024


#endif /* __LWIPOPTS_H__ */
