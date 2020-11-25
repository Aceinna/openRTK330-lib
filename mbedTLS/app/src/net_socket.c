#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <string.h>
#include <stdint.h>
#if defined(MBEDTLS_NET_C)

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdlib.h>
#endif

#include "mbedtls/net_sockets.h"

#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"


#include "ethernetif.h"
#include "stm32f4xx_hal.h"


static struct netif netif;
static int net_would_block(const mbedtls_net_context *ctx);

void mbedtls_net_init(mbedtls_net_context *ctx)
{
    ctx->fd = -1;
}

/*
 * Initiate a TCP connection with host:port and the given protocol
 */
int mbedtls_net_connect(mbedtls_net_context *ctx, const char *host, const char *port, int proto)
{
    int ret;
    struct addrinfo hints;
    struct addrinfo *list;
    struct addrinfo *current;

    /* Do name resolution with both IPv6 and IPv4 */
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = proto == MBEDTLS_NET_PROTO_UDP ? SOCK_DGRAM : SOCK_STREAM;
    hints.ai_protocol = proto == MBEDTLS_NET_PROTO_UDP ? IPPROTO_UDP : IPPROTO_TCP;

    if (getaddrinfo(host, port, &hints, &list) != 0)
        return MBEDTLS_ERR_NET_UNKNOWN_HOST;

    /* Try the sockaddrs until a connection succeeds */
    ret = MBEDTLS_ERR_NET_UNKNOWN_HOST;
    for (current = list; current != NULL; current = current->ai_next)
    {
        ctx->fd = (int)socket(current->ai_family, current->ai_socktype, current->ai_protocol);
        if (ctx->fd < 0)
        {
            ret = MBEDTLS_ERR_NET_SOCKET_FAILED;
            continue;
        }

        if (connect(ctx->fd, current->ai_addr, (uint32_t)current->ai_addrlen) == 0)
        {
            int32_t timeout = 30000;
            setsockopt(ctx->fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
            timeout = 30000;
            setsockopt(ctx->fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
            int32_t keepIdle = 10;
            int32_t keepInterval = 2;
            int32_t keepCount = 5;
            setsockopt(ctx->fd, SOL_SOCKET, SO_KEEPALIVE, (const char *)&keepIdle, sizeof(keepIdle));
            setsockopt(ctx->fd, IPPROTO_TCP, TCP_KEEPIDLE, (const char *)&keepIdle, sizeof(keepIdle));
            setsockopt(ctx->fd, IPPROTO_TCP, TCP_KEEPINTVL, (const char *)&keepInterval, sizeof(keepInterval));
            setsockopt(ctx->fd, IPPROTO_TCP, TCP_KEEPCNT, (const char *)&keepCount, sizeof(keepCount));
            ret = 0;
            break;
        }

        close(ctx->fd);
        ret = MBEDTLS_ERR_NET_CONNECT_FAILED;
    }

    freeaddrinfo(list);

    return ret;
}

/*
 * Create a listening socket on bind_ip:port
 */
int mbedtls_net_bind(mbedtls_net_context *ctx, const char *bind_ip, const char *port, int proto)
{
    int ret = 0;
    // mbedtls_printf("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);

    return ret;
}

/*
 * Accept a connection from a remote client
 */
int mbedtls_net_accept(mbedtls_net_context *bind_ctx,
                       mbedtls_net_context *client_ctx,
                       void *client_ip, size_t buf_size, size_t *ip_len)
{
    // mbedtls_printf("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
    return 0;
}

/*
 * Set the socket blocking or non-blocking
 */
int mbedtls_net_set_block(mbedtls_net_context *ctx)
{
    // mbedtls_printf("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
    return 0;
}

int mbedtls_net_set_nonblock(mbedtls_net_context *ctx)
{
    // mbedtls_printf("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
    return 0;
}

/*
 * Portable usleep helper
 */
void mbedtls_net_usleep(unsigned long usec)
{
    // mbedtls_printf("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);
}

/*
 * Read at most 'len' characters
 */
int mbedtls_net_recv(void *ctx, unsigned char *buf, size_t len)
{
    int32_t ret;
    int32_t fd = ((mbedtls_net_context *)ctx)->fd;

    if (fd < 0)
    {
        return MBEDTLS_ERR_NET_INVALID_CONTEXT;
    }

    ret = (int32_t)read(fd, buf, len);

    if (ret < 0)
    {
        if (net_would_block(ctx) != 0)
        {
            return MBEDTLS_ERR_SSL_WANT_READ;
        }

        if (errno == EPIPE || errno == ECONNRESET)
        {
            return MBEDTLS_ERR_NET_CONN_RESET;
        }

        if (errno == EINTR)
        {
            return MBEDTLS_ERR_SSL_WANT_READ;
        }

#ifdef STATION_TCP_DEBUG
    printf("mbedtls_net_recv: err net recv failed %ld\r\n", ret);
#endif

        return MBEDTLS_ERR_NET_RECV_FAILED;
    }

    return ret;
}

/*
 * Read at most 'len' characters, blocking for at most 'timeout' ms
 */
int mbedtls_net_recv_timeout(void *ctx, unsigned char *buf, size_t len,
                             uint32_t timeout)
{
    // mbedtls_printf("%s() NOT IMPLEMENTED!!\n", __FUNCTION__);

    return mbedtls_net_recv(ctx, buf, len);
}

static int net_would_block(const mbedtls_net_context *ctx)
{
    /*
   * Never return 'WOULD BLOCK' on a non-blocking socket
   */
    int val = 0;
    UNUSED(val);

    if ((fcntl(ctx->fd, F_GETFL, val) & O_NONBLOCK) != O_NONBLOCK)
        return (0);

    switch (errno)
    {
#if defined EAGAIN
    case EAGAIN:
#endif
#if defined EWOULDBLOCK && EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
        return (1);
    }

    return (0);
}

/*
 * Write at most 'len' characters
 */
int mbedtls_net_send(void *ctx, const unsigned char *buf, size_t len)
{
    int32_t ret;
    int fd = ((mbedtls_net_context *)ctx)->fd;

    if (fd < 0)
    {
        return MBEDTLS_ERR_NET_INVALID_CONTEXT;
    }

    ret = (int32_t)write(fd, buf, len);

    if (ret < 0)
    {
        if (net_would_block(ctx) != 0)
        {
            return MBEDTLS_ERR_SSL_WANT_WRITE;
        }

        if (errno == EPIPE || errno == ECONNRESET)
        {
            return MBEDTLS_ERR_NET_CONN_RESET;
        }

        if (errno == EINTR)
        {
            return MBEDTLS_ERR_SSL_WANT_WRITE;
        }

        return MBEDTLS_ERR_NET_SEND_FAILED;
    }

    return ret;
}

/*
 * Gracefully close the connection
 */
void mbedtls_net_free(mbedtls_net_context *ctx)
{
    if (ctx->fd == -1)
        return;

    shutdown(ctx->fd, 2);
    close(ctx->fd);

    ctx->fd = -1;
}

#endif /* MBEDTLS_NET_C */
