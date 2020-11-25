#include "netconn.h"
#include "string.h"

/** ***************************************************************************
 * @name netconn_read
 * @brief revieve function, copy rx data to rx_buf
 * @param *netcon point to netconn handler
 *        *buf point to recieve buffer
 *        *len point to buffer length
 *        max_length is max buffer length
 * @retval err_t
 ******************************************************************************/
err_t netconn_read(struct netconn *conn, uint8_t *buf, uint16_t *len, uint16_t max_length)
{
	struct netbuf *rx_netbuf;
	uint16_t len_t = 0;
	struct pbuf *q;
	err_t err;

    *len = 0;
	err = netconn_recv(conn, &rx_netbuf);
	if (err == ERR_OK)
	{
		taskENTER_CRITICAL();
		for (q = rx_netbuf->p; q != NULL; q = q->next)
		{
			if (q->len > (max_length - len_t))
				memcpy(buf + len_t, q->payload, (max_length - len_t));
			else
				memcpy(buf + len_t, q->payload, q->len);
			len_t += q->len;
			if (len_t > max_length) {
				len_t -= q->len;
				break;
			} else if (len_t == max_length) {
				break;
			}
		}
		*len = len_t;
		taskEXIT_CRITICAL();
	}
	netbuf_delete(rx_netbuf);
	
	return err;
}

/** ***************************************************************************
 * @name netconn_push_rx_data() 
 * @brief revieve data and push to rx fifo
 * @param *netcon point to netconn handler
 *         *rx_fifo point to rx fifo
 * @retval success(ERR_OK) fail(other)
 ******************************************************************************/
err_t netconn_push_rx_data(struct netconn *conn, fifo_type *rx_fifo)
{
	struct netbuf *rx_netbuf;
	struct pbuf *q;
	err_t err;
    uint16_t len = 0;

	err = netconn_recv(conn, &rx_netbuf);
	if (err == ERR_OK)
	{
		taskENTER_CRITICAL();
		for (q = rx_netbuf->p; q != NULL; q = q->next)
		{
            fifo_push(rx_fifo, q->payload, q->len);
            len += q->len;
		}
		taskEXIT_CRITICAL();
	}
	netbuf_delete(rx_netbuf);

	return err;
}

/** ***************************************************************************
 * @name netconn_push_tx_data
 * @brief push data to tx fifo
 * @param *buf point to send data buffer
 *        len send length
 * @retval success(1) fail(0)
 ******************************************************************************/
uint8_t netconn_push_tx_data(NETCONN_STATE *state, fifo_type *tx_fifo, uint8_t *buf, uint16_t len)
{
    if (netconn_is_interactive(state)) {
		taskENTER_CRITICAL();
        fifo_push(tx_fifo, buf, len);
		taskEXIT_CRITICAL();
        return 1;
    }
    return 0;
}

/** ***************************************************************************
 * @name netconn_get_rx_data
 * @brief get data from rx fifo
 * @param *buf point to rx data buffer
 *        len send length
 * @retval success(1) fail(0)
 ******************************************************************************/
uint16_t netconn_get_rx_data(NETCONN_STATE *state, fifo_type *rx_fifo, uint8_t* buf, uint16_t len)
{
    if (netconn_is_interactive(state)) {
        return fifo_get(rx_fifo, buf, len);
    }
    return 0;
}

/** ***************************************************************************
 * @name netconn_check_ret
 * @brief 
 * @param 
 * @retval N/A
 ******************************************************************************/
void netconn_check_ret(err_t err, NETCONN_STATE *state)
{
    if (ERR_IS_FATAL(err)) {
        netconn_link_down(state);
    }
}

/** ***************************************************************************
 * @name netconn_link_down
 * @brief link down netconn, just set state
 * @param 
 * @retval 
 ******************************************************************************/
void netconn_link_down(NETCONN_STATE *state)
{
    if (*state >= NETCONN_STATE_CONNECT && *state <= NETCONN_STATE_INTERACTIVE) {
        *state = NETCONN_STATE_LINK_DOWN;
    }
}

/** ***************************************************************************
 * @name netconn_is_interactive
 * @brief get ntrip state
 * @param 
 * @retval true: is connected  false: others
 ******************************************************************************/
uint8_t netconn_is_interactive(NETCONN_STATE *state)
{
    return (*state == NETCONN_STATE_INTERACTIVE);
}
