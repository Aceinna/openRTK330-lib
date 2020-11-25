#include "ntrip_client.h"
#include "station_tcp.h"
#include "string.h"
#include "osapi.h"
#include "calibrationAPI.h"
#include "platformAPI.h"
#include "compat-1.3.h"
#include "base64.h"

/** ***************************************************************************
 * @name fill_ntrip_client_request_payload() 
 * @brief local rtk
 * @param *payload point to buffer
 *        *payloadLen point to buffer length
 * @retval N/A
 ******************************************************************************/
static void fill_ntrip_client_request_payload(uint8_t *payload, uint16_t *payloadLen)
{
    uint8_t temp[25];
    uint8_t key[100];
    uint8_t base64_buf[128];

    strcpy((char *)payload, "GET /");
    strcat((char *)payload, (const char *)get_ntrip_client_mount_point());
    strcat((char *)payload, " HTTP/1.1\r\n");
    strcat((char *)payload, "User-Agent: NTRIP Aceinna/0.1\r\n");

    strcat((char *)payload, "Ntrip-Sn:");
    sprintf((char *)temp, "%ld", GetUnitSerialNum());
    strcat((char *)payload, (const char *)temp);
    strcat((char *)payload, "\r\n");
    
    strcat((char *)payload, "Ntrip-Pn:");
    strcpy((char *)temp, (const char *)platformBuildInfo());
    for (uint8_t i = 0; i < strlen((const char*)temp); i++) {
        if (temp[i] == ' ') {
            temp[i] = 0;
            break;
        }
    }
    strcat((char *)payload, (const char *)temp);
    strcat((char *)payload, "\r\n");

    strcat((char *)payload, "Authorization: Basic ");
	
    strcpy((char *)key, (const char *)get_ntrip_client_username());
    strcat((char *)key, ":");
    strcat((char *)key, (const char *)get_ntrip_client_password());

    size_t len = strlen((const char*)key);
    base64_encode(base64_buf, sizeof(base64_buf), &len, key, len);
    strcat((char *)payload, (const char *)base64_buf);
    strcat((char *)payload, "\r\n\r\n");
    
    *payloadLen = strlen((const char *)payload);
}

int32_t ntrip_client_request(struct netconn *conn, NETCONN_STATE *state, uint8_t *buf, uint16_t size)
{
    int32_t res = -1;
    uint16_t len = 0;
    err_t err = ERR_OK;

    memset(buf, 0, size);
    fill_ntrip_client_request_payload(buf, &len);
    err = netconn_write(conn, buf, len, NETCONN_COPY);
    if (!err) {
        OS_Delay(1500);
        err = netconn_read(conn, buf, &len, size-1);
        if (!err) {
            buf[len] = 0;
            if (len && strstr((char *)buf, "ICY 200 OK") != NULL) {
                res = 0;
            } else {
#ifdef STATION_TCP_DEBUG
    printf("NTRIP-CLIENT: request fail %s\r\n", buf);
#endif
            }
        }
    }

    netconn_check_ret(err, state);

    return res;
}
