#ifndef _ACEINNA_CLIENT_API_H_
#define _ACEINNA_CLIENT_API_H_

#include <stdio.h>

void aceinna_client_init(uint32_t uid);
void aceinna_client_thread(void);
int32_t is_aceinna_client_tls(void);
int32_t aceinna_client_push_nmea(char* buf, uint16_t len);
uint16_t aceinna_client_po_lla_message(int32_t lat, int32_t lon, int32_t alt, uint8_t *buf);


#endif
