#ifndef BASE_STATION

#include <string.h>
#include <stdlib.h>
#include "httpd.h"
#include "lwip/tcp.h"
#include "fs.h"
#include "lwip_comm.h"
#include "app_version.h"
#include "stm32f4xx_hal.h"
#include "m_ntrip_client.h"
#include "calibrationAPI.h"
#include "platformAPI.h"
#include "user_config.h"
#include "cJSON.h"
#include "car_data.h"

const char radioEthMode[2][15] = {
	"radioDhcp",
    "radioStatic",
};

const char radioWheelPinMode[2][15] = {
	"radioWheel",
    "radioSPI",
};

const char radioCanMode[2][15] = {
	"radioCAR",
    "radioJ1939",
};

#define NUM_CONFIG_SSI_TAGS 8
#define NUM_CONFIG_CGI_URIS 4
#define NUM_CONFIG_JS_URIS 5

const char *ntrip_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *user_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ethnet_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *odo_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

const char *ntrip_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ntrip_state_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *user_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ethnet_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *odo_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

static const char *ssiTAGs[] =
	{
        "productName",
        "imu",
        "pn",
        "firmwareVersion",
        "serialNumwer",
		"appVersion",
        "ntripConnect",
        "ntripStream",
};

static const tCGI cgiURIs[] =
	{
        {"/NtripConfig.cgi", ntrip_config_cgi_handler},
		{"/UserConfig.cgi", user_config_cgi_handler},
		{"/EthnetConfig.cgi", ethnet_config_cgi_handler},
        {"/OdoConfig.cgi", odo_config_cgi_handler},
};

static const tJS jsURIs[] =
	{
		{"/NtripConfig.js", ntrip_config_js_handler},
        {"/NtripState.js", ntrip_state_js_handler},
		{"/UserConfig.js", user_config_js_handler},
        {"/EthnetConfig.js", ethnet_config_js_handler},
        {"/OdoConfig.js", odo_config_js_handler},
};

// SSI Handler
static u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen)
{
    uint8_t temp[25];

	switch (iIndex)
	{
    case 0:
        strcpy(pcInsert, PRODUCT_NAME_STRING);
        break;
    case 1:
        strcpy(pcInsert, (const char*)GetUnitVersion());
        break;
	case 2:
        strcpy((char *)temp, (const char *)platformBuildInfo());
        for (uint8_t i = 0; i < strlen((const char*)temp); i++)
        {
            if (temp[i] == ' ')
            {
                temp[i] = 0;
                break;
            }
        }
		strcpy(pcInsert, (const char*)temp);
		break;
	case 3:
        strcpy((char *)temp, (const char *)platformBuildInfo());
        uint8_t i = 0;
        for (i = 0; i < strlen((const char*)temp); i++)
        {
            if (temp[i] == ' ')
            {
                temp[i] = 0;
                break;
            }
        }
		strcpy(pcInsert, (const char*)&temp[i+1]);
        break;
    case 4:
        sprintf(pcInsert, "%ld", GetUnitSerialNum());
        break;
    case 5:
        strcpy(pcInsert, APP_VERSION_STRING);
        break;
    case 6:
        if (is_ntrip_interactive())
        {
            strcpy(pcInsert, "CONNECTED");
        }
        else
        {
            strcpy(pcInsert, "DISCONNECTED");
        }
        break;
    case 7:
        if (is_ntrip_interactive() && is_ntrip_stream_available())
        {
            strcpy(pcInsert, "AVAILABLE");
        }
        else
        {
            strcpy(pcInsert, "UNAVAILABLE");
        }
        break;
    default:
        break;
	}
	return strlen(pcInsert);
}

// CGI Handler
static int FindCGIParameter(const char *pcToFind, char *pcParam[], int iNumParams)
{
	int iLoop;
	for (iLoop = 0; iLoop < iNumParams; iLoop++)
	{
		if (strcmp(pcToFind, pcParam[iLoop]) == 0)
		{
			return (iLoop);
		}
	}
	return (-1);
}


static void escape_symbol(const char* src, char* dest)
{
    uint16_t len = strlen(src);
    uint16_t i;
    uint8_t symbol[2];

    for (i = 0; i < len; i++) {
        if (src[i] == '%') {
            if (i < len-2) {
                symbol[0] = src[i+1];
                symbol[1] = src[i+2];
                i = i + 2;
                if (symbol[0] == '5' && symbol[1] == 'B') {
                    *dest++ = '[';
                } else if (symbol[0] == '5' && symbol[1] == 'D') {
                    *dest++ = ']';
                } else if (symbol[0] == '7' && symbol[1] == 'B') {
                    *dest++ = '{';
                } else if (symbol[0] == '2' && symbol[1] == '2') {
                    *dest++ = '"';
                } else if (symbol[0] == '3' && symbol[1] == 'A') {
                    *dest++ = ':';
                } else if (symbol[0] == '2' && symbol[1] == 'C') {
                    *dest++ = ',';
                } else if (symbol[0] == '7' && symbol[1] == 'D') {
                    *dest++ = '}';
                }
            }
        } else {
            *dest++ = src[i];
        }
    }
    *dest = 0;
}

const char *odo_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    int index_wlmode, index_canMesg, index_gear, index_canmode;
    char buf[512] = {0};
    cJSON *root = NULL, *data = NULL;
    uint8_t index = 0;

    if (iNumParams == 4) {
        index_wlmode = FindCGIParameter("wlmode", pcParam, iNumParams);
		index_canMesg = FindCGIParameter("canMesg", pcParam, iNumParams);
        index_gear = FindCGIParameter("gear", pcParam, iNumParams);
        index_canmode = FindCGIParameter("canmode", pcParam, iNumParams);

        if (index_wlmode != -1 && index_canMesg != -1 && index_gear != -1 && index_canmode != -1) {
            escape_symbol((const char*)pcValue[index_canMesg], buf);

            if (!strcmp(pcValue[index_wlmode], "WHEELTICK")) {
                gOdoConfigurationStruct.wheeltick_pin_mode = 0;
            } else if (!strcmp(pcValue[index_wlmode], "SPI_NSS")) {
                gOdoConfigurationStruct.wheeltick_pin_mode = 1;
            }

            if (!strcmp(pcValue[index_canmode], "CAR")) {
                gOdoConfigurationStruct.can_mode = 0;
            } else if (!strcmp(pcValue[index_canmode], "J1939")) {
                gOdoConfigurationStruct.can_mode = 1;
            }

            gOdoConfigurationStruct.odo_mesg[0].usage = 0;
            gOdoConfigurationStruct.odo_mesg[1].usage = 0;
            gOdoConfigurationStruct.odo_mesg[2].usage = 0;
            root = cJSON_Parse((const char *)buf);
            if (root != NULL) {
                data = root->child;
                while (data != NULL) {
                    cJSON *MesgID = cJSON_GetObjectItem(data, "MesgID");
                    cJSON *StartBit = cJSON_GetObjectItem(data, "StartBit");
                    cJSON *Length = cJSON_GetObjectItem(data, "Length");
                    cJSON *Endian = cJSON_GetObjectItem(data, "Endian");
                    cJSON *Sign = cJSON_GetObjectItem(data, "Sign");
                    cJSON *Factor = cJSON_GetObjectItem(data, "Factor");
                    cJSON *Offset = cJSON_GetObjectItem(data, "Offset");
                    cJSON *Unit = cJSON_GetObjectItem(data, "Unit");
                    cJSON *Source = cJSON_GetObjectItem(data, "Source");
                    if (MesgID != NULL && StartBit != NULL && Length != NULL
                        && Endian != NULL && Sign != NULL && Factor != NULL
                        && Offset != NULL && Unit != NULL && Source != NULL) 
                    {
                        gOdoConfigurationStruct.odo_mesg[index].mesgID = atoi(MesgID->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].startbit = atoi(StartBit->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].length = atoi(Length->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].endian = atoi(Endian->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].sign = atoi(Sign->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].factor = atof(Factor->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].offset = atof(Offset->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].unit = atoi(Unit->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].source = atoi(Source->valuestring);
                        gOdoConfigurationStruct.odo_mesg[index].usage = 0x55;
                        index++;
                        if (index >= 3) {
                            break;
                        }
                    }
                    data = data->next;
                }
                cJSON_Delete(root);
            }

            escape_symbol((const char*)pcValue[index_gear], buf);
            root = cJSON_Parse((const char *)buf);
            if (root != NULL) {
                cJSON *gear_p = cJSON_GetObjectItem(root, "P");
                cJSON *gear_r = cJSON_GetObjectItem(root, "R");
                cJSON *gear_n = cJSON_GetObjectItem(root, "N");
                cJSON *gear_d = cJSON_GetObjectItem(root, "D");

                if (gear_p != NULL && gear_r != NULL && gear_n != NULL && gear_d != NULL) {
                    gOdoConfigurationStruct.gears[0] = atoi(gear_p->valuestring);
                    gOdoConfigurationStruct.gears[1] = atoi(gear_r->valuestring);
                    gOdoConfigurationStruct.gears[2] = atoi(gear_n->valuestring);
                    gOdoConfigurationStruct.gears[3] = atoi(gear_d->valuestring);
                }
                cJSON_Delete(root);
            }

            car_can_initialize();
            gOdoConfigurationStruct.flag = 0xaa5555aa;
            SaveUserConfig();
        }
    }

    return "/OdoCfg.shtml";
}

const char *ethnet_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_eth_mode, index_static_ip, index_static_netmask, index_static_gateway;
    ip_addr_t ip_addr_ip;
    ip_addr_t ip_addr_netmask;
    ip_addr_t ip_addr_gateway;

	if (iNumParams == 4)
	{
        index_eth_mode = FindCGIParameter("ethMode", pcParam, iNumParams);
		index_static_ip = FindCGIParameter("defaultIp", pcParam, iNumParams);
		index_static_netmask = FindCGIParameter("defaultNetmask", pcParam, iNumParams);
		index_static_gateway = FindCGIParameter("defaultGateway", pcParam, iNumParams);

		if (index_eth_mode != -1 && index_static_ip != -1 
            && index_static_netmask != -1 && index_static_gateway != -1)
		{
			if (ipaddr_aton((const char*)pcValue[index_static_ip], &ip_addr_ip) == 1
				&& ipaddr_aton((const char*)pcValue[index_static_netmask], &ip_addr_netmask) == 1
                && ipaddr_aton((const char*)pcValue[index_static_gateway], &ip_addr_gateway) == 1)
			{
				if ((!strcmp(pcValue[index_eth_mode], "dhcp") && get_eth_mode() != ETHMODE_DHCP) ||
					(!strcmp(pcValue[index_eth_mode], "static") && get_eth_mode() != ETHMODE_STATIC) ||
                    memcmp(get_static_ip(), &ip_addr_ip, 4) != 0 ||
					memcmp(get_static_netmask(), &ip_addr_netmask, 4) != 0 ||
					memcmp(get_static_gateway(), &ip_addr_gateway, 4) != 0)
				{
                    if (!strcmp(pcValue[index_eth_mode], "dhcp") && get_eth_mode() == ETHMODE_STATIC)
					{
                        set_eth_mode(ETHMODE_DHCP);
					}
					else if (!strcmp(pcValue[index_eth_mode], "static") && get_eth_mode() == ETHMODE_DHCP)
					{
                        set_eth_mode(ETHMODE_STATIC);
					}

                    set_static_ip((uint8_t*)&ip_addr_ip);
                    set_static_netmask((uint8_t*)&ip_addr_netmask);
                    set_static_gateway((uint8_t*)&ip_addr_gateway);

                    netif_ethernet_config_changed();

					SaveUserConfig();
				}
			}
			else 
			{
				// err ip
			}
		}
	}
	
	return "/EthCfg.shtml";
}

const char *ntrip_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_ip, index_port, index_mountPoint, index_username, index_password;
	uint16_t port = 0;

	if (iNumParams == 5)
	{
        index_ip = FindCGIParameter("ip", pcParam, iNumParams);
		index_port = FindCGIParameter("port", pcParam, iNumParams);
		index_mountPoint = FindCGIParameter("mountPoint", pcParam, iNumParams);
        index_username = FindCGIParameter("username", pcParam, iNumParams);
		index_password = FindCGIParameter("password", pcParam, iNumParams);
		
		if (index_ip != -1 && index_port != -1 && index_mountPoint != -1 && index_username != -1 && index_password != -1)
		{
			port = atoi(pcValue[index_port]);

			if (strcmp(get_ntrip_client_ip(), pcValue[index_ip]) != 0
				|| get_ntrip_client_port() != port 
				|| strcmp(get_ntrip_client_mount_point(), pcValue[index_mountPoint]) != 0
				|| strcmp(get_ntrip_client_username(), pcValue[index_username]) != 0
				|| strcmp(get_ntrip_client_password(), pcValue[index_password]) != 0)
			{
                set_ntrip_client_ip((const char *)pcValue[index_ip]);
                set_ntrip_client_port(port);
                set_ntrip_client_mount_point((const char *)pcValue[index_mountPoint]);
                set_ntrip_client_username((const char *)pcValue[index_username]);
                set_ntrip_client_password((const char *)pcValue[index_password]);

				netif_ntrip_config_changed();

				SaveUserConfig();
			}
		}
	}

	return "/NtripCfg.shtml";
}

const char *user_config_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_userPacketType, index_userPacketRate;
	int index_leverArmBx, index_leverArmBy, index_leverArmBz;
	int index_pointOfInterestBx, index_pointOfInterestBy, index_pointOfInterestBz;
	int index_rotationRbvx, index_rotationRbvy, index_rotationRbvz;
	BOOL result;

	if (iNumParams == 11)
	{
		index_userPacketType = FindCGIParameter("userPacketType", pcParam, iNumParams);
		index_userPacketRate = FindCGIParameter("userPacketRate", pcParam, iNumParams);
		index_leverArmBx = FindCGIParameter("leverArmBx", pcParam, iNumParams);
		index_leverArmBy = FindCGIParameter("leverArmBy", pcParam, iNumParams);
		index_leverArmBz = FindCGIParameter("leverArmBz", pcParam, iNumParams);
		index_pointOfInterestBx = FindCGIParameter("pointOfInterestBx", pcParam, iNumParams);
		index_pointOfInterestBy = FindCGIParameter("pointOfInterestBy", pcParam, iNumParams);
		index_pointOfInterestBz = FindCGIParameter("pointOfInterestBz", pcParam, iNumParams);
		index_rotationRbvx = FindCGIParameter("rotationRbvx", pcParam, iNumParams);
		index_rotationRbvy = FindCGIParameter("rotationRbvy", pcParam, iNumParams);
		index_rotationRbvz = FindCGIParameter("rotationRbvz", pcParam, iNumParams);

		if (index_userPacketType != -1 && index_userPacketRate != -1
			&& index_leverArmBx != -1 && index_leverArmBy != -1 && index_leverArmBz != -1
			&& index_pointOfInterestBx != -1 && index_pointOfInterestBy != -1 && index_pointOfInterestBz != -1
			&& index_rotationRbvx != -1 && index_rotationRbvy != -1 && index_rotationRbvz != -1)
		{
			result = valid_user_config_parameter(USER_USER_PACKET_TYPE, (uint8_t*)pcValue[index_userPacketType]);
			if (result){
                set_user_packet_type((uint8_t*)pcValue[index_userPacketType]);
			}

			uint16_t userPacketRate = atoi(pcValue[index_userPacketRate]);
			result = valid_user_config_parameter(USER_USER_PACKET_RATE, (uint8_t*)&userPacketRate);
			if (result){
				set_user_packet_rate(userPacketRate);
			}

            set_lever_arm_bx(atof(pcValue[index_leverArmBx]));
            set_lever_arm_by(atof(pcValue[index_leverArmBy]));
            set_lever_arm_bz(atof(pcValue[index_leverArmBz]));
            set_point_of_interest_bx(atof(pcValue[index_pointOfInterestBx]));
            set_point_of_interest_by(atof(pcValue[index_pointOfInterestBy]));
            set_point_of_interest_bz(atof(pcValue[index_pointOfInterestBz]));
            set_rotation_rbvx(atof(pcValue[index_rotationRbvx]));
            set_rotation_rbvy(atof(pcValue[index_rotationRbvy]));
            set_rotation_rbvz(atof(pcValue[index_rotationRbvz]));

            ins_init();

			SaveUserConfig();
		}
	}

	return "/UserCfg.shtml";
}

#define HTTP_JS_RESPONSE_SIZE 1024
CCMRAM uint8_t http_response[HTTP_JS_RESPONSE_SIZE];
CCMRAM uint8_t http_response_body[HTTP_JS_RESPONSE_SIZE];
// JS Handler
const char *odo_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    char* buf = NULL;
    cJSON *data = NULL;
    uint8_t i;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "OdoConfigCallback({\"wlmode\":\"%s\",\"canmode\":\"%s\",\"canMesg\":[",
            radioWheelPinMode[gOdoConfigurationStruct.wheeltick_pin_mode],
            radioCanMode[gOdoConfigurationStruct.can_mode]);

    for (i = 0; i < 3; i++) {
        if (gOdoConfigurationStruct.odo_mesg[i].usage == 0x55) {
            data = cJSON_CreateObject();
            cJSON_AddItemToObject(data, "MesgID", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].mesgID));
            cJSON_AddItemToObject(data, "StartBit", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].startbit));
            cJSON_AddItemToObject(data, "Length", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].length));
            cJSON_AddItemToObject(data, "Endian", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].endian));
            cJSON_AddItemToObject(data, "Sign", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].sign));
            cJSON_AddItemToObject(data, "Factor", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].factor));
            cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].offset));
            cJSON_AddItemToObject(data, "Unit", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].unit));
            cJSON_AddItemToObject(data, "Source", cJSON_CreateNumber(gOdoConfigurationStruct.odo_mesg[i].source));

            buf = cJSON_PrintUnformatted(data);
            if (buf != NULL) {
                if (i != 0) {
                    strcat((char *)http_response_body, ",");
                }
                strcat((char *)http_response_body, buf);
                free(buf);
            }
            cJSON_Delete(data);
        }
    }
    strcat((char *)http_response_body, "],\"gear\":");

    data = cJSON_CreateObject();
    cJSON_AddItemToObject(data, "P", cJSON_CreateNumber(gOdoConfigurationStruct.gears[0]));
    cJSON_AddItemToObject(data, "R", cJSON_CreateNumber(gOdoConfigurationStruct.gears[1]));
    cJSON_AddItemToObject(data, "N", cJSON_CreateNumber(gOdoConfigurationStruct.gears[2]));
    cJSON_AddItemToObject(data, "D", cJSON_CreateNumber(gOdoConfigurationStruct.gears[3]));
    buf = cJSON_PrintUnformatted(data);
    if (buf != NULL) {
        strcat((char *)http_response_body, buf);
        free(buf);
    }
    cJSON_Delete(data);

    strcat((char *)http_response_body, "})");

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *ethnet_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    uint8_t *mac = get_static_mac();
    uint8_t *staticIp = get_static_ip();
    uint8_t *gateway = get_static_gateway();
    uint8_t *netmask = get_static_netmask();

	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "EthnetConfigCallback({\"ethMode\":\"%s\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"defaultIp\":\"%d.%d.%d.%d\",\"defaultGateway\":\"%d.%d.%d.%d\",\"defaultNetmask\":\"%d.%d.%d.%d\"})",
            radioEthMode[get_eth_mode()],
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
			staticIp[0], staticIp[1], staticIp[2], staticIp[3],
			gateway[0], gateway[1], gateway[2], gateway[3],
			netmask[0], netmask[1], netmask[2], netmask[3]);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *ntrip_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "NtripConfigCallback({\"ip\":\"%s\",\"port\":\"%d\",\"mountPoint\":\"%s\",\"username\":\"%s\",\"password\":\"%s\"})",
			get_ntrip_client_ip(),
			get_ntrip_client_port(),
			get_ntrip_client_mount_point(),
            get_ntrip_client_username(),
            get_ntrip_client_password()
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *user_config_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	uint8_t userPacketType[3];

	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	memcpy(userPacketType, get_user_packet_type(), 2);
	userPacketType[2] = 0;
	sprintf((char *)http_response_body, "UserConfigCallback({\"userPacketType\":\"%s\",\"userPacketRate\":\"%d\",\"leverArmBx\":\"%f\",\"leverArmBy\":\"%f\",\"leverArmBz\":\"%f\",\"pointOfInterestBx\":\"%f\",\"pointOfInterestBy\":\"%f\",\"pointOfInterestBz\":\"%f\",\"rotationRbvx\":\"%f\",\"rotationRbvy\":\"%f\",\"rotationRbvz\":\"%f\"})",
			userPacketType,
            get_user_packet_rate(),
            get_lever_arm_bx(),
            get_lever_arm_by(),
            get_lever_arm_bz(),
            get_point_of_interest_bx(),
            get_point_of_interest_by(),
            get_point_of_interest_bz(),
            get_rotation_rbvx(),
            get_rotation_rbvy(),
            get_rotation_rbvz()
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *ntrip_state_js_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    sprintf((char *)http_response_body, "NtripStateCallback({\"connect\":\"%s\",\"stream\":\"%s\"})",
        (is_ntrip_interactive())? "CONNECTED":"DISCONNECTED",
        (is_ntrip_interactive() && is_ntrip_stream_available())? "AVAILABLE":"UNAVAILABLE");

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

void httpd_ssi_init(void)
{
	http_set_ssi_handler(ssi_handler, ssiTAGs, NUM_CONFIG_SSI_TAGS);
}

void httpd_cgi_init(void)
{
	http_set_cgi_handlers(cgiURIs, NUM_CONFIG_CGI_URIS);
}

void httpd_js_init(void)
{
	http_set_js_handlers(jsURIs, NUM_CONFIG_JS_URIS);
}


#else   // BASE_STATION


#include <string.h>
#include <stdlib.h>
#include "httpd.h"
#include "lwip/tcp.h"
#include "fs.h"
#include "lwip_comm.h"
#include "app_version.h"
#include "stm32f4xx_hal.h"
#include "calibrationAPI.h"
#include "platformAPI.h"
#include "user_config.h"
#include "ntrip_server.h"
#include "user_message.h"
#include "lcsystem.h"
#include "station_tcp.h"
#include "aceinna_client.h"

#define NUM_CONFIG_SSI_TAGS 6
#define NUM_CONFIG_CGI_URIS 5
#define NUM_CONFIG_JS_URIS 5

const char *ntrip_client_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *aceinna_client_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ntrip_server_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *user_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ethnet_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);

const char *position_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *work_config_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *user_config_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ethnet_config_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ethnet_summary_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[]);

static uint8_t tool_itoa(int32_t value, char *sp, uint8_t radix)
{
    char     tmp[12];   // int32
    char     *tp = tmp;
    int32_t  i;
    uint32_t v;
    int32_t  sign = 0;
    uint8_t  len = 0;

    sign = (radix == 10 && value < 0);
    if (sign) {
        v = -value;
    } else {
        v = value;
    }

    while (v || tp == tmp)
    {
        i = v % radix;
        v /= radix;
        if (i < 10) {
          *tp++ = i + '0';
        } else {
          *tp++ = i + 'A' - 10;
        }
    }

    if (radix != 10) { ///< zero fill non decimal values
        while (tp < &(tmp[4])) {
          *tp++ = '0';
        }
    }

    if (sign) { 
        *sp++ = '-';
        len = 1;
    }
    /// reverse and put in output buffer
    len += tp - tmp;
    while (tp > tmp) {
        tp--;
        *sp++ = *tp;
    }
    *sp = '\0';
    return len;
}

static void tool_itof(float f, char *sp, uint8_t sigDigits)
{
    char numberIntString[12] = {0};
    char numberSigString[12] = {0};
    uint8_t intStringLen, sigStringLen;
    int32_t i, n;
    uint8_t append_zero, sign = 0;

    i = (int32_t) f; ///< just get the number to the left of the decimal
    n = i;
    if (f < 0) {
        n = -n;
        sign = 1;
    } else {
        sign = 0;
    }

    /// now get the number of significant digits to the right
    f = f - i;
    if (f < 0) {
        f = -f;
    }
    if (sigDigits > 6) { /// max 6 sig digits
        sigDigits = 6;
    }
    f = f * pow(10, sigDigits);
    i = round(f);
    if (i >= pow(10, sigDigits)) {
        n++;
        i = 0;
    }

    intStringLen = tool_itoa(n, numberIntString, 10);
    sigStringLen = tool_itoa(i, numberSigString, 10);

    if (sign) {
        *sp++ = '-';
    }
    for (i = 0; i < intStringLen; i++) {
        *sp++ = numberIntString[i];
    }
    *sp++ = '.';
    append_zero = sigDigits - sigStringLen;
    while (append_zero) {
        *sp++ = '0';
        append_zero--;
    }
    for (i = 0; i < sigStringLen; i++) {
        *sp++ = numberSigString[i];
    }
    *sp = '\0';
}

static void tool_itod(double d, char *sp, uint8_t sigDigits)
{
    char numberIntString[12] = {0};
    char numberSigString[12] = {0};
    uint8_t intStringLen, sigStringLen;
    int32_t i, n;
    uint8_t append_zero, sign = 0;

    i = (int32_t) d; ///< just get the number to the left of the decimal
    n = i;
    if (d < 0) {
        n = -n;
        sign = 1;
    } else {
        sign = 0;
    }

    /// now get the number of significant digits to the right
    d = d - i;
    if (d < 0) {
        d = -d;
    }
    if (sigDigits > 8) { /// max 8 sig digits
        sigDigits = 8;
    }
    d = d * pow(10, sigDigits);
    i = round(d);
    if (i >= pow(10, sigDigits)) {
        n++;
        i = 0;
    }

    intStringLen = tool_itoa(n, numberIntString, 10);
    sigStringLen = tool_itoa(i, numberSigString, 10);

    if (sign) {
        *sp++ = '-';
    }
    for (i = 0; i < intStringLen; i++) {
        *sp++ = numberIntString[i];
    }
    *sp++ = '.';
    append_zero = sigDigits - sigStringLen;
    while (append_zero) {
        *sp++ = '0';
        append_zero--;
    }
    for (i = 0; i < sigStringLen; i++) {
        *sp++ = numberSigString[i];
    }
    *sp = '\0';
}

static uint32_t string_append(char *strDest, const char *strScr)
{
    uint32_t len = 0;
    
    if ((strDest != NULL) && (strScr != NULL)) {
        while (*strScr) {
            *strDest++ = *strScr++;
            len++;
        }
        *strDest = '\0';
    }
    return len;
}

static char *string_copy(char *strDest, const char *strScr)
{
    char *address = strDest;

    if ((strDest != NULL) && (strScr != NULL)) {
        while (*strScr)
            *strDest++ = *strScr++;
        *strDest = '\0';
    }
    return address;
}

static char *string_cat(char *strDest, const char *strScr)
{
    char *address = strDest;

    if ((strDest != NULL) && (strScr != NULL)) {
        while (*strDest)
            strDest++;
        while (*strScr)
            *strDest++ = *strScr++;
        *strDest = '\0';
    }
    return address;
}

static uint32_t string_length(const char *str)
{
    uint32_t len = 0;

    if (str != NULL) {
        while (*str++)
            len++;
    }
    return len;
}


uint8_t workpage_disp = 0;

static const char *ssiTAGs[] =
	{
        "productName",
        "imu",
        "pn",
        "firmwareVersion",
        "serialNumwer",
		"appVersion"
};

static const tCGI cgiURIs[] =
	{
        {"/ntripClientConfig.cgi", ntrip_client_config_cgi_handler},
        {"/aceinnaClientConfig.cgi", aceinna_client_config_cgi_handler},
        {"/ntripServerConfig.cgi", ntrip_server_config_cgi_handler},
		{"/userConfig.cgi", user_config_cgi_handler},
		{"/ethConfig.cgi", ethnet_config_cgi_handler},
};

static const tJS jsURIs[] =
	{
        {"/position.js", position_js_handler},
		{"/workConfig.js", work_config_js_handler},
        {"/userConfig.js", user_config_js_handler},
		{"/ethConfig.js", ethnet_config_js_handler},
        {"/ethSummary.js", ethnet_summary_js_handler},
};

// SSI Handler
static u16_t ssi_handler(int index, char *pcInsert, int iInsertLen)
{
    char temp[25] = {0};
    uint8_t i;

	switch (index)
	{
    case 0:
        string_copy(pcInsert, PRODUCT_NAME_STRING);
        break;
    case 1:
        string_copy(pcInsert, (const char*)GetUnitVersion());
        break;
	case 2:
        string_copy(temp, (const char *)platformBuildInfo());
        for (i = 0; i < string_length((const char*)temp); i++)
        {
            if (temp[i] == ' ')
            {
                temp[i] = 0;
                break;
            }
        }
		string_copy(pcInsert, (const char*)temp);
		break;
	case 3:
        string_copy(temp, (const char *)platformBuildInfo());
        for (i = 0; i < string_length((const char*)temp); i++)
        {
            if (temp[i] == ' ')
            {
                temp[i] = 0;
                break;
            }
        }
		string_copy(pcInsert, (const char*)&temp[i+1]);
        break;
    case 4:
        tool_itoa(GetUnitSerialNum(), temp, 10);
        string_copy(pcInsert, (const char*)temp);
        break;
    case 5:
        string_copy(pcInsert, APP_VERSION_STRING);
        break;
    default:
        break;
	}
	return string_length(pcInsert);
}

// CGI Handler
static int FindCGIParameter(const char *pcToFind, char *pcParam[], int iNumParams)
{
	int iLoop;
	for (iLoop = 0; iLoop < iNumParams; iLoop++)
	{
		if (strcmp(pcToFind, pcParam[iLoop]) == 0)
		{
			return (iLoop);
		}
	}
	return (-1);
}

const char *ethnet_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_eth_mode, index_static_ip, index_static_netmask, index_static_gateway;
    ip_addr_t ip_addr_ip;
    ip_addr_t ip_addr_netmask;
    ip_addr_t ip_addr_gateway;
    int eth_mode_t;
    uint8_t is_save = false;

	if (iNumParams == 1 || iNumParams == 4) {
        index_eth_mode = FindCGIParameter("ethmode", pcParam, iNumParams);
        if (index_eth_mode != -1) {
            eth_mode_t = atoi(pcValue[index_eth_mode]);
            if (eth_mode_t == ETHMODE_DHCP || eth_mode_t == ETHMODE_STATIC) {
                set_eth_mode(eth_mode_t);
                if (iNumParams == 1) {
                    is_save = true;
                }
            }
        }

        if (iNumParams == 4) {
            index_static_ip = FindCGIParameter("staticIp", pcParam, iNumParams);
            index_static_netmask = FindCGIParameter("staticNetmask", pcParam, iNumParams);
            index_static_gateway = FindCGIParameter("staticGateway", pcParam, iNumParams);
            if (index_static_ip != -1 && index_static_netmask != -1 && index_static_gateway != -1) {
                if (ipaddr_aton((const char*)pcValue[index_static_ip], &ip_addr_ip) == 1
                    && ipaddr_aton((const char*)pcValue[index_static_netmask], &ip_addr_netmask) == 1
                    && ipaddr_aton((const char*)pcValue[index_static_gateway], &ip_addr_gateway) == 1) {
                    set_static_ip((uint8_t*)&ip_addr_ip);
                    set_static_netmask((uint8_t*)&ip_addr_netmask);
                    set_static_gateway((uint8_t*)&ip_addr_gateway);
                    is_save = true;
                }
            }
        }
        if (is_save) {
            netif_ethernet_config_changed();
            SaveUserConfig();
        }
	}
	
	return "/ethCfg.shtml";
}

const char *user_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_userPacketType, index_userPacketRate;
    int index_canEcuAddress, index_canBaudrate, index_canPacketType;
    int index_canPacketRate, index_canTermresistor, index_canBaudrateDetect;
	int index_leverArmBx, index_leverArmBy, index_leverArmBz;
	int index_pointOfInterestBx, index_pointOfInterestBy, index_pointOfInterestBz;
	int index_rotationRbvx, index_rotationRbvy, index_rotationRbvz;
	BOOL result;

	if (iNumParams == 17)
	{
		index_userPacketType = FindCGIParameter("userPacketType", pcParam, iNumParams);
		index_userPacketRate = FindCGIParameter("userPacketRate", pcParam, iNumParams);
        index_canEcuAddress = FindCGIParameter("canEcuAddress", pcParam, iNumParams);
		index_canBaudrate = FindCGIParameter("canBaudrate", pcParam, iNumParams);
		index_canPacketType = FindCGIParameter("canPacketType", pcParam, iNumParams);
        index_canPacketRate = FindCGIParameter("canPacketRate", pcParam, iNumParams);
		index_canTermresistor = FindCGIParameter("canTermresistor", pcParam, iNumParams);
		index_canBaudrateDetect = FindCGIParameter("canBaudrateDetect", pcParam, iNumParams);
		index_leverArmBx = FindCGIParameter("leverArmBx", pcParam, iNumParams);
		index_leverArmBy = FindCGIParameter("leverArmBy", pcParam, iNumParams);
		index_leverArmBz = FindCGIParameter("leverArmBz", pcParam, iNumParams);
		index_pointOfInterestBx = FindCGIParameter("pointOfInterestBx", pcParam, iNumParams);
		index_pointOfInterestBy = FindCGIParameter("pointOfInterestBy", pcParam, iNumParams);
		index_pointOfInterestBz = FindCGIParameter("pointOfInterestBz", pcParam, iNumParams);
		index_rotationRbvx = FindCGIParameter("rotationRbvx", pcParam, iNumParams);
		index_rotationRbvy = FindCGIParameter("rotationRbvy", pcParam, iNumParams);
		index_rotationRbvz = FindCGIParameter("rotationRbvz", pcParam, iNumParams);

		if (index_userPacketType != -1 && index_userPacketRate != -1
            && index_canEcuAddress != -1 && index_canBaudrate != -1 && index_canPacketType != -1
            && index_canPacketRate != -1 && index_canTermresistor != -1 && index_canBaudrateDetect != -1
			&& index_leverArmBx != -1 && index_leverArmBy != -1 && index_leverArmBz != -1
			&& index_pointOfInterestBx != -1 && index_pointOfInterestBy != -1 && index_pointOfInterestBz != -1
			&& index_rotationRbvx != -1 && index_rotationRbvy != -1 && index_rotationRbvz != -1)
		{
			result = valid_user_config_parameter(USER_USER_PACKET_TYPE, (uint8_t*)pcValue[index_userPacketType]);
			if (result){
                set_user_packet_type((uint8_t*)pcValue[index_userPacketType]);
			}

			uint16_t userPacketRate = atoi(pcValue[index_userPacketRate]);
			result = valid_user_config_parameter(USER_USER_PACKET_RATE, (uint8_t*)&userPacketRate);
			if (result){
                set_user_packet_rate(userPacketRate);
			}

            set_can_ecu_address(atoi(pcValue[index_canEcuAddress]));
            set_can_baudrate(atoi(pcValue[index_canBaudrate]));
            set_can_packet_type(atoi(pcValue[index_canPacketType]));
            set_can_packet_rate(atoi(pcValue[index_canPacketRate]));
            set_can_termresistor(atoi(pcValue[index_canTermresistor]));
            set_can_baudrate_detect(atoi(pcValue[index_canBaudrateDetect]));

            set_lever_arm_bx(atof(pcValue[index_leverArmBx]));
            set_lever_arm_by(atof(pcValue[index_leverArmBy]));
            set_lever_arm_bz(atof(pcValue[index_leverArmBz]));
            set_point_of_interest_bx(atof(pcValue[index_pointOfInterestBx]));
            set_point_of_interest_by(atof(pcValue[index_pointOfInterestBy]));
            set_point_of_interest_bz(atof(pcValue[index_pointOfInterestBz]));
            set_rotation_rbvx(atof(pcValue[index_rotationRbvx]));
            set_rotation_rbvy(atof(pcValue[index_rotationRbvy]));
            set_rotation_rbvz(atof(pcValue[index_rotationRbvz]));

            ins_init();

			SaveUserConfig();
		}
	}

	return "/userCfg.shtml";
}

const char *ntrip_client_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_enable, index_ip, index_port, index_mountPoint, index_username, index_password;

	if (iNumParams == 6)
	{
        index_enable = FindCGIParameter("ncEn", pcParam, iNumParams);
        index_ip = FindCGIParameter("cIp", pcParam, iNumParams);
		index_port = FindCGIParameter("cPort", pcParam, iNumParams);
		index_mountPoint = FindCGIParameter("cMoP", pcParam, iNumParams);
        index_username = FindCGIParameter("cUser", pcParam, iNumParams);
		index_password = FindCGIParameter("cPwd", pcParam, iNumParams);

        if (index_enable != -1 &&index_ip != -1 && index_port != -1 && index_mountPoint != -1 && index_username != -1 && index_password != -1)
        {
            set_ntrip_client_ip((const char *)pcValue[index_ip]);
            set_ntrip_client_port(atoi(pcValue[index_port]));
            set_ntrip_client_mount_point((const char *)pcValue[index_mountPoint]);
            set_ntrip_client_username((const char *)pcValue[index_username]);
            set_ntrip_client_password((const char *)pcValue[index_password]);

            if (atoi(pcValue[index_enable]) == 1) {
                set_station_mode(MODE_NTRIP_CLIENT);
                netif_station_tcp_config_changed();
            }

            SaveUserConfig();
        }
	}

    workpage_disp = 0;

	return "/workCfg.shtml";
}

const char *aceinna_client_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_enable, index_ip, index_port, index_mountPoint, index_username, index_password;

	if (iNumParams == 6)
	{
        index_enable = FindCGIParameter("acEn", pcParam, iNumParams);
        index_ip = FindCGIParameter("aIp", pcParam, iNumParams);
		index_port = FindCGIParameter("aPort", pcParam, iNumParams);
		index_mountPoint = FindCGIParameter("aMoP", pcParam, iNumParams);
        index_username = FindCGIParameter("aUser", pcParam, iNumParams);
		index_password = FindCGIParameter("aPwd", pcParam, iNumParams);

        if (index_enable != -1 &&index_ip != -1 && index_port != -1 && index_mountPoint != -1 && index_username != -1 && index_password != -1)
        {
            set_aceinna_client_ip((const char *)pcValue[index_ip]);
            set_aceinna_client_port(atoi(pcValue[index_port]));
            set_aceinna_client_mount_point((const char *)pcValue[index_mountPoint]);
            set_aceinna_client_username((const char *)pcValue[index_username]);
            set_aceinna_client_password((const char *)pcValue[index_password]);

            if (atoi(pcValue[index_enable]) == 1) {
                set_station_mode(MODE_ACEINNA_CLIENT);
                netif_station_tcp_config_changed();
            }

            SaveUserConfig();
        }
	}

    workpage_disp = 1;

	return "/workCfg.shtml";
}

const char *ntrip_server_config_cgi_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_enable, index_ip, index_port, index_mountPoint, index_password;
	int index_station_id, index_antenna_height;
    int index_reference_latitude, index_reference_longitude, index_reference_height, index_base_position_type;
    
	if (iNumParams == 11)
	{
        index_enable = FindCGIParameter("nsEn", pcParam, iNumParams);
        index_ip = FindCGIParameter("sIp", pcParam, iNumParams);
		index_port = FindCGIParameter("sPort", pcParam, iNumParams);
		index_mountPoint = FindCGIParameter("sMoP", pcParam, iNumParams);
		index_password = FindCGIParameter("sPwd", pcParam, iNumParams);
		index_station_id = FindCGIParameter("staID", pcParam, iNumParams);
		index_antenna_height = FindCGIParameter("anth", pcParam, iNumParams);
		index_reference_latitude = FindCGIParameter("refLat", pcParam, iNumParams);
        index_reference_longitude = FindCGIParameter("refLon", pcParam, iNumParams);
		index_reference_height = FindCGIParameter("refHei", pcParam, iNumParams);
        index_base_position_type = FindCGIParameter("bpt", pcParam, iNumParams);

		if (index_enable != -1 && index_ip != -1 && index_port != -1 && index_mountPoint != -1 
            && index_password != -1 && index_station_id != -1 && index_antenna_height != -1
            && index_reference_latitude != -1 && index_reference_longitude != -1 && index_reference_height != -1
            && index_base_position_type != -1)
		{            
            set_ntrip_server_ip(pcValue[index_ip]);
            set_ntrip_server_port(atoi(pcValue[index_port]));
            set_ntrip_server_mount_point(pcValue[index_mountPoint]);
            set_ntrip_server_password(pcValue[index_password]);

            set_station_id(atoi(pcValue[index_station_id]));
            set_antenna_height(atof(pcValue[index_antenna_height]));
            set_reference_latitude(atof(pcValue[index_reference_latitude]));
            set_reference_longitude(atof(pcValue[index_reference_longitude]));
            set_reference_height(atof(pcValue[index_reference_height]));
            set_base_position_type(atoi(pcValue[index_base_position_type]));

            if (atoi(pcValue[index_enable]) == 1) {
                set_station_mode(MODE_NTRIP_SERVER);
                netif_station_tcp_config_changed();
                base_station_run_update();
            }

            SaveUserConfig();
		}
	}

    workpage_disp = 2;

	return "/workCfg.shtml";
}

// JS Handler
#define HTTP_JS_RESPONSE_SIZE 1024
CCMRAM char http_response[HTTP_JS_RESPONSE_SIZE];
CCMRAM char http_response_body[HTTP_JS_RESPONSE_SIZE];
static char gps[80], bds[80], glo[60], gal[60];
const char *position_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
    uint32_t len = 0;
    char temp[20] = {0};
    uint8_t vel_mode;
    uint8_t stationMode = 0, stationStatus = 0, basePosStatus = 0, baseRun;
    double basePosLatitude = 0.0, basePosLongitude = 0.0, basePosHeight = 0.0;
    uint8_t i = 0;
    uint8_t gps_n = 0, bds_n = 0, glo_n = 0, gal_n = 0;
    uint8_t gps_len = 0, bds_len = 0, glo_len = 0, gal_len = 0;
    uint32_t js_len = 0;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    stationMode = get_station_mode();
    basePosStatus = get_base_position_type();
    baseRun = base_station_get_run_status();

    if (basePosStatus == BASE_POSITION_REFERENCE) {
        basePosStatus = 0;
    } else if (basePosStatus == BASE_POSITION_SPP) {
        basePosStatus = 1;
    } else if (basePosStatus == BASE_POSITION_RTK) {
        basePosStatus = 3; 
    } else {
        basePosStatus = 5;
    }

    if (stationMode == MODE_NTRIP_CLIENT) {
        if (station_tcp_is_interactive()) {
            if (station_tcp_is_stream_timeout()) {
                stationStatus = 2;
            } else {
                stationStatus = 1;
            }
        } else {
            stationStatus = 0;
        } 
    } else if (stationMode == MODE_ACEINNA_CLIENT) {
        if (aceinna_client_info.mode == 1) {
            if (station_tcp_is_interactive()) {
                if (station_tcp_is_stream_timeout()) {
                    stationStatus = 6;
                } else {
                    stationStatus = 5;
                }
            } else {
                stationStatus = 4;
            }
        } else {
            stationStatus = 3;
        }

    } else if (stationMode == MODE_NTRIP_SERVER) {
        if (baseRun == 1) {
            if (station_tcp_is_interactive()) {
                if (station_tcp_is_stream_timeout()) {
                    stationStatus = 9;
                } else {
                    stationStatus = 8;
                }
            } else {
                stationStatus = 7;
            }

            if (basePosStatus == 0) {
                basePosLatitude = get_reference_latitude();
                basePosLongitude = get_reference_longitude();
                basePosHeight = get_reference_height();
            } else {
                basePosLatitude = get_station_pos_lat() * R2D;
                basePosLongitude = get_station_pos_lon() * R2D;
                basePosHeight = get_station_pos_height();
                basePosStatus++;
            }
        } else if (baseRun == 2) { // SPP calcing
            stationStatus = 6;
        } else if (baseRun == 3) { // rtk calcing
            if (station_tcp_is_interactive()) {
                if (station_tcp_is_stream_timeout()) {
                    stationStatus = 2;
                } else {
                    stationStatus = 1;
                }
            } else {
                stationStatus = 0;
            } 
        }
    }

    len = string_append(http_response_body, "posCallback({");

    len += string_append(&http_response_body[len], "\"staM\":");
    tool_itoa(stationMode, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"staS\":");
    tool_itoa(stationStatus, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    if (mGnssInsSystem.mlc_STATUS == 4) {
        if (inspvaxstr.pos_type != 1 && inspvaxstr.pos_type != 4 && inspvaxstr.pos_type != 5){
            vel_mode = 2; // INS_FREE
        } else {
            vel_mode = g_gnss_sol.vel_mode;
        }

        len += string_append(&http_response_body[len], ",\"week\":");
        tool_itoa(inspvaxstr.header.gps_week, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"tow\":");
        tool_itod((double) inspvaxstr.header.gps_millisecs/1000, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"rtkm\":");
        tool_itoa(g_gnss_sol.gnss_fix_type, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"lat\":");
        tool_itod(inspvaxstr.latitude, temp, 8);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"lon\":");
        tool_itod(inspvaxstr.longitude, temp, 8);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"alt\":");
        tool_itod(inspvaxstr.height, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"svs\":");
        tool_itoa(g_gnss_sol.num_sats, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"hdop\":");
        tool_itof(g_gnss_sol.dops[2], temp, 1);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"age\":");
        tool_itof(g_gnss_sol.sol_age, temp, 1);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"is\":");
        tool_itoa(inspvaxstr.ins_status, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"ipt\":");
        tool_itoa(inspvaxstr.pos_type, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"vm\":");
        tool_itoa(vel_mode, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"n\":");
        tool_itod(inspvaxstr.north_velocity, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"e\":");
        tool_itod(inspvaxstr.east_velocity, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"u\":");
        tool_itod(inspvaxstr.up_velocity, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"r\":");
        tool_itod(inspvaxstr.roll, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"p\":");
        tool_itod(inspvaxstr.pitch, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"h\":");
        tool_itod(inspvaxstr.azimuth, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);
        len += string_append(&http_response_body[len], ",");

    } else {

        len += string_append(&http_response_body[len], ",\"week\":");
        tool_itoa(g_gnss_sol.gps_week, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"tow\":");
        tool_itod((double) g_gnss_sol.gps_tow / 1000, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        if (stationMode == MODE_NTRIP_SERVER) {
            len += string_append(&http_response_body[len], ",\"rtkm\":");
            tool_itoa(basePosStatus, temp, 10);
            len += string_append(&http_response_body[len], (const char*)temp);

            len += string_append(&http_response_body[len], ",\"lat\":");
            tool_itod(basePosLatitude, temp, 8);
            len += string_append(&http_response_body[len], (const char*)temp);

            len += string_append(&http_response_body[len], ",\"lon\":");
            tool_itod(basePosLongitude, temp, 8);
            len += string_append(&http_response_body[len], (const char*)temp);

            len += string_append(&http_response_body[len], ",\"alt\":");
            tool_itod(basePosHeight, temp, 3);
            len += string_append(&http_response_body[len], (const char*)temp);

        } else {
            len += string_append(&http_response_body[len], ",\"rtkm\":");
            tool_itoa(g_gnss_sol.gnss_fix_type, temp, 10);
            len += string_append(&http_response_body[len], (const char*)temp);

            len += string_append(&http_response_body[len], ",\"lat\":");
            tool_itod(g_gnss_sol.latitude * RAD_TO_DEG, temp, 8);
            len += string_append(&http_response_body[len], (const char*)temp);

            len += string_append(&http_response_body[len], ",\"lon\":");
            tool_itod(g_gnss_sol.longitude * RAD_TO_DEG, temp, 8);
            len += string_append(&http_response_body[len], (const char*)temp);

            len += string_append(&http_response_body[len], ",\"alt\":");
            tool_itod(g_gnss_sol.height, temp, 3);
            len += string_append(&http_response_body[len], (const char*)temp);
        }

        len += string_append(&http_response_body[len], ",\"svs\":");
        tool_itoa(g_gnss_sol.num_sats, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"hdop\":");
        tool_itof(g_gnss_sol.dops[2], temp, 1);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"age\":");
        tool_itof(g_gnss_sol.sol_age, temp, 1);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"is\":");
        tool_itoa(inspvaxstr.ins_status, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"ipt\":");
        tool_itoa(inspvaxstr.pos_type, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"vm\":");
        tool_itoa(g_gnss_sol.vel_mode, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"n\":");
        tool_itof(g_gnss_sol.vel_ned[0], temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"e\":");
        tool_itof(g_gnss_sol.vel_ned[1], temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"u\":");
        tool_itof(-g_gnss_sol.vel_ned[2], temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"r\":0.000,");
        len += string_append(&http_response_body[len], "\"p\":0.000,");
        len += string_append(&http_response_body[len], "\"h\":0.000,");
    }

    memset(gps, 0, 80);
    memset(bds, 0, 80);
    memset(glo, 0, 60);
    memset(gal, 0, 60);

    for (i = 0; i < g_gnss_sol.rov_n; i++) {
        tool_itoa(g_gnss_sol.rov_satellite[i].satelliteId, temp, 10);
        switch (g_gnss_sol.rov_satellite[i].systemId)
        {
        case 0:
            gps_n++;
            if (gps_n > 1) {
                gps_len += string_append(&gps[gps_len], ",");
            }
            gps_len += string_append(&gps[gps_len], temp);
            break;
        case 1:
            glo_n++;
            if (glo_n > 1) {
                glo_len += string_append(&glo[glo_len], ",");
            }
            glo_len += string_append(&glo[glo_len], temp);
            break;
        case 2:
            gal_n++;
            if (gal_n > 1) {
                gal_len += string_append(&gal[gal_len], ",");
            }
            gal_len += string_append(&gal[gal_len], temp);
            break;
        case 4:
            bds_n++;
            if (bds_n > 1) {
                bds_len += string_append(&bds[bds_len], ",");
            }
            bds_len += string_append(&bds[bds_len], temp);
            break;
        default:
            break;
        }
    }

    len += string_append(&http_response_body[len], "\"bds\":\"(");
    tool_itoa(bds_n, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], "):");
    len += string_append(&http_response_body[len], bds);

    len += string_append(&http_response_body[len], "\",\"gps\":\"(");
    tool_itoa(gps_n, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], "):");
    len += string_append(&http_response_body[len], gps);

    len += string_append(&http_response_body[len], "\",\"gps\":\"(");
    tool_itoa(gps_n, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], "):");
    len += string_append(&http_response_body[len], gps);

    len += string_append(&http_response_body[len], "\",\"glo\":\"(");
    tool_itoa(glo_n, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], "):");
    len += string_append(&http_response_body[len], glo);

    len += string_append(&http_response_body[len], "\",\"gal\":\"(");
    tool_itoa(gal_n, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], "):");
    len += string_append(&http_response_body[len], gal);

    len += string_append(&http_response_body[len], "\"})");

    js_len = string_append(http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:");
    tool_itoa(len, temp, 10);
    js_len += string_append(&http_response[js_len], (const char*)temp);
    js_len += string_append(&http_response[js_len], "\r\n\r\n");
    js_len += string_append(&http_response[js_len], http_response_body);

	return http_response;
}

const char *user_config_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
	char userPacketType[3];
    char temp[20] = {0};
    uint32_t len = 0;
    uint32_t js_len = 0;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	memcpy(userPacketType, get_user_packet_type(), 2);
	userPacketType[2] = 0;

    len = string_append(http_response_body, "userConfigCallback({");

    len += string_append(&http_response_body[len], "\"userPacketType\":\"");
    len += string_append(&http_response_body[len], userPacketType);

    len += string_append(&http_response_body[len], "\",\"userPacketRate\":");
    tool_itoa(get_user_packet_rate(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"canEcuAddress\":");
    tool_itoa(get_can_ecu_address(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"canBaudrate\":");
    tool_itoa(get_can_baudrate(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"canPacketType\":");
    tool_itoa(get_can_packet_type(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"canPacketRate\":");
    tool_itoa(get_can_packet_rate(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"canTermresistor\":");
    tool_itoa(get_can_termresistor(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"canBaudrateDetect\":");
    tool_itoa(get_can_baudrate_detect(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"leverArmBx\":");
    tool_itof(get_lever_arm_bx(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"leverArmBy\":");
    tool_itof(get_lever_arm_by(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"leverArmBz\":");
    tool_itof(get_lever_arm_bz(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"pointOfInterestBx\":");
    tool_itof(get_point_of_interest_bx(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"pointOfInterestBy\":");
    tool_itof(get_point_of_interest_by(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"pointOfInterestBz\":");
    tool_itof(get_point_of_interest_bz(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"rotationRbvx\":");
    tool_itof(get_rotation_rbvx(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"rotationRbvy\":");
    tool_itof(get_rotation_rbvy(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"rotationRbvz\":");
    tool_itof(get_rotation_rbvz(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "})");

    js_len = string_append(http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:");
    tool_itoa(len, temp, 10);
    js_len += string_append(&http_response[js_len], (const char*)temp);
    js_len += string_append(&http_response[js_len], "\r\n\r\n");
    js_len += string_append(&http_response[js_len], http_response_body);

	return http_response;
}

const char *ethnet_config_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
    char temp[20] = {0};
    uint32_t len = 0;
    uint32_t js_len = 0;
    uint8_t *ip = NULL;

	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    len = string_append(http_response_body, "ethConfigCallback({");
    len += string_append(&http_response_body[len], "\"dhcp\":");
    tool_itoa(get_eth_mode(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"ip\":\"");
    ip = get_netif_ip();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"netmask\":\"");
    ip = get_netif_netmask();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"gateway\":\"");
    ip = get_netif_gateway();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"mac\":\"");
    ip = get_static_mac();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[4], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[5], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"sIp\":\"");
    ip = get_static_ip();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"sNetmask\":\"");
    ip = get_static_netmask();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"sGateway\":\"");
    ip = get_static_gateway();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\"})");

    js_len = string_append(http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:");
    tool_itoa(len, temp, 10);
    js_len += string_append(&http_response[js_len], (const char*)temp);
    js_len += string_append(&http_response[js_len], "\r\n\r\n");
    js_len += string_append(&http_response[js_len], http_response_body);

    return http_response;
}

const char *ethnet_summary_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
    char temp[20] = {0};
    uint32_t len = 0;
    uint32_t js_len = 0;
    uint8_t *ip = NULL;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    len = string_append(http_response_body, "ethSummaryCallback({");
    len += string_append(&http_response_body[len], "\"dhcp\":");
    tool_itoa(get_eth_mode(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"ip\":\"");
    ip = get_netif_ip();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"netmask\":\"");
    ip = get_netif_netmask();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\",\"gateway\":\"");
    ip = get_netif_gateway();
    tool_itoa(ip[0], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[1], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[2], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);
    len += string_append(&http_response_body[len], ".");
    tool_itoa(ip[3], temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "\"})");

    js_len = string_append(http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:");
    tool_itoa(len, temp, 10);
    js_len += string_append(&http_response[js_len], (const char*)temp);
    js_len += string_append(&http_response[js_len], "\r\n\r\n");
    js_len += string_append(&http_response[js_len], http_response_body);

    return http_response;
}

const char *work_config_js_handler(int index, int iNumParams, char *pcParam[], char *pcValue[])
{
    char temp[20] = {0};
    uint32_t len = 0;
    uint32_t js_len = 0;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    len = string_append(http_response_body, "workConfigCallback({");
    len += string_append(&http_response_body[len], "\"disp\":");
    tool_itoa(workpage_disp, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"cIp\":\"");
    len += string_append(&http_response_body[len], get_ntrip_client_ip());

    len += string_append(&http_response_body[len], "\",\"cPort\":");
    tool_itoa(get_ntrip_client_port(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"cMoP\":\"");
    len += string_append(&http_response_body[len], get_ntrip_client_mount_point());

    len += string_append(&http_response_body[len], "\",\"cUser\":\"");
    len += string_append(&http_response_body[len], get_ntrip_client_username());

    len += string_append(&http_response_body[len], "\",\"cPwd\":\"");
    len += string_append(&http_response_body[len], get_ntrip_client_password());

    len += string_append(&http_response_body[len], "\",\"aIp\":\"");
    len += string_append(&http_response_body[len], get_aceinna_client_ip());

    len += string_append(&http_response_body[len], "\",\"aPort\":");
    tool_itoa(get_aceinna_client_port(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"aMoP\":\"");
    len += string_append(&http_response_body[len], get_aceinna_client_mount_point());

    len += string_append(&http_response_body[len], "\",\"aUser\":\"");
    len += string_append(&http_response_body[len], get_aceinna_client_username());

    len += string_append(&http_response_body[len], "\",\"aPwd\":\"");
    len += string_append(&http_response_body[len], get_aceinna_client_password());

    len += string_append(&http_response_body[len], "\",\"sIp\":\"");
    len += string_append(&http_response_body[len], get_ntrip_server_ip());

    len += string_append(&http_response_body[len], "\",\"sPort\":");
    tool_itoa(get_ntrip_server_port(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"sMoP\":\"");
    len += string_append(&http_response_body[len], get_ntrip_server_mount_point());

    len += string_append(&http_response_body[len], "\",\"sPwd\":\"");
    len += string_append(&http_response_body[len], get_ntrip_server_password());

    len += string_append(&http_response_body[len], "\",\"bpt\":");
    tool_itoa(get_base_position_type(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"staID\":");
    tool_itoa(get_station_id(), temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"anth\":");
    tool_itod(get_antenna_height(), temp, 4);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"refLat\":");
    tool_itod(get_reference_latitude(), temp, 8);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"refLon\":");
    tool_itod(get_reference_longitude(), temp, 8);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"refHei\":");
    tool_itod(get_reference_height(), temp, 3);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], "})");

    js_len = string_append(http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:");
    tool_itoa(len, temp, 10);
    js_len += string_append(&http_response[js_len], (const char*)temp);
    js_len += string_append(&http_response[js_len], "\r\n\r\n");
    js_len += string_append(&http_response[js_len], http_response_body);

	return http_response;
}

void httpd_ssi_init(void)
{
	http_set_ssi_handler(ssi_handler, ssiTAGs, NUM_CONFIG_SSI_TAGS);
}

void httpd_cgi_init(void)
{
	http_set_cgi_handlers(cgiURIs, NUM_CONFIG_CGI_URIS);
}

void httpd_js_init(void)
{
	http_set_js_handlers(jsURIs, NUM_CONFIG_JS_URIS);
}


#endif
