#include "httpd.h"
#include "lwip/tcp.h"
#include "fs.h"
#include "LwipComm.h"
#include "appVersion.h"
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "ntripClient.h"

#include "calibrationAPI.h"
#include "platformAPI.h"
#include "UserConfiguration.h"


const char radioRTKType[2][15] = {
	"radioLocalRTK",
    "radioCloudRTK",
};

const char radioEthMode[2][15] = {
	"radioDhcp",
    "radioStatic",
};

const char radioBaseStream[2][15] = {
    "radioBaseOff",
	"radioBaseOn",
};

#define NUM_CONFIG_SSI_TAGS 8
#define NUM_CONFIG_CGI_URIS 4
#define NUM_CONFIG_JS_URIS 4

const char *NTRIP_CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *START_NTRIP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *USER_CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ETHNET_CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

const char *NTRIP_CONFIG_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *NTRIP_STATE_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *USER_CONFIG_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *ETHNET_CONFIG_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);


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
        {"/NtripConfig.cgi", NTRIP_CONFIG_CGI_Handler},
        {"/StartNtrip.cgi", START_NTRIP_CGI_Handler},		
		{"/UserConfig.cgi", USER_CONFIG_CGI_Handler},
		{"/EthnetConfig.cgi", ETHNET_CONFIG_CGI_Handler},
};

static const tJS jsURIs[] =
	{
		{"/NtripConfig.js", NTRIP_CONFIG_JS_Handler},
        {"/NtripState.js", NTRIP_STATE_JS_Handler},
		{"/UserConfig.js", USER_CONFIG_JS_Handler},
        {"/EthnetConfig.js", ETHNET_CONFIG_JS_Handler},
};

// SSI Handler
static u16_t SSIHandler(int iIndex, char *pcInsert, int iInsertLen)
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
        if (NTRIP_client_state == NTRIP_STATE_INTERACTIVE)
        {
            strcpy(pcInsert, "CONNECTED");
        }
        else
        {
            strcpy(pcInsert, "DISCONNECTED");
        }
        break;
    case 7:
        if (NTRIP_client_state == NTRIP_STATE_INTERACTIVE && ntripStreamCount < NTRIP_STREAM_CONNECTED_MAX_COUNT)
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

const char *ETHNET_CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
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
				if ((!strcmp(pcValue[index_eth_mode], "dhcp") && gUserConfiguration.ethMode != ETHMODE_DHCP) ||
					(!strcmp(pcValue[index_eth_mode], "static") && gUserConfiguration.ethMode != ETHMODE_STATIC) ||
                    memcmp(gUserConfiguration.staticIp, &ip_addr_ip, 4) != 0 ||
					memcmp(gUserConfiguration.netmask, &ip_addr_netmask, 4) != 0 ||
					memcmp(gUserConfiguration.gateway, &ip_addr_gateway, 4) != 0)
				{
                    if (!strcmp(pcValue[index_eth_mode], "dhcp") && gUserConfiguration.ethMode == ETHMODE_STATIC)
					{
						gUserConfiguration.ethMode = ETHMODE_DHCP;
					}
					else if (!strcmp(pcValue[index_eth_mode], "static") && gUserConfiguration.ethMode == ETHMODE_DHCP)
					{
						gUserConfiguration.ethMode = ETHMODE_STATIC;
					}

					memcpy(gUserConfiguration.staticIp, &ip_addr_ip, 4);
					memcpy(gUserConfiguration.netmask, &ip_addr_netmask, 4);
					memcpy(gUserConfiguration.gateway, &ip_addr_gateway, 4);

                    netif_ethernet_config_changed();

					SaveUserConfig();
				}
				else
				{
					// no change
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

const char *NTRIP_CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_ip, index_port, index_mountPoint, index_username, index_password, index_base;
	uint16_t port;

	if (iNumParams == 6)
	{
        index_ip = FindCGIParameter("ip", pcParam, iNumParams);
		index_port = FindCGIParameter("port", pcParam, iNumParams);
		index_mountPoint = FindCGIParameter("mountPoint", pcParam, iNumParams);
        index_username = FindCGIParameter("username", pcParam, iNumParams);
		index_password = FindCGIParameter("password", pcParam, iNumParams);
        index_base = FindCGIParameter("baseStream", pcParam, iNumParams);
		
		if (index_ip != -1 && index_port != -1 && index_mountPoint != -1
            && index_username != -1 && index_password != -1 && index_base != -1)
		{
			if (strlen(pcValue[index_mountPoint]) > 3 && pcValue[index_mountPoint][0] == '%' 	
				&& pcValue[index_mountPoint][1] == '2'&& pcValue[index_mountPoint][2] == 'F')
			{
				port = atoi(pcValue[index_port]);

				if (strcmp((const char*)gUserConfiguration.ip, pcValue[index_ip]) != 0
					|| gUserConfiguration.port != port 
					|| strcmp((const char*)&gUserConfiguration.mountPoint[1], &pcValue[index_mountPoint][3]) != 0
                    || strcmp((const char*)gUserConfiguration.username, pcValue[index_username]) != 0
					|| strcmp((const char*)gUserConfiguration.password, pcValue[index_password]) != 0)
				{
                    memset(gUserConfiguration.ip, 0, sizeof(gUserConfiguration.ip));
					strcpy((char*)gUserConfiguration.ip, (const char*)pcValue[index_ip]);
					gUserConfiguration.port = port;
                    memset(gUserConfiguration.mountPoint, 0, sizeof(gUserConfiguration.mountPoint));
					gUserConfiguration.mountPoint[0] = '/';
					strcpy((char*)&gUserConfiguration.mountPoint[1], (const char*)&pcValue[index_mountPoint][3]);
                    memset(gUserConfiguration.username, 0, sizeof(gUserConfiguration.username));
					strcpy((char*)gUserConfiguration.username, (const char*)pcValue[index_username]);
                    memset(gUserConfiguration.password, 0, sizeof(gUserConfiguration.password));
                    strcpy((char*)gUserConfiguration.password, (const char*)pcValue[index_password]);

                    netif_ntrip_config_changed();

					SaveUserConfig();
				}
				else
				{
					// no change
				}

                if (!strcmp(pcValue[index_base], "BaseOn"))
                {
                    NTRIP_base_stream = BSAE_ON;
                }
                else if (!strcmp(pcValue[index_base], "BaseOff"))
                {
                    NTRIP_base_stream = BSAE_OFF;
                }
			}
			else
			{
				// err ip
			}
		}
	}

	return "/NtripCfg.shtml";
}

const char *USER_CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int index_userUartBaudRate, index_userPacketType, index_userPacketRate;
	int index_lpfAccelFilterFreq, index_lpfRateFilterFreq;
	int index_orientation, index_profile;
	int index_leverArmBx, index_leverArmBy, index_leverArmBz;
	int index_pointOfInterestBx, index_pointOfInterestBy, index_pointOfInterestBz;
	int index_rotationRbvx, index_rotationRbvy, index_rotationRbvz;
	uint8_t ori[7] = "+x+y+z";

	if (iNumParams == 16)
	{
		index_userUartBaudRate = FindCGIParameter("userUartBaudRate", pcParam, iNumParams);
		index_userPacketType = FindCGIParameter("userPacketType", pcParam, iNumParams);
		index_userPacketRate = FindCGIParameter("userPacketRate", pcParam, iNumParams);
		index_lpfAccelFilterFreq = FindCGIParameter("lpfAccelFilterFreq", pcParam, iNumParams);
		index_lpfRateFilterFreq = FindCGIParameter("lpfRateFilterFreq", pcParam, iNumParams);
		index_orientation = FindCGIParameter("orientation", pcParam, iNumParams);
		index_profile = FindCGIParameter("profile", pcParam, iNumParams);
		index_leverArmBx = FindCGIParameter("leverArmBx", pcParam, iNumParams);
		index_leverArmBy = FindCGIParameter("leverArmBy", pcParam, iNumParams);
		index_leverArmBz = FindCGIParameter("leverArmBz", pcParam, iNumParams);
		index_pointOfInterestBx = FindCGIParameter("pointOfInterestBx", pcParam, iNumParams);
		index_pointOfInterestBy = FindCGIParameter("pointOfInterestBy", pcParam, iNumParams);
		index_pointOfInterestBz = FindCGIParameter("pointOfInterestBz", pcParam, iNumParams);
		index_rotationRbvx = FindCGIParameter("rotationRbvx", pcParam, iNumParams);
		index_rotationRbvy = FindCGIParameter("rotationRbvy", pcParam, iNumParams);
		index_rotationRbvz = FindCGIParameter("rotationRbvz", pcParam, iNumParams);

		if (index_userUartBaudRate != -1 && index_userPacketType != -1 && index_userPacketRate != -1 
			&& index_lpfAccelFilterFreq != -1 && index_lpfRateFilterFreq != -1 
			&& index_orientation != -1 && index_profile != -1
			&& index_leverArmBx != -1 && index_leverArmBy != -1 && index_leverArmBz != -1
			&& index_pointOfInterestBx != -1 && index_pointOfInterestBy != -1 && index_pointOfInterestBz != -1
			&& index_rotationRbvx != -1 && index_rotationRbvy != -1 && index_rotationRbvz != -1)
		{
			gUserConfiguration.userUartBaudRate = atol(pcValue[index_userUartBaudRate]);
            memset(gUserConfiguration.userPacketType, 0, sizeof(gUserConfiguration.userPacketType));
			strcpy((char*)gUserConfiguration.userPacketType, (const char*)pcValue[index_userPacketType]);
			gUserConfiguration.userPacketRate = atol(pcValue[index_userPacketRate]);
			gUserConfiguration.lpfAccelFilterFreq = atol(pcValue[index_lpfAccelFilterFreq]);
			gUserConfiguration.lpfRateFilterFreq = atol(pcValue[index_lpfRateFilterFreq]);
            memset(gUserConfiguration.orientation, 0, sizeof(gUserConfiguration.orientation));
			strcpy((char*)gUserConfiguration.orientation, (const char*)ori);
			gUserConfiguration.profile = atoi(pcValue[index_profile]);
			gUserConfiguration.leverArmBx = atof(pcValue[index_leverArmBx]);
			gUserConfiguration.leverArmBy = atof(pcValue[index_leverArmBy]);
			gUserConfiguration.leverArmBz = atof(pcValue[index_leverArmBz]);
			gUserConfiguration.pointOfInterestBx = atof(pcValue[index_pointOfInterestBx]);
			gUserConfiguration.pointOfInterestBy = atof(pcValue[index_pointOfInterestBy]);
			gUserConfiguration.pointOfInterestBz = atof(pcValue[index_pointOfInterestBz]);
			gUserConfiguration.rotationRbvx = atof(pcValue[index_rotationRbvx]);
			gUserConfiguration.rotationRbvy = atof(pcValue[index_rotationRbvy]);
			gUserConfiguration.rotationRbvz = atof(pcValue[index_rotationRbvz]);
			SaveUserConfig();
		}
	}

	return "/UserCfg.shtml";
}

const char *START_NTRIP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    if (NTRIP_client_start == NTRIP_START_OFF)
    {
        NTRIP_client_start = NTRIP_START_ON;
    }
    else
    {
        NTRIP_client_start = NTRIP_START_OFF;
    }
    
    return "/NtripCfg.shtml";
}

#define HTTP_JS_RESPONSE_SIZE 1024
CCMRAM uint8_t http_response[HTTP_JS_RESPONSE_SIZE];
CCMRAM uint8_t http_response_body[HTTP_JS_RESPONSE_SIZE];
// JS Handler
const char *ETHNET_CONFIG_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "EthnetConfigCallback({\"ethMode\":\"%s\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"defaultIp\":\"%d.%d.%d.%d\",\"defaultGateway\":\"%d.%d.%d.%d\",\"defaultNetmask\":\"%d.%d.%d.%d\"})",
            radioEthMode[gUserConfiguration.ethMode],
            gUserConfiguration.mac[0], gUserConfiguration.mac[1], gUserConfiguration.mac[2], gUserConfiguration.mac[3], gUserConfiguration.mac[4], gUserConfiguration.mac[5],
			gUserConfiguration.staticIp[0], gUserConfiguration.staticIp[1], gUserConfiguration.staticIp[2], gUserConfiguration.staticIp[3],
			gUserConfiguration.gateway[0], gUserConfiguration.gateway[1], gUserConfiguration.gateway[2], gUserConfiguration.gateway[3],
			gUserConfiguration.netmask[0], gUserConfiguration.netmask[1], gUserConfiguration.netmask[2], gUserConfiguration.netmask[3]);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *NTRIP_CONFIG_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "NtripConfigCallback({\"ip\":\"%s\",\"port\":\"%d\",\"mountPoint\":\"%s\",\"username\":\"%s\",\"password\":\"%s\",\"baseStream\":\"%s\"})",
			gUserConfiguration.ip,
			gUserConfiguration.port,
			gUserConfiguration.mountPoint,
            gUserConfiguration.username,
            gUserConfiguration.password,
            radioBaseStream[NTRIP_base_stream]
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *USER_CONFIG_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "UserConfigCallback({\"userUartBaudRate\":\"%llu\",\"userPacketType\":\"%s\",\"userPacketRate\":\"%llu\",\"lpfAccelFilterFreq\":\"%llu\",\"lpfRateFilterFreq\":\"%llu\",\"orientation\":\"%s\",\"profile\":\"%lu\",\"leverArmBx\":\"%f\",\"leverArmBy\":\"%f\",\"leverArmBz\":\"%f\",\"pointOfInterestBx\":\"%f\",\"pointOfInterestBy\":\"%f\",\"pointOfInterestBz\":\"%f\",\"rotationRbvx\":\"%f\",\"rotationRbvy\":\"%f\",\"rotationRbvz\":\"%f\"})",
			gUserConfiguration.userUartBaudRate,
			gUserConfiguration.userPacketType,
			gUserConfiguration.userPacketRate,
			gUserConfiguration.lpfAccelFilterFreq,
			gUserConfiguration.lpfRateFilterFreq,
			gUserConfiguration.orientation,
			gUserConfiguration.profile,
			gUserConfiguration.leverArmBx,
			gUserConfiguration.leverArmBy,
			gUserConfiguration.leverArmBz,
			gUserConfiguration.pointOfInterestBx,
			gUserConfiguration.pointOfInterestBy,
			gUserConfiguration.pointOfInterestBz,
			gUserConfiguration.rotationRbvx,
			gUserConfiguration.rotationRbvy,
			gUserConfiguration.rotationRbvz
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *NTRIP_STATE_JS_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    sprintf((char *)http_response_body, "NtripStateCallback({\"connect\":\"%s\",\"stream\":\"%s\"})",
        (NTRIP_client_state == NTRIP_STATE_INTERACTIVE)? "CONNECTED":"DISCONNECTED",
        (NTRIP_client_state == NTRIP_STATE_INTERACTIVE && ntripStreamCount < NTRIP_STREAM_CONNECTED_MAX_COUNT)? "AVAILABLE":"UNAVAILABLE");

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

void httpd_ssi_init(void)
{
	http_set_ssi_handler(SSIHandler, ssiTAGs, NUM_CONFIG_SSI_TAGS);
}

void httpd_cgi_init(void)
{
	http_set_cgi_handlers(cgiURIs, NUM_CONFIG_CGI_URIS);
}

void httpd_js_init(void)
{
	http_set_js_handlers(jsURIs, NUM_CONFIG_JS_URIS);
}
