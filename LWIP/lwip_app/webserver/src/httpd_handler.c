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
#include "station_tcp.h"
#include "aceinna_client_api.h"
#include "car_data.h"
#include "cJSON.h"
#include "ins_interface_API.h"

const char radioWheelPinMode[2][15] = {
	"radioWheel",
    "radioSPI",
};

const char radioCanMode[2][15] = {
	"radioCAR",
    "radioJ1939",
};

#define NUM_CONFIG_SSI_TAGS 6
#define NUM_CONFIG_CGI_URIS 7
#define NUM_CONFIG_JS_URIS 7

const char *ntrip_client_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *openarc_client_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *ntrip_server_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *ins_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *can_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *user_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *eth_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);

const char *position_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *work_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *ins_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *can_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *user_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *eth_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);
const char *eth_summary_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[]);


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

static void _escape_symbol(const char* src, char* dest)
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
        {"/ntripclientcfg.cgi", ntrip_client_cfg_cgi_handler},
        {"/openarcclientcfg.cgi", openarc_client_cfg_cgi_handler},
        {"/ntripservercfg.cgi", ntrip_server_cfg_cgi_handler},
		{"/inscfg.cgi", ins_cfg_cgi_handler},
		{"/cancfg.cgi", can_cfg_cgi_handler},
		{"/usercfg.cgi", user_cfg_cgi_handler},
        {"/ethcfg.cgi", eth_cfg_cgi_handler},
};

static const tJS jsURIs[] =
	{
        {"/position.js", position_js_handler},
        {"/workcfg.js", work_cfg_js_handler},
		{"/inscfg.js", ins_cfg_js_handler},
        {"/cancfg.js", can_cfg_js_handler},
        {"/usercfg.js", user_cfg_js_handler},
        {"/ethcfg.js", eth_cfg_js_handler},
        {"/ethsummary.js", eth_summary_js_handler},
};

// SSI Handler
static u16_t ssi_handler(int32_t index, char *pcInsert, int32_t iInsertLen)
{
    char temp[25] = {0};

	switch (index)
	{
    case 0:
        strcpy(pcInsert, PRODUCT_NAME_STRING);
        break;
    case 1:
        strcpy(pcInsert, (const char*)GetUnitVersion());
        break;
	case 2:
        if (strlen((const char *)platformBuildInfo()) < 25) {
            strcpy((char *)temp, (const char *)platformBuildInfo());
            for (uint8_t i = 0; i < strlen((const char *)temp); i++) {
                if (temp[i] == ' ') {
                    temp[i] = 0;
                    break;
                }
            }
            strcpy(pcInsert, (const char*)temp);
        }
		break;
	case 3:
        if (strlen((const char *)platformBuildInfo()) < 25) {
            strcpy((char *)temp, (const char *)platformBuildInfo());
            uint8_t i;
            for (i = 0; i < strlen((const char *)temp); i++) {
                if (temp[i] == ' ') {
                    temp[i] = 0;
                    break;
                }
            }
            strcpy(pcInsert, (const char*)&temp[i+1]);
        }
        break;
    case 4:
        tool_itoa(GetUnitSerialNum(), temp, 10);
        string_copy(pcInsert, (const char*)temp);

        sprintf(pcInsert, "%ld", GetUnitSerialNum());
        
        break;
    case 5:
        strcpy(pcInsert, APP_VERSION_STRING);
        break;
    default:
        break;
	}
	return string_length(pcInsert);
}

// CGI Handler
static int32_t _find_cgi_parameter(const char *pcToFind, char *pcParam[], int32_t iNumParams)
{
	int32_t loop;
    for (loop = 0; loop < iNumParams; loop++) {
        if (strcmp(pcToFind, pcParam[loop]) == 0) {
            return (loop);
        }
    }
    return (-1);
}


const char *ntrip_client_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	int32_t index_enable, index_ip, index_port, index_mountPoint, index_username, index_password;

	if (iNumParams == 6) {
        index_enable = _find_cgi_parameter("ncEn", pcParam, iNumParams);
        index_ip = _find_cgi_parameter("cIp", pcParam, iNumParams);
		index_port = _find_cgi_parameter("cPort", pcParam, iNumParams);
		index_mountPoint = _find_cgi_parameter("cMoP", pcParam, iNumParams);
        index_username = _find_cgi_parameter("cUser", pcParam, iNumParams);
		index_password = _find_cgi_parameter("cPwd", pcParam, iNumParams);

        if (index_enable != -1 && index_ip != -1 && index_port != -1 &&
            index_mountPoint != -1 && index_username != -1 && index_password != -1) {

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

const char *openarc_client_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	int32_t index_enable, index_ip, index_port, index_mountPoint, index_username, index_password;

	if (iNumParams == 6) {
        index_enable = _find_cgi_parameter("acEn", pcParam, iNumParams);
        index_ip = _find_cgi_parameter("aIp", pcParam, iNumParams);
		index_port = _find_cgi_parameter("aPort", pcParam, iNumParams);
		index_mountPoint = _find_cgi_parameter("aMoP", pcParam, iNumParams);
        index_username = _find_cgi_parameter("aUser", pcParam, iNumParams);
		index_password = _find_cgi_parameter("aPwd", pcParam, iNumParams);

        if (index_enable != -1 &&index_ip != -1 && index_port != -1 
            && index_mountPoint != -1 && index_username != -1 && index_password != -1) {

            set_aceinna_client_ip((const char *)pcValue[index_ip]);
            set_aceinna_client_port(atoi(pcValue[index_port]));
            set_aceinna_client_mount_point((const char *)pcValue[index_mountPoint]);
            set_aceinna_client_username((const char *)pcValue[index_username]);
            set_aceinna_client_password((const char *)pcValue[index_password]);

            if (atoi(pcValue[index_enable]) == 1) {
                set_station_mode(MODE_OPENARC_CLIENT);
                netif_station_tcp_config_changed();
            }

            SaveUserConfig();
        }
	}

    workpage_disp = 1;

	return "/workCfg.shtml";
}

const char *ntrip_server_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	int32_t index_enable, index_ip, index_port, index_mountPoint, index_password;
	int32_t index_station_id, index_antenna_height;
    int32_t index_reference_latitude, index_reference_longitude, index_reference_height, index_base_position_type;
    
	if (iNumParams == 11) {
        index_enable = _find_cgi_parameter("nsEn", pcParam, iNumParams);
        index_ip = _find_cgi_parameter("sIp", pcParam, iNumParams);
		index_port = _find_cgi_parameter("sPort", pcParam, iNumParams);
		index_mountPoint = _find_cgi_parameter("sMoP", pcParam, iNumParams);
		index_password = _find_cgi_parameter("sPwd", pcParam, iNumParams);
		index_station_id = _find_cgi_parameter("staID", pcParam, iNumParams);
		index_antenna_height = _find_cgi_parameter("anth", pcParam, iNumParams);
		index_reference_latitude = _find_cgi_parameter("refLat", pcParam, iNumParams);
        index_reference_longitude = _find_cgi_parameter("refLon", pcParam, iNumParams);
		index_reference_height = _find_cgi_parameter("refHei", pcParam, iNumParams);
        index_base_position_type = _find_cgi_parameter("bpt", pcParam, iNumParams);

		if (index_enable != -1 && index_ip != -1 && index_port != -1 && index_mountPoint != -1 
            && index_password != -1 && index_station_id != -1 && index_antenna_height != -1
            && index_reference_latitude != -1 && index_reference_longitude != -1 && index_reference_height != -1
            && index_base_position_type != -1) {
                            
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
                base_station_run_update();
                netif_station_tcp_config_changed();
            }

            SaveUserConfig();
		}
	}

    workpage_disp = 2;

	return "/workCfg.shtml";
}

const char *ins_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	int32_t index_prix, index_priy, index_priz;
	int32_t index_vrpx, index_vrpy, index_vrpz;
	int32_t index_userx, index_usery, index_userz;
	int32_t index_rbvx, index_rbvy, index_rbvz;

	if (iNumParams == 12) {
		index_prix = _find_cgi_parameter("prix", pcParam, iNumParams);
		index_priy = _find_cgi_parameter("priy", pcParam, iNumParams);
		index_priz = _find_cgi_parameter("priz", pcParam, iNumParams);
		index_vrpx = _find_cgi_parameter("vrpx", pcParam, iNumParams);
		index_vrpy = _find_cgi_parameter("vrpy", pcParam, iNumParams);
		index_vrpz = _find_cgi_parameter("vrpz", pcParam, iNumParams);
		index_userx = _find_cgi_parameter("userx", pcParam, iNumParams);
		index_usery = _find_cgi_parameter("usery", pcParam, iNumParams);
		index_userz = _find_cgi_parameter("userz", pcParam, iNumParams);
        index_rbvx = _find_cgi_parameter("rbvx", pcParam, iNumParams);
		index_rbvy = _find_cgi_parameter("rbvy", pcParam, iNumParams);
		index_rbvz = _find_cgi_parameter("rbvz", pcParam, iNumParams);

		if (index_prix != -1 && index_priy != -1 && index_priz != -1
			&& index_vrpx != -1 && index_vrpy != -1 && index_vrpz != -1
			&& index_userx != -1 && index_usery != -1 && index_userz != -1
            && index_rbvx != -1 && index_rbvy != -1 && index_rbvz != -1) {

            set_pri_lever_arm_bx(atof(pcValue[index_prix]));
            set_pri_lever_arm_by(atof(pcValue[index_priy]));
            set_pri_lever_arm_bz(atof(pcValue[index_priz]));
            set_vrp_lever_arm_bx(atof(pcValue[index_vrpx]));
            set_vrp_lever_arm_by(atof(pcValue[index_vrpy]));
            set_vrp_lever_arm_bz(atof(pcValue[index_vrpz]));
            set_user_lever_arm_bx(atof(pcValue[index_userx]));
            set_user_lever_arm_by(atof(pcValue[index_usery]));
            set_user_lever_arm_bz(atof(pcValue[index_userz]));
            set_rotation_rbvx(atof(pcValue[index_rbvx]));
            set_rotation_rbvy(atof(pcValue[index_rbvy]));
            set_rotation_rbvz(atof(pcValue[index_rbvz]));

            ins_init();

			SaveUserConfig();
		}
	}

	return "/insCfg.shtml";
}

const char *can_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
    int32_t index_wlmode, index_canMesg, index_gear, index_canmode;
    int32_t index_canEcuAddr, index_canBaudrate, index_canPacketType, index_canPacketRate, index_canTermresistor, index_canBaudrateDet;
    char buf[512] = {0};
    cJSON *root = NULL, *data = NULL;

    if (iNumParams == 10) {
        index_wlmode = _find_cgi_parameter("wlmode", pcParam, iNumParams);
		index_canMesg = _find_cgi_parameter("canMesg", pcParam, iNumParams);

        index_canEcuAddr = _find_cgi_parameter("canEcuAddr", pcParam, iNumParams);
        index_canBaudrate = _find_cgi_parameter("canBaudrate", pcParam, iNumParams);
        index_canPacketType = _find_cgi_parameter("canPacketType", pcParam, iNumParams);
        index_canPacketRate = _find_cgi_parameter("canPacketRate", pcParam, iNumParams);
        index_canTermresistor = _find_cgi_parameter("canTermresistor", pcParam, iNumParams);
        index_canBaudrateDet = _find_cgi_parameter("canBaudrateDet", pcParam, iNumParams);

        index_gear = _find_cgi_parameter("gear", pcParam, iNumParams);
        index_canmode = _find_cgi_parameter("canmode", pcParam, iNumParams);

        if (index_wlmode != -1 && index_canMesg != -1 && index_gear != -1 && index_canmode != -1
            && index_canEcuAddr != -1 && index_canBaudrate != -1 && index_canPacketType != -1 
            && index_canPacketRate != -1 && index_canTermresistor != -1 && index_canBaudrateDet != -1) {
            _escape_symbol((const char*)pcValue[index_canMesg], buf);

            if (!strcmp(pcValue[index_wlmode], "WHEELTICK")) {
                gUserConfiguration.wheeltick_pin_mode = 0;
            } else if (!strcmp(pcValue[index_wlmode], "SPI_NSS")) {
                gUserConfiguration.wheeltick_pin_mode = 1;
            }

            if (!strcmp(pcValue[index_canmode], "CAR")) {
                gUserConfiguration.can_mode = 0;
            } else if (!strcmp(pcValue[index_canmode], "J1939")) {
                gUserConfiguration.can_mode = 1;
            }

            set_can_ecu_address(atoi(pcValue[index_canEcuAddr]));
            set_can_baudrate(atoi(pcValue[index_canBaudrate]));
            set_can_packet_type(atoi(pcValue[index_canPacketType]));
            set_can_packet_rate(atoi(pcValue[index_canPacketRate]));
            set_can_termresistor(atoi(pcValue[index_canTermresistor]));
            set_can_baudrate_detect(atoi(pcValue[index_canBaudrateDet]));

            gUserConfiguration.odo_mesg[0].usage = 0;
            gUserConfiguration.odo_mesg[1].usage = 0;
            gUserConfiguration.odo_mesg[2].usage = 0;
            root = cJSON_Parse((const char *)buf);
            if (root != NULL) {
                data = root->child;
                uint8_t i = 0;
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
                        gUserConfiguration.odo_mesg[i].mesgID = atoi(MesgID->valuestring);
                        gUserConfiguration.odo_mesg[i].startbit = atoi(StartBit->valuestring);
                        gUserConfiguration.odo_mesg[i].length = atoi(Length->valuestring);
                        gUserConfiguration.odo_mesg[i].endian = atoi(Endian->valuestring);
                        gUserConfiguration.odo_mesg[i].sign = atoi(Sign->valuestring);
                        gUserConfiguration.odo_mesg[i].factor = atof(Factor->valuestring);
                        gUserConfiguration.odo_mesg[i].offset = atof(Offset->valuestring);
                        gUserConfiguration.odo_mesg[i].unit = atoi(Unit->valuestring);
                        gUserConfiguration.odo_mesg[i].source = atoi(Source->valuestring);
                        gUserConfiguration.odo_mesg[i].usage = 0x55;
                        i++;
                        if (i >= 3) {
                            break;
                        }
                    }
                    data = data->next;
                }
                cJSON_Delete(root);
            }

            _escape_symbol((const char*)pcValue[index_gear], buf);
            root = cJSON_Parse((const char *)buf);
            if (root != NULL) {
                cJSON *gear_p = cJSON_GetObjectItem(root, "P");
                cJSON *gear_r = cJSON_GetObjectItem(root, "R");
                cJSON *gear_n = cJSON_GetObjectItem(root, "N");
                cJSON *gear_d = cJSON_GetObjectItem(root, "D");

                if (gear_p != NULL && gear_r != NULL && gear_n != NULL && gear_d != NULL) {
                    gUserConfiguration.gears[0] = atoi(gear_p->valuestring);
                    gUserConfiguration.gears[1] = atoi(gear_r->valuestring);
                    gUserConfiguration.gears[2] = atoi(gear_n->valuestring);
                    gUserConfiguration.gears[3] = atoi(gear_d->valuestring);
                }
                cJSON_Delete(root);
            }

            car_can_initialize();
            SaveUserConfig();
        }
    }

    return "/canCfg.shtml";
}

const char *user_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	int32_t index_rawimu, index_bestgnss, index_inspvax, index_odospeed, index_satellites;
	int32_t index_gnins, index_gpgga, index_gprmc, index_pashr, index_gsa, index_zda, index_vtg;

	if (iNumParams == 12) {
		index_rawimu = _find_cgi_parameter("rawimu", pcParam, iNumParams);
		index_bestgnss = _find_cgi_parameter("bestgnss", pcParam, iNumParams);
		index_inspvax = _find_cgi_parameter("inspvax", pcParam, iNumParams);
		index_odospeed = _find_cgi_parameter("odospeed", pcParam, iNumParams);
		index_satellites = _find_cgi_parameter("satellites", pcParam, iNumParams);

		index_gnins = _find_cgi_parameter("gnins", pcParam, iNumParams);
		index_gpgga = _find_cgi_parameter("gpgga", pcParam, iNumParams);
		index_gprmc = _find_cgi_parameter("gprmc", pcParam, iNumParams);
		index_pashr = _find_cgi_parameter("pashr", pcParam, iNumParams);
        index_gsa = _find_cgi_parameter("gsa", pcParam, iNumParams);
		index_zda = _find_cgi_parameter("zda", pcParam, iNumParams);
		index_vtg = _find_cgi_parameter("vtg", pcParam, iNumParams);

		if (index_rawimu != -1 && index_bestgnss != -1 && index_inspvax != -1
			&& index_odospeed != -1 && index_satellites != -1 && index_gnins != -1
			&& index_gpgga != -1 && index_gprmc != -1 && index_pashr != -1
            && index_gsa != -1 && index_zda != -1 && index_vtg != -1) {
            
            set_rawimu_packet_rate(atoi(pcValue[index_rawimu]));
            set_bestgnss_packet_rate(atoi(pcValue[index_bestgnss]));
            set_inspvax_packet_rate(atoi(pcValue[index_inspvax]));
            set_odospeed_packet_rate(atoi(pcValue[index_odospeed]));
            set_satellites_packet_rate(atoi(pcValue[index_satellites]));
            set_gnins_packet_rate(atoi(pcValue[index_gnins]));
            set_gpgga_packet_rate(atoi(pcValue[index_gpgga]));
            set_gprmc_packet_rate(atoi(pcValue[index_gprmc]));
            set_pashr_packet_rate(atoi(pcValue[index_pashr]));
            set_gsa_packet_rate(atoi(pcValue[index_gsa]));
            set_zda_packet_rate(atoi(pcValue[index_zda]));
            set_vtg_packet_rate(atoi(pcValue[index_vtg]));

			SaveUserConfig();
		}
	}

	return "/userCfg.shtml";
}

const char *eth_cfg_cgi_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
    int32_t index_eth_mode, index_static_ip, index_static_netmask, index_static_gateway;
    ip_addr_t ip_addr_ip;
    ip_addr_t ip_addr_netmask;
    ip_addr_t ip_addr_gateway;
    int32_t eth_mode_t;
    uint8_t is_save = false;

	if (iNumParams == 1 || iNumParams == 4) {
        index_eth_mode = _find_cgi_parameter("ethmode", pcParam, iNumParams);
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
            index_static_ip = _find_cgi_parameter("staticIp", pcParam, iNumParams);
            index_static_netmask = _find_cgi_parameter("staticNetmask", pcParam, iNumParams);
            index_static_gateway = _find_cgi_parameter("staticGateway", pcParam, iNumParams);
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


// JS Handler
#define HTTP_JS_RESPONSE_SIZE 1024
CCMRAM char http_response[HTTP_JS_RESPONSE_SIZE];
CCMRAM char http_response_body[HTTP_JS_RESPONSE_SIZE];
static char gps[80], bds[80], glo[60], gal[60];
const char *position_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
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

    if (stationMode == MODE_NTRIP_SERVER) {
        if (baseRun == 1) {
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
        }
    }

    len = string_append(http_response_body, "posCallback({");

    len += string_append(&http_response_body[len], "\"staM\":");
    tool_itoa(stationMode, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    len += string_append(&http_response_body[len], ",\"staS\":");
    tool_itoa(sta_status, temp, 10);
    len += string_append(&http_response_body[len], (const char*)temp);

    if (get_mGnssInsSystem_mlc_STATUS() == 4 && stationMode != MODE_NTRIP_SERVER) {
        if (g_ins_sol.pos_type != 1 && g_ins_sol.pos_type != 4 && g_ins_sol.pos_type != 5){
            vel_mode = 2; // INS_FREE
        } else {
            vel_mode = g_gnss_sol.vel_mode;
        }

        len += string_append(&http_response_body[len], ",\"week\":");
        tool_itoa(g_ins_sol.gps_week, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"tow\":");
        tool_itod((double) g_ins_sol.gps_millisecs/1000, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"rtkm\":");
        tool_itoa(g_gnss_sol.gnss_fix_type, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"lat\":");
        tool_itod(g_ins_sol.latitude, temp, 8);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"lon\":");
        tool_itod(g_ins_sol.longitude, temp, 8);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"alt\":");
        tool_itod(g_ins_sol.height, temp, 3);
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
        tool_itoa(g_ins_sol.ins_status, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"ipt\":");
        tool_itoa(g_ins_sol.pos_type, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"vm\":");
        tool_itoa(vel_mode, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"n\":");
        tool_itod(g_ins_sol.north_velocity, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"e\":");
        tool_itod(g_ins_sol.east_velocity, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"u\":");
        tool_itod(g_ins_sol.up_velocity, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"r\":");
        tool_itod(g_ins_sol.roll, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"p\":");
        tool_itod(g_ins_sol.pitch, temp, 3);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"h\":");
        tool_itod(g_ins_sol.azimuth, temp, 3);
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
        tool_itoa(g_ins_sol.ins_status, temp, 10);
        len += string_append(&http_response_body[len], (const char*)temp);

        len += string_append(&http_response_body[len], ",\"ipt\":");
        tool_itoa(g_ins_sol.pos_type, temp, 10);
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

const char *work_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
    char temp[20] = {0};
    uint32_t len = 0;
    uint32_t js_len = 0;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    len = string_append(http_response_body, "workcfgCallback({");
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

const char *ins_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "inscfgCallback({\"prix\":%.3f,\"priy\":%.3f,\"priz\":%.3f,\"vrpx\":%.3f,\"vrpy\":%.3f,\"vrpz\":%.3f,\"userx\":%.3f,\"usery\":%.3f,\"userz\":%.3f,\"rbvx\":%.3f,\"rbvy\":%.3f,\"rbvz\":%.3f})",
            get_pri_lever_arm_bx(),
            get_pri_lever_arm_by(),
            get_pri_lever_arm_bz(),
            get_vrp_lever_arm_bx(),
            get_vrp_lever_arm_by(),
            get_vrp_lever_arm_bz(),
            get_user_lever_arm_bx(),
            get_user_lever_arm_by(),
            get_user_lever_arm_bz(),
            get_rotation_rbvx(),
            get_rotation_rbvy(),
            get_rotation_rbvz()
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *can_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
    char* buf = NULL;
    cJSON *data = NULL;
    uint8_t i;
    uint8_t radio_wheeltick_pin_mode, radio_can_mode;

    memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

    if (gUserConfiguration.wheeltick_pin_mode == 0) {
        radio_wheeltick_pin_mode = 0;
    } else {
        radio_wheeltick_pin_mode = 1;
    }
    if (gUserConfiguration.can_mode == 0) {
        radio_can_mode = 0;
    } else {
        radio_can_mode = 1;
    }

	sprintf((char *)http_response_body, "cancfgCallback({\"wlmode\":\"%s\",\"canmode\":\"%s\",\"canEcuAddr\":%d,\"canBaudrate\":%d,\"canPacketType\":%d,\"canPacketRate\":%d,\"canTermresistor\":%d,\"canBaudrateDet\":%d,\"canMesg\":[",
            radioWheelPinMode[radio_wheeltick_pin_mode],
            radioCanMode[radio_can_mode],
            get_can_ecu_address(),
            get_can_baudrate(),
            get_can_packet_type(),
            get_can_packet_rate(),
            get_can_termresistor(),
            get_can_baudrate_detect()
            );

    for (i = 0; i < 3; i++) {
        if (gUserConfiguration.odo_mesg[i].usage == 0x55) {
            data = cJSON_CreateObject();
            cJSON_AddItemToObject(data, "MesgID", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].mesgID));
            cJSON_AddItemToObject(data, "StartBit", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].startbit));
            cJSON_AddItemToObject(data, "Length", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].length));
            cJSON_AddItemToObject(data, "Endian", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].endian));
            cJSON_AddItemToObject(data, "Sign", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].sign));
            cJSON_AddItemToObject(data, "Factor", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].factor));
            cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].offset));
            cJSON_AddItemToObject(data, "Unit", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].unit));
            cJSON_AddItemToObject(data, "Source", cJSON_CreateNumber(gUserConfiguration.odo_mesg[i].source));

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
    cJSON_AddItemToObject(data, "P", cJSON_CreateNumber(gUserConfiguration.gears[0]));
    cJSON_AddItemToObject(data, "R", cJSON_CreateNumber(gUserConfiguration.gears[1]));
    cJSON_AddItemToObject(data, "N", cJSON_CreateNumber(gUserConfiguration.gears[2]));
    cJSON_AddItemToObject(data, "D", cJSON_CreateNumber(gUserConfiguration.gears[3]));
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

const char *user_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "usercfgCallback({\"rawimu\":%d,\"bestgnss\":%d,\"inspvax\":%d,\"odospeed\":%d,\"satellites\":%d,\"gnins\":%d,\"gpgga\":%d,\"gprmc\":%d,\"pashr\":%d,\"gsa\":%d,\"zda\":%d,\"vtg\":%d})",
            get_rawimu_packet_rate(),
            get_bestgnss_packet_rate(),
            get_inspvax_packet_rate(),
            get_odospeed_packet_rate(),
            get_satellites_packet_rate(),
            get_gnins_packet_rate(),
            get_gpgga_packet_rate(),
            get_gprmc_packet_rate(),
            get_pashr_packet_rate(),
            get_gsa_packet_rate(),
            get_zda_packet_rate(),
            get_vtg_packet_rate()
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *eth_cfg_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
    uint8_t *ip = get_netif_ip();
    uint8_t *netmask = get_netif_netmask();
    uint8_t *gateway = get_netif_gateway();
    uint8_t *mac = get_static_mac();
    uint8_t *static_ip = get_static_ip();
    uint8_t *static_netmask = get_static_netmask();
    uint8_t *static_gateway = get_static_gateway();

	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "ethcfgCallback({\"dhcp\":%d,\"ip\":\"%d.%d.%d.%d\",\
\"netmask\":\"%d.%d.%d.%d\",\"gateway\":\"%d.%d.%d.%d\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\
\"sIp\":\"%d.%d.%d.%d\",\"sNetmask\":\"%d.%d.%d.%d\",\"sGateway\":\"%d.%d.%d.%d\"})",
            get_eth_mode(),
            ip[0], ip[1], ip[2], ip[3],
            netmask[0], netmask[1], netmask[2], netmask[3],
            gateway[0], gateway[1], gateway[2], gateway[3],
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
			static_ip[0], static_ip[1], static_ip[2], static_ip[3],
			static_netmask[0], static_netmask[1], static_netmask[2], static_netmask[3],
            static_gateway[0], static_gateway[1], static_gateway[2], static_gateway[3]
			);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen((const char*)http_response_body), http_response_body);

	return (char *)http_response;
}

const char *eth_summary_js_handler(int32_t index, int32_t iNumParams, char *pcParam[], char *pcValue[])
{
    uint8_t *ip = get_netif_ip();
    uint8_t *netmask = get_netif_netmask();
    uint8_t *gateway = get_netif_gateway();

	memset(http_response, 0, HTTP_JS_RESPONSE_SIZE);
	memset(http_response_body, 0, HTTP_JS_RESPONSE_SIZE);

	sprintf((char *)http_response_body, "ethsummaryCallback({\"dhcp\":%d,\"ip\":\"%d.%d.%d.%d\",\"netmask\":\"%d.%d.%d.%d\",\"gateway\":\"%d.%d.%d.%d\"})",
            get_eth_mode(),
            ip[0], ip[1], ip[2], ip[3],
            netmask[0], netmask[1], netmask[2], netmask[3],
            gateway[0], gateway[1], gateway[2], gateway[3]
            );

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
