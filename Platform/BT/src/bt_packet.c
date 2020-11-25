#include "bt_packet.h"
#include <stdlib.h>
#include <string.h>
#include "app_version.h"
#include "calibrationAPI.h"
#include "platformAPI.h"
#include "user_config.h"
#include "uart.h"
#include "cJSON.h"

char esp_bt_cmd[BT_CMD_MAX][CMD_MAX_LEN] = 
{
    "get rtk message",
    "get rtk sn"
};

static char get_strtrim(char *pstr,char *dest,uint32_t len)  
{
    char cmp[len + 1];
    int j = 0;
	for(int i = 0;i < len;i++)
    {
        if((*(pstr + i) != ' ') && (*(pstr + i) != '\r') && (*(pstr + i) != '\n'))
        {
            cmp[j++] = *(pstr + i);
        }
    }
    cmp[j] = '\0';
	strcpy((char *)dest,(const char *)cmp);
    int len_now = strlen((const char*)dest);
    int count = len - len_now;
    return count;
}

static int get_bt_cmd_index(void* cmd)
{
    static int is_get_rtk_mes_rev_flag = 0;
    char cmd_trim[strlen(cmd) + 1];
    char bt_cmd_trim[CMD_MAX_LEN] = {0};
    get_strtrim(cmd,cmd_trim,strlen(cmd));
    for(int i = 0;i < BT_CMD_NUM;i++)
    {
        get_strtrim(esp_bt_cmd[i],bt_cmd_trim,strlen(esp_bt_cmd[i]));        
        //if(strcmp(bt_cmd_trim,cmd_trim) == 0)
        if(((is_get_rtk_mes_rev_flag == 0)&&(strstr(cmd_trim,bt_cmd_trim) != NULL)) \
        || ((is_get_rtk_mes_rev_flag == 1)&&(strcmp(cmd_trim,bt_cmd_trim) == 0)))
        {
            if((i == RTK_MES_CMD)&&(is_get_rtk_mes_rev_flag == 0))
            {
                is_get_rtk_mes_rev_flag = 1;
            }
            return i;
        }
        memset(bt_cmd_trim,0,sizeof(bt_cmd_trim));
    }
    return -1;
}

static void send_rtk_json_to_esp32(void)
{
    cJSON *root, *fmt;
    char *out;
    char compile_time[50] = __DATE__;
    
    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "openrtk config", fmt = cJSON_CreateObject());
    cJSON_AddItemToObject(fmt, "Product Name", cJSON_CreateString(PRODUCT_NAME_STRING));
    cJSON_AddItemToObject(fmt, "Product PN", cJSON_CreateString((const char *)platformBuildInfo()));
    cJSON_AddItemToObject(fmt, "Product SN", cJSON_CreateNumber(GetUnitSerialNum()));
    cJSON_AddItemToObject(fmt, "Version", cJSON_CreateString(APP_VERSION_STRING));
    cJSON_AddItemToObject(fmt, "Compile Time", cJSON_CreateString(compile_time));

    char packet_type_str[5] = "s1";
    uint16_t user_packet_rate = 100;

    cJSON_AddItemToObject(fmt, "userPacketType", cJSON_CreateString(packet_type_str));
    cJSON_AddItemToObject(fmt, "userPacketRate", cJSON_CreateNumber(user_packet_rate));

    float *ins_para = get_user_ins_para();
    cJSON_AddItemToObject(fmt, "leverArmBx", cJSON_CreateNumber(*ins_para));
    cJSON_AddItemToObject(fmt, "leverArmBy", cJSON_CreateNumber(*(ins_para + 1)));
    cJSON_AddItemToObject(fmt, "leverArmBz", cJSON_CreateNumber(*(ins_para + 2)));
    cJSON_AddItemToObject(fmt, "pointOfInterestBx", cJSON_CreateNumber(*(ins_para + 6)));
    cJSON_AddItemToObject(fmt, "pointOfInterestBy", cJSON_CreateNumber(*(ins_para + 7)));
    cJSON_AddItemToObject(fmt, "pointOfInterestBz", cJSON_CreateNumber(*(ins_para + 8)));
    cJSON_AddItemToObject(fmt, "rotationRbvx", cJSON_CreateNumber(*(ins_para + 9)));
    cJSON_AddItemToObject(fmt, "rotationRbvy", cJSON_CreateNumber(*(ins_para + 10)));
    cJSON_AddItemToObject(fmt, "rotationRbvz", cJSON_CreateNumber(*(ins_para + 11)));

    out = cJSON_Print(root);
    cJSON_Delete(root);

    uart_write_bytes(UART_BT,out,strlen(out),1);
    OS_Delay(10);
    uart_write_bytes(UART_BT, out, strlen(out), 1);
    free(out);
}

static void bt_app_json_parse(cJSON* root)
{
	BOOL result;
    cJSON *packetCode, *packetRate;
    cJSON *leverArmBx, *leverArmBy, *leverArmBz;
    cJSON *pointOfInterestBx, *pointOfInterestBy, *pointOfInterestBz;
    cJSON *rotationRbvx, *rotationRbvy, *rotationRbvz;
    
    packetCode = cJSON_GetObjectItem(root,"userPacketType");
    packetRate = cJSON_GetObjectItem(root,"userPacketRate");
    leverArmBx = cJSON_GetObjectItem(root, "leverArmBx");
    leverArmBy = cJSON_GetObjectItem(root, "leverArmBy");
    leverArmBz = cJSON_GetObjectItem(root, "leverArmBz");
    pointOfInterestBx = cJSON_GetObjectItem(root, "pointOfInterestBx");
    pointOfInterestBy = cJSON_GetObjectItem(root, "pointOfInterestBy");
    pointOfInterestBz = cJSON_GetObjectItem(root, "pointOfInterestBz");
    rotationRbvx = cJSON_GetObjectItem(root, "rotationRbvx");
    rotationRbvy = cJSON_GetObjectItem(root, "rotationRbvy");
    rotationRbvz = cJSON_GetObjectItem(root, "rotationRbvz");
    
    if (packetCode != NULL && packetRate != NULL 
        && leverArmBx != NULL && leverArmBy != NULL && leverArmBz != NULL
        && pointOfInterestBx != NULL && pointOfInterestBy != NULL && pointOfInterestBz != NULL
        && rotationRbvx != NULL && rotationRbvy != NULL && rotationRbvz != NULL){

        set_pri_lever_arm_bx(leverArmBx->valuedouble);
        set_pri_lever_arm_by(leverArmBy->valuedouble);
        set_pri_lever_arm_bz(leverArmBz->valuedouble);
        set_user_lever_arm_bx(pointOfInterestBx->valuedouble);
        set_user_lever_arm_by(pointOfInterestBy->valuedouble);
        set_user_lever_arm_bz(pointOfInterestBz->valuedouble);
        set_rotation_rbvx(rotationRbvx->valuedouble);
        set_rotation_rbvy(rotationRbvy->valuedouble);
        set_rotation_rbvz(rotationRbvz->valuedouble);

        ins_init();

        SaveUserConfig();
        uart_write_bytes(UART_BT,"##para received!##",strlen("##para received!##"),1);
        send_rtk_json_to_esp32();
    }
}

int bt_uart_parse(uint8_t* bt_buff)     //TODO:
{
    static int is_first_rev_bt_mes = 0;
    if (bt_buff != NULL)
    {
        if(is_first_rev_bt_mes == 0)
        {
            uart_write_bytes(UART_BT,"openrtk start run\r\n",strlen("openrtk start run\r\n"),1);
            is_first_rev_bt_mes = 1;
            return BT_CMD;
        }
        cJSON *root;
        root = cJSON_Parse((const char *)bt_buff);
        if (root != NULL)
        {
            bt_app_json_parse(root);
            cJSON_Delete(root);
            return RTK_JSON;
        }
        else
        {
            int cmd_type = get_bt_cmd_index(bt_buff);
            if(cmd_type == -1)
            {
                return BT_BASE;
            }
            switch(cmd_type)
            {
                case RTK_SN_CMD:
                    break;
                case RTK_MES_CMD:
                    send_rtk_json_to_esp32();
                    break;
                default:
                    break;
            }
            return BT_CMD;
        }
    }
    return BT_BASE;
}
