/*******************************************************************************
* File Name          : shell.c
* Author             : daich
* Revision           : 1.0
* Date               : 
* Description        : shell.c
*
* HISTORY***********************************************************************
* 10/12/2019  |                                             | Daich
* Description: created
* 16/12/2019  |                                             | Daich
* Description: add json cmd parse
* 06/01/2020  |                                             | Daich
* Description: add parse_json_cmd function to parse mobile bt app cmd
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "shell.h"      
#include "uart.h"
#include "log.h" 
#include "cJSON.h"
#include "driver.h"
#include "configuration.h"
#include "uart.h"
#include "json_parse.h"
#ifndef BAREMETAL_OS
    #include "UserConfiguration.h"
#endif
#define LINUX_CONSOLE
#define BT_CMD_NUM  2
#define CMD_MAX_LEN 20
#define DEBUG_CMD_NUM  2


volatile uint16_t   shell_rx_rdy = 0;
volatile uint16_t   shell_rdy_flag = 0;
volatile uint8_t    shell_rx_buff[SHELL_RX_MAX+1]="\0";
cJSON* rtk_json = NULL;


static volatile uint16_t    shell_rx_index = 0;

static volatile uint8_t     shell_tx_buff[SHELL_TX_MAX+1]="\0";
static volatile uint16_t    shell_tx_size  = 0;
static volatile uint16_t    shell_tx_index = 0;
static int bt_cmd_index = -1;

#ifndef BAREMETAL_OS
extern osSemaphoreId bt_uart_sem;
extern osSemaphoreId debug_uart_sem;
extern osMutexId bt_mutex;
extern CCMRAM UserConfigurationStruct gUserConfiguration;
#endif

static void bt_app_json_parse(cJSON* root);

enum BT_CMD_TYPE
{
    RTK_MES_CMD = 0,
    RTK_SN_CMD,
    BT_CMD_MAX
};
enum DEBUG_CMD_TYPE
{
    BT_NAME_CMD_DEBUG = 0,
    RTK_MES_CMD_DEBUG,
    DEBUG_CMD_MAX 
};

char debug_uart_cmd[DEBUG_CMD_MAX][CMD_MAX_LEN] = 
{
    "ls bt",
    "get rtk message"
};
char esp_bt_cmd[BT_CMD_MAX][CMD_MAX_LEN] = 
{
    "get rtk message",
    "get rtk sn"
};
extern ConfigurationStruct gConfiguration;
void debug_uart_handle(void)               //TODO:
{
    uint8_t dataBuffer[512];
    int bytes_in_buffer = 0;
    do
    {
        bytes_in_buffer = uart_read_bytes(UART_DEBUG,dataBuffer, 1,0);
        if(bytes_in_buffer > 0)
        {
            if( (shell_rx_index > 0) && ('\b' == dataBuffer[0]))  //\b
            {
                shell_rx_index = shell_rx_index <2? 0:shell_rx_index - 1;
                shell_rx_buff[shell_rx_index] = '\0';
#ifdef DEVICE_DEBUG
                uart_write_bytes(UART_DEBUG,(const char*)dataBuffer,1,1);
                uart_write_bytes(UART_DEBUG,(const char*)"\033[K",strlen("\033[K"),1);  //set back
#endif
            } 
            else if( shell_rx_index < SHELL_RX_MAX)
            {
                shell_rx_buff[shell_rx_index] = dataBuffer[0];
                shell_rx_index++;
#ifdef DEVICE_DEBUG
                uart_write_bytes(UART_DEBUG,(const char*)dataBuffer,1,1);
#endif
            }
            else
            {
                shell_rx_index = 0;
            }
#ifdef LINUX_CONSOLE
            if( (shell_rx_index >= 1) && ('\r' == shell_rx_buff[shell_rx_index-1]))      //\r
            {
                shell_rx_buff[shell_rx_index + 1] = '\0';
                shell_rx_rdy = shell_rx_index;
                shell_rx_index = 0;
                shell_rdy_flag = 1;
            }
#if 0
            if( (shell_rx_index > 0) && ('\b' == shell_rx_buff[shell_rx_index-1]))  //\b
            {
                shell_rx_index = shell_rx_index <2? 0:shell_rx_index - 1;
                uart_write_bytes(UART_DEBUG,(const char*)"\033[K",strlen("\033[K"),1);
            } 
#endif
#else
            if( (shell_rx_index >=2) && ('\r' == shell_rx_buff[shell_rx_index-2]) &&
                ('\n' == shell_rx_buff[shell_rx_index-1])  )      
            {
                shell_rx_rdy = shell_rx_index;
                shell_rx_index = 0;
            }
            else if( (shell_rx_index > 0) && ('\b' == shell_rx_buff[shell_rx_index-1]) )  
            {
                shell_rx_index = shell_rx_index <2? 0:shell_rx_index-2;
                printf(" \b");              
            } 
#endif
        }
    }while(bytes_in_buffer);
}

void shell_service(void)
{
    uint8_t* ptSrc;
    if(shell_rdy_flag == 1)
    {
        shell_rdy_flag = 0;
#ifdef LINUX_CONSOLE
        if(1 > shell_rx_rdy)
        {
            shell_rx_buff[0]  = 0;
            return;
        }
#else
        if(2 > shell_rx_rdy)
        {
            shell_rx_buff[0]  = 0;
            return;
        }
#endif
#ifdef LINUX_CONSOLE
        else if('\r' == shell_rx_buff[shell_rx_rdy-1])
#else
        else if( ('\r' == shell_rx_buff[shell_rx_rdy-2]) && ('\n' == shell_rx_buff[shell_rx_rdy-1]) )
#endif
        {
            ptSrc = (uint8_t *)shell_rx_buff;
#ifdef DEVICE_DEBUG
            printf("\r\n");     //new line
#endif
#ifdef LINUX_CONSOLE
            if(1 == shell_rx_rdy)
#else
            if(2 == shell_rx_rdy)
#endif
            {
#ifdef DEVICE_DEBUG
                printf("Shell:OK!\r\n");
#endif
            }
            else if(is_cmd_right(ptSrc,debug_uart_cmd[BT_NAME_CMD_DEBUG])) 
            {
                //uint8_t bt_buff[GPS_BUFF_SIZE];
                shell_rx_buff[0]  = 0;
                shell_rx_rdy      = 0;
                bt_cmd_index = BT_NAME_CMD_DEBUG;
#if 0
                char* bt_name = get_rtk_json_item_value(rtk_json,"bt_name");

                printf("bt_name = %s\n",bt_name);
#endif
#if 1
#ifndef BAREMETAL_OS
                osMutexWait(bt_mutex,portMAX_DELAY);
#endif
                uart_write_bytes(UART_BT, (char *)"esp32_config", sizeof("esp32_config"), 1);
#if 0
                OS_Delay(200);
                int BtLen = uart_read_bytes(UART_BT, bt_buff, GPS_BUFF_SIZE, 0);
                osMutexRelease(bt_mutex);
                if(BtLen > 0)
                {
                    cJSON *root;
                    root = cJSON_Parse((const char *)bt_buff);
                    if (root != NULL)
                    {
                        cJSON *esp_config,*item_btname,*item_bt_mac;
                        esp_config = cJSON_GetObjectItem(root,"esp32 config");
                        item_btname = cJSON_GetObjectItem(esp_config, "bt_name");
                        item_bt_mac = cJSON_GetObjectItem(esp_config, "esp32_serial");
                        RTK_LOG(ATS_LOG_MES,"ESP32 BT NAME","%s",item_btname->valuestring);
                        RTK_LOG(ATS_LOG_MES,"ESP32 BT MAC","%s",item_bt_mac->valuestring);
                        free(esp_config);
                        free(item_btname);
                        free(item_bt_mac);
                    }
                    free(root);
                }
                else
                {
                    RTK_LOG(ATS_LOG_MES,"console","%s","time out");
                }
#endif
#endif
            }
            else
            {
#ifdef DEVICE_DEBUG
                printf("\r\nAT: Cmd Error:\r\n"); 
#endif
            }
            shell_rx_buff[0]  = 0;
            shell_rx_rdy      = 0;
        }
    }
}


char get_strtrim(char *pstr,char *dest,uint32_t len)  
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

char strtrimall(unsigned char *pstr,uint32_t len)  //p ' ' q vallue
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
	strcpy((char *)pstr,(const char *)cmp);
    int len_now = strlen((const char*)pstr);
    int count = len - len_now;
    return count;
}


bool is_cmd_right(void * buffer,void * cmd)
{
    strtrimall(buffer,strlen(buffer));
    strtrimall(cmd,strlen(cmd));
    if(strcmp((char*)buffer,(char*)cmd) == 0)
    {
        return true;
    }
    return false;
}

void ConsoleTask(void const *argument)
{
	while (1)
	{
        debug_uart_handle();
        shell_service();
        OS_Delay(20);
	}
}

int get_bt_cmd_index(void* cmd)
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
            switch(bt_cmd_index)
            {
                case BT_NAME_CMD_DEBUG:
                {
                    get_rtk_json_item_value(root,"bt_name");
#ifdef DEVICE_DEBUG
                    printf("bt_name = %s\r\n",bt_name);
#endif
                    cJSON_Delete(root);
                    return RTK_JSON;
                    break;
                }
                default:
                    cJSON_Delete(root);
                    return RTK_JSON;
                    break;
            }
        }
#if 0
            cJSON *item_packet_type;
            item_packet_type = cJSON_GetObjectItem(root,"UserPacketType");
            if(item_packet_type != NULL)
            {
                RTK_LOG(ATS_LOG_MES,"UserPacketType","%s",item_packet_type->valuestring);
                int packet_type = 0;
                sscanf(item_packet_type->valuestring, "%x", &packet_type);
                gConfiguration.packetCode = packet_type;
                printf("gConfiguration.packetCode = %x\r\n",gConfiguration.packetCode);
            }
            free(item_packet_type);
            return RTK_JSON;
#endif
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
                    if(rtk_json == NULL)
                    {
                        create_json_object(&rtk_json);
                    }
                    send_rtk_json_message(rtk_json);
                    break;
                default:
                    break;
            }
            return BT_CMD;
        }

    }
    return BT_BASE;
}

void parse_debug_cmd()
{
    //res = osSemaphoreWait(debug_uart_sem, 0);
    //if(osSemaphoreWait(p_uart_obj[UART_DEBUG]->uart_idle_sem,0) == osOK)
    if(uart_sem_wait(UART_DEBUG,0) == RTK_SEM_OK)
    {
        debug_uart_handle();
        shell_service();
    }
}

#if 0
int get_bt_cmd_index()
{
    return bt_cmd_index;
}
#endif

static void bt_app_json_parse(cJSON* root)
{

#ifndef BAREMETAL_OS
    cJSON *leverArmBx, *leverArmBy, *leverArmBz;
    cJSON *pointOfInterestBx, *pointOfInterestBy, *pointOfInterestBz;
    cJSON *rotationRbvx, *rotationRbvy, *rotationRbvz;
    cJSON *packetCode;
    packetCode = cJSON_GetObjectItem(root,"UserPacketType");
    leverArmBx = cJSON_GetObjectItem(root, "leverArmBx");
    leverArmBy = cJSON_GetObjectItem(root, "leverArmBy");
    leverArmBz = cJSON_GetObjectItem(root, "leverArmBz");
    pointOfInterestBx = cJSON_GetObjectItem(root, "pointOfInterestBx");
    pointOfInterestBy = cJSON_GetObjectItem(root, "pointOfInterestBy");
    pointOfInterestBz = cJSON_GetObjectItem(root, "pointOfInterestBz");
    rotationRbvx = cJSON_GetObjectItem(root, "rotationRbvx");
    rotationRbvy = cJSON_GetObjectItem(root, "rotationRbvy");
    rotationRbvz = cJSON_GetObjectItem(root, "rotationRbvz");
    if(packetCode != NULL)
    {
        gConfiguration.packetCode = ((uint16_t)*(packetCode->valuestring) << 8) + *(packetCode->valuestring + 1);
        uint16_t type = ((uint16_t)*(packetCode->valuestring + 1) << 8) + *(packetCode->valuestring);
        setUserPacketType((uint8_t *)&type,TRUE);
        strcpy((char*)gUserConfiguration.userPacketType, (const char*)packetCode->valuestring);
        SaveUserConfig();
    }
    if(leverArmBx != NULL)
    {
        if(leverArmBx->type == cJSON_String)
        {
            gUserConfiguration.leverArmBx = (strlen(leverArmBx->valuestring) == 0)? \
            gUserConfiguration.leverArmBx : atof(leverArmBx->valuestring);
        }
        else if(leverArmBx->type == cJSON_Number)
        {
            gUserConfiguration.leverArmBx = (leverArmBx->valuedouble);
        }
    }
    if(leverArmBy != NULL)
    {
        if(leverArmBy->type == cJSON_String)
        {
            gUserConfiguration.leverArmBy = (strlen(leverArmBy->valuestring) == 0)? \
            gUserConfiguration.leverArmBy : atof(leverArmBy->valuestring);
        }
        else if(leverArmBy->type == cJSON_Number)
        {
            gUserConfiguration.leverArmBy = leverArmBy->valuedouble;
        }
    }
    if(leverArmBz != NULL)
    {
        if(leverArmBz->type == cJSON_String)
        {
            gUserConfiguration.leverArmBz = (strlen(leverArmBz->valuestring) == 0)? \
            gUserConfiguration.leverArmBz : atof(leverArmBz->valuestring);
        }
        else if(leverArmBz->type == cJSON_Number)
        {
            gUserConfiguration.leverArmBz = leverArmBz->valuedouble;
        }
    }
    if(pointOfInterestBx != NULL)
    {
        if(pointOfInterestBx->type == cJSON_String)
        {
            gUserConfiguration.pointOfInterestBx = (strlen(pointOfInterestBx->valuestring) == 0)? \
            gUserConfiguration.pointOfInterestBx : atof(pointOfInterestBx->valuestring);
        }
        else if(pointOfInterestBx->type == cJSON_Number)
        {
            gUserConfiguration.pointOfInterestBx = pointOfInterestBx->valuedouble;
        }
    }
    if(pointOfInterestBy != NULL)
    {
        if(pointOfInterestBy->type == cJSON_String)
        {
            gUserConfiguration.pointOfInterestBy = (strlen(pointOfInterestBy->valuestring) == 0)? \
            gUserConfiguration.pointOfInterestBy : atof(pointOfInterestBy->valuestring);
        }
        else if(pointOfInterestBy->type == cJSON_Number)
        {
            gUserConfiguration.pointOfInterestBy = pointOfInterestBy->valuedouble;
        }
    }
    if(pointOfInterestBz != NULL)
    {
        if(pointOfInterestBz->type == cJSON_String)
        {
            gUserConfiguration.pointOfInterestBz = (strlen(pointOfInterestBz->valuestring) == 0)? \
            gUserConfiguration.pointOfInterestBz : atof(pointOfInterestBz->valuestring);
        }
        else if(pointOfInterestBz->type == cJSON_Number)
        {
            gUserConfiguration.pointOfInterestBz = pointOfInterestBz->valuedouble;
        }
    }
    if(rotationRbvx != NULL)
    {
        if(rotationRbvx->type == cJSON_String)
        {
            gUserConfiguration.rotationRbvx = (strlen(rotationRbvx->valuestring) == 0)? \
            gUserConfiguration.rotationRbvx : atof(rotationRbvx->valuestring);
        }
        else if(rotationRbvx->type == cJSON_Number)
        {
            gUserConfiguration.rotationRbvx = rotationRbvx->valuedouble;
        }
    }
    if(rotationRbvy != NULL)
    {
        if(rotationRbvy->type == cJSON_String)
        {
            gUserConfiguration.rotationRbvy = (strlen(rotationRbvy->valuestring) == 0)? \
            gUserConfiguration.rotationRbvy : atof(rotationRbvy->valuestring);
        }
        else if(rotationRbvy->type == cJSON_Number)
        {
            gUserConfiguration.rotationRbvy = rotationRbvy->valuedouble;
        }
    }
    if(rotationRbvz != NULL)
    {
        if(rotationRbvz->type == cJSON_String)
        {
            gUserConfiguration.rotationRbvz = (strlen(rotationRbvz->valuestring) == 0)? \
            gUserConfiguration.rotationRbvz : atof(rotationRbvz->valuestring);
        }
        else if(rotationRbvz->type == cJSON_Number)
        {
            gUserConfiguration.rotationRbvz = rotationRbvz->valuedouble;
        }
    }
    if((leverArmBx != NULL) || (leverArmBy != NULL) || (leverArmBz != NULL) || \
        (pointOfInterestBx != NULL) || (pointOfInterestBy != NULL) || (pointOfInterestBz != NULL) || \
        (rotationRbvx != NULL) || (rotationRbvy != NULL) || (rotationRbvz != NULL))
        {
            SaveUserConfig();
            int size = sizeof(gUserConfiguration);            
            EEPROM_LoadUserConfig((void*)&gUserConfiguration, &size);
            uart_write_bytes(UART_BT,"##para received!##",strlen("##para received!##"),1);
            send_rtk_json_to_esp32();
        }

#endif
}