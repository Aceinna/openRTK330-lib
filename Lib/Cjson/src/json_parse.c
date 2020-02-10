/*******************************************************************************
* File Name          : json_parse.c
* Author             : Daich
* Revision           : 1.0
* Date               : 29/09/2019
* Description        : json_parse.c
*
* HISTORY***********************************************************************
* 29/09/2019  |                                             | Daich
* Description: add send_rtk_json_message
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "json_parse.h"
#include "log.h"
#include "appVersion.h"
#include "configuration.h"
#include "ucb_packet.h"
#include "uart.h"
#include "platformAPI.h"
#ifndef SENSOR_UNUSED
	#include "calibrationAPI.h"
#endif
int change_item(char *text,char* json_to_write,char* key,char* value)
{
	char *out;
    cJSON *root;
	root = cJSON_Parse(text);
	if (!root)
    {
        printf("not json format\r\n");
        return json_false;
    }
	else
	{
        cJSON* item_replace = cJSON_GetObjectItem(root,"openrtk config");
        cJSON_ReplaceItemInObject(item_replace,key,cJSON_CreateString(value));
#if 0
        item = cJSON_GetObjectItem(root,"rtk config");
        memcpy(cJSON_GetObjectItem(item,key)->valuestring,value,strlen(value)); // how to modify value safety?
#endif
        out = cJSON_Print(root);    
        {
            memcpy(json_to_write,out,strlen(out) + 1);
        }           
        cJSON_Delete(root);
        free(out);
        return json_true;
	}
}


void create_json_object(cJSON** json)
{
    *json = cJSON_CreateObject();
}


char* get_rtk_json_item_value(cJSON *json,char* key)        //TODO:
{
    //int json_array_size = cJSON_GetArraySize();
#if 1
    if(json == NULL)
    {
        return NULL;
    }
    int json_item_count = cJSON_GetArraySize(json);
    for(int i=0; i < json_item_count; i++)   
    {
        cJSON * item = cJSON_GetArrayItem(json, i);    
        if(item->type == cJSON_Object)
        {
            cJSON* item_sub = cJSON_GetObjectItem(item,key);
            if(item_sub != NULL)
            {
                return item_sub->valuestring;
            }
#if 0
            for(int j=0;j < cJSON_GetArraySize(item); j++)
            {
                if
            }
#endif
        }
    }
#endif
    return NULL;
}

void send_rtk_json_message(cJSON* root)
{
	cJSON *fmt;
    char *out;
	//root=cJSON_CreateObject();
    if(root != NULL)
    {
        printf("root = %p\n",root);
        char compile_time[50] = __DATE__;
        //root = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "openrtk config", fmt=cJSON_CreateObject());
        cJSON_AddItemToObject(fmt, "Product Name", cJSON_CreateString(PRODUCT_NAME_STRING));
        cJSON_AddItemToObject(fmt, "Product PN", cJSON_CreateString((const char *)platformBuildInfo()));
        cJSON_AddItemToObject(fmt, "Product SN", cJSON_CreateNumber(GetUnitSerialNum()));
        cJSON_AddItemToObject(fmt, "Version", cJSON_CreateString(APP_VERSION_STRING));
        cJSON_AddItemToObject(fmt, "Compile Time", cJSON_CreateString(compile_time));  
        int packet_type = configGetPacketCode();
        char packet_type_str[5] = {0};
        packet_type_str[0] = packet_type >> 8;
        packet_type_str[1] = packet_type;
        cJSON_AddItemToObject(fmt, "UserPacketType", cJSON_CreateString(packet_type_str));
        out=cJSON_Print(root);
        //cJSON_Delete(root);       //not delete other code use
        printf("%s\n",out);
        uart_write_bytes(UART_BT,out,strlen(out),1);
        free(out);
    }
}
#if 0
{
	cJSON *root,*fmt;
    char *out;
	root=cJSON_CreateObject();
    char compile_time[50] = __DATE__;
	root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "openrtk config", fmt=cJSON_CreateObject());
	cJSON_AddItemToObject(fmt, "Product Name", cJSON_CreateString(PRODUCT_NAME_STRING));
	cJSON_AddItemToObject(fmt, "Product PN", cJSON_CreateString((const char *)platformBuildInfo()));
	cJSON_AddItemToObject(fmt, "Product SN", cJSON_CreateNumber(GetUnitSerialNum()));
   	cJSON_AddItemToObject(fmt, "Version", cJSON_CreateString(APP_VERSION_STRING));
    cJSON_AddItemToObject(fmt, "Compile Time", cJSON_CreateString(compile_time));  
    int packet_type = configGetPacketCode();
    char packet_type_str[5] = {0};
    packet_type_str[0] = packet_type >> 8;
    packet_type_str[1] = packet_type;
	cJSON_AddItemToObject(fmt, "UserPacketType", cJSON_CreateString(packet_type_str));
	out=cJSON_Print(root);
    cJSON_Delete(root);
    printf("%s\n",out);
    uart_write_bytes(UART_BT,out,strlen(out),1);
    free(out);
}
#endif