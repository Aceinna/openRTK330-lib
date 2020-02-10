/*******************************************************************************
* File Name          : log.c
* Author             : Daich
* Revision           : 1.0
* Date               : 19/09/2019
* Description        : log source file
*
* HISTORY***********************************************************************
* 19/09/2019  |                                             | Daich
*
*******************************************************************************/
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include "stm32f4xx_hal.h"
#include "log.h"
#include "uart.h"

#define APP_TAG     "ACEINNA_OPENRTK"
#define STDOUT_FILENO   1
#define STDERR_FILENO   2


ats_log_level_t LOG_LEVEL = ATS_LOG_DEBUG;

const char* log_type[7] = {
    "none",
    "error",
    "warn",
    "info",
    "mes",
    "dbg",
    "test",
};


__weak int _write(int fd, char* ptr, int len)
{
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
    {
        //uart_write_bytes(UART_DEBUG,ptr,len,1);
        HAL_UART_Transmit(&huart_debug, (uint8_t *)ptr, len, 0xFFFF); // FOR TESTING ONLY?
        return len;
    }
    return 0;
}

void ats_log_write(ats_log_level_t level, const char *tag, const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    vprintf(format, arg);
    va_end(arg);
}
#define LOG_BUF_SIZE (1024)

void ats_log_level_set(ats_log_level_t log_level_set)
{
    RTK_LOG(ATS_LOG_INFO,APP_TAG,"log level: %s",log_type[log_level_set]);
    LOG_LEVEL = log_level_set;
}


void RTK_LOG(ats_log_level_t level,const char *tag,const char *format, ...)
{
    char buf[512];
    char mes_type[50];
    char* corlor_type = NULL;
    char log_str[1024];
    if((level > ATS_LOG_DEBUG) || (level < ATS_LOG_ERROR) || (LOG_LEVEL < level))
    {
        return;
    }
    switch(level)
    {
        case ATS_LOG_NONE:
            break;
        case ATS_LOG_ERROR:
            strcpy(mes_type,"ERROR:");
            corlor_type = LOG_COLOR_E;
            break;
        case ATS_LOG_WARN:
            strcpy(mes_type,"WARNING:");
            corlor_type = LOG_COLOR_W;
            break;
        case ATS_LOG_INFO:
            strcpy(mes_type,"INFORMATION:");        
            corlor_type = LOG_COLOR_I;
            break;
        case ATS_LOG_MES:
            strcpy(mes_type,"MESSAGE:");        
            corlor_type = LOG_COLOR_M;
            break;            
        case ATS_LOG_DEBUG:
            strcpy(mes_type,"DEBUG:");       
            corlor_type = LOG_COLOR_D; 
            break;
        case ATS_LOG_VERBOSE: 
            break;            
        default:
            sprintf(mes_type,"MESSAGE:");
            corlor_type = LOG_COLOR_E;
            break;
    }
    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, LOG_BUF_SIZE, format, arg);
    va_end(arg);
    sprintf(log_str,"%s%s %s: %s %s\r\n",corlor_type,mes_type,tag,buf,LOG_DEFAULT_COLOR);
    printf("%s",log_str);
}


uint32_t ats_log_timestamp()
{
    return 0;
}

void RTK_LOG_BUFF(ats_log_level_t log_level,const char *tag, const void *buffer, uint16_t buff_len)
{
    if ( buff_len == 0 ) return;
    char hex_buffer[3*BYTES_PER_LINE+1];
    const char *ptr_line;
    int bytes_cur_line;

    do {
        if ( buff_len > BYTES_PER_LINE ) {
            bytes_cur_line = BYTES_PER_LINE;
        } else {
            bytes_cur_line = buff_len;
        }
        ptr_line = buffer;
        for( int i = 0; i < bytes_cur_line; i ++ ) {
            sprintf(hex_buffer + 3*i, "%02x ", ptr_line[i] );
        }
        RTK_LOG(log_level, tag, "%s", hex_buffer);
        buffer += bytes_cur_line;
        buff_len -= bytes_cur_line;
    } while( buff_len );
}


void RTK_LOG_HEXDUMP(ats_log_level_t log_level,const char *tag, const void *buffer, uint16_t buff_len)
{

    if ( buff_len == 0 ) return;
    const char *ptr_line;
    char hd_buffer[10+3+BYTES_PER_LINE*3+3+BYTES_PER_LINE+1+1];
    char *ptr_hd;
    int bytes_cur_line;

    do {
        if ( buff_len > BYTES_PER_LINE ) {
            bytes_cur_line = BYTES_PER_LINE;
        } else {
            bytes_cur_line = buff_len;
        }
        ptr_line = buffer;
        ptr_hd = hd_buffer;
        ptr_hd += sprintf( ptr_hd, "%p ", buffer );
        for( int i = 0; i < BYTES_PER_LINE; i ++ ) {
            if ( (i&7)==0 ) {
                ptr_hd += sprintf( ptr_hd, "  " );
            }
            if ( i < bytes_cur_line ) {
                ptr_hd += sprintf( ptr_hd, " %02x", ptr_line[i] );
            } else {
                ptr_hd += sprintf( ptr_hd, "   " );
            }
        }
        ptr_hd += sprintf( ptr_hd, "  ||" );
        for( int i = 0; i < bytes_cur_line; i ++ ) {
            if (isprint((int)ptr_line[i]) ) {
                ptr_hd += sprintf( ptr_hd, "%c", ptr_line[i] );
            } else {
                ptr_hd += sprintf( ptr_hd, "." );
            }
        }
        ptr_hd += sprintf( ptr_hd, "||" );

        RTK_LOG(log_level, tag, "%s", hd_buffer);
        buffer += bytes_cur_line;
        buff_len -= bytes_cur_line;
    } while( buff_len );
}

