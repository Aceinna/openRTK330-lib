/*******************************************************************************
* File Name          : timer.h
* Author             : Daich
* Revision           : 1.0
* Date               : 10/10/2019
* Description        : timer.h
*
* HISTORY***********************************************************************
* 18/10/2019  |                                             | Daich
* Description: create
* 18/10/2019  |                                             | Daich
* Description: //#define UART_BLOCK to save memory
* 25/10/2019  |                                             | Neil
* Description: modify double get_time_of_msec(),Otherwise, a data overflow will occur 
*******************************************************************************/
#ifndef _TIMER_H_
#define _TIMER_H_
//#pragma once
#include <time.h>
#include "cmsis_os.h"
typedef struct mcu_time_base_t_
{
    time_t time;
    time_t msec;
} mcu_time_base_t;

extern volatile mcu_time_base_t g_MCU_time;

void MX_TIM1_Init(void);
void MX_TIM_SENSOR_Init(void);
time_t get_time_of_msec();
volatile mcu_time_base_t *get_mcu_time();

void release_sem(osSemaphoreId sem);
double get_gnss_time();
#endif
