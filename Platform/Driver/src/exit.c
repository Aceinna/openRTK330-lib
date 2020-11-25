/*******************************************************************************
* File Name          : exit.c
* Author             : Daich
* Revision           : 1.0
* Date               : 18/10/2019
* Description        : exit.c
*
* HISTORY***********************************************************************
* 18/10/2019  |                                             | Daich
* Description: create
*******************************************************************************/
#include "stm32f4xx_hal.h"
#include "boardDefinition.h"
#include "exit.h"
#include "timer.h"
#include "gnss_data_api.h"
#include "led.h"
#include "bsp.h"
#include "uart.h"
#include "osapi.h"
#include "string.h"
#include "app_version.h"
#include "ins_interface_API.h"

volatile mcu_time_base_t g_obs_rcv_time;


__weak uint8_t get_gnss_signal_flag()
{
    return 0;
}
__weak time_t get_obs_time()
{
    return 0;
}


void pps_exit_init(void)
{
    // pps
    GPIO_InitTypeDef GPIO_InitStruct;    
    GPIO_InitStruct.Pin = ST_PPS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ST_PPS_PORT, &GPIO_InitStruct);
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(ST_PPS_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ST_PPS_IRQn);
}


extern volatile mcu_time_base_t g_MCU_time;
uint8_t g_pps_flag = 0;
extern uint32_t usCnt;
extern TIM_HandleTypeDef htim_sensor;
void ST_PPS_IRQ(void)
{
    OSEnterISR();   
    LED_PPS_TOOGLE();

    g_pps_flag = 1;
    uint8_t PPSstate = HAL_GPIO_ReadPin(ST_PPS_PORT,ST_PPS_PIN);
    if (PPSstate == 0)
    {
        if (g_MCU_time.msec < 500)
        {
            g_MCU_time.msec = 499;
            htim_sensor.Instance->CNT = 44960;
        }
        else if(g_MCU_time.msec > 500)
        {
            g_MCU_time.msec = 500;
            htim_sensor.Instance->CNT = 1;
        }

        if (strstr((char *)APP_VERSION_STRING, "RAWDATA"))
        {
            if(get_gnss_signal_flag() && (g_MCU_time.msec - g_obs_rcv_time.msec) >= 0 && (g_MCU_time.time == g_obs_rcv_time.time))
                g_MCU_time.time = get_obs_time(); 
        }
   
    
    }
    HAL_GPIO_EXTI_IRQHandler(ST_PPS_PIN);
    OSExitISR();    
}


void PULSE_IRQ()
{
    add_wheel_tick_count();
    set_wheel_tick_fwd(HAL_GPIO_ReadPin(FWD_PORT,FWD_PIN));
    HAL_GPIO_EXTI_IRQHandler(PULSE_PIN);
}

