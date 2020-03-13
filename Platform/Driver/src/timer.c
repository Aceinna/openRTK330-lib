/*******************************************************************************
* File Name          : timer.c
* Author             : Daich
* Revision           : 1.0
* Date               : 18/10/2019
* Description        : timer.c
*
* HISTORY***********************************************************************
* 18/10/2019  |                                             | Daich
* Description: create
* 18/10/2019  |                                             | Daich
* Description: packet dataAcqSem turn time_ms from float to int
* 25/10/2019  |                                             | Neil
* Description: modify double get_time_of_msec(),Otherwise, a data overflow will occur
*******************************************************************************/
#include "timer.h"
#include "constants.h"
#include "osapi.h"
#include "rtcm.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#define SENSOR_TIMER_IRQ                       TIM2_IRQHandler

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim_sensor;



/* TIM1 init function */
void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
    _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
    _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
    _Error_Handler(__FILE__, __LINE__);
    }

}
void MX_TIM_SENSOR_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim_sensor.Instance = TIM2;
    htim_sensor.Init.Prescaler = 1;
    htim_sensor.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_sensor.Init.Period = 44960;//224804;
    htim_sensor.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim_sensor) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim_sensor, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_sensor, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
        HAL_TIM_Base_Start_IT(&htim_sensor); //start tim2
    /* USER CODE END TIM2_Init 2 */
        HAL_TIM_IRQHandler(&htim_sensor);


}

volatile mcu_time_base_t g_MCU_time;


void release_sem(osSemaphoreId sem)
{
    osSemaphoreRelease(sem);
}
time_t get_time_of_msec()
{
    return (g_MCU_time.time * 1000 + g_MCU_time.msec);
}

volatile mcu_time_base_t* get_mcu_time()
{
    return &g_MCU_time;
}
volatile uint32_t usCnt = 0;
static void timer_isr_if(TIM_HandleTypeDef* timer)
{
    if(timer == &htim_sensor)
    {
        usCnt ++;
        {
            g_MCU_time.msec += 1;
            if(g_MCU_time.msec >= 1000)
            {
                g_MCU_time.msec = 0;
                g_MCU_time.time ++;
            }
            if(g_MCU_time.msec % 10 == 0)
            {
                release_sem(g_sem_imu_data_acq);
            }  
        }

  
    }
    HAL_TIM_IRQHandler(timer);    
}

void SENSOR_TIMER_IRQ(void)
{
    OSEnterISR(); 
    timer_isr_if(&htim_sensor);
    OSExitISR();
}