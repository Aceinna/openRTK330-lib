/*******************************************************************************
* File Name          : led.c
* Author             : Daich
* Revision           : 1.0
* Date               : 11/10/2019
* Description        : led drive source file
*
* HISTORY***********************************************************************
* 11/10/2019  |                                             | Daich
* Description: create
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include "stm32f4xx_hal.h"
#include "led.h"
#include "driver.h"


rtk_ret_e led_driver_install()
{
    GPIO_InitTypeDef  gpio_init_structure;

    LED_STATUS_CLK_ENABLE();
    LED_RTCM_CLK_ENABLE();
    LED_PPS_CLK_ENABLE();

  // Configure the GPIO_LED pins
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    // LED1
    gpio_init_structure.Pin   = LED_STATUS_PIN;
    HAL_GPIO_Init(LED_STATUS_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);

    // LED2
    gpio_init_structure.Pin   = LED_RTCM_PIN;
    HAL_GPIO_Init(LED_RTCM_PORT, &gpio_init_structure);
    // By default, turn off LED by setting a high level on corresponding GPIO
    HAL_GPIO_WritePin(LED_RTCM_PORT, LED_RTCM_PIN, GPIO_PIN_SET);

    // LED3
    gpio_init_structure.Pin   = LED_PPS_PIN;
    HAL_GPIO_Init(LED_PPS_PORT, &gpio_init_structure);
    // By default, turn off LED by setting a high level on corresponding GPIO
    HAL_GPIO_WritePin(LED_PPS_PORT, LED_PPS_PIN, GPIO_PIN_SET);
    return RTK_OK;
}


void LED1_On(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); 
}

void LED2_On(void)
{
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET); 
}

void LED3_On(void)
{
    HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET); 
}

void LED1_Off(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET); 
}

void LED2_Off(void)
{
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET); 
}

void LED3_Off(void)
{
    HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET); 
}

void LED1_Toggle(void)
{
    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
}

void LED2_Toggle(void)
{
    HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);
}

void LED3_Toggle(void)
{
    HAL_GPIO_TogglePin(LED3_PORT, LED3_PIN);
}