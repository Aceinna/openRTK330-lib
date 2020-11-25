/** ***************************************************************************
 * @file   configureGPIO.c, Initialize the GPIO pins to output for the OpenRTK330 
 *        board
 * @date   September, 2020
 * @brief  Copyright (c) 2020,
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/
#include "configureGPIO.h"
#include "constants.h"
#include "stm32f4xx_hal_conf.h"
#include "bsp.h"

void ResetSTIForNormalMode()
{
    HAL_GPIO_WritePin(ST_RESET_PORT,ST_RESET_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_BOOT_PORT,ST_BOOT_PIN,GPIO_PIN_RESET);
    DelayMs(10);
    HAL_GPIO_WritePin(ST_RESET_PORT,ST_RESET_PIN,GPIO_PIN_SET);
    DelayMs(10);
}

void SetBufCtr(uint8_t state)
{
    if(state)
      HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_SET);
    else 
      HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_RESET);
}

void ResetSTIForBootMode()
{
    // SetBufCtr(GPIO_PIN_SET);
    SetBufCtr(GPIO_PIN_RESET);
    // GPIOC->BSRRH = ST_RESET_PIN; // Set nRST low
    HAL_GPIO_WritePin(ST_RESET_PORT,ST_RESET_PIN,GPIO_PIN_RESET);
    DelayMs(10);
    HAL_GPIO_WritePin(ST_BOOT_PORT ,ST_BOOT_PIN ,GPIO_PIN_SET);
    // GPIOB->BSRRL = ST_BOOT_PIN;  // Set BOOT pin high
    DelayMs(10);
    // GPIOC->BSRRL = ST_RESET_PIN; // Set nRST high
    HAL_GPIO_WritePin(ST_RESET_PORT,ST_RESET_PIN,GPIO_PIN_SET);
    DelayMs(10);
}

