/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "boardDefinition.h"
extern DMA_HandleTypeDef hdma_usart_user_rx;         
extern DMA_HandleTypeDef hdma_usart_user_tx;

extern DMA_HandleTypeDef hdma_uart_debug_rx;
extern DMA_HandleTypeDef hdma_uart_debug_tx;

extern DMA_HandleTypeDef hdma_usart_gps_rx;
extern DMA_HandleTypeDef hdma_usart_gps_tx;

extern DMA_HandleTypeDef hdma_usart_bt_rx;         
extern DMA_HandleTypeDef hdma_usart_bt_tx;

extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==DEBUG_UART)
    {
    /* USER CODE BEGIN UART5_MspInit 0 */

    /* USER CODE END UART5_MspInit 0 */
        /* Peripheral clock enable */
        DEBUG_UART_RCC_CLK_ENABLE();
        //DEBUG_UART_TX_RCC_CLK_ENABLE();
        DEBUG_UART_PORT_RCC_CLK_ENABLE();
        /**UART5 GPIO Configuration    
        PC12     ------> UART5_TX
        PD2     ------> UART5_RX 
        */
        GPIO_InitStruct.Pin = DEBUG_UART_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = DEBUG_UART_TX_AF;
        HAL_GPIO_Init(DEBUG_UART_TX_GPIO_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = DEBUG_UART_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = DEBUG_UART_RX_AF;
        HAL_GPIO_Init(DEBUG_UART_RX_GPIO_PORT, &GPIO_InitStruct);

        /* UART5 DMA Init */
        /* UART5_RX Init */
        hdma_uart_debug_rx.Instance = DEBUG_UART_DMA_RX_STREAM;
        hdma_uart_debug_rx.Init.Channel = DEBUG_UART_DMA_CHANNEL;
        hdma_uart_debug_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart_debug_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart_debug_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart_debug_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart_debug_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart_debug_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart_debug_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_uart_debug_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart_debug_rx) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart,hdmarx,hdma_uart_debug_rx);

        /* UART5_TX Init */
        hdma_uart_debug_tx.Instance = DEBUG_UART_DMA_TX_STREAM;
        hdma_uart_debug_tx.Init.Channel = DEBUG_UART_DMA_CHANNEL;
        hdma_uart_debug_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_uart_debug_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart_debug_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart_debug_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart_debug_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart_debug_tx.Init.Mode = DMA_NORMAL;
        hdma_uart_debug_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_uart_debug_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart_debug_tx) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart,hdmatx,hdma_uart_debug_tx);

        /* UART5 interrupt Init */
        HAL_NVIC_SetPriority(DEBUG_UART_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DEBUG_UART_IRQn);
        /* USER CODE BEGIN UART5_MspInit 1 */

        /* USER CODE END UART5_MspInit 1 */
    }
    else if(huart->Instance==USER_USART)
    {
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */
        /* Peripheral clock enable */
        USER_USART_RCC_CLK_ENABLE();
  
        USER_USART_PORT_RCC_CLK_ENABLE();
        /**USART1 GPIO Configuration    
        PA10     ------> USART1_RX
        PA9     ------> USART1_TX 
        */


        GPIO_InitStruct.Pin = USER_USART_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = USER_USART_TX_AF;
        HAL_GPIO_Init(USER_USART_TX_GPIO_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = USER_USART_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = USER_USART_RX_AF;
        HAL_GPIO_Init(USER_USART_RX_GPIO_PORT, &GPIO_InitStruct);

        /* USART1 DMA Init */
        /* USART1_RX Init */
        hdma_usart_user_rx.Instance = USER_USART_DMA_RX_STREAM;
        hdma_usart_user_rx.Init.Channel = USER_USART_DMA_CHANNEL;
        hdma_usart_user_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart_user_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart_user_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart_user_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart_user_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart_user_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart_user_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart_user_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart_user_rx) != HAL_OK)
        {
        Error_Handler();
        }

        __HAL_LINKDMA(huart,hdmarx,hdma_usart_user_rx);

        /* USART1_TX Init */
        hdma_usart_user_tx.Instance = USER_USART_DMA_TX_STREAM;
        hdma_usart_user_tx.Init.Channel = USER_USART_DMA_CHANNEL;
        hdma_usart_user_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart_user_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart_user_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart_user_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart_user_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart_user_tx.Init.Mode = DMA_NORMAL;
        hdma_usart_user_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart_user_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart_user_tx) != HAL_OK)
        {
        Error_Handler();
        }

        __HAL_LINKDMA(huart,hdmatx,hdma_usart_user_tx);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USER_USART_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USER_USART_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */

    /* USER CODE END USART1_MspInit 1 */
    }
    else if(huart->Instance==BT_USART)
    {
        /* USER CODE BEGIN USART2_MspInit 0 */

        /* USER CODE END USART2_MspInit 0 */
        /* Peripheral clock enable */
        BT_UART_RCC_CLK_ENABLE();
        BT_UART_PORT_RCC_CLK_ENABLE();
  
        /**USART2 GPIO Configuration    
        PD6     ------> USART2_RX
        PD5     ------> USART2_TX 
        */
        GPIO_InitStruct.Pin = BT_USART_RX_PIN|BT_USART_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = BT_USART_TX_RX_AF;
        HAL_GPIO_Init(BT_USART_TX_RX_GPIO_PORT, &GPIO_InitStruct);


        /* USART2 DMA Init */
        /* USART2_RX Init */
        hdma_usart_bt_rx.Instance = BT_USART_DMA_RX_STREAM;
        hdma_usart_bt_rx.Init.Channel = BT_USART_DMA_CHANNEL;
        hdma_usart_bt_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart_bt_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart_bt_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart_bt_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart_bt_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart_bt_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart_bt_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart_bt_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart_bt_rx) != HAL_OK)
        {
        _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart,hdmarx,hdma_usart_bt_rx);

        /* USART2_TX Init */
        hdma_usart_bt_tx.Instance = BT_USART_DMA_TX_STREAM;
        hdma_usart_bt_tx.Init.Channel = BT_USART_DMA_CHANNEL;
        hdma_usart_bt_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart_bt_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart_bt_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart_bt_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart_bt_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart_bt_tx.Init.Mode = DMA_NORMAL;
        hdma_usart_bt_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart_bt_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart_bt_tx) != HAL_OK)
        {
        _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart,hdmatx,hdma_usart_bt_tx);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(BT_USART_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(BT_USART_IRQn);
        //HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
    }
    else if(huart->Instance == GPS_USART)
    {
#if 1
    /* USER CODE BEGIN USART3_MspInit 0 */

    /* USER CODE END USART3_MspInit 0 */
        /* Peripheral clock enable */
        GPS_UART_RCC_CLK_ENABLE();
        GPS_UART_PORT_RCC_CLK_ENABLE();
        /**USART3 GPIO Configuration    
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX 
        */
        GPIO_InitStruct.Pin = GPS_USART_TX_PIN|GPS_USART_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPS_USART_TX_RX_AF;
        HAL_GPIO_Init(GPS_USART_TX_RX_GPIO_PORT, &GPIO_InitStruct);

        /* USART3 DMA Init */
        /* USART3_RX Init */
        hdma_usart_gps_rx.Instance = GPS_USART_DMA_RX_STREAM;
        hdma_usart_gps_rx.Init.Channel = GPS_USART_DMA_CHANNEL;
        hdma_usart_gps_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart_gps_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart_gps_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart_gps_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart_gps_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart_gps_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart_gps_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart_gps_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart_gps_rx) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }


        __HAL_LINKDMA(huart,hdmarx,hdma_usart_gps_rx);

        /* USART3_TX Init */
        hdma_usart_gps_tx.Instance = GPS_USART_DMA_TX_STREAM;
        hdma_usart_gps_tx.Init.Channel = GPS_USART_DMA_CHANNEL;
        hdma_usart_gps_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart_gps_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart_gps_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart_gps_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart_gps_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart_gps_tx.Init.Mode = DMA_NORMAL;
        hdma_usart_gps_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart_gps_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart_gps_tx) != HAL_OK)
        {
        _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart,hdmatx,hdma_usart_gps_tx);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(GPS_USART_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(GPS_USART_IRQn);
        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
#endif
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();
  
    /**UART5 GPIO Configuration    
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* UART5 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA10     ------> USART1_RX
    PA9     ------> USART1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10|GPIO_PIN_9);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PD6     ------> USART2_RX
    PD5     ------> USART2_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6|GPIO_PIN_5);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (hcan->Instance == CAN1)
    {
        USER_CAN_CLK_ENABLE();
        USER_CAN_PORT_RCC_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_Initure;
        GPIO_Initure.Pin = USER_CAN_AB_PIN;
        GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(USER_CAN_AB_PORT, &GPIO_Initure);

        GPIO_Initure.Pin = USER_CAN_120R_CTL_PIN;
        HAL_GPIO_Init(USER_CAN_120R_CTL_PORT, &GPIO_Initure);

        HAL_GPIO_WritePin(USER_CAN_120R_CTL_PORT, USER_CAN_120R_CTL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(USER_CAN_AB_PORT, USER_CAN_AB_PIN, GPIO_PIN_RESET);

        GPIO_InitStruct.Pin = USER_CAN_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = USER_CAN_TX_AF;
        HAL_GPIO_Init(USER_CAN_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = USER_CAN_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = USER_CAN_RX_AF;
        HAL_GPIO_Init(USER_CAN_RX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USER_CAN_RX_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USER_CAN_RX_IRQn);

        HAL_NVIC_SetPriority(USER_CAN_TX_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USER_CAN_TX_IRQn);
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        USER_CAN_FORCE_RESET();
        USER_CAN_RELEASE_RESET();

        HAL_GPIO_DeInit(USER_CAN_AB_PORT, USER_CAN_AB_PIN);
        HAL_GPIO_DeInit(USER_CAN_120R_CTL_PORT, USER_CAN_120R_CTL_PIN);
        HAL_GPIO_DeInit(USER_CAN_TX_PORT, USER_CAN_TX_PIN);
        HAL_GPIO_DeInit(USER_CAN_RX_PORT, USER_CAN_RX_PIN);

        HAL_NVIC_DisableIRQ(USER_CAN_RX_IRQn);
        HAL_NVIC_DisableIRQ(USER_CAN_TX_IRQn);
    }
}

void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
    __HAL_RCC_RNG_CLK_ENABLE();
}

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
