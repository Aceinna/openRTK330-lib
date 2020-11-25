/*******************************************************************************
* File Name          : uart.c
* Author             : yundong
* Revision           : 1.0
* Date               : 2019/03/23
* Description        : spi drive source file
*
********************************************************************************/
#include "stm32f4xx_hal.h"
#include "boardDefinition.h"
#include "uart.h"
#include "spi.h"
#include "string.h"
#include "user_config.h"
#include "exit.h"

SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi5_rx;
DMA_HandleTypeDef hdma_spi5_tx;

uint8_t spi_rx[SPI_BUF_SIZE] = {0};
uint8_t spi_tx[SPI_BUF_SIZE] = {0};
uint8_t spi_ready_flag = 0;
void MX_SPI5_Init(void)
{

  // for (int i = 0; i < SPI_BUF_SIZE; i++)
  //   spi_tx[i] = i+1;
  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = USER_SPI;
  hspi5.Init.Mode = SPI_MODE_SLAVE;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  // if (HAL_SPI_TransmitReceive_DMA(&hspi5, spi_tx, spi_rx, SPI_BUF_SIZE) != HAL_OK)
  // {
  //   /* Transfer error in transmission process */
  //   Error_Handler();
  // }
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hspi->Instance == USER_SPI)
  {
    /* USER CODE BEGIN SPI5_MspInit 0 */

    /* USER CODE END SPI5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI5_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();

    /*Configure GPIO pin : PF6 */
    GPIO_InitStruct.Pin = USER_SPI_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_SPI_NSS_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(USER_SPI_NSS_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USER_SPI_NSS_RX_IRQn);

    /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO 
    */
    GPIO_InitStruct.Pin = USER_SPI_MOSI_PIN | USER_SPI_MISO_PIN | USER_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(USER_SPI_MOSI_PORT, &GPIO_InitStruct);

    /* SPI5 DMA Init */
    /* SPI5_RX Init */
    hdma_spi5_rx.Instance = DMA2_Stream3;
    hdma_spi5_rx.Init.Channel = DMA_CHANNEL_2;
    hdma_spi5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi5_rx.Init.Mode = DMA_NORMAL;
    hdma_spi5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmarx, hdma_spi5_rx);

    /* SPI5_TX Init */
    hdma_spi5_tx.Instance = DMA2_Stream4;
    hdma_spi5_tx.Init.Channel = DMA_CHANNEL_2;
    hdma_spi5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi5_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi5_tx.Init.Mode = DMA_NORMAL;
    hdma_spi5_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi5_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi, hdmatx, hdma_spi5_tx);

    /* USER CODE BEGIN SPI5_MspInit 1 */

    /* USER CODE END SPI5_MspInit 1 */
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{

  if (hspi->Instance == USER_SPI)
  {
    /* USER CODE BEGIN SPI5_MspDeInit 0 */

    /* USER CODE END SPI5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI5_CLK_DISABLE();

    /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO 
    */
    HAL_GPIO_DeInit(USER_SPI_MOSI_PORT, USER_SPI_MOSI_PIN | USER_SPI_MISO_PIN | USER_SPI_SCK_PIN);

    /* SPI5 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);
    /* USER CODE BEGIN SPI5_MspDeInit 1 */

    /* USER CODE END SPI5_MspDeInit 1 */
  }
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi5_rx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi5_tx);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
}
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */

void SPI_IRQ()
{
    memcpy(spi_tx,spi_buff,SPI_BUF_SIZE);
    
    if (spi_ready_flag)
    {
        spi_ready_flag = 0;
        HAL_SPI_TransmitReceive_DMA(&hspi5, spi_tx, spi_rx, SPI_BUF_SIZE);
    }
    HAL_GPIO_EXTI_IRQHandler(USER_SPI_NSS_PIN);
}

void SPI_PULSE_IRQ()
{
    int wheeltick_pin_mode = get_wheeltick_pin_mode();
    if(wheeltick_pin_mode == 0)
    {
        PULSE_IRQ();
    }   
    else if(wheeltick_pin_mode == 1)
    {
        SPI_IRQ();
    } 
}

