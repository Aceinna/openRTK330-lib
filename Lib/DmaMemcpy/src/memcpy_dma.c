/*******************************************************************************
* File Name          : dma_memcpy.h
* Author             : Daich
* Revision           : 1.0
* Date               : 30/09/2019
* Description        : zero-copy by dma
*
* HISTORY***********************************************************************
* 30/09/2019  |                                             | Daich
* Description: create
* 15/10/2019  |                                             | Daich
* Description: modify dma_memcpy_test
*******************************************************************************/

#include "stm32f4xx_hal.h"
#ifndef BAREMETAL_OS
    #include "osapi.h"
    #include "osresources.h"
#else
    #include "bare_osapi.h"
#endif

#include <stdio.h>
#include "memcpy_dma.h"

#define DMA_MEMCPY_IRQ        DMA2_Stream0_IRQHandler

DMA_HandleTypeDef hdma_memtomem;
const uint8_t src_const_buffer[24] = {
  0x31,0x31,0x31,0x31,
  0x31,0x31,0x31,0x31,
  0x31,0x31,0x31,0x31,
  0x31,0x31,0x31,0x31,
  0x31,0x31,0x31,0x31,
  0x31,0x31,0x31,0x31,
  };

static void delay_ms(uint32_t time)
{    
   uint32_t i=0;  
   while(time--)
   {
      i = 18000;  
      while(i--)
      {
        asm("nop");
      }
   }
}

int dma_memcpy_test(void)
{      
  dma_memcpy_init();
  printf("start test\n");
  while (1)
  {
    uint8_t dst_buffer[32];
    dma_memcpy((uint32_t)dst_buffer,(uint32_t)src_const_buffer,24);
    printf("test = %s\n",dst_buffer);
    delay_ms(2000);
  }
}


void dma_memcpy_init(void) 
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_memtomem.Instance = DMA2_Stream0;
    hdma_memtomem.Init.Channel = DMA_CHANNEL_1;
    hdma_memtomem.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem.Init.PeriphInc = DMA_PINC_ENABLE;
    hdma_memtomem.Init.MemInc = DMA_MINC_ENABLE;
    hdma_memtomem.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem.Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem.Init.Mode = DMA_NORMAL;
    hdma_memtomem.Init.Priority = DMA_PRIORITY_LOW ; 
    hdma_memtomem.Init.FIFOMode = DMA_FIFOMODE_DISABLE;  
    __HAL_DMA_ENABLE_IT(&hdma_memtomem,DMA_FLAG_TCIF1_5);
    HAL_DMA_Init(&hdma_memtomem);
}

int buffer_cmp(const uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer != *pBuffer1)
    {
      return -1;
    }
    
    pBuffer++;
    pBuffer1++;
  }
  return 1;  
}

void dma_memcpy(uint32_t dst_address, uint32_t src_address, uint32_t data_len)
{
    while(HAL_DMA_GetState(&hdma_memtomem) == HAL_DMA_STATE_BUSY)
    {
        ;
    }
    HAL_DMA_Start_IT(&hdma_memtomem, src_address, dst_address, data_len);
    //HAL_DMA_Start(&hdma_memtomem,SrcAddress,DstAddress,DataLength);
}

DMA_MEM_STATE_E get_dma_mem_state()
{
    return HAL_DMA_GetState(&hdma_memtomem);
}

void DMA_MEMCPY_IRQ(void)      //TODO:
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  HAL_DMA_IRQHandler(&hdma_memtomem);

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}
