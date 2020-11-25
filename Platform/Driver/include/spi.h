/*******************************************************************************
* File Name          : spi.h
* Author             : yundong
* Revision           : 1.0
* Date               : 2020/03/23
* Description        : spi drive head file
*******************************************************************************/
#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
    extern "C" {
#endif

#define SPI_BUF_SIZE (434)


extern SPI_HandleTypeDef hspi5;
void MX_SPI5_Init(void);

extern uint8_t spi_buff[SPI_BUF_SIZE];
extern uint8_t spi_ready_flag;

#ifdef __cplusplus
}
#endif

#endif