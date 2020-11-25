/*******************************************************************************
* File Name          : uart.c
* Author             : Daich
* Revision           : 1.0
* Date               : 10/10/2019
* Description        : uart drive source file
*
* HISTORY***********************************************************************
* 10/10/2019  |                                             | Daich
* Description: create
* HISTORY***********************************************************************
* 17/10/2019  |                                             | Daich
* Description: add update_fifo_in function 
               use malloc to apply uart rx buff
* 25/10/2019  |                                             | Daich
* Description: modify uart_write_bytes todo
* 26/10/2019  |                                             | Daich
* Description: clear uart gState
* 16/12/2019  |                                             | Daich
* Description: uart idle interrupt release sem
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include "stm32f4xx_hal.h"
#include "uart.h"
#include "driver.h"
#include "utils.h"
#include "boardDefinition.h"
#include "FreeRTOS.h"
#include "osapi.h"


// osSemaphoreDef(RX_ACQ_SEM);
// osSemaphoreDef(TX_ACQ_SEM);


static uint8_t uart_user_buff[IMU_BUFF_SIZE];
static uint8_t uart_bt_buff[GPS_BUFF_SIZE];
static uint8_t uart_gps_buff[GPS_BUFF_SIZE];
static uint8_t uart_debug_buff[GPS_BUFF_SIZE];


DMA_HandleTypeDef hdma_usart_user_rx;
DMA_HandleTypeDef hdma_usart_user_tx;
DMA_HandleTypeDef hdma_uart_debug_rx;
DMA_HandleTypeDef hdma_uart_debug_tx;
DMA_HandleTypeDef hdma_usart_gps_rx;
DMA_HandleTypeDef hdma_usart_gps_tx;
DMA_HandleTypeDef hdma_usart_bt_rx;
DMA_HandleTypeDef hdma_usart_bt_tx;

fifo_type uart_gps_rx_fifo;
fifo_type uart_debug_rx_fifo;
fifo_type uart_bt_rx_fifo;
fifo_type uart_user_rx_fifo;

#ifdef USER_UART_DMA_FIFO
static uint8_t user_uart_dma_tx_buff[DMA_TX_FIFO_BUF_SIZE];
uart_tx_dma_fifo_s user_uart_dma_tx_fifo;

#endif
#ifdef DEBUG_UART_DMA_FIFO
static uint8_t debug_uart_dma_tx_buff[DEBUG_DMA_TX_FIFO_BUF_SIZE];
uart_tx_dma_fifo_s debug_uart_dma_tx_fifo;
#endif

static const struct uart_config_t uart_config[UART_MAX] = {
    {
        .uart_base_addr = (uint32_t)UART_USER_BASE, 
        .rec_buff_size = IMU_BUFF_SIZE,
		.rec_buff = uart_user_buff,
        .hdma_usart_rx = &hdma_usart_user_rx,
        .hdma_usart_tx = &hdma_usart_user_tx,
    }, 
    {
        .uart_base_addr = (uint32_t)UART_BT_BASE, 
        .rec_buff_size = GPS_BUFF_SIZE,
		.rec_buff = uart_bt_buff,
        .hdma_usart_rx = &hdma_usart_bt_rx,
        .hdma_usart_tx = &hdma_usart_bt_tx,
    }, 
    {
        .uart_base_addr = (uint32_t)UART_GPS_BASE, 
        .rec_buff_size = GPS_BUFF_SIZE,
		.rec_buff =	uart_gps_buff,
        .hdma_usart_rx = &hdma_usart_gps_rx,
        .hdma_usart_tx = &hdma_usart_gps_tx,
    }, 
    {
        .uart_base_addr = (uint32_t)UART_DEBUG_BASE, 
        .rec_buff_size = GPS_BUFF_SIZE,
		.rec_buff = uart_debug_buff,
        .hdma_usart_rx = &hdma_uart_debug_rx,
        .hdma_usart_tx = &hdma_uart_debug_tx,
    },         
};

uart_obj_t *p_uart_obj[UART_MAX] = {0};
uart_obj_t uart_obj[UART_MAX];


static int uart_receive_dma(uart_port_e uart_num)
{
    HAL_UART_Receive_DMA(p_uart_obj[uart_num]->huart, p_uart_obj[uart_num]->uart_rx_fifo->buffer, p_uart_obj[uart_num]->uart_rx_fifo->size);
    return RTK_OK;
}

static int uart_rx_dma_enable(uart_port_e uart_num)
{
    __HAL_DMA_ENABLE(p_uart_obj[uart_num]->hdma_usart_rx);
    return RTK_OK;
}

static int uart_dma_stop(uart_port_e uart_num)
{
    HAL_UART_DMAStop(p_uart_obj[uart_num]->huart);
    return RTK_OK;
}

static int uart_dma_enanle_it(uart_port_e uart_num,uint32_t it_type)
{
    __HAL_UART_ENABLE_IT(p_uart_obj[uart_num]->huart,it_type);
    return RTK_OK;
}
    

int uart_read_bytes(uart_port_e uart_num, uint8_t* buf, uint32_t len, TickType_t ticks_to_wait)
{
	uint16_t lenght;
	uint16_t in = p_uart_obj[uart_num]->uart_rx_fifo->in;
	uint16_t i;

#ifdef UART_BLOCK    
    if(ticks_to_wait > 0)
    {
        if(osSemaphoreWait(p_uart_obj[uart_num]->rx_sem,(portTickType)ticks_to_wait) != osOK)
        {
            //printf("time out\n");
            return 0;
        }
    }
#endif
	lenght = (in + p_uart_obj[uart_num]->uart_rx_fifo->size - p_uart_obj[uart_num]->uart_rx_fifo->out)%p_uart_obj[uart_num]->uart_rx_fifo->size;
	if(lenght > len)
		lenght = len;
	for(i = 0; i < lenght; i++)     //TODO: memcpy?
	{
		buf[i] = p_uart_obj[uart_num]->uart_rx_fifo->buffer[(p_uart_obj[uart_num]->uart_rx_fifo->out + i)%p_uart_obj[uart_num]->uart_rx_fifo->size];
	}
	p_uart_obj[uart_num]->uart_rx_fifo->out = (p_uart_obj[uart_num]->uart_rx_fifo->out + lenght)%p_uart_obj[uart_num]->uart_rx_fifo->size;
    //printf("len = %d\n",lenght);
    return lenght;

}

static uint8_t data_to_write[DMA_TX_FIFO_BUF_SIZE];
static uint8_t data_to_debug[DEBUG_DMA_TX_FIFO_BUF_SIZE];
int uart_write_bytes(uart_port_e uart_num, const char* src, size_t size, bool is_wait)	//TODO:
{
#ifdef USER_UART_DMA_FIFO
    if(uart_num == UART_USER)
    {
        fifo_push(&user_uart_dma_tx_fifo.uart_tx_fifo,(uint8_t*)src,size);
        user_uart_dma_tx_fifo.frame_num+= 1;
        user_uart_dma_tx_fifo.data_total_num+= size;
        if(user_uart_dma_tx_fifo.data_total_num > DMA_TX_FIFO_BUF_SIZE)
        {
        }
        if(p_uart_obj[UART_USER]->huart->gState == HAL_UART_STATE_READY)
        {
            int data_len = fifo_get(&user_uart_dma_tx_fifo.uart_tx_fifo,data_to_write,DMA_TX_FIFO_BUF_SIZE);
            user_uart_dma_tx_fifo.frame_num = 0;
            user_uart_dma_tx_fifo.data_total_num = 0;
            HAL_UART_Transmit_DMA(p_uart_obj[UART_USER]->huart, data_to_write, data_len);
        }
        return RTK_OK;
    }
#ifdef DEBUG_UART_DMA_FIFO
    else if(uart_num == UART_DEBUG)
    {
        fifo_push(&debug_uart_dma_tx_fifo.uart_tx_fifo,(uint8_t*)src,size);
        debug_uart_dma_tx_fifo.frame_num+= 1;
        debug_uart_dma_tx_fifo.data_total_num+= size;
        if(debug_uart_dma_tx_fifo.data_total_num > DEBUG_DMA_TX_FIFO_BUF_SIZE)
        {
        }
        if(p_uart_obj[UART_DEBUG]->huart->gState == HAL_UART_STATE_READY)
        {
            int data_len = fifo_get(&debug_uart_dma_tx_fifo.uart_tx_fifo,data_to_debug,DEBUG_DMA_TX_FIFO_BUF_SIZE);
            debug_uart_dma_tx_fifo.frame_num = 0;
            debug_uart_dma_tx_fifo.data_total_num = 0;
            HAL_UART_Transmit_DMA(p_uart_obj[UART_DEBUG]->huart, data_to_debug, data_len);
        }
        return RTK_OK;
    }
    else
#endif
#endif
    {
        while(HAL_UART_Transmit_DMA(p_uart_obj[uart_num]->huart, (uint8_t *)src, size) == HAL_BUSY)
        {
            if(is_wait == 0)
                return 0;
            osDelay(1);
        }
    }
    return RTK_OK;
}


int uart_driver_install(uart_port_e uart_num, fifo_type* uart_rx_fifo,UART_HandleTypeDef* huart,int baudrate)
{
    rtk_ret_e ret;

    if(p_uart_obj[uart_num] == NULL) 
	{
        p_uart_obj[uart_num] = &uart_obj[uart_num];
        if(p_uart_obj[uart_num] == NULL) {
            return RTK_FAIL;
        }
        osSemaphoreDef(UART_IDLE_SEM);
        if((uart_num == UART_BT) || (uart_num == UART_DEBUG))
        {
            p_uart_obj[uart_num]->uart_idle_sem = osSemaphoreCreate(osSemaphore(UART_IDLE_SEM), 1);
        }        
        p_uart_obj[uart_num]->hdma_usart_rx = uart_config[uart_num].hdma_usart_rx;
        p_uart_obj[uart_num]->hdma_usart_tx = uart_config[uart_num].hdma_usart_tx;
        p_uart_obj[uart_num]->uart_num = uart_num;
        p_uart_obj[uart_num]->baudrate = baudrate;

		//uint8_t* uart_x_buff = (uint8_t*)malloc(sizeof(uint8_t) * uart_config[uart_num].rec_buff_size);
		
        fifo_init(uart_rx_fifo, uart_config[uart_num].rec_buff, uart_config[uart_num].rec_buff_size);       
        p_uart_obj[uart_num]->uart_rx_fifo = uart_rx_fifo;     
		p_uart_obj[uart_num]->huart = huart;

        //osSemaphoreDef(TX_ACQ_SEM);
        //p_uart_obj[uart_num]->tx_sem = osSemaphoreCreate(osSemaphore(TX_ACQ_SEM), 1);
#ifdef UART_BLOCK
        osSemaphoreDef(RX_ACQ_SEM);
        p_uart_obj[uart_num]->rx_sem = osSemaphoreCreate(osSemaphore(RX_ACQ_SEM), 1);     //TODO:
#endif
		p_uart_obj[uart_num]->uart_rx_fifo = uart_rx_fifo;
    } 
	else 
	{
        return RTK_FAIL;
    }
    p_uart_obj[uart_num]->huart->Instance = (USART_TypeDef*)(uart_config[uart_num].uart_base_addr);
    p_uart_obj[uart_num]->huart->Init.BaudRate = baudrate;
    p_uart_obj[uart_num]->huart->Init.WordLength = UART_WORDLENGTH_8B;
    p_uart_obj[uart_num]->huart->Init.StopBits = UART_STOPBITS_1;
    p_uart_obj[uart_num]->huart->Init.Parity = UART_PARITY_NONE;
    p_uart_obj[uart_num]->huart->Init.Mode = UART_MODE_TX_RX;
    p_uart_obj[uart_num]->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    p_uart_obj[uart_num]->huart->Init.OverSampling = UART_OVERSAMPLING_16;
	if(uart_num == UART_USER)
	{
	    if (HAL_UART_DeInit(p_uart_obj[uart_num]->huart) != HAL_OK)
	    {
	        _Error_Handler(__FILE__, __LINE__);
	    }
	}
    if (HAL_UART_Init(p_uart_obj[uart_num]->huart) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    uart_receive_dma(p_uart_obj[uart_num]->uart_num);       //TODO:
    uart_rx_dma_enable(p_uart_obj[uart_num]->uart_num);
    uart_dma_enanle_it(p_uart_obj[uart_num]->uart_num,UART_IT_IDLE);

    p_uart_obj[uart_num]->init_flag = 1;

#ifdef USER_UART_DMA_FIFO
    if(uart_num == UART_USER)           // TODO:
    {
        user_uart_dma_tx_fifo.data_total_num = 0;
        user_uart_dma_tx_fifo.is_dma_busy = 0;
        user_uart_dma_tx_fifo.frame_num = 0;
        user_uart_dma_tx_fifo.is_data_available = 0;
        fifo_init(&user_uart_dma_tx_fifo.uart_tx_fifo, user_uart_dma_tx_buff, DMA_TX_FIFO_BUF_SIZE); 
    }
#endif
#ifdef DEBUG_UART_DMA_FIFO
    if(uart_num == UART_DEBUG)           // TODO:
    {
        debug_uart_dma_tx_fifo.data_total_num = 0;
        debug_uart_dma_tx_fifo.is_dma_busy = 0;
        debug_uart_dma_tx_fifo.frame_num = 0;
        debug_uart_dma_tx_fifo.is_data_available = 0;
        fifo_init(&debug_uart_dma_tx_fifo.uart_tx_fifo, debug_uart_dma_tx_buff, DEBUG_DMA_TX_FIFO_BUF_SIZE); 
    }
#endif
    return ret;
}

rtk_ret_e uart_driver_delete(uart_port_e uart_num)
{
	return RTK_OK;
}

void update_fifo_in(uart_port_e uart_num)
{
    p_uart_obj[uart_num]->uart_rx_fifo->in = p_uart_obj[uart_num]->uart_rx_fifo->size - __HAL_DMA_GET_COUNTER(p_uart_obj[uart_num]->hdma_usart_rx);
}


static void uart_isr_if(uart_port_e uart_num)
{
    if (RESET != __HAL_UART_GET_FLAG(p_uart_obj[uart_num]->huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(p_uart_obj[uart_num]->huart);
        if(uart_num != UART_GPS)
        {
            p_uart_obj[uart_num]->uart_rx_fifo->in = p_uart_obj[uart_num]->uart_rx_fifo->size - __HAL_DMA_GET_COUNTER(p_uart_obj[uart_num]->hdma_usart_rx);
        }
    }
    if (RESET != __HAL_UART_GET_FLAG(p_uart_obj[uart_num]->huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(p_uart_obj[uart_num]->huart);
        // uart_dma_stop(uart_num);
        // uart_receive_dma(uart_num);       //TODO:
        // uart_rx_dma_enable(uart_num);
        // uart_dma_enanle_it(uart_num,UART_IT_IDLE);
    }
    if (RESET != __HAL_UART_GET_FLAG(p_uart_obj[uart_num]->huart, UART_FLAG_NE))
    {               
        __HAL_UART_CLEAR_NEFLAG(p_uart_obj[uart_num]->huart);
    }
#if 1
    if (RESET != __HAL_UART_GET_FLAG(p_uart_obj[uart_num]->huart, UART_FLAG_ORE))
    {               
        __HAL_UART_CLEAR_OREFLAG(p_uart_obj[uart_num]->huart);
    }
#endif

    HAL_UART_IRQHandler(p_uart_obj[uart_num]->huart);
    if((uart_num == UART_BT) || (uart_num == UART_DEBUG))
    {
        if (RESET == __HAL_UART_GET_FLAG(p_uart_obj[uart_num]->huart, UART_FLAG_IDLE))
        {
            osSemaphoreRelease(p_uart_obj[uart_num]->uart_idle_sem);
        }
    }
#ifdef UART_BLOCK
    osSemaphoreRelease(p_uart_obj[uart_num]->rx_sem);
#endif
}

 void DEBUG_UART_IRQ(void)
{
    OSEnterISR();    
    uart_isr_if(UART_DEBUG);
#if SHELL_ENABLE == true
    osSemaphoreRelease(debug_uart_sem);
#endif
    OSExitISR();
}

void USER_USART_IRQ(void)
{
    OSEnterISR();    
    uart_isr_if(UART_USER);
    OSExitISR();
}


void BT_USART_IRQ(void)
{
    OSEnterISR();    
    uart_isr_if(UART_BT);
#if SHELL_ENABLE == true
    osSemaphoreRelease(bt_uart_sem);
#endif
    if (RESET == __HAL_UART_GET_FLAG(p_uart_obj[UART_BT]->huart, UART_FLAG_IDLE))
    {
        //osSemaphoreRelease(bt_uart_sem);
    }
    OSExitISR();
}



void GPS_USART_IRQ(void)
{
    OSEnterISR();    
    uart_isr_if(UART_GPS);
    OSExitISR();
}

static void uart_dma_tx_isr_if(uart_port_e uart_num)
{
	if(__HAL_DMA_GET_COUNTER(p_uart_obj[uart_num]->hdma_usart_tx) == 0)
    {
        p_uart_obj[uart_num]->huart->gState = HAL_UART_STATE_READY;
    }
    HAL_DMA_IRQHandler(p_uart_obj[uart_num]->hdma_usart_tx);

    if(p_uart_obj[UART_USER]->huart->gState == HAL_UART_STATE_READY)
    {
        int data_len = fifo_get(&user_uart_dma_tx_fifo.uart_tx_fifo,data_to_write,DMA_TX_FIFO_BUF_SIZE);
        user_uart_dma_tx_fifo.frame_num = 0;
        user_uart_dma_tx_fifo.data_total_num = 0;
        HAL_UART_Transmit_DMA(p_uart_obj[UART_USER]->huart, data_to_write, data_len);
    }
}

void USER_USART_DMA_TX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_tx_isr_if(UART_USER);
    OSExitISR();
}


void BT_USART_DMA_TX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_tx_isr_if(UART_BT);
    OSExitISR();
}

void GPS_USART_DMA_TX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_tx_isr_if(UART_GPS);
    OSExitISR();
}

void DEBUG_UART_DMA_TX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_tx_isr_if(UART_DEBUG);
    OSExitISR();
}



static void uart_dma_rx_isr_if(uart_port_e uart_num)
{
    HAL_DMA_IRQHandler(p_uart_obj[uart_num]->hdma_usart_rx);
}

void USER_USART_DMA_RX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_rx_isr_if(UART_USER);
    OSExitISR();
}

void BT_USART_DMA_RX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_rx_isr_if(UART_BT);
    OSExitISR();
}

void GPS_USART_DMA_RX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_rx_isr_if(UART_GPS);
    OSExitISR();
}

void DEBUG_UART_DMA_RX_IRQHandler(void)
{
    OSEnterISR();    
    uart_dma_rx_isr_if(UART_DEBUG);
    OSExitISR();
}

rtk_ret_e uart_sem_wait(uart_port_e uart_num,uint32_t millisec)
{
    if((uart_num == UART_BT) || (uart_num == UART_DEBUG))
    {
        if(osSemaphoreWait(p_uart_obj[uart_num]->uart_idle_sem,millisec) == osOK)
        {
            return RTK_SEM_OK;
        }
    }
    return -1;
}