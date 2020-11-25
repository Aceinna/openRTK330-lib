/*******************************************************************************
* File Name          : can.c
* Author             : ymj
* Revision           : 1.0
* Date               : 3/31/2020
* Description        : can.c
* HISTORY***********************************************************************
* 3/31/2020  |                                             | ymj
* Description: create
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "can.h"
#include "sae_j1939.h"
#include "boardDefinition.h"
#include "osapi.h"
#include "user_message_can.h"
#include "user_config.h"
#include "car_data.h"

CAN_HandleTypeDef canHandle;
CAN_FilterTypeDef canFilter;

void (*gCANTxCompleteCallback)(void) = NULL;    // callback function pointer of CAN transmit
void (*gCANRxCompleteCallback)(void) = NULL;    // callback function pointer of CAN receive

uint32_t canRxIntCounter = 0;                   // counter of CAN receiver
uint32_t canStartDetectRxIntCounter = 0;        // counter of CAN receiver for auto detection

/** ***************************************************************************
 * @name _can_set_baudrate() Set up bit timing
 * @brief Provides a function to set bit timing according to the expected baud rate
 *
 * @param [in] CAN_InitTypeDef CAN init structure definition
 *             br, baud rate
 * @retval N/A
 ******************************************************************************/
static void _can_set_baudrate(_ECU_BAUD_RATE br, CAN_InitTypeDef *pCan_initTypeDef)
{
    switch (br)
    {
    case _ECU_125K:
        pCan_initTypeDef->Prescaler = 40;
        pCan_initTypeDef->SyncJumpWidth = CAN_SJW_1TQ;
        pCan_initTypeDef->TimeSeg1 = CAN_BS1_6TQ;
        pCan_initTypeDef->TimeSeg2 = CAN_BS2_2TQ;
        break;
    case _ECU_250K:
        pCan_initTypeDef->Prescaler = 20;
        pCan_initTypeDef->SyncJumpWidth = CAN_SJW_1TQ;
        pCan_initTypeDef->TimeSeg1 = CAN_BS1_6TQ;
        pCan_initTypeDef->TimeSeg2 = CAN_BS2_2TQ;
        break;
    case _ECU_500K:
        pCan_initTypeDef->Prescaler = 10;
        pCan_initTypeDef->SyncJumpWidth = CAN_SJW_1TQ;
        pCan_initTypeDef->TimeSeg1 = CAN_BS1_6TQ;
        pCan_initTypeDef->TimeSeg2 = CAN_BS2_2TQ;
        break;
    case _ECU_1000K:
        pCan_initTypeDef->Prescaler = 5;
        pCan_initTypeDef->SyncJumpWidth = CAN_SJW_1TQ;
        pCan_initTypeDef->TimeSeg1 = CAN_BS1_6TQ;
        pCan_initTypeDef->TimeSeg2 = CAN_BS2_2TQ;
        break;
    default:
        break;
    }

    return;
}

/** ***************************************************************************
 * @name _can_init() CAN interface initializes
 * @brief Provides a function of CAN's initialization
 * 
 * @param [in] mode: CAN_operating_mode
 *        [in] baudRate: ECU baudRate
 * @retval
 ******************************************************************************/
static void _can_init(uint32_t mode, int baudRate)
{
    HAL_CAN_DeInit(&canHandle);

    canHandle.Instance = USER_CAN;
    canHandle.Init.TimeTriggeredMode = DISABLE;
    canHandle.Init.AutoBusOff = ENABLE;
    canHandle.Init.AutoWakeUp = ENABLE;
    canHandle.Init.AutoRetransmission = DISABLE;
    canHandle.Init.ReceiveFifoLocked = DISABLE;
    canHandle.Init.TransmitFifoPriority = ENABLE;
    canHandle.Init.Mode = mode;

    if (baudRate > _ECU_1000K)
        _can_set_baudrate(_ECU_250K, &canHandle.Init);
    else
        _can_set_baudrate(baudRate, &canHandle.Init);

    if (HAL_CAN_Init(&canHandle) != HAL_OK)
    {
        Error_Handler();
    }
}

/** ***************************************************************************
 * @name_CAN_Init_Filter_Engine () CAN's filter initialization
 * @brief Performs the filter of CAN interface 
 *
 * @param [in]
 * @retval
 ******************************************************************************/
static void _can_filter_commom_init(uint32_t filterMode, uint32_t filterScale)
{
    // initialize common parameters
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterMode = filterMode;
    canFilter.FilterScale = filterScale;
    canFilter.SlaveStartFilterBank = 14;
    canFilter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&canHandle, &canFilter) != HAL_OK)
    {
        Error_Handler();
    }
}

uint32_t filterNum = 0;
void can_config_filter_mask_message(uint32_t baseID, uint32_t baseMask)
{
    uint32_t mask, filtr;
    // initialize filter ECU ID
    filtr = (baseID << 3) | (USER_CAN_IDE << 2) | (USER_CAN_RTR << 1);
    mask = (baseMask << 3) | (USER_CAN_IDE << 2) | (USER_CAN_RTR << 1);

    canFilter.FilterIdHigh = filtr >> 16;
    canFilter.FilterIdLow = filtr;
    canFilter.FilterMaskIdHigh = mask >> 16;
    canFilter.FilterMaskIdLow = mask;
    canFilter.FilterBank = filterNum++;

    if (HAL_CAN_ConfigFilter(&canHandle, &canFilter) != HAL_OK)
    {
        Error_Handler();
    }
}

void can_config_filter_list_message(uint32_t ID1, uint32_t ID2)
{
    canFilter.FilterIdHigh = ((ID1 >> 16) << 5) & 0xffff;
    canFilter.FilterIdLow = (ID1 << 5) & 0xffff;
    canFilter.FilterMaskIdHigh = ((ID2 >> 16) << 5) & 0xffff;
    canFilter.FilterMaskIdLow = (ID2 << 5) & 0xffff;
    canFilter.FilterBank = filterNum++;

    if (HAL_CAN_ConfigFilter(&canHandle, &canFilter) != HAL_OK)
    {
        Error_Handler();
    }
}

void can_config_filter_clear(void)
{
    uint8_t filterNumTemp = filterNum;

    filterNum = 0;
    if (filterNumTemp > 0 && filterNumTemp <= 16) {
        while (filterNumTemp) {
            can_config_filter_list_message(0x00, 0x00);
            filterNumTemp--;
        }
        filterNum = 0;
    }
}

static void _can_start(void)
{
    if (HAL_CAN_Start(&canHandle) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&canHandle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanRxMsg can_rx_msg;

    if (hcan == &canHandle)
    {
        if (gUserConfiguration.can_mode == 0) {
            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_msg.rx_header, can_rx_msg.data) == HAL_OK)
            {
                if (can_rx_msg.rx_header.IDE == CAN_ID_STD && can_rx_msg.rx_header.DLC == 8)
                {
                    car_can_data_process(can_rx_msg.rx_header.StdId, can_rx_msg.data);
                }
            }
        } else if (gUserConfiguration.can_mode == 1) {
            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &gEcuInst.curr_rx_desc->rx_buffer.rx_header, gEcuInst.curr_rx_desc->rx_buffer.data) == HAL_OK)
            {
                canRxIntCounter++;
                if (gCANTxCompleteCallback != NULL) {
                    gCANRxCompleteCallback();
                }
            }
        }
    }
}

void USER_CAN_RX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&canHandle);
}

void HAL_CAN_TxComplete(CAN_HandleTypeDef *hcan)
{
    if (gUserConfiguration.can_mode == 1) {
        if (gCANTxCompleteCallback != NULL) {
            gCANTxCompleteCallback();
        }
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_TxComplete(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_TxComplete(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_TxComplete(hcan);
}

void USER_CAN_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&canHandle);
}

BOOL can_transmit(CanTxMsg* canTxMsg)
{
    HAL_StatusTypeDef status;
    uint32_t txMailbox;

    status = HAL_CAN_AddTxMessage(&canHandle, &canTxMsg->tx_header, canTxMsg->data, &txMailbox);

    if (status != HAL_OK)
    {
        return FALSE;
    }

    return TRUE;
}

/** ***************************************************************************
 * @name can_rtx_fun_config() assign callback funtion's pointers
 * @brief Performs tx/rx callback functions 
 *
 * @param [in] callback1 -- transmit function
 *             callback2 -- receive function
 * @retval N/A
 ******************************************************************************/
void can_rtx_fun_config(void (*callback1)(void), void (*callback2)(void))
{
    gCANTxCompleteCallback = callback1;
    gCANRxCompleteCallback = callback2;
}

/** ***************************************************************************
 * @name can_config() user interface, CAN, initialization 
 * @brief Provide an API of CAN initialization to system
 *        initialize CAN1 of version 3rd PCB
 * @param [in]  
 * @retval N/A
 ******************************************************************************/
void can_config(uint8_t mode, int baudRate)
{
    if (mode == 0) {
        _can_init(CAN_MODE_NORMAL, baudRate); // CAN_MODE_SILENT
        _can_filter_commom_init(CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_16BIT);
        can_config_filter_clear();

        can_config_filter_car();

    } else if (mode == 1) {
        _can_init(CAN_MODE_NORMAL, baudRate);
        _can_filter_commom_init(CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT);
        can_config_filter_clear();

        can_config_filter_j1939();
    }

    _can_start();
}

/** ***************************************************************************
 * @name can_detect_baudrate() Baud rate auto detection
 * @brief Traverse all of supported channels and find out the baud rate used 
 *        by host CAN device
 * @param [in] rate, 500kbps, 250kbps or 125kbps
 * @retval _ECU_500K, _ECU_250K or _ECU_125K
 ******************************************************************************/
static _ECU_BAUD_RATE _detect_baudrate(_ECU_BAUD_RATE rate)
{
    _ECU_BAUD_RATE found_rate;

    if (rate == _ECU_1000K)
        return rate;

    _can_init(CAN_MODE_NORMAL, rate);

    OS_Delay(CAN_DETECT_TIME);

    if (canRxIntCounter > canStartDetectRxIntCounter + 1)
    {
        found_rate = rate;
        return found_rate;
    }

    return _ECU_1000K;
}

BOOL can_detect_baudrate(_ECU_BAUD_RATE *rate)
{
    static int8_t retry_time = CAN_BAUD_RATE_RETRY;
    int result;
    // Auto-detection
    if (gEcuInst.state == _ECU_BAUDRATE_DETECT)
    {
        canStartDetectRxIntCounter = canRxIntCounter;

        result = _detect_baudrate(*rate);

        if (result != _ECU_1000K)
        {
            gEcuConfig.baudRate = result;

            _can_init(CAN_MODE_NORMAL, gEcuConfig.baudRate);

            set_can_baudrate(result);
        }
        else
        {
            if (retry_time-- > 0)
            {
                (*rate)++;
                if (*rate >= _ECU_1000K)
                {
                    *rate = _ECU_125K;
                }
                return FALSE;
            }

            _can_init(CAN_MODE_NORMAL, gEcuConfig.baudRate);
        }

        gEcuInst.state = _ECU_CHECK_ADDRESS;
    }
    return TRUE;
}
