#include "netif/ethernetif.h"
#include "lwip_comm.h"
#include "netif/etharp.h"
#include "cmsis_os.h"
#include "osapi.h"
#include "string.h"
#include "user_config.h"
#include "boardDefinition.h"

// Declare the ETH structure
ETH_HandleTypeDef EthHandle;
ETH_DMADescTypeDef DMATxDscrTab[ETH_TXBUFNB];
ETH_DMADescTypeDef DMARxDscrTab[ETH_RXBUFNB];
uint8_t ETH_Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];
uint8_t ETH_Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];

/* Semaphore to signal incoming packets */
osSemaphoreId s_xSemaphore = NULL;


void KSZ8041NL_reset_port_init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    ETH_RESET_CLK_ENABLE();

    GPIO_Initure.Pin = ETH_RESET_PIN;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ETH_RESET_PORT, &GPIO_Initure);
}

void KSZ8041NL_reset(void)
{
    //__set_PRIMASK(1); // close all interupt
    HAL_GPIO_WritePin(ETH_RESET_PORT, ETH_RESET_PIN, GPIO_PIN_RESET);
    OS_Delay(50);
    HAL_GPIO_WritePin(ETH_RESET_PORT, ETH_RESET_PIN, GPIO_PIN_SET);
    OS_Delay(5);
    //__set_PRIMASK(0); // open all interupt
}

/** ***************************************************************************
 * @name HAL_ETH_MspInit
 * @brief called by HAL_ETH_Init 
 *  init RMII interface and enable ETH_IRQn
 *  RMII
    ETH_MDIO -------------------------> PA2    
    ETH_MDC --------------------------> PC1    
    ETH_RMII_REF_CLK------------------> PA1    
    ETH_RMII_CRS_DV ------------------> PA7    
    ETH_RMII_RXD0 --------------------> PC4    
    ETH_RMII_RXD1 --------------------> PC5    
    ETH_RMII_TX_EN -------------------> PG11   
    ETH_RMII_TXD0 --------------------> PG13   
    ETH_RMII_TXD1 --------------------> PG14   
    ETH_RESET-------------------------> PE0 
 * @param *heth point to EthHandle
 * @retval N/A
 ******************************************************************************/
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    GPIO_InitTypeDef GPIO_Initure;

    ETH_PORT_CLK_ENABLE();
    ETH_MDIO_CLK_ENABLE();
    ETH_MDC_CLK_ENABLE();         
    ETH_RMII_REF_CLK_CLK_ENABLE();
    ETH_RMII_CRS_DV_CLK_ENABLE();
    ETH_RMII_RXD0_CLK_ENABLE();
    ETH_RMII_RXD1_CLK_ENABLE();
    ETH_RMII_TX_EN_CLK_ENABLE();
    ETH_RMII_TXD0_CLK_ENABLE();
    ETH_RMII_TXD1_CLK_ENABLE();

    GPIO_Initure.Mode = GPIO_MODE_AF_PP;
    GPIO_Initure.Pull = GPIO_NOPULL;
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;
    GPIO_Initure.Alternate = ETH_PORT_AF;

    GPIO_Initure.Pin = ETH_MDIO_PIN;
    HAL_GPIO_Init(ETH_MDIO_PORT, &GPIO_Initure);
    GPIO_Initure.Pin = ETH_MDC_PIN;
    HAL_GPIO_Init(ETH_MDC_PORT, &GPIO_Initure);

    GPIO_Initure.Pin = ETH_RMII_REF_CLK_PIN;
    HAL_GPIO_Init(ETH_RMII_REF_CLK_PORT, &GPIO_Initure);
    GPIO_Initure.Pin = ETH_RMII_CRS_DV_PIN;
    HAL_GPIO_Init(ETH_RMII_CRS_DV_PORT, &GPIO_Initure);

    GPIO_Initure.Pin = ETH_RMII_RXD0_PIN;
    HAL_GPIO_Init(ETH_RMII_RXD0_PORT, &GPIO_Initure);
    GPIO_Initure.Pin = ETH_RMII_RXD1_PIN;
    HAL_GPIO_Init(ETH_RMII_RXD1_PORT, &GPIO_Initure);
    GPIO_Initure.Pin = ETH_RMII_TX_EN_PIN;
    HAL_GPIO_Init(ETH_RMII_TX_EN_PORT, &GPIO_Initure);

    GPIO_Initure.Pin = ETH_RMII_TXD0_PIN;
    HAL_GPIO_Init(ETH_RMII_TXD0_PORT, &GPIO_Initure);
    GPIO_Initure.Pin = ETH_RMII_TXD1_PIN;
    HAL_GPIO_Init(ETH_RMII_TXD1_PORT, &GPIO_Initure);

    HAL_NVIC_SetPriority(ETH_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

/** ***************************************************************************
 * @name ETH_IRQHandler —— ETH interrupt
 * @brief push data to LWIP
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void ETH_IRQHandler(void)
{
    OSEnterISR();
    HAL_ETH_IRQHandler(&EthHandle);
    OSExitISR();
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    osSemaphoreRelease(s_xSemaphore);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMA_FLAG_FBE))
    {	  	
 	    /* Clear the Eth DMA Tx IT pending bits */
 	    __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMA_IT_FBE);
  	}
}

/** ***************************************************************************
 * @name low_level_init
 * @brief In this function, the hardware should be initialized.
 * @param [in] netif : the already initialized lwip network interface structure
 * for this ethernetif
 * @retval N/A
 ******************************************************************************/
static void low_level_init(struct netif *netif)
{
    uint8_t* p_mac = get_static_mac();

    KSZ8041NL_reset_port_init();

    KSZ8041NL_reset();

    EthHandle.Instance = ETH;
    EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    EthHandle.Init.Speed = ETH_SPEED_100M;
    EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    EthHandle.Init.PhyAddress = KSZ8041NL_PHY_ADDRESS;
    EthHandle.Init.MACAddr = p_mac; //gnetif.hwaddr; 
    EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

    if (HAL_ETH_Init(&EthHandle) == HAL_OK)
    {
        netif->flags |= NETIF_FLAG_LINK_UP;
    }

    HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &ETH_Tx_Buff[0][0], ETH_TXBUFNB);
    HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &ETH_Rx_Buff[0][0], ETH_RXBUFNB);

    /* set netif MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set netif MAC hardware address */
    netif->hwaddr[0] = p_mac[0];
    netif->hwaddr[1] = p_mac[1];
    netif->hwaddr[2] = p_mac[2];
    netif->hwaddr[3] = p_mac[3];
    netif->hwaddr[4] = p_mac[4];
    netif->hwaddr[5] = p_mac[5];

    netif->mtu = 1500;

    /* Accept broadcast address and ARP traffic */
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    osSemaphoreDef(SEM);
    s_xSemaphore = osSemaphoreCreate(osSemaphore(SEM), 1);

    /* create the task that handles the ETH_MAC */
    osThreadDef(EthIf, ethernetif_input, osPriorityHigh, 0, INTERFACE_THREAD_STACK_SIZE);
    osThreadCreate(osThread(EthIf), netif);

    /* Enable MAC and DMA transmission and reception */
    HAL_ETH_Start(&EthHandle);
}

/** ***************************************************************************
 * @name low_level_output
 * @brief This function should do the actual transmission of the packet. 
 * The packet is contained in the pbuf that is passed to the function. 
 * This pbuf might be chained.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @param [in] p : the MAC packet to send
 * @retval ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 ******************************************************************************/
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    err_t errval;
    struct pbuf *q;
    uint8_t *buffer = (uint8_t *)(EthHandle.TxDesc->Buffer1Addr);
    __IO ETH_DMADescTypeDef *DmaTxDesc;
    uint32_t framelength = 0;
    uint32_t bufferoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;

    DmaTxDesc = EthHandle.TxDesc;
    bufferoffset = 0;

    for (q = p; q != NULL; q = q->next)
    {
        if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
        {
            errval = ERR_USE;
            goto error;
        }

        byteslefttocopy = q->len;
        payloadoffset = 0;

        while ((byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE)
        {
            memcpy((uint8_t *)((uint8_t *)buffer + bufferoffset), (uint8_t *)((uint8_t *)q->payload + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset));

            DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);

            if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
            {
                errval = ERR_USE;
                goto error;
            }

            buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);

            byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
            payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
            framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
            bufferoffset = 0;
        }

        memcpy((uint8_t *)((uint8_t *)buffer + bufferoffset), (uint8_t *)((uint8_t *)q->payload + payloadoffset), byteslefttocopy);
        bufferoffset = bufferoffset + byteslefttocopy;
        framelength = framelength + byteslefttocopy;
    }

    HAL_ETH_TransmitFrame(&EthHandle, framelength);

    errval = ERR_OK;

error:

    if ((EthHandle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
    {
        EthHandle.Instance->DMASR = ETH_DMASR_TUS;

        EthHandle.Instance->DMATPDR = 0;
    }
    return errval;
}


/** ***************************************************************************
 * @name low_level_input
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @retval a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
 ******************************************************************************/
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p = NULL, *q = NULL;
    uint16_t len = 0;
    uint8_t *buffer;
    __IO ETH_DMADescTypeDef *dmarxdesc;
    uint32_t bufferoffset = 0;
    uint32_t payloadoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t i = 0;

    if (HAL_ETH_GetReceivedFrame_IT(&EthHandle) != HAL_OK)
        return NULL;

    len = EthHandle.RxFrameInfos.length;
    buffer = (uint8_t *)EthHandle.RxFrameInfos.buffer;

    if (len > 0)
    {
        p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    }

    if (p != NULL)
    {
        dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
        bufferoffset = 0;

        for (q = p; q != NULL; q = q->next)
        {
            byteslefttocopy = q->len;
            payloadoffset = 0;

            while ((byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE)
            {
                memcpy((uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));
                
                dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
                buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);

                byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
                payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
                bufferoffset = 0;
            }

            memcpy((uint8_t *)((uint8_t *)q->payload + payloadoffset), (uint8_t *)((uint8_t *)buffer + bufferoffset), byteslefttocopy);
            bufferoffset = bufferoffset + byteslefttocopy;
        }
    }

    dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
    for (i = 0; i < EthHandle.RxFrameInfos.SegCount; i++)
    {
        dmarxdesc->Status |= ETH_DMARXDESC_OWN;
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
    }

    EthHandle.RxFrameInfos.SegCount = 0;

    if ((EthHandle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
    {
        EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
        EthHandle.Instance->DMARPDR = 0;
    }
    return p;
}


/** ***************************************************************************
 * @name ethernetif_input
 * @brief This function is the ethernetif_input task, it is processed when a packet 
 * is ready to be read from the interface. It uses the function low_level_input() 
 * that should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 * 
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @retval N/A
 ******************************************************************************/
void ethernetif_input(void const *argument)
{
    struct pbuf *p;
    struct netif *netif = (struct netif *)argument;

    for (;;)
    {
        if (osSemaphoreWait(s_xSemaphore, osWaitForever) == osOK)
        {
            do
            {
                p = low_level_input(netif);
                if (p != NULL)
                {
                    if (netif->input(p, netif) != ERR_OK)
                    {
                        pbuf_free(p);
                    }
                }
            } while (p != NULL);
        }
    }
}


/** ***************************************************************************
 * @name ethernetif_init
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 * 
 * This function should be passed as a parameter to netif_add().
 * 
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 ******************************************************************************/
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = HOSTNAME;
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}


/** ***************************************************************************
 * @name ethernetif_link_state_check
 * @brief This function sets the netif link status.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @return N/A  ETH_netlink_status_check
 ******************************************************************************/
void ethernetif_link_state_check(struct netif *netif)
{
    uint32_t regvalue = 0;

    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_BSR, &regvalue);

    if ((regvalue & PHY_LINKED_STATUS) != (uint16_t)RESET)
    {
        netif_set_link_up(netif);
    }
    else
    {
        netif_set_link_down(netif);
    }
}

/** ***************************************************************************
 * @name ethernetif_update_config
 * @brief Link callback function, this function is called on change of link status
 *         to update low level driver configuration.
 * @param [in] netif : the lwip network interface structure for this ethernetif
 * @return N/A
 ******************************************************************************/
void ethernetif_update_config(struct netif *netif)
{
    __IO uint32_t tickstart = 0;
    uint32_t regvalue = 0;

    if (netif_is_link_up(netif))
    {
        if (EthHandle.Init.AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
        {
            HAL_ETH_WritePHYRegister(&EthHandle, PHY_BCR, PHY_AUTONEGOTIATION);

            tickstart = HAL_GetTick();

            do
            {
                HAL_ETH_ReadPHYRegister(&EthHandle, PHY_BSR, &regvalue);

                if ((HAL_GetTick() - tickstart) > 1000)
                {
                    goto error;
                }

            } while (((regvalue & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));

            HAL_ETH_ReadPHYRegister(&EthHandle, PHY_SR, &regvalue);

            if ((regvalue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
            {
                EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
            }
            else
            {
                EthHandle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
            }

            if (regvalue & PHY_SPEED_STATUS)
            {
                EthHandle.Init.Speed = ETH_SPEED_10M;
            }
            else
            {
                EthHandle.Init.Speed = ETH_SPEED_100M;
            }
        }
        else
        {
        error:
            assert_param(IS_ETH_SPEED(EthHandle.Init.Speed));
            assert_param(IS_ETH_DUPLEX_MODE(EthHandle.Init.DuplexMode));

            HAL_ETH_WritePHYRegister(&EthHandle, PHY_BCR, ((uint16_t)(EthHandle.Init.DuplexMode >> 3) | (uint16_t)(EthHandle.Init.Speed >> 1)));
        }

        HAL_ETH_ConfigMAC(&EthHandle, (ETH_MACInitTypeDef *)NULL);

        HAL_ETH_Start(&EthHandle);
    }
    else
    {
        HAL_ETH_Stop(&EthHandle);
    }

    ethernetif_notify_conn_changed(netif);
}


u32_t sys_now(void)
{
    return HAL_GetTick();
}
