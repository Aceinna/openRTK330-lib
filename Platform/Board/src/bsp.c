/******************************************************************************
* @file bsp.c
* @brief
* File description:
*		Use the file to Configure Cortex M3
*
* $Rev: 16166 $
* @date: 2011-03-09 11:53:45 -0800 (Wed, 09 Mar 2011) $
* @author: whpeng $
* HISTORY***********************************************************************
* 16/10/2019  |                                             | Daich
* Description:  Solve the bug that the DelayMs function only delays a fixed time
                and the old function will be losed by the optimization option o1
* 18/10/2019  |                                             | Daich
* Description:  esp32 boot mode. 
                exit interface
                timer it interface...
*******************************************************************************/
#include "stm32f4xx_hal.h"
#include "boardDefinition.h"
#include "configureGPIO.h"
#include "eepromAPI.h"
#include "bsp.h"
#include "led.h"
#include "rtcm.h"
#include "exit.h"
#include "rng.h"

ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart_debug;
UART_HandleTypeDef huart_user;
UART_HandleTypeDef huart_bt;
UART_HandleTypeDef huart_gps;


#if 0
/* UART5 init function */
void MX_UART5_Init(void)
{

  huart_debug.Instance = UART5;
  huart_debug.Init.BaudRate = 460800;
  huart_debug.Init.WordLength = UART_WORDLENGTH_8B;
  huart_debug.Init.StopBits = UART_STOPBITS_1;
  huart_debug.Init.Parity = UART_PARITY_NONE;
  huart_debug.Init.Mode = UART_MODE_TX_RX;
  huart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart_debug) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart_user.Instance = USART1;
  huart_user.Init.BaudRate = 460800;
  huart_user.Init.WordLength = UART_WORDLENGTH_8B;
  huart_user.Init.StopBits = UART_STOPBITS_1;
  huart_user.Init.Parity = UART_PARITY_NONE;
  huart_user.Init.Mode = UART_MODE_TX_RX;
  huart_user.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_user.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart_user) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
  huart_bt.Instance = USART2;
  huart_bt.Init.BaudRate = 460800;
  huart_bt.Init.WordLength = UART_WORDLENGTH_8B;
  huart_bt.Init.StopBits = UART_STOPBITS_1;
  huart_bt.Init.Parity = UART_PARITY_NONE;
  huart_bt.Init.Mode = UART_MODE_TX_RX;
  huart_bt.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_bt.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart_bt) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart_gps.Instance = USART3;
  huart_gps.Init.BaudRate = 460800;
  huart_gps.Init.WordLength = UART_WORDLENGTH_8B;
  huart_gps.Init.StopBits = UART_STOPBITS_1;
  huart_gps.Init.Parity = UART_PARITY_NONE;
  huart_gps.Init.Mode = UART_MODE_TX_RX;
  huart_gps.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_gps.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart_gps) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
#endif
/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

   //spi5 dma 
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);   
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/** ****************************************************************************
 * @name BSP_DEBUG_GPIOS_Gpio_Init
 * @brief for hardware debug
 * @retval N/A
 ******************************************************************************/
void BSP_DEBUG_GPIOS_Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_WritePin(DEBUG_GPIO1_PORT, DEBUG_GPIO1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DEBUG_GPIO2_PORT, DEBUG_GPIO2_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = DEBUG_GPIO1_PIN;
    HAL_GPIO_Init(DEBUG_GPIO1_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DEBUG_GPIO2_PIN;
    HAL_GPIO_Init(DEBUG_GPIO2_PORT, &GPIO_InitStruct);
}

/** ****************************************************************************
 * @name LED - API
 * @brief 
 * @retval N/A
 ******************************************************************************/
void BSP_LEDS_Gpio_Init(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

    LED1_CLK_ENABLE();
    LED2_CLK_ENABLE();
    LED3_CLK_ENABLE();

    // Configure the GPIO_LED pins
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;

    // LED1
    gpio_init_structure.Pin   = LED1_PIN;
    HAL_GPIO_Init(LED1_PORT, &gpio_init_structure);

    // LED2
    gpio_init_structure.Pin   = LED2_PIN;
    HAL_GPIO_Init(LED2_PORT, &gpio_init_structure);
    
    // LED3
    gpio_init_structure.Pin   = LED3_PIN;
    HAL_GPIO_Init(LED3_PORT, &gpio_init_structure);

    // By default, turn off LED by setting a high level on corresponding GPIO
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET);
}


/** ****************************************************************************
 * @name BSP_STA9100_Interface_Gpio_Init
 * @brief
 * @retval N/A
 ******************************************************************************/
void BSP_STA9100_Interface_Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_STDBY_PORT, ST_STDBY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_WKUP_PORT, ST_WKUP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = ST_BOOT_PIN;
    HAL_GPIO_Init(ST_BOOT_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = ST_RESET_PIN;
    HAL_GPIO_Init(ST_RESET_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = ST_STDBY_PIN;
    HAL_GPIO_Init(ST_STDBY_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = ST_WKUP_PIN;
    HAL_GPIO_Init(ST_WKUP_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = ST_PROG_BUF_CTL_PIN;
    HAL_GPIO_Init(ST_PROG_BUF_CTL_PORT, &GPIO_InitStruct);
}


/** ****************************************************************************
 * @name BSP_ESP32_Interface_Gpio_Init
 * @brief
 * @retval N/A
 ******************************************************************************/
void BSP_ESP32_Interface_Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //HAL_GPIO_WritePin(BT_BOOT_CTL_GPIO_PORT, BT_BOOT_CTL_PIN, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(BT_BOOT_CTL_GPIO_PORT, BT_BOOT_CTL_PIN, GPIO_PIN_SET);
#if 0
    GPIO_InitStruct.Pin = BT_BOOT_CTL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BT_BOOT_CTL_GPIO_PORT, &GPIO_InitStruct);
#else
    HAL_GPIO_WritePin(BT_RESET_GPIO_PORT, BT_BOOT_CTL_PIN|BT_RESET_PIN,GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BT_BOOT_CTL_PIN|BT_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BT_RESET_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BT_BOOT_CTL_GPIO_PORT, BT_BOOT_CTL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BT_BOOT_CTL_GPIO_PORT, BT_RESET_PIN, GPIO_PIN_SET);
#endif
}


void esp32_reset()
{
    HAL_GPIO_WritePin(GPIOF,BT_RESET_PIN,GPIO_PIN_RESET);
    DelayMs(100);
    HAL_GPIO_WritePin(BT_RESET_GPIO_PORT, BT_RESET_PIN,GPIO_PIN_SET);
}

/** ****************************************************************************
 * @name set_esp32_to_boot_mode
 * @brief ESP32 reset
 * @retval N/A
 ******************************************************************************/
void set_esp32_to_boot_mode(void)    //TODO:
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    //disable rtk mcu usart2 tx
    HAL_GPIO_WritePin(BT_USART_TX_GPIO_PORT, BT_USART_TX_PIN ,GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = BT_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BT_USART_TX_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOF,BT_BOOT_CTL_PIN|BT_RESET_PIN,GPIO_PIN_RESET);  //reset 0v   boot 0v
    DelayMs(2000);
    HAL_GPIO_WritePin(BT_RESET_GPIO_PORT, BT_RESET_PIN,GPIO_PIN_SET);      //reset 3.3v boot3.3v
}
/** ****************************************************************************
 * @name BSP_SENSOR_Interface_Gpio_Init
 * @brief
 * @retval N/A
 ******************************************************************************/
void BSP_SENSOR_Interface_Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    SPI(SPI_MOSI,OFF);
    SPI(SPI_SCK,OFF);
    SPI(SPI_NSS_ALL,OFF);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = SPI_MOSI_PIN;
    HAL_GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SPI_SCK_PIN;
    HAL_GPIO_Init(SPI_SCK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = SPI_NSS_PINS;
    HAL_GPIO_Init(SPI_NSS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_MISO_PINS;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SPI_MISO_PORT, &GPIO_InitStruct);
}

/** ***************************************************************************
 * @name SetIntVectorOffset
 * @brief
 * @retval N/A
 ******************************************************************************/
void SetIntVectorOffset(uint32_t offset)
{
    SCB->VTOR = FLASH_BASE | offset;
}

/** ***************************************************************************
 * @name SystemClock_Config  System Clock Configuration
 * @brief System Clock Configuration
 * @retval N/A
 ******************************************************************************/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 232;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** ***************************************************************************
 * @name BSP_GPIO_Init
 * @brief board gpios init
 * @retval N/A
 ******************************************************************************/
void BSP_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    BSP_DEBUG_GPIOS_Gpio_Init();
    //BSP_LEDS_Gpio_Init();
    led_driver_install();
    BSP_STA9100_Interface_Gpio_Init();
    pps_exit_init();
    BSP_ESP32_Interface_Gpio_Init();
    BSP_SENSOR_Interface_Gpio_Init();
    BSP_Spi_Pins_For_Test();
    MX_ADC1_Init();
}


/** ***************************************************************************
 * @name BoardInit The RTK330 board initialization
 * @param [in] N/A
 * @param [out] N/A
 * @retval N/A
 ******************************************************************************/
void BoardInit(void)
{
    /* Vector Table Relocation in Internal FLASH */
    SetIntVectorOffset(APP_NVIC_OFFSET); 

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Init gpio pins */
    BSP_GPIO_Init();

    /* RNG */
    RNG_Init();
}


/** ****************************************************************************
 * @name BSP_BootMode_Detect_Gpio_Init
 * @brief Switch to enter bootmode
 * @retval N/A
 ******************************************************************************/
void BSP_BootMode_Detect_Gpio_Init(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

    ST_MODE_CLK_ENABLE();
    //Configure the BOOT mode detect pin
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_MODE_PIN;
    HAL_GPIO_Init(ST_MODE_PORT, &gpio_init_structure);
}

int BootMode_Detect(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(ST_MODE_PORT, ST_MODE_PIN);
    return (int)(state == 0);
}

/** ****************************************************************************
 * @name STA9100 - API
 * @brief 
 * @retval N/A
 ******************************************************************************/
void BSP_STA9100_Gpio_Init(void)
{
    GPIO_InitTypeDef  gpio_init_structure;
    
    ST_RESET_CLK_ENABLE();
    ST_STDBY_CLK_ENABLE();
    ST_WKUP_CLK_ENABLE();
    ST_BOOT_CLK_ENABLE();

    // Configure the RESET pin
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_RESET_PIN;
    HAL_GPIO_Init(ST_RESET_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_SET);

    // Configure the STDBY pin
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_STDBY_PIN;
    HAL_GPIO_Init(ST_STDBY_PORT, &gpio_init_structure);

    // Configure the WKUP pin
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_WKUP_PIN;
    HAL_GPIO_Init(ST_WKUP_PORT, &gpio_init_structure);

    // Configure the BOOT pin for ST normal mode
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_BOOT_PIN;
    HAL_GPIO_Init(ST_BOOT_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_RESET);
}

void BSP_STA9100_Reset(void)
{
    HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_RESET);
    DelayMs(100);
    HAL_GPIO_WritePin(ST_RESET_PORT, ST_RESET_PIN, GPIO_PIN_SET);
    DelayMs(100);
}

void BSP_STA9100_Enter_BOOT_MODE(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

    // Configure the ST UART2 MCU pins as inputs
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_UART2_TX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = ST_UART2_RX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);

    // Configure the ST UART BUF control pin and set it high for enabling buffer
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_PROG_BUF_CTL_PIN;
    HAL_GPIO_Init(ST_PROG_BUF_CTL_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_SET);
    // Set BOOT pin
    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_SET);
}

void BSP_STA9100_Enter_PASSTHROUIGH_MODE(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

    // Configure the ST UART2 MCU pins as inputs
    gpio_init_structure.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_UART2_TX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = ST_UART2_RX_PIN;
    HAL_GPIO_Init(ST_UART2_PORT, &gpio_init_structure);

    // Configure the ST UART BUF control pin and set it high for enabling buffer
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = ST_PROG_BUF_CTL_PIN;
    HAL_GPIO_Init(ST_PROG_BUF_CTL_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(ST_PROG_BUF_CTL_PORT, ST_PROG_BUF_CTL_PIN, GPIO_PIN_SET);
    // Set BOOT pin
    HAL_GPIO_WritePin(ST_BOOT_PORT, ST_BOOT_PIN, GPIO_PIN_RESET);
}






/** ****************************************************************************
 * @name ResetForEnterBootMode
 * @brief choose bootmode 
 * @retval N/A
 ******************************************************************************/
void ResetForEnterBootMode(void)
{
    BSP_BootMode_Detect_Gpio_Init();

    int state = BootMode_Detect(); 
    if (state){ // iap download STA9100 FW and ESP32 FW
        BSP_STA9100_Gpio_Init();

        BSP_STA9100_Enter_BOOT_MODE();

        BSP_STA9100_Reset();

        set_esp32_to_boot_mode();

        LED2_Off();
#if 0
        while (1) 
        {
            LED1_Toggle();  // GREEN 
            DelayMs(500); 
        }
#endif
        return ;
    }

    HAL_GPIO_WritePin(GPIOC,ST_WKUP_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC,ST_STDBY_PIN,GPIO_PIN_SET);

    ResetSTIForNormalMode();

    HAL_GPIO_WritePin(GPIOC,ST_PROG_BUF_CTL_PIN,GPIO_PIN_RESET);
}


/** ****************************************************************************
 * @name SystemReset - API
 * @brief system reset
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @retval N/A
 ******************************************************************************/
void HW_SystemReset(void)
{
    *((uint32_t *)0xE000ED0C) = 0x05fa0004; 
}

static BOOL testMode   = FALSE;
static BOOL testStatus = TRUE;

void HW_SetTestMode(BOOL fOn)
{
	testMode = fOn;
}

BOOL HW_IsTestOK(void)
{
	if(testMode){
		return testStatus;
	}
	return TRUE;
}

void HW_BootJump(uint32_t addr)
{
    asm("LDR SP, [R0]");
    asm("LDR PC, [R0, #4]");
}

void DelayMs(uint32_t msec)
{
    uint32_t i = 0;  
    while(msec--)
    {
        i = 18000;  //Լ1ms
        while(i--)
        {
            asm("nop");
        }
    }
#if 0
    for (int i = msec; i > 0; i--)
    {
        for (int j = 0; j < 30000000; j++)
            i--;
    }
#endif
}

/** ****************************************************************************
 * @name BSP_BootMode_Detect_Gpio_Init
 * @brief Switch to enter bootmode
 * @retval N/A
 ******************************************************************************/
void BSP_Spi_Pins_For_Test(void)
{
    GPIO_InitTypeDef  gpio_init_structure;

    USER_SPI_DRDY_GPIO_CLK_ENABLE();
    USER_SPI_NSS_GPIO_CLK_ENABLE();
    USER_SPI_MOSI_GPIO_CLK_ENABLE();
    USER_SPI_MISO_GPIO_CLK_ENABLE();
    USER_SPI_SCK_GPIO_CLK_ENABLE();

    //Configure the BOOT mode detect pin
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Pin   = USER_SPI_DRDY_PIN;
    HAL_GPIO_Init(USER_SPI_DRDY_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = USER_SPI_NSS_PIN;
    HAL_GPIO_Init(USER_SPI_NSS_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = USER_SPI_MOSI_PIN;
    HAL_GPIO_Init(USER_SPI_MOSI_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = USER_SPI_MISO_PIN;
    HAL_GPIO_Init(USER_SPI_MISO_PORT, &gpio_init_structure);
    gpio_init_structure.Pin   = USER_SPI_SCK_PIN;
    HAL_GPIO_Init(USER_SPI_SCK_PORT, &gpio_init_structure);
}

void DRDY_Toggle(void)
{
    HAL_GPIO_TogglePin(USER_SPI_DRDY_PORT, USER_SPI_DRDY_PIN);
}


void NSS_Toggle(void)
{
    HAL_GPIO_TogglePin(USER_SPI_NSS_PORT, USER_SPI_NSS_PIN);
}

void SCK_Toggle(void)
{
    HAL_GPIO_TogglePin(USER_SPI_SCK_PORT, USER_SPI_SCK_PIN);
}

void DRDY_ON(void)
{
    HAL_GPIO_WritePin(USER_SPI_DRDY_PORT, USER_SPI_DRDY_PIN, GPIO_PIN_SET);
}

void DRDY_OFF(void)
{
    HAL_GPIO_WritePin(USER_SPI_DRDY_PORT, USER_SPI_DRDY_PIN, GPIO_PIN_RESET);
}

void NSS_ON(void)
{
    HAL_GPIO_WritePin(USER_SPI_NSS_PORT, USER_SPI_NSS_PIN, GPIO_PIN_SET);
}

void NSS_OFF(void)
{
    HAL_GPIO_WritePin(USER_SPI_NSS_PORT, USER_SPI_NSS_PIN, GPIO_PIN_RESET);
}

void SCK_ON(void)
{
    HAL_GPIO_WritePin(USER_SPI_SCK_PORT, USER_SPI_SCK_PIN, GPIO_PIN_SET);
}

void SCK_OFF(void)
{
    HAL_GPIO_WritePin(USER_SPI_SCK_PORT, USER_SPI_SCK_PIN, GPIO_PIN_RESET);
}


void wt_pulse_detect_init(void)
{
    PUlSE_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PULSE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PULSE_PORT, &GPIO_InitStruct);

    FWD_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = FWD_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_PULLUP
    HAL_GPIO_Init(FWD_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(PUlSE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PUlSE_IRQn);
}
