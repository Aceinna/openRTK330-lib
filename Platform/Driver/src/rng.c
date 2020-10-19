#include "rng.h"
#include "stm32f4xx_hal.h"

RNG_HandleTypeDef RngHandle;
/** ***************************************************************************
 * @name RNG_Init
 * @brief N/A
 * @retval N/A
 ******************************************************************************/
void RNG_Init(void)
{
    RngHandle.Instance = RNG;
    if (HAL_RNG_Init(&RngHandle) != HAL_OK)
    {
        Error_Handler();
    }
}


