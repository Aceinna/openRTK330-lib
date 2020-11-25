#include "mbedtls_config.h"

#ifdef MBEDTLS_ENTROPY_HARDWARE_ALT

#include "main.h"
#include <string.h>

#include "mbedtls/entropy_poll.h"

#include "stm32f4xx_hal.h"

extern RNG_HandleTypeDef RngHandle;

int mbedtls_hardware_poll(void *Data, unsigned char *Output, size_t Len, size_t *oLen)
{
    uint32_t index;
    uint32_t randomValue;

    for (index = 0; index < Len / 4; index++)
    {
        if (HAL_RNG_GenerateRandomNumber(&RngHandle, &randomValue) == HAL_OK)
        {
            *oLen += 4;
            memset(&(Output[index * 4]), (int)randomValue, 4);
        }
        else
        {
            Error_Handler();
        }
    }

    return 0;
}

#endif /*MBEDTLS_ENTROPY_HARDWARE_ALT*/
