/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
* 16/10/2019  |                                             | Daich
* Description: remove uart it interface to uart.c
* 16/10/2019  |                                             | Daich
* Description: remove some unused var
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Indices.h"
#include "boardDefinition.h"
#include "utils.h"
#include "rtcm.h"
#include "led.h"
#include "timer.h"


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

volatile struct sHardFaultStacked
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} HardFault;

#if defined(__ICCARM__)
// Copies of register values pushed onto stack before invoking HardFault handler

// Copy HardFault register values from stack to debugger-friendly memory locations
static void save_fault_cpu_state(uint32_t *stackPtr)
{
    HardFault.r0 = stackPtr[0];
    HardFault.r1 = stackPtr[1];
    HardFault.r2 = stackPtr[2];
    HardFault.r3 = stackPtr[3];
    HardFault.r12 = stackPtr[4];
    HardFault.lr = stackPtr[5];
    HardFault.pc = stackPtr[6];
    HardFault.psr = stackPtr[7];
}

#define IAR_FUNC_ENTRY_PUSHES 2

#endif // __ICCARM__

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
  /* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
    //volatile uint32_t r0;
    //volatile uint32_t r1;
    //volatile uint32_t r2;
    //volatile uint32_t r3;
    //volatile uint32_t r12;
    //volatile uint32_t lr; /* Link register. */
    //volatile uint32_t pc; /* Program counter. */
    //volatile uint32_t psr;/* Program status register. */

    HardFault.r0 = pulFaultStackAddress[0];
    HardFault.r1 = pulFaultStackAddress[1];
    HardFault.r2 = pulFaultStackAddress[2];
    HardFault.r3 = pulFaultStackAddress[3];

    HardFault.r12 = pulFaultStackAddress[4];
    HardFault.lr = pulFaultStackAddress[5];
    HardFault.pc = pulFaultStackAddress[6];
    HardFault.psr = pulFaultStackAddress[7];

    /* When the following line is hit, the variables contain the register values. */
    for (;;)
        ;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

#if defined(__ICCARM__)
     // NOTE: this only works with IAR
    if (__get_CONTROL() & 2)
    {
        // Process SP in use.
        save_fault_cpu_state((uint32_t *)__get_PSP() + IAR_FUNC_ENTRY_PUSHES);
    }
    else
    {
        // Main SP in use.
        save_fault_cpu_state((uint32_t *)__get_MSP() + IAR_FUNC_ENTRY_PUSHES);
    }
#else
    __asm volatile(
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n");
#endif // __ICCARM__

    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
    /* USER CODE BEGIN MemoryManagement_IRQn 1 */

    /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
    /* USER CODE BEGIN BusFault_IRQn 1 */

    /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
    /* USER CODE BEGIN UsageFault_IRQn 1 */

    /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
/**
  * @brief This function handles EXTI line3 interrupts.
  */

