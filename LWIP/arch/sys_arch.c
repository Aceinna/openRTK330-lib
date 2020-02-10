/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/* lwIP includes. */
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "stm32f4xx_hal.h"

#include "arch/sys_arch.h"

// create messagebox
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	osMessageQDef(QUEUE, size, void *);

	*mbox = osMessageCreate(osMessageQ(QUEUE), NULL);

#if SYS_STATS
	++lwip_stats.sys.mbox.used;
	if (lwip_stats.sys.mbox.max < lwip_stats.sys.mbox.used)
	{
		lwip_stats.sys.mbox.max = lwip_stats.sys.mbox.used;
	}
#endif /* SYS_STATS */
	if (*mbox == NULL)
		return ERR_MEM;

	return ERR_OK;
}

// delete messagebox
void sys_mbox_free(sys_mbox_t *mbox)
{
	if (osMessageWaiting(*mbox))
	{
		/* Line for breakpoint.  Should never break here! */
		portNOP();
#if SYS_STATS
		lwip_stats.sys.mbox.err++;
#endif /* SYS_STATS */

		// TODO notify the user of failure.
	}

	osMessageDelete(*mbox);

#if SYS_STATS
	--lwip_stats.sys.mbox.used;
#endif /* SYS_STATS */
}

// post message util success
void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	while (osMessagePut(*mbox, (uint32_t)msg, osWaitForever) != osOK)
		;
}

// post message one time
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	err_t result;

	if (osMessagePut(*mbox, (uint32_t)msg, 0) == osOK)
	{
		result = ERR_OK;
	}
	else
	{
		// could not post, queue must be full
		result = ERR_MEM;

#if SYS_STATS
		lwip_stats.sys.mbox.err++;
#endif /* SYS_STATS */
	}

	return result;
}

// wait message
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	osEvent event;
	uint32_t starttime = osKernelSysTick();
	;

	if (timeout != 0)
	{
		event = osMessageGet(*mbox, timeout);

		if (event.status == osEventMessage)
		{
			*msg = (void *)event.value.v;
			return (osKernelSysTick() - starttime);
		}
		else
		{
			return SYS_ARCH_TIMEOUT;
		}
	}
	else
	{
		event = osMessageGet(*mbox, osWaitForever);
		*msg = (void *)event.value.v;
		return (osKernelSysTick() - starttime);
	}
}

// try to get message
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	osEvent event;

	event = osMessageGet(*mbox, 0);

	if (event.status == osEventMessage)
	{
		*msg = (void *)event.value.v;
		return ERR_OK;
	}
	else
	{
		return SYS_MBOX_EMPTY;
	}
}

// check a messagebox
int sys_mbox_valid(sys_mbox_t *mbox)
{
	if (*mbox == SYS_MBOX_NULL)
		return 0;
	else
		return 1;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
	*mbox = SYS_MBOX_NULL;
}

// create a sem
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	osSemaphoreDef(SEM);

	*sem = osSemaphoreCreate(osSemaphore(SEM), 1);

	if (*sem == NULL)
	{
#if SYS_STATS
		++lwip_stats.sys.sem.err;
#endif /* SYS_STATS */
		return ERR_MEM;
	}

	if (count == 0) // Means it can't be taken
	{
		osSemaphoreWait(*sem, 0);
	}

#if SYS_STATS
	++lwip_stats.sys.sem.used;
	if (lwip_stats.sys.sem.max < lwip_stats.sys.sem.used)
	{
		lwip_stats.sys.sem.max = lwip_stats.sys.sem.used;
	}
#endif /* SYS_STATS */

	return ERR_OK;
}

// wait a sem
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	uint32_t starttime = osKernelSysTick();

	if (timeout != 0)
	{
		if (osSemaphoreWait(*sem, timeout) == osOK)
		{
			return (osKernelSysTick() - starttime);
		}
		else
		{
			return SYS_ARCH_TIMEOUT;
		}
	}
	else
	{
		while (osSemaphoreWait(*sem, osWaitForever) != osOK)
			;
		return (osKernelSysTick() - starttime);
	}
}

// send a sem
void sys_sem_signal(sys_sem_t *sem)
{
	osSemaphoreRelease(*sem);
}

// delete a sem
void sys_sem_free(sys_sem_t *sem)
{
#if SYS_STATS
	--lwip_stats.sys.sem.used;
#endif /* SYS_STATS */

	osSemaphoreDelete(*sem);
}

// check a sem
int sys_sem_valid(sys_sem_t *sem)
{
	if (*sem == SYS_SEM_NULL)
		return 0;
	else
		return 1;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
	*sem = SYS_SEM_NULL;
}

osMutexId lwip_sys_mutex;
osMutexDef(lwip_sys_mutex);
// Initialize sys arch
void sys_init(void)
{
	lwip_sys_mutex = osMutexCreate(osMutex(lwip_sys_mutex));
}

#if LWIP_COMPAT_MUTEX == 0
/* Create a new mutex*/
err_t sys_mutex_new(sys_mutex_t *mutex)
{

	osMutexDef(MUTEX);

	*mutex = osMutexCreate(osMutex(MUTEX));

	//*mutex = xSemaphoreCreateMutex();
	if (*mutex == NULL)
	{
#if SYS_STATS
		++lwip_stats.sys.mutex.err;
#endif /* SYS_STATS */
		return ERR_MEM;
	}

#if SYS_STATS
	++lwip_stats.sys.mutex.used;
	if (lwip_stats.sys.mutex.max < lwip_stats.sys.mutex.used)
	{
		lwip_stats.sys.mutex.max = lwip_stats.sys.mutex.used;
	}
#endif /* SYS_STATS */
	return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* Deallocate a mutex*/
void sys_mutex_free(sys_mutex_t *mutex)
{
#if SYS_STATS
	--lwip_stats.sys.mutex.used;
#endif /* SYS_STATS */

	osMutexDelete(*mutex);
}
/*-----------------------------------------------------------------------------------*/
/* Lock a mutex*/
void sys_mutex_lock(sys_mutex_t *mutex)
{
	osMutexWait(*mutex, osWaitForever);
}

/*-----------------------------------------------------------------------------------*/
/* Unlock a mutex*/
void sys_mutex_unlock(sys_mutex_t *mutex)
{
	osMutexRelease(*mutex);
}
#endif /*LWIP_COMPAT_MUTEX*/

/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread , void *arg, int stacksize, int prio)
{
  const osThreadDef_t os_thread_def = { (char *)name, (os_pthread)thread, (osPriority)prio, 0, stacksize};
  return osThreadCreate(&os_thread_def, arg);
}

sys_prot_t sys_arch_protect(void)
{
	osMutexWait(lwip_sys_mutex, osWaitForever);
	return (sys_prot_t)1;
}

void sys_arch_unprotect(sys_prot_t pval)
{
	(void)pval;
	osMutexRelease(lwip_sys_mutex);
}
