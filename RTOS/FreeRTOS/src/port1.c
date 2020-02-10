#include "osapi.h"


void OSDisableHook()
{
    portENTER_CRITICAL();
}

void OSEnableHook()
{
	portEXIT_CRITICAL();
}

void OS_Delay(uint32_t msec)
{
    osDelay (msec);
}

inline void OSDisableHookIfNotInIsr()
{
      if(!inHandlerMode()){
        portENTER_CRITICAL();
      }
}

inline void OSEnableHookIfNotInIsr()
{
      if(!inHandlerMode()){
         portEXIT_CRITICAL();
      }
}