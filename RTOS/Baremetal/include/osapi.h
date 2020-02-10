#ifndef _OS_API_H
#define _OS_API_H
#include <stdint.h>

extern int gIsrDisableCount;

#define   OSEnterISR() do{ gIsrDisableCount++; __ASM  ("cpsid i");} while(0)
#define   OSExitISR()  do{ gIsrDisableCount--; if (gIsrDisableCount == 0) { __ASM  ("cpsie i"); }}while(0)

#define   ENTER_CRITICAL() do{ gIsrDisableCount++; __ASM  ("cpsid i");} while(0)
#define   EXIT_CRITICAL()  do{ gIsrDisableCount--; if (gIsrDisableCount == 0) { __ASM  ("cpsie i"); }}while(0)

void      DelayMs(int ms);
void      OS_Delay(int ms);

#endif
