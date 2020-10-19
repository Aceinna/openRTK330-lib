/**
******************************************************************************
* @file    hwAPI.h 
******************************************************************************
*/
#ifndef __HW_API_H
#define __HW_API_H

#include "stdint.h"
#include "constants.h"


// GPIO - related fucntions
void    HW_SetTestMode(BOOL fOn);
BOOL    HW_IsTestOK();

// system related functions
void    HW_SystemReset(void);
BOOL    HW_IsBootModeEnforced();
void    HW_EnforceBootMode();
BOOL    HW_IsAppModeEnforced();
void    HW_EnforceAppMode();
void    HW_HDTestMode();
void    HW_ClearBootSignature();
bool    IsNeedToHardwareTest();
BOOL    IsNeedToUpdateApp();

extern BOOL fSPI;
extern BOOL fUART;


#endif //__UART_H