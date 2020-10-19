#ifndef BASE_STATION

#ifndef _M_BASE64_H  
#define _M_BASE64_H  

#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
  
void base64_encode(uint8_t *str, uint8_t *res);
void bae64_decode(uint8_t *code, uint8_t *res);

#endif
  
#endif
