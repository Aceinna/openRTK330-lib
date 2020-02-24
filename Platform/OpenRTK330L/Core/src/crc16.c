#include "stdint.h"
#include "crc16.h"


uint16_t CalculateCRC (uint8_t *buf, uint16_t  length)
{
	uint16_t crc = 0x1D0F;  //non-augmented inital value equivalent to the augmented initial value 0xFFFF
	
	for (int i=0; i < length; i++) {
		crc ^= buf[i] << 8;
		
		for (int j=0; j<8; j++) {
			if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
			else {
                crc = crc << 1;
            }
		}
	}
	
	return ((crc << 8 ) & 0xFF00) | ((crc >> 8) & 0xFF);
}
