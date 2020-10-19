/** ***************************************************************************
 * @file eepromAPI.h 
 * @Author
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef _EEPROM_API_H
#define _EEPROM_API_H

#include <stdint.h> 
#include "constants.h"

#define EEPROM_CAL_ADDR1        0x080Ed000
#define EEPROM_CAL_ADDR2        0x080Ee000
#define EEPROM_CAL_ADDR3        0x080Ef000
#define EEPROM_CONFIG_ADDR      0x080Ec800
#define CAL_PARTITION_SIZE      4096        // 4K
#define CAL_CRC_OFFSET          4092
#define EEPROM_USER_ADDR        0x080C0000
#define EEPROM_PAGE_SIZE        0x800
#define BOOT_SIGNATURE_ADDR     0x2004ff10  
#define APP_START_ADDR          0x08010000
#define APP_SIGNATURE_ADDR      0x080A0000

extern void EEPROM_ReadWords(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL EEPROM_WriteWords(uint16_t addr, uint16_t num, void *source) ;
extern void EEPROM_ReadByte(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL EEPROM_WriteByte(uint16_t addr, uint16_t num, void *source) ;
extern void EEPROM_ReadSerialNumber(void *destination) ;
extern void EEPROM_ReadProdConfig(void *destination) ;
extern void EEPROM_ReadCalibration(int idx, void* destination);
extern void EEPROM_ReadFactoryConfiguration(void* destination);
extern BOOL EEPROM_ReadFromCalPartition( uint16_t offset, uint16_t num, void  *destination);
extern BOOL EEPROM_WriteToCalPartition(uint16_t offset, uint16_t num, void *source);
extern uint8_t *EEPROM_GetCalTabPtr(int idx);
extern BOOL EEPROM_WriteApp(uint32_t addr,uint8_t *buf, uint16_t len);

#endif /* S_EEPROM_H */ 


