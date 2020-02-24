/** ****************************************************************************
 * @file s_eeprom.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief File description:
 *   This driver emulates a serial EEPROM by writing to internal flash.
 *  NOTE: This does a direct erase and write, if power is pulled part way through
 *  the erase cycle (~3s) or the write cycle ( <1s), the system will lose its
 *  configuration and calibration.
 *  FIXME: A future improvement would be to use two flash sectors and ping pong
 *  between them so that the important information is never lost.
 *
 * ******* IMPLEMENTATION NOTES AND SUGGESTIONS FOR MODIFICATION
 * We have a fake EERPOM that is implemented using on-chip flash. I chose the
 * uppermost flash sector. It is incredibly large but having it be the uppermost
 * means we don't run into problems as the code grows.
 * The three most important things you'll need to know:
 *  - where all the pieces are located
 *  - removing default values from being programmed
 *  - moving to a different sector.
 ******* WHERE ALL THE PIECES ARE LOCATED
 * The variable gEepromInFlash is defined in s_eeprom.c. It is a structure, meant
 * to represent the EEPROM variables in flash. Note that the EEPROM was implemented
 * from a C2xxx processor which has 16 bit bytes (see email "The tale of 16 bit
 * bytes" send to Joe and Tony on 6/24/13). Some quirkiness in the variable
 * definition is to match the oddness caused by the 16 bit bytes.
 *
 * The gEepromInFlash is defined at a fixed location by this pragma line:
 * #pragma location="FLASH_BASED_EEPROM"  // defined in the linker's ICF file
 *
 * As noted in the comment, this is defined in the linker ICF file. This is in
 * the project under board\stm32f2xx_flash.icf. The linker file is fairly odd,
 * very compiler specific. The IAR documentation gives some examples but it takes
 * awhile to build up to the complexity of the default file (which I only
 * changed a little).
 *
 * The first important thing to see is the ROM definition:
 * define symbol __ICFEDIT_region_ROM_start__ = 0x08000000;
 * define symbol __ICFEDIT_region_ROM_end__   = 0x080DFFFF; // end one sector early
 * Note the comment- this used to say "__ICFEDIT_region_ROM_end__   = 0x080FFFFF"
 * which would allocate all of the code space to the noted region.
 * This is actually done in the line
 *    place in ROM_region   { readonly };
 * which puts all of the readonly sections (i.e. compiled code) into the ROM_region.
 *
 * The main modification to the file is this section:
 * define symbol _region_FLASH_EEPROM_start__   = 0x080E0000;
 * define symbol _region_FLASH_EEPROM_end__     = 0x080FFFFF;
 * define region FLASH_EEPROM_region   = mem:[from _region_FLASH_EEPROM_start__   to _region_FLASH_EEPROM_end__];
 * define block FLASH_EEPROM_block { section FLASH_BASED_EEPROM };
 * place in FLASH_EEPROM_region { block FLASH_EEPROM_block };
 *
 * This allocated a block to belong to FLASH_EEPROM_region and puts the
 * FLASH_BASED_EEPROM section into that block/region.
 *
 ******* REMOVING DEFAULT VALUES FROM BEING PROGRAMMED
 * Once you can program the unit with proper configuration and calibration values,
 * you won't want it to revert to the defaults whenever you reprogram the code.
 *
 * To program the defaults, you have
 *
 * #pragma location="FLASH_BASED_EEPROM"  // 0x08060000 defined in the linker's ICF file
 * __root const uEeprom gEepromInFlash  = {
 *    .table = {
 *        .configuration = {
 *            .port1Usage = 1, // PRI_UCB_PORT
 *            .calibrationCRC = 53088,
 * 	},
 *	};
 *
 * To not program defaults, change the declaration of gEepromInFlash to be:
 * __root __no_init const uEeprom gEepromInFlash;
 *
 * This will make it so the memory doesn't get changed by the linker or loaded
 * to flash when programming.
 *
 ******* MOVING TO A DIFFERENT SECTOR
 * You may need to move to a different sector if the size of the flash on the
 * chip changes.
 *
 * If you need to move the EEPROM section, modify
 * _region_FLASH_EEPROM_start_ and _end_ accordingly.
 * Also modify __ICFEDIT_region_ROM_start__ and _end__.
 * (I usually find it is easiest to draw a memory map and then make the changes
 * in the linker file once I've sorted out the numbers and section sizes.)
 *
 ******************************************************************************/

#include <stdint.h>
#include <string.h> // memcpy

//**************************
#include "stm32f4xx_hal.h"
#include "crc16.h"
#include "eepromAPI.h"
#include "configuration.h"

#include "parameters.h"

#include "hwApi.h"
#include "Indices.h"
#include "s_eeprom.h"
#ifndef BAREMETAL_OS
    #include "osapi.h"
	#include "calibration.h"
#else
    #include "bare_osapi.h"
#endif
// #include "UserConfiguration.h"
//#define CALIBRATION_OFFSET (0x200) // 0x100 = 256 in bytes
#define EEPROM_LAST_ADDR ((EEPROM_CAL_ADDR1 + 0x3000) - 1)
//#include "watchdog.h"

const uint8_t *pBootFlagFlash = (uint8_t *)BOOT_FLAG_ADDR;

#pragma pack(1)
typedef union {
    struct
    {
        uint8_t memory[2048];
    } data;
    struct
    {
        uint16_t countOfEepromErases;
        ConfigurationStruct configuration;
    } table;
} uConfigInEeprom;

typedef union {
    struct
    {
        uint8_t memory[4092]; // 4096 - 4
        uint32_t calCRC;
    } data;
    struct
    {
        CalibrationStruct calibration;
    } table;
} uCalInEeprom;
#pragma pack() // undo pragma pack 1

const uCalInEeprom *pCalInFlash[3] = {(uCalInEeprom *)EEPROM_CAL_ADDR1, (uCalInEeprom *)EEPROM_CAL_ADDR2, (uCalInEeprom *)EEPROM_CAL_ADDR3};
const uConfigInEeprom *pConfigInFlash = (uConfigInEeprom *)EEPROM_CONFIG_ADDR;
static uConfigInEeprom gConfigRamShadow;

/** ***************************************************************************
 * @name _configurationOffset() LOCAL returns address offset of the
         configuration to the start of the flash region
 * @brief
 *
 * @param [in] N/A
 * @retval offset
 ******************************************************************************/
static uint16_t _configurationOffset()
{
    return 2; // 2 bytes from beginning
}

/** ***************************************************************************
 * @name readEEPROMCalOffsetAndLength() LOCAL returns address offset of the
         configuration to the calibration fields
 * @brief
 *
 * @param [in] offset - address offset between the regions
 * @param [in] length - static defined length
 * @retval N/A
 ******************************************************************************/
void readEEPROMCalAddrAndLength(int idx, uint32_t *addr, int *length)
{
    *addr = (uint32_t) & (pCalInFlash[idx]);
    *length = 0xFFE; // 4K - 4
}

/** ***************************************************************************
 * @name readEEPROMCalOffsetAndLength() LOCAL returns address offset of the
         configuration to the calibration fields
 * @brief
 *
 * @param [in] offset - address offset between the regions
 * @param [in] length - static defined length
 * @retval N/A
 ******************************************************************************/
uint8_t *getEEPROMCalPartitionData(int idx, uint16_t *length)
{
    uint8_t *ptr = (uint8_t *)pCalInFlash[idx];
    *length = 4096 - 4; // 4K - 4
    return ptr;
}

#define ERROR 1
#define NO_ERROR 0

static uint8_t writeToEEPROMCal(uint32_t *calCRC)
{
    uint16_t calSize = sizeof(CalibrationStruct);
    uint16_t *data = NULL;
    uint16_t offset = 0;
    uint32_t start;
    HAL_StatusTypeDef status;

    for (uint8_t i = 0; i < NUM_SENSOR_CHIPS; i++)
    {
        start = (uint32_t) pCalInFlash[i];
        offset = 0;
        data = (uint16_t*) &gCalibration[i];

        while (offset < calSize)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, start + offset, *data);
            if (status != HAL_OK)
            {
                return 0;
            }
            offset += 2;
            data++;
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start+CAL_CRC_OFFSET, calCRC[i]);
        if (status != HAL_OK)
        {
            return 0;
        }
    }
    return 1;
}
/** ***************************************************************************
 * @name _s_eepromWrite() LOCAL write 8-bit byte stream into simulated EEPROM.
 *       Since this writes the whole sector, there are no offsets or sizes;
 *       changes should be made to gEepromRamShadow before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (1), no error (0)
 ******************************************************************************/
BOOL _s_eepromWrite(uint8_t erase)
{
    // Only supports writing configuration
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t PageError;
    int16_t num = sizeof(ConfigurationStruct) + 2;
    uint32_t start = (uint32_t)pConfigInFlash;
    uint16_t offset = 0;
    uint32_t *data;
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t calCRC[NUM_SENSOR_CHIPS];

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(0xffff);

    if (erase)
    {
        for (uint8_t i = 0; i < NUM_SENSOR_CHIPS; i++)
        {
            readEEPROMCalibration(i, &gCalibration[i]);
            calCRC[i] = pCalInFlash[i]->data.calCRC;
        }

        pEraseInit.Banks = FLASH_BANK_1;
        pEraseInit.Sector = FLASH_SECTOR_11;
        pEraseInit.NbSectors = 1;
        pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;

        status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
        if (status != HAL_OK)
        {
            EXIT_CRITICAL();
            return ERROR;
        }

        gConfigRamShadow.table.countOfEepromErases++;

        if (!writeToEEPROMCal(calCRC))
        {
            EXIT_CRITICAL();
            return FALSE;
        }
    }

    while (num > 0)
    {
        data = (uint32_t *)&gConfigRamShadow.data.memory[offset];
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start + offset, *data); // stm32fxx_flash.c
        if (status != HAL_OK)
        {
            EXIT_CRITICAL();
            return ERROR;
        }
        offset += 4;
        num -= 4;
    }

    HAL_FLASH_Lock();
    EXIT_CRITICAL();

    return NO_ERROR;
} /* end _s_eepromWrite() */

/** ***************************************************************************
 * @name writeEEPROMByte() write 8-bit byte stream into EEPROM:
 * @brief eeprom is written with addresses that were developed with the xbow
 *       protocol.
 * @param [in] addr - startAdd: offset address to start writing;
 * @param [in] num -  number of the 8-bit bytes
 * @param [in] source *buffer: pointer to the 8-bit byte buffer to be written;
 * @retval error (1), no error (0)
 ******************************************************************************/
BOOL writeEEPROMByte(uint16_t addr, uint16_t num, void *source)
{
    const uint8_t kEraseValue = 0xFF;
    uint8_t *src = (uint8_t *)source;
    uint8_t changed = FALSE;
    uint8_t eraseOnly = TRUE;
    uint16_t maxSize = sizeof(ConfigurationStruct) + 2;
    uint16_t offset = _configurationOffset();
    int i;

    //    PetWatchdog(); // Reloads timer with counter value in the reload register
    memcpy(&gConfigRamShadow, pConfigInFlash, maxSize);

    addr = addr * SIZEOF_WORD;
    offset += addr;

    if ((offset + num) > maxSize)
    {
        return NO_ERROR; // it is ok for it to try to write outside the bounds
        // it is just setting the old EEPROM to values that it shouldn't...
    }

    for (i = 0; i < num; i++)
    {
        if (gConfigRamShadow.data.memory[offset + i] != src[i])
        {
            gConfigRamShadow.data.memory[offset + i] = src[i];
            changed = TRUE;
        }
        if (src[i] != kEraseValue)
        {
            eraseOnly = FALSE;
        }
    }

    if (eraseOnly)
    {
        if (changed)
        {
            memset(&gConfigRamShadow, 0xFF, sizeof(gConfigRamShadow));
        }
        else
        {
            return NO_ERROR;
        }
    }
    /// if something changed, erase flash and write new data

    if (changed)
    {
        return _s_eepromWrite(eraseOnly);
    }
    //    PetWatchdog(); // Reloads timer with counter value in the reload register

    return NO_ERROR;
} /* end writeEEPROMByte() */

/** ***************************************************************************
 * @name writeEEPROMWords() write multiple words into the EEPROM.
 * @brief byte swaps the source before writing to match Nav-View.
 * Trace:
 * [SDD_WRITE_EEPROM_WORDS_01 <-- SRC_WRITE_EEPROM_WORDS]
 * [SDD_WRITE_EEPROM_WORDS_02 <-- SRC_WRITE_EEPROM_WORDS]
 * [SDD_WRITE_EEPROM_WORDS_03 <-- SRC_WRITE_EEPROM_WORDS]
 *
 * @param [in] addr - the xbow 16-bit word EEPROM address to write to.
 * @param [in] num -  the number of 16-bit fields to write.
 * @param [in] source points to the beginning of an array of data to be written.
 * @retval  FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
BOOL writeEEPROMWords(uint16_t addr, uint16_t num, void *source)
{
    BOOL ret;
    int i;
    uint16_t *src;
    uint16_t swap;
    uint16_t addrTemp;
    uint16_t j = 0;    // increment the src array (16-bit byte)
    uint8_t count = 0; // which half of the 16-bit word (which byte) the data
                       // is saved to.
    addrTemp = addr;

    //    PetWatchdog(); // Reloads timer with counter value in the reload register
    // swap bytes
    src = (uint16_t *)source;
    for (i = 0; i < num; i++)
    {
        /// Convoluted logic added to handle the conversion from a 16-bit word to
        ///   an 8-bit byte containing the version string (a char). This applies
        ///   only for the memory locations between 0x0102 and 0x0142. 0x204-0x284
        ///
        /// Note: the way the system loads the initial data onto the unit affects
        ///       if the byte/word needs to be swapped.  For instance, the initial
        ///       configuration is loaded using a 'set-field' command.  Upon a mag-
        ///       align, the unit uses this function to reload the configuration.
        ///       However, the bytes do not get swapped.  The reason that addresses
        ///       between 0x0000 and 0x0100 are not affected by the following if-
        ///       statements.  Any use of the writeEEPROM command may necessitate
        ///       testing to ensure this logic works properly.

        // FIXME: this makes no sense the word offsets need to be muliplied by 2
        // this address range is in the reserved area.
        if ((addrTemp >= 0x0102 && addrTemp < 0x0142))
        {
            // This section of code applies to the version-string part of the EEPROM
            swap = src[i];
            /// Reset the location variables upon entering the versionString
            ///    location in memory
            if (addrTemp == 0x0102)
            {
                count = 0;
                j = i;
            }

            /// If the first byte of a 16-bit word, then shift the byte to the right
            if (count == 0)
            {
                src[j] = ((swap >> 8) & 0xFF); //( swap << 8 );
                count = 1;
            }
            else if (count == 1)
            { // FIXME: this makes no sense you just shifted the upper byter into the lower position
                src[j] = src[j] | swap;
                j++;
                count = 0;
            }
        }
        else if ((addrTemp >= 0x0100 && addrTemp < 0x0102) || addrTemp >= 0x0142)
        {
            swap = src[i];
            swap = (swap << 8) | (swap >> 8); // endianess
            src[i] = swap;
        }
        addrTemp++;
        //        PetWatchdog(); // Reloads timer with counter value in the reload register
    }

    // Does not change (byte swap) data in EEPROM addresses 0x0000 to 0x0102
    ret = writeEEPROMByte(addr, num * SIZEOF_WORD, source);

    return ret;
} /* end writeEEPROMWords() */

/** ***************************************************************************
 * @name readEEPROMWords() wrapper - read multiple-words from the EEPROM.
 * @brief
 * Trace:
 * [SDD_READ_EEPROM_WORDS_01 <-- SRC_READ_EEPROM_WORDS]
 * [SDD_READ_EEPROM_WORDS_02 <-- SRC_READ_EEPROM_WORDS]
 *
 * @param [in] addr - the 16-bit word EEPROM address to read from.
 * @param [in] num -  the number of 16-bit words to write.
 * @param [in] *destination: points to the beginning of an array where the
 *             EEPROM data will be put.
 * @retval  *   FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
void readEEPROMWords(uint16_t addr,
                     uint16_t num,
                     void *destination)
{
    readEEPROMByte(addr,
                   num * SIZEOF_WORD,
                   destination);
} /* end readEEPROMWords() */

/** ***************************************************************************
 * @name readEEPROMByte() read bytes from the EEPROM.
 * @brief
 * Trace:
 * [SDD_READ_EEPROM_TWO_WORDS_01 <-- SRC_READ_EEPROM_TWO_WORDS]
 * [SDD_READ_EEPROM_TWO_WORDS_02 <-- SRC_READ_EEPROM_TWO_WORDS]
 * [SDD_READ_EEPROM_TWO_WORDS_03 <-- SRC_READ_EEPROM_TWO_WORDS]
 *
 * @param [in] addr - the 16-bit word EEPROM address to read from.
 * @param [in] num -  the number of bytes to read.
 * @param [in] *destination: points to the beginning of an array where the
 *             EEPROM data will be put.
 * @param [out] *destination - data in destination
 * @retval    FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
void readEEPROMByte(uint16_t addr, uint16_t num, void *destination)
{
    uint16_t offset = _configurationOffset();
    uint16_t maxSize = sizeof(ConfigurationStruct);
    uint8_t *dst = (uint8_t *)destination;
    uint8_t *src;

    //    PetWatchdog(); // Reloads timer with counter value in the reload register

    addr = addr * SIZEOF_WORD; // 16 bit bytes in 440 to 8 bit bytes here

    memset(dst, 0, num);

    if (addr + num > maxSize)
    {
        return;
    }

    offset += addr;

    src = (uint8_t *)&pConfigInFlash->data.memory[offset];
    while (num > 0)
    {
        // Original code - no swap
        *dst = *src;
        src++;
        dst++;
        num--;
        *dst = *src;
        src++;
        dst++;
        num--;
        //        PetWatchdog(); // Reloads timer with counter value in the reload register
    }
} /*end readEEPROMByte() */

/** ***************************************************************************
 * @name readEEPROMCal() read bytes from the EEPROM cal partition.
 * @brief
 * Trace:
 * [SDD_READ_EEPROM_TWO_WORDS_01 <-- SRC_READ_EEPROM_TWO_WORDS]
 * [SDD_READ_EEPROM_TWO_WORDS_02 <-- SRC_READ_EEPROM_TWO_WORDS]
 * [SDD_READ_EEPROM_TWO_WORDS_03 <-- SRC_READ_EEPROM_TWO_WORDS]
 *
 * @param [in] offset  - the 16-bit offset from EEPROM start.
 * @param [in] num     - the number of bytes to read.
 * @param [in] *destination: points to the beginning of an array where the
 *             EEPROM data will be put.
 * @param [out] *destination - data in destination
 * @retval    FALSE: writing fails; TRUE: writing succeeds.
 ******************************************************************************/
BOOL readFromEEPROMCalPartition(uint16_t offset, uint16_t num, void *destination)
{
    uint8_t *start = (uint8_t *)EEPROM_CAL_ADDR1;
    uint8_t *dst = (uint8_t *)destination;
    uint8_t *src;

    //    PetWatchdog(); // Reloads timer with counter value in the reload register

    if ((int)(start + num) > EEPROM_LAST_ADDR)
    {
        return FALSE;
    }

    src = start + offset;
    memcpy(dst, src, num);
    return TRUE;

} /*end readEEPROMRaw*/

static uint8_t writeToEEPROMConfiguration(void)
{
    int16_t num = sizeof(ConfigurationStruct) + 2;
    uint32_t *data = (uint32_t *) &gConfigRamShadow;
    uint16_t offset = 0;
    uint32_t start = (uint32_t)pConfigInFlash;

    HAL_StatusTypeDef status;

    while (num > 0)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start + offset, *data);
        if (status != HAL_OK)
        {
            return 0;
        }
        data++;
        offset += 4;
        num -= 4;
    }
    return 1;
}
/** ***************************************************************************
 * @name writeEEPROMCal() write 8-bit byte stream into EEPROM:
 * @brief eeprom is written with addresses that were developed with the xbow
 *       protocol.
 * @param [in] addr - startAdd: offset address to start writing;
 * @param [in] num -  number of the 8-bit bytes
 * @param [in] source *buffer: pointer to the 8-bit byte buffer to be written;
 * @retval error (1), no error (0)
 ******************************************************************************/
BOOL writeToEEPROMCalPartition(uint16_t offset, uint16_t num, void *source)
{
    uint8_t *start = (uint8_t *)EEPROM_CAL_ADDR1;
    uint32_t *data = (uint32_t *)source;
    uint32_t PageError;

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef pEraseInit;

    if ((int)(num + start) > EEPROM_LAST_ADDR || (offset % 8 != 0)) //only offset even to 8 bytes
    { 
        return FALSE;
    }

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(0xffff);

    if (offset == 0)
    {
        memcpy(&gConfigRamShadow, pConfigInFlash, sizeof(ConfigurationStruct) + 2);

        pEraseInit.Banks = FLASH_BANK_1;
        pEraseInit.Sector = FLASH_SECTOR_11;
        pEraseInit.NbSectors = 1;
        pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;

        status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
        if (status != HAL_OK)
        {
            EXIT_CRITICAL();
            return FALSE;
        }

        if (!writeToEEPROMConfiguration())
        {
            EXIT_CRITICAL();
            return FALSE;
        }
    }

    while (num > 0)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)start + offset, *data);
        if (status != HAL_OK)
        {
            break;
        }
        data++;
        offset += 4;
        num -= 4;
    }

    EXIT_CRITICAL();
    if (status != HAL_OK)
    {
        return FALSE;
    }

    return TRUE;
} /* end writeEEPROMCal() */

/** ***************************************************************************
 * @name writeEEPROMApp() write 8-bit byte stream into EEPROM:
 * @brief eeprom is written with addresses that were developed with the xbow
 *       protocol.
 * @param [in] addr - startAdd: offset address to start writing;
 * @param [in] num -  number of the 8-bit bytes
 * @param [in] source *buffer: pointer to the 8-bit byte buffer to be written;
 * @retval error (1), no error (0)
 ******************************************************************************/
BOOL writeToEEPROMAppPartition(uint32_t offset, uint16_t num, void *source)
{
    //add eeprom delete
    //    uint8_t     *start  = (uint8_t*)APP_START_ADDR;
    //     uint32_t    dst;
    //     uint8_t     *src    = (uint8_t*) source;
    //     int         pageNum;
    //     int         pageOffset;
    //     uint32_t    PageError;
    //     uint64_t    data;

    //     HAL_StatusTypeDef status;
    //     FLASH_EraseInitTypeDef pEraseInit;

    //     if(num%8 != 0){
    //         num = (num | 0x07) + 1;
    //     }

    //     if ((int)(offset + num) > APP_MAX_SIZE || (offset %8 != 0)){   //only offset even to 8 bytes
    //         return FALSE;
    //     }

    //     pageNum    = offset/FLASH_PAGE_SIZE + EEPROM_APP_PAGE1;  // EEPROM takes 4 pages
    //     pageOffset = offset%FLASH_PAGE_SIZE;                     // offset inside page

    //     ENTER_CRITICAL();

    //     status = HAL_FLASH_Unlock();
    //     __HAL_FLASH_CLEAR_FLAG(0xffff);

    //     if(!pageOffset){
    //         // erase page here
    //         pEraseInit.Banks     = FLASH_BANK_1;
    //         pEraseInit.NbPages   = 1;
    //         pEraseInit.Page      = pageNum;
    //         pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    //         status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    //         if(status != HAL_OK){
    //             EXIT_CRITICAL();
    //             return FALSE;
    //         }
    //     }

    //    dst  = APP_START_ADDR + offset;

    //    while (num > 0) {
    //         data    = 0;
    //         memcpy((uint8_t*)&data, src,8);
    //         if((dst != APP_SIGNATURE_ADDR1) && (dst != APP_SIGNATURE_ADDR2)){
    //             status  = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst, data); // stm32fxx_flash.c
    //             if(status != HAL_OK){
    //                 break;
    //             }
    //         }
    //         src    += 8;
    //         num    -= 8;
    //         dst    += 8;
    //         offset += 8;
    //         if(num > 0 ){
    //             pageOffset = offset%FLASH_PAGE_SIZE;                       // check for new page
    //             if (!pageOffset)
    //             {
    //                 pageNum  = offset/FLASH_PAGE_SIZE + EEPROM_APP_PAGE1;
    //                 // erase page here
    //                 pEraseInit.Banks     = FLASH_BANK_1;
    //                 pEraseInit.NbPages   = 1;
    //                 pEraseInit.Page      = pageNum;
    //                 pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    //                 status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    //                 if (status != HAL_OK)
    //                 {
    //                     break;
    //                 }
    //             }
    //         }
    //     }

    //     EXIT_CRITICAL();
    //     if(status != HAL_OK){
    //         return FALSE;
    //     }

    // //    PetWatchdog(); // Reloads timer with counter value in the reload register

    return TRUE;
} /* end writeEEPROMCal() */

// individual getters
/** ***************************************************************************
 * @name readEEPROMSerialNumber() read serial number from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to return data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMSerialNumber(void *destination)
{
    uint32_t *sn;
    sn = destination;
    *sn = pCalInFlash[0]->table.calibration.serialNumber;
}

/** ***************************************************************************
 * @name readEEPROMProdConfig() read product configuration from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to return data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMProdConfig(void *destination)
{
    union ProductConfiguration *pc;
    pc = destination;
    *pc = pCalInFlash[0]->table.calibration.productConfiguration;
}

/** ***************************************************************************
 * @name readEEPROMCalibration() read calibration from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to return data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMCalibration(int idx, void *destination)
{
    //    volatile int eepromSize = sizeof(uEeprom);
    memcpy(destination, pCalInFlash[idx], sizeof(CalibrationStruct));
}

uint8_t *getEEPROMCalTabPtr(int idx)
{
    return (uint8_t *)pCalInFlash[idx];
}

/** ***************************************************************************
 * @name readEEPROMConfiguration() read configuration from the EEPROM.
 *
 * @param [out] *destination - pointer to buffer to retuen data to
 * @retval  N/A
 ******************************************************************************/
void readEEPROMConfiguration(void *destination)
{
    //    PetWatchdog(); // Reloads timer with counter value in the reload register
    memcpy(destination, &pConfigInFlash->table.configuration, sizeof(ConfigurationStruct));
    //    PetWatchdog(); // kick the dog
}

/** ***************************************************************************
 * @name SetBORLevel() set the BOR level to desired threshold.
 *
 * @param [dat] 
 * @retval  N/A
 ******************************************************************************/
/*
TODO 
void setBORLevel(float voltage)
{
    uint32_t status;
    uint8_t  level;
    FLASH_OBProgramInitTypeDef *pOBInit;
        
    if(voltage < 2.1){
        level = OB_BOR_OFF;
    }else if(voltage < 2.4){
        level = OB_BOR_LEVEL1;
    }else if(voltage < 2.7){
        level = OB_BOR_LEVEL2;
    }else{
        level = OB_BOR_LEVEL3;
    }
}
*/
//add eeprom delete

// BOOL calSectorsLocked(void)
// {
//     FLASH_OBProgramInitTypeDef tmp;
//     tmp.WRPArea = OB_WRPAREA_BANK1_AREAA;
//     HAL_FLASHEx_OBGetConfig(&tmp);
//     if(tmp.WRPStartOffset == 0x3c && tmp.WRPEndOffset == 0x40){
//         return TRUE;
//     }
//     return FALSE;
// }

// // enable write protection for xbow config sector
// BOOL lockCalSectors(void)
// {
//     FLASH_OBProgramInitTypeDef tmp;
//     HAL_StatusTypeDef res;
//     if(calSectorsLocked()){
//         return TRUE;
//     }
//     ENTER_CRITICAL();
//     tmp.WRPArea        = OB_WRPAREA_BANK1_AREAA;
//     tmp.OptionType     = OPTIONBYTE_WRP;
//     tmp.WRPStartOffset = 0x3c;
//     tmp.WRPEndOffset   = 0x40;
//     res = HAL_FLASHEx_OBProgram(&tmp);
//     EXIT_CRITICAL();

//     if(res != HAL_OK){
//         // protection errror
//         return FALSE;
//     }
//     return TRUE;
// }

// // disable write protection for xbow config sector
// BOOL unlockCalSectors(void)
// {
//     FLASH_OBProgramInitTypeDef tmp;
//     HAL_StatusTypeDef res;
//     if(!calSectorsLocked()){
//         return TRUE;
//     }
//     ENTER_CRITICAL();
//     tmp.WRPArea        = OB_WRPAREA_BANK1_AREAA;
//     tmp.OptionType     = OPTIONBYTE_WRP;
//     tmp.WRPStartOffset = 0xFF;
//     tmp.WRPEndOffset   = 0x00;
//     res = HAL_FLASHEx_OBProgram(&tmp);
//     EXIT_CRITICAL();

//     if(res != HAL_OK){
//         // protection errror
//         return FALSE;
//     }
//     return TRUE;
// }

/** ***************************************************************************
 * @name validateUserConfigInEeprom - validating of user configuration structure 
 *       in EEPROM
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] userConfigSize - pointer to variable, which initialized with the
 *                              size of user configuration structure
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL validateUserConfigInEeprom(int *userConfigSize)
{
    uint64_t crc, configCrc, size;
    uint64_t *dataPtr = (uint64_t *)EEPROM_USER_ADDR;

    configCrc = dataPtr[0];        // CRC
    size = dataPtr[1];             // Total Number of bytes in user config structure in eeprom
    if (size > EEPROM_PAGE_SIZE || // check for max size in flash
        size != *userConfigSize)
    { // check if image fits into user storage in RAM
        return FALSE;
    }
    crc = CalculateCRC((uint8_t *)dataPtr + 8, size - 8);
    if (crc == configCrc)
    {
        *userConfigSize = size;
        return TRUE;
    }
    return FALSE; //
}

/** ***************************************************************************
 * @name loadUserConfigInEeprom - loading user configuration structure from 
 *       predefined flash sector
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL loadUserConfigFromEeprom(uint8_t *ptrToUserConfigInRam, int *userConfigSize)
{
    memcpy(ptrToUserConfigInRam, (uint8_t *)EEPROM_USER_ADDR, *userConfigSize);
    return TRUE;
}

/** ***************************************************************************
 * @name saveUserConfigInEeprom - saving of user configuration structure un the 
 *       predefined flash sector
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
//add eeprom delete

// BOOL  saveUserConfigInEeprom(uint8_t *ptrToUserConfigStruct, int userConfigStructLen)
// {
//     uint16_t    offset   = 0;
//     uint16_t    num      =  userConfigStructLen;
//     uint64_t   *paramPtr =  (uint64_t*)ptrToUserConfigStruct;
//     uint32_t   pError;
//     uint64_t   data;

//     if(userConfigStructLen > EEPROM_PAGE_SIZE){
//         return FALSE;
//     }

//     if((num & 0x07) != 0){
//         num = (num | 0x07) + 1; // make it even to 8
//     }

//     paramPtr[1] = num;  //  Total size of user config structure, including Crc and data size
//     paramPtr[0] = CalculateCRC((uint8_t*)ptrToUserConfigStruct + 8, num - 8);
//     // calculate CRC over user configuration structure

//     uint32_t    dst;
//     uint8_t     *src     = (uint8_t*)ptrToUserConfigStruct;
//     int         pageNum  = EEPROM_USER_PAGE;

//     HAL_StatusTypeDef      status;
//     FLASH_EraseInitTypeDef pEraseInit;

//     ENTER_CRITICAL();

//     status = HAL_FLASH_Unlock();
//     __HAL_FLASH_CLEAR_FLAG(0xffff);

//     // erase page here
//     pEraseInit.Banks     = FLASH_BANK_1;
//     pEraseInit.NbPages   = 1;
//     pEraseInit.Page      = pageNum;
//     pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
//     status = HAL_FLASHEx_Erase(&pEraseInit, &pError);
//     if(status != HAL_OK){
//         EXIT_CRITICAL();
//         return FALSE;
//     }

//    dst = EEPROM_USER_ADDR;

//    while (num > 0) {
//         memcpy((uint8_t*)&data, src,8);
//         src+=8;
//         status  = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst, data); // stm32fxx_flash.c
//         if(status != HAL_OK){
//             break;
//         }
//         num    -= 8;
//         dst    += 8;
//         offset += 8;
//     }

//     EXIT_CRITICAL();

//     if(status != HAL_OK){
//         return FALSE;
//     }
//     return TRUE;
// } /* saveUserConfigInEeprom */

/** ***************************************************************************
 * @name  applyAppSignature() marking application as ready
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL ApplyAppSignature(BOOL bootMode)
{
    HAL_StatusTypeDef status;
    uint64_t sig = 0xFFFFFFFFFFFFFFFFLL;
    uint32_t s1 = *(uint32_t *)APP_SIGNATURE_ADDR1;
    uint32_t s2 = *(uint32_t *)APP_SIGNATURE_ADDR2;
    uint32_t stackPtr = *(uint32_t *)APP_START_ADDR;
    uint32_t *sptr = (uint32_t *)&sig;

    // rudimental sanity check

    if ((stackPtr & 0xFFFF0000) != 0x20000000)
    {
        // APP is not there
        return FALSE;
    }

    if (bootMode)
    {
        if (s1 == APP_SIGNATURE)
        {
            return TRUE;
        }
        if (s1 != 0xFFFFFFFF)
        {
            return FALSE;
        }
    }
    else
    {
        if (s2 == APP_SIGNATURE)
        {
            return TRUE;
        }
        if (s2 != 0xFFFFFFFF)
        {
            return FALSE;
        }
    }

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();

    if (status != HAL_OK)
    {
        return FALSE;
    }

    __HAL_FLASH_CLEAR_FLAG(0xffff);

    sptr[0] = APP_SIGNATURE;

    if (bootMode)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_SIGNATURE_ADDR1, sig);
    }
    else
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_SIGNATURE_ADDR2, sig);
    }

    EXIT_CRITICAL();

    if (status != HAL_OK)
    {
        return FALSE;
    }

    return TRUE;
}

BOOL appStartedFirstTime(void)
{
    uint32_t *sigPtr2 = (uint32_t *)APP_SIGNATURE_ADDR2;

    if (!calSectorsLocked())
    {
        return FALSE;
    }

    if (*sigPtr2 == 0xFFFFFFFF)
    {
        return ApplyAppSignature(FALSE);
    }

    return FALSE;
}

const uint32_t bootSig[] = {
    0x15422764,
    0x21263548,
    0x37364888,
    0x03208807};

const uint32_t appSig[] = {
    0x04091962,
    0x83201501,
    0x13208807,
    0x67380090};

const uint32_t testSig[] = {
    0x12568564,
    0x95843513,
    0x85698521,
    0x34589667};

BOOL EEPROM_SaveBootFlag(uint8_t *ptrToUserConfigStruct, int userConfigStructLen)
{
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t    PageError;
    uint16_t    offset   =  0;
    uint16_t    num      =  userConfigStructLen;
    uint32_t    start    =  (uint32_t)pBootFlagFlash;
    uint32_t   *dataPtr  =  (uint32_t*)ptrToUserConfigStruct;
    uint64_t   *paramPtr =  (uint64_t*)ptrToUserConfigStruct;

    paramPtr[1] = num;  //  Total size of user config structure, including Crc and data size 
    paramPtr[0] = CalculateCRC((uint8_t*)ptrToUserConfigStruct + 8, num - 8);
    // calculate CRC over user configuration structure

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(0xffff);

	pEraseInit.Banks = FLASH_BANK_1;
	pEraseInit.Sector = FLASH_SECTOR_9;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;

	status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	if (status != HAL_OK)
	{
		EXIT_CRITICAL();
		return FALSE;
	}
	while (num > 0)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start + offset, *dataPtr++);
        if (status != HAL_OK)
        {
            EXIT_CRITICAL();
            return FALSE;
        }
        offset += 4;
        num -= 4;
    }
    HAL_FLASH_Lock();
    EXIT_CRITICAL();
    return TRUE;
}


BOOL  SaveBootFlag(void)
{
    int size;
    BOOL status;

    size   = sizeof(bootSig);
    status = EEPROM_SaveBootFlag((uint8_t *)&bootSig, size);

    if(status){
        return TRUE; 
    }

    return FALSE;
}

BOOL  SaveAppFlag(void)
{
    int size;
    BOOL status;

    size   = sizeof(appSig);
    status = EEPROM_SaveBootFlag((uint8_t *)&appSig, size);

    if(status){
        return TRUE; 
    }

    return FALSE;
}


BOOL HW_IsBootModeEnforced()
{
    uint32_t *ptr = (uint32_t *)BOOT_SIGNATURE_ADDR;
    for (int i = 0; i < 4; i++)
    {
        if (*ptr++ != bootSig[i])
        {
            return FALSE;
        }
    }
    return true;
}
BOOL  SaveBootFlag(void);
void HW_EnforceBootMode()
{
    uint32_t *ptr = (uint32_t *)BOOT_SIGNATURE_ADDR;
    for (int i = 0; i < 4; i++)
    {
        *ptr++ = bootSig[i];
    }
    SaveBootFlag();
}

BOOL HW_IsAppModeEnforced()
{
    uint32_t *ptr = (uint32_t *)BOOT_SIGNATURE_ADDR;
    for (int i = 0; i < 4; i++)
    {
        if (*ptr++ != appSig[i])
        {
            return FALSE;
        }
    }
    return true;
}

void HW_EnforceAppMode()
{
    uint32_t *ptr = (uint32_t *)BOOT_SIGNATURE_ADDR;
    for (int i = 0; i < 4; i++)
    {
        *ptr++ = appSig[i];
    }
}

void HW_HDTestMode()
{
    uint32_t *ptr = (uint32_t *)BOOT_SIGNATURE_ADDR;
    for (int i = 0; i < 4; i++)
    {
        *ptr++ = testSig[i];
    }
}

void HW_ClearBootSignature()
{
    uint32_t *ptr = (uint32_t *)BOOT_SIGNATURE_ADDR;
    for (int i = 0; i < 4; i++)
    {
        *ptr++ = 0;
    }
}

bool IsNeedToHardwareTest()
{
    bool need = FALSE;
    uint32_t stackPtr = *((uint32_t *)BOOT_SIGNATURE_ADDR);
    
    if (stackPtr == 0x12568564)
    {
        need = TRUE;
    }
    *((uint32_t *)BOOT_SIGNATURE_ADDR) = 0;
    return need;
}

BOOL needUpdate = FALSE, forcedBootMode = FALSE, spiBootMode = FALSE;

BOOL IsNeedToUpdateApp()
{
    BOOL need = FALSE;
    //uint32_t data = *((uint32_t *)APP_SIGNATURE_ADDR1);
    uint32_t stackPtr = *((uint32_t *)APP_SIGNATURE_ADDR);
#if 0
    if (data != APP_SIGNATURE)
    {
        need = TRUE;
    }
#endif
    //if (stackPtr != 0x2000FF00)
    if (stackPtr == 0x15422764)
    {
        need = TRUE;
    }

    if (fSPI)
    {
        spiBootMode = true;
    }

    return need;
}


BOOL writeFlash(uint32_t addr,uint8_t *buf, uint16_t len)
{
    uint32_t    dst;
    uint8_t     *src    = (uint8_t*) buf;
    uint32_t    PageError;
    uint32_t    data;
    int num   = len;

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef pEraseInit;

    if(len%4 != 0){
        len = (len | 0x7) + 1;  //
        while(1);
    }
//    PetWatchdog(); // Reloads timer with counter value in the reload register

    if ((int)(addr + len) > EEPROM_LAST_ADDR){  
        return FALSE;
    }

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(0xffff); 
    
    if(addr == 0){
        // erase sectors here
#if 0
        pEraseInit.Banks     = FLASH_BANK_1;
        pEraseInit.NbSectors = 4;                   // 512K
        pEraseInit.Sector    = APP_START_SECTOR;
        pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
#endif

        pEraseInit.Banks = FLASH_BANK_1;
        pEraseInit.Sector = FLASH_SECTOR_4;
        pEraseInit.NbSectors = 5;           //5
        pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;

        status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
        if(status != HAL_OK){
            EXIT_CRITICAL();
            return FALSE;
        }
    }

    //dst   = APP_START_SECTOR + addr;         //TODO:
    dst = APP_START_ADDR + addr;
    while (num > 0) {
        data    = 0;
        memcpy((uint8_t*)&data, src, 4);
        if(dst != APP_SIGNATURE_ADDR){
            status  = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dst, data); 
            if(status != HAL_OK){
                break;
            }
        }
        src    += 4;
        dst    += 4;
        num    -= 4;
//        PetWatchdog(); // Reloads timer with counter value in the reload register
    }

    EXIT_CRITICAL();
    if(status != HAL_OK){
        return FALSE;
    }

//    PetWatchdog(); // Reloads timer with counter value in the reload register

    return TRUE;
}