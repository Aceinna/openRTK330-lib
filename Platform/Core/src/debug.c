 /** ***************************************************************************
 * @file   debug.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  This set of functions formats and sends ASCII messages out USART DEBUG
 *        serial to a serial console commandLine recives serial messages from the
 *        console.
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/


#include "debug.h"
#include "utilities.h" // for itoa
#include "GlobalConstants.h"
#include "platformAPI.h"
#include "uart.h"
#include "string.h"

#define _CR    0x0D
#define _LF    0x0A
#define	_BELL  0x07
#define _TAB   0x09
#define _SPACE 0x20
#define _BS    0x08
#define _DEL   0x7F

//#define PORT_DBG_RX_BUF_SIZE COM_BUF_SIZE // 512
//#define PORT_DBG_TX_BUF_SIZE COM_BUF_SIZE




// local version
void itoa(int value, char s[], int base, BOOL notSigned);
void itoa_64bit(int64_t value, char s[], int base);

//int debugSerialChan = UART_CHANNEL_2;   // defaul channel

// void  InitDebugSerialCommunication(uint32_t baudrate)
// {
//     debugSerialChan = platformGetSerialChannel(0);
// #ifndef CAN_BUS_COMM
//     uart_init(debugSerialChan, baudrate);
// #endif
// }


/** ****************************************************************************
 * @name DebugSerialPutChar
 * @brief Write character out Serial Port.
 * @param [In] character
 * @retval character
 *******************************************************************************/
unsigned char DebugPutChar (unsigned char c)
{ 
#ifdef DEVICE_DEBUG
    uart_write_bytes(UART_DEBUG, &c1, 1, 1);
#endif
    return c;
}


/** ****************************************************************************
 * @name DebugSerialReadLine
 * @brief Read a line of characters from a serial terminal Will block until a
 *        complete line is read.
 *        Offers terminal-like editing support:
 *                BS(\x08) or DEL(\x7F) delete previous character
 *                TAB(\t) replace by single space
 *                CR(\r) expanded to  CR+LF
 * @param [In] buf - buffer for storing the line
 * @param [In] *index - pointer to current location in the buffer
 * @param [In] len - total length of the buffer
 * @param [Out] *index should be passed in, same as it was before
 * @retval TRUE if _LF is the last character, FALSE otherwise
 *******************************************************************************/
int DebugReadLine(uint8_t  *buf, uint32_t *index, uint32_t len)
{
    uint8_t c = 0, lf = 0;
    // int num;

    while (c != _LF){
        //num = uart_read(debugSerialChan, &c, 1);
        // if(num <= 0){
        //     break;
        // }
        
        if (_TAB == c) {
            c = _SPACE;
        }
        
        if (_CR == c) {
            continue;
        }

        /// Handle special character
        switch (c) {
        case _BS:
        case _DEL:
            if (*index > 0) {
                DebugPutChar(_BS);
                DebugPutChar(_SPACE);
                DebugPutChar(_BS);
                (*index)--;
            }
            break;
        case _CR:
            c = _LF; 
            DebugPutChar(c);
            buf[*index] = c;
            (*index)++;
            lf++;
            break;
        case _LF:
            DebugPutChar(c);
            buf[*index] = c;
            *index = 0;
            lf++;
            break;
        default:
            /// Only keep printable characters
            if ((c >= 0x20) && (c <= 0x7E)) {
                /// check for room in the buffer, saving two characters for CR+LF
                if (*index < (len - 2)) {
                     buf[*index] = c;
                    (*index)++;
                    DebugPutChar(c);
                } else { /// buffer full, send BELL
                    DebugPutChar(_BELL);
                }
            }
        } //end switch
    } // end while
 return (c == _LF);
}

/** ***************************************************************************
 * @name DebugPrintString()
 * @brief sequence through a string one character at a time and send each out
 * the DEBUG serial
 *
 * @param [in] str - the data to send out
 * @retval success = 0
 ******************************************************************************/
void DebugPrintString(const char * str)
{
#ifdef DEVICE_DEBUG
    uart_write_bytes(UART_DEBUG, str, strlen(str), 1);
#endif
}

/** ***************************************************************************
 * @name DebugPrintInt()
 * @brief combine a string and integer then convert the integer to a string and
 * send out the DEBUG serial
 *
 * @param [in] str - string data to send out in front of the integer
 * @param [in] i - integer to convert to a string in decimal format then send out
 * @retval success = 0
 ******************************************************************************/
void DebugPrintInt(const char *s,
                   int        i)
{
    /** 32 bit int: -2147483646
        digits      01234567890 + NULL */
    char numberString[11];

    DebugPrintString(s);
    itoa(i, numberString, 10, FALSE); ///< base 10
    DebugPrintString(numberString);
}

void DebugPrintUInt(const char *s,
                    uint32_t    i)
{
    /** 32 bit int: -2147483646
        digits      01234567890 + NULL */
    char numberString[11];

    DebugPrintString(s);
    itoa(i, numberString, 10, TRUE); ///< base 10
    DebugPrintString(numberString);
}


void DebugPrintLongInt( const char *s, int64_t i )
{
    /* 64 bit int: 9,223,372,036,854,780,000 */
    /* digits      01234567890 + NULL */
    char numberString[20];
    DebugPrintString(s);
    itoa_64bit(i, numberString, 10); // base 10
    DebugPrintString(numberString);
}

/** ***************************************************************************
 * @name DebugPrintHex()
 * @brief combine a string and integer then convert the integer to a hex string
 * and send out the DEBUG serial. Does NOT add a '0x'
 *
 * @param [in] str - string data to send out in front of the integer
 * @param [in] i - integer to convert to a hex base string then send out
 * @retval success = 0
 ******************************************************************************/
void DebugPrintHex(const char *s,
                   int         i)
{
    /** 32 bit int: FFFFFFFF
        digits      01234567 + NULL */
    char numberString[9];

    DebugPrintString(s);
    itoa(i, numberString, 16, FALSE); ///< base 16
    DebugPrintString(numberString);

}

/** ***************************************************************************
 * @name DebugPrintFloat()
 * @brief combine a string and float then convert the float to a series of
 * digits including the - sign then send out the DEBUG serial.
 *
 * @param [in] str - string data to send out in front of the integer
 * @param [in] f - floating point number to convert to a string then send out
 * @param [in] signbits - flag to indicat a + or - number
 * @retval success = 0
 ******************************************************************************/
void DebugPrintFloat(const char *s,
                     float      f,
                     int        sigDigits)
{
    char numberString[11];
    int i, n;

    DebugPrintString(s);
    i = (int) f; ///< just get the number to the left of the decimal
    n = i;
    if (f < 0) {
        n = -n;
        DebugPrintString("-");
    }else{
        DebugPrintString(" ");
    }
    DebugPrintInt("", n);

    /// now get the number of significant digits to the right
    f = f - i;
    if (f < 0) {
      f = -f;
    }
    if (sigDigits > (sizeof(numberString) -1) ) {
        sigDigits = sizeof(numberString) -1;
    }

    DebugPrintString(".");
    while (sigDigits) {
        f *= 10;
        sigDigits--;
        i = (int) f;
        DebugPrintInt("", i);
        f = f - i;
    }
}

/** ***************************************************************************
 * @name DebugPrintEndLine()
 * @brief send a ascii return and newline out to a serial debug console
 * via the DEBUG serial.
 *
 * @param [in] N/A
 * @retval success = 0
 ******************************************************************************/
void DebugPrintEndline()
{
    DebugPrintString("\r\n");
}

#define MAX_INT32_STRING 11 // 10 digits plus a NULL
#define RADIX_DECIMAL    10
/** ***************************************************************************
 * @name itoa()
 * @brief local version of integer to ascii
 *
 * @param [in] value - integer to convert
 * @param [in] sp - output buffer
 * @param [in] radix - number base default base 10
 * @retval N/A
 ******************************************************************************/
void itoa(int  value, char *sp, int  radix, BOOL notSigned)
{
    char     tmp[MAX_INT32_STRING]; // [11]
    char     *tp = tmp;
    int      i;
    unsigned v;
    int      sign = 0;

    if(notSigned){
        v = (unsigned)value;
    }else{
    sign = (radix == RADIX_DECIMAL && value < 0);
    if (sign) {
        v = -value;
    } else {
        v = (unsigned)value;
        }
    }

    while (v || tp == tmp)
    {
        i = v % radix;
        v /= radix;
        if (i < RADIX_DECIMAL) {
          *tp++ = i+'0';
        } else {
          *tp++ = i + 'A' - RADIX_DECIMAL;
        }
    }

    if (radix != RADIX_DECIMAL) { ///< zero fill non decimal values
        while (tp < &(tmp[4])) {
          *tp++ = '0';
        }
    }

    if (sign) { *sp++ = '-'; }
    /// reverse and put in output buffer
    while (tp > tmp) {
        tp--;
        *sp = *tp;
        sp++;
    }
    *sp = '\0';
}

#define MAX_INT64_STRING 20 // 10 digits plus a NULL

void itoa_64bit( int64_t value, char *sp, int radix )
{
    char tmp[MAX_INT64_STRING];
    char *tp = tmp;
    int i;
    uint64_t v;
    int sign;

    sign = ( (radix == RADIX_DECIMAL) && (value < 0) );
    if (sign) {
        v = (uint64_t)( -value );
    } else {
        v = (uint64_t)value;
    }

    while( v || tp == tmp )
    {
        i = v % radix;
        v /= radix;
        if (i < RADIX_DECIMAL) {
          *tp++ = i+'0';
        } else {
          *tp++ = i + 'A' - RADIX_DECIMAL;
        }
    }

    if (radix != RADIX_DECIMAL) { // zero fill non decimal values
        while (tp < &(tmp[4])) {
          *tp++ = '0';
        }
    }

    if (sign) { *sp++ = '-'; }
    // reverse and put in output buffer
    while (tp > tmp) {
        tp--;
        *sp = *tp;
        sp++;
    }
    *sp = '\0';

}

static char buffer[500];

int  tprintf(char *format, ...)
{
  int len;
  va_list args;
  va_start (args, format);
  len = vsprintf (buffer,format, args);
  va_end (args);
  DebugPrintString(buffer);

  return len;
}

