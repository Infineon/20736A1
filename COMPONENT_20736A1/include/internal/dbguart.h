/*******************************************************************
*
* Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*
********************************************************************
*
********************************************************************
*    File Name: uart.h
*
*    Abstract: UART device interface declarations. Supports USB/HCI
*               API
*
*    $History:$
*
********************************************************************
*/

#ifndef __DBGUART_H__
#define __DBGUART_H__

#include <types.h>

#ifdef __cplusplus
extern "C" {
#endif

extern BOOL8 traceEnable;

#define DEFAULT_DBGUART_BAUDRATE 115200

#ifdef BT_DEBUG_UART

#if defined(DUP_ENABLED)


// debug uart protocol
// packet format:
// UINT8    magic_number; (0xFE)
// UINT8    packet_type;
//              1 -- HCI_CMD
//              2 -- HCI_ACL_TO_HOST
//              3 -- HCI_SCO_TO_HOST
//              4 -- HCI_EVENT
//              7 -- DIAG
//              8 -- string
//              9 -- memory profile
//              0x82 -- HCI_ACL_FROM_HOST
//              0x83 -- HCI_SCO_FROM_HOST
//              0xa -- DUP_CMD (from host)
//              0xa -- DUP_RSP (to host)
//              others -- reserved
// UINT16   length;
// UINT8    payload[length];

#define DUP_MAGIC                   (0xFE)
enum
{
    DUP_HCI_CMD = 1,
    DUP_HCI_ACL_TO_HOST = 2,
    DUP_HCI_ACL_FROM_HOST = (DUP_HCI_ACL_TO_HOST | 0x80),
    DUP_HCI_SCO_TO_HOST = 3,
    DUP_HCI_SCO_FROM_HOST = (DUP_HCI_SCO_TO_HOST | 0x80),
    DUP_HCI_EVENT = 4,
    DUP_REQ = 5,
    DUP_RSP = 6,
    DUP_DIAG = 7,
    DUP_HCI_DIAG_FROM_HOST = 7,
    DUP_HCI_DIAG_TO_HOST = (DUP_HCI_DIAG_FROM_HOST | 0x80),
    DUP_STRING = 8,
    DUP_MEMPROFILE = 9,
};

// DUP_REQ payload format:
// UINT8 reqOpCode;
//              1 -- memory peek
//              2 -- memory poke
//              others -- reserved
// variable data depending on the reqOpCode

enum
{
    DUP_REQ_OP_MEM_PEEK = 1,
    DUP_REQ_OP_MEM_POKE = 2,
};

enum
{
    DUP_RSP_STATUS_OK = 0,
    DUP_RSP_STATUS_FAILED = 1,
};

#pragma pack(1)

typedef PACKED struct
{
    UINT8   magic;
    UINT8   pktType;
    UINT16  len;
} DUP_HEADER_TYPE;

typedef PACKED struct
{
    UINT8   magic;
    UINT8   pktType;
    UINT16  len;
    UINT8   opCode;
} DUP_REQ_HEADER_TYPE;

typedef PACKED struct
{
    DUP_REQ_HEADER_TYPE header;
    UINT32 addr;
    UINT8 len;
}DUP_REQ_MEM_PEEK_TYPE;

typedef PACKED struct
{
    DUP_REQ_HEADER_TYPE header;
    UINT32 addr;
    UINT8 len;
    UINT8 data[64];
}DUP_REQ_MEM_POKE_TYPE;

typedef PACKED union
{
    DUP_REQ_HEADER_TYPE header;
    DUP_REQ_MEM_PEEK_TYPE memPeek;
    DUP_REQ_MEM_POKE_TYPE memPoke;
} DUP_REQ_TYPE;

typedef PACKED struct
{
    UINT8 status;
    UINT8 len;
    UINT8 data[64];
} DUP_RSP_MEM_PEEK_TYPE;

typedef PACKED struct
{
    UINT8 status;
    UINT8 len;
} DUP_RSP_MEM_POKE_TYPE;

#pragma pack()

UINT32  dbguart_logDupPkt(void* buffer, UINT32 length, UINT32 type);
void    dbguart_poll(void);

#else

#define dbguart_logDupPkt(buffer, length, type)
#define dbguart_poll()

#endif // DUP_ENABLED


void    dbguart_init(char* string, UINT32 baudrate);
#if (defined(BB_2046B0) || defined(BB_2046B1) || defined(BB_2070A0) || defined(BB_2072) || defined(BT_DEBUG_WRITE_BYTE))
UINT32  dbguart_write(UINT8 byte);
#else
UINT32  dbguart_write(UINT32 dword);
#endif
UINT32  dbguart_read(UINT8* c);
void    uart_trace_write(UINT32 dword);
void    uart_write_no_wait(UINT32 dword);

extern void dbguart_print_byte_inAscii         (char *commentStr, UINT8 byte);
extern void dbguart_print_halfword_inAscii     (char *commentStr, UINT16 halfWord);
extern void dbguart_print_word_inAscii         (char *commentStr, UINT32 word);

#ifdef PERIPHERAL_UART
BOOL32 dbguart_RxFifoNotEmpty(void);
#endif


UINT32 dbguart_print(char * string);

#define bt_debug_print(string)                 dbguart_print(string)
extern void debug_Printf( char* format, ... );


#else  // BT_DEBUG_UART

#define dbguart_print(x)
#define bt_debug_print(x)

#define dbguart_init(x, y)
#define dbguart_read(c) DEBUG_UART_RX_EMPTY

#if (defined(BB_2046B0) || defined(BB_2046B1) || defined(BB_2070A0) || defined(BB_2072) || defined(BT_DEBUG_WRITE_BYTE))
#define dbguart_write(byte) DEBUG_UART_RX_EMPTY
#else
#define dbguart_write(dword) DEBUG_UART_RX_EMPTY
#endif
#define debug_Printf(...)
#endif  //BT_DEBUG_UART

#ifdef ADDL_DBG_UART_API_SUPPORTED
extern void ConvertByteToAscii (UINT8 byte, char * pString);
#endif

#ifdef BT_LOGGING_TRACE_ENABLE
// ! print data to console/debug UART. It is optimized to be fast.
void debug_Printf_data( unsigned char * buf, UINT32 nbytes);
#else
#define debug_Printf_data(buf,len)
#endif


#ifdef __cplusplus
}
#endif

/*
******************************************************************************************
*
* $HISTORY:$
*
******************************************************************************************
*/
#endif // __DBGUART_H__
