// Copyright (c) 2023 Ziga Miklosic
// All Rights Reserved
////////////////////////////////////////////////////////////////////////////////
/**
*@file      uart.h
*@brief     UART LL drivers based on STM32 HAL library
*@author    Ziga Miklosic
*@email		ziga.miklosic@gmail.si
*@date      21.04.2023
*@version   V0.1.0
*/
////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup UART_API
* @{ <!-- BEGIN GROUP -->
*
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef __UART_H
#define __UART_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "../../uart_cfg.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 *  Module version
 */
#define UART_VER_MAJOR          ( 0 )
#define UART_VER_MINOR          ( 1 )
#define UART_VER_DEVELOP        ( 0 )

/**
 *  UART status
 */
typedef enum
{
    eUART_OK        = 0x00U,    /**<Normal operation */
    eUART_ERROR     = 0x01U,    /**<General error code */
    eUART_WAR_EMPTY = 0x02U,    /**<Buffer empty warning */
    eUART_WAR_FULL  = 0x04U,    /**<Buffer full warning */
} uart_status_t;

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_init        (const uart_ch_t uart_ch);
uart_status_t uart_deinit      (const uart_ch_t uart_ch);
uart_status_t uart_is_init     (const uart_ch_t uart_ch, bool * const p_is_init);
uart_status_t uart_transmit    (const uart_ch_t uart_ch, const uint8_t * const p_data, const uint32_t size);
uart_status_t uart_receive     (const uart_ch_t uart_ch, uint8_t * const p_data);
uart_status_t uart_clear_rx_buf(const uart_ch_t uart_ch);
uart_status_t uart_clear_tx_buf(const uart_ch_t uart_ch);

#endif // __UART_H

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
