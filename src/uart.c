// Copyright (c) 2023 Ziga Miklosic
// All Rights Reserved
////////////////////////////////////////////////////////////////////////////////
/**
*@file      uart.c
*@brief     UART LL drivers based on STM32 HAL library
*@author    Ziga Miklosic
*@email     ziga.miklosic@gmail.si
*@date      21.04.2023
*@version   V0.1.0
*/
////////////////////////////////////////////////////////////////////////////////
/*!
* @addtogroup UART
* @{ <!-- BEGIN GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "uart.h"
#include "../../uart_cfg.h"

// Fifo
#include "middleware/ring_buffer/src/ring_buffer.h"

// Gpio
#include "drivers/peripheral/gpio/gpio/src/gpio.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 *  Compatibility check with RING_BUFFER
 *
 *  Support version V2.x.x
 */
_Static_assert( 2 == RING_BUFFER_VER_MAJOR );

/**
 *  UART control
 */
typedef struct
{
    UART_HandleTypeDef  handle;         /**<UART handler */
    p_ring_buffer_t     tx_buf;         /**<Transmission buffer */
    p_ring_buffer_t     rx_buf;         /**<Reception buffer */
    bool                is_init;        /**<Initialization flag */
} uart_ctrl_t;

/**
 *  FIFO buffer attributes
 */
static const ring_buffer_attr_t g_buf_attr =
{
   .item_size   = sizeof( uint8_t ),    // Byte size
   .override    = false,                // Do not lost data
   .p_mem       = NULL,                 // Dynamically allocate
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/**
 *  UART control block
 */
static uart_ctrl_t g_uart[eUART_CH_NUM_OF] = {0};


////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////
static uart_status_t    uart_init_fifo      (const uart_ch_t uart_ch, const uint32_t tx_size, const uint32_t rx_size);
static void             uart_enable_clock   (const USART_TypeDef * p_inst);
static void             uart_disable_clock  (const USART_TypeDef * p_inst);
static void             uart_init_gpio      (const uart_pin_cfg_t * const p_pin_cfg);
static void             uart_deinit_gpio    (const uart_pin_cfg_t * const p_pin_cfg);
static inline bool      uart_find_channel   (const USART_TypeDef * p_inst, uart_ch_t * const p_ch);
static inline void      uart_process_isr    (const USART_TypeDef * p_inst);


////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief    	Initialize UART FIFO buffers
*
* @param[in]	uart_ch     - UART communication channel
* @param[in]	tx_size     - Size of TX FIFO
* @param[in]	rx_size     - Size of RX FIFO
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static uart_status_t uart_init_fifo(const uart_ch_t uart_ch, const uint32_t tx_size, const uint32_t rx_size)
{
    uart_status_t           status      = eUART_OK;
    ring_buffer_status_t    fifo_status = eRING_BUFFER_OK;

    // Init FIFO
    fifo_status |= ring_buffer_init( &g_uart[ uart_ch ].rx_buf, rx_size, &g_buf_attr );
    fifo_status |= ring_buffer_init( &g_uart[ uart_ch ].tx_buf, tx_size, &g_buf_attr );

    // Init Rx FIFO
    if ( eRING_BUFFER_OK != fifo_status )
    {
        status = eUART_ERROR;

        UART_ASSERT( 0 );
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Enable UART clock
*
* @param[in]    p_inst  - UART peripheral instance
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void uart_enable_clock(const USART_TypeDef * p_inst)
{
#if defined(USART1)
	if ( USART1 == p_inst )
	{
		__HAL_RCC_USART1_CLK_ENABLE();
	}
#endif

#if defined(USART2)
	if ( USART2 == p_inst )
	{
		__HAL_RCC_USART2_CLK_ENABLE();
	}
#endif

#if defined(USART3)
	if ( USART3 == p_inst )
	{
		__HAL_RCC_USART3_CLK_ENABLE();
	}
#endif

#if defined(UART4)
    if ( UART4 == p_inst )
    {
        __HAL_RCC_UART4_CLK_ENABLE();
    }
#endif

#if defined(UART5)
    if ( UART5 == p_inst )
    {
        __HAL_RCC_UART5_CLK_ENABLE();
    }
#endif

#if defined(UART7)
    if ( UART7 == p_inst )
    {
        __HAL_RCC_UART7_CLK_ENABLE();
    }
#endif

#if defined(UART8)
    if ( UART8 == p_inst )
    {
        __HAL_RCC_UART8_CLK_ENABLE();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Disable UART clock
*
* @param[in]    p_inst  - UART peripheral instance
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void uart_disable_clock(const USART_TypeDef * p_inst)
{
#if defined(USART1)
	if ( USART1 == p_inst )
	{
		__HAL_RCC_USART1_CLK_DISABLE();
	}
#endif

#if defined(USART2)
	if ( USART2 == p_inst )
	{
		__HAL_RCC_USART2_CLK_DISABLE();
	}
#endif

#if defined(USART3)
	if ( USART3 == p_inst )
	{
		__HAL_RCC_USART3_CLK_DISABLE();
	}
#endif

#if defined(UART4)
    if ( UART4 == p_inst )
    {
        __HAL_RCC_UART4_CLK_DISABLE();
    }
#endif

#if defined(UART5)
    if ( UART5 == p_inst )
    {
        __HAL_RCC_UART5_CLK_DISABLE();
    }
#endif

#if defined(UART7)
    if ( UART7 == p_inst )
    {
        __HAL_RCC_UART7_CLK_DISABLE();
    }
#endif

#if defined(UART8)
    if ( UART8 == p_inst )
    {
        __HAL_RCC_UART8_CLK_DISABLE();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Init UART pin
*
* @param[in]    p_pin_cfg   - Pointer to pin configuration
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void uart_init_gpio(const uart_pin_cfg_t * const p_pin_cfg)
{
    GPIO_InitTypeDef gpio_init = {0};

    // Enable clock
    gpio_enable_port_clock( p_pin_cfg->p_port );

    // Prepare gpio init structure
    gpio_init.Pin        = p_pin_cfg->pin;
    gpio_init.Mode       = GPIO_MODE_AF_PP;
    gpio_init.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Pull       = p_pin_cfg->pull;
    gpio_init.Alternate  = p_pin_cfg->af;

    // Init gpio
    HAL_GPIO_Init( p_pin_cfg->p_port, &gpio_init );
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        De-Init UART pin
*
* @param[in]    p_pin_cfg   - Pointer to pin configuration
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void uart_deinit_gpio(const uart_pin_cfg_t * const p_pin_cfg)
{
    // Init gpio
    HAL_GPIO_DeInit( p_pin_cfg->p_port, p_pin_cfg->pin );
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Find UART channel by hardware instance
*
* @param[in]    p_inst      - UART periphery (UART4, UART5,...)
* @param[out]   p_opt       - UART enumeration option
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static inline bool uart_find_channel(const USART_TypeDef * p_inst, uart_ch_t * const p_ch)
{
    bool found = false;

    for ( uint8_t ch = 0U; ch < eUART_CH_NUM_OF; ch++ )
    {
        // Get UART configurations
        const uart_cfg_t * p_uart_cfg = uart_cfg_get_config( ch );

        if ( p_inst == p_uart_cfg->p_instance )
        {
            found = true;
            *p_ch = ch;
            break;
        }
    }

    return found;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Process UART ISR
*
* @param[in]    uart_ch    - UART communication channel
* @return       void
*/
////////////////////////////////////////////////////////////////////////////////
static inline void uart_process_isr(const USART_TypeDef * p_inst)
{
    uint8_t     u8_data     = 0U;
    uart_ch_t  uart_ch    = 0;

    // Find UART channel by hardware instance
    if ( true == uart_find_channel( p_inst, &uart_ch ))
    {
        // UART overrun error
        if ( __HAL_UART_GET_FLAG( &g_uart[uart_ch].handle, UART_FLAG_ORE ))
        {
            __HAL_UART_CLEAR_FLAG( &g_uart[uart_ch].handle, UART_FLAG_ORE );

            // Discard received character
            (void) g_uart[uart_ch].handle.Instance->RDR;
        }

        // Framing error
        else if ( __HAL_UART_GET_FLAG( &g_uart[uart_ch].handle, UART_FLAG_FE ))
        {
            __HAL_UART_CLEAR_FLAG( &g_uart[uart_ch].handle, UART_FLAG_FE );

            // Discard received character
            (void) g_uart[uart_ch].handle.Instance->RDR;
        }

        // Noise error
        else if ( __HAL_UART_GET_FLAG( &g_uart[uart_ch].handle, UART_FLAG_NE ))
        {
            __HAL_UART_CLEAR_FLAG( &g_uart[uart_ch].handle, UART_FLAG_NE );

            // Discard received character
            (void) g_uart[uart_ch].handle.Instance->RDR;
        }

        // UART read data register not empty
        else if( __HAL_UART_GET_FLAG( &g_uart[uart_ch].handle, UART_FLAG_RXNE ))
        {
            // Get received character
            u8_data = g_uart[uart_ch].handle.Instance->RDR;

            // Put to buffer
            (void) ring_buffer_add( g_uart[uart_ch].rx_buf, &u8_data );
        }

        // UART transmit data register empty
        else if( __HAL_UART_GET_FLAG( &g_uart[uart_ch].handle, UART_FLAG_TXE ))
        {
            // Take data from Tx buffer and send it
            if ( eRING_BUFFER_OK == ring_buffer_get( g_uart[uart_ch].tx_buf, &u8_data ))
            {
                g_uart[uart_ch].handle.Instance->TDR = u8_data;
            }

            // Tx FIFO empty -> stop TXE interrupt
            else
            {
                __HAL_UART_DISABLE_IT( &g_uart[uart_ch].handle, UART_IT_TXE );
            }
        }

        else
        {
            // No actions...
        }
    }
}

#if defined(USART1)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        USART 1 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void USART1_IRQHandler(void)
    {
        uart_process_isr( USART1 );
    }
#endif

#if defined(USART2)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        USART 2 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void USART2_IRQHandler(void)
    {
        uart_process_isr( USART2 );
    }
#endif

#if defined(USART3)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        USART 3 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void USART3_IRQHandler(void)
    {
        uart_process_isr( USART3 );
    }
#endif

#if defined(UART4)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        UART 4 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void UART4_IRQHandler(void)
    {
        uart_process_isr( UART4 );
    }
#endif

#if defined(UART5)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        UART 5 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void UART5_IRQHandler(void)
    {
        uart_process_isr( UART5 );
    }
#endif

#if defined(UART7)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        UART 7 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void UART7_IRQHandler(void)
    {
        uart_process_isr( UART7 );
    }
#endif

#if defined(UART8)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        UART 8 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void UART8_IRQHandler(void)
    {
        uart_process_isr( UART8 );
    }
#endif

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup UART_API
* @{ <!-- BEGIN GROUP -->
*
* 	Following function are part of UART API.
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Initialize UART
*
* @param[in]    uart_ch     - UART communication channel
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_init(const uart_ch_t uart_ch)
{
    uart_status_t status = eUART_OK;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );

    if ( uart_ch < eUART_CH_NUM_OF )
    {
        if  ( false == g_uart[uart_ch].is_init )
        {
            // Get UART configurations
            const uart_cfg_t * p_uart_cfg = uart_cfg_get_config( uart_ch );

            // Init FIFO
            status |= uart_init_fifo( uart_ch, p_uart_cfg->tx_buf_size, p_uart_cfg->rx_buf_size );

            // Init GPIOs
            uart_init_gpio( &( p_uart_cfg->tx_pin ));
            uart_init_gpio( &( p_uart_cfg->rx_pin ));

            // Enable clock
            uart_enable_clock( p_uart_cfg->p_instance );

            // Prepare HAL init structure
            g_uart[uart_ch].handle.Instance                    = p_uart_cfg->p_instance;
            g_uart[uart_ch].handle.Init.BaudRate               = p_uart_cfg->baudrate;
            g_uart[uart_ch].handle.Init.WordLength             = UART_WORDLENGTH_8B;
            g_uart[uart_ch].handle.Init.StopBits               = UART_STOPBITS_1;
            g_uart[uart_ch].handle.Init.Parity                 = UART_PARITY_NONE;
            g_uart[uart_ch].handle.Init.Mode                   = UART_MODE_TX_RX;
            g_uart[uart_ch].handle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
            g_uart[uart_ch].handle.Init.OverSampling           = UART_OVERSAMPLING_16;
            g_uart[uart_ch].handle.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;

            // Check if request for swapping pins
            if ( true == p_uart_cfg->swap_txrx )
            {
                g_uart[uart_ch].handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
                g_uart[uart_ch].handle.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
            }
            else
            {
                g_uart[uart_ch].handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
            }

            // Init uart
            if ( HAL_OK != HAL_UART_Init( &g_uart[uart_ch].handle ))
            {
                status = eUART_ERROR;
            }

            // Init success
            if ( eUART_OK == status )
            {
                // Enable reception buffer not empty interrupt
                __HAL_UART_ENABLE_IT( &g_uart[uart_ch].handle, UART_IT_RXNE );

                // Error interrupt (frame error, noise error, overrun error)
                 __HAL_UART_ENABLE_IT( &g_uart[uart_ch].handle, UART_IT_ERR );

                // Setup UART interrupt priority and enable it
                NVIC_SetPriority( p_uart_cfg->irq_num, p_uart_cfg->irq_prio );
                NVIC_EnableIRQ( p_uart_cfg->irq_num );

                // Enable UART
                __HAL_UART_ENABLE( &g_uart[uart_ch].handle );

                // Init success
                g_uart[uart_ch].is_init = true;
            }
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        De-Initialize UART
*
* @param[in]    uart_ch     - UART communication channel
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_deinit(const uart_ch_t uart_ch)
{
    uart_status_t status = eUART_OK;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );

    if ( uart_ch < eUART_CH_NUM_OF )
    {
        if  ( true == g_uart[uart_ch].is_init )
        {
            // Get UART configurations
            const uart_cfg_t * p_uart_cfg = uart_cfg_get_config( uart_ch );

            // De-init UART
            if ( HAL_OK != HAL_UART_DeInit( &g_uart[uart_ch].handle ))
            {
                status = eUART_ERROR;
            }

            // Disable reception buffer not empty interrupt
            __HAL_UART_DISABLE_IT( &g_uart[uart_ch].handle, UART_IT_RXNE );

            // Disable interrupt (frame error, noise error, overrun error)
             __HAL_UART_DISABLE_IT( &g_uart[uart_ch].handle, UART_IT_ERR );

            // De-init gpios
            uart_deinit_gpio( &( p_uart_cfg->tx_pin ));
            uart_deinit_gpio( &( p_uart_cfg->rx_pin ));

            // Disable clock
            uart_disable_clock( p_uart_cfg->p_instance );

            // De-Init success
            if ( eUART_OK == status )
            {
                g_uart[uart_ch].is_init = false;
            }
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Get UART initialization flag
*
* @param[in]    uart_ch     - UART communication channel
* @param[out]   p_is_init   - Pointer to init flag
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_is_init(const uart_ch_t uart_ch, bool * const p_is_init)
{
    uart_status_t status = eUART_OK;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );
    UART_ASSERT( NULL != p_is_init );

    if (    ( uart_ch < eUART_CH_NUM_OF )
        &&  ( NULL != p_is_init ))
    {
        *p_is_init = g_uart[uart_ch].is_init;
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Transmit data over UART
*
* @note     This function is blocking!
*
* @param[in]    uart_ch     - UART communication channel
* @param[in]    p_data      - Pointer to data to send
* @param[in]    size        - Size of data to send in bytes
* @param[in]    timeout     - Timeout in miliseconds
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_transmit(const uart_ch_t uart_ch, const uint8_t * const p_data, const uint32_t size, const uint32_t timeout)
{
    uart_status_t status = eUART_OK;

    // Transmit data
    status = uart_transmit_it( uart_ch, p_data, size );

    if ( eUART_OK == status )
    {
        // Number of bytes in TX FIFO
        uint32_t tx_cnt = 0U;

        // Get current time
        const uint32_t now = UART_GET_SYSTICK();

        // Wait for transmission to complete
        do
        {
            // Check for timeout
            if (((uint32_t) ( UART_GET_SYSTICK() - now )) > timeout )
            {
                status = eUART_ERROR_TIMEOUT;
                break;
            }

            // Get number of bytes in Tx FIFO
            ring_buffer_get_taken( g_uart[uart_ch].tx_buf, &tx_cnt );

        } while( tx_cnt > 0U );
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Received data over UART
*
* @note     This function is blocking!
*
* @note     If timeout = 0, then this function is effectively non-blocking and
*           it will return "ERROR_TIMEOUT"!
*
* @param[in]    uart_ch     - UART communication channel
* @param[out]   p_data      - Pointer to data to send
* @param[in]    size        - Amount of data bytes to be received
* @param[in]    timeout     - Timeout in miliseconds
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_receive(const uart_ch_t uart_ch, uint8_t * const p_data, const uint32_t size, const uint32_t timeout)
{
    uart_status_t   status  = eUART_OK;
    uint32_t        rx_cnt  = 0;

    UART_ASSERT( size > 0U );

    if ( size > 0U )
    {
        // Get current time
        const uint32_t now = UART_GET_SYSTICK();

        // Wait until all requested data are received
        while( rx_cnt < size )
        {
            // Check for timeout
            if (((uint32_t) ( UART_GET_SYSTICK() - now )) > timeout )
            {
                status = eUART_ERROR_TIMEOUT;
                break;
            }

            // Byte received?
            if ( eUART_OK == uart_receive_it( uart_ch, (uint8_t*) &p_data[rx_cnt] ))
            {
                // Byte received
                rx_cnt++;
            }
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}


////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Transmit data over UART
*
* @note     This function is non-blocking!
*
* @note     In case Tx FIFO gets full it returns "eUART_WAR_FULL".
*
* @note     Fractional messaging is prevented by checking size of TX fifo
*           before copy of transmit data takes place. Consequence of that
*           might be that message will not be transmitted as it currently
*           cannot be fitted into Tx FIFO.
*
* @param[in]    uart_ch     - UART communication channel
* @param[in]    p_data      - Pointer to data to send
* @param[in]    size        - Size of data to send in bytes
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_transmit_it(const uart_ch_t uart_ch, const uint8_t * const p_data, const uint32_t size)
{
    uart_status_t   status          = eUART_OK;
    uint32_t        buf_free_space  = 0U;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );
    UART_ASSERT( true == g_uart[uart_ch].is_init );
    UART_ASSERT( NULL != p_data );
    UART_ASSERT( size <= UART_CFG_MTU );

    if ( uart_ch < eUART_CH_NUM_OF )
    {
        if  (   ( true == g_uart[uart_ch].is_init )
            &&  ( NULL != p_data )
            &&  ( size <= UART_CFG_MTU ))
        {
            // Enter critical
            __disable_irq();

            // Check if there is space in Tx FIFO
            (void) ring_buffer_get_free( g_uart[uart_ch].tx_buf, &buf_free_space );

            // There is space in Tx FIFO for complete message
            if ( size <= buf_free_space )
            {
                // Put all data to Tx FIFO
                for ( uint32_t byte_idx = 0; byte_idx < size; byte_idx++ )
                {
                    (void) ring_buffer_add( g_uart[uart_ch].tx_buf, (uint8_t*) &p_data[byte_idx] );
                }

                // Raise TX empty IRQ
                // NOTE: Later in irq message is being transmitted
                __HAL_UART_ENABLE_IT( &g_uart[uart_ch].handle, UART_IT_TXE );
            }

            // No space in Tx FIFO
            else
            {
                status = eUART_WAR_FULL;
            }

            // Exit critical
            __enable_irq();
        }
        else
        {
            status = eUART_ERROR;
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Received data over UART
*
* @note     This function is non-blocking!
* @note     Function returns "eUART_WAR_EMPTY" when Rx FIFO is empty.
*
* @param[in]    uart_ch     - UART communication channel
* @param[out]   p_data      - Pointer to data to send
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_receive_it(const uart_ch_t uart_ch, uint8_t * const p_data)
{
    uart_status_t status = eUART_OK;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );
    UART_ASSERT( true == g_uart[uart_ch].is_init );
    UART_ASSERT( NULL != p_data );

    if ( uart_ch < eUART_CH_NUM_OF )
    {
        if  (   ( true == g_uart[uart_ch].is_init )
            &&  ( NULL != p_data ))
        {
            // Get data from RX FIFO
            if ( eRING_BUFFER_OK != ring_buffer_get( g_uart[uart_ch].rx_buf, (uint8_t*) p_data ))
            {
                status = eUART_WAR_EMPTY;
            }
        }
        else
        {
            status = eUART_ERROR;
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Clear all content in RX FIFO
*
* @param[in]    uart_ch     - UART communication channel
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_clear_rx_buf(const uart_ch_t uart_ch)
{
    uart_status_t status = eUART_OK;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );
    UART_ASSERT( true == g_uart[uart_ch].is_init );

    if ( uart_ch < eUART_CH_NUM_OF )
    {
        if ( true == g_uart[uart_ch].is_init )
        {
            // Reset/Clear RX FIFO
            if ( eRING_BUFFER_OK != ring_buffer_reset( g_uart[uart_ch].rx_buf ))
            {
                status = eUART_ERROR;
            }
        }
        else
        {
            status = eUART_ERROR;
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Clear all content in TX FIFO
*
* @param[in]    uart_ch     - UART communication channel
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
uart_status_t uart_clear_tx_buf(const uart_ch_t uart_ch)
{
    uart_status_t status = eUART_OK;

    UART_ASSERT( uart_ch < eUART_CH_NUM_OF );
    UART_ASSERT( true == g_uart[uart_ch].is_init );

    if ( uart_ch < eUART_CH_NUM_OF )
    {
        if ( true == g_uart[uart_ch].is_init )
        {
            // Reset/Clear RX FIFO
            if ( eRING_BUFFER_OK != ring_buffer_reset( g_uart[uart_ch].tx_buf ))
            {
                status = eUART_ERROR;
            }
        }
        else
        {
            status = eUART_ERROR;
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
