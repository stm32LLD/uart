# **STM32 UART Low Level Driver**
Following repository constains STM32 UART Low Level (LL) driver C implementation based on STM32 HAL library.

Transmission and reception of UART messages are completely non-blocking and implemented using FIFO buffers.

STM32 UART LL driver is supporting following STM32 device family:
- STM32L4/L4+:  Has UART4 (optional), UART5 (optional)
- STM32H7:      Has UART4, UART5, UART7, UART8

## **Dependencies**

### **1. Ring Buffer**
STM32 UART LL driver module needs [Ring Buffer](https://github.com/Misc-library-for-DSP/ring_buffer) C module in order to provide FIFO buffer funtionalities. Ring Buffer module is part of *General Embedded C Libraries Ecosystem*.

It is mandatory to be under following path in order to be compatible "General Embedded C Libraries Ecosystem":
```
root/middleware/ring_buffer/src/ring_buffer.h
```

### **2. GPIO STM32 LL Driver**
STM32 UART LL driver module needs [GPIO](https://github.com/stm32LLD/gpio) C module in order to initialize UART pins clock.

It is mandatory to be under following path in order to be compatible "General Embedded C Libraries Ecosystem":
```
root/drivers/peripheral/gpio/gpio/src/gpio.h
```

### **3. STM32 HAL library**
STM32 UART LL driver module uses STM32 HAL library.


## **Limitations**

### **1. Initialization phase**
Following STM32 HAL UART initialization fields are hard coded:
```C
g_uart[uart_opt].handle.Init.WordLength             = UART_WORDLENGTH_8B;
g_uart[uart_opt].handle.Init.StopBits               = UART_STOPBITS_1;
g_uart[uart_opt].handle.Init.Parity                 = UART_PARITY_NONE;
g_uart[uart_opt].handle.Init.Mode                   = UART_MODE_TX_RX;
g_uart[uart_opt].handle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
g_uart[uart_opt].handle.Init.OverSampling           = UART_OVERSAMPLING_16;
g_uart[uart_opt].handle.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
g_uart[uart_opt].handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
```

### **2. STM32 MCU support**
For now only *STM32L475* MCU is supported. Following sections of UART LL driver module shall be changed in order to support other STM32 MCU families:
- Enable/Disable UART clocks: Look at *uart_enable_clock* function for more info...
- UART Interrupt routines: Look at *UART4_IRQHandler* for more info...

## **API**
| API Functions | Description | Prototype |
| --- | ----------- | ----- |
| **uart_init** | Initialization of module | uart_status_t uart_init(const uart_ch_t uart_ch) |
| **uart_deinit** | De-initialization of module | uart_status_t uart_deinit(const uart_ch_t uart_ch) |
| **uart_is_init** | Get initialization state of UART module | uart_status_t uart_is_init(const uart_ch_t uart_ch, bool * const p_is_init) |
| **uart_transmit** | Transmit data over UART | uart_status_t uart_transmit(const uart_ch_t uart_ch, const uint8_t * const p_data, const uint32_t size) |
| **uart_receive** | Receive data over UART| uart_status_t uart_receive(const uart_ch_t uart_ch, uint8_t * const p_data) |


## **Usage**

**GENERAL NOTICE: Put all user code between sections: USER CODE BEGIN & USER CODE END!**

**1. Copy template files to root directory of the module**

Copy configuration file *uart_cfg* to root directory and replace file extension (.htmp/.ctmp -> .h/.c).

**2. Change default HAL library include to target microprocessor inside ***uart_cfg.h***:**

Following example shows HAL library include for STM32L4 family:
```C
// USER INCLUDE BEGIN...

#include "stm32l4xx_hal.h"

// USER INCLUDE END...
```

**3. Configure UART module for application needs by changing ***uart_cfg.h***. Configuration options are following:**

| Configuration | Description |
| --- | --- |
| **UART_CFG_MTU** 			| Maximum size of message to transmit in bytes |
| **USBD_CFG_ASSERT_EN** 		        | Enable/Disable assertions |
| **USBD_ASSERT** 		                | Assert definition |

**4. List all needed UART channels inside ***uart_cfg.h***:**
```C
/**
 *  UART Communication Channels
 *
 *  @note   Must start with enumeration of 0!
 */
typedef enum
{
    // USER CODE BEGIN...

    eUART_WIFI = 0,
    eUART_BLE,

    // USER CODE END...

    eUART_CH_NUM_OF     /**<Number of all UART channels */
} uart_ch_t;
```

**5. Configure all needed UART channels inside ***uart_cfg.c***:**
```C
/**
 *      UART Channel Configuration
 */
static const uart_cfg_t g_uart_cfg[eUART_CH_NUM_OF] =
{
    // USER CODE BEGIN...

    // =============================================================
    //      UART 4 Settings
    // =============================================================
    [eUART_WIFI] =
    {
        .p_instance     = UART4,
        .baudrate       = 115200U,
        .rx_buf_size    = 512U,
        .tx_buf_size    = 512U,
        .tx_pin =
        {
            .p_port = TX4_WIFI__PORT,
            .pin    = TX4_WIFI__PIN,
            .pull   = TX4_WIFI__PULL,
            .af     = TX4_WIFI__AF
        },
        .rx_pin =
        {
            .p_port = RX4_WIFI__PORT,
            .pin    = RX4_WIFI__PIN,
            .pull   = RX4_WIFI__PULL,
            .af     = RX4_WIFI__AF
        },
        .irq_prio   = PROJ_CFG_IRQ_PRIO_UART4,
        .irq_num    = UART4_IRQn,
    },

    // =============================================================
    //      UART 5 Settings
    // =============================================================
    [eUART_BLE] =
    {
        .p_instance     = UART5,
        .baudrate       = 115200U,
        .rx_buf_size    = 512U,
        .tx_buf_size    = 512U,
        .tx_pin =
        {
            .p_port = TX5_BLE__PORT,
            .pin    = TX5_BLE__PIN,
            .pull   = TX5_BLE__PULL,
            .af     = TX5_BLE__AF
        },
        .rx_pin =
        {
            .p_port = RX5_BLE__PORT,
            .pin    = RX5_BLE__PIN,
            .pull   = RX5_BLE__PULL,
            .af     = RX5_BLE__AF
        },
        .irq_prio   = PROJ_CFG_IRQ_PRIO_UART5,
        .irq_num    = UART5_IRQn,
    },

    // USER CODE END...
};
```

**6. Initialize UART module**
```C
// Init UART to WIFI device
if ( eUART_OK != uart_init( eUART_WIFI ))
{
    // Initialization failed...

    // Further actions here...
}

// Init UART to BLE device
if ( eUART_OK != uart_init( eUART_BLE ))
{
    // Initialization failed...

    // Further actions here...
}
```

**7. Transmit data over UART**
```C
const uint8_t * msg = (uint8_t*) "Hello World\r\n";
const uint32_t size = strlen((const char*) msg );

// Transmit msg to BLE UART channel
if ( eUART_OK != uart_transmit( eUART_BLE, (const uint8_t*) msg, size ))
{
    // Transmission failed...

    // Further actions here...
}
```

**8. Receive data over UART**
```C
static uint8_t   rx_data[128] = {0};
       uint32_t  rx_data_idx  = 0U;

// Get all received bytes from WIFI device
while( eUART_OK == uart_receive( eUART_WIFI, &rx_data[ rx_data_idx ] ))
{
    rx_data_idx++;
}

// Furhter actions with received data here...
```
