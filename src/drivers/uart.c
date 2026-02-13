/**
 * UART Driver
 * 
 * Interrupt-driven UART with ring buffers
 * 
 * Supported peripherals:
 *   USART1 - ESC telemetry, half-duplex on PB6 (AF7) @ 115200
 *            Single-wire bidirectional for SRXL2 protocol
 *   UART4  - CRSF receiver (PA0=RX, PA1=TX, AF8) @ 420000
 *   USART6 - MSP OSD (PC6=RX, PC7=TX, AF8) @ 115200
 */

#include "uart.h"
#include "target.h"
#include "stm32f7xx_hal.h"

/* Ring buffer size (must be power of 2) */
#define UART_RX_BUF_SIZE 256
#define UART_RX_BUF_MASK (UART_RX_BUF_SIZE - 1)

/* Ring buffer structure */
typedef struct {
    volatile uint8_t  data[UART_RX_BUF_SIZE];
    volatile uint16_t head;  /* Write index (ISR) */
    volatile uint16_t tail;  /* Read index (main) */
} ring_buffer_t;

/* USART1 - ESC (half-duplex on PB6) */
static UART_HandleTypeDef huart1;
static ring_buffer_t uart1_rx_buf;
static bool uart1_initialized = false;

/* UART4 - CRSF */
static UART_HandleTypeDef huart4;
static ring_buffer_t uart4_rx_buf;
static bool uart4_initialized = false;

/* USART6 - MSP OSD */
static UART_HandleTypeDef huart6;
static bool uart6_initialized = false;

/* RX byte counters (for debugging) */
volatile uint32_t uart1_rx_count = 0;
volatile uint32_t uart4_rx_count = 0;

/* Forward declarations */
static bool uart1_init_halfduplex(uint32_t baudrate);
static void uart1_deinit(void);
static bool uart4_init(uint32_t baudrate);
static bool uart6_init(uint32_t baudrate);

/* ============================================================================
 * Public API
 * ============================================================================ */

bool uart_init(uart_port_t port, uint32_t baudrate)
{
    switch (port) {
        case UART_CRSF:
            return uart4_init(baudrate);
        case UART_ESC:
            return uart1_init_halfduplex(baudrate);
        case UART_AUX:
            return uart6_init(baudrate);
        default:
            return false;
    }
}

void uart_deinit(uart_port_t port)
{
    switch (port) {
        case UART_ESC:
            uart1_deinit();
            break;
        default:
            break;
    }
}

uint16_t uart_available(uart_port_t port)
{
    ring_buffer_t *buf = NULL;
    
    switch (port) {
        case UART_CRSF:
            if (!uart4_initialized) return 0;
            buf = &uart4_rx_buf;
            break;
        case UART_ESC:
            if (!uart1_initialized) return 0;
            buf = &uart1_rx_buf;
            break;
        default:
            return 0;
    }
    
    return (buf->head - buf->tail) & UART_RX_BUF_MASK;
}

int16_t uart_read_byte(uart_port_t port)
{
    ring_buffer_t *buf = NULL;
    
    switch (port) {
        case UART_CRSF:
            if (!uart4_initialized) return -1;
            buf = &uart4_rx_buf;
            break;
        case UART_ESC:
            if (!uart1_initialized) return -1;
            buf = &uart1_rx_buf;
            break;
        default:
            return -1;
    }
    
    if (buf->head == buf->tail) {
        return -1;  /* Buffer empty */
    }
    
    uint8_t data = buf->data[buf->tail];
    buf->tail = (buf->tail + 1) & UART_RX_BUF_MASK;
    return data;
}

uint16_t uart_read(uart_port_t port, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    int16_t byte;
    
    while (count < len && (byte = uart_read_byte(port)) >= 0) {
        data[count++] = (uint8_t)byte;
    }
    
    return count;
}

void uart_send(uart_port_t port, const uint8_t *data, uint16_t len)
{
    UART_HandleTypeDef *huart = NULL;
    
    switch (port) {
        case UART_CRSF:
            if (!uart4_initialized) return;
            huart = &huart4;
            break;
        case UART_ESC:
            if (!uart1_initialized) return;
            huart = &huart1;
            break;
        case UART_AUX:
            if (!uart6_initialized) return;
            huart = &huart6;
            break;
        default:
            return;
    }

    /* For half-duplex (USART1), HAL handles direction automatically */
    HAL_UART_Transmit(huart, (uint8_t *)data, len, 100);
    
    /* For half-duplex, wait for transmission complete before returning
     * so the line is released for the slave to respond */
    if (port == UART_ESC) {
        while (!__HAL_UART_GET_FLAG(huart, UART_FLAG_TC)) {
            /* Wait for TC flag */
        }
    }
}

void uart_send_byte(uart_port_t port, uint8_t byte)
{
    uart_send(port, &byte, 1);
}

/* Simple printf helper for integers */
static void print_int(int32_t val)
{
    extern void usb_cdc_print(const char *str);
    extern void usb_cdc_send(const uint8_t *data, uint16_t len);
    char buf[16];
    int i = 0;
    bool neg = false;
    
    if (val < 0) {
        neg = true;
        val = -val;
    }
    
    if (val == 0) {
        buf[i++] = '0';
    } else {
        while (val > 0) {
            buf[i++] = '0' + (val % 10);
            val /= 10;
        }
    }
    
    if (neg) {
        usb_cdc_print("-");
    }
    
    /* Reverse and print */
    while (i > 0) {
        uint8_t c = buf[--i];
        usb_cdc_send(&c, 1);
    }
}

void uart_debug_dump(uart_port_t port)
{
    if (port != UART_ESC || !uart1_initialized) return;
    
    const char hex[] = "0123456789ABCDEF";
    uint8_t buf[32];
    
    /* Copy last 32 bytes without consuming */
    for (int i = 0; i < 32; i++) {
        uint16_t idx = (uart1_rx_buf.head - 32 + i) & UART_RX_BUF_MASK;
        buf[i] = uart1_rx_buf.data[idx];
    }
    
    /* Print via USB CDC */
    extern void usb_cdc_print(const char *str);
    extern void usb_cdc_send(const uint8_t *data, uint16_t len);
    
    for (int i = 0; i < 32; i++) {
        usb_cdc_send((uint8_t*)&hex[(buf[i] >> 4) & 0xF], 1);
        usb_cdc_send((uint8_t*)&hex[buf[i] & 0xF], 1);
        usb_cdc_print(" ");
        if (i == 15) usb_cdc_print("\r\n             ");
    }
    usb_cdc_print("\r\n");

    usb_cdc_print("  RX head:   ");
    print_int(uart1_rx_buf.head);
    usb_cdc_print("\r\n");
}

/* ============================================================================
 * USART1 - ESC Telemetry (Half-duplex on PB6)
 * 
 * Single-wire bidirectional UART for SRXL2 protocol.
 * PB6 = USART1_TX (AF7) configured as open-drain with pull-up.
 * Half-duplex mode enabled via HDSEL bit.
 * ============================================================================ */

static bool uart1_init_halfduplex(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure GPIO pin PB6 for half-duplex: */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;     /* Push-pull */
    GPIO_InitStruct.Pull = GPIO_NOPULL;         /*  External or line capacitance handles it */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Initialize ring buffer */
    uart1_rx_buf.head = 0;
    uart1_rx_buf.tail = 0;
    
    /* Configure USART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    
    if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) {
        return false;
    }
    
    /* Enable RXNE interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    
    /* Enable USART1 interrupt in NVIC */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    uart1_initialized = true;
    return true;
}

static void uart1_deinit(void)
{
    if (!uart1_initialized) return;
    
    /* Disable interrupt */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    
    /* Deinit UART */
    HAL_UART_DeInit(&huart1);
    
    /* Reset GPIO to default (input) */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
    
    /* Disable USART1 clock */
    __HAL_RCC_USART1_CLK_DISABLE();
    
    uart1_initialized = false;
}

/* USART1 interrupt handler */
void USART1_IRQHandler(void)
{
    /* Check for RX not empty */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t data = (uint8_t)(huart1.Instance->RDR & 0xFF);
        
        /* Store in ring buffer */
        uint16_t next_head = (uart1_rx_buf.head + 1) & UART_RX_BUF_MASK;
        if (next_head != uart1_rx_buf.tail) {
            /* Buffer not full */
            uart1_rx_buf.data[uart1_rx_buf.head] = data;
            uart1_rx_buf.head = next_head;
        }
        uart1_rx_count++;
    }
    
    /* Clear overrun error if set (prevents lockup) */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    
    /* Clear any other error flags */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FEFLAG(&huart1);
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)) {
        __HAL_UART_CLEAR_NEFLAG(&huart1);
    }
}

/* ============================================================================
 * UART4 - CRSF Receiver
 * Pins: PA0 = RX, PA1 = TX (AF8) with TX/RX swap enabled
 * ============================================================================ */

static bool uart4_init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable clocks */
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure GPIO pins: PA0 (RX), PA1 (TX)
     * From Rotorflight dump:
     *   resource SERIAL_TX 4 A01  -> PA1 is TX
     *   resource SERIAL_RX 4 A00  -> PA0 is RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  /* Pull-up for idle high */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Initialize ring buffer */
    uart4_rx_buf.head = 0;
    uart4_rx_buf.tail = 0;
    
    /* Configure UART4 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = baudrate;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    
    /* Enable TX/RX pin swap - Nexus routes PA0 to RX, PA1 to TX */
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
    huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        return false;
    }
    
    /* Enable RXNE interrupt */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
    
    /* Enable UART4 interrupt in NVIC */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);  /* Medium priority */
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    
    uart4_initialized = true;
    return true;
}

/* UART4 interrupt handler */
void UART4_IRQHandler(void)
{
    /* Check for RX not empty */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE)) {
        uint8_t data = (uint8_t)(huart4.Instance->RDR & 0xFF);
        
        /* Store in ring buffer */
        uint16_t next_head = (uart4_rx_buf.head + 1) & UART_RX_BUF_MASK;
        if (next_head != uart4_rx_buf.tail) {
            /* Buffer not full */
            uart4_rx_buf.data[uart4_rx_buf.head] = data;
            uart4_rx_buf.head = next_head;
        }
        uart4_rx_count++;
    }
    
    /* Clear overrun error if set (prevents lockup) */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart4);
    }
    
    /* Clear any other error flags */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FEFLAG(&huart4);
    }
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_NE)) {
        __HAL_UART_CLEAR_NEFLAG(&huart4);
    }
}

/* ============================================================================
 * USART6 - MSP OSD (Port B)
 * Pins: PC7 = TX, PC6 = RX (AF8)
 * TX-only for DisplayPort OSD, RX enabled but unused
 * ============================================================================ */

static bool uart6_init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIO: PC6 (RX), PC7 (TX) */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure USART6 */
    huart6.Instance = USART6;
    huart6.Init.BaudRate = baudrate;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart6) != HAL_OK) {
        return false;
    }

    uart6_initialized = true;
    return true;
}
