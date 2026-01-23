/**
 * UART Driver
 * 
 * Interrupt-driven UART with ring buffers
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

/* UART handles and buffers */
static UART_HandleTypeDef huart4;
static ring_buffer_t uart4_rx_buf;
static bool uart4_initialized = false;

/* Forward declarations */
static bool uart4_init(uint32_t baudrate);

/* ============================================================================
 * Public API
 * ============================================================================ */

bool uart_init(uart_port_t port, uint32_t baudrate)
{
    switch (port) {
        case UART_CRSF:
            return uart4_init(baudrate);
        case UART_ESC:
            /* TODO: UART3 for ESC telemetry */
            return false;
        case UART_AUX:
            /* TODO: UART6 spare */
            return false;
        default:
            return false;
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
        default:
            return;
    }
    
    HAL_UART_Transmit(huart, (uint8_t *)data, len, 100);
}

void uart_send_byte(uart_port_t port, uint8_t byte)
{
    uart_send(port, &byte, 1);
}

/* ============================================================================
 * UART4 - CRSF Receiver
 * Pins: PA0 = RX, PA1 = TX (AF8)
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
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    
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
        /* If buffer full, data is dropped */
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
