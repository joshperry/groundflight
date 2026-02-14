/**
 * Mock HAL types for host-side testing
 *
 * Provides just enough type definitions for source headers to compile
 * on x86 without the real STM32 HAL.
 */

#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* Stub GPIO typedef (matches stm32f7xx.h layout) */
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

/* Stub UART handle */
typedef struct {
    uint32_t dummy;
} UART_HandleTypeDef;

/* Stub SPI handle */
typedef struct {
    uint32_t dummy;
} SPI_HandleTypeDef;

/* Stub TIM handle */
typedef struct {
    uint32_t dummy;
} TIM_HandleTypeDef;

/* NVIC stub */
static inline void NVIC_SystemReset(void) {}

/* System clock (used by main.c status command) */
static uint32_t SystemCoreClock = 216000000;

#endif /* MOCK_HAL_H */
