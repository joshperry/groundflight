/**
 * GroundFlight HAL Stubs
 * 
 * Minimal definitions to compile without full STM32Cube HAL.
 * Replace with real HAL by cloning STM32CubeF7 into lib/
 */

#ifndef HAL_STUBS_H
#define HAL_STUBS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Core Cortex-M7 Definitions
 * ============================================================================ */

/* CMSIS-style register access */
#define __IO    volatile
#define __I     volatile const
#define __O     volatile

/* Memory-mapped peripheral base addresses (STM32F722) */
#define PERIPH_BASE         ((uint32_t)0x40000000)
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE     (PERIPH_BASE + 0x10000000)

/* GPIO base addresses */
#define GPIOA_BASE          (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE          (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE          (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE          (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE          (AHB1PERIPH_BASE + 0x1000)

/* RCC base address */
#define RCC_BASE            (AHB1PERIPH_BASE + 0x3800)

/* ============================================================================
 * GPIO Registers
 * ============================================================================ */

typedef struct {
    __IO uint32_t MODER;    /* Mode register */
    __IO uint32_t OTYPER;   /* Output type register */
    __IO uint32_t OSPEEDR;  /* Output speed register */
    __IO uint32_t PUPDR;    /* Pull-up/pull-down register */
    __IO uint32_t IDR;      /* Input data register */
    __IO uint32_t ODR;      /* Output data register */
    __IO uint32_t BSRR;     /* Bit set/reset register */
    __IO uint32_t LCKR;     /* Lock register */
    __IO uint32_t AFR[2];   /* Alternate function registers */
} GPIO_TypeDef;

#define GPIOA   ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC   ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD   ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE   ((GPIO_TypeDef *)GPIOE_BASE)

/* GPIO mode values */
#define GPIO_MODE_INPUT     0x00
#define GPIO_MODE_OUTPUT    0x01
#define GPIO_MODE_AF        0x02
#define GPIO_MODE_ANALOG    0x03

/* ============================================================================
 * RCC Registers (Clock Control)
 * ============================================================================ */

typedef struct {
    __IO uint32_t CR;           /* Clock control register */
    __IO uint32_t PLLCFGR;      /* PLL configuration register */
    __IO uint32_t CFGR;         /* Clock configuration register */
    __IO uint32_t CIR;          /* Clock interrupt register */
    __IO uint32_t AHB1RSTR;     /* AHB1 peripheral reset register */
    __IO uint32_t AHB2RSTR;     /* AHB2 peripheral reset register */
    __IO uint32_t AHB3RSTR;     /* AHB3 peripheral reset register */
    uint32_t      RESERVED0;
    __IO uint32_t APB1RSTR;     /* APB1 peripheral reset register */
    __IO uint32_t APB2RSTR;     /* APB2 peripheral reset register */
    uint32_t      RESERVED1[2];
    __IO uint32_t AHB1ENR;      /* AHB1 peripheral clock enable */
    __IO uint32_t AHB2ENR;      /* AHB2 peripheral clock enable */
    __IO uint32_t AHB3ENR;      /* AHB3 peripheral clock enable */
    uint32_t      RESERVED2;
    __IO uint32_t APB1ENR;      /* APB1 peripheral clock enable */
    __IO uint32_t APB2ENR;      /* APB2 peripheral clock enable */
    /* ... more registers ... */
} RCC_TypeDef;

#define RCC     ((RCC_TypeDef *)RCC_BASE)

/* RCC AHB1ENR bit definitions */
#define RCC_AHB1ENR_GPIOAEN     (1 << 0)
#define RCC_AHB1ENR_GPIOBEN     (1 << 1)
#define RCC_AHB1ENR_GPIOCEN     (1 << 2)
#define RCC_AHB1ENR_GPIODEN     (1 << 3)
#define RCC_AHB1ENR_GPIOEEN     (1 << 4)

/* ============================================================================
 * SysTick
 * ============================================================================ */

typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t LOAD;
    __IO uint32_t VAL;
    __I  uint32_t CALIB;
} SysTick_TypeDef;

#define SysTick_BASE    (0xE000E010)
#define SysTick         ((SysTick_TypeDef *)SysTick_BASE)

#define SysTick_CTRL_ENABLE     (1 << 0)
#define SysTick_CTRL_TICKINT    (1 << 1)
#define SysTick_CTRL_CLKSOURCE  (1 << 2)
#define SysTick_CTRL_COUNTFLAG  (1 << 16)

/* ============================================================================
 * System Functions
 * ============================================================================ */

/* Implemented in target.c */
void SystemInit(void);
void SystemCoreClockUpdate(void);

extern uint32_t SystemCoreClock;

/* Simple delay using SysTick */
static inline void delay_ms(uint32_t ms) {
    /* Rough delay - assumes ~16MHz HSI default clock */
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __asm volatile ("nop");
    }
}

/* ============================================================================
 * Compatibility Macros
 * ============================================================================ */

/* Enable/disable interrupts */
#define __disable_irq()  __asm volatile ("cpsid i" ::: "memory")
#define __enable_irq()   __asm volatile ("cpsie i" ::: "memory")

/* Memory barriers */
#define __DSB()  __asm volatile ("dsb 0xF" ::: "memory")
#define __ISB()  __asm volatile ("isb 0xF" ::: "memory")
#define __DMB()  __asm volatile ("dmb 0xF" ::: "memory")

#endif /* HAL_STUBS_H */
