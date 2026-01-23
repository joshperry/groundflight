/**
 * GroundFlight Target: RadioMaster Nexus
 * 
 * Board-specific initialization for STM32F722RET6
 */

#include "target.h"

/* ============================================================================
 * Global Variables
 * ============================================================================ */

uint32_t SystemCoreClock = 16000000U;  /* Updated by clock config */

static volatile uint32_t systick_ms = 0;

/* APB prescaler table - needed by HAL_RCC_GetPCLKx for UART baud rate calculation */
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

/* ============================================================================
 * System Initialization (called from startup)
 * ============================================================================ */

void SystemInit(void)
{
    /* Enable FPU (done in startup, but ensure it's on) */
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    
    /* Reset RCC to default state */
    RCC->CR |= RCC_CR_HSION;
    RCC->CFGR = 0;
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
    RCC->PLLCFGR = 0x24003010;  /* Default value */
    RCC->CR &= ~RCC_CR_HSEBYP;
    RCC->CIR = 0;  /* Disable all clock interrupts */
    
    /* Configure flash latency for higher speeds */
    FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;
    
    SystemCoreClock = 16000000U;
}

void SystemCoreClockUpdate(void)
{
    /* TODO: Calculate actual clock from RCC registers */
}

/* ============================================================================
 * Clock Configuration
 * ============================================================================ */

static void configure_system_clock(void)
{
    /* Configure for 216 MHz with USB 48 MHz
     * 
     * HSE = 8 MHz
     * PLL_M = 8   -> VCO input = 1 MHz
     * PLL_N = 432 -> VCO output = 432 MHz
     * PLL_P = 2   -> SYSCLK = 216 MHz
     * PLL_Q = 9   -> USB = 48 MHz
     */
    
    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    
    /* Configure PLL */
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |      /* PLLM = 8 */
                   (432 << RCC_PLLCFGR_PLLN_Pos) |    /* PLLN = 432 */
                   (0 << RCC_PLLCFGR_PLLP_Pos) |      /* PLLP = 2 (0 = /2) */
                   (9 << RCC_PLLCFGR_PLLQ_Pos) |      /* PLLQ = 9 */
                   RCC_PLLCFGR_PLLSRC_HSE;            /* HSE as source */
    
    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    
    /* Configure prescalers
     * AHB = SYSCLK / 1 = 216 MHz
     * APB1 = AHB / 4 = 54 MHz (max for APB1)
     * APB2 = AHB / 2 = 108 MHz (max for APB2)
     */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 |    /* AHB = SYSCLK */
                RCC_CFGR_PPRE1_DIV4 |   /* APB1 = AHB/4 */
                RCC_CFGR_PPRE2_DIV2;    /* APB2 = AHB/2 */
    
    /* Switch to PLL as system clock */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    /* Update SystemCoreClock */
    SystemCoreClock = 216000000U;
}

/* ============================================================================
 * Target Initialization
 * ============================================================================ */

void target_init(void)
{
    /* Configure system clocks for 216 MHz + USB 48 MHz */
    configure_system_clock();
    
    /* Initialize HAL (uses SystemCoreClock for timing) */
    HAL_Init();
    
    /* Configure SysTick for 1ms interrupt */
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure LED pins as output */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = (1 << LED1_GPIO_PIN) | (1 << LED2_GPIO_PIN);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
    
    /* Start with LEDs off */
    target_led_off();
}

void target_clock_init(void)
{
    /* Clock is configured in target_init() */
}

/* ============================================================================
 * LED Control
 * ============================================================================ */

void target_led_on(void)
{
    /* Active-low LED */
    HAL_GPIO_WritePin(LED1_GPIO_PORT, (1 << LED1_GPIO_PIN), GPIO_PIN_RESET);
}

void target_led_off(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_PORT, (1 << LED1_GPIO_PIN), GPIO_PIN_SET);
}

void target_led_toggle(void)
{
    HAL_GPIO_TogglePin(LED1_GPIO_PORT, (1 << LED1_GPIO_PIN));
}

/* ============================================================================
 * Timing
 * ============================================================================ */

uint32_t target_millis(void)
{
    return systick_ms;
}

void target_delay_ms(uint32_t ms)
{
    uint32_t start = systick_ms;
    while ((systick_ms - start) < ms) {
        /* Wait */
    }
}

/* ============================================================================
 * Reboot to Bootloader (DFU mode)
 * ============================================================================ */

/* STM32F7 system memory (built-in bootloader) base address */
#define SYSTEM_MEMORY_ADDR      0x1FF00000

/* Magic value for bootloader request */
#define BOOTLOADER_MAGIC        0xDEADBEEF

void target_reboot_to_bootloader(void)
{
    /* Disable interrupts during shutdown */
    __disable_irq();
    
    /* Soft disconnect USB - tell host we're gone */
    /* USB OTG device registers are at offset 0x800 from peripheral base */
    volatile uint32_t *USB_DEV_DCTL = (volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + 0x800 + 0x04);
    *USB_DEV_DCTL |= (1 << 1);  /* SDIS bit */
    USB_OTG_FS->GCCFG = 0;
    
    /* Small delay for USB to disconnect */
    for (volatile int i = 0; i < 50000; i++);
    
    /* Disable USB clock */
    RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
    __DSB();
    
    /* Enable PWR and backup domain access */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR1 |= PWR_CR1_DBP;
    while (!(PWR->CR1 & PWR_CR1_DBP));
    
    /* Enable LSI for RTC (backup registers need RTC enabled) */
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));
    
    /* Enable RTC with LSI */
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;  /* LSI as RTC source */
    RCC->BDCR |= RCC_BDCR_RTCEN;
    __DSB();
    
    /* Write magic to RTC backup register 0 */
    RTC->BKP0R = BOOTLOADER_MAGIC;
    __DSB();
    
    /* Re-enable interrupts for reset to work */
    __enable_irq();
    
    /* System reset */
    NVIC_SystemReset();
    
    while (1);
}

/**
 * Check for bootloader magic and jump if found
 * Called very early in startup, before SystemInit
 */
void check_bootloader_request(void)
{
    /* Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    for (volatile int i = 0; i < 100; i++);
    
    /* Enable backup domain access */
    PWR->CR1 |= PWR_CR1_DBP;
    for (volatile int i = 0; i < 100; i++);
    
    /* Check RTC backup register for magic */
    if (RTC->BKP0R == BOOTLOADER_MAGIC) {
        /* Clear magic */
        RTC->BKP0R = 0;
        __DSB();
        
        /* Enable SYSCFG clock */
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        for (volatile int i = 0; i < 100; i++);
        
        /* Remap system memory to 0x00000000 */
        SYSCFG->MEMRMP = 0x01;
        __DSB();
        __ISB();
        
        /* Get bootloader vectors */
        uint32_t bootloader_stack = *((volatile uint32_t *)SYSTEM_MEMORY_ADDR);
        uint32_t bootloader_entry = *((volatile uint32_t *)(SYSTEM_MEMORY_ADDR + 4));
        
        /* Set stack pointer and jump */
        __set_MSP(bootloader_stack);
        ((void (*)(void))bootloader_entry)();
        
        while (1);
    }
}

/* ============================================================================
 * SysTick Interrupt Handler
 * ============================================================================ */

void SysTick_Handler(void)
{
    systick_ms++;
    HAL_IncTick();  /* Required for HAL timeouts */
}

/* ============================================================================
 * HAL Required Callbacks
 * ============================================================================ */

uint32_t HAL_GetTick(void)
{
    return systick_ms;
}
