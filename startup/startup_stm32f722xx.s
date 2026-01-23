/**
 * GroundFlight Startup Code
 * Target: STM32F722RET6 (ARM Cortex-M7)
 * 
 * Based on STM32CubeF7 startup template, simplified for GroundFlight.
 */

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-sp-d16
    .thumb

/* Global symbols from linker script */
.global g_pfnVectors
.global Default_Handler

/* Start address for .data section (in flash) */
.word _sidata
/* Start address for .data section (in RAM) */
.word _sdata
/* End address for .data section */
.word _edata
/* Start address for .bss section */
.word _sbss
/* End address for .bss section */
.word _ebss

/**
 * @brief  Reset Handler - Entry point after reset
 */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer */
    ldr   sp, =_estack

    /* Check for reboot-to-bootloader request FIRST, before anything else */
    bl    check_bootloader_request

    /* Copy .data section from Flash to RAM */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
    movs  r3, #0
    b     LoopCopyDataInit

CopyDataInit:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4

LoopCopyDataInit:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   CopyDataInit

    /* Zero fill .bss section */
    ldr   r2, =_sbss
    ldr   r4, =_ebss
    movs  r3, #0
    b     LoopFillZerobss

FillZerobss:
    str   r3, [r2]
    adds  r2, r2, #4

LoopFillZerobss:
    cmp   r2, r4
    bcc   FillZerobss

    /* Enable FPU (Coprocessors CP10 and CP11) */
    ldr   r0, =0xE000ED88
    ldr   r1, [r0]
    orr   r1, r1, #(0xF << 20)
    str   r1, [r0]
    dsb
    isb

    /* Call SystemInit (clock setup) */
    bl    SystemInit

    /* Call static constructors (C++ only, but harmless) */
    bl    __libc_init_array

    /* Call main() */
    bl    main

    /* If main returns, loop forever */
LoopForever:
    b     LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  Default Handler for unused interrupts
 */
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
    b     Infinite_Loop
.size Default_Handler, .-Default_Handler

/**
 * @brief  Interrupt Vector Table
 * 
 * Note: Only commonly used vectors are named. Others default to Default_Handler.
 */
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    /* Core exceptions */
    .word _estack               /* 0x00: Initial Stack Pointer */
    .word Reset_Handler         /* 0x04: Reset */
    .word NMI_Handler           /* 0x08: NMI */
    .word HardFault_Handler     /* 0x0C: Hard Fault */
    .word MemManage_Handler     /* 0x10: Memory Management */
    .word BusFault_Handler      /* 0x14: Bus Fault */
    .word UsageFault_Handler    /* 0x18: Usage Fault */
    .word 0                     /* 0x1C: Reserved */
    .word 0                     /* 0x20: Reserved */
    .word 0                     /* 0x24: Reserved */
    .word 0                     /* 0x28: Reserved */
    .word SVC_Handler           /* 0x2C: SVCall */
    .word DebugMon_Handler      /* 0x30: Debug Monitor */
    .word 0                     /* 0x34: Reserved */
    .word PendSV_Handler        /* 0x38: PendSV */
    .word SysTick_Handler       /* 0x3C: SysTick */

    /* STM32F722 External Interrupts */
    .word WWDG_IRQHandler               /* 0x40: Window Watchdog */
    .word PVD_IRQHandler                /* 0x44: PVD through EXTI */
    .word TAMP_STAMP_IRQHandler         /* 0x48: Tamper and TimeStamp */
    .word RTC_WKUP_IRQHandler           /* 0x4C: RTC Wakeup */
    .word FLASH_IRQHandler              /* 0x50: Flash */
    .word RCC_IRQHandler                /* 0x54: RCC */
    .word EXTI0_IRQHandler              /* 0x58: EXTI Line 0 */
    .word EXTI1_IRQHandler              /* 0x5C: EXTI Line 1 */
    .word EXTI2_IRQHandler              /* 0x60: EXTI Line 2 */
    .word EXTI3_IRQHandler              /* 0x64: EXTI Line 3 */
    .word EXTI4_IRQHandler              /* 0x68: EXTI Line 4 */
    .word DMA1_Stream0_IRQHandler       /* 0x6C: DMA1 Stream 0 */
    .word DMA1_Stream1_IRQHandler       /* 0x70: DMA1 Stream 1 */
    .word DMA1_Stream2_IRQHandler       /* 0x74: DMA1 Stream 2 */
    .word DMA1_Stream3_IRQHandler       /* 0x78: DMA1 Stream 3 */
    .word DMA1_Stream4_IRQHandler       /* 0x7C: DMA1 Stream 4 */
    .word DMA1_Stream5_IRQHandler       /* 0x80: DMA1 Stream 5 */
    .word DMA1_Stream6_IRQHandler       /* 0x84: DMA1 Stream 6 */
    .word ADC_IRQHandler                /* 0x88: ADC1, ADC2, ADC3 */
    .word CAN1_TX_IRQHandler            /* 0x8C: CAN1 TX */
    .word CAN1_RX0_IRQHandler           /* 0x90: CAN1 RX0 */
    .word CAN1_RX1_IRQHandler           /* 0x94: CAN1 RX1 */
    .word CAN1_SCE_IRQHandler           /* 0x98: CAN1 SCE */
    .word EXTI9_5_IRQHandler            /* 0x9C: EXTI Lines 5-9 */
    .word TIM1_BRK_TIM9_IRQHandler      /* 0xA0: TIM1 Break / TIM9 */
    .word TIM1_UP_TIM10_IRQHandler      /* 0xA4: TIM1 Update / TIM10 */
    .word TIM1_TRG_COM_TIM11_IRQHandler /* 0xA8: TIM1 Trigger / TIM11 */
    .word TIM1_CC_IRQHandler            /* 0xAC: TIM1 Capture Compare */
    .word TIM2_IRQHandler               /* 0xB0: TIM2 */
    .word TIM3_IRQHandler               /* 0xB4: TIM3 */
    .word TIM4_IRQHandler               /* 0xB8: TIM4 */
    .word I2C1_EV_IRQHandler            /* 0xBC: I2C1 Event */
    .word I2C1_ER_IRQHandler            /* 0xC0: I2C1 Error */
    .word I2C2_EV_IRQHandler            /* 0xC4: I2C2 Event */
    .word I2C2_ER_IRQHandler            /* 0xC8: I2C2 Error */
    .word SPI1_IRQHandler               /* 0xCC: SPI1 */
    .word SPI2_IRQHandler               /* 0xD0: SPI2 */
    .word USART1_IRQHandler             /* 0xD4: USART1 */
    .word USART2_IRQHandler             /* 0xD8: USART2 */
    .word USART3_IRQHandler             /* 0xDC: USART3 */
    .word EXTI15_10_IRQHandler          /* 0xE0: EXTI Lines 10-15 */
    .word RTC_Alarm_IRQHandler          /* 0xE4: RTC Alarm A/B */
    .word OTG_FS_WKUP_IRQHandler        /* 0xE8: USB OTG FS Wakeup */
    .word TIM8_BRK_TIM12_IRQHandler     /* 0xEC: TIM8 Break / TIM12 */
    .word TIM8_UP_TIM13_IRQHandler      /* 0xF0: TIM8 Update / TIM13 */
    .word TIM8_TRG_COM_TIM14_IRQHandler /* 0xF4: TIM8 Trigger / TIM14 */
    .word TIM8_CC_IRQHandler            /* 0xF8: TIM8 Capture Compare */
    .word DMA1_Stream7_IRQHandler       /* 0xFC: DMA1 Stream 7 */
    .word FMC_IRQHandler                /* 0x100: FMC */
    .word SDMMC1_IRQHandler             /* 0x104: SDMMC1 */
    .word TIM5_IRQHandler               /* 0x108: TIM5 */
    .word SPI3_IRQHandler               /* 0x10C: SPI3 */
    .word UART4_IRQHandler              /* 0x110: UART4 */
    .word UART5_IRQHandler              /* 0x114: UART5 */
    .word TIM6_DAC_IRQHandler           /* 0x118: TIM6 / DAC1&2 underrun */
    .word TIM7_IRQHandler               /* 0x11C: TIM7 */
    .word DMA2_Stream0_IRQHandler       /* 0x120: DMA2 Stream 0 */
    .word DMA2_Stream1_IRQHandler       /* 0x124: DMA2 Stream 1 */
    .word DMA2_Stream2_IRQHandler       /* 0x128: DMA2 Stream 2 */
    .word DMA2_Stream3_IRQHandler       /* 0x12C: DMA2 Stream 3 */
    .word DMA2_Stream4_IRQHandler       /* 0x130: DMA2 Stream 4 */
    .word 0                             /* 0x134: Reserved */
    .word 0                             /* 0x138: Reserved */
    .word 0                             /* 0x13C: Reserved */
    .word 0                             /* 0x140: Reserved */
    .word 0                             /* 0x144: Reserved */
    .word 0                             /* 0x148: Reserved */
    .word OTG_FS_IRQHandler             /* 0x14C: USB OTG FS */
    .word DMA2_Stream5_IRQHandler       /* 0x150: DMA2 Stream 5 */
    .word DMA2_Stream6_IRQHandler       /* 0x154: DMA2 Stream 6 */
    .word DMA2_Stream7_IRQHandler       /* 0x158: DMA2 Stream 7 */
    .word USART6_IRQHandler             /* 0x15C: USART6 */
    .word I2C3_EV_IRQHandler            /* 0x160: I2C3 Event */
    .word I2C3_ER_IRQHandler            /* 0x164: I2C3 Error */
    .word OTG_HS_EP1_OUT_IRQHandler     /* 0x168: USB OTG HS EP1 Out */
    .word OTG_HS_EP1_IN_IRQHandler      /* 0x16C: USB OTG HS EP1 In */
    .word OTG_HS_WKUP_IRQHandler        /* 0x170: USB OTG HS Wakeup */
    .word OTG_HS_IRQHandler             /* 0x174: USB OTG HS */
    .word 0                             /* 0x178: Reserved */
    .word AES_IRQHandler                /* 0x17C: AES */
    .word RNG_IRQHandler                /* 0x180: RNG */
    .word FPU_IRQHandler                /* 0x184: FPU */
    .word UART7_IRQHandler              /* 0x188: UART7 */
    .word UART8_IRQHandler              /* 0x18C: UART8 */
    .word SPI4_IRQHandler               /* 0x190: SPI4 */
    .word SPI5_IRQHandler               /* 0x194: SPI5 */
    .word 0                             /* 0x198: Reserved */
    .word SAI1_IRQHandler               /* 0x19C: SAI1 */
    .word 0                             /* 0x1A0: Reserved */
    .word 0                             /* 0x1A4: Reserved */
    .word 0                             /* 0x1A8: Reserved */
    .word SAI2_IRQHandler               /* 0x1AC: SAI2 */
    .word QUADSPI_IRQHandler            /* 0x1B0: QuadSPI */
    .word LPTIM1_IRQHandler             /* 0x1B4: LPTIM1 */
    .word 0                             /* 0x1B8: Reserved */
    .word 0                             /* 0x1BC: Reserved */
    .word 0                             /* 0x1C0: Reserved */
    .word 0                             /* 0x1C4: Reserved */
    .word 0                             /* 0x1C8: Reserved */
    .word 0                             /* 0x1CC: Reserved */
    .word SDMMC2_IRQHandler             /* 0x1D0: SDMMC2 */

/**
 * @brief  Weak aliases for interrupt handlers
 * 
 * Override these in your C code to handle specific interrupts.
 */
    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    /* Device-specific weak aliases */
    .weak WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler

    .weak PVD_IRQHandler
    .thumb_set PVD_IRQHandler, Default_Handler

    .weak TAMP_STAMP_IRQHandler
    .thumb_set TAMP_STAMP_IRQHandler, Default_Handler

    .weak RTC_WKUP_IRQHandler
    .thumb_set RTC_WKUP_IRQHandler, Default_Handler

    .weak FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler

    .weak RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler

    .weak EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler

    .weak EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler

    .weak EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler

    .weak EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler

    .weak EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler

    .weak DMA1_Stream0_IRQHandler
    .thumb_set DMA1_Stream0_IRQHandler, Default_Handler

    .weak DMA1_Stream1_IRQHandler
    .thumb_set DMA1_Stream1_IRQHandler, Default_Handler

    .weak DMA1_Stream2_IRQHandler
    .thumb_set DMA1_Stream2_IRQHandler, Default_Handler

    .weak DMA1_Stream3_IRQHandler
    .thumb_set DMA1_Stream3_IRQHandler, Default_Handler

    .weak DMA1_Stream4_IRQHandler
    .thumb_set DMA1_Stream4_IRQHandler, Default_Handler

    .weak DMA1_Stream5_IRQHandler
    .thumb_set DMA1_Stream5_IRQHandler, Default_Handler

    .weak DMA1_Stream6_IRQHandler
    .thumb_set DMA1_Stream6_IRQHandler, Default_Handler

    .weak ADC_IRQHandler
    .thumb_set ADC_IRQHandler, Default_Handler

    .weak CAN1_TX_IRQHandler
    .thumb_set CAN1_TX_IRQHandler, Default_Handler

    .weak CAN1_RX0_IRQHandler
    .thumb_set CAN1_RX0_IRQHandler, Default_Handler

    .weak CAN1_RX1_IRQHandler
    .thumb_set CAN1_RX1_IRQHandler, Default_Handler

    .weak CAN1_SCE_IRQHandler
    .thumb_set CAN1_SCE_IRQHandler, Default_Handler

    .weak EXTI9_5_IRQHandler
    .thumb_set EXTI9_5_IRQHandler, Default_Handler

    .weak TIM1_BRK_TIM9_IRQHandler
    .thumb_set TIM1_BRK_TIM9_IRQHandler, Default_Handler

    .weak TIM1_UP_TIM10_IRQHandler
    .thumb_set TIM1_UP_TIM10_IRQHandler, Default_Handler

    .weak TIM1_TRG_COM_TIM11_IRQHandler
    .thumb_set TIM1_TRG_COM_TIM11_IRQHandler, Default_Handler

    .weak TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    .weak TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler

    .weak TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler

    .weak TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler, Default_Handler

    .weak I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler

    .weak I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler

    .weak I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler, Default_Handler

    .weak I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler, Default_Handler

    .weak SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler

    .weak USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler

    .weak USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler

    .weak USART3_IRQHandler
    .thumb_set USART3_IRQHandler, Default_Handler

    .weak EXTI15_10_IRQHandler
    .thumb_set EXTI15_10_IRQHandler, Default_Handler

    .weak RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler, Default_Handler

    .weak OTG_FS_WKUP_IRQHandler
    .thumb_set OTG_FS_WKUP_IRQHandler, Default_Handler

    .weak TIM8_BRK_TIM12_IRQHandler
    .thumb_set TIM8_BRK_TIM12_IRQHandler, Default_Handler

    .weak TIM8_UP_TIM13_IRQHandler
    .thumb_set TIM8_UP_TIM13_IRQHandler, Default_Handler

    .weak TIM8_TRG_COM_TIM14_IRQHandler
    .thumb_set TIM8_TRG_COM_TIM14_IRQHandler, Default_Handler

    .weak TIM8_CC_IRQHandler
    .thumb_set TIM8_CC_IRQHandler, Default_Handler

    .weak DMA1_Stream7_IRQHandler
    .thumb_set DMA1_Stream7_IRQHandler, Default_Handler

    .weak FMC_IRQHandler
    .thumb_set FMC_IRQHandler, Default_Handler

    .weak SDMMC1_IRQHandler
    .thumb_set SDMMC1_IRQHandler, Default_Handler

    .weak TIM5_IRQHandler
    .thumb_set TIM5_IRQHandler, Default_Handler

    .weak SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler, Default_Handler

    .weak UART4_IRQHandler
    .thumb_set UART4_IRQHandler, Default_Handler

    .weak UART5_IRQHandler
    .thumb_set UART5_IRQHandler, Default_Handler

    .weak TIM6_DAC_IRQHandler
    .thumb_set TIM6_DAC_IRQHandler, Default_Handler

    .weak TIM7_IRQHandler
    .thumb_set TIM7_IRQHandler, Default_Handler

    .weak DMA2_Stream0_IRQHandler
    .thumb_set DMA2_Stream0_IRQHandler, Default_Handler

    .weak DMA2_Stream1_IRQHandler
    .thumb_set DMA2_Stream1_IRQHandler, Default_Handler

    .weak DMA2_Stream2_IRQHandler
    .thumb_set DMA2_Stream2_IRQHandler, Default_Handler

    .weak DMA2_Stream3_IRQHandler
    .thumb_set DMA2_Stream3_IRQHandler, Default_Handler

    .weak DMA2_Stream4_IRQHandler
    .thumb_set DMA2_Stream4_IRQHandler, Default_Handler

    .weak OTG_FS_IRQHandler
    .thumb_set OTG_FS_IRQHandler, Default_Handler

    .weak DMA2_Stream5_IRQHandler
    .thumb_set DMA2_Stream5_IRQHandler, Default_Handler

    .weak DMA2_Stream6_IRQHandler
    .thumb_set DMA2_Stream6_IRQHandler, Default_Handler

    .weak DMA2_Stream7_IRQHandler
    .thumb_set DMA2_Stream7_IRQHandler, Default_Handler

    .weak USART6_IRQHandler
    .thumb_set USART6_IRQHandler, Default_Handler

    .weak I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler, Default_Handler

    .weak I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler, Default_Handler

    .weak OTG_HS_EP1_OUT_IRQHandler
    .thumb_set OTG_HS_EP1_OUT_IRQHandler, Default_Handler

    .weak OTG_HS_EP1_IN_IRQHandler
    .thumb_set OTG_HS_EP1_IN_IRQHandler, Default_Handler

    .weak OTG_HS_WKUP_IRQHandler
    .thumb_set OTG_HS_WKUP_IRQHandler, Default_Handler

    .weak OTG_HS_IRQHandler
    .thumb_set OTG_HS_IRQHandler, Default_Handler

    .weak AES_IRQHandler
    .thumb_set AES_IRQHandler, Default_Handler

    .weak RNG_IRQHandler
    .thumb_set RNG_IRQHandler, Default_Handler

    .weak FPU_IRQHandler
    .thumb_set FPU_IRQHandler, Default_Handler

    .weak UART7_IRQHandler
    .thumb_set UART7_IRQHandler, Default_Handler

    .weak UART8_IRQHandler
    .thumb_set UART8_IRQHandler, Default_Handler

    .weak SPI4_IRQHandler
    .thumb_set SPI4_IRQHandler, Default_Handler

    .weak SPI5_IRQHandler
    .thumb_set SPI5_IRQHandler, Default_Handler

    .weak SAI1_IRQHandler
    .thumb_set SAI1_IRQHandler, Default_Handler

    .weak SAI2_IRQHandler
    .thumb_set SAI2_IRQHandler, Default_Handler

    .weak QUADSPI_IRQHandler
    .thumb_set QUADSPI_IRQHandler, Default_Handler

    .weak LPTIM1_IRQHandler
    .thumb_set LPTIM1_IRQHandler, Default_Handler

    .weak SDMMC2_IRQHandler
    .thumb_set SDMMC2_IRQHandler, Default_Handler
