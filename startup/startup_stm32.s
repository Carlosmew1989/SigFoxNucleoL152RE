/**
  ******************************************************************************
  * @file      startup_stm32.s
  * @author    Ac6
  * @version   V1.0.0
  * @date      12-June-2014
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m3
  .thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss

.equ  BootRAM,        0xF1E0F85F
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:

/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs r3, #0
 	str  r3, [r2]
	adds r2, r2, #4

LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss

/* Call the clock system intitialization function.*/
    bl  SystemInit
/* Call static constructors */
    bl __libc_init_array
/* Call the application's entry point.*/
	bl	main

LoopForever:
    b LoopForever

.size	Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex-M.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	SysTick_Handler
    .word   WWDG_IRQHandler                   /* Window WatchDog              */
  	.word   PVD_IRQHandler                    /* PVD through EXTI Line detection */
  	.word	TAMPER_STAMP_IRQHandler   		/* Tamper and Time Stamp */
  	.word 	RTC_WKUP_IRQHandler
  	.word 	FLASH_IRQHandler
  	.word	RCC_IRQHandler
  	.word	EXTI0_IRQHandler
  	.word	EXTI1_IRQHandler
  	.word	EXTI2_IRQHandler
  	.word	EXTI3_IRQHandler
  	.word	EXTI4_IRQHandler
  	.word	DMA1_Channel1_IRQHandler
  	.word	DMA1_Channel2_IRQHandler
  	.word	DMA1_Channel3_IRQHandler
  	.word	DMA1_Channel4_IRQHandler
  	.word	DMA1_Channel5_IRQHandler
  	.word	DMA1_Channel6_IRQHandler
  	.word	DMA1_Channel7_IRQHandler
  	.word	ADC1_IRQHandler
  	.word	USB_HP_IRQHandler
  	.word	USB_LP_IRQHandler
  	.word   DAC_IRQHandler            /* DAC */
    .word   COMP_IRQHandler           /* COMP through EXTI Line*/
    .word   EXTI9_5_IRQHandler        /* EXTI Line 9..5*/
    .word   LCD_IRQHandler            /* LCD*/
    .word   TIM9_IRQHandler           /* TIM9*/
    .word   TIM10_IRQHandler          /* TIM10*/
    .word   TIM11_IRQHandler          /* TIM11*/
    .word   TIM2_IRQHandler           /* TIM2*/
    .word   TIM3_IRQHandler           /* TIM3*/
    .word   TIM4_IRQHandler           /* TIM4*/
    .word   I2C1_EV_IRQHandler        /* I2C1 Event*/
    .word   I2C1_ER_IRQHandler        /* I2C1 Error*/
    .word   I2C2_EV_IRQHandler        /* I2C2 Event*/
    .word   I2C2_ER_IRQHandler        /* I2C2 Error*/
    .word   SPI1_IRQHandler           /* SPI1*/
    .word   SPI2_IRQHandler           /* SPI2*/
    .word   USART1_IRQHandler         /* USART1*/
    .word   USART2_IRQHandler         /* USART2*/
    .word   USART3_IRQHandler         /* USART3*/
    .word   EXTI15_10_IRQHandler      /* EXTI Line 15..10*/
    .word   RTC_Alarm_IRQHandler      /* RTC Alarm through EXTI Line*/
    .word   USB_FS_WKUP_IRQHandler    /* USB FS Wakeup from suspend*/
    .word   TIM6_IRQHandler           /* TIM6*/
    .word   TIM7_IRQHandler           /* TIM7*/
    .word   SDIO_IRQHandler           /* SDIO*/
    .word   TIM5_IRQHandler           /* TIM5  */
    .word   SPI3_IRQHandler           /* SPI3*/
    .word   UART4_IRQHandler          /* UART4*/
    .word   UART5_IRQHandler          /* UART5*/
    .word   DMA2_Channel1_IRQHandler  /* DMA2 Channel 1*/
    .word   DMA2_Channel2_IRQHandler  /* DMA2 Channel 2*/
    .word   DMA2_Channel3_IRQHandler  /* DMA2 Channel 3*/
    .word   DMA2_Channel4_IRQHandler  /* DMA2 Channel 4*/
    .word   DMA2_Channel5_IRQHandler  /* DMA2 Channel 5*/
    .word   AES_IRQHandler            /* AES*/
    .word   COMP_ACQ_IRQHandler       /* Comparator Channel Acquisition*/

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

  	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

  	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak	PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler

	.weak	TAMPER_STAMP_IRQHandler
	.thumb_set TAMPER_STAMP_IRQHandler,Default_Handler

	.weak	RTC_WKUP_IRQHandler
	.thumb_set RTC_WKUP_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler

	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler

	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler

	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler

	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler

	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler

	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler

	.weak	ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler

	.weak	USB_HP_IRQHandler
	.thumb_set USB_HP_IRQHandler,Default_Handler

	.weak	USB_LP_IRQHandler
	.thumb_set USB_LP_IRQHandler,Default_Handler

	.weak	DAC_IRQHandler
	.thumb_set DAC_IRQHandler,Default_Handler

	.weak	COMP_IRQHandler
	.thumb_set COMP_IRQHandler,Default_Handler

	.weak	EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler

	.weak	LCD_IRQHandler
	.thumb_set LCD_IRQHandler,Default_Handler

	.weak	TIM9_IRQHandler
	.thumb_set TIM9_IRQHandler,Default_Handler

	.weak	TIM10_IRQHandler
	.thumb_set TIM10_IRQHandler,Default_Handler

	.weak	TIM11_IRQHandler
	.thumb_set TIM11_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler

	.weak	TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler

	.weak	I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak	USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler

	.weak	EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler

	.weak	RTC_Alarm_IRQHandler
	.thumb_set RTC_Alarm_IRQHandler,Default_Handler

	.weak	USB_FS_WKUP_IRQHandler
	.thumb_set USB_FS_WKUP_IRQHandler,Default_Handler

	.weak	TIM6_IRQHandler
	.thumb_set TIM6_IRQHandler,Default_Handler

	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler

	.weak	SDIO_IRQHandler
	.thumb_set SDIO_IRQHandler,Default_Handler

	.weak	TIM5_IRQHandler
	.thumb_set TIM5_IRQHandler,Default_Handler

	.weak	SPI3_IRQHandler
	.thumb_set SPI3_IRQHandler,Default_Handler

	.weak	UART4_IRQHandler
	.thumb_set UART4_IRQHandler,Default_Handler

	.weak	UART5_IRQHandler
	.thumb_set UART5_IRQHandler,Default_Handler

	.weak	DMA2_Channel1_IRQHandler
	.thumb_set DMA2_Channel1_IRQHandler,Default_Handler

	.weak	DMA2_Channel2_IRQHandler
	.thumb_set DMA2_Channel2_IRQHandler,Default_Handler

	.weak	DMA2_Channel3_IRQHandler
	.thumb_set DMA2_Channel3_IRQHandler,Default_Handler

	.weak	DMA2_Channel4_IRQHandler
	.thumb_set DMA2_Channel4_IRQHandler,Default_Handler

	.weak	DMA2_Channel5_IRQHandler
	.thumb_set DMA2_Channel5_IRQHandler,Default_Handler

	.weak	AES_IRQHandler
	.thumb_set AES_IRQHandler,Default_Handler

	.weak	COMP_ACQ_IRQHandler
	.thumb_set COMP_ACQ_IRQHandler,Default_Handler

/*	.weak	SystemInit */

/************************ (C) COPYRIGHT Ac6 *****END OF FILE****/
