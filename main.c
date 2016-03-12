/* {{{1 License
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

	***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
	***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
	the FAQ page "My application does not run, what could be wrong?".  Have you
	defined configASSERT()?

	http://www.FreeRTOS.org/support - In return for receiving this top quality
	embedded software for free we request you assist our global community by
	participating in the support forum.

	http://www.FreeRTOS.org/training - Investing in training allows your team to
	be as productive as possible as early as possible.  Now you can receive
	FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
	Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!

    Author:
      Dr. Johann Pfefferl <johann.pfefferl@siemens.com>
      Siemens AG
}}} */

/* {{{1 Includes */
#include <stdint.h>
#include "inmate.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* }}} */

#ifdef CONFIG_UART_OXPCIE952
#define UART_BASE       0xe010
#else
#define UART_BASE       0x3f8
#endif

/* {{{1 Defines */
#define TIMER_IRQ 27
#define BEATS_PER_SEC configTICK_RATE_HZ
#define X86_SLEEP asm volatile("hlt" : : : "memory")
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

#define UART_BUFSIZE 72

#define UART_LOCK xSemaphoreTake(uart_mutex, portMAX_DELAY)
#define UART_UNLOCK xSemaphoreGive(uart_mutex)
#define UART_OUTPUT(args...) do { if(pdPASS == UART_LOCK) { printk(args); UART_UNLOCK;} } while(0)

/* }}} */

/* {{{1 Prototypes */
void vPortTimerHandler( void );
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationIRQHandler( unsigned ulICCIAR );
void __div0(void);
/* }}} */

/* {{{1 Global variables */
static TaskHandle_t uart_task_handle;
static SemaphoreHandle_t uart_mutex;
/* }}} */

/* {{{1 FreeRTOS debug hooks */

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
  printk("assert\n");
  volatile unsigned long ul = 0;

  ( void ) pcFile;
  ( void ) ulLine;

  vTaskSuspendAll();
  taskENTER_CRITICAL();
  {
    /* Set ul to a non-zero value using the debugger to step out of this
       function. */
    printk("%s %s: line=%lu\n", __func__, pcFile, ulLine);
    while( ul == 0 ) {
      portNOP();
    }
  }
  taskEXIT_CRITICAL();
}

void vApplicationMallocFailedHook( void )
{
  printk("malloc failed\n");
  /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available in the FreeRTOS heap.  pvPortMalloc() is called
     internally by FreeRTOS API functions that create tasks, queues, software
     timers, and semaphores.  The size of the FreeRTOS heap is set by the
     configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  taskDISABLE_INTERRUPTS();
  printk("%s\n", __func__);
  while(1) {
    portNOP();
  }
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;
  printk("stack overflow\n");
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  vTaskSuspendAll();
  taskDISABLE_INTERRUPTS();
    printk("%s task=%s\n", __func__, pcTaskName);
  for( ;; )
    X86_SLEEP;
}

void __div0(void)
{
  printk("PANIC: Div by zero error\n");
  X86_SLEEP;
}

static int32_t timer_value_for_period;
static unsigned timer_frq;

static inline void timer_on(void)
{
	/* arm_write_sysreg(CNTV_CTL_EL0, 1); */
}

static inline void timer_set(int32_t val)
{
	/* arm_write_sysreg(CNTV_TVAL_EL0, val); */

}

static inline void timer_set_next_event(void)
{
  int32_t time_drift;
  /* The timer indicates an overtime with a negative value inside this register */
  /* arm_read_sysreg(CNTV_TVAL_EL0, time_drift); */
  /* If the drift is greater than timer_value_for_period we have lost a time period */
  //configASSERT(-time_drift < timer_value_for_period);
  /* Correct next period by this time drift. The drift is caused by the software */
	timer_set(timer_value_for_period + time_drift);
}

/* Function called by FreeRTOS_Tick_Handler as last action */
void vClearTickInterrupt(void)
{
  printk("clear tick\n");
  timer_set_next_event();
	//timer_on();
}

static int timer_init(unsigned beats_per_second)
{
	timer_value_for_period = timer_frq / beats_per_second;
	timer_set(timer_value_for_period);
  timer_on();

	return 0;
}
/* }}} */

/* {{{1 UART handling */
static void serial_print(char *buf, int n)
{
  buf[n] = 0;
  UART_OUTPUT("TUA\t%d %s\n", n, buf);
}

void vConfigureTickInterrupt(void)
{
  /* Register the standard FreeRTOS Cortex-A tick handler as the timer's
     interrupt handler.  The handler clears the interrupt using the
     configCLEAR_TICK_INTERRUPT() macro, which is defined in FreeRTOSConfig.h. */
  printk("configure tick interrupt\n");
  timer_init(BEATS_PER_SEC);
}

void vApplicationIRQHandler(unsigned int irqn)
{
  switch(irqn) {
    case TIMER_IRQ:
      //timer_off();
      //FreeRTOS_Tick_Handler();
      printk("timer irq\n");
      vPortTimerHandler();
      break;
    case 0x3ff:
      /* This irq should be ignored. It is no longer relevant */
      break;
    default:
      printk("Spurious irq %d\n", irqn);
      break;
  }
}

/* }}} */

void printTask(void *pvParam)
{
  char *str = (char *)pvParam;
  while(1) {
    printk(str);
    vTaskDelay(20);
  }
}

#define USE_CACHE_MMU 1

/* {{{1 main */
void inmate_main(void)
{
  unsigned i;

  printk_uart_base = UART_BASE;
  uart_mutex = xSemaphoreCreateMutex();

  char* str1 = "Task1\n";
  char* str2 = "Task2\n";
  char* str3 = "Task3\n";

  xTaskCreate(printTask, "Task1", configMINIMAL_STACK_SIZE, (void *)str1, 1, NULL);
  printk("Task1 is created\n");

  xTaskCreate(printTask, "Task2", configMINIMAL_STACK_SIZE, (void *)str2, 1, NULL);
  printk("Task2 is created\n");

  xTaskCreate(printTask, "Task3", configMINIMAL_STACK_SIZE, (void *)str3, 1, NULL);
  printk("Task3 is created\n");

  vTaskStartScheduler();
  while(1);
  return 0;
}
/* }}} */

/* vim:foldmethod=marker
 ;*/
