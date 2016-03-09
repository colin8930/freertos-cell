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
#include "string.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* }}} */

#ifdef CONFIG_UART_OXPCIE952
#define UART_BASE       0xe000
#else
#define UART_BASE       0x2f8
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

static void uartTask(void *pvParameters)
{
  uint32_t c;
  char s[80];
  int idx = 0;
  while(pdTRUE) {
    if(pdTRUE == xTaskNotifyWait(0, 0, &c, pdMS_TO_TICKS(250))) {
      s[idx] = c;
      if('\r' == s[idx] || idx >= sizeof(s)-1) {
        serial_print(s, idx);
        idx = 0;
      }
      else
        ++idx;
    }
    else if(idx) { /* Buffer not empty */
      serial_print(s, idx);
      idx = 0;
    }
  }
}

void vConfigureTickInterrupt( void )
{
  /* Register the standard FreeRTOS Cortex-A tick handler as the timer's
     interrupt handler.  The handler clears the interrupt using the
     configCLEAR_TICK_INTERRUPT() macro, which is defined in FreeRTOSConfig.h. */
  timer_init(BEATS_PER_SEC);
}

void vApplicationIRQHandler(unsigned int irqn)
{
  switch(irqn) {
    case TIMER_IRQ:
      //timer_off();
      //FreeRTOS_Tick_Handler();
      vPortTimerHandler();
      break;
    /* UART irq is not support now */
    /*
    case UART7_IRQ:
      handle_uart_irq();
      break;
    */
    case 0x3ff:
      /* This irq should be ignored. It is no longer relevant */
      break;
    default:
      printk("Spurious irq %d\n", irqn);
      break;
  }
}

/* }}} */

/* {{{1 FreeRTOS application tasks */

static void testTask( void *pvParameters )
{
  unsigned id = (unsigned)pvParameters;
  TickType_t period = ++id * pdMS_TO_TICKS(100);
  char buf[128];
  unsigned cnt = 0;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while(pdTRUE) {
    sprintf(buf, "T%02u\tperiod:%5u;\tloop:%5u;\ttick:%6u\n", id, (unsigned)period, cnt++, (unsigned)xTaskGetTickCount());
    UART_OUTPUT(buf);
#if 0
    if(0x7 == (0x7 & cnt)) /* Force a task switch */
      taskYIELD();
#else
    vTaskDelayUntil(&pxPreviousWakeTime, period);
#endif
  }
  vTaskDelete( NULL );
}

static void sendTask(void *pvParameters)
{
  TaskHandle_t recvtask = pvParameters;
  TickType_t pxPreviousWakeTime = xTaskGetTickCount();
  while(1) {
    UART_OUTPUT("Sending ...\n");
    xTaskNotify(recvtask, 0, eIncrement);
    vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(1000));
  }
}

static void recvTask(void *pvParameters)
{
  while(1) {
    uint32_t value;
    if(pdTRUE == xTaskNotifyWait(0, 0, &value, portMAX_DELAY)) {
      UART_OUTPUT("Value received: %u\n", (unsigned)value);
    }
    else {
      printk("No value received\n");
    }
  }
}

#define USE_CACHE_MMU 1

/* {{{1 main */
void inmate_main(void)
{
  unsigned i;

  printk_uart_base = UART_BASE;

  uart_mutex = xSemaphoreCreateMutex();

  xTaskCreate( uartTask, /* The function that implements the task. */
      "uartstat", /* The text name assigned to the task - for debug only; not used by the kernel. */
      configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
      NULL,                                                            /* The parameter passed to the task */
      configMAX_PRIORITIES-1, /* The priority assigned to the task. */
      &uart_task_handle );

  if(1) for(i = 0; i < 20; i++) {
    int prio = 1 + i % (configMAX_PRIORITIES-1);
    printk("Create task %u with prio %d\n", i, prio);
    xTaskCreate( testTask, /* The function that implements the task. */
        "test", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        (void*)i, 								/* The parameter passed to the task */
        prio, /* The priority assigned to the task. */
        NULL );								    /* The task handle is not required, so NULL is passed. */
  }

  if(1) { /* Task notification test */
    TaskHandle_t recv_task_handle;
    xTaskCreate( recvTask, /* The function that implements the task. */
        "receive", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        NULL, 								/* The parameter passed to the task */
        configMAX_PRIORITIES-2, /* The priority assigned to the task. */
        &recv_task_handle );		/* The task handle */
    xTaskCreate( sendTask, /* The function that implements the task. */
        "sender", /* The text name assigned to the task - for debug only; not used by the kernel. */
        configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
        recv_task_handle, 				/* The parameter passed to the task */
        configMAX_PRIORITIES-1, /* The priority assigned to the task. */
        NULL );								    /* The task handle is not required, so NULL is passed. */
  }
  vTaskStartScheduler();
  printk("vTaskStartScheduler terminated: strange!!!\n");
	while (1) {
    X86_SLEEP;
  }
}
/* }}} */

/* vim:foldmethod=marker
 */
