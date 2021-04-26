//____________________________________________________________________
// Copyright Info      : All Rights Reserved - (c) Abmiq Micro Inc.,
//
// Name                : nex_porting.c
//
// Creation Date       : 20 July 2020
//
// Description         : Contain functions that are FreeRTOS OS dependent and
//                       platform or architecture related porting functions
//
// Changed History     :
// <Date>        	<Author>        <Version>        <Description>
// 29 July 2020      Ambiq           0.9              FreeRTOS porting functions file
//____________________________________________________________________

#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

#include "nex_debug.h"

#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_bsp.h"

#define  FMC_BOARD_EMMC_TEST

#define MAX_MUTEXLOCK  (3 * MAX_SLOTS * MAX_SDHC)
#define MAX_TIMER      (2 * MAX_SLOTS * MAX_SDHC)
#define MAX_SEMAPHORE  (4 * MAX_SLOTS * MAX_SDHC)
#define MAX_TASKLET    (6 * MAX_SLOTS * MAX_SDHC)
#define MAX_COMPLETION (5 * MAX_SLOTS * MAX_SDHC)
#define MAX_WORKQUEUE  (1 * MAX_SLOTS * MAX_SDHC)

#define SEMAPHORE_MAX_COUNT 100

//
// Platform or Arch related porting functions
//

VOID M_NEX_DELAY_MILLISEC(ULONG ms)
{
    am_util_delay_ms(ms);
}

VOID M_NEX_DELAY_MIRCOSEC(ULONG us)
{
    am_util_delay_us(us);
}

VOID Nex_Request_Irq(UINT32 irq)
{
    NVIC_SetPriority((IRQn_Type)irq, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)irq);
}

VOID Nex_Free_Irq(UINT32 irq)
{
    NVIC_DisableIRQ((IRQn_Type)irq);
}

INT32 Nex_Hw_Init(VOID)
{
    UINT32 retval;

    //
    // Power on the SDIO peripheral
    //
    retval = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_SDIO);

    if (AM_HAL_STATUS_FAIL == retval)
        return FAILURE;

    //
    // Enable the clock to SDIO peripheral
    //
    MCUCTRL->SDIOCTRL |= 0x3;

    //
    // Enable SDIO PINs
    //
    am_bsp_sdio_pins_enable(SDIO_BUS_WIDTH_4);

    //
    // Enable the SDIO CD (GPIO75) & WP (GPIO74) pins
    // remapping to FPGA GP36 and GP35
    //
    GPIO->SDIFCDWP_b.SDIFCD = 75;
    GPIO->SDIFCDWP_b.SDIFWP = 74;
        
#ifdef FMC_BOARD_EMMC_TEST
    //
    // board level shift control for eMMC
    //
    am_hal_gpio_pinconfig(76, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(76);
#else
    //
    // board level shift control for SD card
    //
    am_hal_gpio_pinconfig(76, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_set(76);    
#endif

    am_hal_interrupt_master_enable();

    return SUCCESS;
}

INT32 Nex_Hw_DeInit(VOID)
{
    UINT32 retval;

    am_bsp_sdio_pins_disable(SDIO_BUS_WIDTH_4);

    //
    // Disable the clock to SDIO peripheral
    //
    MCUCTRL->SDIOCTRL &= ~0x3;
    
    retval = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_SDIO);
    return AM_HAL_STATUS_FAIL == retval ? FAILURE : SUCCESS;
}

static struct sdhc_host_info SDHC_Inst = {
    .regBaseAddr[0][0] = SDIO_BASE,
    .irq[0]            = SDIO_IRQn,
    .slots[0]          = 1,
    .sdhc_cnt          = 1,
    .dma_level         = SDMA_MODE,
    .debug_level       = 1,
    .io_volt           = 0,
};

struct sdhc_host_info *GetSdhcHostInfo(void)
{
    return &SDHC_Inst;
}

//
// common data structure which maintains the function pointer and data
// of workqueue/tasklet/timer
//
typedef VOID (*nex_work_func_t)(ULONG data);

struct Nex_work_struct
{
    nex_work_func_t func;
    ULONG data;
};

//
// Tasklet & workqueue mapping
//

//
// Nex RTOS Tasks' configurations
//
#define configTaskLetSTACK_SIZE   512
#define configWorkQueueSTACK_SIZE 512

//
// the WQ Task is handling the card Insert & remove
// so set the highest priority (but lower than timer daemon task) to it 
// and make sure this task will be executed immediately
// when return from the ISR
//
#define tskWorkQueuePriority configTIMER_TASK_PRIORITY - 1 // 2

//
// TaskLet as generic defered ISR service of the IRQ
//
#define tskTaskLetPriority configTIMER_TASK_PRIORITY - 2 // 1

static TaskHandle_t xTaskLetHandle;
static TaskHandle_t xWorkQueueHandle;

static EventGroupHandle_t xTaskLetEventGroup;
static EventBits_t xTaskLetBits;

static INT32 tasklet_id = 0;
static struct Nex_work_struct nex_tsk[MAX_TASKLET];

static INT32 workq_id = 0;
static struct Nex_work_struct nex_wq[MAX_WORKQUEUE];

void vTaskLet_Task(void *data)
{
    int i;
    EventBits_t uxBits;
    
    while (1)
    {
    
        DBG_PRINT("vTaskLet_Task waiting for the event\n");

        uxBits = xEventGroupWaitBits(xTaskLetEventGroup, /* The event group being tested. */
                                     xTaskLetBits,       /* The bits within the event group to wait for. */
                                     pdTRUE,             /* should be cleared before returning. */
                                     pdFALSE,            /* Don't wait for both bits, either bit will do. */
                                     portMAX_DELAY       /* Wait for ever. */
        );

        for (i = 0; i < tasklet_id; i++)
        {
            if (uxBits & (0x1 << i))
                nex_tsk[i].func(nex_tsk[i].data);
        }
    }
}

INT32 Nex_Task_Init(VOID (*func)(ULONG), ULONG data)
{
    if (tasklet_id > (MAX_TASKLET - 1))
        return FAILURE;
    nex_tsk[tasklet_id].func = func;
    nex_tsk[tasklet_id].data = data;

    xTaskLetBits |= (0x1 << tasklet_id);
    tasklet_id++;

    return tasklet_id;
}


VOID Nex_Task_Schedule(INT32 task_id, UCHAR in_isr)
{
    DBG_TRACE_INIT();

    if (in_isr)
    {
        DBG_PRINT("In the context of ISR\n");
        BaseType_t xHigherPriorityTaskWoken, xResult;
        xResult = xEventGroupSetBitsFromISR(xTaskLetEventGroup, 0x1 << (task_id - 1),
                                            &xHigherPriorityTaskWoken);
        if (xResult != pdFAIL)
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        DBG_PRINT("In the context of Task\n");
        xEventGroupSetBits(xTaskLetEventGroup, 0x1 << (task_id - 1));
    }

    DBG_TRACE_EXIT();
}

VOID Nex_Task_Kill(INT32 task_id)
{
    xTaskLetBits &= ~(0x1 << (tasklet_id - 1));
}

void vWorkQueue_Task(void *data)
{
    while (1)
    {
        DBG_PRINT("vWorkQueue_Task is waiting for card insert & remove event\n");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DBG_PRINT("vWorkQueue_Task is detecting the card\n");
		//
		// There is only one workqueue task
		//
		nex_wq[0].func(nex_wq[0].data);
        DBG_PRINT("vWorkQueue_Task card detect done\n");
    }
}

INT32 Nex_Init_Work(VOID (*func)(ULONG), ULONG data)
{
    if (workq_id > (MAX_WORKQUEUE - 1))
        return FAILURE;
	
    nex_wq[workq_id].func = func;
    nex_wq[workq_id].data = data;

    workq_id++;

    return workq_id;
}

VOID Nex_Schedule_Work(INT32 workq_id)
{
    // Always invoked from the context of TaskLet Task
    xTaskNotifyGive(xWorkQueueHandle);
}

//
// Create and initialize the Thread and Event group
//
INT32 Nex_RTOS_Init(VOID)
{
    INT32 retval;

    DBG_TRACE_INIT();

    xTaskLetEventGroup = xEventGroupCreate();
    if (NULL == xTaskLetEventGroup)
        goto _exit0;

    retval = xTaskCreate(vTaskLet_Task, "TaskLet Task", configTaskLetSTACK_SIZE,
                         NULL, tskTaskLetPriority, &xTaskLetHandle);
    
    if (retval < 0)
        goto _exit1;
    
    DBG_PRINT("TaskLet Task is created\n");
    
    retval = xTaskCreate(vWorkQueue_Task, "WQ Task", configWorkQueueSTACK_SIZE,
                         NULL, tskWorkQueuePriority, &xWorkQueueHandle);

    if (retval < 0)
        goto _exit2;

    DBG_PRINT("vWorkQueue_Task is created\n");
    
    DBG_TRACE_EXIT();
    return SUCCESS;

_exit2:
    vTaskDelete(xTaskLetHandle);
_exit1:
    vEventGroupDelete(xTaskLetEventGroup);
_exit0:
    DBG_PRINT("Nex_RTOS_Init failed to create Tasks\n");
    DBG_TRACE_EXIT();
    return FAILURE;
}

//
// Timer mapping
//
static INT32 timer_id = 0;

static struct
{
    TimerHandle_t timerHandle;
    CHAR tname[12];
} timerlist[MAX_TIMER];

static struct Nex_work_struct nex_timer[MAX_TIMER];

static void Nex_Timer(TimerHandle_t pxTimer)
{
    int i, id = -1;
    for (i = 0; i < MAX_TIMER; i++)
    {
        if (pxTimer == timerlist[i].timerHandle)
        {
            id = i;
            break;
        }
    }
    nex_timer[id].func(nex_timer[id].data);
}

INT32 Nex_Timer_Init(VOID (*func)(ULONG), ULONG data)
{
    if (timer_id > (MAX_TIMER - 1))
        return FAILURE;
    nex_timer[timer_id].func = func;
    nex_timer[timer_id].data = data;
    timer_id++;

    am_util_stdio_sprintf(timerlist[timer_id - 1].tname, "NexTimer%1d", timer_id - 1);
    timerlist[timer_id - 1].timerHandle =
        xTimerCreate(timerlist[timer_id - 1].tname, pdMS_TO_TICKS(1000), pdFALSE, (void *)0, Nex_Timer);

    return timerlist[timer_id - 1].timerHandle == NULL ? FAILURE : timer_id;
}


//
// This function is called in the context of Task
//
INT32 M_NEX_MODIFY_TIMER(INT32 timer_id, ULONG expires)
{
    //
    // expire is microsecond
    //
    TickType_t ticks = pdMS_TO_TICKS(expires);

    return xTimerChangePeriod(timerlist[timer_id - 1].timerHandle, ticks, 100);
}

INT32 M_NEX_DELETE_TIMER(INT32 timer_id)
{
    return xTimerStop(timerlist[timer_id - 1].timerHandle, 100);
}

//
// Completion Mapping
//
SemaphoreHandle_t comp[MAX_COMPLETION];
static INT32 comp_id = 0;

INT32 M_NEX_INIT_COMPLETION(VOID)
{
    if (comp_id > (MAX_COMPLETION - 1))
        return FAILURE;
    comp[comp_id] = xSemaphoreCreateBinary();
    comp_id++;
    return comp[comp_id - 1] == NULL ? FAILURE : comp_id;
}

VOID Nex_Block_till_complete(INT32 comp_id)
{
    xSemaphoreTake(comp[comp_id - 1], portMAX_DELAY);
}

VOID M_NEX_COMPLETE(INT32 comp_id)
{
    //
    // In the context of 'nex_task_end' TaskLet and 'card_detect' WQ Task
    //
    xSemaphoreGive(comp[comp_id - 1]);
}

//
// Mutex
//
static INT32 mutex_id = 0;
SemaphoreHandle_t mutexs[MAX_MUTEXLOCK];

INT32 Nex_Mutex_Init(VOID)
{
    if (mutex_id > (MAX_MUTEXLOCK - 1))
        return FAILURE;

    mutexs[mutex_id++] = xSemaphoreCreateMutex();

    return mutexs[mutex_id - 1] == NULL ? FAILURE : mutex_id;
}

VOID Nex_Mutex_Lock(INT32 mutex_id)
{
    xSemaphoreTake(mutexs[mutex_id - 1], portMAX_DELAY);
}

VOID Nex_Mutex_Unlock(INT32 mutex_id)
{
    xSemaphoreGive(mutexs[mutex_id - 1]);
}

//
// Critical section protection 
//
VOID Nex_Critical_Enter(VOID)
{
    taskENTER_CRITICAL();
}

VOID Nex_Critical_Exit(VOID)
{
    taskEXIT_CRITICAL();
}

//
// Sleep waiting
//
VOID Nex_MSleep(UINT32 msecs)
{
    vTaskDelay(pdMS_TO_TICKS(msecs));
}

//
// Dynamic memory allocation
//
VOID *Nex_Alloc(UINT32 size)
{
    return pvPortMalloc((size_t)size);
}

static void vProcessPortFree(void *pvParameter1, uint32_t ulParameter2)
{
    const VOID *p;

    //
    // the first parameter is the pointer to the allocated memory
    //
    p = (const VOID *)pvParameter1;

    // Free the allocated memory
    vPortFree((void *)p);
}

VOID Nex_Free(const VOID *p)
{
    BaseType_t xHigherPriorityTaskWoken;
    
    xHigherPriorityTaskWoken = pdFALSE;
    xTimerPendFunctionCallFromISR(vProcessPortFree, (VOID *)p, 0x0, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#define MAX_THREAD            1
#define NEX_THREAD_STACK_SIZE 256

//
// Actually only one task 'nex_api_thread' is created
// it is assigned the lowest priority
//
#define NEX_DEFAULT_PRIORITY (tskIDLE_PRIORITY + 1) // 1

struct Nex_Thread
{
    TaskHandle_t handle;
    CHAR name[10];
    INT32 (*func)(void *);
    VOID *arg;
} nex_thread[MAX_THREAD];

static INT32 thread_id = 0;

void Nex_Thread_func(void *arg)
{
    int i;
    struct Nex_Thread *pThread = (struct Nex_Thread *)arg;

    for (i = 0; i < MAX_THREAD; i++)
    {
        if (&nex_thread[i] == pThread)
        {
            pThread->func(pThread->arg);
            break;
        }
    }
}

INT32 Nex_Create_Thread(INT32 (*function)(VOID *), VOID *arg)
{
	BaseType_t xReturned;
	
    if (thread_id > (MAX_THREAD - 1))
        return FAILURE;

    am_util_stdio_sprintf(nex_thread[thread_id].name, "NexThread%1d", thread_id);
    nex_thread[thread_id].func = function;
    nex_thread[thread_id].arg = arg;
    xReturned = xTaskCreate(Nex_Thread_func, nex_thread[thread_id].name, NEX_THREAD_STACK_SIZE,
                            &nex_thread[thread_id], NEX_DEFAULT_PRIORITY, 
                            &nex_thread[thread_id].handle);

    if (xReturned == pdPASS)
        return ++thread_id;
    else
        return FAILURE;
}

INT32 Nex_Kill_Thread(INT32 thread_id)
{
    if (thread_id > (MAX_THREAD - 1))
        return FAILURE;

    TaskHandle_t xHandle = (TaskHandle_t)(nex_thread[thread_id - 1].handle);
    vTaskDelete(xHandle);

    return SUCCESS;
}

//
// Nex counting semaphore
//
static INT32 sema_id = 0;
SemaphoreHandle_t sema[MAX_SEMAPHORE];

INT32 Nex_Init_Semaphore(INT32 count)
{
    if (sema_id > (MAX_SEMAPHORE - 1))
        return FAILURE;

    sema[sema_id++] = xSemaphoreCreateCounting(SEMAPHORE_MAX_COUNT, count);

    return sema[sema_id - 1] == NULL ? FAILURE : sema_id;
}

INT32 Nex_Acquire_Semaphore(INT32 sema_id)
{
    return xSemaphoreTake(sema[sema_id - 1], portMAX_DELAY);
}

VOID Nex_Release_Semaphore(INT32 semd_id)
{
    xSemaphoreGive(sema[semd_id - 1]);
}

VOID Nex_DeInitOsdepVariables(VOID)
{
    timer_id  = 0;
    sema_id   = 0;
    mutex_id  = 0;
    comp_id   = 0;

    tasklet_id = 0;
    workq_id   = 0;
    thread_id  = 0;
}

