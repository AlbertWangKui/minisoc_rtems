/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file clock.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/24
 * @brief RTEMS时钟驱动
 */
#include <bsp_common.h>
#include <rtems.h>
#include <rtems/timecounter.h>
#include <errno.h>
#include "common_defines.h"
#include "bsp_config.h"
#include "bsp_api.h"
#include "stars_timer.h"

#define SECOND_TO_MICROSEC_RATIO    (1000000)
#define RTEMS_FATAL_SOURCE_DEAD     (0xdeadbeef)
#define BSP_TIMER_DIV_FACTOR        (3)

static TmrReg_s *timerTick = (TmrReg_s *)SYS_TICK_TIMER_BASE_ADRS;
static TmrReg_s *timerHr = (TmrReg_s *)SYS_HR_TIMER_BASE_ADRS;
static struct timecounter bspHrTimer;

static U32 bspGetHrTimerCount(struct timecounter *tc)
{
    return timerHr->rdlVal;
}

U32 cpu_counter_ticks(void)
{
    return timerHr->rdlVal;
}

void bsp_clock_driver_support_initialize_hardware(void)
{
    U32 sysClkOfTick = 0;
    U32 sysClkOfHr = 0;
    U32 period = 0;
    U32 div = 0;

    period = rtems_configuration_get_microseconds_per_tick();

    if (peripsClockFreqGet(SYS_TICK_TIMER_DEV,&sysClkOfTick) != EXIT_SUCCESS) {
        sysClkOfTick = 0;
        goto exit;
    }
    if (peripsClockFreqGet(SYS_HR_TIMER_DEV,&sysClkOfHr) != EXIT_SUCCESS) {
        sysClkOfHr = 0;
        goto exit;
    }

    /* 设置系统tick时钟 */
    timerTick->tmrEn.fields.en = 0;
    timerTick->irqEn.fields.irqEn = 1;
    timerTick->irqStat.fields.irqStat = 1;
    timerTick->divisor.fields.divisor = BSP_TIMER_DIV_FACTOR;
    div = 1 + timerTick->divisor.fields.divisor;
    timerTick->setlVal = (U32)(((U64)period * (U64)sysClkOfTick) /
        (SECOND_TO_MICROSEC_RATIO * (U64)div));
    timerTick->sethVal = (((U64)period * (U64)sysClkOfTick) /
        (SECOND_TO_MICROSEC_RATIO * (U64)div)) >> 32;
    timerTick->tmrEn.dword = 0; /* clear oneshot */
    timerTick->tmrEn.fields.en = 1;

    /* 设置高精度时钟 */
    timerHr->tmrEn.fields.en = 0;
    timerHr->irqEn.fields.irqEn = 0;
    timerHr->irqStat.fields.irqStat = 1;
    timerHr->divisor.fields.divisor = BSP_TIMER_DIV_FACTOR;
    timerHr->setlVal = 0xffffffff;
    timerHr->sethVal = 0x0;
    timerHr->tmrEn.dword = 0; /* clear oneshot */
    timerHr->tmrEn.fields.en = 1;
    bspHrTimer.tc_get_timecount = bspGetHrTimerCount;
    bspHrTimer.tc_counter_mask = 0xffffffff;
    bspHrTimer.tc_frequency = sysClkOfHr / (BSP_TIMER_DIV_FACTOR + 1);
    bspHrTimer.tc_quality = RTEMS_TIMECOUNTER_QUALITY_CLOCK_DRIVER;
    rtems_timecounter_install(&bspHrTimer);

exit:
    if (sysClkOfTick == 0 || sysClkOfHr == 0) {
        rtems_fatal_error_occurred(RTEMS_FATAL_SOURCE_DEAD);
    }
    return;
}

void bsp_clock_driver_support_at_tick(void)
{
    timerTick->irqStat.fields.irqStat = 1;
}

void bsp_clock_driver_support_install_isr(rtems_isr_entry Clock_isr)
{
    rtems_status_code status = RTEMS_SUCCESSFUL;

    status = rtems_interrupt_handler_install(
        SYS_INT_NUM_TIMER0, "Clock", RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler)Clock_isr, NULL);

    if (status != RTEMS_SUCCESSFUL) {
        rtems_fatal_error_occurred(RTEMS_FATAL_SOURCE_DEAD);
    }
}

U32 _CPU_Counter_frequency(void)
{
    U32 clk = 0;

    if (peripsClockFreqGet(SYS_TICK_TIMER_DEV, &clk) != EXIT_SUCCESS) {
        clk = 0;
    }
    return clk / (BSP_TIMER_DIV_FACTOR + 1);
}

CPU_Counter_ticks _CPU_Counter_read(void)
{
    return cpu_counter_ticks();
}
