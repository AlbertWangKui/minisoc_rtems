/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_interrupt.h
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   porting inner interrupt
 * @note    NA
 */

#ifndef __OPS_INNER_INTERRUPT_H
#define __OPS_INNER_INTERRUPT_H

#ifndef __OPS_INTERRUPT_H
#error "include osp_interrupt.h instead include this header file"
#endif

#include <rtems.h>
#include <rtems/score/processormask.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef rtems_interrupt_level InnerOspInterruptLevel_t;

typedef rtems_interrupt_lock InnerOspInterruptLock_t;

typedef rtems_interrupt_lock_context InnerOspInterruptLockContext_t;

#define INNER_OSP_INTERRUPT_LOCK_ACQUIRE(_lock, _lock_context) rtems_interrupt_lock_acquire(_lock, _lock_context)

#define INNER_OSP_INTERRUPT_LOCK_RELEASE(_lock, _lock_context) rtems_interrupt_lock_release(_lock, _lock_context)

#define INNER_OSP_INTERRUPT_LOCK_ACQUIRE_ISR(_lock, _lock_context)                                                     \
    rtems_interrupt_lock_acquire_isr(_lock, _lock_context)

#define INNER_OSP_INTERRUPT_LOCK_RELEASE_ISR(_lock, _lock_context)                                                     \
    rtems_interrupt_lock_release_isr(_lock, _lock_context)

#define INNER_OSP_INTERRUPT_LOCK_INITIALIZE(_lock, _name) rtems_interrupt_lock_initialize(_lock, _name)

#define INNER_OSP_INTERRUPT_LOCK_DESTROY(_lock) rtems_interrupt_lock_destroy(_lock)

#define INNER_OSP_INTERRUPT_LOCAL_DISABLE(_isr_cookie) rtems_interrupt_local_disable(_isr_cookie)

#define INNER_OSP_INTERRUPT_LOCAL_ENABLE(_isr_cookie) rtems_interrupt_local_enable(_isr_cookie)

#define INNER_OSP_INTERRUPT_LOCK_INITIALIZER(_name) RTEMS_INTERRUPT_LOCK_INITIALIZER(_name)

#ifdef __cplusplus
}
#endif

#endif
