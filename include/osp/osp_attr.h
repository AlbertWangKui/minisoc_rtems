/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_attr.h
 * @author  tianye
 * @date    2020.09.09
 * @brief   osp属性定义
 * @note    NA
 */

#ifndef __OSP_ATTR_H__
#define __OSP_ATTR_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  This defines the type used to contain Classic API attributes.  These
 *  are primarily used when creating objects.
 */
typedef uint32_t OspAttribute;

/**
 * This is the default value for an attribute set.
 */
#define OSP_DEFAULT_ATTRIBUTES 0x00000000

/**
 *  This is the attribute constant to indicate local resource.
 */
#define OSP_LOCAL 0x00000000

/**
 *  This is the attribute constant to indicate global resource.
 */
#define OSP_GLOBAL 0x00000002

/**
 *  This is the attribute constant which reflects that blocking
 *  tasks will be managed using FIFO discipline.
 */
#define OSP_FIFO 0x00000000

/**
 *  This is the attribute constant which reflects that blocking
 *  tasks will be managed using task priority discipline.
 */
#define OSP_PRIORITY 0x00000004

/**
 * OSP Task Specific Attributes
 */

/**
 *  This attribute constant indicates that the task will not use the
 *  floating point hardware.  If the architecture permits it, then
 *  the FPU will be disabled when the task is executing.
 */
#define OSP_NO_FLOATING_POINT 0x00000000

/**
 *  This attribute constant indicates that the task will use the
 *  floating point hardware.  There will be a floating point
 *  context associated with this task.
 */
#define OSP_FLOATING_POINT 0x00000001

/**
 * OSP Semaphore Specific Attributes
 */

/**
 *  This is the mask for the attribute bits associated with the
 *  Classic API Semaphore Manager.
 */
#define OSP_SEMAPHORE_CLASS 0x00000030

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will be a counting semaphore.
 */
#define OSP_COUNTING_SEMAPHORE 0x00000000

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will be a proper binary semaphore or mutex.
 */
#define OSP_BINARY_SEMAPHORE 0x00000010

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will be a simple binary semaphore.
 */
#define OSP_SIMPLE_BINARY_SEMAPHORE 0x00000020

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will NOT use the Priority Inheritance Protocol.
 */
#define OSP_NO_INHERIT_PRIORITY 0x00000000

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will use the Priority Inheritance Protocol.
 *
 *  @note The semaphore instance must be a binary semaphore.
 */
#define OSP_INHERIT_PRIORITY 0x00000040

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will NOT use the Priority Ceiling Protocol.
 */
#define OSP_NO_PRIORITY_CEILING 0x00000000

/**
 *  This attribute constant indicates that the Classic API Semaphore
 *  instance created will use the Priority Ceiling Protocol.
 *
 *  @note The semaphore instance must be a binary semaphore.
 */
#define OSP_PRIORITY_CEILING 0x00000080

/**
 *  This attribute constant indicates that the Classic API Semaphore instance
 *  created will NOT use the Multiprocessor Resource Sharing Protocol.
 */
#define OSP_NO_MULTIPROCESSOR_RESOURCE_SHARING 0x00000000

/**
 *  This attribute constant indicates that the Classic API Semaphore instance
 *  created will use the Multiprocessor Resource Sharing Protocol.
 *
 *  @note The semaphore instance must be a binary semaphore.
 */
#define OSP_MULTIPROCESSOR_RESOURCE_SHARING 0x00000100

/**
 * OSP Barrier Specific Attributes
 */

/**
 *  This attribute constant indicates that the Classic API Barrier
 *  instance created will use an automatic release protocol.
 */
#define OSP_BARRIER_AUTOMATIC_RELEASE 0x00000010

/**
 *  This attribute constant indicates that the Classic API Barrier
 *  instance created will use the manual release protocol.
 */
#define OSP_BARRIER_MANUAL_RELEASE 0x00000000

/**
 * OSP Internal Task Specific Attributes
 */

/**
 *  This attribute constant indicates that the task was created
 *  by the application using normal Classic API methods.
 */
#define OSP_APPLICATION_TASK 0x00000000

/**
 *  This attribute constant indicates that the task was created
 *  by OSP as a support task.
 */
#define OSP_SYSTEM_TASK 0x00008000

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
