/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_options.h
 * @author  tianye@tianye.com
 * @date    2020.08.04
 * @brief   porting task
 * @note    NA
 */

#ifndef __OSP_OPTIONS_H__
#define __OSP_OPTIONS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup ClassicOptions Classic API Options
 *
 * @ingroup RTEMSAPIClassic
 *
 * This encapsulates functionality related to the options argument
 * to Classic API blocking operations. The primary option is whether
 * or not a task is willing to wait for the operation to complete.
 */

/**
 *  The following type defines the control block used to manage
 *  option sets.
 */
typedef uint32_t OspOption;

/**
 *  The following constants define the individual options which may
 *  be used to compose an option set.
 */
#define OSP_DEFAULT_OPTIONS 0x00000000

/**
 *  This option constants indicates that the task is to wait on resource.
 */
#define OSP_WAIT 0x00000000
/**
 *  This option constants indicates that the task is to not wait on
 *  the resource.  If it is not available, return immediately with
 *  a status to indicate unsatisfied.
 */
#define OSP_NO_WAIT 0x00000001

/**
 *  This option constants indicates that the task wishes to wait until
 *  all events of interest are available.
 */
#define OSP_EVENT_ALL 0x00000000

/**
 *  This option constants indicates that the task wishes to wait until
 *  ANY events of interest are available.
 */
#define OSP_EVENT_ANY 0x00000002

/**
 * @defgroup rtems_interrupt_extension Interrupt Manager Extension
 *
 * @ingroup ClassicINTR
 *
 * In addition to the Classic API interrupt handler with a handle are
 * supported.  You can also install multiple shared handler for one interrupt
 * vector.
 */

/**
 * @brief Makes the interrupt handler unique.  Prevents other handler from
 * using the same interrupt vector.
 */
#define OSP_INTERRUPT_UNIQUE (0x00000001)

/**
 * @brief Allows that this interrupt handler may share a common interrupt
 * vector with other handler.
 */
#define OSP_INTERRUPT_SHARED (0x00000000)

/**
 * @brief Forces that this interrupt handler replaces the first handler with
 * the same argument.
 */
#define OSP_INTERRUPT_REPLACE (0x00000002)

#ifdef __cplusplus
}
#endif

#endif