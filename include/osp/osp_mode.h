/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_mode.h
 * @author  tianye
 * @date    2020.09.09
 * @brief   osp模式定义
 * @note    NA
 */

#ifndef __OSP_MODE_H__
#define __OSP_MODE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @defgroup ClassicModes Modes
 *
 *  @ingroup RTEMSAPIClassic
 *
 *  This encapsulates functionality related to the task modes supported
 *  by the Classic API Task Manager.
 */
/**@{*/

/**
 *  The following type defines the control block used to manage
 *  each a mode set.
 */
typedef uint32_t OspMode;

/**
 *  The following constants define the individual modes and masks
 *  which may be used to compose a mode set and to alter modes.
 */
#define OSP_ALL_MODE_MASKS 0x0000ffff

/**
 *  This mode constant is the default mode set.
 */
#define OSP_DEFAULT_MODES 0x00000000

/**
 *  This mode constant is used when the user wishes to obtain their
 *  current execution mode.
 */
#define OSP_CURRENT_MODE 0

///< This mode constant corresponds to the timeslice enable/disable bit.
#define OSP_TIMESLICE_MASK 0x00000200

///< This mode constant corresponds to the preemption enable/disable bit.
#define OSP_PREEMPT_MASK 0x00000100

///< This mode constant corresponds to the signal enable/disable bit.
#define OSP_ASR_MASK 0x00000400

/**
 * This mode constant corresponds to the interrupt enable/disable bits.
 * 不同体系结构体rtems定义的不一样，参考 CPU_MODES_INTERRUPT_MASK , arm定义的是0x1
 */
#define OSP_INTERRUPT_MASK 0x1

///< This mode constant is used to indicate preemption is enabled.
#define OSP_PREEMPT 0x00000000
///< This mode constant is used to indicate preemption is disabled.
#define OSP_NO_PREEMPT 0x00000100

///< This mode constant is used to indicate timeslicing is disabled.
#define OSP_NO_TIMESLICE 0x00000000
///< This mode constant is used to indicate timeslicing is enabled.
#define OSP_TIMESLICE 0x00000200

///< This mode constant is used to indicate signal processing is enabled.
#define OSP_ASR 0x00000000
///< This mode constant is used to indicate signal processing is disabled.
#define OSP_NO_ASR 0x00000400

#ifdef __cplusplus
}
#endif

#endif
