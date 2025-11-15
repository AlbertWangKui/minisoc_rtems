/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file        drv_timer_api.h
 * @author      zuomeng1@starsmicrosystem.com
 * @date        2025/08/09
 * @brief       timer driver head file for API
 * @note        NULL
 */
#ifndef __DRV_TIMER_API_H__
#define __DRV_TIMER_API_H__

#include "common_defines.h"
#include "bsp_device.h"

/**
 * @brief init timer.
 * @param [in] devId device id.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 timerInit(DevList_e devId);

/**
 * @brief deinit timer.
 * @param [in] devId device id.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 timerDeInit(DevList_e devId);

/**
 * @brief Set timer configure parameter.
 * @param [in] devId device id.
 * @param [in] mSec period ms.
 * @param [in] timeout timeout callback.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 timerSetCfg(DevList_e devId, U32 mSec, void (*timeout)(void));

/**
 * @brief Start timer.
 * @param [in] devId device id.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 timerStart(DevList_e devId);

/**
 * @brief Stop timer.
 * @param [in] devId device id.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 timerStop(DevList_e devId);

#endif /* __DRV_TIMER_API_H__ */
