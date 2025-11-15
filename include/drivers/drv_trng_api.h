/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * 
 * @file drv_trng_api.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/05/19
 * @brief  
 * 
 * @par ChangeLog:
 * 
 * Date         Author          Description
 * 2025/05/19   yangzhl3        the first version
 * 
 * 
 */

#ifndef __TRNG_API_H__
#define __TRNG_API_H__

#include "common_defines.h"
#include "bsp_device.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  
 * @details none
 * @param [in] trngId
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EBUSY : -16
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 trngInit(DevList_e trngId);

/**
 * @brief  trng get random data to buffer
 * @details none
 * @param [in] trng_id
 * @param [out] random_out data output buffer
 * @param [in] lenDw is the count of U32, lenDw > 0
 *              eg: when lenDw > 0, output data len: lenDw * 4
 *                  when lenDw = 0, output data len: invalid
 * @return  EXIT_SUCCESS : 0
 *          -EXIT_FAILURE : -1
 *          -EIO : -5
 *          -EBUSY : -16
 *          -EINVAL : -22
 * @warning 阻塞；可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 trngGetRandom(DevList_e trng_id, U32 *random_out, U32 lenDw);

#ifdef __cplusplus
}
#endif

#endif /* __TRNG_API_H__ */
