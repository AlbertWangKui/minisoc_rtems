/*
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_qspi_ctrl_api.h
 * @author yuxy3@starsmicrosystem.com
 * @date 2025/06/04
 * @brief  qspictrl header file
 * @note
 */

#ifndef __DRV_QSPI_CTRL_API_H__
#define __DRV_QSPI_CTRL_API_H__

#include "common_defines.h"
#include "bsp_device.h"

/**
 * @brief  Qspictrl_s module initialization
 * @param [in] deviD : module ID number
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 qspictrlInit(DevList_e devId);

/**
 * @brief  Qspictrl_s write function
 * @param [in] devId : module ID number
 * @param [in] cs: chip select
 * @param [in] addr: destinate addr
 * @param [in] buf : Data
 * @param [in] size: Data size
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 qspictrlWrite(DevList_e devId, U8 cs, U8 addr, void *buf,  U32 size);

/**
 * @brief  Qspictrl_s read function
 * @param [in] DevList_e devId : module ID number
 * @param [in] cs: chip select
 * @param [in] addr: destinate addr
 * @param [in] buf : Data
 * @param [in] size: Data size
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 qspictrlRead(DevList_e devId, U8 cs, U8 addr, void *buf, U32 size);

#endif
