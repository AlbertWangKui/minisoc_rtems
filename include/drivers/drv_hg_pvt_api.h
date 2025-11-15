/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_hg_pvt_api.h
 * @author taohb@starsmicrosystem.com
 * @date 2024.08.27
 * @brief hg pvt driver.
 */

#ifndef __DRV_HG_PVT_API_H__
#define __DRV_HG_PVT_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"

#define DTS_TEMPERATURE_TESTPOINT_NUMBER_MAX (16)

typedef struct DtsChTempCaliParam {
    U32 paramM;
    U32 paramN;
    U32 testPointNum;
    /* the number of param_k must be == testpoint_num */
    U32 paramK[DTS_TEMPERATURE_TESTPOINT_NUMBER_MAX];
} DtsChTempCaliParam_s;

typedef void (*dtsTempAlarmIsrCb)(U32 channel);

/**
 * @brief get pvt channel number.
 * @return pvt channel number.
 */
U32 pvtGetChannelNum(void);

/**
 * @brief get tmon temperature testpoint number for the specail channel.
 * @param [in] channel, channel id of pvt.
 * @return temperature testpoint number for the specail channel.
 */
U32 tmonGetTempTestpointNum(U32 channel);

/**
 * @brief get dts temperature testpoint number for the specail channel.
 * @param [in] channel, channel id of pvt.
 * @return temperature testpoint number for the specail channel.
 */
U32 dtsGetTempTestpointNum(U32 channel);

/**
 * @brief get dts voltage testpoint number for the specail channel.
 * @param [in] channel, channel id of pvt.
 * @return voltage testpoint number for the specail channel.
 */
U32 dtsGetVoltTestpointNum(U32 channel);

/**
 * @brief Interface for get current processor temperature by tmon.
 * @param [in] channel, channel id to get temperature.
 * @param [in] testpoint, test point of current channel.
 * @param [out] temperature, Pointer to storage processor temperature value.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 tmonGetTemperature(U32 channel, U32 testpoint, float *temperature);

/**
 * @brief Interface for get current processor temperature by dts.
 * @param [in] channel, channel id to get temperature.
 * @param [in] testpoint, test point of current channel.
 * @param [out] temperature, Pointer to storage processor temperature value.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsGetTemperature(U32 channel, U32 testpoint, float *temperature);

/**
 * @brief get voltage origin value from dts. This is a test function.
 * @param [in] channel, channel id of pvt.
 * @param [out] valueArray, array to storage volatage value, which size is testPoints.
 * @param [in] testPoints, testpoint number for valueArray which should be less than
 *      current channel's max volatage testPoints.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsVoltRegValueRead(U32 channel, U32 *valueArray,
    U32 testPoints);

/**
 * @brief import temperature calibration parameter of dts.
 * @param [in] channel, channel id.
 * @param [in] caliParam, dts temperature's calibration paramter, it's usally saved in efuse by ate.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTempCaliParamImport(U32 channel,
    DtsChTempCaliParam_s *caliParam);

/**
 * @brief set dts temperature threshold.
 * @param [in] channel, channel id.
 * @param [in] temperatureThreshold, unit is Centigrade.
 * @param [in] isrCb, isr callback.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 dtsTempAlarmThresholdSet(U32 channel, S16 temperatureThreshold,
    dtsTempAlarmIsrCb isrCb);

/**
 * @brief enable pvt channel interrupt. interrupt is disable by default.
 * @param [in] channel, channel id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 pvtInterruptEnable(U32 channel);

/**
 * @brief disable pvt channel interrupt. interrupt is disable by default.
 * @param [in] channel, channel id.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 pvtInterruptDisable(U32 channel);

/**
 * @brief pvt init. 
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 * @note interrupt is disable by default.
 */
S32 pvtInit(void);

/**
 * @brief pvt deinitialization
 * @return -EXIT_FAILURE or EXIT_SUCCESS.
 */
S32 pvtDeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _HG_PVT_H */
