/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 * 
 * @file drv_pwm.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/03/17
 * @brief  
 * 
 * @par ChangeLog:
 * 
 * Date         Author          Description
 * 2025/03/17   yangzhl3        移植自产品软件
 * 
 * 
 */
#ifndef __DRV_PWM_API_H__
#define __DRV_PWM_API_H__

#ifdef __cplusplus
    extern "C" {
#endif

#include "common_defines.h"
#include "bsp_device.h"

/**
 * PWM 输出通道模式枚举
 */
typedef enum {
    PWM_OC_DIS = 0,  ///< oc disable
    PWM_OC_HIGH,     ///< force high
    PWM_OC_LOW,      ///< force low
    PWM_OC_RESVER,   ///< resver
    PWM_OC_INV_LOW,  ///< low
    PWM_OC_INV_HIGH, ///< high
    PWM_OC_MODE1, ///< pwm 向上计数，一旦数值小于比较寄存器值时通道1为有效电平，向下计数时，同条件1为无效电平，0为有效
    PWM_OC_MODE2, ///< pwm 与上相反
} PwmOcMode_e;


/**
 * PWM 极性配置
 */
typedef enum {
    PWM_POLARITY_NORMAL = 0,
    PWM_POLARITY_REVERSE,
} PwmOutputPolarity_s;

/**
 * PWM 配置参数
 */
typedef struct {
    U32 chid;
    U32 freq;
    U32 duty;
    U32 prescaler;
    U32 polarity;
    U32 preload;
}PwmOutParameter_s;

/**
 * @brief 初始化PWM模块：注册设备到系统、获取设备信息
 * @param [in] devId,PWM设备枚举ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmInit(DevList_e  devId);

/**
 * @brief 去初始化设备：关闭PWM设备下所有通道，卸载驱动
 * @param [in] devId,PWM设备枚举ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmDeInit(DevList_e devId);

/**
 * @brief 配置PWM参数
 * @param [in] devId,PWM设备枚举ID
 * @param [in] cfg,PWM参数,包括通道ID，周期、占空比、分频系数、输出极性和计数器预装载值
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmSetCfg(DevList_e devId, PwmOutParameter_s *cfg);

/**
 * @brief 设置PWM波输出频率
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [in] freq,PWM波输出频率
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmSetFreq(DevList_e devId, U8 chid,  U32 freq);

/**
 * @brief 设置占空比
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [in] duty,占空比（0~100），值除以100为占空比
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmSetDuty(DevList_e devId, U8 chid,  U32 duty);

/**
 * @brief 开启PWM
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmStart(DevList_e devId, U32 chid);

/**
 * @brief 关闭PWM
 * @param [in] devId,PWM设备枚举ID
 * @param [in] chid,PWM 某个设备的通道ID
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 pwmStop(DevList_e devId, U32 chid);


#ifdef __cplusplus
}
#endif

#endif
