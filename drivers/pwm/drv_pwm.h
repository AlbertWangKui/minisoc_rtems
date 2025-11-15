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
#ifndef __DRV_PWM_H__
#define __DRV_PWM_H__

#ifdef __cplusplus
    extern "C" {
#endif

#include "bsp_sbr.h"
#include "sbr_api.h"

#define PWM_DUTY_CYCLE_MAX      (100)
#define PWM_DATA_REG(n)         (0x30 + (n)*4)
#define PWM_CAPTURE_MODE_REG(n) (0x18 + (((n)-1) / 2) * 4)

/**
 * pwm control register
 */
typedef struct PwmCtrl {
    U32 countEn : 1;      ///< bit       0   0-禁止; 1-使能计数器
    U32 updateDis : 1;    ///< bit       1   0 -允许更新; 1-禁止
    U32 rsvd1 : 2;        ///< bits      2-3
    U32 direction : 1;    ///< bit       4   0 -向上计数; 1-向下计数
    U32 centerAlign : 2;  ///< bits      5-6 00 -边沿对齐模式，根据dir other -中央对齐模式
    U32 AutoReloadEn : 1; ///< bit       7   0-tim_arr没有缓冲，写入就刷新；1- 更新事件刷新
    U32 ClockDiv : 2;     ///< bits      8-9 分频因子
    U32 rsvd2 : 22;       ///< bits      10-31
} PwmCtrl_s;

/**
 * pwm interrupt enable
 */
typedef struct PwmIntrCtrl {
    U32 rsvd1 : 1;       ///< bit       0
    U32 chan1IntrEn : 1; ///< bit       1   1-通道1中断使能
    U32 chan2IntrEn : 1; ///< bit       2   1-通道2中断使能
    U32 chan3IntrEn : 1; ///< bit       3   1-通道3中断使能
    U32 chan4IntrEn : 1; ///< bits      4   1-通道4中断使能
    U32 rsvd2 : 2;       ///< bit       5-6
    U32 brakeIntrEn : 1; ///< bit       7   1-刹车中断使能
    U32 rsvd3 : 24;      ///< bits      8-31
} PwmIntrCtrl_s;

/**
 * pwm interrupt Reg
 */
typedef struct PwmIntr {
    U32 rsvd1 : 1;     ///< bit       0
    U32 chan1Intr : 1; ///< bit       1   读通道1中断，写1中断清零
    U32 chan2Intr : 1; ///< bit       2   读通道2中断，写1中断清零
    U32 chan3Intr : 1; ///< bit       3   读通道3中断，写1中断清零
    U32 chan4Intr : 1; ///< bits      4   读通道4中断，写1中断清零
    U32 rsvd2 : 2;     ///< bit       5-6
    U32 brakeIntr : 1; ///< bit       7   读刹车中断，写1中断清零
    U32 rsvd3 : 24;    ///< bits      8-31
} PwmIntr_s;

/**
 * pwm internal Flush Reg
 */
typedef struct PwmInternalFlush {
    U32 timeUpdateEvent : 1; ///< bit       0   写1会发生一次更新事件
    U32 rsvd1 : 31;          ///< bits      1-31
} PwmInternalFlush_s;

/**
 * pwm capture Reg 控制 12 / 34
 */
typedef struct PwmOutCapture {
    U32 cc13Select : 2;    ///< bits      0-1  00-cc1配置为输出 01-cc1配置为输入
    U32 rsvd1 : 1;         ///< bit       2
    U32 oc13PreloadEn : 1; ///< bit       3   0-禁止预装载功能
    U32 oc13Ref : 3;       ///< bits      4-6   oc1 pwm动作设置
    U32 rsvd2 : 1;         ///< bit       7
    U32 cc24Select : 2;    ///< bits      8-9  00-cc2配置为输出 01-cc2配置为输入
    U32 rsvd3 : 1;         ///< bit       10
    U32 oc24PreloadEn : 1; ///< bit       11   0-禁止预装载功能
    U32 oc24Ref : 3;       ///< bits      12-14   oc2 pwm动作设置
    U32 rsvd4 : 17;        ///< bits      15-31
} PwmOutCapture_s;

/**
 * pwm capture Reg
 */
typedef struct PwmInCapture {
    U32 cc13Select : 2; ///< bits      0-1  00-cc1配置为输出 01-cc1配置为输入
    U32 rsvd1 : 2;      ///< bits      2-3
    U32 ic13Freq : 4;   ///< bits      4-7   采样率配置
    U32 cc24Select : 2; ///< bits      8-9  00-cc2配置为输出 01-cc2配置为输入
    U32 rsvd2 : 22;     ///< bits      10-31
} PwmInCapture_s;

/**
 * pwm capture Enable Reg
 */
typedef struct PwmCaptureCtrl {
    U32 cc1Enable : 1;           ///< bit      0  0-关闭 1-开启OC1信号正常输出
    U32 cc1Polarity : 1;         ///< bit      1  0-根据CC1配置输入输出相关
    U32 cc1ComOutpuEn : 1;       ///< bit      2  0-关闭 1-开启OC1N信号正常输出
    U32 cc1ComOuputPolarity : 1; ///< bit      3  0-OC1N高电平有效 1-OC1N低电平有效
    U32 cc2Enable : 1;           ///< bit      4  0-关闭 1-开启OC2信号正常输出
    U32 cc2Polarity : 1;         ///< bit      5  0-根据CC2配置输入输出相关
    U32 cc2ComOutpuEn : 1;       ///< bit      6  0-关闭 1-开启OC2N信号正常输出
    U32 cc2ComOuputPolarity : 1; ///< bit      7  0-OC2N高电平有效 1-OC2N低电平有效
    U32 cc3Enable : 1;           ///< bit      8  0-关闭 1-开启OC3信号正常输出
    U32 cc3Polarity : 1;         ///< bit      9  0-根据CC3配置输入输出相关
    U32 cc3ComOutpuEn : 1;       ///< bit      10 0-关闭 1-开启OC3N信号正常输出
    U32 cc3ComOuputPolarity : 1; ///< bit      11 0-OC3N高电平有效 1-OC3N低电平有效
    U32 cc4Enable : 1;           ///< bit      12 0-关闭 1-开启OC4信号正常输出
    U32 cc4Polarity : 1;         ///< bit      13 0-根据CC4配置输入输出相关
    U32 cc4ComOutpuEn : 1;       ///< bit      14 0-关闭 1-开启OC4N信号正常输出
    U32 cc4ComOuputPolarity : 1; ///< bit      15 0-OC4N高电平有效 1-OC4N低电平有效
    U32 rsvd2 : 16;              ///< bits     16-31
} PwmCaptureCtrl_s;

/**
 * pwm main count Reg
 */
typedef struct PwmMainCount {
    U32 mainCnt : 16; ///< bits      0-15  主计数器值
    U32 rsvd1 : 16;   ///< bits      16-31
} PwmMainCount_s;

/**
 * pwm 预分频 Reg
 */
typedef struct PwmPreDivClk {
    U32 divClk : 16; ///< bits      0-15  分频因子
    U32 rsvd1 : 16;  ///< bits      16-31
} PwmPreDivClk_s;

/**
 * pwm 自动装载 reg
 */
typedef struct PwmPreLoad {
    U32 preCount : 16; ///< bits      0-15  预装载初值
    U32 rsvd1 : 16;    ///< bits      16-31
} PwmPreLoad_s;

/**
 * pwm 重复执行计数器 reg
 */
typedef struct PwmRepeatCnt {
    U32 repCount : 8; ///< bits      0-7  重复执行计数器
    U32 rsvd1 : 24;   ///< bits      8-31
} PwmRepeatCnt_s;

/**
 * pwm 捕获比较 reg
 */
typedef struct PwmCaptureData {
    U32 dataCount : 16; ///< bits      0-15  捕获比较通道1/2/3/4 的值
    U32 rsvd1 : 16;     ///< bits      16-31
} PwmCaptureData_s;

/**
 * pwm 刹车控制 Reg
 */
typedef struct PwmBrakeCtrl {
    U32 deadTime : 8;      ///< bits       0-7   插入互补输出之间的死区持续时间
    U32 rsvd1 : 4;         ///< bits       8-11
    U32 brakeEnable : 1;   ///< bit       12   1-使能刹车功能
    U32 brakePolarity : 1; ///< bit       13   刹车极性信号是0或者1
    U32 rsvd2 : 18;        ///< bits      14-31
} PwmBrakeCtrl_s;

/**
 * reg offset
 */
typedef enum PwmRegOffset {
    PWM_CTRL          = 0x000,
    PWM_INTR_CTRL     = 0x00C,
    PWM_INTR          = 0x010,
    PWM_INTER_FLUSH   = 0x014,
    PWM_CAPTURE1      = 0x018,
    PWM_CAPTURE2      = 0x01C,
    PWM_CAPTURE_CTRL  = 0x020,
    PWM_MAIN_COUNT    = 0x024,
    PWM_PRE_DIV_CLK   = 0x028,
    PWM_PRE_LOAD      = 0x02C,
    PWM_REPEAT_CNT    = 0x030,
    PWM_CAPTURE_DATA1 = 0x034,
    PWM_CAPTURE_DATA2 = 0x038,
    PWM_CAPTURE_DATA3 = 0x03C,
    PWM_CAPTURE_DATA4 = 0x040,
    PWM_BRAKE_CTRL    = 0x044,
} PwmRegOffset_e;

typedef union PwmReg {
    U32 all;
    PwmCtrl_s pwmRegCtrl; ///< pwm control register
    PwmIntrCtrl_s pwmIntrCtrl; ///< pwm interrupt enable
    PwmIntr_s pwmIntr; ///< pwm interrupt Reg
    PwmInternalFlush_s pwmInterFlush; ///< pwm internal Flush Reg
    PwmOutCapture_s pwmOutCapture; ///< pwm capture Reg 控制 12 / 34
    PwmInCapture_s pwmInCapture; ///< pwm capture Reg
    PwmCaptureCtrl_s pwmCaptureCtrl; ///< pwm capture Enable Reg
    PwmMainCount_s pwmMaincnt; ///< pwm main count Reg
    PwmPreDivClk_s pwmPreDivClk; ///< pwm 预分频 Reg
    PwmPreLoad_s pwmPreLoad; ///< pwm 自动装载 reg
    PwmRepeatCnt_s pwmRepeatCnt; ///< pwm 重复执行计数器 reg
    PwmCaptureData_s pwmCaptureData; ///<  pwm 捕获比较 reg，channel 1/2/3/4 有四个寄存器
    PwmBrakeCtrl_s pwmBrakeCtrl; ///< pwm 刹车控制 Reg
} PwmReg_u;

typedef struct {
    SbrPwmCfg_s sbrCfg;
} PwmDrvData_s;

#ifdef __cplusplus
}
#endif

#endif
