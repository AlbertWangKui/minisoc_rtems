/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_pwm.h
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @version 1.0
 * @brief  PWM driver header file for Minisoc platform
 * 
 * This file contains the register definitions, data structures and macros
 * for the PWM (Pulse Width Modulation) driver on the Minisoc platform.
 * It defines the hardware register layout, control structures and constants
 * needed to configure and control PWM channels.
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   yangkl         the first version
 *
 */

#ifndef __DRV_PWM_H__
#define __DRV_PWM_H__

#ifdef __cplusplus
    extern "C" {
#endif

#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

/* PWM Configuration Constants */
#define PWM_DUTY_CYCLE_MAX          (100)                    ///< Maximum duty cycle value (100%)
#define PWM_INTR_STAT_ALL           (0x9e)                   ///< Bit mask for all PWM interrupt status bits
#define PWM_MAX_PRESCALER           (0xffff)                 ///< Maximum prescaler value for PWM frequency division
#define PWM_MAX_PRELOAD             (0xffff)                 ///< Maximum preload value for PWM counter
#define PWM_LOCK_TIMEOUT_MS         (1000)                   ///< Timeout in milliseconds for device lock acquisition
#define PWM_CHANNEL_MAX             (4)                      ///< Maximum number of PWM channels supported
#define PWM_DUTY_MAX                (100)                    ///< Maximum duty cycle percentage
#define PWM_DUTY_COMPARE_MAX_VAL    (0xffff)                 ///< Maximum value for PWM duty compare register
#define PWM_CNT_FREQ                (1000000)                ///< PWM counter base frequency in Hz (1MHz)

/* PWM Calculation Macros */
#define PWM_GET_PRELOAD_CNT(pwmFreq)     ((U32)PWM_CNT_FREQ / pwmFreq)  ///< Calculate preload value: clk / psc / pwmFreq
#define PWM_GET_DUTY_CMPARE_VAL(preloadCnt, duty)  (((U32)(preloadCnt) * (duty)) / 100 + 1)  ///< Calculate duty compare value using integer arithmetic

/**
 * pwm control register
 */
typedef union PwmCtrl {
    struct __attribute__((aligned(4))) {
        U32 countEn : 1;      ///< bit 0: 0-Disable counter; 1-Enable counter
        U32 updateDis : 1;    ///< bit 1: 0-Allow updates; 1-Disable updates
        U32 rsvd1 : 2;        ///< bits 2-3: Reserved bits
        U32 direction : 1;    ///< bit 4: 0-Up counting; 1-Down counting
        U32 centerAlign : 2;  ///< bits 5-6: 00-Edge aligned mode, other-Center aligned mode
        U32 AutoReloadEn : 1; ///< bit 7: 0-ARR register not buffered; 1-ARR buffered
        U32 ClockDiv : 2;     ///< bits 8-9: Clock division factor
        U32 rsvd2 : 22;       ///< bits 10-31: Reserved bits
    };
    U32 dword;               ///< Full 32-bit register access
} PwmCtrl_s;

/**
 * pwm interrupt enable
 */
typedef union PwmIntrCtrl {
    struct __attribute__((aligned(4))) {
        U32 rsvd1 : 1;       ///< bit 0: Reserved bit
        U32 chan1IntrEn : 1; ///< bit 1: 1-Enable channel 1 interrupt
        U32 chan2IntrEn : 1; ///< bit 2: 1-Enable channel 2 interrupt
        U32 chan3IntrEn : 1; ///< bit 3: 1-Enable channel 3 interrupt
        U32 chan4IntrEn : 1; ///< bit 4: 1-Enable channel 4 interrupt
        U32 rsvd2 : 2;       ///< bits 5-6: Reserved bits
        U32 brakeIntrEn : 1; ///< bit 7: 1-Enable brake interrupt
        U32 rsvd3 : 24;      ///< bits 8-31: Reserved bits
    };
    U32 dword;               ///< Full 32-bit register access
} PwmIntrCtrl_s;

/**
 * pwm interrupt Reg
 */
typedef union PwmIntrStat {
    struct __attribute__((aligned(4))) {
        U32 rsvd1 : 1;     ///< bit 0: Reserved bit
        U32 chan1Intr : 1; ///< bit 1: Read channel 1 interrupt status, write 1 to clear
        U32 chan2Intr : 1; ///< bit 2: Read channel 2 interrupt status, write 1 to clear
        U32 chan3Intr : 1; ///< bit 3: Read channel 3 interrupt status, write 1 to clear
        U32 chan4Intr : 1; ///< bit 4: Read channel 4 interrupt status, write 1 to clear
        U32 rsvd2 : 2;     ///< bits 5-6: Reserved bits
        U32 brakeIntr : 1; ///< bit 7: Read brake interrupt status, write 1 to clear
        U32 rsvd3 : 24;    ///< bits 8-31: Reserved bits
    };
    U32 dword;             ///< Full 32-bit register access
} PwmIntrStat_s;

/**
 * pwm internal Flush Reg
 */
typedef union PwmInternalFlush {
    struct __attribute__((aligned(4))) {
        U32 timeUpdateEvent : 1; ///< bit 0: Write 1 to trigger an update event
        U32 rsvd1 : 31;          ///< bits 1-31: Reserved bits
    };
    U32 dword;                   ///< Full 32-bit register access
} PwmInternalFlush_s;

/**
 * pwm input capture Register
 */
typedef union PwmInCapture {
    struct __attribute__((aligned(4))) {
        U32 cc13Select : 2;     ///< bits 0-1: 00-CC1 configured as output, 01-CC1 configured as input
        U32 rsvd1 : 2;          ///< bits 2-3: Reserved bits
        U32 ic13Freq : 4;       ///< bits 4-7: Sampling frequency configuration
        U32 cc24Select : 2;     ///< bits 8-9: 00-CC2 configured as output, 01-CC2 configured as input
        U32 rsvd2 : 2;          ///< bits 10-11: Reserved bits
        U32 ic24Freq : 4;       ///< bits 12-15: Sampling frequency configuration
        U32 rsvd3 : 16;         ///< bits 16-31: Reserved bits
    };
    U32 dword;                   ///< Full 32-bit register access
} PwmInCapture_s;

/**
 * pwm output Register
 */
typedef union PwmOutCompare {
    struct __attribute__((aligned(4))) {
        U32 cc13Select : 2;     ///< bits 0-1: 00-CC1 configured as output, 01-CC1 configured as input
        U32 rsvd1 : 1;          ///< bit 2: Reserved bit
        U32 oc13PreloadEn : 1;  ///< bit 3: 0-Disable preload function, 1-Enable preload function
        U32 oc13Ref : 3;        ///< bits 4-6: OC1 PWM action setting
        U32 rsvd2 : 1;          ///< bit 7: Reserved bit
        U32 cc24Select : 2;     ///< bits 8-9: 00-CC2 configured as output, 01-CC2 configured as input
        U32 rsvd3 : 1;          ///< bit 10: Reserved bit
        U32 oc24PreloadEn : 1;  ///< bit 11: 0-Disable preload function, 1-Enable preload function
        U32 oc24Ref : 3;        ///< bits 12-14: OC2 PWM action setting
        U32 rsvd4 : 17;         ///< bits 15-31: Reserved bits
    };
    U32 dword;                   ///< Full 32-bit register access
} PwmOutCompare_s;

typedef union pwmInOutCtrl {
    PwmInCapture_s    pwmInCapture;   ///< Input capture configuration
    PwmOutCompare_s   pwmOutCompare;  ///< Output compare configuration
} PwmInOutCtrl_u;

/**
 * pwm capture Enable Reg
 */
typedef union PwmCapCmpCtrl {
    struct __attribute__((aligned(4))) {
        U32 cc1Enable : 1;           ///< bit 0: 0-Disable, 1-Enable OC1 signal output
        U32 cc1Polarity : 1;         ///< bit 1: 0-Normal polarity, 1-Inverted polarity
        U32 cc1ComOutpuEn : 1;       ///< bit 2: 0-Disable, 1-Enable OC1N signal output
        U32 cc1ComOuputPolarity : 1; ///< bit 3: 0-OC1N active high, 1-OC1N active low
        U32 cc2Enable : 1;           ///< bit 4: 0-Disable, 1-Enable OC2 signal output
        U32 cc2Polarity : 1;         ///< bit 5: 0-Normal polarity, 1-Inverted polarity
        U32 cc2ComOutpuEn : 1;       ///< bit 6: 0-Disable, 1-Enable OC2N signal output
        U32 cc2ComOuputPolarity : 1; ///< bit 7: 0-OC2N active high, 1-OC2N active low
        U32 cc3Enable : 1;           ///< bit 8: 0-Disable, 1-Enable OC3 signal output
        U32 cc3Polarity : 1;         ///< bit 9: 0-Normal polarity, 1-Inverted polarity
        U32 cc3ComOutpuEn : 1;       ///< bit 10: 0-Disable, 1-Enable OC3N signal output
        U32 cc3ComOuputPolarity : 1; ///< bit 11: 0-OC3N active high, 1-OC3N active low
        U32 cc4Enable : 1;           ///< bit 12: 0-Disable, 1-Enable OC4 signal output
        U32 cc4Polarity : 1;         ///< bit 13: 0-Normal polarity, 1-Inverted polarity
        U32 cc4ComOutpuEn : 1;       ///< bit 14: 0-Disable, 1-Enable OC4N signal output
        U32 cc4ComOuputPolarity : 1; ///< bit 15: 0-OC4N active high, 1-OC4N active low
        U32 rsvd2 : 16;              ///< bits 16-31: Reserved bits
    };
    U32 dword;                       ///< Full 32-bit register access
} PwmCapCmpCtrl_s;

/**
 * pwm main count Reg
 */
typedef union PwmMainCount {
    struct __attribute__((aligned(4))) {
        U32 mainCnt : 16; ///< bits 0-15: Main counter value
        U32 rsvd1 : 16;   ///< bits 16-31: Reserved bits
    };
    U32 dword;            ///< Full 32-bit register access
} PwmMainCount_s;

/**
 * PWM prescaler register
 */
typedef union PwmPreDivClk {
    struct __attribute__((aligned(4))) {
        U32 divClk : 16; ///< bits 0-15: Clock division factor
        U32 rsvd1 : 16;  ///< bits 16-31: Reserved bits
    };
    U32 dword;           ///< Full 32-bit register access
} PwmPreDivClk_s;

/**
 * PWM auto-reload register
 */
typedef union PwmPreLoad {
    struct __attribute__((aligned(4))) {
        U32 preCount : 16; ///< bits 0-15: Preload value for PWM counter
        U32 rsvd1 : 16;    ///< bits 16-31: Reserved bits
    };
    U32 dword;             ///< Full 32-bit register access
} PwmPreLoad_s;

/**
 * PWM repeat counter register
 */
typedef union PwmRepeatCnt {
    struct __attribute__((aligned(4))) {
        U32 repCount : 8; ///< bits 0-7: Repeat counter value
        U32 rsvd1 : 24;   ///< bits 8-31: Reserved bits
    };
    U32 dword;            ///< Full 32-bit register access
} PwmRepeatCnt_s;

/**
 * PWM capture compare register
 */
typedef union PwmCaptureData {
    struct __attribute__((aligned(4))) {
        U32 dataCount : 16; ///< bits 0-15: Capture/compare value for channel 1/2/3/4
        U32 rsvd1 : 16;     ///< bits 16-31: Reserved bits
    };
    U32 dword;             ///< Full 32-bit register access
} PwmCaptureData_s;

/**
 * PWM brake control register
 */
typedef union PwmBrakeCtrl {
    struct __attribute__((aligned(4))) {
        U32 deadTime : 8;      ///< bits 0-7: Dead time duration between complementary outputs
        U32 rsvd1 : 4;         ///< bits 8-11: Reserved bits
        U32 brakeEnable : 1;   ///< bit 12: 1-Enable brake function
        U32 brakePolarity : 1; ///< bit 13: Brake polarity signal (0 or 1)
        U32 rsvd2 : 18;        ///< bits 14-31: Reserved bits
    };
    U32 dword;                ///< Full 32-bit register access
} PwmBrakeCtrl_s;

/* 
* PWM registers
*/

typedef struct PwmReg {
    volatile PwmCtrl_s pwmRegCtrl;           ///< 0x00: PWM control register
    volatile U32 reserved0[2];               ///< 0x04-0x08: Reserved space
    volatile PwmIntrCtrl_s pwmIntrCtrl;      ///< 0x0C: PWM interrupt enable register
    volatile PwmIntrStat_s pwmIntrStat;      ///< 0x10: PWM interrupt status register
    volatile PwmInternalFlush_s pwmInterFlush; ///< 0x14: PWM internal flush register
    volatile PwmInOutCtrl_u ch12InOutCtrl;   ///< 0x18: Channel 1&2 input/output control
    volatile PwmInOutCtrl_u ch34InOutCtrl;   ///< 0x1C: Channel 3&4 input/output control
    volatile PwmCapCmpCtrl_s pwmCapCmpCtrl;   ///< 0x20: PWM capture/compare control register
    volatile PwmMainCount_s pwmMainCnt;      ///< 0x24: PWM main count register
    volatile PwmPreDivClk_s pwmPreDivClk;    ///< 0x28: PWM prescaler clock register
    volatile PwmPreLoad_s pwmPreLoad;        ///< 0x2C: PWM preload register
    volatile PwmRepeatCnt_s pwmRepeatCnt;    ///< 0x30: PWM repeat counter register
    volatile PwmCaptureData_s pwmCaptureData[4]; ///< 0x34-0x40: PWM capture/compare data for channels 1-4
    volatile PwmBrakeCtrl_s pwmBrakeCtrl;    ///< 0x44: PWM brake control register
} __attribute__((packed, aligned(4))) PwmReg_s;

typedef struct {
    pwmIrqCallBack cb;    ///< Interrupt callback function pointer
    void *arg;            ///< Argument to pass to the callback function
} PwmIrqCbItem_s;

typedef struct {
    PwmIrqCbItem_s irqCb;  ///< Interrupt callback information
    SbrPwmCfg_s sbrCfg;    ///< SBR configuration data
    U32 preloadRegCnt;     ///< Preload register counter value
} PwmDrvData_s;

#ifdef __cplusplus
}
#endif

#endif
