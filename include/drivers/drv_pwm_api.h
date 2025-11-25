/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_pwm_api.h
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @version 1.0
 * @brief  PWM driver API header file for Minisoc platform
 * 
 * This file contains the API definitions for the PWM (Pulse Width Modulation) 
 * driver for the Minisoc platform. It includes enumerations for PWM
 * output compare modes, interrupt types, and polarity settings, as well as
 * structure definitions for PWM output parameters and function declarations
 * for PWM control operations.
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/10/22   yangkl         the first version
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
 * @brief PWM interrupt callback function type
 * 
 * This function type defines the callback function that will be called when a PWM
 * interrupt occurs. The callback function receives the user-provided argument and
 * the interrupt status.
 * 
 * @param arg User-provided argument passed to the callback function
 * @param intrStat Interrupt status bitmask indicating which interrupts occurred
 */
typedef void (*pwmIrqCallBack)(void *arg, U32 intrStat);

/**
 * @brief PWM output compare modes
 * 
 * This enumeration defines the different output compare modes supported by the PWM.
 * These modes determine the behavior of the PWM output signal.
 */
typedef enum {
    PWM_OC_DIS = 0,  ///< oc disable
    PWM_OC_HIGH,     ///< force high
    PWM_OC_LOW,      ///< force low
    PWM_OC_RSVD,     ///< reserved
    PWM_OC_INV_LOW,  ///< low
    PWM_OC_INV_HIGH, ///< high
    PWM_OC_MODE1,    ///< pwm 向上计数，一旦数值小于比较寄存器值时通道1为有效电平，向下计数时，同条件1为无效电平，0为有效
    PWM_OC_MODE2,    ///< pwm 与上相反
} PwmOcMode_e;

/**
 * @brief PWM interrupt types
 * 
 * This enumeration defines the different types of PWM interrupts that can be enabled
 * and handled by the driver.
 */
typedef enum PwmIntrType{
    PWM_CHAN_1_INT = 1,  ///< Channel 1 interrupt - Triggered when channel 1 counter matches compare value
    PWM_CHAN_2_INT = 2,  ///< Channel 2 interrupt - Triggered when channel 2 counter matches compare value
    PWM_CHAN_3_INT = 3,  ///< Channel 3 interrupt - Triggered when channel 3 counter matches compare value
    PWM_CHAN_4_INT = 4,  ///< Channel 4 interrupt - Triggered when channel 4 counter matches compare value
    PWM_BRAKE_INT  = 7,  ///< Brake interrupt - Triggered when PWM brake is activated
    PWM_ALL_INT = 8,     ///< All interrupts - Enables all PWM interrupt types
} PwmIntrType_e;

/**
 * @brief PWM channels
 * 
 * This enumeration defines the available PWM channels.
 */
typedef enum PwmChannel{
    PWM_CHAN_0 = 0,      /**< PWM Channel 0 */
    PWM_CHAN_1 = 1,      /**< PWM Channel 1 */
    PWM_CHAN_2 = 2,      /**< PWM Channel 2 */
    PWM_CHAN_3 = 3,      /**< PWM Channel 3 */
    PWM_ALL_INVALID = 4, /**< Invalid channel value */
} PwmChannel_e;

/**
 * @brief PWM polarity settings
 * 
 * This enumeration defines the polarity settings for PWM output signals.
 * The polarity determines whether the output is active high or active low.
 */
typedef enum {
    PWM_POLARITY_NORMAL = 0,   ///< Normal polarity - Output is active high
    PWM_POLARITY_REVERSE,      ///< Reverse polarity - Output is active low
    PWM_POLARITY_INVALID,      ///< Invalid polarity value
} PwmOutputPolarity_e;

/**
 * @brief PWM output parameters
 * 
 * This structure contains the configuration parameters for a PWM channel output.
 * These parameters are used to configure the PWM output signal characteristics.
 */
typedef struct {
    U32 chId;               ///< Channel ID - Identifies the PWM channel (0-3)
    U32 duty;               ///< Duty cycle - Percentage of time the signal is active (0-100)
    PwmOutputPolarity_e polarity;  ///< Polarity - Signal polarity (0: normal, 1: reverse)
}PwmOutParameter_s;

/**
 * @brief Initialize PWM device
 * 
 * This function initializes the PWM device, including enabling the clock,
 * resetting the peripheral, allocating driver data, and installing the interrupt handler.
 * 
 * @param devId Device ID
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device lock fails or driver is already initialized
 * @return -ENODEV if driver doesn't match
 * @return -EIO if clock enable, reset, or interrupt handler installation fails
 * @return -ENOMEM if memory allocation fails
 * 
 * @note This function must be called before any other PWM functions.
 * @note The device must be unlocked before calling this function.
 * 
 * @see pwmDeInit
 */
S32 pwmInit(DevList_e devId);

/**
 * @brief Deinitialize PWM device
 * 
 * This function deinitializes the PWM device, including disabling interrupts,
 * removing the interrupt handler, resetting the peripheral, and uninstalling the driver.
 * 
 * @param devId Device ID
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if interrupt handler removal, peripheral reset, or driver uninstall fails
 * 
 * @note This function should be called when the PWM device is no longer needed.
 * @note All PWM channels will be stopped before the device is deinitialized.
 * 
 * @see pwmInit
 */
S32 pwmDeInit(DevList_e devId);

/**
 * @brief Configure PWM output parameters
 * 
 * This function configures the PWM output parameters for a specific channel.
 * This includes the channel ID, duty cycle, and polarity.
 * 
 * @param devId Device ID
 * @param cfg Pointer to PWM output configuration
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if cfg is NULL or contains invalid parameters
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if getting driver data or register access fails
 * 
 * @note The configuration structure must be properly initialized before calling this function.
 * @note The channel must be started after configuration to generate output.
 * 
 * @see pwmSetFreq
 * @see pwmSetDuty
 * @see pwmStart
 */
S32 pwmSetCfg(DevList_e devId, PwmOutParameter_s *cfg);

/**
 * @brief Set PWM frequency
 * 
 * This function sets the PWM frequency for the specified device. The frequency
 * determines how fast the PWM signal cycles.
 * 
 * @param devId Device ID
 * @param Freq PWM frequency in Hz (must be > 0)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if Freq is 0
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if getting driver data, register access, or frequency conversion fails
 * 
 * @note The frequency must be within the supported range of the PWM hardware.
 * @note Changing the frequency affects all PWM channels on the device.
 * @note Frequency calculation: Freq = clk / (prescaler + 1) / preload
 * 
 * @see pwmSetDuty
 * @see pwmSetCfg
 */
S32 pwmSetFreq(DevList_e devId, U32 Freq);

/**
 * @brief Start PWM output on a channel
 * 
 * This function starts PWM output on a specific channel. The channel must be
 * configured before starting.
 * 
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if chId is invalid
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if getting driver data, register access, or channel enable fails
 * 
 * @note The channel must be configured before starting.
 * @note The PWM main counter will be enabled.
 * @note This function will not disable other already enabled channels.
 * 
 * @see pwmStop
 * @see pwmSetCfg
 */
S32 pwmStart(DevList_e devId, U32 chId);

/**
 * @brief Stop PWM output on a channel
 * 
 * This function stops PWM output on a specific channel. The channel can be
 * restarted later if needed.
 * 
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if chId is invalid
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if getting driver data, register access, or channel disable fails
 * 
 * @note The PWM main counter will be disabled.
 * @note This function will not affect other already enabled channels.
 * 
 * @see pwmStart
 * @see pwmSetCfg
 */
S32 pwmStop(DevList_e devId, U32 chId);

/**
 * @brief Register PWM interrupt callback function
 * 
 * This function registers a callback function for a specific PWM interrupt type.
 * The callback function will be called when the specified interrupt occurs.
 * 
 * @param devId Device ID
 * @param intrType Interrupt type (PWM_CHAN_1_INT, PWM_CHAN_2_INT, etc.)
 * @param callback Callback function pointer
 * @param arg Argument to pass to the callback function
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if intrType is invalid or callback is NULL
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if getting driver data or interrupt enable/disable fails
 * 
 * @note Only one callback can be registered per interrupt type.
 * @note The callback function should be short and efficient to avoid delaying other interrupts.
 * @note Registration process: 1. Disable specified interrupt 2. Set callback function and argument 3. Re-enable interrupt
 * 
 * @see pwmCallbackUnRegister
 * @see pwmIrqCallBack
 */
S32 pwmCallbackRegister(DevList_e devId,
                         PwmIntrType_e intrType,
                         pwmIrqCallBack callback,
                         void *arg);

/**
 * @brief Unregister PWM interrupt callback function
 * 
 * This function unregisters the callback function for a specific PWM interrupt type.
 * After calling this function, no callback will be called when the specified interrupt occurs.
 * 
 * @param devId Device ID
 * @param intrType Interrupt type (PWM_CHAN_1_INT, PWM_CHAN_2_INT, etc.)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if intrType is invalid
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if interrupt disable fails
 * 
 * @note This function should be called when the callback is no longer needed.
 * @note This function disables the specified interrupt type and clears the callback function.
 * 
 * @see pwmCallbackRegister
 * @see pwmIrqCallBack
 */
S32 pwmCallbackUnRegister(DevList_e devId, PwmIntrType_e intrType);

/**
 * @brief Set PWM channel duty cycle
 * 
 * This function sets the duty cycle for a specific PWM channel. The duty cycle
 * determines the percentage of time the signal is active during each period.
 * 
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @param duty Duty cycle percentage (0-100)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if duty is invalid (> PWM_DUTY_MAX) or chId is invalid
 * @return -EBUSY if device lock fails
 * @return -ENODEV if driver doesn't match or device is not initialized
 * @return -EIO if getting driver data, register access, or duty cycle calculation fails
 * @return -EINVAL if duty compare value is invalid
 * 
 * @note The duty cycle must be between 0 and PWM_DUTY_MAX.
 * @note The channel must be configured before setting the duty cycle.
 * @note Duty cycle calculation: dutyCompareVal = PWM_GET_DUTY_CMPARE_VAL(preloadCnt, duty)
 * @note The preload count must be set by pwmSetFreq before calling this function.
 * 
 * @see pwmSetFreq
 * @see pwmStart
 */
S32 pwmSetDuty(DevList_e devId, U32 chId, U32 duty);

#ifdef __cplusplus
}
#endif

#endif
