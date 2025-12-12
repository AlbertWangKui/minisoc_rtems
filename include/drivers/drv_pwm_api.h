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
    PWM_CHAN_0 = 0,      ///< PWM Channel 0
    PWM_CHAN_1 = 1,      ///< PWM Channel 1
    PWM_CHAN_2 = 2,      ///< PWM Channel 2
    PWM_CHAN_3 = 3,      ///< PWM Channel 3
    PWM_ALL_INVALID = 4, ///< Invalid channel value
} PwmChannel_e;

/**
 * @brief PWM polarity settings
 *
 * This enumeration defines the polarity settings for PWM output signals.
 * The polarity determines whether the output is active high or active low.
 */
typedef enum {
    PWM_POLARITY_ACT_HIGH = 0,   ///< Output is active high
    PWM_POLARITY_ACT_LOW = 1,    ///< Output is active low
    PWM_POLARITY_INVALID,        ///< Invalid polarity value
} PwmOutputPolarity_e;

/**
 * @brief PWM output parameters
 *
 * This structure contains the configuration parameters for a PWM channel output.
 * These parameters are used to configure the PWM output signal characteristics.
 */
typedef struct {
    U32 chId;                      ///< Channel ID - Identifies the PWM channel (0-3)
    U32 freq;                      ///< Frequency - Frequency of the PWM signal in Hz
    U32 duty;                      ///< Duty cycle - Percentage of time the signal is active (0-100)
    PwmOutputPolarity_e polarity;  ///< Polarity - Signal polarity (0: normal, 1: reverse)
} PwmOutParameter_s;

/**
 * @brief Initialize PWM device
 *
 * This function initializes the PWM device, including enabling the clock,
 * resetting the peripheral, allocating driver data, and installing the interrupt handler.
 *
 * @param devId Device ID
 * @return EXIT_SUCCESS on success - PWM device initialized successfully
 * @return -EINVAL if dev is NULL - Invalid device pointer
 * @return -ENOMEM if memory allocation fails - Insufficient memory for device structure
 * @return -EBUSY if device lock fails - Device is busy or locked by another operation
 * @return -EIO if hardware initialization fails - Hardware access error
 *
 * @note This function must be called before any other PWM functions.
 * @note The device must be unlocked before calling this function.
 * @note The interrupt handler will be installed if not already installed.
 *
 * @see pwmDeInit
 */
S32 pwmInit(DevList_e devId);

/**
 * @brief Deinitialize PWM device
 *
 * This function deinitializes the PWM device, including disabling interrupts,
 * removing the interrupt handler, resetting the peripheral, disabling the clock,
 * and uninstalling the driver.
 *
 * @param devId Device ID
 * @return EXIT_SUCCESS on success - PWM device deinitialized successfully
 * @return -EINVAL if dev is NULL - Invalid device pointer
 * @return -EBUSY if device lock fails - Device is busy or locked by another operation
 * @return -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @return -EIO if interrupt handler removal, peripheral reset, clock disable, or driver uninstall fails - Hardware access error
 *
 * @note This function should be called when the PWM device is no longer needed.
 * @note All PWM channels will be stopped before the device is deinitialized.
 *
 * @see pwmInit
 */
S32 pwmDeInit(DevList_e devId);

/**
 * @brief Start PWM output on a channel
 *
 * This function starts PWM output on a specific channel. The channel must be
 * configured before starting.
 *
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @retval  EXIT_SUCCESS on success - PWM channel started successfully
 * @retval  -EINVAL if chId is invalid - Invalid channel ID
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data or register access fails - Hardware access error
 *
 * @note The channel must be configured before starting.
 * @note The PWM main counter will be enabled.
 * @note This function will not disable other already enabled channels.
 *
 * @see pwmStop
 * @see pwmCfg
 */
S32 pwmStart(DevList_e devId, PwmChannel_e chId);

/**
 * @brief Stop PWM output on a channel
 *
 * This function stops PWM output on a specific channel. The channel can be
 * restarted later if needed.
 *
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @retval  EXIT_SUCCESS on success - PWM channel stopped successfully
 * @retval  -EINVAL if chId is invalid - Invalid channel ID
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data or register access fails - Hardware access error
 *
 * @note The PWM main counter will be disabled only when all channels are disabled.
 * @note This function will not affect other already enabled channels.
 *
 * @see pwmStart
 * @see pwmCfg
 */
S32 pwmStop(DevList_e devId, PwmChannel_e chId);

/**
 * @brief Register PWM interrupt callback function
 *
 * This function registers a callback function for PWM interrupts.
 * The callback function will be called when any PWM interrupt occurs.
 *
 * @param devId Device ID
 * @param callback Callback function pointer
 * @param arg Argument to pass to the callback function
 * @retval  EXIT_SUCCESS on success - Callback function registered successfully
 * @retval  -EINVAL if callback is NULL - Invalid callback function pointer
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data, register access, or interrupt configuration fails - Hardware access error
 *
 * @note Only one callback can be registered at a time.
 * @note The callback function should be short and efficient to avoid delaying other interrupts.
 * @note Registration process: 1. Disable all interrupts 2. Set callback function and argument 3. Restore interrupt state
 *
 * @see pwmCallbackUnRegister
 * @see pwmIrqCallBack
 */
S32 pwmCallbackRegister(DevList_e devId,
                         pwmIrqCallBack callback,
                         void *arg);

/**
 * @brief Unregister PWM interrupt callback function
 *
 * This function unregisters the PWM interrupt callback function.
 * After calling this function, no callback will be called when PWM interrupts occur.
 *
 * @param devId Device ID
 * @retval  EXIT_SUCCESS on success - Callback function unregistered successfully
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data or interrupt configuration fails - Hardware access error
 *
 * @note This function should be called when the callback is no longer needed.
 * @note This function disables all PWM interrupts and clears the callback function.
 *
 * @see pwmCallbackRegister
 * @see pwmIrqCallBack
 */
S32 pwmCallbackUnRegister(DevList_e devId);

/**
 * @brief Enable PWM interrupt
 *
 * This function enables the specified PWM interrupt type.
 *
 * @param devId Device ID
 * @param intrType Interrupt type (PWM_CHAN_1_INT, PWM_CHAN_2_INT, etc.)
 * @retval  EXIT_SUCCESS on success - Interrupt enabled successfully
 * @retval  -EINVAL if intrType is invalid - Invalid interrupt type specified
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data or interrupt configuration fails - Hardware access error
 *
 * @note This function should be called after registering a callback function.
 * @note Multiple interrupt types can be enabled simultaneously.
 *
 * @see pwmInterruptDisable
 * @see pwmCallbackRegister
 */
S32 pwmInterruptEnable(DevList_e devId, PwmIntrType_e intrType);

/**
 * @brief Disable PWM interrupt
 *
 * This function disables the specified PWM interrupt type.
 *
 * @param devId Device ID
 * @param intrType Interrupt type (PWM_CHAN_1_INT, PWM_CHAN_2_INT, etc.)
 * @retval  EXIT_SUCCESS on success - Interrupt disabled successfully
 * @retval  -EINVAL if intrType is invalid - Invalid interrupt type specified
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data or interrupt configuration fails - Hardware access error
 *
 * @note This function can be called to temporarily disable specific interrupts.
 * @note Disabling PWM_ALL_INT will disable all PWM interrupts.
 *
 * @see pwmInterruptEnable
 * @see pwmCallbackRegister
 */
S32 pwmInterruptDisable(DevList_e devId, PwmIntrType_e intrType);

/**
 * @brief Configure PWM output parameters
 *
 * This function configures the PWM output parameters for a specific channel.
 * This includes the channel ID, frequency, duty cycle, and polarity.
 *
 * @param devId Device ID
 * @param cfg Pointer to PWM output configuration
 * @retval  EXIT_SUCCESS on success - PWM output configured successfully
 * @retval  -EINVAL if cfg is NULL or contains invalid parameters (frequency=0, invalid chId, duty>100, invalid polarity) - Invalid configuration parameters
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data, register access, or frequency conversion fails - Hardware access error
 *
 * @note The configuration structure must be properly initialized before calling this function.
 * @note The channel must be started after configuration to generate output.
 * @note This function temporarily disables the PWM main counter during configuration.
 *
 * @see pwmSetFreq
 * @see pwmSetDuty
 * @see pwmStart
 */
S32 pwmCfg(DevList_e devId, PwmOutParameter_s *cfg);

/**
 * @brief Set PWM frequency
 *
 * This function sets the PWM frequency for the specified device. The frequency
 * determines how fast the PWM signal cycles.
 *
 * @param devId Device ID
 * @param Freq PWM frequency in Hz (must be > 0)
 * @retval  EXIT_SUCCESS on success - PWM frequency set successfully
 * @retval  -EINVAL if Freq is 0, exceeds maximum, or preload count is too low/high - Invalid frequency value
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data, register access, or frequency conversion fails - Hardware access error
 *
 * @note The frequency must be within the supported range of the PWM hardware.
 * @note Changing the frequency affects all PWM channels on the device.
 * @note Frequency calculation: Freq = clk / (prescaler + 1) / preload
 * @note The preload count is stored in driver data for duty cycle calculations.
 *
 * @see pwmSetDuty
 * @see pwmCfg
 */
S32 pwmSetFreq(DevList_e devId, U32 Freq);

/**
 * @brief Set PWM channel duty cycle
 *
 * This function sets the duty cycle for a specific PWM channel. The duty cycle
 * determines the percentage of time the signal is active during each period.
 *
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @param duty Duty cycle percentage (0-100)
 * @retval  EXIT_SUCCESS on success - PWM duty cycle set successfully
 * @retval  -EINVAL if duty is invalid (> 100) or chId is invalid - Invalid duty cycle or channel ID
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data, preload count is 0, or register access fails - Hardware access error
 *
 * @note The duty cycle must be between 0 and 100.
 * @note The channel must be configured before setting the duty cycle.
 * @note Duty cycle calculation: dutyCompareVal = PWM_GET_DUTY_CMPARE_VAL(preloadCnt, duty)
 * @note The preload count must be set by pwmSetFreq before calling this function.
 *
 * @see pwmSetFreq
 * @see pwmStart
 */
S32 pwmSetDuty(DevList_e devId, PwmChannel_e chId, U32 duty);

/**
 * @brief Set PWM channel output polarity
 *
 * This function sets the output polarity for a specific PWM channel. The polarity
 * determines whether the PWM signal is active high or active low.
 *
 * @param devId Device ID
 * @param chId Channel ID (0-3)
 * @param polarity Output polarity setting
 * @retval  EXIT_SUCCESS on success - PWM polarity set successfully
 * @retval  -EINVAL if polarity is invalid or chId is invalid - Invalid polarity or channel ID
 * @retval  -EBUSY if device lock fails - Device is busy or locked by another operation
 * @retval  -ENODEV if driver doesn't match or device is not initialized - Device not found or not initialized
 * @retval  -EIO if getting driver data or hardware configuration fails - Hardware access error
 *
 * @note The channel must be configured before setting the polarity.
 * @note Polarity settings: 0 for active high, 1 for active low.
 * @note This function configures both the output mode and polarity for the specified channel.
 *
 * @see pwmCfg
 * @see pwmSetDuty
 * @see PwmOutputPolarity_e
 */
S32 pwmSetPolarity(DevList_e devId, PwmChannel_e chId, PwmOutputPolarity_e polarity);

#ifdef __cplusplus
}
#endif

#endif
