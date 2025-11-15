/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_interrupt.c
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   封装rtems操作系统中断对外接口
 * @note    NA
 */

#include <rtems/bspIo.h>

#include <osp_interrupt.h>
#include "include/osp_inner_common.h"

#include <bspopts.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>

#include <bsp/arm-gic.h>
#include <bsp/arm-gic-irq.h>

#include <rtems/score/smpimpl.h>
#include <rtems/irq-extension.h>

#if !defined(PS3OS_SMP)

#undef rtems_interrupt_disable
#undef rtems_interrupt_enable
#undef rtems_interrupt_flash

extern rtems_interrupt_level rtems_interrupt_disable(void);
extern void rtems_interrupt_enable(rtems_interrupt_level previous_level);
extern void rtems_interrupt_flash(rtems_interrupt_level previous_level);

#endif

#undef rtems_interrupt_is_in_progress
extern bool rtems_interrupt_is_in_progress(void);

#if !defined(PS3OS_SMP)
/**
 * @brief   使能中断
 * @param   previousLevel [in],
 * 需要传入需要设置的cpsr值，这个值必须是ospInterruptEnables的返回值
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospInterruptEnable(OspInterruptLevel_t previousLevel)
{
    rtems_interrupt_enable(previousLevel);
}

/**
 * @brief   禁止中断
 * @param
 * @return  返回当前cpsr的值 ，
 * 在调用ospInterruptEnables的时候需要把这个参数作为入参传入
 * @warning 无
 * @note    无
 */
OspInterruptLevel_t ospInterruptDisable(void)
{
    return rtems_interrupt_disable( );
}

/**
 * @brief   相当于连续调用 ospInterruptEnable 和 ospInterruptDisable
 * @param   previousLevel [in],
 * 需要传入需要设置的cpsr值，这个值必须是ospInterruptEnables的返回值
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospInterruptFlash(OspInterruptLevel_t previousLevel)
{
    rtems_interrupt_flash(previousLevel);
}


/**
 * @brief   获取中断是否打开
 * @param   无
 * @return  true(disabled) or false(enabled)
 * @warning 无
 * @note    无
 */
bool ospInterruptIsDisabled(void)
{
    return (bool)_ISR_Get_level();
}

#endif

/**
 * @brief   判断当前是否在中断上下文
 * @param   [in]
 * @return  true or false
 * @warning 无
 * @note    无
 */

bool ospInterruptIsInProgress(void)
{
    return rtems_interrupt_is_in_progress( );
}

/**
 * @brief   获取中断等级
 * @param   level , 任务模式, 例如 OSP_DEFAULT_MODES
 * @return  返回被 OSP_INTERRUPT_MASK 过滤的值(中断等级)
 * @warning 无
 * @note    无
 */
OspMode ospInterruptLevelBody(uint32_t level)
{
    return (OspMode)rtems_interrupt_level_body(level);
}

/**
 * @brief   注册中断处理函数
 * @param   vector ,硬件中断号
 * @param   info [in] ,描述中断处理入口的字符串
 * @param   options ,选择中断处理方式，有以下选项：
            OSP_INTERRUPT_UNIQUE  (最常用)当前中断号只对应一个中断处理函数
            OSP_INTERRUPT_SHARED  当前中断号可被多个中断处理函数共用
            OSP_INTERRUPT_REPLACE 当前要注册的@handler会强制替换最初的@handler
 * @param   handler ,待注册的中断处理函数
 * @param   arg [in/out],中断处理函数的参数
 * @return  注册操作的结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptHandlerInstall(OspVector vector, const char *info, OspOption options,
                                           OspInterruptHandler handler, void *arg)
{
    rtems_status_code ret;
    OspStatusCode_e ospResult;

    ret       = rtems_interrupt_handler_install(vector, info, options, handler, arg);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}

/**
 * @brief   删除中断处理函数
 * @param   vector ,硬件中断号
 * @param   info [in] ,描述中断处理入口的字符串
 * @param   handler ,待删除的中断处理函数
 * @param   void *arg [in/out],中断处理函数的参数
 * @return  删除操作的结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptHandlerRemove(OspVector vector, OspInterruptHandler handler, void *arg)
{
    rtems_status_code ret;
    OspStatusCode_e ospResult;

    ret       = rtems_interrupt_handler_remove(vector, handler, arg);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}

/**
 * @brief   设置中断控制器触发方式
 * @param   id ,硬件中断号
 * @param   mode  ,触发方式
 * @return
 * @warning 无
 * @note    无
 */
void ospInterruptIDSetTriggerMode(uint32_t id, OspTriggerMode_e mode)
{
    gic_id_set_trigger_mode(ARM_GIC_DIST, id, (gic_trigger_mode)mode);
}

/**
 * @brief   转换OSP中断优先级为GIC中断优先级
 * @param   Priority , 中断优先级
 * @return  GIC中断优先级
 * @warning 无
 * @note    GIC-400支持32个优先级, 范围是 0 ~ 0xF8 , 步长为 8
 *          实际和左猛/袁清波对齐(2021.01.26), 使用 5 个优先级
 */
static uint8_t ospConvertInterruptPriority(OspInterruptPriority_e Priority)
{
    uint8_t GicPriority = 0;

    switch(Priority) {
    case INTERRUPT_PRIORITY_LOWEST:
        GicPriority = 0xF8;
        break;
    case INTERRUPT_PRIORITY_LOW:
        GicPriority = 0xC0;
        break;
    case INTERRUPT_PRIORITY_NORMAL:
        GicPriority = 0x7F; ///< 和 PRIORITY_DEFAULT 保持一致
        break;
    case INTERRUPT_PRIORITY_HIGH:
        GicPriority = 0x40;
        break;
    case INTERRUPT_PRIORITY_HIGHEST:
        GicPriority = 0x00;
        break;
    }

    return GicPriority;
}

/**
 * @brief   设置中断优先级
 * @param   vector ,   硬件中断号
 * @param   Priority , 中断优先级
 * @return  操作结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptSetPriority(OspVector vector, OspInterruptPriority_e priority)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet = OSP_SUCCESSFUL;

    uint8_t gicPriority = ospConvertInterruptPriority(priority);
    ret                 = arm_gic_irq_set_priority(vector, gicPriority);
    if(ret != RTEMS_SUCCESSFUL) {
        ospRet = ospConvertReturnValue(ret);
    }

    return ospRet;
}

/**
 * @brief   设置中断亲和性
 * @param   vector , 硬中断号
 * @param   affinity_size  ,affinity bit域的大小
 * @param   affinity, bit域 ，每个bit 代表亲和一个core
 * @return
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospInterruptSetAffinity(OspVector vector, size_t affinity_size,const cpu_set_t *affinity)
{
    rtems_status_code ret;
    OspStatusCode_e ospRet = OSP_SUCCESSFUL;


    ret                 = rtems_interrupt_set_affinity(vector, affinity_size ,affinity);
    if(ret != RTEMS_SUCCESSFUL) {
        ospRet = ospConvertReturnValue(ret);
        goto exit;
    }

exit:
    return ospRet;
}

OspStatusCode_e ospInterruptGenSoftIrq(OspVector vector, OspIrqTargetFilter_e filter, uint8_t targets)
{
    OspStatusCode_e ret;

    ret = (OspStatusCode_e)arm_gic_irq_generate_software_irq(
            (rtems_vector_number)vector,
            (arm_gic_irq_software_irq_target_filter)filter,
            targets
            );
    return ret;
}

/**
 * @brief   使能中断向量
 * @param   vector ,   硬件中断号
 * @return  无
 * @warning 无
 * @note    无
 */
void ospInterruptVectorEnable(OspVector vector)
{
    bsp_interrupt_vector_enable(vector);
}

/**
 * @brief   去使能中断向量
 * @param   vector ,   硬件中断号
 * @return  无
 * @warning 无
 * @note    无
 */
void ospInterruptVectorDisable(OspVector vector)
{
    bsp_interrupt_vector_disable(vector);
}

/**
 * @brief   清理特定vector中断对应的中断控制器信息
 * @param   vector ,   硬件中断号
 * @return  无
 * @warning 无
 * @note    无
 */
void ospInterruptVectorUninit(OspVector vector)
{
#ifdef PS3OS_NIC_100G
    return bsp_interrupt_vector_uninit(vector);
#else
    (void)vector;
    return;
#endif
}

void ospIPISendBroadcast( uint32_t ipi )
{
#if defined(PS3OS_SMP)
    _SMP_IPI_Send_broadcast( ipi );
#else
    (void)ipi;
#endif
}


