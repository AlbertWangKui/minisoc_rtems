/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_v1.0.c
 * @author liuzhsh1 (liuzhsh1@starsmicrosystem.com)
 * @date 2025/09/22
 * @brief  SGPIO (Serial GPIO) 驱动实现文件，适用于 v1.0 硬件
 *        实现 SGPIO 控制器的初始化、配置和控制功能
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/09/22   liuzhsh1        the first version
 *
 */

#include "common_defines.h"
#include "log_msg.h"
#include "udelay.h"
#include "drv_sgpio_v1.0.h"
#include "drv_sgpio_api.h"
#include "stringto_wrapper.h"
#include "bsp_config.h"
#include "bsp_api.h"
#include "osp_interrupt.h"

typedef volatile struct {
    U32 id;
    void *sgpioReg;
} SgpioIsrCtx_s;

static SgpioIsrCtx_s sgpioIsrCtx[SGPIO_MAX_NUM] = {
    {0, NULL}, {0, NULL}, {0, NULL}, {0, NULL}
};

/**
 * @brief SGPIO ISR Handler function pointer type
 */
typedef void (*SgpioIsrHandler)(void *arg);

/**
 * @brief LED状态编码映射表
 */
static const U8 sgpioLedPatternMap[][8] = {
    ///< Error LED (3 bits)
    [SGPIO_LED_ERROR] = {
        [SGPIO_LED_ON] = 0,
        [SGPIO_LED_OFF] = 1,
        [SGPIO_LED_BLINK_A_HIGH_LOW] = 2,
        [SGPIO_LED_BLINK_A_LOW_HIGH] = 3,
        [SGPIO_LED_BLINK_B_HIGH_LOW] = 6,
        [SGPIO_LED_BLINK_B_LOW_HIGH] = 7
    },
    ///< Locate LED (2 bits)
    [SGPIO_LED_LOCATE] = {
        [SGPIO_LED_ON] = 0,
        [SGPIO_LED_OFF] = 1,
        [SGPIO_LED_BLINK_A_HIGH_LOW] = 2,
        [SGPIO_LED_BLINK_A_LOW_HIGH] = 3,
    },
    ///< Activity LED (3 bits)
    [SGPIO_LED_ACTIVITY] = {
        [SGPIO_LED_ON] = 0,
        [SGPIO_LED_OFF] = 1,
        [SGPIO_LED_BLINK_A_HIGH_LOW] = 2,
        [SGPIO_LED_BLINK_A_LOW_HIGH] = 3,
        [SGPIO_LED_EVENT_RISING] = 4,
        [SGPIO_LED_EVENT_FALLING] = 5,
        [SGPIO_LED_BLINK_B_HIGH_LOW] = 6,
        [SGPIO_LED_BLINK_B_LOW_HIGH] = 7
    }
};

static S32 sgpioGetRegsChecked(DevList_e devId, SgpioDrvData_s **drv, SgpioReg_s **regs)
{
    if (devId < DEVICE_SGPIO0 || devId >= DEVICE_SGPIO0 + SGPIO_MAX_NUM)
        return -EINVAL;

    if (drv == NULL || regs == NULL)
        return -EINVAL;

    if (getDevDriver(devId, (void **)drv) != EXIT_SUCCESS)
        return -EIO;

    if ((*drv) == NULL || (*drv)->sbrCfg.regAddr == NULL)
        return -EIO;

    *regs = (SgpioReg_s *)(*drv)->sbrCfg.regAddr;
    return EXIT_SUCCESS;
}

///< TODO： need replace when soc top pinmux api ready
void sgpioConfigurePinmux()
{
    U32 pinmuxAddr = SGPIO_PINMUX_BASE;
    U32 addr = 0;
    U32 pinmuxVal = 0;
    U32 offsets[] = { 0x208, 0x210, 0x218, 0x220, 0x228, 0x230, 0x238, 0x240 };
    int i;

    for (i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        addr = pinmuxAddr + offsets[i];
        pinmuxVal = reg32Read(addr);
        pinmuxVal &= ~(0xf);
        pinmuxVal |= 5;
        reg32Write(addr, pinmuxVal);
    }

}

static S32 sgpioGetDevCfg(DevList_e devId, SbrSgpioCfg_s *sbrCfg)
{
#if 0
    /* 从SBR读取SGPIO配置 */
    if (devSbrRead(devId, sbrCfg, 0, sizeof(SbrSgpioCfg_s)) != sizeof(SbrSgpioCfg_s)) {
        LOGE("%s: failed to read SBR config for device %d\r\n",
             __func__, devId);
        return -EIO;
    }
#else ///< TODO: 临时配置，后续删除
    sbrCfg->regAddr = (void *)(SGPIO_BASE_ADDR + (devId - DEVICE_SGPIO0) * SGPIO_DEVICE_OFFSET +
        SGPIO_8485_REG_OFFSET);
    sbrCfg->driveNum = 64;
    sbrCfg->cfgClk = 100000; ///< 100KHz
    sbrCfg->ActivityMaskhigh = 0xffffffff;
    sbrCfg->ActivityMasklow = 0xffffffff;
    sbrCfg->blinkEn = 0; ///< 0: enable, 1: disable
    sbrCfg->blinkMode = 1; ///< 0: from external input trig, 1: sio self trigger
    sbrCfg->blinkFreq = 4; ///< blink频率, 单位Hz
    sbrCfg->blinkOutSel = 0; ///< 0: sgpio0, 1: sgpio1, 2: sgpio2, 3: sgpio3
    sbrCfg->cfg1Timing = 0x00427300;
    sbrCfg->mode = 0; ///< 0: normal, 1: GP, 2: 1005
    sbrCfg->irqEn = 1; ///< disable irq
    sbrCfg->irqNum = SGPIO_IRQ_NUM + (devId - DEVICE_SGPIO0); ///< irq number
    sbrCfg->irqPrio = SGPIO_IRQ_PRI; ///< irq priority
#endif
    /* 验证配置数据的有效性 */
    if (sbrCfg->regAddr == NULL) {
        LOGE("%s: invalid register address in SBR config\r\n", __func__);
        return -EINVAL;
    }

#ifdef CONFIG_DUMP_SBR
    LOGD("SGPIO Debug Config:\n"
         "  regAddr: %p\n"
         "  driveNum: %u\n"
         "  cfgClk: %u Hz\n"
         "  ActivityMaskhigh: 0x%08x\n"
         "  ActivityMasklow: 0x%08x\n"
         "  blinkEn: %u\n"
         "  blinkMode: %u\n"
         "  blinkFreq: %u Hz\n"
         "  cfg1Timing: 0x%08x\n"
         "  blinkOutSel: %u\n"
         "  mode: %u\n"
         "  irqEn: %u\n"
         "  irqNum: %u\n"
         "  irqPrio: %u\n",
        sbrCfg->regAddr, sbrCfg->driveNum, sbrCfg->cfgClk, sbrCfg->ActivityMaskhigh,
        sbrCfg->ActivityMasklow, sbrCfg->blinkEn, sbrCfg->blinkMode, sbrCfg->blinkFreq,
        sbrCfg->cfg1Timing, sbrCfg->blinkOutSel, sbrCfg->mode, sbrCfg->irqEn, sbrCfg->irqNum,
        sbrCfg->irqPrio);
#endif

    if (sbrCfg->driveNum < SGPIO_DRIVER_COUNT_MIN || sbrCfg->driveNum > SGPIO_DRIVER_COUNT_MAX) {
        LOGE("%s: invalid drive number %d in SBR config\r\n", __func__, sbrCfg->driveNum);
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief SGPIO中断服务程序
 *
 * 处理SGPIO控制器的中断事件，包括启动中断、停止中断和驱动器错误中断。
 * 读取中断状态寄存器，识别中断类型，记录日志，并清除相应的中断标志位。
 *
 * @param arg 中断上下文参数，指向SgpioIsrCtx_s结构体，包含SGPIO控制器ID和寄存器基址
 *
 * @note 该函数在中断上下文中运行，应保持简洁高效
 * @note 如果参数或寄存器为空，函数会直接返回以确保安全
 */
static void sgpioIsr(void *arg)
{
    if (arg == NULL)
        return;

    SgpioIsrCtx_s *ctx = (SgpioIsrCtx_s *)arg;
    if (ctx->sgpioReg == NULL)
        return; ///< avoid null deref in ISR

    SgpioReg_s *sgpioRegs = (SgpioReg_s *)ctx->sgpioReg;
    U32 intClr = 0;
    U32 sgpioId = ctx->id;

    LOGD("sgpio[%d] int status: 0x%08x\r\n", sgpioId, sgpioRegs->sioIntStatus);
    if (sgpioRegs->sioIntStatus & SGPIO_START_INT) {
        LOGD("sgpio[%d] sio start int\r\n", sgpioId);
        SET_BIT(intClr, SGPIO_START_INT_CLR);
    }
    if (sgpioRegs->sioIntStatus & SGPIO_STOP_INT) {
        LOGD("sgpio[%d] sio stop int\r\n",sgpioId);
        SET_BIT(intClr, SGPIO_STOP_INT_CLR);
    }
    if (sgpioRegs->sioIntStatus & SGPIO_DRV_ERR_INT) {
        LOGE("sgpio[%d] driver out of range int\r\n", sgpioId);
        SET_BIT(intClr, SGPIO_DRV_ERR_INT_CLR);
    }
    /* clear interrupt */
    if (intClr != 0) {
        LOGD("sgpio[%d] clearing interrupts: 0x%08x\r\n", sgpioId, intClr);
        sgpioRegs->sioIntCfg |= intClr;
    }
}

/**
 * @brief 注册SGPIO中断服务程序
 *
 * 该函数用于为指定的SGPIO设备注册中断服务程序(ISR)，配置中断处理函数和中断优先级。
 * 函数会初始化SGPIO中断配置寄存器，设置中断上下文，并向操作系统注册中断处理程序。
 *
 * @param devId    SGPIO设备ID，用于指定目标设备
 * @param handler  中断处理函数指针，不能为NULL
 * @param irqNum   中断号，用于标识具体的中断源
 * @param irqPrio  中断优先级，数值越小优先级越高
 *
 * @return 执行状态码：
 *         - EXIT_SUCCESS: 中断注册成功
 *         - -EINVAL: 参数无效（如handler为NULL）
 *         - -EIO: 获取设备驱动失败
 *         - 其他错误码来自sgpioGetRegsChecked函数的返回值
 *
 * @note 函数会清空SGPIO中断配置寄存器(sioIntCfg)，初始化中断上下文结构体，
 *       并使用ospInterruptHandlerInstall和ospInterruptSetPriority注册中断
 */
static S32 sgpioRegisterIsr(DevList_e devId, SgpioIsrHandler handler, U16 irqNum, U16 irqPrio)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (handler == NULL) {
        return -EINVAL;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    sgpioRegs->sioIntCfg = 0;
    sgpioIsrCtx[devId - DEVICE_SGPIO0].id = (devId - DEVICE_SGPIO0);
    sgpioIsrCtx[devId - DEVICE_SGPIO0].sgpioReg = (void *)sgpioRegs;

    ospInterruptHandlerInstall(irqNum, "sgpio 1.0", OSP_INTERRUPT_UNIQUE,
        (void (*)(void *))handler, (void *)&sgpioIsrCtx[devId - DEVICE_SGPIO0]);
    ospInterruptSetPriority(irqNum, irqPrio);

    return EXIT_SUCCESS;
}

/**
 * 取消注册SGPIO设备的中断服务例程
 *
 * @param devId    SGPIO设备ID
 * @param handler  要取消注册的中断处理函数
 *
 * @return 执行结果：
 *         - EXIT_SUCCESS: 操作成功
 *         - 其他错误码: 操作失败
 */
static S32 sgpioUnRegisterIsr(DevList_e devId, SgpioIsrHandler handler)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ospInterruptVectorDisable((sgpioDrvData->sbrCfg.irqNum));
    ret = ospInterruptHandlerRemove(sgpioDrvData->sbrCfg.irqNum, (OspInterruptHandler)handler,
        (void *)&sgpioIsrCtx[devId - DEVICE_SGPIO0]);
    if (ret != OSP_SUCCESSFUL) {
        LOGE("%s: irq handler remove failed: %d\r\n", __func__, ret);
        return ret;
    }

    return ret;
}

/**
* @brief 使能指定的SGPIO设备
*
* 该函数用于使能指定的SGPIO设备。
*
* @param devId 设备标识符，用于指定需要使能的SGPIO设备
* @return 如果函数执行成功，返回EXIT_SUCCESS；如果获取设备驱动失败，返回-EIO
*/
static S32 sgpioEnable(DevList_e devId)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }
    
    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    sgpioRegs->sgpioCfg0.fields.sgpioEn = 1;

    return ret;
}

/**
* @brief 禁用指定的SGPIO设备
*
* 该函数用于禁用指定的SGPIO设备。
*
* @param devId 设备ID，类型为DevList_e
*
* @return 返回值类型为S32，表示操作结果。
*         - EXIT_SUCCESS（0）：操作成功
*         - -EIO：输入/输出错误
*/
static S32 sgpioDisable(DevList_e devId)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    sgpioRegs->sgpioCfg0.fields.sgpioEn = 0;

    return ret;
}

/**
* @brief 使能SGPIO设备Sload功能,级联场景，只有first sio device的initiator配置为1，级联 device的initiator配置为0。
* @param devId 设备ID
* @return 如果函数调用成功，返回EXIT_SUCCESS；如果失败，返回-EIO
*/
static S32 sgpioSloadEnable(DevList_e devId)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    sgpioRegs->vendorSpec0.fields.firstSioDevice = 1;
    return ret;
}

/**
* @brief 设置SGPIO设备的SLOAD值
*
* 设置指定SGPIO设备的SLOAD值。GP模式下的SLOAD {L3, L2, L1, L0}, NORMAL模式下的SLOAD必须为0。
*
* @param devId 设备ID，用于指定要设置的SGPIO设备
* @param sloadValue 要设置的SLOAD值
*
* @return 成功时返回EXIT_SUCCESS，失败时返回-EIO
*/
static S32 sgpioSetGpTxSload(DevList_e devId, U32 sloadValue)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< 验证 sloadValue 范围
    if (sloadValue > 0xF) {
        LOGE("%s: invalid sloadValue %u (max 0xF)\r\n", __func__, sloadValue);
        return -EINVAL;
    }

    ///< 检查模式一致性: NORMAL 模式 (mode == 0) 要求 sloadValue == 0
    if (sgpioDrvData->sbrCfg.mode == 0 && sloadValue != 0) {
        LOGE("%s: NORMAL mode requires sloadValue=0, got %u\r\n", __func__, sloadValue);
        return -EINVAL;
    }

    sgpioRegs->vendorSpec1.fields.gpModeCtrlSel = SGPIO_MODE_GP_WHEN_TX_SLOAD_IS_ODD;
    sgpioRegs->txgpCfg.fields.gpTxSload = sloadValue;
    return ret;
}

/**
* @brief 设置SGPIO的活动定时参数
*
* 设置SGPIO的活动定时参数，包括拉伸活动关闭时间、拉伸活动开启时间、强制活动关闭时间和最大活动开启时间。
*
* @param sgpioRegs 指向SGPIO寄存器的指针
* @param stretchActOff 拉伸活动关闭时间（单位：1/64s），范围为0-15
* @param stretchActOn 拉伸活动开启时间（单位：1/64s），范围为0-15
* @param forceActOff 强制活动关闭时间（单位：1/8s），范围为0-15
* @param maxActOn 最大活动开启时间（单位： 1/4s），范围为0-15
*
* @return 返回操作结果，成功返回EXIT_SUCCESS，失败返回-EINVAL
*/
static S32 sgpioSetActivityTiming(
    SgpioReg_s *sgpioRegs, U8 stretchActOff, U8 stretchActOn, U8 forceActOff, U8 maxActOn)
{
    U32 currentValue = 0;
    U32 newValue = 0;
    const U32 MASK = 0xFFFF0000; // 掩码，覆盖 [31:16] 位

    if (sgpioRegs == NULL) {
        return -EINVAL;
    }

    if (stretchActOff > SGPIO_CFG1_STRETCH_VALUE_MAX || stretchActOn > SGPIO_CFG1_STRETCH_VALUE_MAX || 
        forceActOff > SGPIO_CFG1_STRETCH_VALUE_MAX || maxActOn > SGPIO_CFG1_STRETCH_VALUE_MAX) {
        LOGE("invalid timing param %d, %d, %d, %d\r\n", stretchActOff, stretchActOn, forceActOff,
            maxActOn);
        return -EINVAL;
    }

    ///< 准备要写入的新值
    newValue =
        (stretchActOff << SGPIO_CFG1_STRETCH_ACTIVITY_OFF_SHIFT) | (stretchActOn << SGPIO_CFG1_STRETCH_ACTIVITY_ON_SHIFT) \
        | (forceActOff << SGPIO_CFG1_FORCE_ACTIVITY_OFF_SHIFT) | (maxActOn << SGPIO_CFG1_FORCE_ACTIVITY_ON_SHIFT);

    ///< 读-改-写
    currentValue = sgpioRegs->sgpioCfg1.dword;
    currentValue &= ~MASK;
    currentValue |= newValue;
    sgpioRegs->sgpioCfg1.dword = currentValue;
    return EXIT_SUCCESS;
}

/**
* @brief 设置SGPIO的闪烁速率
*
* 设置SGPIO寄存器的闪烁速率，用于控制LED等设备的闪烁频率。
*
* @param sgpioRegs 指向SgpioReg_s结构体的指针，包含SGPIO的配置寄存器
* @param patternARate 模式A的闪烁速率，取值范围为0到15, 单位 1/8s
* @param patternBrate 模式B的闪烁速率，取值范围为0到15，单位 1/8s
*
* @return 如果成功，返回EXIT_SUCCESS；如果参数无效，返回-EINVAL
*/
static S32 sgpioSetBlinkRates(SgpioReg_s *sgpioRegs, U8 patternARate, U8 patternBrate)
{
    U32 currentValue = 0;
    U32 newValue = 0;
    const U32 MASK = 0x0000FF00; ///< 掩码，覆盖 [15:8] 位

    if (sgpioRegs == NULL) {
        return -EINVAL;
    }

    if (patternARate > SGPIO_CFG1_PATTERN_VALUE_MAX ||
        patternBrate > SGPIO_CFG1_PATTERN_VALUE_MAX) {
        LOGE("invalid blink rate param %d, %d\r\n", patternARate, patternBrate);
        return -EINVAL;
    }

    ///< 准备要写入的新值
    newValue = (patternARate << SGPIO_CFG1_PATTERN_A_BLINK_FREQ_SHIFT) |
        (patternBrate << SGPIO_CFG1_PATTERN_B_BLINK_FREQ_SHIFT);

    currentValue = sgpioRegs->sgpioCfg1.dword;
    currentValue &= ~MASK;
    currentValue |= newValue;
    sgpioRegs->sgpioCfg1.dword = currentValue;

    return EXIT_SUCCESS;
}

/**
 * @brief 配置SGPIO设备的CFG1寄存器
 *
 * 该函数用于设置SGPIO设备的CFG1寄存器配置，包括活动定时参数和闪烁速率参数。
 * 函数会从输入的配置值中提取各个位域参数，并分别设置到对应的硬件寄存器中。
 *
 * @param devId 设备ID，用于标识具体的SGPIO设备
 * @param cfg 配置值，包含多个位域参数：
 *            - 活动关闭拉伸时间
 *            - 活动开启拉伸时间  
 *            - 强制活动关闭
 *            - 最大活动开启时间
 *            - 模式A闪烁频率
 *            - 模式B闪烁频率
 *
 * @return 执行结果：
 *         - EXIT_SUCCESS: 配置成功
 *         - -EIO: 设备访问失败或配置失败
 */
static S32 sgpioSetCfg1(DevList_e devId, U32 cfg)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;
    SgpioCfg1_u sgpioCfg1 = { 0 };
    U8 stretchActOff = 0;
    U8 stretchActOn = 0;
    U8 forceActOff = 0;
    U8 maxActOn = 0;
    U8 patternARate = 0;
    U8 patternBrate = 0;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    sgpioCfg1.dword = cfg;

    ///< 提取位域值
    stretchActOff = (U8)((sgpioCfg1.dword & SGPIO_CFG1_STRETCH_ACTIVITY_OFF_MASK) >> SGPIO_CFG1_STRETCH_ACTIVITY_OFF_SHIFT);
    stretchActOn = (U8)((sgpioCfg1.dword & SGPIO_CFG1_STRETCH_ACTIVITY_ON_MASK) >> SGPIO_CFG1_STRETCH_ACTIVITY_ON_SHIFT);
    forceActOff = (U8)((sgpioCfg1.dword & SGPIO_CFG1_FORCE_ACTIVITY_OFF_MASK) >> SGPIO_CFG1_FORCE_ACTIVITY_OFF_SHIFT);
    maxActOn = (U8)((sgpioCfg1.dword & SGPIO_CFG1_FORCE_ACTIVITY_ON_MASK) >> SGPIO_CFG1_FORCE_ACTIVITY_ON_SHIFT);
    patternARate = (U8)((sgpioCfg1.dword & SGPIO_CFG1_PATTERN_A_BLINK_FREQ_MASK) >> SGPIO_CFG1_PATTERN_A_BLINK_FREQ_SHIFT);
    patternBrate = (U8)((sgpioCfg1.dword & SGPIO_CFG1_PATTERN_B_BLINK_FREQ_MASK) >> SGPIO_CFG1_PATTERN_B_BLINK_FREQ_SHIFT);

    ///< 设置活动定时参数
    if (sgpioSetActivityTiming(sgpioRegs, stretchActOff, stretchActOn, forceActOff, maxActOn) != EXIT_SUCCESS) {
        LOGE("sgpio set activity timing failed\r\n");
        return -EIO;
    }

    ///< 设置闪烁速率参数
    if (sgpioSetBlinkRates(sgpioRegs, patternARate, patternBrate) != EXIT_SUCCESS) {
        LOGE("sgpio set blink rates failed\r\n");
        return -EIO;
    }

    return EXIT_SUCCESS;
}

/**
* @brief 驱动SGPIO引脚活动掩码
*
* 驱动SGPIO引脚活动掩码，用于设置SGPIO驱动器的输出引脚的活动掩码。
*
* @param devId 设备ID，用于指定操作的设备
* @param driverHigh32Mask 高32位驱动掩码，用于设置高32位驱动器的活动掩码(0 表示不驱动, 1 表示驱动)
* @param driverLow32Mask 低32位驱动掩码，用于设置低32位驱动器的活动掩码(0 表示不驱动, 1 表示驱动)
*
* @return 成功返回EXIT_SUCCESS，失败返回-EIO
*/
static S32 sgpioDriveActivityMask(DevList_e devId, U32 driverHigh32Mask, U32 driverLow32Mask)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    sgpioRegs->driverStrobe0 = driverLow32Mask;
    sgpioRegs->driverStrobe1 = driverHigh32Mask;
    return EXIT_SUCCESS;
}

/**
 * @brief 设置SGPIO模块的串行时钟频率
 *
 * 该函数用于配置指定SGPIO设备的串行时钟频率，包括分频系数、时钟周期和100us计数器值。
 * 函数会根据输入的频率值计算相应的寄存器配置，并设置到硬件寄存器中。
 *
 * @param devId SGPIO设备ID，用于标识要配置的设备
 * @param hz 要设置的时钟频率，单位Hz，范围必须在SGPIO_MIN_CLK到SGPIO_MAX_CLK之间
 *
 * @return 执行结果
 *        - EXIT_SUCCESS 设置成功
 *        - -EIO 设备驱动获取失败
 *        - -EINVAL 输入频率超出有效范围或时钟周期计算溢出
 */
static S32 sgpioSetSClock(DevList_e devId, U32 hz)
{
    U32 clkDiv = 0;
    U32 clkPeriod = 0;
    U32 us100Cnt = 0;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    if (hz > SGPIO_MAX_CLK || hz < SGPIO_MIN_CLK) {
        ret = -EINVAL;
    } else {
        /**
         * 分频计算公式hz = source clk/(((clkdivide * 8) + 1) *2)
         */
        clkDiv = ((SGPIO_SOURCE_CLK / (2 * hz)) - 1) / 8;
        sgpioRegs->vendorSpec0.fields.clkDivide = clkDiv;

        /**
         * 10e9 / hz, clock blink period is base on clock output
         */
        if (1000000000UL / hz > UINT32_MAX) {
            LOGE("%s error: potential overflow in clkPeriod calculation\n", __func__);
            return -EINVAL;
        }
        clkPeriod = 1000000000UL / hz; /* clock period unit: ns */
        sgpioRegs->sioClkPeriod = clkPeriod;

        /**
         * freq(10KHz) = source clk / (us100Cnt)
         * 计算100us计数器值，用于10KHz基准频率: us100Cnt = source clk / 10000
        */
        us100Cnt = (SGPIO_SOURCE_CLK / 10000);
        sgpioRegs->us100Cnt = us100Cnt;

        ret = EXIT_SUCCESS;
    }
    return ret;
}

/**
 * @brief 根据期望频率和时钟频率计算N值
 *
 * @param desiredFreq 期望频率(Hz)，不能为0
 * @param clkFreq 时钟频率(Hz)，不能为0
 * @return U32 计算得到的N值，如果参数无效或计算结果溢出则返回0
 *
 * @note 计算公式: Freq = 4*CLK/N => N = 4*CLK/Freq
 */
static U32 sgpioCalculateNValue(U32 desiredFreq, U32 clkFreq)
{
    if (desiredFreq == 0) {
        LOGE("%s error: desiredFreq can't be 0\n", __func__);
        return 0;
    }

    if (clkFreq == 0) {
        LOGE("%s error: clkFreq can't be 0\n", __func__);
        return 0;
    }

    /**
     * 根据期望频率和时钟频率计算N值
     * 公式: Freq = 1/(N/4 * 1/CLK) = 4*CLK/N
     * 所以: N = 4*CLK/Freq
     */
    U64 nValue = (U64)4 * clkFreq / desiredFreq;

    if (nValue > 0xFFFFFFFF) {
        LOGE("Error: N value overflow\n");
        return 0;
    }
    LOGD("N value: CLK=%uHz, desiredFreq=%uHz, N=%llu\n", clkFreq, desiredFreq, nValue);

    return (U32)nValue;
}

/**
* @brief 配置SGPIO的闪烁功能
*
* 配置SGPIO的闪烁功能，包括闪烁使能、闪烁模式、输出选择和频率。
*
* @param devId 设备ID
* @param blinkEn 闪烁使能，0 == enable, 1 == disable
* @param blinkMode 闪烁模式,0 == from external input trig, 1 == sio self trigger
* @param blinkOutSel 闪烁输出选择,0 == sgpio0, 1 == sgpio1, 2 == sgpio2, 3 == sgpio3
* @param freq 闪烁频率，单位Hz，范围为1Hz到100kHz
*
* @return 成功返回EXIT_SUCCESS，失败返回错误码
*         -EIO: 设备I/O错误
*         -EINVAL: 无效参数，如频率超出范围或计算N值失败
*/
static S32 sgpioBlinkConfig(DevList_e devId, U8 blinkEn, U8 blinkMode, U8 blinkOutSel, U32 freq)
{
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;
    U32 nValue = 0;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    ret = sgpioGetRegsChecked(devId, &sgpioDrvData, &sgpioRegs);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }
    if (freq < 1 || freq > 100000) {
        LOGE("err: freq %uHz exceed (1Hz - 100kHz)\n", freq);
        return -EINVAL;
    }

    ///< 计算N值
    nValue = sgpioCalculateNValue(freq, SGPIO_SOURCE_CLK);
    if (nValue == 0) {
        LOGE("%s: failed to calculate N value for freq %uHz\r\n", __func__, freq);
        return -EINVAL;
    }

    ///< 配置blink频率
    sgpioRegs->sioBlinkReg = nValue; ///< 设置N值, N = 4 * PCLK / Freq
    ///< 设置闪烁选择寄存器
    sgpioRegs->sioBlinkSel.dword =
        (blinkMode & 0x01) << 4 | (blinkEn & 0x01) << 3 | (blinkOutSel & 0x3);

    LOGD("SGPIO Blink config: freq=%uHz, N value=%u\r\n", freq, nValue);
    return EXIT_SUCCESS;
}

/**
 * @brief 设置SGPIO设备协议模式
 *
 * 根据指定的协议模式配置SGPIO控制器的寄存器，支持STA1005和SFF8485两种模式
 *
 * @param devId 设备ID，范围从DEVICE_SGPIO0到(DEVICE_SGPIO0 + SGPIO_MAX_NUM - 1)
 * @param mode 协议模式，支持SGPIO_STA1005和SGPIO_SFF8485
 *
 * @return 成功返回EXIT_SUCCESS，失败返回错误码：
 *         -EINVAL：参数无效
 *         -EIO：设备驱动数据获取失败
 */
static S32 sgpioSetProtocol(DevList_e devId, SgpioProtocol_e mode)
{
    U32 sgpioCtrlId, sta1005RegAddr, val;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioTopCfg_u *ptrSgpioTopRegs = NULL;

    ///< 1. 验证参数
    if (devId < DEVICE_SGPIO0 || devId >= (DEVICE_SGPIO0 + SGPIO_MAX_NUM)) {
        LOGE("Error: Invalid device ID %d\n", devId);
        return -EINVAL;
    }
    sgpioCtrlId = devId - DEVICE_SGPIO0;

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    if (sgpioDrvData->sbrCfg.regAddr == NULL) {
        LOGE("%s: invalid register address\r\n", __func__);
        return -EINVAL;
    }

    sta1005RegAddr = ((U32)sgpioDrvData->sbrCfg.regAddr - SGPIO_8485_REG_OFFSET + SGPIO_1005_REG_OFFSET);
    ptrSgpioTopRegs = (SgpioTopCfg_u *)(SGPIO_TOP_CRG_REG_BASE + SGPIO_TOP_CRG_REG_OFFSET);

    switch (mode) {
    case SGPIO_STA1005:
        ptrSgpioTopRegs->dword &= ~(1UL << sgpioCtrlId); ///< disable sta1005 mode (clear bit)
        val = reg32Read(sta1005RegAddr);
        val |= 0x0f;
        reg32Write(sta1005RegAddr, val); ///< enable sta1005 mode
        break;
    case SGPIO_SFF8485:
        ptrSgpioTopRegs->dword |= (1UL << sgpioCtrlId); ///< enable sff8485 mode (set bit)
        break;
    default:
        LOGE("Error: Invalid protocol mode %d\n", mode);
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief 设置SGPIO活动映射关系
 *
 * 该函数用于配置SGPIO控制器中指定驱动器与PHY编号的映射关系。
 * 通过读写SGPIO_TOP_CRG_MAP_REG相关寄存器来实现映射配置。
 *
 * @param devId SGPIO设备ID，取值范围：DEVICE_SGPIO0 到 (DEVICE_SGPIO0 + SGPIO_MAX_NUM - 1)
 * @param phyNum PCIe PHY编号，取值范围：0-255
 * @param sgpioActivitySel SGPIO驱动器选择，取值范围：0到(设备驱动器数量-1，最大63)
 *
 * @return 执行状态码
 *   - EXIT_SUCCESS: 操作成功
 *   - -EBUSY: 设备忙，获取锁失败
 *   - -ENODEV: 设备未初始化
 *   - -EINVAL: 参数无效
 *   - -EIO: 获取驱动器数量失败
 */
S32 sgpioSetPcieActivityMapping(DevList_e devId, U32 phyNum, U8 sgpioActivitySel)
{
    S32 ret = EXIT_SUCCESS;
    U32 deviceCount = 0;
    U32 regAddr = 0;
    U32 regVal = 0;
    U32 sgpioCtrlId = 0;
    U32 regIndex = 0;
    U32 byteOffset = 0;

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        return ret;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    ///< 1. 验证参数
    if (devId < DEVICE_SGPIO0 || devId >= (DEVICE_SGPIO0 + SGPIO_MAX_NUM)) {
        LOGE("Error: Invalid device ID %d\n", devId);
        ret = -EINVAL;
        goto exit;
    }
    sgpioCtrlId = devId - DEVICE_SGPIO0;

    if (phyNum > 0xFF) {
        LOGE("Error: Invalid PCIe PHY number %u\n", phyNum);
        ret = -EINVAL;
        goto exit;
    }

    ret = sgpioGetDriveCount(devId, &deviceCount);
    if (ret != EXIT_SUCCESS) {
        LOGE("Error: Failed to get drive count for SGPI0 controller %d\n", sgpioCtrlId);
        ret = -EIO;
        goto exit;
    }

    if (sgpioActivitySel >= deviceCount) {
        LOGE("Error: SGPIO driver selection %u is out of range (0-63)\n", sgpioActivitySel);
        ret = -EINVAL;
        goto exit;
    }

    ///< 2. 计算寄存器地址
    ///< 每个寄存器4字节，控制4个driver
    regIndex = sgpioActivitySel / 4;
    ///< 每个driver在寄存器内占8-bit (1 byte)
    byteOffset = sgpioActivitySel % 4;

    ///< 计算最终的寄存器地址
    regAddr =
        SGPIO_TOP_CRG_MAP_REG_ADDR + (sgpioCtrlId * SGPIO_TOP_CRG_MAP_BLOCK_SIZE) + (regIndex * 4);

    ///< 3. 读-改-写
    regVal = reg32Read(regAddr);
    ///< 清除对应的8-bit
    regVal &= ~(0xFF << (byteOffset * 8));
    ///< 写入新的phyNum
    regVal |= (phyNum & 0xFF) << (byteOffset * 8);
    reg32Write(regAddr, regVal);

    LOGD("Set SGPIO%u, Driver %u -> PHY %u. Reg[0x%08x]=0x%08x\n", sgpioCtrlId, sgpioActivitySel,
        phyNum, regAddr, regVal);

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief 打印SGPIO控制器中PCIe活动映射表（驱动器到PHY的映射）
 * 
 * @param devId 设备ID，必须是有效的SGPIO设备ID
 * 
 * @return 执行状态：
 *         - EXIT_SUCCESS: 成功
 *         - -EBUSY: 设备忙，获取锁失败
 *         - -ENODEV: 设备未初始化
 *         - -EINVAL: 无效的设备ID
 *         - -EIO: 获取驱动器数量失败
 * 
 * @note 函数会打印出SGPIO控制器中所有驱动器到PHY的映射关系表，
 *       格式为"Driver\tPHY"，最多显示64个驱动器。
 */
S32 sgpioDumpPcieActivityMapping(DevList_e devId)
{
    U32 sgpioCtrlId = 0;
    U32 regAddr = 0;
    U32 regVal = 0;
    U32 deviceCount = 0;
    U32 regIndex = 0;
    U32 byteOffset = 0;
    U32 phyNum = 0;
    S32 ret = EXIT_SUCCESS;
    U32 i = 0;

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        return ret;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    ///< 1. 验证参数
    if (devId < DEVICE_SGPIO0 || devId >= (DEVICE_SGPIO0 + SGPIO_MAX_NUM)) {
        LOGE("Error: Invalid device ID %d\n", devId);
        ret = -EINVAL;
        goto exit;
    }
    sgpioCtrlId = devId - DEVICE_SGPIO0;

    ret = sgpioGetDriveCount(devId, &deviceCount);
    if (ret != EXIT_SUCCESS) {
        LOGE("Error: Failed to get drive count for SGPIO controller %d\n", sgpioCtrlId);
        ret = -EIO;
        goto exit;
    }

    ///< 限制deviceCount不超过最大值
    if (deviceCount > SGPIO_DRIVER_COUNT_MAX) {
        deviceCount = SGPIO_DRIVER_COUNT_MAX;
    }

    ///< 打印表头
    LOGI("SGPIO%u PCIe Activity Mapping Table (Driver -> PHY):\n", sgpioCtrlId);
    LOGI("Driver\tPHY\n");
    LOGI("------\t---\n");

    ///< 2. 遍历所有可能的driver (最多64个)
    for (i = 0; i < deviceCount; i++) {
        ///< 计算寄存器地址
        regIndex = i / 4;
        byteOffset = i % 4;

        regAddr = SGPIO_TOP_CRG_MAP_REG_ADDR + (sgpioCtrlId * SGPIO_TOP_CRG_MAP_BLOCK_SIZE) +
            (regIndex * 4);

        ///< 读取寄存器值
        regVal = reg32Read(regAddr);

        ///< 提取对应的PHY编号
        phyNum = (regVal >> (byteOffset * 8)) & 0xFF;

        ///< 打印映射关系
        LOGI("%u\t%u\n", i, phyNum);
    }

exit:
    devUnlockByDriver(devId);
    return ret;
}

static S32 sgpioHwInit(DevList_e devId, SgpioDrvData_s *sgpioDrvData)
{
    S32 ret = EXIT_SUCCESS;

    if (sgpioDrvData == NULL) {
        LOGE("%s: invalid drv data\r\n", __func__);
        return -EINVAL;
    }

    if (sgpioDrvData->sbrCfg.mode == 2) {
        ret = sgpioSetProtocol(devId, SGPIO_STA1005);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: set protocol failed %d\r\n", __func__, ret);
            return ret;
        }
    } else if (sgpioDrvData->sbrCfg.mode == 0 || sgpioDrvData->sbrCfg.mode == 1) {
        ret = sgpioSetProtocol(devId, SGPIO_SFF8485);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: set protocol failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioSloadEnable(devId);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio sload enable failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioSetSClock(devId, sgpioDrvData->sbrCfg.cfgClk);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio set sclock failed %d\r\n", __func__, ret);
            return ret;
        }

        /* register isr */
        if (sgpioDrvData->sbrCfg.irqEn == 1) {
            ret = sgpioRegisterIsr(
                devId, sgpioIsr, sgpioDrvData->sbrCfg.irqNum, sgpioDrvData->sbrCfg.irqPrio);
            if (ret != EXIT_SUCCESS) {
                LOGE("%s: sgpio register isr failed %d\r\n", __func__, ret);
                return ret;
            }
        }

        /* 模式选择 */
        if (sgpioDrvData->sbrCfg.mode == 0) {
            /* normal mode */
            ret = sgpioSetGpTxSload(devId, 0x0);
            if (ret != EXIT_SUCCESS) {
                LOGE("%s: sgpio set gp tx sload failed %d\r\n", __func__, ret);
                return ret;
            }
        } else {
            /* GP mode */
            ret = sgpioSetGpTxSload(devId, 0x1);
            if (ret != EXIT_SUCCESS) {
                LOGE("%s: sgpio set gp tx sload failed %d\r\n", __func__, ret);
                return ret;
            }
        }

        ret = sgpioSetCfg1(devId, sgpioDrvData->sbrCfg.cfg1Timing);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio set cfg1 failed %d\r\n", __func__, ret);
            return ret;
        }

        // TODO maybe need sbr
        ret = sgpioDriveActivityMask(devId, 0xffffffff, 0xffffffff);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio drive activity mask failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioSetDriveCount(devId, sgpioDrvData->sbrCfg.driveNum);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio set drive count failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioBlinkConfig(devId, sgpioDrvData->sbrCfg.blinkEn, sgpioDrvData->sbrCfg.blinkMode,
            sgpioDrvData->sbrCfg.blinkOutSel, sgpioDrvData->sbrCfg.blinkFreq);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio blink config failed %d\r\n", __func__, ret);
            return ret;
        }

        ret = sgpioEnable(devId);
        if (ret != EXIT_SUCCESS) {
          LOGE("%s: sgpio enable failed %d\r\n", __func__, ret);
          return ret;
        }

    } else {
        LOGE("%s: invalid sgpio mode %d\r\n", __func__, sgpioDrvData->sbrCfg.mode);
        return -EINVAL;
    }
    return ret;
}

/**
 * @brief SGPIO设备初始化函数
 *
 * 初始化指定设备的SGPIO控制器，包括设备锁定验证、驱动匹配检查、
 * 内存分配、设备配置获取、硬件复位、驱动安装、pinmux配置和硬件初始化等步骤。
 *
 * @param devId 设备ID，指定要初始化的SGPIO设备
 *
 * @return 成功返回EXIT_SUCCESS(0)，失败返回负数错误码：
 *         -EBUSY   设备忙或已初始化
 *         -EINVAL  设备不匹配或无效
 *         -ENOMEM  内存分配失败
 *         -EIO     设备配置获取失败、复位失败或驱动安装失败
 */
S32 sgpioInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (isDrvInit(devId) == true) {
        ret = -EBUSY;
        goto exit;
    }

    sgpioDrvData = (SgpioDrvData_s *)calloc(1, sizeof(SgpioDrvData_s));
    if (sgpioDrvData == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    if (sgpioGetDevCfg(devId, &sgpioDrvData->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto cleanup;
    }

    if (drvInstall(devId, sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto cleanup;
    }

    ret = peripsReset(devId);
    if ((ret != EXIT_SUCCESS) && (ret != -ENXIO)) {
        LOGE("%s: perips reset failed %d\r\n", __func__, ret);
        goto cleanup;
    }

    ret = peripsClockEnable(devId);
    if ((ret != EXIT_SUCCESS) && (ret != -ENXIO)) {
        LOGE("%s: perips clock enable failed %d\r\n", __func__, ret);
        goto cleanup;
    }

    ///< TODO：需要soc提供pinmux配置函数
    sgpioConfigurePinmux();

    ret = sgpioHwInit(devId, sgpioDrvData);
    if (ret != EXIT_SUCCESS) {
        goto cleanup;
    }

cleanup:
    if (ret != EXIT_SUCCESS) {
        if (isDrvInit(devId)) {
            ///< 如果驱动已注册，则调用drvUninstall来清理（包括释放sgpioDrvData）
            if (drvUninstall(devId) != EXIT_SUCCESS) {
                LOGE("%s: drvUninstall failed during cleanup\n", __func__);
                if (sgpioDrvData != NULL) { ///< 作为最后的保障，防止内存泄漏
                    free(sgpioDrvData);
                }
            }
        } else if (sgpioDrvData != NULL) {
            ///< 如果驱动未注册但内存已分配，则手动释放
            free(sgpioDrvData);
        }
    }
exit:
    devUnlockByDriver(devId);
    return ret;
}

/*
 * sgpioDeInit - SGPIO设备反初始化函数
 *
 * 此函数用于反初始化指定的SGPIO设备，包括禁用设备、注销中断服务程序、
 * 卸载驱动、复位外设和关闭时钟等操作。
 *
 * @param devId: 设备ID，指定要反初始化的SGPIO设备
 *
 * @return: 执行结果
 *          EXIT_SUCCESS - 反初始化成功
 *          -EBUSY      - 设备忙，锁定失败
 *          -EINVAL     - 设备ID不匹配SGPIO驱动
 *          -ENODEV     - 设备未初始化
 *          -EIO        - 设备驱动获取失败或驱动卸载失败
 */
S32 sgpioDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    U8 irqEn = 0;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvInit(devId)) {
        ret = -ENODEV;
        goto exit;
    }

    ret = sgpioDisable(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: sgpio disable failed %d\r\n", __func__, ret);
        ///< 继续清理，不直接退出
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    irqEn = sgpioDrvData->sbrCfg.irqEn;
    if (irqEn == 1) {
        /* unregister isr */
        ret = sgpioUnRegisterIsr(devId, sgpioIsr);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: sgpio unregister isr failed %d\r\n", __func__, ret);
            goto exit;
        }
    }

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    ret = peripsReset(devId);
    if ((ret != EXIT_SUCCESS) && (ret != -ENXIO)) {
        LOGE("%s: perips reset failed %d\r\n", __func__, ret);
        ///< 继续清理，不直接退出
    }

    ret = peripsClockDisable(devId);
    if ((ret != EXIT_SUCCESS) && (ret != -ENXIO)) {
        LOGE("%s: perips clock disable failed %d\r\n", __func__, ret);
        ///< 继续清理，不直接退出
    }

    ret = EXIT_SUCCESS;
exit:
    /* unlock */
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief 获取SGPIO驱动计数
 *
 * @param devId 设备ID
 * @param count 用于存储驱动计数的指针
 *
 * @return 执行状态码
 *   - EXIT_SUCCESS 成功
 *   - -EBUSY 设备忙，获取锁失败
 *   - -ENODEV 设备未初始化
 *   - -EINVAL 无效参数或设备不匹配
 *   - -EIO 获取设备驱动数据失败
 */
S32 sgpioGetDriveCount(DevList_e devId, U32 *count)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    if (count == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    *count = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr)->sgpioCfg0.fields.sdriveCount;
    ret = EXIT_SUCCESS;

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief 设置SGPIO驱动器的驱动器数量
 *
 * 该函数用于配置指定SGPIO设备的驱动器数量，包括设置SGPIO配置寄存器
 * 和厂商特定寄存器中的驱动器数量字段。
 *
 * @param devId 设备ID，指定要配置的SGPIO设备
 * @param count 要设置的驱动器数量，必须在有效范围内
 *              (SGPIO_DRIVER_COUNT_MIN 到 sbrCfg.driveNum)
 *
 * @return 执行状态码：
 *         - EXIT_SUCCESS: 操作成功完成
 *         - -EBUSY: 设备忙，无法获取锁
 *         - -EIO: 设备未初始化或不匹配，或获取设备驱动失败
 *         - -EINVAL: 驱动器数量参数超出有效范围
 */
S32 sgpioSetDriveCount(DevList_e devId, U32 count)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *sgpioRegs = NULL;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EIO;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if ((count > sgpioDrvData->sbrCfg.driveNum) || (count < SGPIO_DRIVER_COUNT_MIN)) {
        ret = -EINVAL;
        goto exit;
    }

    sgpioRegs = ((SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr);

    sgpioRegs->sgpioCfg0.fields.sdriveCount = count;
    sgpioRegs->vendorSpec1.fields.sgpioDeviceCnt = count;

    ret = EXIT_SUCCESS;

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief 设置SGPIO LED状态
 *
 * @param devId 设备ID
 * @param phyNum 物理编号
 * @param ledType LED类型(SGPIO_LED_ERROR/ACTIVITY/LOCATE)
 * @param ledState LED状态(SGPIO_LED_STATE_OFF/ON/BLINK)
 *
 * @return 成功返回EXIT_SUCCESS，失败返回错误码:
 *         -EBUSY: 设备忙
 *         -ENODEV: 设备未初始化
 *         -EINVAL: 参数无效
 *         -EIO: 获取驱动数据失败
 */
S32 sgpioSetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e ledState)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 txnIndex, txnOffset;
    U32 currentValue, newValue = 0;
    U32 mask = 0, pattern = 0;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    // 参数检查
    if (ledType >= SGPIO_LED_MAX || ledState >= SGPIO_LED_STATE_MAX) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (phyNum >= sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: phyNum %d is out of range\r\n", __func__, phyNum);
        ret = -EINVAL;
        goto exit;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    ///< 计算driver索引和txn寄存器索引及偏移
    ///< 每个txn控制4个驱动器
    txnIndex = phyNum / 4;
    txnOffset = phyNum % 4;

    ///< 获取当前值
    currentValue = reg->txn[txnIndex].dword;

    ///< 根据LED类型确定掩码和位移
    switch (ledType) {
    case SGPIO_LED_ERROR: ///< [2:0]
        mask = 0x7 << (txnOffset * 8);
        pattern = sgpioLedPatternMap[SGPIO_LED_ERROR][ledState] << (txnOffset * 8);
        break;
    case SGPIO_LED_LOCATE: ///< [4:3]
        mask = 0x3 << (3 + txnOffset * 8);
        pattern = sgpioLedPatternMap[SGPIO_LED_LOCATE][ledState] << (3 + txnOffset * 8);
        break;
    case SGPIO_LED_ACTIVITY: ///< [7:5]
        mask = 0x7 << (5 + txnOffset * 8);
        pattern = sgpioLedPatternMap[SGPIO_LED_ACTIVITY][ledState] << (5 + txnOffset * 8);
        break;
    default:
        ret = -EINVAL;
        goto exit;
    }

    ///< 清除原值并设置新值
    newValue = (currentValue & ~mask) | pattern;
    reg->txn[txnIndex].dword = newValue;

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief 获取SGPIO LED的状态
 *
 * @param devId 设备ID，标识要操作的SGPIO设备
 * @param phyNum 物理编号，指定要查询的驱动器编号
 * @param ledType LED类型，指定要查询的LED类型（错误/定位/活动）
 * @param pLedState 输出参数，用于返回LED状态
 *
 * @return 执行结果：
 *         - EXIT_SUCCESS: 成功
 *         - -EBUSY: 设备忙，获取锁失败
 *         - -ENODEV: 设备未初始化
 *         - -EINVAL: 参数无效或设备不匹配
 *         - -EIO: 获取设备驱动数据失败
 */
S32 sgpioGetLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, SgpioLedState_e *pLedState)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 txnIndex, txnOffset;
    U32 currentValue;
    U32 mask, value;
    U32 pattern;
    int i;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    ///< 参数检查
    if (ledType >= SGPIO_LED_MAX || pLedState == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (phyNum >= sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: phyNum %d is out of range\r\n", __func__, phyNum);
        ret = -EINVAL;
        goto exit;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    ///< 计算driver索引和txn寄存器索引及偏移
    ///< 每个txn控制4个驱动器
    txnIndex = phyNum / 4;
    txnOffset = phyNum % 4;

    ///< 获取当前值
    currentValue = reg->txn[txnIndex].dword;

    ///< 根据LED类型确定掩码和位移
    switch (ledType) {
    case SGPIO_LED_ERROR: ///< [2:0]
        mask = 0x7 << (txnOffset * 8);
        value = (currentValue & mask) >> (txnOffset * 8);
        break;
    case SGPIO_LED_LOCATE: ///< [4:3]
        mask = 0x3 << (3 + txnOffset * 8);
        value = (currentValue & mask) >> (3 + txnOffset * 8);
        break;
    case SGPIO_LED_ACTIVITY: ///< [7:5]
        mask = 0x7 << (5 + txnOffset * 8);
        value = (currentValue & mask) >> (5 + txnOffset * 8);
        break;
    default:
        ret = -EINVAL;
        goto exit;
    }

    ///< 查找匹配的LED状态
    pattern = (U8)value;
    *pLedState = SGPIO_LED_OFF; ///< 默认状态

    for (i = 0; i < SGPIO_LED_STATE_MAX; i++) {
        if (sgpioLedPatternMap[ledType][i] == pattern) {
            *pLedState = (SgpioLedState_e)i;
            break;
        }
    }

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief  Set sgpio general purpose driver led state directly
 *
 * 该函数用于控制SGPIO设备的LED指示灯状态，支持活动灯、定位灯和错误灯三种类型。
 * 每个物理端口对应3个LED位(活动、定位、错误)，通过寄存器位操作控制LED的开关。
 *
 * @param devId    设备ID，指定要操作的SGPIO设备
 * @param phyNum   物理端口号，范围0到驱动器数量-1
 * @param ledType  LED类型，支持SGPIO_LED_ACTIVITY(活动)、SGPIO_LED_LOCATE(定位)、SGPIO_LED_ERROR(错误)
 * @param ledOn    LED开关状态，true表示开启，false表示关闭
 *
 * @return 执行结果，EXIT_SUCCESS表示成功，负数表示错误码：
 *         -EBUSY  设备忙，无法获取锁
 *         -ENODEV 设备未初始化
 *         -EINVAL 参数无效或设备不匹配
 *         -EIO    获取设备驱动数据失败
 */
S32 sgpioSetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool ledOn)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 totalBitOffset, startRegIndex, startBitOffset;
    U32 bitInLedType;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    ///< 参数检查
    if (phyNum >= sgpioDrvData->sbrCfg.driveNum || ledType >= SGPIO_LED_MAX) {
        ret = -EINVAL;
        goto exit;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    ///< 3-bit 布局: [bit0:activity, bit1:locate, bit2:error]
    switch (ledType) {
    case SGPIO_LED_ERROR:
        bitInLedType = 2;
        break;
    case SGPIO_LED_LOCATE:
        bitInLedType = 1;
        break;
    case SGPIO_LED_ACTIVITY:
        bitInLedType = 0;
        break;
    default: ///< Should not happen
        ret = -EINVAL;
        goto exit;
    }

    ///< 计算目标位在192-bit虚拟空间中的总偏移
    totalBitOffset = phyNum * 3 + bitInLedType;

    ///< 计算目标位所在的寄存器索引和寄存器内偏移
    startRegIndex = totalBitOffset / 32;
    startBitOffset = totalBitOffset % 32;

    ///< 检查寄存器索引是否越界
    if (startRegIndex >= 6) {
        ret = -EINVAL;
        goto exit;
    }

    ///< 读取、修改、写回寄存器
    U32 value = reg->txgp[startRegIndex].dword;
    if (ledOn) {
        value |= (1UL << startBitOffset);
    } else {
        value &= ~(1UL << startBitOffset);
    }
    reg->txgp[startRegIndex].dword = value;

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * 获取GP模式下指定PHY的LED状态
 *
 * @param devId    设备ID，指定要操作的SGPIO设备
 * @param phyNum   物理端口号，范围0到SGPIO_DRIVER_COUNT_MAX-1
 * @param ledType  LED类型，指定要查询的LED种类（错误/定位/活动）
 * @param pLedOn   输出参数，返回LED状态（true表示点亮，false表示熄灭）
 *
 * @return 执行结果代码：
 *         - EXIT_SUCCESS: 操作成功
 *         - -EBUSY: 设备忙，获取锁失败
 *         - -ENODEV: 设备未初始化或不存在
 *         - -EINVAL: 参数无效或设备不匹配
 *         - -EIO: 获取设备驱动数据失败
 */
S32 sgpioGetGpLedState(DevList_e devId, U8 phyNum, SgpioLedType_e ledType, bool *pLedOn)
{
    S32 ret = EXIT_SUCCESS;
    SgpioDrvData_s *sgpioDrvData = NULL;
    SgpioReg_s *reg = NULL;
    U32 totalBitOffset, regIndex, bitOffset;
    U32 bitInLedType;

    /* lock */
    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        return -EBUSY;
    }

    if (isDrvInit(devId) == false) {
        ret = -ENODEV;
        goto exit;
    }

    ///< 参数检查
    if (pLedOn == NULL || ledType >= SGPIO_LED_MAX) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_V1P0_SGPIO)) {
        ret = -EINVAL;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&sgpioDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    if (phyNum >= sgpioDrvData->sbrCfg.driveNum) {
        LOGE("%s: phyNum %d is out of range\r\n", __func__, phyNum);
        ret = -EINVAL;
        goto exit;
    }

    reg = (SgpioReg_s *)sgpioDrvData->sbrCfg.regAddr;

    ///< 3-bit布局: [bit0:activity, bit1:locate, bit2:error]
    switch (ledType) {
    case SGPIO_LED_ERROR:
        bitInLedType = 2;
        break;
    case SGPIO_LED_LOCATE:
        bitInLedType = 1;
        break;
    case SGPIO_LED_ACTIVITY:
        bitInLedType = 0;
        break;
    default:
        ret = -EINVAL;
        goto exit;
    }

    totalBitOffset = phyNum * 3 + bitInLedType;
    regIndex = totalBitOffset / 32;
    bitOffset = totalBitOffset % 32;

    if (regIndex >= 6) {
        ret = -EINVAL;
        goto exit;
    }

    U32 value = reg->txgp[regIndex].dword;
    *pLedOn = (value & (1UL << bitOffset)) ? true : false;

exit:
    devUnlockByDriver(devId);
    return ret;
}