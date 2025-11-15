/**
 * copyright (C), 2021, Start Micro System Technologies Co. Ltd.
 *
 * @file        drv_rtems_sgpio.c
 * @author      yezh
 * @date        2021/05/26
 * @brief       expander sgpio driver file
 * @note        代码未测试@2025-06-10
 */

#include "drv_sgpio.h"
#include "drv_sgpio_api.h"

#include "udelay.h"

#define SGPIO_PORT_MAX  96 /* TODO yangzhl3 move it to common_config.h */

static SgpioCtrl_s gSgpioCtrl[SGPIO_PORT_MAX] =
{
    {SGPIO_PORT0_BASEADDR,0,0},
    {SGPIO_PORT1_BASEADDR,0,0}
};

/**
 * @brief   : 获取全局结构体
 * @param   : sgpioId -sgpio IP 索引
 * @return  : 返回全局结构体指针;
 */
SgpioCtrl_s *sgpioCtrlGet(SgpioPortId_e sgpioId)
{
    SgpioCtrl_s *sgpioCtrl = NULL;
    if(sgpioId >= SGPIO_PORT_MAX) {
        LOGE("sgpio Id %d is invalid\n", sgpioId);
        goto end;
    }
    sgpioCtrl = &gSgpioCtrl[sgpioId];
end:
    return sgpioCtrl;
}

/**
 * @brief   : sgpio使能控制
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 */
static void sgpioRegCfg0Enable(SgpioCtrl_s *sgpioCtrl, U8 enable)
{
    SgpioReg_u regVal;

    regVal.all                 = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0));
    regVal.sgpioConfig0.gpioEn = enable;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0), regVal.all);
}

/**
 * @brief   : sgpio sio blink 功能使能控制
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 */
static void sgpioRegSioBlinkEnable(SgpioCtrl_s *sgpioCtrl, U8 enable)
{
    SgpioReg_u regVal;

    regVal.all                           = REG32_READ(sgpioCtrl->base, SGPIO_SIO_OFFSET(0));
    regVal.sgpioSioBlinkSel.sioBlinkMask = enable;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SIO_OFFSET(0), regVal.all);
}

/**
 * @brief   : 获取配置寄存器个数
 * @param   : sgpioCtrl -全局结构体
 * @return  : void;
 */
static S32 sgpioRegCfgCountGet(SgpioCtrl_s *sgpioCtrl)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0));
    return (S32)regVal.sgpioConfig0.cfgRegCnt;
}

/**
 * @brief   : 获取PG寄存器个数
 * @param   : sgpioCtrl -全局结构体
 * @return  : void;
 */
static S32 sgpioRegPGCountGet(SgpioCtrl_s *sgpioCtrl)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0));
    return (S32)regVal.sgpioConfig0.GPRegCnt;
}

/**
 * @brief   : 获取sgpio的版本
 * @param   : sgpioCtrl -全局结构体
 * @return  : version;
 */
static U32 sgpioRegVersionGet(SgpioCtrl_s *sgpioCtrl)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0));
    return regVal.sgpioConfig0.version;
}

/**
 * @brief   : 设置指定寄存器的bitOrder
 * @param   : sgpioCtrl -全局结构体
 * @param   : regIdx -寄存器组的索引
 * @param   : data -设置的值
 * @return  : void;
 */
static void sgpioRegBitOrderSet(SgpioCtrl_s *sgpioCtrl, U32 regIdx, U32 data)
{
    REG32_WRITE(sgpioCtrl->base, SGPIO_BITORDER_SEL_OFFSET(regIdx), data);
}

/**
 * @brief   : 获取指定寄存器的bitOrder
 * @param   : sgpioCtrl -全局结构体
 * @param   : regIdx -寄存器组的索引
 * @return  : 寄存器value;
 */
static U32 sgpioRegBitOrderGet(SgpioCtrl_s *sgpioCtrl, U32 regIdx)
{
    U32 regVal;
    regVal = REG32_READ(sgpioCtrl->base, SGPIO_BITORDER_SEL_OFFSET(regIdx));
    return regVal;
}

/**
 * @brief   : 获取指定寄存器的vender SPEC
 * @param   : sgpioCtrl -全局结构体
 * @param   : regIdx -寄存器组的索引
 * @return  : 寄存器value;
 */
static U32 sgpioRegVenderSpecGet(SgpioCtrl_s *sgpioCtrl, U32 regIdx)
{
    U32 regVal;
    regVal = REG32_READ(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(regIdx));
    return regVal;
}

/**
 * @brief   : sgpio状态机控制
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 */
static void sgpioRegFsmResetEnable(SgpioCtrl_s *sgpioCtrl, U8 enable)
{
    SgpioReg_u regVal;
    regVal.all                  = REG32_READ(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(0));
    regVal.sgpioVspec0.resetFsm = enable;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(0), regVal.all);
}

/**
 * @brief   : sgpio的sload设置
 * @param   : sgpioCtrl -全局结构体
 * @param   : value -sload值
 * @return  : void;
 */
static void sgpioRegSloadSet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all                         = REG32_READ(sgpioCtrl->base, SGPIO_GP_TRANS_OFFSET(0));
    regVal.sgpioGPTranCfg.sloadPattern = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_GP_TRANS_OFFSET(0), regVal.all);
}

/**
 * @brief   : sgpio的sload读取
 * @param   : sgpioCtrl -全局结构体
 * @return  : sload;
 */
static U32 sgpioRegSloadGet(SgpioCtrl_s *sgpioCtrl)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_GP_TRANS_OFFSET(0));
    return regVal.sgpioGPTranCfg.sloadPattern;
}

/**
 * @brief   : sgpio的剩余重复次数
 * @param   : sgpioCtrl -全局结构体
 * @param   : value -timer次数
 * @return  : void;
 */
static void sgpioRegTransTimeSet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all                  = REG32_READ(sgpioCtrl->base, SGPIO_GP_TRANS_OFFSET(0));
    regVal.sgpioGPTranCfg.count = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_GP_TRANS_OFFSET(0), regVal.all);
}

/**
 * @brief   : sgpio的driver 数设置，需要两个寄存器数据一致
 * @param   : sgpioCtrl -全局结构体
 * @param   : value -设置值
 * @return  : void;
 */
static void sgpioRegDriverCountSet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all                        = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0));
    regVal.sgpioConfig0.supportDrvCnt = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0), regVal.all);

    regVal.all                  = REG32_READ(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(1));
    regVal.sgpioVspec1.devCount = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(1), regVal.all);
}

/**
 * @brief   : sgpio的driver 数获取
 * @param   : sgpioCtrl -全局结构体
 * @return  : 寄存器值;
 */
static U32 sgpioRegDriverCountGet(SgpioCtrl_s *sgpioCtrl)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0));
    return regVal.sgpioConfig0.supportDrvCnt;
}

/**
 * @brief   : sgpio的系统时钟使能
 * @param   : sgpioId -sgpio control 索引
 * @return  : void;
 */
void sgpioSysClkEnable(SgpioPortId_e sgpioId)
{
    S32 bitn              = 0;
    volatile U32 *clkCtrl = NULL;

    clkCtrl = (volatile U32 *)(SGPIO_SYS_BASE + SGPIO_SYS_CLK_OFFSET);
    if(sgpioId) {
        bitn = SGPIO_PORT1_CLK_BIT;
    } else {
        bitn = SGPIO_PORT0_CLK_BIT;
    }
    SET_BIT(*clkCtrl, bitn);
}

/**
 * @brief   : sgpio的系统复位
 * @param   : sgpioId -sgpio control 索引
 * @return  : void;
 */
void sgpioSysReset(SgpioPortId_e sgpioId)
{
    S32 bitn                = 0;
    volatile U32 *resetCtrl = NULL;

    resetCtrl = (volatile U32 *)(SGPIO_SYS_BASE + SGPIO_SYS_RESET_OFFSET);
    if(sgpioId) {
        bitn = SGPIO_PORT1_CLK_BIT;
    } else {
        bitn = SGPIO_PORT0_CLK_BIT;
    }
#ifdef PS3_PRODUCT_EXPANDER
    CLR_BIT(*resetCtrl, bitn);
    (void)udelay(1000);
    SET_BIT(*resetCtrl, bitn);
#else

#ifdef PS3_MODEL_V300
    reg32Write(SGPIO_SYS_BASE + SGPIO_SYS_RESET_CCU_OFFSET, CCU_RESET_VAL);
#endif
    SET_BIT(*resetCtrl, bitn);
    (void)udelay(1000);
    CLR_BIT(*resetCtrl, bitn);
#ifdef PS3_MODEL_V300
    reg32Write(SGPIO_SYS_BASE + SGPIO_SYS_RESET_CCU_OFFSET, SGPIO_VALUE_0);
#endif

#endif
}

#ifdef PS3_PRODUCT_SWITCH
/**
 * @brief   : sgpio1功能使能，需要配置复用寄存器
 * @param   : void
 * @return  : void;
 */
void sgpioConfigPinMux(void)
{
    U32 map;
    ///< 该GPIO的第一功能是LED，使用SGPIO需要配置第二功能模式
    map = (reg32Read(SGPIO1_PIN_MUX_ADDR1)) | SGPIO1_PIN_MUX_VAL1;
    reg32Write(SGPIO1_PIN_MUX_ADDR1, map);
}

/**
 * @brief   : sgpio1功能使能，需要配置复用寄存器
 * @param   : void
 * @return  : void;
 */
void sgpio8485PinMux(void)
{
    U32 map;
    ///< 该GPIO的第一功能是LED，使用SGPIO需要配置第二功能模式
    map = (reg32Read(SGPIO_8485_BASE)) | SGPIO_8485_BIT;
    reg32Write(SGPIO_8485_BASE, map);
}
#else
/**
 * @brief   : sgpio1功能使能，需要配置复用寄存器
 * @param   : void
 * @return  : void;
 */
void sgpioConfigPinMux(void)
{
    ///< 该GPIO的第一功能是LED，使用SGPIO需要配置第二功能模式
    reg32Write(SGPIO1_PIN_MUX_ADDR1, SGPIO1_PIN_MUX_VAL1);
    reg32Write(SGPIO1_PIN_MUX_ADDR2, SGPIO1_PIN_MUX_VAL2);
}
#endif

/**
 * @brief   : 获取sgpio最大支持phy
 * @param   : void
 * @return  : phy num;
 */
U32 sgpioPhyNumGet(void)
{
    U32 phyNum = SGPIO_PHY_NUM_MAX;
    return phyNum;
}

/**
 * @brief   : sgpio 根据寄存器分组，解析reg个数
 * @param   : sgpioId -sgpio control 索引
 * @param   : regGroup -寄存器组枚举
 * @return  : reg num;
 */
S32 sgpioGroupRegCntGet(SgpioPortId_e sgpioId, SgpioRegGroup_e regGroup)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;
    S32 regCount           = 0;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }
    switch(regGroup) {
    case SGPIO_SELECT_CONFIG_SGPIO:
        regCount = sgpioRegCfgCountGet(sgpioCtrl);
        break;
    case SGPIO_SELECT_RECEIVE:
    case SGPIO_SELECT_TRANSMIT:
    case SGPIO_SELECT_BIT_ORDER_SEL:
        regCount = (S32)(sgpioPhyNumGet( ) + SGPIO_PHY_PER_REG - 1) / SGPIO_PHY_PER_REG;
        break;
    case SGPIO_SELECT_GP_RECEIVE:
    case SGPIO_SELECT_GP_TRANSMIT:
        regCount = sgpioRegPGCountGet(sgpioCtrl);
        regCount = +1; ///< 加上GP_cfg寄存器
        break;
    case SGPIO_SELECT_VENDER_SPECIFIC:
        regCount = SGPIO_VENDER_SPEC_REGS;
        break;
    default:
        LOGE("sgpio reg group params is invalid\n");
        regCount = -SGPIO_RET_ERR;
        break;
    }
    ret = regCount;
end:
    return ret;
}

/**
 * @brief   : sgpio 版本获取
 * @param   : sgpioId -sgpio control 索引
 * @return  : revion;
 */
S32 sgpioVersionGet(SgpioPortId_e sgpioId)
{
    S32 ret                = -SGPIO_RET_ERR;
    U32 version            = 0;
    SgpioCtrl_s *sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }
    version = sgpioRegVersionGet(sgpioCtrl);

    ret = (S32)version;
end:
    return ret;
}

/**
 * @brief   : sgpio 根据寄存器组，写连续数据到寄存器
 * @param   : sgpioId -sgpio control 索引
 * @param   : regGroup -寄存器组枚举类型
 * @param   : regIdx -reg起始索引
 * @param   : regCount -连续的写寄存器个数
 * @param   : pData -写数据buf
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioGroupRegsWrite(SgpioPortId_e sgpioId, SgpioRegGroup_e regGroup, U32 regIdx, U32 regCount, U32 *pData)
{
    S32 ret                = -SGPIO_RET_ERR;
    volatile U32 *pDst     = NULL; ///< 首地址
    SgpioCtrl_s *sgpioCtrl = NULL;
    U32 i;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

#if 0
    if((NULL == pData) || ((regIdx + regCount) > sgpioGroupRegCntGet(sgpioId, regGroup))) {
        LOGE("pData:%p regIdx:%d regCount:%d xxx:%d\n",pData,regIdx,
            regCount,sgpioGroupRegCntGet(sgpioId, regGroup));
        LOGE("sgpio group regs params is invalid\n");
        goto end;
    }
#endif
    ///< DRV_LOG_INFO("Write:  sgpioId=%u, regGroup=%u, regIdx=%u, regCount=%u, pData=%u",
    ///<     sgpioId, regGroup, regIdx, regCount, *pData);
    switch(regGroup) {
    case SGPIO_SELECT_CONFIG_SGPIO:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_CFG_REG_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_TRANSMIT:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_TRANS_REG_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_GP_TRANSMIT:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_GP_TRANS_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_VENDER_SPECIFIC:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_SPEC_REG_OFFSET(regIdx));
        break;
    case SGPIO_SELECT_BIT_ORDER_SEL:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_BITORDER_SEL_OFFSET(regIdx));
        break;
    default:
        LOGE("sgpio reg group write is invalid\n");
        goto end;
    }
    for(i = 0; i < regCount; i++) {
        ///< DRV_LOG_INFO("write: sgpioCtrl->base=%u, i=%u, regCount=%u, pDst=%p, pData[%u]=%u",
        ///<     sgpioCtrl->base, i,regCount,pDst,i,pData[i]);
        *pDst = pData[i];
        pDst++;
    }
    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio 根据寄存器组，读连续寄存器到databuf
 * @param   : sgpioId -sgpio control 索引
 * @param   : regGroup -寄存器组枚举类型
 * @param   : regIdx -reg起始索引
 * @param   : regCount -连续的写寄存器个数
 * @param   : pData -读数据buf
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioGroupRegsRead(SgpioPortId_e sgpioId, SgpioRegGroup_e regGroup, U32 regIdx, U32 regCount, U32 *pData)
{
    S32 ret                = -SGPIO_RET_ERR;
    volatile U32 *pDst     = NULL; ///< 首地址
    SgpioCtrl_s *sgpioCtrl = NULL;
    U32 i;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

#if 0
    if((NULL == pData) || ((regIdx + regCount) > sgpioGroupRegCntGet(sgpioId, regGroup))) {
        LOGE("sgpio group regs params is invalid\n");
        goto end;
    }
#endif

    ///< DRV_LOG_INFO("Read: sgpioId=%u, regGroup=%u, regIdx=%u, regCount=%u, pData=%u",
    ///<     sgpioId, regGroup, regIdx, regCount, *pData);

    switch(regGroup) {
    case SGPIO_SELECT_CONFIG_SGPIO:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_CFG_REG_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_RECEIVE:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_RECV_REG_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_GP_RECEIVE:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_GP_RECV_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_TRANSMIT:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_TRANS_REG_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_GP_TRANSMIT:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_GP_TRANS_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_VENDER_SPECIFIC:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_SPEC_REG_OFFSET(regIdx));
        break;

    case SGPIO_SELECT_BIT_ORDER_SEL:
        pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_BITORDER_SEL_OFFSET(regIdx));
        break;
    default:
        LOGE("sgpio reg group write is invalid\n");
        goto end;
    }
    for(i = 0; i < regCount; i++) {
        pData[i] = *pDst;
        ///< DRV_LOG_INFO("Read: sgpioCtrl->base=%u, i=%u, regCount=%u, pDst=%p, pData[%u]=%u",
        ///<     sgpioCtrl->base, i, regCount, pDst,i,pData[i]);
        pDst++;
    }
    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio 指定bitorder寄存器设置
 * @param   : sgpioId -sgpio control 索引
 * @param   : bitOrder -bitorder寄存器组的寄存器索引
 * @param   : value -配置值
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioBitOrderSelWrite(SgpioPortId_e sgpioId, U32 bitOrder, U32 value)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    sgpioRegBitOrderSet(sgpioCtrl, bitOrder, value);

    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio 读取指定bitorder寄存器的值
 * @param   : sgpioId -sgpio control 索引
 * @param   : bitOrder -bitorder寄存器组的寄存器索引
 * @param   : pData -bitorder值
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioBitOrderSelRead(SgpioPortId_e sgpioId, U32 bitOrder, U32 *pData)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;
    U32 regVal;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    regVal = sgpioRegBitOrderGet(sgpioCtrl, bitOrder);
    *pData = regVal;
    ret    = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio DriveStretchActOff寄存器设置
 *默认值中stretch_act_off为0，当进行active点灯的时候，
 *其对应的计数器会挂死。虽然默认配置中没有用到stretch_act_off，
 *但是还是要避免这个计数器处于一个异常状态
 * @param   : sgpioId -sgpio control 索引
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioDriveStretchActOffAll(SgpioPortId_e sgpioId)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;
    SgpioReg_u regVal;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    regVal.all                         = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(1));
    regVal.sgpioConfig1.activeOffStr = SGPIO_ENABLE;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(1), regVal.all);

    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio DriveStrove寄存器设置
 * @param   : sgpioId -sgpio control 索引
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioDriveStroveEnableAll(SgpioPortId_e sgpioId)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    REG32_WRITE(sgpioCtrl->base, SGPIO_DRIVE_STROBE(0), U32_MAX);
    REG32_WRITE(sgpioCtrl->base, SGPIO_DRIVE_STROBE(1), U32_MAX);

    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio 读取两个vender spec寄存器
 * @param   : sgpioId -sgpio control 索引
 * @param   : pVenderSpec0 -vender spec0数据缓存指针
 * @param   : pVenderSpec1 -vender spec1数据缓存指针
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioVenderSpecGet(SgpioPortId_e sgpioId, U32 *pVenderSpec0, U32 *pVenderSpec1)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }
    *pVenderSpec0 = sgpioRegVenderSpecGet(sgpioCtrl, 0);
    *pVenderSpec1 = sgpioRegVenderSpecGet(sgpioCtrl, 1);

    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio 复位状态机
 * @param   : sgpioId -sgpio control 索引
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioFsmReset(SgpioPortId_e sgpioId)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }
    sgpioRegFsmResetEnable(sgpioCtrl, 1);
    udelay(1000);
    sgpioRegFsmResetEnable(sgpioCtrl, 0);
    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio sload配置
 * @param   : sgpioId -sgpio control 索引
 * @param   : value -sload 值
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioSloadSet(SgpioPortId_e sgpioId, U32 value)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    sgpioRegSloadSet(sgpioCtrl, value);
    ret = SGPIO_RET_OK;

end:
    return ret;
}

/**
 * @brief   : sgpio sload获取
 * @param   : sgpioId -sgpio control 索引
 * @param   : value -sload 值获取指针
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioSloadGet(SgpioPortId_e sgpioId, U32 *value)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    *value = sgpioRegSloadGet(sgpioCtrl);
    ret    = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio driver count配置
 * @param   : sgpioId -sgpio control 索引
 * @param   : value -配置 值
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioDriverCountSet(SgpioPortId_e sgpioId, U32 value)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    sgpioRegDriverCountSet(sgpioCtrl, value);
    ret = SGPIO_RET_OK;

end:
    return ret;
}

/**
 * @brief   : sgpio driver count获取
 * @param   : sgpioId -sgpio control 索引
 * @param   : value -指针
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioDriverCountGet(SgpioPortId_e sgpioId, U32 *value)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    *value = sgpioRegDriverCountGet(sgpioCtrl);
    ret    = SGPIO_RET_OK;

end:
    return ret;
}

/**
 * @brief   : sgpio 功能使能
 * @param   : sgpioId -sgpio control 索引
 * @param   : enable -使能标志位
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioCfgEnable(SgpioPortId_e sgpioId, U8 enable)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    sgpioRegCfg0Enable(sgpioCtrl, enable);

    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sio blink 功能使能
 * @param   : sgpioId -sgpio control 索引
 * @param   : enable -使能标志位
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioSioBlinkEnable(SgpioPortId_e sgpioId, U8 enable)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is inval\n");
        goto end;
    }

    sgpioRegSioBlinkEnable(sgpioCtrl, enable);

    ret = SGPIO_RET_OK;
end:
    return ret;
}

/**
 * @brief   : sgpio 接收设备的数据
 * @param   : sgpioId -sgpio control 索引
 * @param   : pdata -接收数据指针
 * @param   : bufSize -数据缓存大小
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioRecvDeviceData(SgpioPortId_e sgpioId, U32 *pData, U32 bufSize)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;
    U32 regCnt             = 0;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    if((NULL == pData) || (bufSize % SGPIO_PHY_PER_REG)) {
        LOGE("sgpio recv device param is invalid\n");
        goto end;
    }

    regCnt = bufSize / SGPIO_PHY_PER_REG;
    ret    = sgpioGroupRegsRead(sgpioId, SGPIO_SELECT_RECEIVE, 0, regCnt, pData);
    if(ret < 0) {
        LOGE("sgpio group regs read failed\n");
        goto end;
    }
end:
    return ret;
}

/**
 * @brief   : sgpio GP 模式接收设备的数据
 * @param   : sgpioId -sgpio control 索引
 * @param   : pdata -接收数据指针
 * @param   : bufSize -数据缓存大小
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioGPRecvDeviceData(SgpioPortId_e sgpioId, U32 *pData, U32 bufSize)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;
    U32 regCnt             = 0;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    if((NULL == pData) || (bufSize % SGPIO_PHY_PER_REG)) {
        LOGE("sgpio recv device param is invalid\n");
        goto end;
    }

    if(bufSize > (U32)(sgpioRegPGCountGet(sgpioCtrl) * SGPIO_PHY_PER_REG)) {
        LOGE("sgpio GP recv device beyond limit\n");
        goto end;
    }
    regCnt = bufSize / SGPIO_PHY_PER_REG;
    ret    = sgpioGroupRegsRead(sgpioId, SGPIO_SELECT_GP_RECEIVE, 1, regCnt, pData);
    if(ret < 0) {
        LOGE("sgpio group regs read failed\n");
        goto end;
    }
end:
    return ret;
}

/**
 * @brief   : sgpio GP模式发送数据
 * @param   : sgpioId -sgpio control 索引
 * @param   : pdata -接收数据指针
 * @param   : devNum -指定phy
 * @param   : timers -重复的次数
 * @param   : sload -sload 模式
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioGPTransData(SgpioPortId_e sgpioId, U32 *pData, U32 devNum, U32 timers, U32 sload)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;
    U32 regCnt             = 0;

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    if((NULL == pData) || (devNum > sgpioPhyNumGet( ))) {
        LOGE("sgpio GP mode trans data params is invalid\n");
        goto end;
    }
    ///< 计算regCnt数
    regCnt = ((devNum * SGPIO_BITS_PER_PHY) + SGPIO_BITS_PER_REG - 1) / SGPIO_BITS_PER_REG;
    ret    = sgpioGroupRegsWrite(sgpioId, SGPIO_SELECT_GP_TRANSMIT, 1, regCnt, pData);
    if(ret < 0) {
        LOGE("sgpio group regs write failed\n");
        goto end;
    }
    ///< 设置传输次数和sload
    sgpioRegTransTimeSet(sgpioCtrl, timers);
    sgpioRegSloadSet(sgpioCtrl, sload);
end:
    return ret;
}

/**
 * @brief   : sgpio 根据用户初始化配置
 * @param   : sgpioCtrl -sgpio control
 * @param   : pConfig -用户配置指针
 * @return  : SGPIO_RET_OK 成功;
 */
static void sgpioCfgAndSpecInit(SgpioCtrl_s *sgpioCtrl, SgpioInitCfg_s *pConfig)
{
    S32 i;

    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(0), pConfig->sgpioCfg0);
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(1), pConfig->sgpioCfg1);
    REG32_WRITE(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(0), pConfig->venderSpecific0);
    REG32_WRITE(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(1), pConfig->venderSpecific1);
    for(i = 0; i < SGPIO_REG_GROUP_MAX; i++) {
        REG32_WRITE(sgpioCtrl->base, SGPIO_BITORDER_SEL_OFFSET(i), pConfig->bitOrderSelN[i]);
    }
}

/**
 * @brief   : 配置sio时钟
 * @param   : sgpioId -指定sgpio 控制器
 * @param   : pConfig -用户配置指针
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioSioClkPeriodSet(U32 sgpioId, U32 clkHz)
{
    U32 clkPeriod          = 0;
    S32 ret                = SGPIO_RET_OK;
    volatile U32 *pDst     = NULL; ///< 首地址
    SgpioCtrl_s *sgpioCtrl = NULL;

    if(clkHz<SGPIO_CLK_HZ_MIN || clkHz>SGPIO_CLK_HZ_MAX){
        LOGE("sgpio sio cld period set error, clkHz:%u\n",clkHz);
        ret = -SGPIO_RET_ERR;
        goto end;
    }

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    pDst = (volatile U32 *)(sgpioCtrl->base + SGPIO_SIO_OFFSET(2));
    clkPeriod = 1000000000 / clkHz;
    *pDst     = clkPeriod;

end:
    return ret;
}

/**
 * @brief   : 配置PinMux
 * @param   : sgpioId -指定sgpio 控制器
 * @param   : pConfig -用户配置指针
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioPinMuxSet(U32 sgpioId)
{
    S32 ret = SGPIO_RET_OK;

    if(sgpioId >= SGPIO_PORT_MAX) {
        ret = -SGPIO_RET_ERR;
        LOGE("sgpio Id %d is invalid\n", sgpioId);
        goto end;
    }

    ///< sgpio-1需要配置PinMux
    if(sgpioId == SGPIO_PORT_1){
        sgpioConfigPinMux();
        ///< DRV_LOG_INFO("Hal Sgpio-%u Config PinMux Success!",SGPIO_PORT_1);
    }

end:
    return ret;
}

/**
 * @brief   : sgpio 根据用户初始化配置
 * @param   : sgpioId -指定sgpio 控制器
 * @param   : pConfig -用户配置指针
 * @return  : SGPIO_RET_OK 成功;
 */
S32 sgpioDevInit(SgpioPortId_e sgpioId, SgpioInitCfg_s *pConfig)
{
    S32 ret                = -SGPIO_RET_ERR;
    SgpioCtrl_s *sgpioCtrl = NULL;

    if((sgpioId >= SGPIO_PORT_MAX) || (NULL == pConfig)) {
        LOGE("sgpio init params is invalid\n");
        goto end;
    }

    sgpioCtrl = sgpioCtrlGet(sgpioId);
    if(NULL == sgpioCtrl) {
        goto end;
    }

    ///< 配置基地址
    if(sgpioId == SGPIO_PORT_0) {
        sgpioCtrl->base = SGPIO_PORT0_BASEADDR;
    } else if(sgpioId == SGPIO_PORT_1) {
        sgpioCtrl->base = SGPIO_PORT1_BASEADDR;
    }

#ifdef PS3_PRODUCT_EXPANDER
    ///< 时钟配置
    sgpioSysClkEnable(sgpioId);
#endif

    ///< 复位设置
    sgpioSysReset(sgpioId);
#ifdef PS3_PRODUCT_SWITCH
    sgpio8485PinMux();
#endif

    if(sgpioId == SGPIO_PORT_1) {
        ///< sgpio1 需要设置复用功能
        sgpioConfigPinMux( );
    }
    ///< 使能设置
    (void)sgpioCfgEnable(sgpioId, SGPIO_ENABLE);

    ///< 配置初始化，内部函数
    sgpioCfgAndSpecInit(sgpioCtrl, pConfig);
    ///< 初始化完成
    ret = -SGPIO_RET_ERR;
end:
    return ret;
}
#ifdef PS3_MODEL_V300
/**
 * @brief   : sgpio blink out 是否使能
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 */
S32 sgpioRegSioBlinkOutOne(U8 enable)
{
    SgpioReg_u regVal;
    SgpioCtrl_s *sgpioCtrl = NULL;
    S32 ret = SGPIO_RET_OK;

    sgpioCtrl = sgpioCtrlGet(SGPIO_PORT_0);
    if(NULL == sgpioCtrl) {
        ret = -SGPIO_RET_ERR;
        LOGE("sgpio Blink HB params is invalid\n");
        goto end;
    }
    regVal.all                           = REG32_READ(sgpioCtrl->base, SGPIO_SIO_OFFSET(SGPIO_VALUE_0));
    regVal.sgpioSioBlinkSel.sioBlinkOutOne = enable;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SIO_OFFSET(SGPIO_VALUE_0), regVal.all);

end:
    return ret;
}

/**
 * @brief   : sgpio 选择从哪个sio输出blink信号
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 */
static void sgpioRegSioBlinkOutSel(SgpioCtrl_s *sgpioCtrl, U8 enable)
{
    SgpioReg_u regVal;

    regVal.all                           = REG32_READ(sgpioCtrl->base, SGPIO_SIO_OFFSET(SGPIO_VALUE_0));
    regVal.sgpioSioBlinkSel.sioBlinkOutSel = enable;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SIO_OFFSET(SGPIO_VALUE_0), regVal.all);
}

/**
 * @brief   : sgpio 控制信号周期
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 * @note    : TODO，补充计算公式
 */
static void sgpioRegSioBlinkFrequencySet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;

    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_SIO_OFFSET(SGPIO_VALUE_1));
    regVal.all = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SIO_OFFSET(SGPIO_VALUE_1), regVal.all);
}

 /**
 * @brief   : 设置sgpio的版本
 * @param   : sgpioCtrl -全局结构体
 * @return  : version;
 */
static void sgpioRegVersionSet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0));

    regVal.sgpioConfig0.version = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0), regVal.all);
    return ;
}

/**
 * @brief   : 设置寄存器个数
 * @param   : sgpioCtrl -全局结构体
 * @return  : version;
 */
static void sgpioRegCfgCountSet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0));

    regVal.sgpioConfig0.cfgRegCnt = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0), regVal.all);
    return ;
}

/**
 * @brief   : 设置sgpio通用目的数据寄存器个数
 * @param   : sgpioCtrl -全局结构体
 * @return  : version;
 */
static void sgpioRegPGCountSet(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0));

    regVal.sgpioConfig0.GPRegCnt = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0), regVal.all);
    return ;
}

/**
 * @brief   : 是否使能gpio
 * @param   : sgpioCtrl -全局结构体
 * @return  : version;
 */
static void sgpioRegGpioEn(SgpioCtrl_s *sgpioCtrl, U32 value)
{
    SgpioReg_u regVal;
    regVal.all = REG32_READ(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0));

    regVal.sgpioConfig0.gpioEn = value;
    REG32_WRITE(sgpioCtrl->base, SGPIO_CFG_REG_OFFSET(SGPIO_VALUE_0), regVal.all);
    return ;
}

/**
 * @brief   : sgpio状态机控制
 * @param   : sgpioCtrl -全局结构体
 * @param   : enable -使能标志位
 * @return  : void;
 */
static void sgpioRegFsmReset(SgpioCtrl_s *sgpioCtrl, U8 enable)
{
    SgpioReg_u regVal;
    regVal.all                  = REG32_READ(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(SGPIO_VALUE_1));
    regVal.sgpioVspec0.resetFsm = enable;
    REG32_WRITE(sgpioCtrl->base, SGPIO_SPEC_REG_OFFSET(SGPIO_VALUE_1), regVal.all);
}

/**
 * @brief   : sgpio sio blink 做心跳灯
 * @param   : period 周期（单位：s）
 * @return  : SGPIO_RET_OK 成功;
 * @return  : -SGPIO_RET_ERR 成功;
*/
S32 sgpioBlinkHB(U32 period)
{
    S32 ret                = SGPIO_RET_OK;
    SgpioCtrl_s *sgpioCtrl = NULL;
    ///< 周期=寄存器配置值*0.25*1/100000000
    U32 value = period * SGPIO_BLINK_HB_PCLK_VALUE;

    sgpioCtrl = sgpioCtrlGet(SGPIO_VALUE_0);
    if(NULL == sgpioCtrl) {
        ret = -SGPIO_RET_ERR;
        LOGE("sgpio Blink HB params is invalid\n");
        goto end;
    }
    ///< 0.sgpio0_vender_specific0（0x58202504）bit[31]配置为0。sgpio0_sio_blink_sel（0x58202800）bit[3]配置为1
    sgpioRegFsmReset(sgpioCtrl, SGPIO_VALUE_0);
    (void)sgpioRegSioBlinkOutOne(SGPIO_VALUE_1);
    ///< 1.先通过sgpio0_sio_blink_reg（0x58202804）用来配置闪烁频率；基于100MHZ进行分频。
    sgpioRegSioBlinkFrequencySet(sgpioCtrl, value);

    ///< 2.再通过sgpio0_sio_blink_sel（0x58202800） bit[0]0用来选择是sgpio0。
    sgpioRegSioBlinkOutSel(sgpioCtrl, SGPIO_VALUE_0);

    ///< 3.配置sgpio0_sgpio_cfg（0x58202000）bit[11:8]为0x0；bit[22:20]为0x2；bit[23]=0x1；bit[19:16]=0x1；
    sgpioRegVersionSet(sgpioCtrl, SGPIO_VALUE_0);
    sgpioRegCfgCountSet(sgpioCtrl, SGPIO_VALUE_2);
    sgpioRegGpioEn(sgpioCtrl, SGPIO_VALUE_1);
    sgpioRegPGCountSet(sgpioCtrl, SGPIO_VALUE_1);
    ///< 配置sgpio0_sgpio_cfg（0x58202000）bit[30:24]=0x4。
    ///< 4.配置sgpio0_vendor_specific1（0x58202504）bit[6:0]=0x4；
    sgpioRegDriverCountSet(sgpioCtrl, SGPIO_VALUE_0X30);

    ///< 5.sgpio0_sio_blink_sel（0x58202800）bit[3]配置为0。灯就可以闪烁。
    (void)sgpioRegSioBlinkOutOne(SGPIO_VALUE_0);

end:
    return ret;
}
#endif
