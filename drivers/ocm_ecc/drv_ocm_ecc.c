/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_ocm_ecc.c
 * @author zhangxin3@starsmicrosystem.com
 * @date 2025/09/22
 * @brief ecc driver
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <bsp/arm-gic-irq.h>
#include <bsp/irq-generic.h>
#include "drv_ocm_ecc.h"

static void ocmEccIsr(void *arg)
{
    ocmEccDrvPrivateData_s *drv = (ocmEccDrvPrivateData_s *)arg;
    U32 errAddr = 0, dataL = 0, dataH = 0;
    U64 errData = 0;
    U32 irqStatus = 0;

    irqStatus = drv->eccReg->intSt.dword;

    LOGD("ocm ecc irq status:0x%08x\r\n", irqStatus);
    if (irqStatus & ECC_SEC_ALL_IRQ_MASK) {
        if (irqStatus & ECC_SEC1_IRQ_MASK) {
            errAddr = drv->eccReg->sec1Addr.dword;
            dataL = drv->eccReg->sec1Dal.dword;
            dataH = drv->eccReg->sec1Dah.dword;
        } else if (irqStatus & ECC_SEC2_IRQ_MASK) {
            errAddr = drv->eccReg->sec2Addr.dword;
            dataL = drv->eccReg->sec2Dal.dword;
            dataH = drv->eccReg->sec2Dah.dword;
        } else if (irqStatus & ECC_SEC3_IRQ_MASK) {
            errAddr = drv->eccReg->sec3Addr.dword;
            dataL = drv->eccReg->sec3Dal.dword;
            dataH = drv->eccReg->sec3Dah.dword;
        } else if (irqStatus & ECC_SEC4_IRQ_MASK) {
            errAddr = drv->eccReg->sec4Addr.dword;
            dataL = drv->eccReg->sec4Dal.dword;
            dataH = drv->eccReg->sec4Dah.dword;
        }

        errData = (U64)dataL | (((U64)dataH) << 32);
        drv->lastSecAddr = errAddr;
        drv->lastSecData = errData;

        if (drv->callback != NULL) {
            drv->callback(OCM_ECC_SEC, errAddr, errData);
        }
        drv->eccReg->intClr.fields.secClr = 1;
    }

    if (irqStatus & ECC_DED_ALL_IRQ_MASK) {
        if (irqStatus & ECC_DED1_IRQ_MASK) {
            errAddr = drv->eccReg->ded1Addr.dword;
            dataL = drv->eccReg->ded1Dal.dword;
            dataH = drv->eccReg->ded1Dah.dword;
        } else if (irqStatus & ECC_DED2_IRQ_MASK) {
            errAddr = drv->eccReg->ded2Addr.dword;
            dataL = drv->eccReg->ded2Dal.dword;
            dataH = drv->eccReg->ded2Dah.dword;
        } else if (irqStatus & ECC_DED3_IRQ_MASK) {
            errAddr = drv->eccReg->ded3Addr.dword;
            dataL = drv->eccReg->ded3Dal.dword;
            dataH = drv->eccReg->ded3Dah.dword;
        } else if (irqStatus & ECC_DED4_IRQ_MASK) {
            errAddr = drv->eccReg->ded4Addr.dword;
            dataL = drv->eccReg->ded4Dal.dword;
            dataH = drv->eccReg->ded4Dah.dword;
        }

        errData = (U64)dataL | (((U64)dataH) << 32);
        drv->lastDedAddr = errAddr;
        drv->lastDedData = errData;

        if (drv->callback != NULL) {
            drv->callback(OCM_ECC_DED, errAddr, errData);
        }
        drv->eccReg->intClr.fields.dedClr = 1;
    }
}

static S32 ocmEccDevCfgGet(DevList_e devId, SbrOcmEccCfg_s *pEccDevCfg)
{
    S32 ret = EXIT_SUCCESS;
    U32 readSize;

    if (pEccDevCfg == NULL) {
        ret = -EIO;
        goto out;
    }

    /* 从SBR读取配置 */
    readSize = devSbrRead(devId, pEccDevCfg, 0, sizeof(SbrOcmEccCfg_s));
    if (readSize != sizeof(SbrOcmEccCfg_s)) {
        LOGE("ocm_ecc: failed to read OCM ECC config from SBR, readSize=%u, expected=%u\r\n",
             readSize, sizeof(SbrOcmEccCfg_s));
        ret = -EIO;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGE("ocm ecc: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, reserved:0x%08x\r\n",
         pEccDevCfg->eccAddr, pEccDevCfg->irqNo, pEccDevCfg->irqPrio, pEccDevCfg->reserved);
#endif

    if (pEccDevCfg->eccAddr == 0 || pEccDevCfg->irqNo == 0) {
        ret = -EIO;
        goto out;
    }

out:
    return ret;
}

/**
 * @brief 初始化
 * @param [in] devId 设备ID
 * @return 0表示成功，<0表示错误
 */
S32 ocmEccInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (isDrvInit(devId) == true) {
        ret = -EBUSY;
        goto out;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    ///< 申请驱动私有数据内存并获取设备配置
    pEccDrvData = (ocmEccDrvPrivateData_s*)calloc(1, sizeof(ocmEccDrvPrivateData_s));
    if (pEccDrvData == NULL) {
        ret = -ENOMEM;
        goto unlock;
    }

    if (ocmEccDevCfgGet(devId, &pEccDrvData->sbrCfg) != EXIT_SUCCESS) { ///< 获取设备配置
        ret = -EIO;
        goto freeMem;
    }

    pEccDrvData->eccReg = (ocmEccReg_s *)pEccDrvData->sbrCfg.eccAddr;
    pEccDrvData->callback = NULL; ///< 初始化中断回调函数为NULL
    ospInterruptHandlerInstall(pEccDrvData->sbrCfg.irqNo, "ocm ecc",
        OSP_INTERRUPT_UNIQUE, ocmEccIsr, pEccDrvData);

    if (arm_gic_irq_set_priority(pEccDrvData->sbrCfg.irqNo, pEccDrvData->sbrCfg.irqPrio) != EXIT_SUCCESS) {
        ret = -EIO;
        goto remove_isr;
    }

    bsp_interrupt_vector_enable(pEccDrvData->sbrCfg.irqNo);

    ///< 安装设备驱动
    if (drvInstall(devId, (void*)pEccDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto disable_vector;
    }

    pEccDrvData->eccReg->intEn.fields.secIrqEnable = 1; ///< 中断使能
    pEccDrvData->eccReg->intEn.fields.dedIrqEnable = 1;

    ret = EXIT_SUCCESS;
    goto unlock;

disable_vector:
    bsp_interrupt_vector_disable(pEccDrvData->sbrCfg.irqNo);
remove_isr:
    ospInterruptHandlerRemove(pEccDrvData->sbrCfg.irqNo, ocmEccIsr, pEccDrvData);
freeMem:
    free(pEccDrvData);
unlock:
    devUnlockByDriver(devId);
out:
    return ret;
}

/**
 * @brief 去初始化
 * @param [in] devId 设备ID
 * @return 0表示成功，<0表示错误
 */
S32 ocmEccDeinit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EBUSY;
        goto out;
    }

    if (getDevDriver(devId,(void **)&pEccDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pEccDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    ospInterruptHandlerRemove(pEccDrvData->sbrCfg.irqNo, ocmEccIsr, pEccDrvData);
    ospInterruptVectorUninit(pEccDrvData->sbrCfg.irqNo);

    drvUninstall(devId); ///< 卸载设备

    devUnlockByDriver(devId);
out:
    return ret;
}

/**
 * @brief 注册中断回调
 * @param [in] devId 设备ID
 * @param [in] func中断回调
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccIrqCbRegister(DevList_e devId, pOcmEccCallback func)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (NULL == func) {
        ret = -EIO;
        goto out;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto out;
    }

    ret = getDevDriver(devId,(void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pEccDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    pEccDrvData->callback = func;

    devUnlockByDriver(devId);
out:
    return ret;
}

/**
 * @brief 注销中断回调
 * @param [in] devId 设备ID
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccIrqCbDeregister(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto out;
    }

    ret = getDevDriver(devId,(void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pEccDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    pEccDrvData->callback = NULL;

    devUnlockByDriver(devId);
out:
    return ret;
}

/**
 * @brief 获取错误计数
 * @param [in] devId 设备ID
 * @param [in] eType表示单bit/多bit
 * @return >= 0表示错误计数个数，<0表示参数错误
 */
S32 ocmEccErrCntGet(DevList_e devId, ocmEccErrType_e eType)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (eType >= OCM_ECC_ETYPE_MAX) {
        ret = -EINVAL;
        goto out;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto out;
    }

    ret = getDevDriver(devId,(void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pEccDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    if (eType == OCM_ECC_SEC) {
        ret = pEccDrvData->eccReg->eccCnt.fields.secCnt;
    } else {
        ret = pEccDrvData->eccReg->eccCnt.fields.dedCnt;
    }
    devUnlockByDriver(devId);

out:
    return ret;
}

/**
 * @brief 清除错误计数
 * @param [in] devId 设备ID
 * @param [in] eType表示单bit/多bit
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccErrCntClear(DevList_e devId, ocmEccErrType_e eType)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (eType >= OCM_ECC_ETYPE_MAX) {
        ret = -EINVAL;
        goto out;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto out;
    }

    ret = getDevDriver(devId,(void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pEccDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    if (eType == OCM_ECC_SEC) {
        pEccDrvData->eccReg->intClr.fields.secCntClr = 1;
    } else {
        pEccDrvData->eccReg->intClr.fields.dedCntClr = 1;
    }
    ret = EXIT_SUCCESS;

    devUnlockByDriver(devId);
out:
    return ret;
}

/**
 * @brief 获取最近的错误数据和地址
 * @param [in] devId 设备ID
 * @param [in] eType表示单bit/多bit
 * @param [out] pErrAddr返回错误数据的地址
 * @param [out] pErrData返回错误数据
 * @return 0表示成功，<0表示参数错误
 */
S32 ocmEccLastErrGet(DevList_e devId, ocmEccErrType_e eType, U32 *pErrAddr, U64 *pErrData)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if ((pErrAddr == NULL) || (pErrData == NULL) || (eType >= OCM_ECC_ETYPE_MAX)) {
        ret = -EINVAL;
        goto out;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EIO;
        goto out;
    }

    ret = getDevDriver(devId,(void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        ret = -EIO;
        goto out;
    }

    if (pEccDrvData == NULL) {
        ret = -EIO;
        goto out;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto out;
    }

    if (eType == OCM_ECC_SEC) {
        *pErrAddr = pEccDrvData->lastSecAddr;
        *pErrData = pEccDrvData->lastSecData;
    } else {
        *pErrAddr = pEccDrvData->lastDedAddr;
        *pErrData = pEccDrvData->lastDedData;
    }
    ret = EXIT_SUCCESS;

    devUnlockByDriver(devId);
out:
    return ret;
}
