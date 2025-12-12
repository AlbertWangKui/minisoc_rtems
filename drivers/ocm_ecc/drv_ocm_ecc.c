/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file    drv_ocm_ecc.c
 * @author  zhangxin3@starsmicrosystem.com
 * @date    2025/09/22
 * @brief   OCM ECC driver implementation
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <bsp/arm-gic-irq.h>
#include <bsp/irq-generic.h>
#include "drv_ocm_ecc.h"

/**
 * @brief OCM ECC interrupt service routine
 * @details This function handles SEC and DED error interrupts, reads error information,
 *          and calls the registered callback function if available
 * @param [in] arg Pointer to driver private data
 */
static void ocmEccIsr(void *arg)
{
    ocmEccDrvPrivateData_s *drv = (ocmEccDrvPrivateData_s *)arg;
    U32 errAddr = 0, dataL = 0, dataH = 0;
    U64 errData = 0;
    U32 irqStatus = 0;

    if ((drv == NULL) || (drv->eccReg == NULL)) {
        return;
    }

    irqStatus = drv->eccReg->intSt.dword;

    LOGD("%s-%d: ocm ecc irq status:0x%08x\r\n", __func__, __LINE__, irqStatus);
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

/**
 * @brief Get OCM ECC device configuration from SBR
 * @details This function reads the OCM ECC device configuration from SBR
 * @param [in] devId Device ID
 * @param [inout] pEccDevCfg Pointer to store the device configuration
 * @return EXIT_SUCCESS on success
 * @return -EIO I/O error (failed to read from SBR or invalid configuration)
 * @return -EINVAL Invalid parameter (invalid configuration values)
 */
static S32 ocmEccDevCfgGet(DevList_e devId, SbrOcmEccCfg_s *pEccDevCfg)
{
    S32 ret = EXIT_SUCCESS;
    U32 readSize;

    if (pEccDevCfg == NULL) {
        ret = -EIO;
        goto out;
    }

    ///< Read configuration from SBR
    readSize = devSbrRead(devId, pEccDevCfg, 0, sizeof(SbrOcmEccCfg_s));
    if (readSize != sizeof(SbrOcmEccCfg_s)) {
        LOGE("%s-%d: failed to read OCM ECC config from SBR, readSize=%u, expected=%u\r\n",
             __func__, __LINE__, readSize, sizeof(SbrOcmEccCfg_s));
        ret = -EIO;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("%s-%d: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, reserved:0x%08x\r\n",
         __func__, __LINE__, pEccDevCfg->eccAddr, pEccDevCfg->irqNo, pEccDevCfg->irqPrio, pEccDevCfg->reserved);
#endif

    if (pEccDevCfg->irqNo == 0 || pEccDevCfg->irqPrio == 0 || pEccDevCfg->eccAddr == 0) {
        ret = -EINVAL;
        goto out;
    }

out:
    return ret;
}

S32 ocmEccInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (isDrvInit(devId) == true) {
        ret = -EINVAL;
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

    ///< Allocate driver private data memory
    pEccDrvData = (ocmEccDrvPrivateData_s*)calloc(1, sizeof(ocmEccDrvPrivateData_s));
    if (pEccDrvData == NULL) {
        ret = -ENOMEM;
        goto unlock;
    }

    if (ocmEccDevCfgGet(devId, &pEccDrvData->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    pEccDrvData->eccReg = (ocmEccReg_s *)pEccDrvData->sbrCfg.eccAddr;
    pEccDrvData->callback = NULL; ///< Initialize interrupt callback to NULL
    ospInterruptHandlerInstall(pEccDrvData->sbrCfg.irqNo, "ocm ecc",
        OSP_INTERRUPT_UNIQUE, ocmEccIsr, pEccDrvData);

    if (arm_gic_irq_set_priority(pEccDrvData->sbrCfg.irqNo, pEccDrvData->sbrCfg.irqPrio) != EXIT_SUCCESS) {
        ret = -EIO;
        goto remove_isr;
    }

    bsp_interrupt_vector_enable(pEccDrvData->sbrCfg.irqNo);

    ///< Install device driver
    if (drvInstall(devId, (void*)pEccDrvData) != EXIT_SUCCESS) {
        ret = -EIO;
        goto disable_vector;
    }

    pEccDrvData->eccReg->intEn.fields.secIrqEnable = 1; ///< Enable interrupt
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

S32 ocmEccDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_OCM_ECC)) {
        ret = -EINVAL;
        goto out;
    }

    if (isDrvInit(devId) == false) {
        ret = -EINVAL;
        goto out;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
        goto out;
    }

    pEccDrvData->eccReg->intEn.dword = 0;
    ospInterruptHandlerRemove(pEccDrvData->sbrCfg.irqNo, ocmEccIsr, pEccDrvData);
    ospInterruptVectorUninit(pEccDrvData->sbrCfg.irqNo);

    if (drvUninstall(devId) != EXIT_SUCCESS) {
        LOGE("%s-%d: failed to uninstall driver for devId %u\r\n", __func__, __LINE__, devId);
        ret = -EIO;
    }

    funcRunEndHelper(devId);
out:
    return ret;
}

S32 ocmEccIrqCbRegister(DevList_e devId, pOcmEccCallback func)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;
    U32 intEn;

    if (func == NULL) {
        ret = -EINVAL;
        goto out;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
        goto out;
    }

    intEn = pEccDrvData->eccReg->intEn.dword;
    pEccDrvData->eccReg->intEn.dword = 0x0;
    pEccDrvData->callback = func;
    pEccDrvData->eccReg->intEn.dword = intEn;

    funcRunEndHelper(devId);
out:
    return ret;
}

S32 ocmEccIrqCbUnregister(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
        goto out;
    }

    pEccDrvData->callback = NULL;

    funcRunEndHelper(devId);
out:
    return ret;
}

S32 ocmEccErrCntGet(DevList_e devId, ocmEccErrType_e eType)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if ((eType < 0) || (eType >= OCM_ECC_ETYPE_MAX)) {
        ret = -EINVAL;
        goto out;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
        goto out;
    }

    if (eType == OCM_ECC_SEC) {
        ret = pEccDrvData->eccReg->eccCnt.fields.secCnt;
    } else {
        ret = pEccDrvData->eccReg->eccCnt.fields.dedCnt;
    }

    funcRunEndHelper(devId);
out:
    return ret;
}

S32 ocmEccErrCntClear(DevList_e devId, ocmEccErrType_e eType)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if ((eType < 0) || (eType >= OCM_ECC_ETYPE_MAX)) {
        ret = -EINVAL;
        goto out;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
        goto out;
    }

    if (eType == OCM_ECC_SEC) {
        pEccDrvData->eccReg->intClr.fields.secCntClr = 1;
    } else {
        pEccDrvData->eccReg->intClr.fields.dedCntClr = 1;
    }
    ret = EXIT_SUCCESS;

    funcRunEndHelper(devId);
out:
    return ret;
}

S32 ocmEccLastErrGet(DevList_e devId, ocmEccErrType_e eType, U32 *pErrAddr, U64 *pErrData)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if ((pErrAddr == NULL) || (pErrData == NULL) || (eType < 0) || (eType >= OCM_ECC_ETYPE_MAX)) {
        ret = -EINVAL;
        goto out;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
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

    funcRunEndHelper(devId);
out:
    return ret;
}

S32 ocmEccErrIrqTrigger(DevList_e devId, ocmEccErrType_e eType)
{
    S32 ret = EXIT_SUCCESS;
    ocmEccDrvPrivateData_s *pEccDrvData = NULL;

    if ((eType < 0) || (eType >= OCM_ECC_ETYPE_MAX)) {
        ret = -EINVAL;
        goto out;
    }

    ret = funcRunBeginHelper(devId, DRV_ID_STARS_OCM_ECC, (void **)&pEccDrvData);
    if (ret != EXIT_SUCCESS) {
        goto out;
    }

    if ((pEccDrvData == NULL) || (pEccDrvData->eccReg == NULL)) {
        ret = -EIO;
        funcRunEndHelper(devId);
        goto out;
    }

    if (eType == OCM_ECC_SEC) {
        pEccDrvData->eccReg->softEccErr.fields.softSecErr = 1;
    } else {
        pEccDrvData->eccReg->softEccErr.fields.softDedErr = 1;
    }
    ret = EXIT_SUCCESS;

    funcRunEndHelper(devId);
out:
    return ret;
}
