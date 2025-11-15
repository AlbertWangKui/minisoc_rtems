/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file qspi_ctrl.c
 * @author yuxy3@starsmicrosystem.com
 * @date 2025/06/04
 * @brief  qspictrl interface & registers
 * @note
 */

#include "common_defines.h"
#include "drv_qspi_ctrl.h"
#include "log_msg.h"
#include "osp_common.h"
#include "bsp_api.h"
#include "udelay.h"

static Qspictrl_s qspictrl_obj;
static QspictrlSlaveMgr_s slaveMgr;
static QspictrlSlaveHcfg_s qspictrl_slave_param_table[] = SUPPORT_QSPICTRL_SLAVE_TABLE;

static void qspictrlPinmux(void)
{
    volatile U32 *pinMuxReg = (volatile U32 *)SYS_FUNCMUX_BASE_ADRS;
    /* GPIO12~GPIO19 */
    *pinMuxReg |= (0xff << 12);
}

static void qspictrlReset(void)
{
    uint32_t *qspictrlRst = (uint32_t *)(SYS_CRG_BASE_ADRS + 0x4c);

    *qspictrlRst &= ~(1 << 0);
    udelay(1);
    *qspictrlRst |= (1 << 0);

    qspictrlPinmux();
}

static U32 qspictrlClockGet(U32 *clk)
{
    if (EXIT_SUCCESS == peripsClockFreqGet(DEVICE_QSPICTRL, clk)){
        return EXIT_SUCCESS;
    }

    LOGD("QSPICTRL get clock failed.");
    return -EXIT_FAILURE;
}

static Qspictrl_s *qspictrlObjGet(void)
{
    return &qspictrl_obj;
}

static QspictrlSlaveMgr_s *qspictrlSlaveMgrGet(void)
{
    return &slaveMgr;
}

static void qspictrlIntmodeEnable(QspictrlRegs_vs *regs)
{
    regs->int_ctrl.fields.int_mode = 1;
}

static void qspictrlIntmodeDisable(QspictrlRegs_vs *regs)
{
    regs->int_ctrl.fields.int_mode = 0;
}

static bool qspictrlChecking(Qspictrl_s *t)
{
    bool ret = true;

    if ((!t) || (!t->initOk)) {
        ret = false;
    }

    return ret;
}

static U32 qspictrlLock(Qspictrl_s *t)
{
    if (!t) {
        return -EXIT_FAILURE;
    }

    if (ospInterruptIsInProgress()) {
        return -EXIT_FAILURE;
    }

    ospRecursiveMutexLock(&t->slaveMutex);

    return EXIT_SUCCESS;
}

static U32 qspictrlUnlock(Qspictrl_s *t)
{
    if (!t) {
        return -EXIT_FAILURE;
    }
    if (ospInterruptIsInProgress()) {
        return -EXIT_FAILURE;
    }
    ospRecursiveMutexUnlock(&t->slaveMutex);

    return EXIT_SUCCESS;
}

static void qspictrlSlaveAddrSet(QspictrlRegs_vs *regs, uint16_t slave_addr)
{
    regs->addr.fields.addr = slave_addr;
}

static void qspictrlDummyCntSet(QspictrlRegs_vs *regs, U32 dummyCnt)
{
    regs->dummy_cnt.fields.dummy_cnt = dummyCnt & 0xff;
}

static void qspictrlCmdSet(QspictrlRegs_vs *regs, bool is_write, uint16_t len)
{
    assert(len <= QSPICTRL_MAX_SEGMENT_LEN);
    regs->cmd.fields.wrdata_len = len;
    regs->cmd.fields.wrcmd = (is_write) ? 0 : 1;
}

static U32 qspictrlTxFifoLevelGet(QspictrlRegs_vs *regs)
{
    return regs->fifo_status.fields.tx_fifo_level;
}

static void qspictrlXferStart(QspictrlRegs_vs *regs)
{
    regs->start.fields.start = 1;
}

static void qspictrlUpdateWr(QspictrlRegs_vs *regs)
{
    regs->update.fields.update = 1;
}

static S32 qspictrlWriteDataSet(QspictrlRegs_vs *regs, U8 *buf, U32 len)
{
    S32 ret = EXIT_SUCCESS;
    U32 i;
    U8 rwdata[32] = { 0 };
    U32 *rwdword = (U32 *)rwdata;
    U32 dwordCnt = ALIGN_UP(len, 4) / 4;

    if (len > QSPICTRL_MAX_SEGMENT_LEN) {
        ret = -EXIT_FAILURE;
        LOGE("%s len %d invalid fail\r\n", __func__, len);
        goto exit;
    }

    for (i = 0; i < len; i++) {
        rwdata[i] = buf[i];
    }

    if (dwordCnt > QSPICTRL_MAX_REG_DATA_SEGNUM) {
        ret = -EXIT_FAILURE;
        LOGE("%s dwordCnt %d invalid \r\n", __func__, dwordCnt);
        goto exit;
    }

    /* write's unit must be 4B */
    for (i = 0; i < dwordCnt; i++) {
        regs->wdata[i].dword = rwdword[i];
    }

exit:
    return ret;
}

static U32 qspictrlClkSet(QspictrlRegs_vs *regs, U32 clk)
{
    U32 qspictrlClk;
    U32 div;

    if (EXIT_SUCCESS != qspictrlClockGet(&qspictrlClk)) {
        return -EXIT_FAILURE;
    }

    if (!clk) {
        return -EXIT_FAILURE;
    }

    div = qspictrlClk / clk;
    regs->div.fields.div = div << 1;

    return EXIT_SUCCESS;
}

static U32 qspictrlCsSet(QspictrlRegs_vs *regs, QspictrlCs_u cs)
{
    if (cs >= QSPICTRL_CS_MAX) {
        return -EXIT_FAILURE;
    }

    regs->slave_sel.fields.slave_sel = cs;

    return EXIT_SUCCESS;
}

static S32 qspictrlHardwareConfig(Qspictrl_s *t, QspictrlSlave_s *dev)
{
    S32 ret = EXIT_SUCCESS;

    if (!qspictrlChecking(t)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = qspictrlLock(t);
    if (ret != EXIT_SUCCESS) {
        LOGD("add qspictrl lock failed!");
        goto exit;
    }

    /* Step 1: init clk */
    ret = qspictrlClkSet(t->regs, dev->hcfg.slaveCfg.clk);
    if (ret != EXIT_SUCCESS) {
        LOGD("qspictrl clk set failed!");
        goto exit_unlock;
    }

    /* Step 2: set cs */
    ret = qspictrlCsSet(t->regs, dev->hcfg.slaveCfg.cs);
       if (ret != EXIT_SUCCESS) {
        LOGD("qspictrl cs set failed!");
        goto exit_unlock;
    }

exit_unlock:
    ret = qspictrlUnlock(t);
    if (ret != EXIT_SUCCESS) {
        LOGD("release qspictrl unlock failed!");
    }
exit:
    return ret;
}

#ifdef QSPICTRL_INT_ENABLE
static void qspictrl_isr(void *arg)
{
    Qspictrl_s *t = (Qspictrl_s *)arg;
    QspictrlRegs_vs *regs = t->regs;
    QspictrlIntRawReg_u intStatus;
    OspStatusCode_e status;

    intStatus.dword = regs->int_raw_status.dword;

    if (intStatus.fields.rx_valid || intStatus.fields.tx_fifo_empty) {
        status = ospSemaphoreRelease(t->semIdInterrupt);
        if (status != OSP_SUCCESSFUL) {
            LOGD("osp_semaphore_release failed %d\n", status);
        }
    }
    /* disable interrupt */
    qspictrlIntmodeDisable(regs);
}

static S32 qspictrlInterruptInit(Qspictrl_s *t)
{
    S32 ret = EXIT_SUCCESS;
    OspStatusCode_e ospStatus;
    QspictrlRegs_vs *regs = t->regs;

    ospStatus = ospInterruptHandlerInstall(t->hcfg.interruptNum, "Qspictrl_s", OSP_INTERRUPT_UNIQUE,
                                           qspictrl_isr, t);
    if (ospStatus != OSP_SUCCESSFUL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
    ospStatus = ospInterruptSetPriority(t->hcfg.interruptNum, t->hcfg.interruptPriority);
    if (ospStatus != OSP_SUCCESSFUL) {
        ospInterruptHandlerRemove(t->hcfg.interruptNum, qspictrl_isr, t);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    ospInterruptVectorEnable(t->hcfg.interruptNum);
    /* config interrupt register and enable */
    regs->int_ctrl.fields.rx_valid_mask = 1;
    regs->int_ctrl.fields.txfifo_empty_mask = 1;
    qspictrlIntmodeEnable(regs);
exit:
    return ret;
}
#endif /* QSPICTRL_INT_ENABLE */

static void qspictrlObjectUnInit(Qspictrl_s *t)
{
    t->initOk = false;
    ospRecursiveMutexDestroy(&t->slaveMutex);

    if (t->semIdInterrupt) {
        ospSemaphoreDelete(t->semIdInterrupt);
        t->semIdInterrupt = 0;
    }
}

static S32 qspictrlObjectInit(Qspictrl_s *t, QspictrlHcfg_s *hcfg)
{
    S32 ret = EXIT_SUCCESS;
    OspStatusCode_e status;

    if (!t) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    /* fix bitfile func mux issue. */
    qspictrlReset();

    memcpy(&t->hcfg, hcfg, sizeof(t->hcfg));
    t->regs = (QspictrlRegs_vs *)hcfg->mapAddr;
    ospRecursiveMutexInit(&t->slaveMutex, "Qspictrl_s");
    status = ospSemaphoreCreate(
        ospBuildName('S', 'P', 'I', 'I'),
        1,
        OSP_SIMPLE_BINARY_SEMAPHORE | OSP_FIFO,
        USER_TASK_PRIORITY_LOWEST,
        &t->semIdInterrupt);

    if (status != OSP_SUCCESSFUL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
#ifdef QSPICTRL_INT_ENABLE
    if (EXIT_SUCCESS != qspictrlInterruptInit(t)) {
        LOGD("%s(): qspictrlInterruptInit() failed\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto error;
    }
#endif
    t->initOk = true;
    goto exit;
error:
    qspictrlObjectUnInit(t);
exit:
    return ret;
}

static S32 qspictrlSlaveSelect(Qspictrl_s *t, QspictrlSlave_s *dev)
{
    S32 ret = EXIT_SUCCESS;

    if (!qspictrlChecking(t)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == t->curSlave) {
        /* unnecessary for set qflash master */
        goto exit;
    }
    /* different chip, set qflash master */
    if (EXIT_SUCCESS != qspictrlHardwareConfig(t, dev)) {
        LOGD("%s(): hardware_config failed\r\n", __func__);
        ret = -EXIT_FAILURE;
        t->curSlave = 0;
        goto exit;
    }
    t->curSlave = dev;

exit:
    return ret;
}

static QspictrlSlave_s *qspictlFindSlaveByCs(U8 cs)
{
    int idx;
    QspictrlSlaveMgr_s *slaveMgr = qspictrlSlaveMgrGet();

    for (idx = 0; idx < slaveMgr->slaveNum; idx++) {
        if (cs == slaveMgr->slaveTbl[idx]->hcfg.slaveCfg.cs)
            return slaveMgr->slaveTbl[idx];
    }

    return NULL;
}

/**
 * @brief  Qspictrl_s read function
 * @param [in] DevList_e devId : module ID number
 * @param [in] cs: channel ID
 * @param [in] addr: destinate addr
 * @param [in] buf : Data
 * @param [in] size: Data size
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 qspictrlRead(DevList_e devId, U8 cs, U8 addr, void *buf, U32 size)
{
    S32 ret = EXIT_SUCCESS;
    Qspictrl_s *t = NULL;
    QspictrlSlave_s *dev = NULL;
    QspictrlRegs_vs *regs = NULL;
    U32 i;
    U32 dummyCnt = 0;
    U32 rIndex = 0;
    U32 wIndex = 0;
    U32 maxSegSize = QSPICTRL_MAX_SEGMENT_LEN, cur_seg_size;
    U32 segNum = size / maxSegSize;
    U32 surplus = size % maxSegSize;
    U32 totalSegNum = segNum + (surplus) ? 1 : 0;
    U32 timeout = WAIT_FOR_QSPICTRL_TIMEOUT;
    U32 coefTimeout = 1000;
    U32 pollTimeout = timeout * coefTimeout;
    U32 sendReadCnt = 0;
    U32 receiveReadCnt = 0;
    U32 outstandingReadCnt = 0;
    volatile U8 *rwdataReg = NULL;
    QspictrlIntRawReg_u intRawStatus;
    U8 *data = (U8 *)buf;

    dev = qspictlFindSlaveByCs(cs);
    if (dev == NULL) {
        LOGD("%s _qspictrl_read get slave object failed %d\n", __func__, cs);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    t = dev->drv;
    ret = qspictrlLock(t);
    if (ret != EXIT_SUCCESS) {
        LOGD("add qspictrl read lock failed!");
        goto exit;
    }

    qspictrlSlaveSelect(t, dev);
    regs = t->regs;
    rwdataReg = (volatile U8 *)regs->rdata;

    if (!qspictrlChecking(t)) {
        ret = -EXIT_FAILURE;
        goto exit_check;
    }

    dummyCnt = t->curSlave->hcfg.slaveCfg.readDummyCnt;
    qspictrlDummyCntSet(regs, dummyCnt);
    /* Write whole packet  */
    do {
        /* check if tx fifo  */
        intRawStatus.dword = regs->int_raw_status.dword;
        if (intRawStatus.fields.crc_error || intRawStatus.fields.rx_overflow) {
            LOGE("%s crc_error %d or rx_overflow %d fail\r\n", __func__,
                intRawStatus.fields.crc_error, intRawStatus.fields.rx_overflow);
            ret = -EXIT_FAILURE;
            goto exit_sem;
        }
        outstandingReadCnt = sendReadCnt - receiveReadCnt;
        if (outstandingReadCnt > 0) {
            if (intRawStatus.fields.rx_valid) {
                receiveReadCnt++;
                qspictrlUpdateWr(regs); ///< update data from fifo to dataid
                if (regs->rdata_id.fields.rdata_crc) {
                    LOGE("%s crc_error\r\n", __func__);
                    ret = -EXIT_FAILURE;
                    goto exit_sem;
                }
                for (i = 0; i < regs->rdata_id.fields.rdata_size; i++) {
                    data[rIndex++] = rwdataReg[i];
                }
            }
        }
        outstandingReadCnt = sendReadCnt - receiveReadCnt;
        if (outstandingReadCnt >= QSPICTRL_MAX_TRANSACATION) {
#ifdef QSPICTRL_INT_ENABLE
            qspictrlIntmodeEnable(regs);
            if (ospSemaphoreObtain(t->semIdInterrupt, OSP_WAIT,
                INNER_MILLISECONDS_TO_TICKS(timeout)) != OSP_SUCCESSFUL) {
                ret = -EXIT_FAILURE;
                LOGD("%s(): read: obtain sem timeout\r\n", __func__);
                goto exit;
            }
#else
            if (!pollTimeout) {
                ret = -EXIT_FAILURE;
                goto exit_sem;
            }
            pollTimeout--;
#endif
            continue;
        }
        if (sendReadCnt >= totalSegNum) {
            /* all read cmd send */
            if (!pollTimeout) {
                ret = -EXIT_FAILURE;
                goto exit_sem;
            }
            pollTimeout--;
            continue;
        }
        if (QSPICTRL_MAX_TRANSACATION == qspictrlTxFifoLevelGet(regs)) {
            if (!pollTimeout) {
                ret = -EXIT_FAILURE;
                goto exit_sem;
            }
            pollTimeout--;
            continue;
        }
        qspictrlSlaveAddrSet(regs, addr + wIndex);
        if (EXIT_SUCCESS != qspictrlWriteDataSet(regs, NULL, 0)) {
            LOGE("%s len qspictrlWriteDataSet fail\r\n", __func__);
            ret = -EXIT_FAILURE;
            goto exit_sem;
        }
        cur_seg_size = (sendReadCnt == segNum) ? surplus : maxSegSize;
        qspictrlCmdSet(regs, false, cur_seg_size);
        qspictrlXferStart(regs);
        wIndex += cur_seg_size;
        sendReadCnt++;
    } while (receiveReadCnt < totalSegNum);
    assert(wIndex == size);
    assert(rIndex == size);
exit_sem:
    qspictrlUpdateWr(regs);   ///< update data from fifo to data id
exit_check:
    ret = qspictrlUnlock(t);
    if (ret != EXIT_SUCCESS) {
        LOGD("release qspictrl read lock failed!");
    }
exit:
    return ret;
}

/**
 * @brief  Qspictrl_s write function
 * @param [in] devId : module ID number
 * @param [in] cs: channel ID
 * @param [in] addr: destinate addr
 * @param [in] buf : Data
 * @param [in] size: Data size
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 qspictrlWrite(DevList_e devId, U8 cs, U8 addr, void *buf, U32 size)
{
    S32 ret = EXIT_SUCCESS;
    QspictrlSlave_s *dev = NULL;
    Qspictrl_s *t = NULL;
    QspictrlRegs_vs *regs = NULL;
    U32 i;
    QspictrlSlave_s *curSlave = NULL;
    U32 dummyCnt = 0;
    U32 index = 0;
    U32 maxSegSize = QSPICTRL_MAX_SEGMENT_LEN;
    U32 segNum = size / maxSegSize;
    U32 surplus = size % maxSegSize;
    U32 timeout = WAIT_FOR_QSPICTRL_TIMEOUT;
    U8 *data = (U8 *)buf;
#ifndef QSPICTRL_INT_ENABLE
    U32 coefTimeout = 1000;
#endif

    dev = qspictlFindSlaveByCs(cs);
    if (dev == NULL) {
        LOGD("%s qspictrlWrite get slave object failed %d\n", __func__, cs);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    t = dev->drv;

    ret = qspictrlLock(t);
    if (ret != EXIT_SUCCESS) {
        LOGD("add qspictrl write lock failed!");
        goto exit;
    }

    qspictrlSlaveSelect(t, dev);
    regs = t->regs;
    curSlave = t->curSlave;

    if (!qspictrlChecking(t) || !curSlave) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    qspictrlSlaveAddrSet(regs, addr);
    dummyCnt = t->curSlave->hcfg.slaveCfg.readDummyCnt;
    qspictrlDummyCntSet(regs, dummyCnt);
    /* Write whole packet  */
    for (i = 0; i < segNum; i++) {
#ifdef QSPICTRL_INT_ENABLE
        while (QSPICTRL_MAX_TRANSACATION == qspictrlTxFifoLevelGet(regs)) {
            qspictrlIntmodeEnable(regs);
            if (ospSemaphoreObtain(t->semIdInterrupt, OSP_WAIT,
                INNER_MILLISECONDS_TO_TICKS(timeout)) != OSP_SUCCESSFUL) {
                ret = -EXIT_FAILURE;
                LOGD("%s(): read: obtain sem timeout\r\n", __func__);
                goto exit;
            }
        }
#else /* QSPICTRL_INT_ENABLE */
        /* check if tx fifo  */
        timeout = WAIT_FOR_QSPICTRL_TIMEOUT * coefTimeout;
        while (QSPICTRL_MAX_TRANSACATION == qspictrlTxFifoLevelGet(regs)) {
            if (!timeout) {
                ret = -EXIT_FAILURE;
                goto exit_sem;
            }
            timeout--;
            udelay(1000);
        }
#endif /* QSPICTRL_INT_ENABLE */
        if (EXIT_SUCCESS != qspictrlWriteDataSet(regs, &data[index], maxSegSize)) {
            LOGE("%s len qspictrlWriteDataSet fail\r\n", __func__);
            ret = -EXIT_FAILURE;
            goto exit_sem;
        }
        qspictrlCmdSet(regs, true, maxSegSize);
        qspictrlXferStart(regs);
        index += maxSegSize;
    }
#ifdef QSPICTRL_INT_ENABLE
    while (QSPICTRL_MAX_TRANSACATION == qspictrlTxFifoLevelGet(regs)) {
        qspictrlIntmodeEnable(regs);
        if (ospSemaphoreObtain(t->semIdInterrupt, OSP_WAIT,
            INNER_MILLISECONDS_TO_TICKS(timeout)) != OSP_SUCCESSFUL) {
            ret = -EXIT_FAILURE;
            LOGD("%s(): read: obtain sem timeout\r\n", __func__);
            goto exit;
        }
    }
#else /* QSPICTRL_INT_ENABLE */
    /* Write surplus packet */
    timeout = WAIT_FOR_QSPICTRL_TIMEOUT * coefTimeout;
    while (QSPICTRL_MAX_TRANSACATION == qspictrlTxFifoLevelGet(regs)) {
        if (!timeout) {
            ret = -EXIT_FAILURE;
            goto exit_sem;
        }
        timeout--;
        udelay(1000);
    }
#endif /* QSPICTRL_INT_ENABLE */
    if (EXIT_SUCCESS != qspictrlWriteDataSet(regs, &data[index], surplus)) {
        LOGE("%s len qspictrlWriteDataSet fail\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit_sem;
    }
    qspictrlCmdSet(regs, true, surplus);
    qspictrlXferStart(regs);
    index += surplus;
    assert(index == size);
exit_sem:
    ret = qspictrlUnlock(t);
    if (ret != EXIT_SUCCESS) {
        LOGD("release qspictrl write lock failed!");
    }
exit:
    return ret;
}

static S32 qspictrlSlaveInit(QspictrlSlave_s *dev, QspictrlSlaveHcfg_s *hcfg)
{
    S32 ret = EXIT_SUCCESS;

    if (!dev) {
        ret = -EXIT_FAILURE;
        return ret;
    }
    memcpy(&dev->hcfg, hcfg, sizeof(dev->hcfg));

    dev->initOk = true;
    return ret;
}

static QspictrlSlave_s *qspictrlSlaveNew(void)
{
    QspictrlSlave_s *cthis = (QspictrlSlave_s *)calloc(1, sizeof(QspictrlSlave_s));
    if (!cthis) {
        return NULL;
    }

    cthis->init = qspictrlSlaveInit;
    return cthis;
}

static void qspictrlSlaveDestroy(QspictrlSlave_s *dev)
{
    if (dev != NULL) {
        dev->init = NULL;
        free(dev);
    }
    return;
}

static bool qspictrlConfigParamIsValid(QspictrlSlaveHcfg_s *config_param)
{
    bool ret = true;

    if (config_param->slaveCfg.clk > QSPICTRL_MAX_CLK) {
        LOGE("%s(): new clk %d invalid\r\n", __func__,
            config_param->slaveCfg.clk);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (config_param->slaveCfg.cs >= QSPICTRL_CS_MAX) {
        LOGE("%s(): new cs %d invalid\r\n", __func__,
            config_param->slaveCfg.cs);
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

/**
 * @brief  Qspictrl_s module initialization
 * @param [in] deviD : module ID number
 * @param [out] none
 * @return EXIT_SUCCESS or -EXIT_FAILURE
 */
S32 qspictrlInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    Qspictrl_s *drv = qspictrlObjGet();
    QspictrlSlaveMgr_s *slaveMgr = qspictrlSlaveMgrGet();
    QspictrlSlave_s *dev = NULL;
    U32 i, j;
    bool isDuplicated;
    QspictrlHcfg_s drv_hcfg = {
        .mapAddr = SYS_QSPICTRL_BASE_ADRS,
        .interruptNum = SYS_INT_NUM_QSPICTRL,
        .interruptPriority = SYS_INT_PRIORITY_TIMESYNC,
    };

    if (isDrvInit(devId)) {
        ret = -EBUSY;
        goto exit;
    }

    if (!slaveMgr || !drv) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
    if (slaveMgr->initOk) {
        goto exit;
    }

    /* Step 1: Init Qspictrl_s driver firstly */
    drv->init = qspictrlObjectInit;
    if (EXIT_SUCCESS != drv->init(drv, &drv_hcfg)) {
        LOGD("%s(): drv init fail\r\n", __func__);
        ret = -EXIT_FAILURE;
        goto exit;
    }
    /* Step 2: Init Qspictrl_s device */
    for (i = 0; i < ARRAY_SIZE(qspictrl_slave_param_table); i++) {
        isDuplicated = false;
        if (!qspictrlConfigParamIsValid(
            (QspictrlSlaveHcfg_s *)&qspictrl_slave_param_table[i])) {
            continue;
        }
        for (j = 0; j < slaveMgr->slaveNum; j++) {
            if (qspictrl_slave_param_table[i].slaveId ==
                slaveMgr->slaveTbl[j]->hcfg.slaveId) {
                isDuplicated = true;
                break;
            }
        }
        if (isDuplicated) {
            LOGE("%s(): slaveId %d duplicate, skip it.\r\n", __func__,
                qspictrl_slave_param_table[i].slaveId);
            continue;
        }
        /* Now this is a new slave id */
        dev = qspictrlSlaveNew();
        if (!dev) {
            LOGD("%s(): new slave%d fail\r\n", __func__,
                qspictrl_slave_param_table[i].slaveId);
            ret = -EXIT_FAILURE;
            goto exit;
        }
        if (EXIT_SUCCESS != dev->init(dev, &qspictrl_slave_param_table[i])) {
            LOGD("%s(): slave%d init fail\r\n", __func__,
                qspictrl_slave_param_table[i].slaveId);
            ret = -EXIT_FAILURE;
            goto slave_free;
        }
        dev->drv = drv;
        slaveMgr->slaveTbl[slaveMgr->slaveNum++] = dev;
    }
    slaveMgr->initOk = true;

    LOGD("%s qspictrlInit OK! %d\n", __func__, slaveMgr->initOk);

exit:
    return ret;
slave_free:
    qspictrlSlaveDestroy(dev);
    dev = NULL;
    return ret;
}
