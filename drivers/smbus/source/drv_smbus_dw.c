/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw.c
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus Core protocol layer implementation
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 *
 * @note This file implements the SMBus Core protocol layer, providing
 *       the foundational SMBus protocol implementation including
 *       initialization, mode switching, ARP logic, and state management.
 *       This layer sits between the HAL layer and the API layers,
 *       implementing the core SMBus protocol logic.
 */
#include "common_defines.h"
#include "sbr_api.h"
#include "drv_smbus_dw.h"
#include "drv_smbus_dw_i2c.h"
#include "drv_smbus_api.h"

/* ======================================================================== */
/*                    SMBus Initialization/Deinitialization                 */
/* ======================================================================== */

/* Forward declarations for modular initialization functions */
static S32 smbusAllocateDriverData(DevList_e devId, SmbusDrvData_s **ppDrvData);
static S32 smbusInitializeHardware(SmbusDrvData_s *pDrvData);
static S32 smbusAllocateSlaveBuffers(SmbusDrvData_s *pDrvData);
static S32 smbusAllocateMasterBuffers(SmbusDrvData_s *pDrvData);
static S32 smbusInitializeArpMaster(SmbusDrvData_s *pDrvData);
static void smbusFreeDriverData(SmbusDrvData_s *pDrvData);

/* ======================================================================== */
/*                    Core Utility Functions                                 */
/* ======================================================================== */

/**
 * @brief Allocate slave buffers for SMBus operation
 * @details Allocates RX and TX buffers for slave mode operation.
 *          These buffers are used for data transfer in slave mode.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Only allocates buffers when in slave mode
 * @note Uses SMBUS_SLAVE_BUF_LEN for buffer size
 * @note Memory is freed in smbusFreeDriverData
 * [CORE] Core protocol layer - memory management
 */
__attribute((unused)) static S32 smbusAllocateSlaveBuffers(SmbusDrvData_s *pDrvData)
{
    if (pDrvData == NULL) {
        return -EINVAL;
    }

    /* Only allocate slave buffers in slave mode */
    if (pDrvData->sbrCfg.masterMode == 0) {
        /* Allocate slave RX buffer */
        pDrvData->pSmbusDev.slaveRxBuf = (U8 *)malloc(SMBUS_SLAVE_BUF_LEN);
        if (pDrvData->pSmbusDev.slaveRxBuf == NULL) {
            LOGE("%s: Failed to allocate slave RX buffer\n", __func__);
            return -ENOMEM;
        }
        pDrvData->pSmbusDev.slaveValidRxLen = 0;

        /* Allocate slave TX buffer */
        pDrvData->pSmbusDev.slaveTxBuf = (U8 *)malloc(SMBUS_SLAVE_BUF_LEN);
        if (pDrvData->pSmbusDev.slaveTxBuf == NULL) {
            LOGE("%s: Failed to allocate slave TX buffer\n", __func__);
            free(pDrvData->pSmbusDev.slaveRxBuf);
            pDrvData->pSmbusDev.slaveRxBuf = NULL;
            return -ENOMEM;
        }
        pDrvData->pSmbusDev.slaveValidTxLen = 0;
        pDrvData->slaveTxIndex = 0;

        LOGD("%s: Slave buffers allocated successfully\n", __func__);
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Allocate master buffers for SMBus operation
 * @details Allocates TX and RX buffers for master mode operation.
 *          These buffers are used for data transfer in master mode.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Only allocates buffers when in master mode
 * @note Uses SMBUS_MAX_BUFFER_SIZE for buffer size
 * @note Memory is freed in smbusFreeDriverData
 * [CORE] Core protocol layer - memory management
 */
static S32 smbusAllocateMasterBuffers(SmbusDrvData_s *pDrvData)
{
    if (pDrvData == NULL) {
        return -EINVAL;
    }

    /* Only initialize master buffers in master mode */
    if (pDrvData->sbrCfg.masterMode == 1) {
        /* Initialize master TX buffer array */
        memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
        pDrvData->pSmbusDev.masterTxBufLen = 0;
        LOGD("%s: Master TX buffer initialized (%u bytes)\n", __func__, sizeof(pDrvData->pSmbusDev.masterTxBuf));

        /* Note: master RX buffer is handled by pDrvData->rxBuffer in legacy mode */

        LOGD("%s: Master buffers initialized successfully\n", __func__);
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Initialize SMBus slave resources only
 * @details Allocates slave RX/TX buffers for devices that are being switched
 *          to slave mode after initial initialization.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure:
 *         -EINVAL: NULL driver data
 *         -ENOMEM: Memory allocation failed
 * @note This function should only be called when switching TO slave mode
 * @note Does NOT perform full device initialization, only slave-specific resources
 * [CORE] Core protocol layer - slave resource initialization
 */
static S32 smbusInitSlaveResources(SmbusDrvData_s *pDrvData)
{
    if (pDrvData == NULL) {
        LOGE("%s: NULL driver data\n", __func__);
        return -EINVAL;
    }

    /* Only allocate if not already allocated */
    if (pDrvData->pSmbusDev.slaveRxBuf == NULL) {
        LOGD("%s: Allocating slave RX buffer\n", __func__);
        pDrvData->pSmbusDev.slaveRxBuf = (U8 *)malloc(SMBUS_SLAVE_BUF_LEN);
        if (pDrvData->pSmbusDev.slaveRxBuf == NULL) {
            LOGE("%s: Failed to allocate slave RX buffer\n", __func__);
            return -ENOMEM;
        }
        pDrvData->pSmbusDev.slaveValidRxLen = 0;
    }

    if (pDrvData->pSmbusDev.slaveTxBuf == NULL) {
        LOGD("%s: Allocating slave TX buffer\n", __func__);
        pDrvData->pSmbusDev.slaveTxBuf = (U8 *)malloc(SMBUS_SLAVE_BUF_LEN);
        if (pDrvData->pSmbusDev.slaveTxBuf == NULL) {
            LOGE("%s: Failed to allocate slave TX buffer\n", __func__);
            free(pDrvData->pSmbusDev.slaveRxBuf);
            pDrvData->pSmbusDev.slaveRxBuf = NULL;
            return -ENOMEM;
        }
        pDrvData->pSmbusDev.slaveValidTxLen = 0;
        pDrvData->slaveTxIndex = 0;
    }

    LOGI("%s: Slave resources initialized successfully\n", __func__);
    return EXIT_SUCCESS;
}

/**
 * @brief Free SMBus slave resources only
 * @details Frees slave RX/TX buffers for devices that are switching
 *          away from slave mode.
 * @param[in] pDrvData Pointer to driver data structure
 * @return void
 * @note This function should only be called when switching FROM slave mode
 * [CORE] Core protocol layer - slave resource cleanup
 */
static void smbusFreeSlaveResources(SmbusDrvData_s *pDrvData)
{
    if (pDrvData == NULL) {
        return;
    }

    if (pDrvData->pSmbusDev.slaveRxBuf != NULL) {
        LOGD("%s: Freeing slave RX buffer\n", __func__);
        free(pDrvData->pSmbusDev.slaveRxBuf);
        pDrvData->pSmbusDev.slaveRxBuf = NULL;
        pDrvData->pSmbusDev.slaveValidRxLen = 0;
    }

    if (pDrvData->pSmbusDev.slaveTxBuf != NULL) {
        LOGD("%s: Freeing slave TX buffer\n", __func__);
        free(pDrvData->pSmbusDev.slaveTxBuf);
        pDrvData->pSmbusDev.slaveTxBuf = NULL;
        pDrvData->pSmbusDev.slaveValidTxLen = 0;
    }

    LOGI("%s: Slave resources freed successfully\n", __func__);
}

/**
 * @brief Copy UDID structure
 * @param dest Destination UDID
 * @param src Source UDID
 * [CORE] Core protocol layer - UDID copying
 */
__attribute((unused)) static void smbusUdidCopy(SmbusUdid_s *dest, const SmbusUdid_s *src)
{
    if (dest == NULL || src == NULL) {
        return;
    }

    dest->nextAvailAddr = src->nextAvailAddr;
    dest->version = src->version;
    dest->vendorId = src->vendorId;
    dest->deviceId = src->deviceId;
    dest->interface = src->interface;
    dest->subsystemVendorId = src->subsystemVendorId;
    memcpy(dest->bytes, src->bytes, sizeof(dest->bytes));
    dest->deviceAddr = src->deviceAddr;
}

/**
 * @brief Get SMBus device configuration from SBR
 * @param devId Device identifier
 * @param sbrCfg Pointer to SBR configuration structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 * [CORE] Core protocol layer - configuration management
 */
__attribute((unused)) static S32 smbusDevCfgGet(DevList_e devId, SbrI2cSmbusCfg_s *smbusSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (smbusSbrCfg == NULL) {
        ret = -EINVAL;
        goto out;
    }

    if (devSbrRead(devId, smbusSbrCfg, 0, sizeof(SbrI2cSmbusCfg_s)) != sizeof(SbrI2cSmbusCfg_s)) {
        ret = -EIO;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("smbus: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
         smbusSbrCfg->regAddr, smbusSbrCfg->irqNo, smbusSbrCfg->irqPrio);
#endif

    if (smbusSbrCfg->irqNo == 0 || smbusSbrCfg->irqPrio == 0 || smbusSbrCfg->regAddr == NULL) {
        ret = -EINVAL;
        goto out;
    }

out:
    return ret;
}

/**
 * @brief Get SMBus register base address
 * @param devId Device identifier
 * @param pCtrlReg Pointer to register map pointer
 * @return EXIT_SUCCESS on success, negative error code on failure
 * [CORE] Core protocol layer - register access
 */
S32 getSmbusReg(DevList_e devId, SmbusRegMap_s **pCtrlReg)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    /* Check driver match */
    if(isDrvInit(devId) == false) {
        return -EINVAL;
    }

    if (pCtrlReg == NULL) {
        return -EINVAL;
    }

    if(getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        return -EIO;
    }

    *pCtrlReg = (SmbusRegMap_s*)pDrvData->sbrCfg.regAddr;

    return ret;
}

S32 smbusDrvLockAndCheck(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    if (devLockByDriver(devId, SMBUS_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (isDrvInit(devId) == false) {
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_DW_I2C)) {
        ret = -EINVAL;
        goto exit;
    }
exit:
    return ret;
}

/* ======================================================================== */
/*                    Core Callback Functions                                 */
/* ======================================================================== */

/**
 * @brief Register callback function for SMBus events
 * @param pDrvData Pointer to driver data structure
 * @param cb Callback function pointer
 * @param userData User data to pass to callback
 * @return 0 on success, -1 on failure
 * [CORE] Core protocol layer - callback management
 */
__attribute((unused)) S32 SmbusRegisterCallback(SmbusDrvData_s *pDrvData, SmbusCallback_t cb, void *userData)
{
    if (!pDrvData) return -EINVAL;

    pDrvData->callback.cb = cb;
    pDrvData->callback.userData = userData;

    LOGD("SMBus: Callback registered!\r\n");
    return EXIT_SUCCESS;
}

/**
 * @brief Unregister callback function
 * @param pDrvData Pointer to driver data structure
 * @return 0 on success, -1 on failure
 * [CORE] Core protocol layer - callback management
 */
__attribute((unused)) S32 SmbusUnregisterCallback(SmbusDrvData_s *pDrvData)
{
    if (!pDrvData) return -EINVAL;

    pDrvData->callback.cb = NULL;
    pDrvData->callback.userData = NULL;

    LOGD("SMBus: Callback unregistered!\r\n");
    return EXIT_SUCCESS;
}

/**
 * @brief Allocate and initialize SMBus driver data structure
 * @details Allocates memory for driver data and initializes basic fields.
 * @param[in] devId SMBus device identifier
 * @param[out] ppDrvData Pointer to store allocated driver data
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note This function performs basic memory allocation only
 * @note Hardware-specific initialization is done in later steps
 * [CORE] Core protocol layer - driver data management
 */
static S32 smbusAllocateDriverData(DevList_e devId, SmbusDrvData_s **ppDrvData)
{
    SmbusDrvData_s *pDrvData;

    if (ppDrvData == NULL) {
        return -EINVAL;
    }
    LOGE("1\r\n");
    /* Allocate driver data structure */
    pDrvData = (SmbusDrvData_s *)calloc(1, sizeof(SmbusDrvData_s));
    if (pDrvData == NULL) {
        LOGE("%s: Failed to allocate driver data\n", __func__);
        return -ENOMEM;
    }

    /* Initialize basic fields */
    pDrvData->devId = devId;
    pDrvData->retryCount = 0;
    pDrvData->enablePec = 0;
    pDrvData->txComplete = 0;
    pDrvData->rxComplete = 0;
    pDrvData->errorCode = 0;
    pDrvData->lastError = 0;
    pDrvData->errorCount = 0;
    pDrvData->slaveTransferActive = 0;

    /* Initialize master buffer variables */
    memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
    pDrvData->pSmbusDev.masterTxBufLen = 0;

    /* Initialize callback queue (fix ISR blocking issue) */
    pDrvData->callbackQueue.head = 0;
    pDrvData->callbackQueue.tail = 0;
    pDrvData->callbackPending = false;

    /* CRITICAL FIX: Initialize callback structure to prevent NULL pointer issues */
    pDrvData->callback.cb = NULL;           /* Initialize callback pointer to NULL */
    pDrvData->callback.userData = NULL;     /* Initialize user data to NULL */

    /* Initialize SMBus device structure with Calmera naming */
    pDrvData->pSmbusDev.busId = devId;
    pDrvData->pSmbusDev.status = 0;
    pDrvData->pSmbusDev.cmdErr = 0;
    pDrvData->pSmbusDev.abortSource = 0;
    pDrvData->pSmbusDev.enabled = 0;
    pDrvData->pSmbusDev.mode = DW_SMBUS_MODE_MASTER;  /* Default to master mode */
    pDrvData->pSmbusDev.flags = 0;
    pDrvData->pSmbusDev.transferTimeout = SMBUS_DEFAULT_TIMEOUT_MS;  /* Default timeout */
    pDrvData->pSmbusDev.transferStartTime = 0;
    memset(pDrvData->pSmbusDev.msgs, 0, sizeof(SmbusMsg_t) *32);
    /* Note: masterRxBuf and masterRxBufLen have been removed - use pDrvData->rxBuffer instead */

    LOGE("2\r\n");
    *ppDrvData = pDrvData;
    return EXIT_SUCCESS;
}

/**
 * @brief Initialize SMBus hardware using HAL probe functions
 * @details Configures hardware based on mode (master/slave) using I2C-compatible
 *          probe functions from the HAL layer.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Uses smbusProbeMaster for master mode
 * @note Uses smbusProbeSlave for slave mode
 * @note Compatible with I2C initialization pattern
 * [CORE] Core protocol layer - hardware initialization
 */
static S32 smbusInitializeHardware(SmbusDrvData_s *pDrvData)
{
    S32 ret;

    if (pDrvData == NULL) {
        return -EINVAL;
    }
    /* Configure SMBus device structure based on SBR settings */
    pDrvData->pSmbusDev.regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    pDrvData->pSmbusDev.addrMode = pDrvData->sbrCfg.addrMode;
    pDrvData->pSmbusDev.slaveAddr = pDrvData->sbrCfg.slaveAddrLow;
    pDrvData->pSmbusDev.irq = pDrvData->sbrCfg.irqNo;
    pDrvData->pSmbusDev.channelNum = pDrvData->devId;
    pDrvData->pSmbusDev.workMode = (pDrvData->sbrCfg.interruptMode == 1) ? 0 : 1;
    pDrvData->pSmbusDev.isSmbus = 1;
    pDrvData->pSmbusDev.transferTimeout = SMBUS_TRANSFER_TIMEOUT_MS;

    /* Configure clock rate based on speed setting */
    switch (pDrvData->sbrCfg.speed) {
        case 0:
            pDrvData->pSmbusDev.clkRate = 100000;  /* 100 kHz Standard mode */
            break;
        case 1:
            pDrvData->pSmbusDev.clkRate = 400000;  /* 400 kHz Fast mode */
            break;
        case 2:
            pDrvData->pSmbusDev.clkRate = 1000000; /* 1 MHz Fast Plus mode */
            break;
        default:
            pDrvData->pSmbusDev.clkRate = 400000; /* Default to Fast mode */
            break;
    }

    /* Configure SMBus controller based on mode using I2C-compatible probe functions */
    if (pDrvData->sbrCfg.masterMode == 1) {
        /* OPTIMIZATION: Allocate master RX buffer only if using legacy interrupt mode */
        if (pDrvData->sbrCfg.interruptMode == 0) {  /* Legacy interrupt mode */
            pDrvData->rxBufferSize = SMBUS_MAX_BUFFER_SIZE;
            pDrvData->rxBuffer = (U8 *)malloc(pDrvData->rxBufferSize);
            if (pDrvData->rxBuffer == NULL) {
                LOGE("%s: Failed to allocate master RX buffer of size %d\n", __func__, pDrvData->rxBufferSize);
                return -ENOMEM;
            }
            pDrvData->rxLength = 0;
            LOGD("%s: Master RX buffer allocated for legacy mode (size=%d)\n", __func__, pDrvData->rxBufferSize);
        } else {
            /* Async mode doesn't need global rxBuffer - data goes directly to message buffers */
            pDrvData->rxBuffer = NULL;
            pDrvData->rxBufferSize = 0;
            pDrvData->rxLength = 0;
            LOGD("%s: Async mode - no global RX buffer needed\n", __func__);
        }

        /* Master mode initialization */
        ret = smbusProbeMaster(&pDrvData->pSmbusDev);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: smbusProbeMaster failed, ret=%d\n", __func__, ret);
            if (pDrvData->rxBuffer != NULL) {
                free(pDrvData->rxBuffer);  /* Clean up allocated memory */
                pDrvData->rxBuffer = NULL;
            }
            return ret;
        }
        pDrvData->pSmbusDev.mode = DW_SMBUS_MODE_MASTER;
        LOGD("%s: Master hardware initialization completed\n", __func__);
    } else {
        /* Slave mode initialization */
        ret = smbusProbeSlave(&pDrvData->pSmbusDev);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: smbusProbeSlave failed, ret=%d\n", __func__, ret);
            return ret;
        }
        pDrvData->pSmbusDev.mode = SMBUS_MODE_SLAVE;
        LOGD("%s: Slave hardware initialization completed\n", __func__);
    }

    /* Mark device as enabled */
    pDrvData->pSmbusDev.enabled = 1;

    /* Create semaphore for synchronization */
    /* Use a fixed name to avoid character calculation issues */
    rtems_name semName = rtems_build_name('S', 'M', 'B', '0');

    /* Debug: print channelNum and semName */
    LOGD("%s: Creating semaphore with channelNum=%d, semName=0x%08X\n",
         __func__, pDrvData->pSmbusDev.channelNum, semName);

    S32 semRet = rtems_semaphore_create(semName, 0,
                                        RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_FIFO, 0,
                                        &(pDrvData->pSmbusDev.semaphoreId));
    LOGD("%s: rtems_semaphore_create returned %d\n", __func__, semRet);
    if (semRet != RTEMS_SUCCESSFUL) {
        LOGE("%s: Semaphore creation failed, ret=%d, channelNum=%d\n",
             __func__, semRet, pDrvData->pSmbusDev.channelNum);
        /* Ensure semaphoreId is set to 0 on failure */
        pDrvData->pSmbusDev.semaphoreId = 0;
        return -EIO;
    }
    LOGD("%s: Semaphore created successfully, ID=%d, channelNum=%d\n",
         __func__, pDrvData->pSmbusDev.semaphoreId, pDrvData->pSmbusDev.channelNum);

    /* FINAL SAFETY FIX: Force correct Slave interrupt mask if in Slave mode */
    if (pDrvData->sbrCfg.masterMode == 0) {
        volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

        /* CRITICAL: Override any previous incorrect interrupt mask settings */
        U32 slaveMask = SMBUS_IC_INTR_RX_FULL_MASK |      /* 0x004 */
                          SMBUS_IC_INTR_RD_REQ_MASK |       /* 0x020 */
                          SMBUS_IC_INTR_TX_ABRT_MASK |      /* 0x040 */
                          SMBUS_IC_INTR_RX_DONE_MASK |      /* 0x080 */
                          SMBUS_IC_INTR_STOP_DET_MASK;     /* 0x200 */

        /* Calculate: 0x004 | 0x020 | 0x040 | 0x080 | 0x200 = 0x2E4 */
        regBase->icIntrMask = slaveMask;

        LOGI("FINAL OVERRIDE: Slave interrupt mask FORCED to 0x%08X (expected: 0x2E4)\n", regBase->icIntrMask);

        /* Verify final setting */
        U32 finalMask = regBase->icIntrMask;
        if (finalMask != slaveMask) {
            LOGE("FINAL OVERRIDE FAILED! Expected: 0x%08X, Actual: 0x%08X\n", slaveMask, finalMask);
        } else {
            LOGI("FINAL OVERRIDE SUCCESS: Slave interrupt mask correctly set to 0x%08X\n", finalMask);
        }
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Clear interrupts based on mask (modular function for both I2C and SMBus)
 * @details Clears specific interrupts based on the provided mask. This function
 *          provides a modular approach to interrupt clearing that can be inherited
 *          by both I2C and SMBus drivers.
 * @param[in] regBase Pointer to SMBus register map
 * @param[in] mask Interrupt mask specifying which interrupts to clear
 * @return void
 *
 * @note If mask equals STARS_I2C_STATUS_INT_ALL, reads ic_clr_intr to clear all interrupts
 * @note Clears specific interrupts based on individual mask bits:
 *       - m_rx_under: Read ic_clr_rx_under
 *       - m_rx_over: Read ic_clr_rx_over
 *       - m_tx_over: Read ic_clr_tx_over
 *       - m_rd_req: Read ic_clr_rd_req
 *       - m_tx_abrt: Read ic_clr_tx_abrt
 *       - m_rx_done: Read ic_clr_rx_done
 *       - m_activity: Read ic_clr_activity
 *       - m_stop_det: Read ic_clr_stop_det
 *       - m_start_det: Read ic_clr_start_det
 *       - m_gen_call: Read ic_clr_gen_call
 *       - m_restart_det: Read ic_clr_restart_det
 * @note For SMBus, also clears ic_clr_smbus_intr with the mask
 * @warning This function should be called with valid register base
 * [CORE] Core protocol layer - interrupt management
 */
static void smbusClearInterrupts(volatile SmbusRegMap_s *regBase, U32 mask)
{
    if (regBase == NULL) {
        return;
    }

    /* Clear all interrupts if mask equals ALL mask */
    if (mask == 0xFFFFFFFF) { /* STARS_I2C_STATUS_INT_ALL equivalent */
        (void)regBase->icClrIntr;
        if (regBase->icClrSmbusIntr != 0) {
            (void)regBase->icClrSmbusIntr;  /* Dummy read to clear SMBus interrupts */
        }
        LOGD("%s: Cleared all interrupts\n", __func__);
        return;
    }

    /* Clear specific I2C interrupts based on mask bits using correct macro definitions */
    if (mask & SMBUS_IC_INTR_RX_UNDER_MASK) {  /* bit[0] - m_rx_under */
        (void)regBase->icClrRxUnder;
    }
    if (mask & SMBUS_IC_INTR_RX_OVER_MASK) {   /* bit[1] - m_rx_over */
        (void)regBase->icClrRxOver;
    }
    if (mask & SMBUS_IC_INTR_TX_OVER_MASK) {   /* bit[3] - m_tx_over */
        (void)regBase->icClrTxOver;
    }
    if (mask & SMBUS_IC_INTR_RD_REQ_MASK) {    /* bit[5] - m_rd_req */
        (void)regBase->icClrRdReq;
    }
    if (mask & SMBUS_IC_INTR_TX_ABRT_MASK) {   /* bit[6] - m_tx_abrt */
        (void)regBase->icClrTxAbrt;
    }
    if (mask & SMBUS_IC_INTR_RX_DONE_MASK) {   /* bit[7] - m_rx_done */
        (void)regBase->icClrRxDone;
    }
    if (mask & SMBUS_IC_INTR_ACTIVITY_MASK) {  /* bit[8] - m_activity */
        (void)regBase->icClrActivity;
    }
    if (mask & SMBUS_IC_INTR_STOP_DET_MASK) {  /* bit[9] - m_stop_det */
        (void)regBase->icClrStopDet;
    }
    if (mask & SMBUS_IC_INTR_START_DET_MASK) { /* bit[10] - m_start_det */
        (void)regBase->icClrStartDet;
    }
    if (mask & SMBUS_IC_INTR_GEN_CALL_MASK) {  /* bit[11] - m_gen_call */
        (void)regBase->icClrGenCall;
    }
    if (mask & SMBUS_IC_INTR_RESTART_DET_MASK) { /* bit[12] - m_restart_det */
        (void)regBase->icClrRestartDet;
    }

    /* Clear SMBus specific interrupts if available */
    if (regBase->icClrSmbusIntr != 0) {
        (void)regBase->icClrSmbusIntr;  /* Dummy read to clear SMBus interrupts */
    }

    LOGD("%s: Cleared interrupts with mask 0x%08X\n", __func__, mask);
}

/**
 * @brief Clear all interrupts (convenience function)
 * @details Clears all I2C and SMBus interrupts. This is a convenience function
 *          that calls smbusClearInterrupts with the ALL interrupt mask.
 * @param[in] regBase Pointer to SMBus register map
 * @return void
 *
 * @note This function is typically called during initialization
 * @warning This function should be called with valid register base
 * [CORE] Core protocol layer - interrupt management
 */
static void smbusClearAllInterrupts(volatile SmbusRegMap_s *regBase)
{
    smbusClearInterrupts(regBase, 0xFFFFFFFF); /* Clear all interrupts */
}

/**
 * @brief Handle master mode interrupts (integrated from i2c_dw_isr)
 * @details Processes master mode interrupt events including TX abort,
 *          RX ready, TX ready. This function integrates I2C master ISR
 *          logic with SMBus protocol handling.
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] intrStat I2C interrupt status
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 *
 * @note Handles ARP failure events and triggers appropriate actions
 * @note Uses SMBus callback functions for event notification
 * [CORE] Core protocol layer - master interrupt handling
 */
static void smbusHandleMasterInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                       U32 intrStat, U32 smbusIntrStat)
{
    DevList_e devId = pDrvData->devId;
    SmbusEventData_u eventData;
    Bool needCallback = false;
    Bool asyncComplete = false;

    /* ========== ASYNC MODE: Handle async interrupt transfers ========== */
    LOGD("SMBus Master: Checking async mode - interruptMode=%d, status=0x%08X, active=%d\n",
         pDrvData->sbrCfg.interruptMode, pDrvData->pSmbusDev.status,
         (pDrvData->pSmbusDev.status & SMBUS_STATUS_ACTIVE) ? 1 : 0);
    LOGD("SMBus Master: Device addresses - pDrvData=%p, pSmbusDev=%p, regBase=%p\n",
         (void*)pDrvData, (void*)&pDrvData->pSmbusDev, (void*)pDrvData->pSmbusDev.regBase);

    if (pDrvData->sbrCfg.interruptMode == 1) {
        LOGD("SMBus Master: Async mode processing - status=0x%08X\n", pDrvData->pSmbusDev.status);

        /* Only process transfer logic if actively transferring */
        if (pDrvData->pSmbusDev.status & (SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS)) {
            LOGD("SMBus Master: Active transfer detected - processing async interrupts\n");
            /* Async interrupt mode handling - similar to I2C implementation */
            SmbusDev_s *dev = &pDrvData->pSmbusDev;

            /* Initialize transfer completion tracking */
            pDrvData->txComplete = 0;

        /* Handle TX abort in async mode */
        if (intrStat & SMBUS_IC_INTR_TX_ABRT_MASK) {
            dev->msgErr |= SMBUS_ERR_TX_ABRT;
            dev->status &= ~SMBUS_STATUS_MASK;
            dev->rxOutstanding = 0;

            /* Preserve abort source */
            dev->abortSource = regBase->icTxAbrtSource.value;

            /* Disable interrupts */
            regBase->icIntrMask = 0;

            /* Clear abort status using dummy read */
            volatile U32 abortDummy = regBase->icClrTxAbrt;  /* Dummy read to clear TX abort */
            (void)abortDummy; /* Suppress unused variable warning */

            LOGE("SMBus Async: TX Abort, source=0x%08X\n", dev->abortSource);
            asyncComplete = true;
        }

        /* Handle ACTIVITY interrupt in async mode */
        if (intrStat & SMBUS_IC_INTR_ACTIVITY_MASK) {
            /* Clear activity status - this interrupt indicates bus activity started */
            (void)regBase->icClrActivity;    /* Dummy read to clear activity */
            LOGI("SMBus Async: Activity detected (0x%08X) and cleared\n", intrStat);
        }

        /* Handle RX full in async mode - CRITICAL: Add missing RX handling */
        if (intrStat & SMBUS_IC_INTR_RX_FULL_MASK) {
            /* Process async RX data using HAL functions */
            SmbusHalOps_s *halOps = smbusGetHalOps();
            if (halOps != NULL && halOps->smbusDwRead != NULL) {
                LOGD("SMBus Async: RX_FULL detected, calling smbusDwRead\n");
                halOps->smbusDwRead(pDrvData);

                /* CRITICAL FIX: After processing, check if FIFO still has data and clear if needed */
                U32 remaining_fifo = regBase->icRxflr;
                if (remaining_fifo > 0) {
                    LOGW("SMBus Async: RX_FULL - FIFO still has %d items after processing, clearing...\n", remaining_fifo);
                    /* Clear remaining data to prevent interrupt loop */
                    while (regBase->icRxflr > 0) {
                        (void)regBase->icDataCmd.value;  /* Read and discard */
                    }
                }
            } else {
                LOGE("SMBus Async: HAL read function not available - halOps=%p, smbusDwRead=%p\n",
                     halOps, halOps ? halOps->smbusDwRead : NULL);

                /* CRITICAL: Clear FIFO to prevent interrupt loop when HAL functions unavailable */
                U32 fifo_level = regBase->icRxflr;
                if (fifo_level > 0) {
                    LOGW("SMBus Async: Clearing FIFO due to HAL unavailability (%d items)\n", fifo_level);
                    while (regBase->icRxflr > 0) {
                        (void)regBase->icDataCmd.value;  /* Read and discard */
                    }
                    /* Force completion to prevent hang */
                    dev->rxOutstanding = 0;
                    dev->msgErr = -EIO;
                }
            }
            LOGD("SMBus Async: RX Full processed\n");
        }

        /* Handle TX empty in async mode */
        if (intrStat & SMBUS_IC_INTR_TX_EMPTY_MASK) {
            /* CRITICAL: Check if all messages are already processed before calling smbusDwXferMsg */
            if (dev->msgWriteIdx >= dev->msgsNum) {
                /* All messages processed - disable TX_EMPTY interrupt immediately */
                LOGW("SMBus Async: All messages already processed (idx=%d/%d), disabling TX_EMPTY\n",
                     dev->msgWriteIdx, dev->msgsNum);

                U32 currentMask = regBase->icIntrMask;
                currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                regBase->icIntrMask = currentMask;

                LOGD("SMBus Async: TX_EMPTY interrupt disabled at ISR level - mask=0x%08X\n", currentMask);

                /* CRITICAL FIX: Mark transfer as complete if all messages are processed and no outstanding RX */
                if (dev->rxOutstanding == 0) {
                    dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
                    asyncComplete = true;
                    LOGD("SMBus Async: Transfer marked complete due to all messages processed\n");
                }
            } else {
                /* Process async TX data using HAL functions */
                SmbusHalOps_s *halOps = smbusGetHalOps();
                if (halOps != NULL && halOps->smbusDwXferMsg != NULL) {
                    LOGD("SMBus Async: Calling smbusDwXferMsg, msgWriteIdx=%d, msgsNum=%d\n",
                         dev->msgWriteIdx, dev->msgsNum);
                    halOps->smbusDwXferMsg(dev);
                } else {
                    LOGE("SMBus Async: HAL functions not available - halOps=%p, smbusDwXferMsg=%p\n",
                         halOps, halOps ? halOps->smbusDwXferMsg : NULL);
                    /* Force completion if HAL functions unavailable */
                    asyncComplete = true;
                }

                /* Check interrupt mask after processing and ensure TX_EMPTY is properly disabled if needed */
                U32 newMask = regBase->icIntrMask;
                LOGD("SMBus Async: TX_EMPTY processed, new mask=0x%08X, TX_EMPTY enabled=%d\n",
                     newMask, (newMask & SMBUS_IC_INTR_TX_EMPTY_MASK) ? 1 : 0);

                /* CRITICAL FIX: If smbusDwXferMsg completed all messages, ensure TX_EMPTY is disabled */
                if (dev->msgWriteIdx >= dev->msgsNum) {
                    U32 updatedMask = regBase->icIntrMask;
                    updatedMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                    regBase->icIntrMask = updatedMask;
                    LOGD("SMBus Async: Post-processing TX_EMPTY disabled - mask=0x%08X\n", updatedMask);
                }
            }
            LOGD("SMBus Async: TX Empty processed\n");
        }

        /* Check for async transfer completion */
        if (((intrStat & (SMBUS_IC_INTR_TX_ABRT_MASK | SMBUS_IC_INTR_STOP_DET_MASK)) || dev->msgErr) &&
            (dev->msgWriteIdx >= dev->msgsNum) && (dev->rxOutstanding == 0)) {
            /* Clear transfer active status */
            dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
            asyncComplete = true;

            LOGE("SMBus Async: Transfer completed (abort/stop), releasing semaphore (semId=%d)\n", dev->semaphoreId);
#if 1
            /* CRITICAL FIX: Ensure TX_EMPTY is disabled when transfer completes */
            U32 abortMask = regBase->icIntrMask;
            abortMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            regBase->icIntrMask = abortMask;
#endif
            LOGD("SMBus Async: TX_EMPTY disabled due to abort/stop completion - mask=0x%08X\n", abortMask);
        }

        /* Simplified STOP detection - ensure semaphore release for write operations */
        else if (intrStat & SMBUS_IC_INTR_STOP_DET_MASK) {
            /* STOP检测到，说明传输完成 */
            if (dev->status & (SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS)) {
                dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
                asyncComplete = true;

                LOGD("SMBus Async: STOP detected, releasing semaphore (semId=%d)\n", dev->semaphoreId);
                /* CRITICAL FIX: Ensure complete interrupt cleanup on STOP detection */
                LOGE("SMBus Async: TX_EMPTY disabled due to STOP detection - mask=0x%08X\n", regBase->icIntrMask);

                /* Force clear any pending TX_EMPTY condition */
                if (dev->msgWriteIdx >= dev->msgsNum) {
                    //regBase->icIntrMask = 0;  /* Disable all interrupts on STOP */
                    LOGD("SMBus Async: All interrupts disabled on STOP detection\n");
                }
            }
        }
        /* Enhanced Timeout protection - force completion if transfer takes too long */
        if (dev->status & SMBUS_STATUS_ACTIVE) {
            U32 currentTime = rtems_clock_get_ticks_since_boot();
            U32 timeoutTicks = RTEMS_MILLISECONDS_TO_TICKS(dev->transferTimeout);

            if ((currentTime - dev->transferStartTime) > timeoutTicks) {
                LOGE("SMBus Async: Transfer timeout in ISR, forcing completion\n");
                LOGE("SMBus Async: Timeout details - msgWriteIdx=%d, msgsNum=%d, rxOutstanding=%d, status=0x%08X\n",
                     dev->msgWriteIdx, dev->msgsNum, dev->rxOutstanding, dev->status);

                dev->status &= ~SMBUS_STATUS_ACTIVE;
                dev->msgErr = -ETIMEDOUT;
                asyncComplete = true;

                /* CRITICAL: Disable ALL interrupts to prevent continuous ISR calls */
                U32 currentMask = regBase->icIntrMask;
                currentMask &= ~(SMBUS_IC_INTR_TX_EMPTY_MASK | SMBUS_IC_INTR_RX_FULL_MASK |
                                SMBUS_IC_INTR_TX_ABRT_MASK | SMBUS_IC_INTR_STOP_DET_MASK |
                                SMBUS_IC_INTR_ACTIVITY_MASK);
                regBase->icIntrMask = currentMask;
                LOGE("SMBus Async: Timeout - Disabled ALL interrupts (mask=0x%08X) to prevent ISR loop\n", currentMask);

                /* CRITICAL: Clear RX FIFO to prevent stale data from triggering more interrupts */
                if (dev->rxOutstanding > 0) {
                    LOGW("SMBus Async: Clearing RX FIFO due to timeout - rxOutstanding=%d\n", dev->rxOutstanding);
                    /* Read and discard RX FIFO data */
                    while (regBase->icRxflr > 0) {
                        volatile U32 dummy = regBase->icDataCmd.value;
                        (void)dummy;
                    }
                    dev->rxOutstanding = 0;
                }
                /* Reset transfer state completely */
                dev->msgWriteIdx = 0;
                dev->msgReadIdx = 0;
                dev->msgsNum = 0;
            }
        }
        /* Always clear handled interrupts in async mode */
        if (intrStat & (SMBUS_IC_INTR_TX_EMPTY_MASK | SMBUS_IC_INTR_RX_FULL_MASK |
                       SMBUS_IC_INTR_TX_ABRT_MASK | SMBUS_IC_INTR_STOP_DET_MASK |
                       SMBUS_IC_INTR_ACTIVITY_MASK)) {
            /* Clear interrupt bits that were handled */
            if (intrStat & SMBUS_IC_INTR_TX_EMPTY_MASK) {
                /* CRITICAL FIX: TX_EMPTY doesn't have a dedicated clear register - it's cleared by writing data */
                /* But we need to ensure it's properly disabled to prevent continuous triggering */
                if (!asyncComplete && (dev->msgWriteIdx >= dev->msgsNum)) {
                    /* Force disable TX_EMPTY if all messages are processed but not marked complete yet */
                    U32 finalMask = regBase->icIntrMask;
                    finalMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                    regBase->icIntrMask = finalMask;
                    LOGD("SMBus Async: TX_EMPTY force-disabled in cleanup - mask=0x%08X\n", finalMask);
                }
                LOGD("SMBus Async: TX_EMPTY interrupt handled\n");
            }
 
            /* CRITICAL FIX: Check for transfer completion status before exiting */
            if (!asyncComplete && !(dev->status & (SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS))) {
                LOGD("SMBus Async: All status bits cleared, marking asyncComplete=true\n");
                asyncComplete = true;
            }

            /* If async transfer is complete, clear interrupts and exit */
            if (asyncComplete) {
                LOGD("SMBus Async: Transfer complete, releasing semaphore before exiting\n");

                /* CRITICAL FIX: Release semaphore to unblock waiting task */
                LOGD("SMBus Async: About to release semaphore - semId=%d\n", dev->semaphoreId);
                if (dev->semaphoreId != (rtems_id)0) {
                    S32 releaseResult = rtems_semaphore_release(dev->semaphoreId);
                    LOGD("SMBus Async: Semaphore released with result=%d (semId=%d)\n",
                         releaseResult, dev->semaphoreId);
                    if (releaseResult != RTEMS_SUCCESSFUL) {
                        LOGE("SMBus Async: Semaphore release failed! result=%d\n", releaseResult);
                    }
                } else {
                    LOGE("SMBus Async: Invalid semaphoreId (0) during completion\n");
                }

                LOGD("SMBus Async: Transfer complete, clearing interrupts and exiting\n");
                goto clear_interrupts;
            }
        }
    }
   }

    /* ========== LEGACY MODE: Original interrupt handling ========== */
    /* ========== TX ABORT ========== */
    if (intrStat & SMBUS_IC_INTR_TX_ABRT_MASK) {
        U32 abrtSource = regBase->icTxAbrtSource.value;

        pDrvData->errorCode |= SMBUS_ERROR_TX_ABORT;
        pDrvData->lastError = SMBUS_ERROR_TX_ABORT;
        pDrvData->errorCount++;

        /* Create ARP failure event */
        SmbusArpFailEvent_s event = {
            .timestamp     = 0,  ///<<TODO: read system timer
            .reason        = (abrtSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_MASK) ?
                                SMBUS_ARP_FAIL_REASON_COLLISION :
                                SMBUS_ARP_FAIL_REASON_TX_ERR,
            .retryCount    = pDrvData->retryCount,
            .txAbrtSource  = abrtSource,
            .rawIntrStat   = intrStat,
        };

        /* Call ARP failure handler if callback is registered */
        if (pDrvData->arpFailHandler) {
            SmbusArpAction_e action = pDrvData->arpFailHandler(devId, &event, pDrvData->arpContext);
        ///<TODO: handle the action (retry, abort, etc.)
            action = action;
        }

        /* Prepare error callback data */
        eventData.error.errorCode = pDrvData->errorCode;
        eventData.error.val = &pDrvData->errorCode;
        needCallback = true;

        /* Disable interrupts and mark transfer as failed */
        regBase->icIntrMask = 0;
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_ACTIVE_MASK;

        LOGE("SMBus Master TX Abort: source=0x%08X, reason=%d\n", abrtSource, event.reason);
    }

    /* ========== RX READY ========== */
    if (intrStat & SMBUS_IC_INTR_RX_FULL_MASK) {
        /* CRITICAL FIX: Only execute Legacy mode when interruptMode = 0 */
        if (pDrvData->sbrCfg.interruptMode == 0) {
            LOGD("SMBus Master Legacy RX Ready - processing\n");
            smbusMasterReadData(pDrvData, regBase);

            /* Prepare RX done callback data */
            //eventData.rxDone.val = pDrvData->rxBuffer;
            eventData.rxDone.len = pDrvData->rxLength;
            needCallback = true;

            LOGD("SMBus Master RX Ready: len=%d\n", pDrvData->rxLength);
        } else {
            /* In async mode, RX_FULL should be handled by the async handler above */
            LOGD("SMBus Master: RX_FULL in async mode - should be handled by async handler\n");
        }
    }
#if 1
    /* ========== TX READY ========== */
    if (intrStat & SMBUS_IC_INTR_TX_EMPTY_MASK) {
        smbusMasterTransferData(pDrvData, regBase);

        /* Check if transfer is complete and handle completion */
        /* CRITICAL: Don't complete transfer if async operations are still active */
        if (pDrvData->txComplete &&
            !(pDrvData->pSmbusDev.status & (SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS))) {
            /* Transfer complete, disable TX_EMPTY interrupt */
            U32 currentMask = regBase->icIntrMask;
            currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            regBase->icIntrMask = currentMask;
            LOGD("SMBus Master: TX_EMPTY disabled due to transfer completion\n");

            /* Clear device active status and release semaphore */
            LOGD("SMBus Master: Transfer completed, releasing semaphore (semId=%d)\n",
                 pDrvData->pSmbusDev.semaphoreId);

            if (pDrvData->pSmbusDev.semaphoreId != (rtems_id)0) {
                S32 releaseResult = rtems_semaphore_release(pDrvData->pSmbusDev.semaphoreId);
                LOGD("SMBus Master: Semaphore release result=%d\n", releaseResult);
            } else {
                LOGE("SMBus Master: Invalid semaphoreId (0)\n");
            }
        }
        LOGD("SMBus Master TX Ready\n");
    }
#endif
    /* ========== SMBus Specific Interrupts (Master) ========== */
    if (smbusIntrStat) {
        smbusHandleMasterSpecificInterrupts(pDrvData, regBase, smbusIntrStat);
    }

    /* Trigger unified callback if needed */
    if (needCallback && pDrvData->callback.cb) {
        U32 eventType = (pDrvData->errorCode != 0) ? SMBUS_EVENT_ERROR : SMBUS_EVENT_RX_DONE;
        pDrvData->callback.cb(devId, eventType, &eventData);
    }

  clear_interrupts:
    /* Clear interrupt status */
    (void)regBase->icClrIntr;  /* Read to clear interrupt status */
    (void)regBase->icClrTxAbrt;  /* Read to clear TX abort */
/* Clear all interrupt status bits using dummy read approach */
    volatile U32 dummy;
    dummy = regBase->icClrIntr;        /* Dummy read to clear combined interrupt */
    dummy = regBase->icClrTxAbrt;      /* Dummy read to clear TX abort */
    dummy = regBase->icClrRxUnder;    /* Dummy read to clear RX underflow */
    dummy = regBase->icClrRxOver;     /* Dummy read to clear RX overflow */
    dummy = regBase->icClrTxOver;     /* Dummy read to clear TX overflow */
    dummy = regBase->icClrRdReq;      /* Dummy read to clear read request */
    dummy = regBase->icClrRxDone;     /* Dummy read to clear RX done */
    dummy = regBase->icClrActivity;   /* Dummy read to clear activity */
    dummy = regBase->icClrStopDet;    /* Dummy read to clear stop detection */
    dummy = regBase->icClrStartDet;   /* Dummy read to clear start detection */
    dummy = regBase->icClrGenCall;    /* Dummy read to clear general call */
    (void)dummy; /* Suppress unused variable warning */
}

/* ======================================================================== */
/*                     ARP Helper Functions                              */
/* ======================================================================== */

/**
 * @brief Clear and free all discovered devices in ARP master device list
 * @details This function safely deallocates all memory associated with the
 *          device list maintained by the ARP master. It iterates through the
 *          linked list of discovered devices, frees each node, and resets the
 *          ARP master state to its initial condition.
 * @param[in,out] master Pointer to the ARP master structure to be cleaned up
 * @return void
 *
 * @note This function should be called during ARP master deinitialization
 *       or when resetting the ARP discovery process
 * @note All memory allocated for device nodes will be freed
 * @warning After calling this function, all previously discovered devices
 *          will be lost and must be rediscovered
 * @warning The function is not thread-safe and should be called with
 *          appropriate synchronization if accessed from multiple threads
 */
void smbusClearDevs(SmbusArpMaster_s *master)
{
    SmbusArpDeviceNode_s *nodeList = NULL;
    SmbusArpDeviceNode_s *next = NULL;

    if (master == NULL) {
        return;
    }

    nodeList = master->deviceList;
    while (nodeList != NULL) {
        next = nodeList->next;
        free(nodeList);
        nodeList = next;
    }

    master->deviceList = NULL;
    master->deviceCount = 0;
    master->nextAddress = master->addressPoolStart;
}

/**
 * @brief Check if address is already used in ARP master device list
 * @param[in] master Pointer to ARP master structure
 * @param[in] address Address to check
 * @return TRUE if address is used, FALSE otherwise
 */
Bool smbusArpIsAddrUsed(SmbusArpMaster_s *master, U8 address)
{
    SmbusArpDeviceNode_s *node = NULL;

    if (master == NULL) {
        return TRUE;
    }

    node = master->deviceList;
    while (node != NULL) {
        if (node->currentAddress == address) {
            return TRUE;
        }
        node = node->next;
    }

    return FALSE;
}


/* ======================================================================== */
/*                    Core ARP Failure Handler                               */
/* ======================================================================== */

/**
 * @brief Handle SMBus ARP operation failures
 * @param devId Device identifier
 * @param event Pointer to ARP failure event structure
 * @param userData User context data
 * @return ARP action to take
 */
__attribute((unused)) static SmbusArpAction_e smbusArpFailHandler(DevList_e devId,
                                            const SmbusArpFailEvent_s *event,
                                            void *userData)
{
    SmbusArpContext_s *context = NULL;
    SmbusArpAction_e action = SMBUS_ARP_ACTION_NONE;

    /* 参数有效性检查 */
    if (event == NULL) {
        return SMBUS_ARP_ACTION_ABORT;
    }

    /* 获取上下文 */
    if (userData != NULL) {
        context = (SmbusArpContext_s *)userData;
    }

    /* 根据失败原因确定处理动作 */
    switch (event->reason) {
        case SMBUS_ARP_FAIL_REASON_COLLISION:
            /* SDA冲突 - 说明有其他设备同时访问 */
            if (context && context->multiMasterMode) {
                /* 多主模式下，使用指数退避重试 */
                action = SMBUS_ARP_ACTION_BACKOFF_RETRY;
            } else {
                /* 单主模式下的冲突异常，上报 */
                action = SMBUS_ARP_ACTION_ESCALATE;
            }
            break;

        case SMBUS_ARP_FAIL_REASON_START_LOST:
        case SMBUS_ARP_FAIL_REASON_DATA_LOST:
            /* START条件或数据仲裁丢失 - 总线竞争失败 */
            if (context && event->retryCount >= context->maxRetries) {
                /* 超过最大重试次数 */
                action = SMBUS_ARP_ACTION_SWITCH_SLAVE;
            } else {
                /* 退避后重试 */
                action = SMBUS_ARP_ACTION_BACKOFF_RETRY;
            }
            break;

        case SMBUS_ARP_FAIL_REASON_STOP_LOST:
            /* STOP条件丢失 - 可能总线异常 */
            if (event->retryCount >= SMBUS_ARP_MAX_RETRIES) {
                action = SMBUS_ARP_ACTION_ESCALATE;
            } else {
                action = SMBUS_ARP_ACTION_RETRY;
            }
            break;

        case SMBUS_ARP_FAIL_REASON_ADDR_NACK:
            /* 地址未应答 - 目标设备不存在或未响应 */
            if (event->retryCount >= SMBUS_ARP_MAX_RETRIES) {
                /* 多次重试失败，放弃 */
                action = SMBUS_ARP_ACTION_ABORT;
            } else {
                /* 简单重试 */
                action = SMBUS_ARP_ACTION_RETRY;
            }
            break;

        case SMBUS_ARP_FAIL_REASON_MULTI_MASTER:
            /* 多主冲突 */
            if (context) {
                context->multiMasterMode = true;
                if (context->consecutiveFailures >= SMBUS_ARP_CONSECUTIVE_FAIL_TH) {
                    /* 连续失败过多，切换为从机 */
                    action = SMBUS_ARP_ACTION_SWITCH_SLAVE;
                } else {
                    /* 使用退避策略重试 */
                    action = SMBUS_ARP_ACTION_BACKOFF_RETRY;
                }
            } else {
                action = SMBUS_ARP_ACTION_BACKOFF_RETRY;
            }
            break;

        case SMBUS_ARP_FAIL_REASON_BUS_BUSY:
            /* 总线忙 */
            if (event->timestamp - (context ? context->lastFailTime : 0)
                > SMBUS_ARP_BUS_BUSY_TIMEOUT_MS) {
                /* 总线长时间占用，可能异常 */
                action = SMBUS_ARP_ACTION_ESCALATE;
            } else {
                /* 等待总线空闲后重试 */
                action = SMBUS_ARP_ACTION_WAIT_NOTIFY;
            }
            break;

        case SMBUS_ARP_FAIL_REASON_UNKNOWN:
        default:
            /* 未知原因 */
            if (event->retryCount >= SMBUS_ARP_MAX_RETRIES) {
                action = SMBUS_ARP_ACTION_ESCALATE;
            } else {
                action = SMBUS_ARP_ACTION_RETRY;
            }
            break;
    }

    /* 更新上下文统计信息 */
    if (context != NULL) {
        context->lastFailTime = event->timestamp;

        /* 更新连续失败计数 */
        if (action == SMBUS_ARP_ACTION_ABORT ||
            action == SMBUS_ARP_ACTION_ESCALATE) {
            context->consecutiveFailures++;
        }

        /* 计算退避时间（指数退避） */
        if (action == SMBUS_ARP_ACTION_BACKOFF_RETRY) {
            uint32_t backoff = SMBUS_ARP_BASE_BACKOFF_MS << event->retryCount;
            if (backoff > SMBUS_ARP_MAX_BACKOFF_MS) {
                backoff = SMBUS_ARP_MAX_BACKOFF_MS;
            }
            context->backoffTimeMs = backoff;
        }
    }

    LOGE("arp failure handler\r\n");
    return action;
}

/**
 * @brief Allocate next available address from ARP master address pool
 * @param master Pointer to ARP master structure
 * @return Allocated address, or 0 if no addresses available
 * [CORE] Core protocol layer - ARP address allocation
 */
static U8 smbusArpAllocAddr(SmbusArpMaster_s *master)
{
    U8 addr;

    if (master == NULL) {
        return 0;
    }

    for (addr = master->nextAddress; addr <= master->addressPoolEnd; addr++) {
        if (!smbusArpIsAddrUsed(master, addr)) {
            master->nextAddress = addr + 1;
            return addr;
        }
    }

    return 0;  /* No available addresses */
}

/* ======================================================================== */
/*                    Core ARP Context Functions                              */
/* ======================================================================== */

/**
 * @brief Update address mapping in ARP master device list
 * @details Updates or creates an entry in the ARP master device list for the
 *          specified UDID with the new address. If the device already exists,
 *          its address is updated. If it doesn't exist, a new entry is created.
 * @param[in,out] master Pointer to the ARP master structure
 * @param[in] udid Pointer to the device UDID structure
 * @param[in] newAddr New address to assign to the device
 * @return 0 on success, -ENOMEM on memory allocation failure
 *
 * @note This function maintains the linked list of discovered devices
 * @note Updates deviceCount and nextAddress in master structure
 * @note Sets device flag to active after successful update
 * @warning The function does not check for address conflicts
 * @warning Caller must ensure newAddr is within valid address range
 *
 * [CORE] Core protocol layer - ARP device mapping management
 */
__attribute((unused)) static S32 smbusUpdateAddrMap(SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 newAddr)
{
    SmbusArpDeviceNode_s *node = master->deviceList;
    SmbusArpDeviceNode_s *prev = NULL;

    /* 1. find if is udid */
    while (node) {
        if (memcmp(&node->udid, udid, sizeof(SmbusUdid_s)) == 0) {
            /* 2. found device */
            node->currentAddress = newAddr;
            node->flags |= SMBUS_DEVICE_FLAG_ACTIVE;  ///<<update flag
            return 0;
        }
        prev = node;
        node = node->next;
    }

    /* 3. not found next node */
    node = (SmbusArpDeviceNode_s *)malloc(sizeof(SmbusArpDeviceNode_s));
    if (!node)
        return -ENOMEM;

    memset(node, 0, sizeof(SmbusArpDeviceNode_s));
    node->currentAddress = newAddr;
    memcpy(&node->udid, udid, sizeof(SmbusUdid_s));
    node->flags = SMBUS_DEVICE_FLAG_NONE;
    node->next = NULL;

    /* 4. add linker tail */
    if (prev)
        prev->next = node;
    else
        master->deviceList = node;

    /* 5. update info  */
    master->deviceCount++;
    master->nextAddress = (newAddr + 1 <= master->addressPoolEnd) ? newAddr + 1 : master->addressPoolStart;

    return 0;
}

/**
 * @brief Compare two UDIDs for equality
 * @param[in] udid1 First UDID
 * @param[in] udid2 Second UDID
 * @return 1 if equal, 0 if different
 */
Bool smbusUdidCompare(const SmbusUdid_s *udid1, const SmbusUdid_s *udid2)
{
    if (udid1 == NULL || udid2 == NULL) {
        return 0;
    }

    /* Compare key fields that uniquely identify a device */
    if (udid1->vendorId != udid2->vendorId) {
        return false;
    }
    if (udid1->deviceId != udid2->deviceId) {
        return false;
    }
    if (udid1->interface != udid2->interface) {
        return false;
    }
    if (udid1->subsystemVendorId != udid2->subsystemVendorId) {
        return false;
    }

    /* Compare subsystem address bytes */
    if (memcmp(udid1->bytes, udid2->bytes, sizeof(udid1->bytes)) != 0) {
        return false;
    }

    return true; /* UDIDs match */
}

/* ======================================================================== */
/*                    Core ARP Device Management Functions                     */
/* ======================================================================== */

/**
 * @brief Install ARP device in master device list
 * @details Adds a new device to the ARP master's device list with the specified
 *          UDID and address. This function performs parameter validation and
 *          conflict checks before adding the device.
 * @param[in,out] master Pointer to the ARP master structure
 * @param[in] udid Pointer to the device UDID structure
 * @param[in] address Address to assign to the device
 * @return 0 on success, negative error code on failure:
 *         -1: Invalid parameters
 *         -2: Address out of range or reserved
 *         -3: Device already exists
 *         -4: Address conflict with existing device
 *         -ENOMEM: Memory allocation failure
 *
 * @note Validates address range and pool boundaries
 * @note Checks for address conflicts with existing devices
 * @note Updates master device count and next available address
 * @warning This function is not thread-safe
 * @warning Caller must ensure master pointer is valid
 *
 */
S32 ArpDevInstall(SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 address)
{
    SmbusArpDeviceNode_s *newNode = NULL;
    SmbusArpDeviceNode_s *current = NULL;

    /* Validate input parameters */
    if (master == NULL || udid == NULL) {
        return -EINVAL; /* Invalid parameter */
    }

    /* Check if address is within the valid pool range */
    if (address < master->addressPoolStart || address > master->addressPoolEnd) {
        return -ERANGE; /* Address out of range */
    }

    /* Check if address is reserved (0x00-0x07 are typically reserved in SMBus) */
    if (address < SMBUS_MIN_VALID_ADDRESS) {
        return -ERANGE; /* Reserved address */
    }

    /* Check if device with same UDID already exists */
    current = master->deviceList;
    while (current != NULL) {
        if (smbusUdidCompare(&current->udid, udid)) {
            /* Device already exists */
            return -EEXIST;
        }
        current = current->next;
    }

    /* Check if address is already used */
    if (smbusArpIsAddrUsed(master, address)) {
        return -EADDRINUSE; /* Address conflict */
    }

    /* Allocate new device node */
    newNode = (SmbusArpDeviceNode_s *)malloc(sizeof(SmbusArpDeviceNode_s));
    if (newNode == NULL) {
        return -ENOMEM;
    }

    /* Initialize new node */
    memset(newNode, 0, sizeof(SmbusArpDeviceNode_s));
    newNode->currentAddress = address;
    memcpy(&newNode->udid, udid, sizeof(SmbusUdid_s));
    newNode->flags = SMBUS_DEVICE_FLAG_ACTIVE;

    /* Add to device list */
    newNode->next = master->deviceList;
    master->deviceList = newNode;
    master->deviceCount++;

    return 0;
}

/**
 * @brief Initialize ARP context structure
 * @details Initializes an ARP context structure with default values for
 *          retry limits, backoff timing, and failure tracking. This function
 *          should be called before using the context in ARP operations.
 * @param[in,out] context Pointer to the ARP context structure to initialize
 * @return void
 *
 * @note Sets maxRetries to SMBUS_ARP_MAX_RETRIES
 * @note Sets backoffTimeMs to SMBUS_ARP_BASE_BACKOFF_MS
 * @note Initializes all counters and flags to default values
 * @warning The function does nothing if context pointer is NULL
 *
 * @par Example Usage:
 * @code
 * SmbusArpContext_s arpContext;
 * smbusArpContextInit(&arpContext);
 * ///<<Now arpContext is ready for use in ARP operations
 * @endcode
 *
 * [CORE] Core protocol layer - ARP context management
 */
static void smbusArpContextInit(SmbusArpContext_s *context)
{
    if (context == NULL) {
        return;
    }

    context->maxRetries = SMBUS_ARP_MAX_RETRIES;
    context->backoffTimeMs = SMBUS_ARP_BASE_BACKOFF_MS;
    context->lastFailTime = 0;
    context->consecutiveFailures = 0;
    context->multiMasterMode = false;
}

/**
 * @brief Auto-install ARP device with automatic address allocation
 * @details Automatically installs an ARP device by finding an available address
 *          in the master's address pool. The function first tries to use the
 *          suggested address from the UDID, then allocates a new address if needed.
 * @param[in,out] master Pointer to the ARP master structure
 * @param[in] udid Pointer to the device UDID structure
 * @param[out] assignedAddress Pointer to store the assigned address
 * @return 0 on success, negative error code on failure
 *
 * @note Returns success if device already exists (no re-installation)
 * @note Updates master's next available address after successful allocation
 * @note Performs automatic address conflict detection and resolution
 * @warning This function is not thread-safe
 * @warning Caller must ensure all pointers are valid
 *
 * [CORE] Core protocol layer - ARP device management
 */
__attribute((unused)) static int ArpDevInstallAuto(SmbusArpMaster_s *master, const SmbusUdid_s *udid,
                                                        U8 *assignedAddress)
{
    U8 address;
    SmbusArpDeviceNode_s *current = NULL;

    if (master == NULL || udid == NULL || assignedAddress == NULL) {
        return -EINVAL;
    }

    /* Check if device already exists */
    current = master->deviceList;
    while (current != NULL) {
        if (smbusUdidCompare(&current->udid, udid)) {
            *assignedAddress = current->currentAddress;
            return 0; /* Already installed */
        }
        current = current->next;
    }

    /* Try to use the suggested address from UDID first */
    if (udid->nextAvailAddr >= master->addressPoolStart &&
        udid->nextAvailAddr <= master->addressPoolEnd &&
        udid->nextAvailAddr >= SMBUS_MIN_VALID_ADDRESS) {
        if (!smbusArpIsAddrUsed(master, udid->nextAvailAddr)) {
            address = udid->nextAvailAddr;
            return ArpDevInstall(master, udid, address);
        }
    }

    /* Allocate a new address automatically */
    address = smbusArpAllocAddr(master);
    if (address == 0) {
        return -ENOSPC; /* No available addresses */
    }

    /* Install with allocated address */
    *assignedAddress = address;
    return ArpDevInstall(master, udid, address);
}

/**
 * @brief Find ARP device by address
 * @details Searches the ARP master's device list for a device with the specified
 *          address. This function performs a linear search through the device list.
 * @param[in] master Pointer to the ARP master structure
 * @param[in] address Device address to search for
 * @return Pointer to device node if found, NULL otherwise
 *
 * @note Performs linear search through device list
 * @note Compares currentAddress field of each device node
 * @warning This function is not thread-safe
 * @warning Caller must ensure master pointer is valid
 *
 * [CORE] Core protocol layer - ARP device management
 */
__attribute((unused)) static SmbusArpDeviceNode_s* ArpDevFindByAddr(SmbusArpMaster_s *master, U8 address)
{
    SmbusArpDeviceNode_s *current = NULL;

    if (master == NULL) {
        return NULL;
    }

    current = master->deviceList;
    while (current != NULL) {
        if (current->currentAddress == address) {
            return current;
        }
        current = current->next;
    }

    return NULL;
}

/**
 * @brief Remove ARP device from device list
 * @details Removes a device from the ARP master's device list based on UDID.
 *          The function searches for the device, removes it from the linked list,
 *          and frees the allocated memory.
 * @param[in,out] master Pointer to the ARP master structure
 * @param[in] udid Pointer to the device's UDID to remove
 * @return 0 on success, -1 if device not found
 *
 * @note Frees memory allocated for the device node
 * @note Updates master's device count
 * @note Handles both head and middle node removal cases
 * @warning This function is not thread-safe
 * @warning Caller must ensure pointers are valid
 *
 * [CORE] Core protocol layer - ARP device management
 */
__attribute((unused)) static S32 ArpDevRemove(SmbusArpMaster_s *master, const SmbusUdid_s *udid)
{
    SmbusArpDeviceNode_s *current = NULL;
    SmbusArpDeviceNode_s *prev = NULL;

    if (master == NULL || udid == NULL) {
        return -EINVAL;
    }

    /* Search for the device to remove */
    current = master->deviceList;
    while (current != NULL) {
        if (smbusUdidCompare(&current->udid, udid)) {
            /* Found device, remove it */
            if (prev == NULL) {
                /* Remove head node */
                master->deviceList = current->next;
            } else {
                /* Remove middle or tail node */
                prev->next = current->next;
            }

            /* Free memory and update count */
            free(current);
            master->deviceCount--;
            return 0;
        }
        prev = current;
        current = current->next;
    }

    /* Device not found */
    return -ENOENT;
}

/**
 * @brief Initialize ARP Master context
 * @details Initializes ARP master configuration and context including
 *          retry parameters, address pool management, and failure handling.
 *          This function modularizes the ARP master initialization logic.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Initializes both ARP configuration and master context
 * @note Allocates memory for ARP failure handler context
 * @note Sets up default address pool for dynamic addressing
 * [CORE] Core protocol layer - ARP initialization
 */
static S32 smbusInitializeArpMaster(SmbusDrvData_s *pDrvData)
{
    if (pDrvData == NULL) {
        return -EINVAL;
    }

    /* Initialize ARP configuration with defaults */
    pDrvData->arpConfig.maxRetries = 3;
    pDrvData->arpConfig.retryStrategy = SMBUS_ARP_STRATEGY_IMMEDIATE_RETRY;
    pDrvData->arpConfig.defaultPriority = SMBUS_ARP_PRIORITY_NORMAL;
    pDrvData->arpConfig.autoSwitchToSlave = 0;
    pDrvData->arpConfig.retryDelayMin = 1;
    pDrvData->arpConfig.retryDelayMax = 100;
    pDrvData->arpConfig.backoffMultiplier = 2;
    pDrvData->arpConfig.failureThreshold = 5;
    pDrvData->arpConfig.busyWaitTimeout = 1000;
    pDrvData->arpState = SMBUS_ARP_STATE_IDLE;
    pDrvData->retryCount = 0;

    /* Initialize ARP Master context */
    pDrvData->arpMaster.busId = pDrvData->devId;
    pDrvData->arpMaster.nextAddress = SMBUS_MIN_DYNAMIC_ADDRESS;
    pDrvData->arpMaster.addressPoolStart = SMBUS_MIN_DYNAMIC_ADDRESS;
    pDrvData->arpMaster.addressPoolEnd = SMBUS_MAX_VALID_ADDRESS;
    pDrvData->arpMaster.deviceList = NULL;
    pDrvData->arpMaster.deviceCount = 0;
    pDrvData->arpMaster.hwContext = NULL;

    /* Initialize ARP failure handler context */
    SmbusArpContext_s *context = calloc(1, sizeof(SmbusArpContext_s));
    if (context != NULL) {
        smbusArpContextInit(context);
        pDrvData->arpFailParam = (void *)context;
    }

    LOGD("%s: ARP Master initialized successfully\n", __func__);
    return EXIT_SUCCESS;
}

/**
 * @brief Free driver data structure and associated memory
 * @details Frees all allocated memory including driver data structure,
 *          slave buffers, and ARP context.
 * @param[in] pDrvData Pointer to driver data structure
 * @return void
 *
 * @note Safe to call with NULL pointer
 * @note Handles both master and slave mode cleanup
 * [CORE] Core protocol layer - memory management
 */
static void smbusFreeDriverData(SmbusDrvData_s *pDrvData)
{
    if (pDrvData == NULL) {
        return;
    }
    smbusClearDevs(&pDrvData->arpMaster);

    /* CRITICAL FIX: Free master RX buffer if allocated */
    if (pDrvData->rxBuffer != NULL) {
        free(pDrvData->rxBuffer);
        pDrvData->rxBuffer = NULL;
        pDrvData->rxLength = 0;
        pDrvData->rxBufferSize = 0;
        LOGD("%s: Master RX buffer freed\n", __func__);
    }

    /* Free slave buffers if allocated */
    if (pDrvData->pSmbusDev.slaveRxBuf != NULL) {
        free(pDrvData->pSmbusDev.slaveRxBuf);
        pDrvData->pSmbusDev.slaveRxBuf = NULL;
        pDrvData->pSmbusDev.slaveValidRxLen = 0;
    }

    if (pDrvData->pSmbusDev.slaveTxBuf != NULL) {
        free(pDrvData->pSmbusDev.slaveTxBuf);
        pDrvData->pSmbusDev.slaveTxBuf = NULL;
        pDrvData->pSmbusDev.slaveValidTxLen = 0;
        pDrvData->slaveTxIndex = 0;
    }

    /* Clear master TX buffer array - no need to free as it's a static array */
    memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
    pDrvData->pSmbusDev.masterTxBufLen = 0;
    LOGD("%s: Master TX buffer array cleared\n", __func__);
    /* Free ARP context if allocated */
    SmbusArpContext_s *context = (SmbusArpContext_s *)pDrvData->arpFailParam;
    if (context != NULL) {
        free(context);
        pDrvData->arpFailParam = NULL;
    }
}

/**
 * @brief Handle slave mode interrupts (integrated from i2c_dw_isr_slave)
 * @details Processes slave mode interrupt events including read/write requests,
 *          stop detection, and SMBus-specific slave events.
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] intrStat I2C interrupt status
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 *
 * @note Integrates I2C slave ISR logic with SMBus protocol handling
 * @note Uses SMBus callback functions for event notification
 * [CORE] Core protocol layer - slave interrupt handling
 */
static void smbusHandleSlaveInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                      U32 intrStat, U32 smbusIntrStat)
{
    U32 rawStat, rxValid, tmp;
    U8 val = 0, slaveActivity;
    U32 readCount = 0;

    /* CRITICAL FIX: Comprehensive parameter validation to prevent core dump */
    if (pDrvData == NULL) {
        LOGE("SMBus Slave: NULL driver data - cannot handle interrupt\n");
        return;
    }

    if (regBase == NULL) {
        LOGE("SMBus Slave: NULL register base - cannot handle interrupt\n");
        return;
    }
    /* Validate device is in slave mode */
    if (pDrvData->pSmbusDev.mode != DW_SMBUS_MODE_SLAVE) {
        LOGW("SMBus Slave: Device not in slave mode (mode=%d) - handling anyway\n",
             pDrvData->pSmbusDev.mode);
        /* Continue processing but log warning */
    }

    /* CRITICAL FIX: Validate hardware register accessibility before accessing */
    /* Try to read a harmless register first to detect hardware issues */
    volatile U32 registerTest = regBase->icEnable.value;
    if (registerTest == 0xFFFFFFFFU) {
        LOGE("SMBus Slave: Hardware registers not accessible (read=0x%08X) - aborting\n", registerTest);
        return;
    }

    /* Read additional status information */
    rawStat = regBase->icRawIntrStat.value;
    tmp = regBase->icStatus.value;
    slaveActivity = (tmp & SMBUS_IC_STATUS_SLAVE_ACTIVITY_MASK) >> SMBUS_IC_STATUS_SLAVE_ACTIVITY_SHIFT;
    rxValid = regBase->icRxflr;

    LOGD("SMBus Slave: activity=%d, raw_intr=0x%08X, rx_valid=%u\n",
         slaveActivity, rawStat, rxValid);

    /* CRITICAL FIX: Read interrupt status BEFORE clearing for proper processing */
    intrStat = regBase->icIntrStat.value;
    LOGD("SMBus Slave: interrupt status before processing = 0x%08X\n", intrStat);

    /* ========== RX FULL (Write from Master) ========== */
    if (intrStat & SMBUS_IC_INTR_RX_FULL_MASK) {
        LOGE("SMBus Slave: RX_FULL interrupt detected! intrStat=0x%08X, rx_valid=%u\n", intrStat, rxValid);

        if (!(pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK)) {
            pDrvData->pSmbusDev.status |= SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
            pDrvData->slaveValidRxLen = 0;  ///<<Reset RX buffer for new transfer

            /* Trigger write request callback */
            smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_WRITE_REQ, NULL, 0);
        }

        /* Read data from FIFO */
        U32 maxReadCount = rxValid;  /* Limit reads to initial FIFO level to prevent infinite loop */

        /* CRITICAL FIX: Additional safety checks for buffer operations */
        if (pDrvData->pSmbusDev.slaveRxBuf == NULL) {
            LOGE("SMBus Slave: NULL RX buffer pointer - cannot read data\n");
            return;
        }

        /* Prevent excessive read counts that could cause system hang */
        if (maxReadCount > SMBUS_SLAVE_BUF_LEN) {
            LOGW("SMBus Slave: Limiting read count from %u to %u\n",
                 maxReadCount, SMBUS_SLAVE_BUF_LEN);
            maxReadCount = SMBUS_SLAVE_BUF_LEN;
        }

        while (readCount < maxReadCount) {
            tmp = regBase->icDataCmd.value;
            if (tmp & SMBUS_IC_DATA_CMD_FIRST_DATA_BYTE_MASK) {
                /* First data byte received */
                smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_WRITE_REQ, NULL, 0);
            }

            /* CRITICAL FIX: Extract data byte from tmp register */
            val = (U8)tmp;

            /* Store data in slave RX buffer with enhanced bounds checking */
            if (pDrvData->slaveValidRxLen < SMBUS_SLAVE_BUF_LEN) {
                pDrvData->pSmbusDev.slaveRxBuf[pDrvData->slaveValidRxLen++] = val;
            } else {
                LOGE("SMBus Slave RX buffer overflow (len=%d, max=%d) - dropping data\n",
                     pDrvData->slaveValidRxLen, SMBUS_SLAVE_BUF_LEN);
                break;  /* Exit loop on buffer overflow */
            }

            readCount++;

            /* Check if more data available */
            tmp = regBase->icStatus.value;
            if (!(tmp & SMBUS_IC_STATUS_RFNE_MASK)) {
                break;  /* No more data in FIFO */
            }
        }
    }

    LOGE("SMBus Slave: RX_FULL processing completed. readCount=%u, slaveValidRxLen=%u\n",
         readCount, pDrvData->slaveValidRxLen);

    /* CRITICAL DEBUG: Print received slave RX buffer data */
    if (pDrvData->slaveValidRxLen > 0 && pDrvData->pSmbusDev.slaveRxBuf != NULL) {
        LOGE("SMBus Slave: Received data in slaveRxBuf (%u bytes): ", pDrvData->slaveValidRxLen);
        U32 printCount = pDrvData->slaveValidRxLen;
        if (printCount > 16) printCount = 16;  /* Limit print to first 16 bytes */

        for (U32 i = 0; i < printCount; i++) {
            LOGE("0x%02X ", pDrvData->pSmbusDev.slaveRxBuf[i]);
        }
        if (pDrvData->slaveValidRxLen > 16) {
            LOGE("... (%u more bytes)", pDrvData->slaveValidRxLen - 16);
        }
        LOGE("\n");

        /* CRITICAL FIX: Immediately sync slaveValidRxLen to device structure */
        /* This is needed because STOP/RESTART detection may not trigger */
        pDrvData->pSmbusDev.slaveValidRxLen = pDrvData->slaveValidRxLen;
        LOGE("SMBus Slave: Synced slaveValidRxLen -> dev->slaveValidRxLen (%u)\n",
             pDrvData->pSmbusDev.slaveValidRxLen);

    } else {
        LOGE("SMBus Slave: No valid data in slaveRxBuf (len=%d, buf=%p)\n",
             pDrvData->slaveValidRxLen, pDrvData->pSmbusDev.slaveRxBuf);
    }

    /* ========== STOP/RESTART Detection ========== */
    if ((intrStat & SMBUS_IC_INTR_STOP_DET_MASK) || (intrStat & SMBUS_IC_INTR_RESTART_DET_MASK)) {
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_READ_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
        }
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
        }

        /* Update valid RX buffer length */
        pDrvData->pSmbusDev.slaveValidRxLen = pDrvData->slaveValidRxLen;

        /* Trigger slave done callback */
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_DONE, &val, 1);
        pDrvData->slaveTransferActive = 0;

        LOGD("SMBus Slave STOP detected\n");
    }

    /* ========== RD REQUEST (Read from Master) ========== */
    if (intrStat & SMBUS_IC_INTR_RD_REQ_MASK) {
        LOGD("SMBus Slave: RD_REQ triggered\n");

        /* 清除 RD_REQ */
        smbusReadClearIntrBitsMasked(pDrvData, SMBUS_IC_INTR_RD_REQ_MASK);

        /* 获取数据 */
        U8 txData = 0xA1;
        if (pDrvData->pSmbusDev.slaveTxBuf != NULL &&
            pDrvData->pSmbusDev.slaveValidTxLen > 0) {
            static U32 txIndex = 0;
            txData = pDrvData->pSmbusDev.slaveTxBuf[txIndex % pDrvData->pSmbusDev.slaveValidTxLen];
            txIndex++;
        }

        LOGD("SMBus Slave: Preparing to send 0x%02X\n", txData);

        /* ⚠️ 关键修改：正确写入 TX FIFO */
        /* 方法 1：直接写 value（推荐） */
        regBase->icDataCmd.value = (U32)txData;

        /* 方法 2：如果上面不行，使用指针 */
        // volatile U32 *pDataCmd = (volatile U32 *)((U32)regBase + 0x10);
        // *pDataCmd = (U32)txData;

        /* ⚠️ 添加内存屏障确保写入生效 */
        __asm__ volatile("" ::: "memory");

        /* 读回验证 */
        U32 txflr = regBase->icTxflr;
        U32 status = regBase->icStatus.value;

        LOGD("SMBus Slave: After TX write - TXFLR=%d, STATUS=0x%08X, TFE=%d\n",
             txflr, status, regBase->icStatus.fields.tfe);

        /* 验证写入是否成功 */
        if (txflr == 0) {
            LOGE("!!! TX FIFO still empty after write!\n");
            /* 尝试再写一次 */
            regBase->icDataCmd.value = (U32)txData;
            LOGD("SMBus Slave: Retry write, TXFLR=%d\n", regBase->icTxflr);
        } else {
            LOGI("✓ TX FIFO write success, level=%d\n", txflr);
        }
    }

    /* ========== TX_EMPTY (Slave mode handling) ========== */
    if (intrStat & SMBUS_IC_INTR_TX_EMPTY_MASK) {
        LOGE("SMBus Slave: TX_EMPTY interrupt detected! status=0x%08X\n", intrStat);

        /* In Slave mode, TX_EMPTY should only be enabled during actual data transmission */
        /* If we get TX_EMPTY without an active read operation, disable it to prevent storm */
        if (!(pDrvData->pSmbusDev.status & SMBUS_STATUS_READ_IN_PROGRESS_MASK)) {
            /* No active read operation - disable TX_EMPTY interrupt */
            U32 currentMask = regBase->icIntrMask;
            currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            regBase->icIntrMask = currentMask;
            LOGE("SMBus Slave: Disabled spurious TX_EMPTY interrupt (mask=0x%08X)\n", currentMask);
        } else {
            /* CRITICAL FIX: Active read operation - must fill TX FIFO immediately */
            LOGE("SMBus Slave: TX_EMPTY during read operation - filling FIFO\n");

            /* Check slave activity status */
            SmbusIcStatusReg_u status;
            status.value = regBase->icStatus.value;
            U32 slaveActivity = (status.value & SMBUS_IC_STATUS_SLAVE_ACTIVITY_MASK) >> SMBUS_IC_STATUS_SLAVE_ACTIVITY_SHIFT;
            U32 txFifoNotFull = status.fields.tfnf;

            LOGE("SMBus Slave: TX_EMPTY status - activity=%d, TFNF=%d\n", slaveActivity, txFifoNotFull);

            if (slaveActivity == 1 && txFifoNotFull == 1) {
                /* Fill TX FIFO with data */
                U8 txData = 0xFF;  // Default response

                /* Try to get data from callback buffer */
                if (pDrvData->pSmbusDev.slaveTxBuf && pDrvData->slaveTxIndex < pDrvData->pSmbusDev.slaveValidTxLen) {
                    txData = pDrvData->pSmbusDev.slaveTxBuf[pDrvData->slaveTxIndex++];
                    LOGE("SMBus Slave: TX_EMPTY filled with buffer data[0x%02X] (idx=%d)\n", txData, pDrvData->slaveTxIndex-1);
                } else {
                    LOGE("SMBus Slave: TX_EMPTY filled with default data[0x%02X]\n", txData);
                }

                /* Write data to TX FIFO */
                regBase->icDataCmd.value = (U32)txData;
                LOGE("SMBus Slave: TX_EMPTY data written to FIFO - 0x%02X\n", txData);
            } else if (slaveActivity == 0) {
                /* No more slave activity - disable TX_EMPTY and end read operation */
                U32 currentMask = regBase->icIntrMask;
                currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                regBase->icIntrMask = currentMask;

                /* Clear read status */
                pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
                pDrvData->slaveTransferActive = 0;

                LOGE("SMBus Slave: Read operation completed - TX_EMPTY disabled (mask=0x%08X)\n", currentMask);
            } else {
                /* FIFO is full - just wait */
                LOGE("SMBus Slave: TX FIFO full, waiting for space\n");
            }
        }
    }

    /* ========== SMBus Specific Interrupts (Slave) ========== */
    if (smbusIntrStat) {
        smbusHandleSlaveSpecificInterrupts(pDrvData, regBase, smbusIntrStat);
    }

    /* CRITICAL FIX: Use comprehensive interrupt clearing to prevent storm */
    if (intrStat != 0 || rawStat != 0) {
        LOGD("SMBus Slave: Comprehensive interrupt clearing (intrStat=0x%08X, rawStat=0x%08X)\n",
             intrStat, rawStat);

        /* CRITICAL FIX: Use the proven clearing function that handles all registers */
        U32 clearedStat = smbusReadClearIntrBits(regBase);
        LOGD("SMBus Slave: smbusReadClearIntrBits cleared = 0x%08X\n", clearedStat);

        /* Additional Write-1-to-clear for main interrupt register */
        if (intrStat != 0) {
            regBase->icIntrStat.value = intrStat;
            LOGD("SMBus Slave: Additional icIntrStat clear = 0x%08X\n", intrStat);
        }

        /* Clear raw interrupt status if needed */
        if (rawStat != 0) {
            regBase->icRawIntrStat.value = rawStat;
            LOGD("SMBus Slave: Additional icRawIntrStat clear = 0x%08X\n", rawStat);
        }
    }
}

/* ======================================================================== */
/*                    Core Interrupt Handler                                 */
/* ======================================================================== */

/**
 * @brief SMBus interrupt service routine (unified I2C/SMBus handler)
 * @details Handles all SMBus and I2C interrupt events. This function provides
 *          a unified interrupt handler that supports both master and slave modes
 *          based on SmbusMode_e. It integrates I2C DW ISR logic with SMBus
 *          protocol handling.
 * @param[in] arg Pointer to driver data structure
 * @return void
 *
 * @note This function is called from the interrupt context
 * @note Uses SmbusMode_e to differentiate between master/slave interrupt handling
 * @note Integrates I2C ISR logic with SMBus protocol handling
 * @note Calls SMBus callback functions for event notification
 * [CORE] Core protocol layer - interrupt handling
 */
static void smbusIsr(void *arg)
{
    SmbusDrvData_s *pDrvData = (SmbusDrvData_s *)arg;
    volatile SmbusRegMap_s *regBase;
    SmbusIntrStatReg_u smbusIntrStat;
    U32 intrStat, enabled;

    if (!pDrvData || !pDrvData->sbrCfg.regAddr) {
        return;
    }

    regBase = (SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

    /* Read interrupt status and enable registers */
    enabled = regBase->icEnable.value;

    /* Debug: Log ISR entry with interrupt status */
    U32 rawIntr = regBase->icRawIntrStat.value;
    LOGI("SMBus ISR Entry: devId=%d, enabled=0x%08X, rawIntr=0x%08X\n",
         pDrvData->devId, enabled, rawIntr);
    intrStat = regBase->icIntrStat.value;
    smbusIntrStat.value = regBase->icSmbusIntrStat.value;

    /* Validate interrupt source - FIXED: Use raw interrupt for validation */
    if (!enabled || (rawIntr == 0) || (rawIntr == 0xFFFFFFFF)) {
        LOGE("SMBus ISR: enabled=0x%08X rawIntr=0x%08X intrStat=0x%08X\n", enabled, rawIntr, intrStat);
        return;
    }

    LOGD("SMBus IRQ: mode=%s, rawIntr=0x%08X, intrStat=0x%08X, smbus=0x%08X\n",
         (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "SLAVE",
         rawIntr, intrStat, smbusIntrStat.value);

    /* 🔥 CRITICAL RUNTIME FIX: Check and force correct Slave interrupt mask */
    if (pDrvData->pSmbusDev.mode != DW_SMBUS_MODE_MASTER) {
        U32 currentMask = regBase->icIntrMask;
        if (currentMask == 0x00080001) {
            LOGW("!!! ISR DETECTED WRONG INTERRUPT MASK - Fixing from 0x%08X to 0x2E4\n", currentMask);
            regBase->icIntrMask = 0x000002E4;

            /* Verify the fix worked */
            U32 verifyMask = regBase->icIntrMask;
            if (verifyMask == 0x000002E4) {
                LOGI("✅ ISR SUCCESSFULLY FIXED interrupt mask to 0x%08X\n", verifyMask);
            } else {
                LOGE("❌ ISR FAILED TO FIX interrupt mask! Still: 0x%08X\n", verifyMask);
            }
        }
    }

    /* Handle interrupts based on current mode - FIXED: Use raw interrupt status */
    if (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) {
        smbusHandleMasterInterrupt(pDrvData, regBase, rawIntr, smbusIntrStat.value);
    } else {
        smbusHandleSlaveInterrupt(pDrvData, regBase, rawIntr, smbusIntrStat.value);
    }
}

/**
 * @brief Initialize SMBus bus
 * @details Initializes the SMBus controller with configuration from SBR,
 *          sets up interrupt handling, and prepares the device for operation.
 *          This function allocates memory for driver data structures and
 *          registers the interrupt service routine.
 * @param[in] devId SMBus device identifier to initialize
 * @return EXIT_SUCCESS on successful initialization, negative error code on failure:
 *         -EBUSY: Device already initialized
 *         -EINVAL: Invalid device identifier or driver mismatch
 *         -ENOMEM: Memory allocation failed
 *         -EIO: Hardware configuration failed
 *         -ETIMEDOUT: Device reset timeout
 * @note This function must be called before any other SMBus operations
 * @note The device is automatically configured according to SBR settings
 * @warning This function is not thread-safe and should be called only once per device
 * [CORE] Core protocol layer - initialization and core protocol logic
 */
S32 smbusInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, SMBUS_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }
    /* Check if already initialized */
    if (isDrvInit(devId) == true) {
        ret = -EBUSY;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_DW_I2C)) {
        ret = -EINVAL;
        goto exit;
    }

    /* Step 1: Allocate driver data structure */
    ret = smbusAllocateDriverData(devId, &pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Driver data allocation failed, ret=%d\n", __func__, ret);
        goto exit;
    }

#if 0
    /* Step 2: Get SBR configuration (not modularized as requested) */
    if (smbusDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: get SBR failed\n", __func__);
        ret = -EIO;
        goto freeMem;
    }
#endif

#ifdef TEST_SUITS_1
    /* Use test configuration for development/testing */
    pDrvData->sbrCfg.regAddr = (void *) (SMBUS_BASE_ADDR + 0 * SMBUS_BASE_OFFSET);
    pDrvData->sbrCfg.irqNo = SYS_INT_NUM_SMBUS;
    pDrvData->sbrCfg.irqPrio = SYS_INT_PRIORITY_SMBUS;
    pDrvData->sbrCfg.masterMode = DW_SMBUS_MODE_MASTER;//DW_SMBUS_MODE_MASTER;
    pDrvData->sbrCfg.interruptMode = 1;    
    pDrvData->sbrCfg.speed = 2;
    pDrvData->sbrCfg.addrMode = 0;        
    pDrvData->sbrCfg.slaveAddrHigh = 0;
    pDrvData->sbrCfg.slaveAddrLow = SMBUS_ARP_DEFAULT_SLAVE_ADDR;
    pDrvData->sbrCfg.enSmbus = 1;
    LOGD("%s: Using test configuration:%d\n", __func__, pDrvData->sbrCfg.masterMode);
#endif

    /* Step 3: Initialize hardware using HAL probe functions */
    ret = smbusInitializeHardware(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Hardware initialization failed, ret=%d\n", __func__, ret);
        goto freeMem;
    }

    /* Step 4: Clear all pending interrupts before installing interrupt handler */
    smbusClearAllInterrupts((volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr);
    LOGD("%s: All interrupts cleared before handler installation\n", __func__);

    /* Step 5: Install interrupt handler (not modularized as requested) */
    LOGD("%s: Installing interrupt handler for IRQ %d, priority %d\n",
         __func__, pDrvData->sbrCfg.irqNo, pDrvData->sbrCfg.irqPrio);

    ret = ospInterruptHandlerInstall(pDrvData->sbrCfg.irqNo, "smbus", OSP_INTERRUPT_UNIQUE,
                                     (OspInterruptHandler)smbusIsr, pDrvData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: IRQ handler already installed\n", __func__);
    } else if (ret == OSP_SUCCESSFUL) {
        LOGD("SMBus interrupt priority: %d\n", pDrvData->sbrCfg.irqPrio);
        ospInterruptVectorEnable(pDrvData->sbrCfg.irqNo);
        LOGD("%s: Interrupt handler installed successfully, vector enabled\n", __func__);

        LOGD("%s: Interrupt vector %d status: %s\n", __func__,
             pDrvData->sbrCfg.irqNo, 1 ? "ENABLED" : "DISABLED");
    } else {
        LOGE("%s: IRQ handler install failed, ret=%d\n", __func__, ret);
        ret = -EIO;
        goto freeMem;
    }

    /* Step 6: Enable peripheral clock if needed */
    ret = peripsClockEnable(devId);
    if (ret != EXIT_SUCCESS && ret != -EINVAL) {
        LOGE("%s: clock enable failed, ret=%d\n", __func__, ret);
        ret = -EIO;
        goto disable_vector;
    }

    /* Step 7: Perform peripheral reset */
    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGW("%s: reset failed\n", __func__);
        /* Continue even if reset fails */
    }

    /* Step 7: Allocate slave buffers for slave mode */
    ret = smbusAllocateSlaveBuffers(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Slave buffer allocation failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    /* Step 8: Allocate master buffers for master mode */
    ret = smbusAllocateMasterBuffers(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Master buffer allocation failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    /* Step 9: Initialize ARP Master context (modularized) */
    ret = smbusInitializeArpMaster(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: ARP Master initialization failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    /* Step 9: Install driver data with system */
    ret = drvInstall(devId, pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: drvInstall failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    LOGI("SMBus device %d initialized successfully in %s mode, semaphoreId=%d\n",
         devId, (pDrvData->sbrCfg.masterMode == 1) ? "master" : "slave",
         pDrvData->pSmbusDev.semaphoreId);
    goto unlock;

disable_vector:
    ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
    ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, smbusIsr, pDrvData);

freeMem:
    smbusFreeDriverData(pDrvData);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Deinitialize SMBus bus
 * @details Cleans up SMBus controller, frees allocated memory,
 *          and unregisters interrupt handlers.
 * @param[in] devId SMBus device identifier to deinitialize
 * @return EXIT_SUCCESS on successful deinitialization, negative error code on failure:
 *         -EINVAL: Invalid device identifier or driver mismatch
 *         -EBUSY: Device busy with ongoing operations
 * @note This function should be called when SMBus operations are complete
 * @warning This function is not thread-safe
 */
S32 smbusDeInit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto unlock;
    }
    /* Destroy semaphore if created */
    if (pDrvData->pSmbusDev.semaphoreId != 0) {
        S32 semRet = rtems_semaphore_delete(pDrvData->pSmbusDev.semaphoreId);
        if (semRet == RTEMS_SUCCESSFUL) {
            LOGD("%s: Semaphore deleted successfully\n", __func__);
        } else {
            LOGW("%s: Semaphore deletion failed, ret=%d\n", __func__, semRet);
        }
        pDrvData->pSmbusDev.semaphoreId = 0;
    }

    /* Disable interrupt vector */
    ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);

    /* Remove interrupt handler */
    S32 irqRet = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, smbusIsr, pDrvData);
    if (irqRet == OSP_SUCCESSFUL) {
        LOGD("%s: Interrupt handler removed successfully\n", __func__);
    } else {
        LOGW("%s: Interrupt handler removal failed, ret=%d\n", __func__, irqRet);
    }

    /* Disable SMBus controller */
    if (pDrvData->pSmbusDev.regBase != NULL) {
        /* Get HAL operations */
        SmbusHalOps_s *halOps = smbusGetHalOps();
        if (halOps != NULL && halOps->disable != NULL) {
            halOps->disable(&pDrvData->pSmbusDev);
        }
    }

    /* Free driver data using centralized cleanup function */
    smbusFreeDriverData(pDrvData);
    /* Unregister driver data */
    drvUninstall(devId);

    LOGI("SMBus device %d deinitialized successfully\r\n", devId);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Master/Slave mode switch
 * @details Dynamically switches the SMBus controller between master and slave
 *          operation modes. This function safely stops ongoing operations,
 *          reconfigures the hardware, and restarts in the new mode.
 * @param[in] devId SMBus device identifier
 * @param[in] param Pointer to switch parameters containing target mode and settings
 * @return EXIT_SUCCESS on successful mode switch, negative error code on failure:
 *         -EINVAL: Invalid parameters or unsupported mode
 *         -EBUSY: Device locked or busy with ongoing operations
 *         -EIO: Hardware reconfiguration failed
 *         -ETIMEDOUT: Mode switch timeout
 * @note The function ensures all ongoing transactions are completed before switching
 * @note All previously configured settings (speed, address mode, etc.) are preserved
 * @warning Mode switching should not be performed during active transactions
 */
S32 smbusMasterSlaveModeSwitch(DevList_e devId, SmbusSwitchParam_s *param)
{
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Parameter validation before any locking */
    if (param == NULL) {
        return -EINVAL;
    }

    /* Validate target mode parameter */
    if (param->targetMode != DW_SMBUS_MODE_MASTER && param->targetMode != DW_SMBUS_MODE_SLAVE) {
        LOGE("%s(): Invalid target mode %d\n", __func__, param->targetMode);
        return -EINVAL;
    }

    /* Validate mode-specific parameters */
    if (param->targetMode == DW_SMBUS_MODE_SLAVE) {
        /* Validate slave address - I2C addresses 0x00 and 0x78-0x7F are reserved */
        if (param->config.slaveConfig.slaveAddr == SMBUS_GENERAL_CALL_ADDR ||
            (param->config.slaveConfig.slaveAddr >= SMBUS_RESERVED_ADDR_START &&
             param->config.slaveConfig.slaveAddr <= SMBUS_RESERVED_ADDR_END)) {
            LOGE("%s(): Invalid slave address 0x%02X (reserved address range)\n",
                 __func__, param->config.slaveConfig.slaveAddr);
            return -EINVAL;
        }

        /* Validate ARP enable flag */
        if (param->config.slaveConfig.enableArp > SMBUS_BOOL_MAX) {
            LOGE("%s(): Invalid ARP enable flag %d (max: %d)\n",
                 __func__, param->config.slaveConfig.enableArp, SMBUS_BOOL_MAX);
            return -EINVAL;
        }
    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        /* Validate address mode */
        if (param->config.masterConfig.addrMode > SMBUS_ADDR_MODE_10BIT) {
            LOGE("%s(): Invalid master address mode %d (max: %d)\n",
                 __func__, param->config.masterConfig.addrMode, SMBUS_ADDR_MODE_10BIT);
            return -EINVAL;
        }

        /* Validate master speed - should be one of the supported SMBus speeds */
        if (param->config.masterConfig.speed != SMBUS_SPEED_100KHZ &&
            param->config.masterConfig.speed != SMBUS_SPEED_400KHZ &&
            param->config.masterConfig.speed != SMBUS_SPEED_1MHZ) {
            LOGE("%s(): Invalid master speed %u Hz. Supported speeds: %u Hz, %u Hz, %u Hz\n",
                 __func__, param->config.masterConfig.speed,
                 SMBUS_SPEED_100KHZ, SMBUS_SPEED_400KHZ, SMBUS_SPEED_1MHZ);
            return -EINVAL;
        }
    }

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< Get driver data
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< Get device pointer from driver data 
    dev = &pDrvData->pSmbusDev;
    if (dev == NULL) {
        LOGE("%s(): Device pointer is NULL\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    /* Update device configuration from switch parameters */
    /* Store timeout in transferTimeout field for temporary storage */
    dev->transferTimeout = param->timeout;

    /* Copy mode-specific configuration */
    if (param->targetMode == DW_SMBUS_MODE_SLAVE) {
        dev->slaveAddr = param->config.slaveConfig.slaveAddr;
        LOGD("%s(): Slave config - addr=0x%02X, arp=%d\n", __func__,
             dev->slaveAddr, param->config.slaveConfig.enableArp);
        /* Note: enableArp could be stored in flags field if needed */
    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        dev->addrMode = param->config.masterConfig.addrMode;
        dev->clkRate = param->config.masterConfig.speed;
        /* Timing parameters will be configured in smbusConfigureMaster() */
        LOGD("%s(): Master config - addrMode=%d, speed=%u, clkRate=%u\n", __func__,
             dev->addrMode, param->config.masterConfig.speed, dev->clkRate);
    }

    /* Get HAL operations for mode switching */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL || halOps->modeSwitchCore == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    /* Call HAL layer mode switch core function */
    ret = halOps->modeSwitchCore(dev, param->targetMode);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): HAL mode switch failed, ret=%d\n", __func__, ret);
        goto unlock;
    }

    LOGD("%s(): Mode switching executed successfully\n", __func__);

    /* ===== 5. CRITICAL FIX: Manage slave resources based on target mode ===== */
    if (param->targetMode == DW_SMBUS_MODE_SLAVE) {
        /* Switching TO slave mode - initialize slave resources */
        ret = smbusInitSlaveResources(pDrvData);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s(): Failed to initialize slave resources, ret=%d\n", __func__, ret);
            goto unlock;
        }

        /* Update driver configuration to reflect slave mode */
        pDrvData->sbrCfg.masterMode = 0;  /* Set to slave mode */

    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        /* Switching TO master mode - free slave resources */
        smbusFreeSlaveResources(pDrvData);

        /* Update driver configuration to reflect master mode */
        pDrvData->sbrCfg.masterMode = 1;  /* Set to master mode */
    }

    /* ===== 6. Clear and reset interrupt state after mode switch ===== */
    if (dev->regBase != NULL) {
        /* Clear all pending interrupts to prevent interrupt storm */
        volatile U32 intrStat = dev->regBase->icIntrStat.value;
        if (intrStat != 0) {
            LOGD("%s(): Clearing pending interrupts: 0x%08X\n", __func__, intrStat);
            dev->regBase->icIntrStat.value = intrStat;  /* Write-1-to-clear */
        }

        /* Small delay to ensure hardware state settles */
        udelay(1000);  /* 1ms delay */
    }

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/* ======================================================================== */
/*                    Core ARP Functions                                    */
/* ======================================================================== */

/**
 * @brief Register ARP failure callback function
 * @details Registers a callback function that will be invoked when ARP operations
 *          encounter failures such as timeouts, bus collisions, or protocol errors.
 *          This allows applications to implement custom error handling and recovery
 *          strategies for ARP operations.
 * @param[in] devId SMBus device identifier
 * @param[in] callback Pointer to the ARP failure callback function
 * @param[in] userData User-defined data that will be passed to the callback
 * @return EXIT_SUCCESS on successful registration, negative error code on failure:
 *         -EINVAL: Invalid parameters or device not found
 *         -ENODEV: Device not found or not initialized
 *         -EBUSY: Device locked by another operation
 *
 * @note The callback will be invoked from interrupt context
 * @note Only one callback can be registered per device at a time
 * @note Passing NULL as callback will unregister any existing callback
 * @note The callback should return an SmbusArpAction_e to indicate the desired action
 * @warning Callback functions should be fast and non-blocking
 * @warning Complex operations should be deferred to task context
 * @warning The callback is called in interrupt context, avoid long operations
 *
 */
S32 smbusRegArpFailCallback(DevList_e devId, SmbusArpFailHandler_t callback, void *userData)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    if (callback == NULL && userData != NULL) {
        LOGE("%s(): Invalid parameters - NULL callback with non-NULL userData\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    ///< Check driver match
    if (!isDrvMatch(devId, DRV_ID_DW_I2C)) {
        ret = -EINVAL;
        goto exit;
    }

    ///< Get driver data
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto exit;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, SMBUS_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    ///< Handle callback registration or update
    if (pDrvData->arpFailHandler != callback) {
        LOGD("%s(): Registering new ARP failure callback for device %d\n", __func__, devId);
        ///< Register new callback - context is already initialized in smbusInit
        pDrvData->arpFailHandler = callback;
        pDrvData->arpFailParam = userData;

        LOGI("%s(): ARP failure callback registered successfully for device %d\n",
             __func__, devId);
    } else {
        ///< Same callback - just update userData if different
        if (pDrvData->arpFailParam != userData) {
            pDrvData->arpFailParam = userData;
            LOGD("%s(): Updated ARP failure callback userData for device %d\n",
                 __func__, devId);
        }
    }
    devUnlockByDriver(devId);

exit:
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): Failed to register ARP failure callback, ret=%d\n",
             __func__, ret);
    }

    return ret;
}


/**
 * @brief Core ARP get UDID function
 * [CORE] Core protocol layer - initialization and core protocol logic
 */
S32 smbusArpGetUdidDirected(DevList_e devId, U8 address, SmbusUdid_s *udid)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase;
    U8 getUdidCmd = SMBUS_CMD_GENERAL_GET_UDID;
    U8 byteCount = 0;
    U8 pec = 0;
    U8 udidData[16];
    U8 i;

    if (udid == NULL) {
        return -EINVAL;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        LOGE("%s: getSmbusReg failed\n", __func__);
        return -ENODEV;
    }

    LOGD("%s: Getting UDID from directed address 0x%02X\n", __func__, address);

    /* Step 1: Set target address to specified address */
    regBase->icTar.fields.icTar = (address & 0x7F);

    /* Step 2: Send Get UDID command */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL || halOps->checkTxReady == NULL) {
        LOGE("%s: HAL operations not available\n", __func__);
        return -ENOTSUP;
    }
    ret = halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for Get UDID command\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = getUdidCmd;

    /* Step 3: Send Byte Count = 0 (Read command) */
    ret = halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for byte count\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.value = (0 & 0xff) | (1 << 8) | (1 << 10); /* Read + STOP */

    /* Step 4: Read Byte Count */
    ret = halOps->checkRxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: RX ready timeout for byte count\n", __func__);
        return -ETIMEDOUT;
    }
    byteCount = (U8)regBase->icDataCmd.value;

    /* Verify byte count should be 16 */
    if (byteCount != 16) {
        LOGW("%s: Unexpected UDID byte count: %d (expected 16)\n", __func__, byteCount);
    }

    /* Step 5: Read 16-byte UDID data */
    for (i = 0; i < 16; i++) {
        ret = halOps->checkRxReady(regBase);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: RX ready timeout for UDID data byte %d\n", __func__, i);
            return -ETIMEDOUT;
        }
        udidData[i] = (U8)regBase->icDataCmd.value;
    }

    /* Step 6: Read PEC */
    ret = halOps->checkRxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: RX ready timeout for PEC\n", __func__);
        return -ETIMEDOUT;
    }
    pec = (U8)regBase->icDataCmd.value;

    /* Step 7: Verify PEC */
    U8 pec_buffer[18];
    pec_buffer[0] = SMBUS_CMD_GENERAL_GET_UDID;  /* Command */
    pec_buffer[1] = 0;  /* Byte count = 0 */
    for (i = 0; i < 16; i++) {
        pec_buffer[i + 2] = udidData[i];
    }

    U8 calculated_pec = smbusPecPktConstruct(address, false, pec_buffer, 2 + 16);
    if (calculated_pec != pec) {
        LOGE("%s: UDID PEC verification failed: calculated=0x%02X, received=0x%02X\n",
             __func__, calculated_pec, pec);
        return -EIO;
    }

    /* Step 8: Copy UDID data to output structure */
    memset(udid, 0, sizeof(SmbusUdid_s));
    udid->nextAvailAddr = udidData[0];
    udid->version = udidData[1];
    udid->vendorId = (udidData[3] << 8) | udidData[2];
    udid->deviceId = (udidData[5] << 8) | udidData[4];
    udid->interface = (udidData[7] << 8) | udidData[6];
    udid->subsystemVendorId = (udidData[9] << 8) | udidData[8];
    udid->deviceAddr = udidData[15];

    /* Copy remaining bytes */
    for (i = 0; i < 6; i++) {
        udid->bytes[i] = udidData[10 + i];
    }

    LOGD("%s: UDID read successfully: vendor=0x%04X, device=0x%04X, addr=0x%02X\n",
         __func__, udid->vendorId, udid->deviceId, udid->deviceAddr);

    return EXIT_SUCCESS;
}

/**
 * @brief Calculate SMBus PEC (Packet Error Code) for buffer
 * @details Calculates SMBus PEC (CRC-8) for a buffer of data. PEC is used
 *          to verify data integrity in SMBus communications.
 * @param[in] pec Initial PEC value (typically 0)
 * @param[in] buf Pointer to data buffer
 * @param[in] len Length of data buffer
 * @return Calculated PEC value
 *
 * @note Uses CRC-8 with polynomial 0x07
 * @note Processes entire buffer sequentially
 * [CORE] Core protocol layer - PEC calculation utilities
 */
U8 smbusCalcPEC(U8 pec, const U8 *buf, U32 len)
{
    for (U32 i = 0; i < len; ++i) {
        pec = smbusCrc8CalcOne(pec, buf[i]);
    }
    return pec;
}

/**
 * @brief Convert I2C 7-bit address to read/write address byte
 * @details Converts a 7-bit I2C address to an 8-bit address byte with the
 *          appropriate read/write bit set.
 * @param[in] addrIn 7-bit I2C address (0-127)
 * @param[in] isWrite True for write operation, false for read operation
 * @return 8-bit address with R/W bit set
 *
 * @note Left-shifts address by 1 and adds R/W bit
 * @note Write operation: R/W bit = 0, Read operation: R/W bit = 1
 * [CORE] Core protocol layer - address conversion utilities
 */
const U8 i2cAddrConvert(U8 addrIn, bool isWrite)
{
    return (addrIn << 1) | (isWrite ? 0 : 1);
}

/**
 * @brief Construct PEC for complete SMBus packet
 * @details Constructs PEC for a complete SMBus packet including device
 *          address, read/write bit, and data payload. This function
 *          calculates the PEC that would be transmitted at the end of
 *          an SMBus transaction.
 * @param[in] addr7bitIn 7-bit SMBus device address
 * @param[in] isWrite True for write operation, false for read operation
 * @param[in] pData Pointer to data payload buffer
 * @param[in] count Number of data bytes in payload
 * @return Calculated PEC value for the complete packet
 *
 * @note Includes device address and R/W bit in PEC calculation
 * @note Validates input parameters for safety
 * @note Returns 0xFF for invalid parameters
 * [CORE] Core protocol layer - PEC calculation utilities
 */
U8 smbusPecPktConstruct(U8 addr7bitIn, bool isWrite, U8 *pData, U32 count)
{
    if ((pData == NULL) && (count > 0)) {
        return 0xFF;
    }

    if (count > SMBUS_MAX_BUFFER_SIZE) {
        return 0xFF;
    }
    U8 pec = 0;
    U8 addr = i2cAddrConvert(addr7bitIn, isWrite);
    U8 addrWithPec = smbusCalcPEC(pec, &addr, 1);
    return smbusCalcPEC(addrWithPec, pData, count);
}

/* ======================================================================== */
/*                    Core Protocol - Timeout Recovery Functions             */
/* ======================================================================== */

/**
 * @brief Handle master timeout recovery based on flow diagram
 * @details Handles timeout recovery for SMBus master operations. This function
 *          implements a comprehensive recovery procedure to safely restore the
 *          bus to an operational state after timeout conditions occur.
 * @param[in] pDrvData Pointer to SMBus driver private data structure
 * @param[in] timeoutType Type of timeout that occurred
 * @return void
 *
 * @note Disables master mode and generates STOP condition to release the bus
 * @note Waits for STOP condition completion with timeout protection
 * @note Resets FIFO buffers and clears pending interrupts
 * @note Re-enables master mode after recovery procedures
 * @note Resets transfer state variables
 * @warning This function should only be called in Master mode
 * @warning Function modifies controller configuration registers directly
 * [CORE] Core protocol layer - timeout recovery operations
 */
void smbusHandleMasterTimeoutRecovery(SmbusDrvData_s *pDrvData, SmbusTimeoutType_e timeoutType)
{
    volatile SmbusRegMap_s *regBase;

    if (pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL) {
        return;
    }

    regBase = (SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

    ///< Log timeout recovery attempt
    LOGW("SMBus Master timeout recovery started - type: %s\n",
         timeoutType == SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND ? "CLOCK_EXTEND" : "CLOCK_LOW");

    ///< Step 1: Disable master mode to stop ongoing transaction
    SmbusIcConReg_u conReg = regBase->icCon;
    conReg.fields.masterMode = 0;
    regBase->icCon = conReg;

    ///< Step 2: Generate STOP condition to release the bus
    conReg.fields.stopDetIfMasterActive = 1;
    regBase->icCon = conReg;

    ///< Step 3: Wait for STOP condition to complete (with timeout)
    U32 timeout = SMBUS_TRANSACTION_TIMEOUT_US;
    while (timeout-- > 0) {
        if (regBase->icRawIntrStat.fields.stopDet) {
            break;
        }
        udelay(1);
    }

    if (timeout == 0) {
        LOGE("SMBus Master timeout recovery - STOP condition timeout\n");
        pDrvData->errorCount++;
        goto recovery_complete;
    }

    ///< Clear STOP detection interrupt
    regBase->icClrIntr;

    ///< Step 4: Reset FIFO buffers
    regBase->icEnable.fields.enable = 0;
    udelay(10);  ///< Small delay to ensure disable takes effect
    regBase->icEnable.fields.enable = 1;

    ///< Step 5: Clear any pending interrupts
    regBase->icClrTxAbrt;          /* Dummy read to clear TX abort */
    regBase->icClrIntr;           /* Dummy read to clear combined interrupt */
    (void)regBase->icClrSmbusIntr; /* Dummy read to clear SMBus interrupts */

    ///< Step 6: Re-enable master mode
    conReg.fields.masterMode = 1;
    conReg.fields.stopDetIfMasterActive = 0;
    regBase->icCon = conReg;

    ///< Log successful recovery
    LOGI("SMBus Master timeout recovery completed successfully\n");

recovery_complete:
    ///< Reset transfer state
    pDrvData->txComplete = 0;
    pDrvData->rxComplete = 0;
    pDrvData->slaveTransferActive = 0;

    return;
}
/**
 * @brief Handle slave timeout recovery based on flow diagram
 * @details Handles timeout recovery for SMBus slave operations. This function
 *          implements a comprehensive recovery procedure to safely restore the
 *          slave device to an operational state after timeout conditions occur.
 * @param[in] pDrvData Pointer to SMBus driver private data structure
 * @param[in] timeoutType Type of timeout that occurred
 * @return void
 *
 * @note Temporarily disables slave mode to clear ongoing transfers
 * @note Resets FIFO buffers and clears pending interrupts
 * @note Re-enables slave mode after recovery procedures
 * @note Handles slave activity state machine reset if needed
 * @note Resets transfer state variables
 * @warning This function should only be called in Slave mode
 * @warning Function modifies controller configuration registers directly
 * [CORE] Core protocol layer - timeout recovery operations
 */
void smbusHandleSlaveTimeoutRecovery(SmbusDrvData_s *pDrvData, SmbusTimeoutType_e timeoutType)
{
    volatile SmbusRegMap_s *regBase;

    if (pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL) {
        return;
    }

    regBase = (SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

    ///< Log timeout recovery attempt
    LOGW("SMBus Slave timeout recovery started - type: %s\n",
         timeoutType == SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND ? "CLOCK_EXTEND" : "CLOCK_LOW");

    ///< Step 1: Disable slave mode temporarily
    SmbusIcConReg_u conReg = regBase->icCon;
    conReg.fields.icSlaveDisable = 1;
    regBase->icCon = conReg;

    ///< Step 2: Clear any ongoing transfer state
    pDrvData->slaveTransferActive = 0;
    pDrvData->txComplete = 0;
    pDrvData->rxComplete = 0;

    ///< Step 3: Reset FIFO buffers
    regBase->icEnable.fields.enable = 0;
    udelay(10);  ///< Small delay to ensure disable takes effect
    regBase->icEnable.fields.enable = 1;

    ///< Step 4: Clear all pending interrupts
    regBase->icClrTxAbrt;          /* Dummy read to clear TX abort */
    regBase->icClrIntr;           /* Dummy read to clear combined interrupt */
    (void)regBase->icClrSmbusIntr; /* Dummy read to clear SMBus interrupts */

    ///< Step 5: Re-enable slave mode
    conReg.fields.icSlaveDisable = 0;
    regBase->icCon = conReg;

    ///< Step 6: Reset slave state machine if needed
    if (regBase->icStatus.fields.slvActivity) {
        ///< Wait for slave activity to clear (with timeout)
        U32 timeout = SMBUS_RX_READY_TIMEOUT_US;
        while (timeout-- > 0 && regBase->icStatus.fields.slvActivity) {
            udelay(1);
        }

        if (timeout == 0) {
            LOGE("SMBus Slave timeout recovery - slave activity timeout\n");
            ///< Force clear slave activity by disabling/enabling
            conReg.fields.icSlaveDisable = 1;
            regBase->icCon = conReg;
            udelay(100);
            conReg.fields.icSlaveDisable = 0;
            regBase->icCon = conReg;
        }
    }
    ///< Log successful recovery
    LOGI("SMBus Slave timeout recovery completed successfully\n");

    return;
}