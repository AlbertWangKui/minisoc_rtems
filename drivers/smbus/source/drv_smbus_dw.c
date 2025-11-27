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
 * 2025/11/23   wangkui         HAL operations optimization - added halOps pointer registration in device initialization
 *                              Enhanced SmbusDev_s structure with HAL operations function table
 *                              Improved driver architecture by eliminating redundant HAL lookups
 *                              Registered HAL ops once during initialization for all subsequent operations
 * @note This file implements the SMBus Core protocol layer, providing
 *       the foundational SMBus protocol implementation including
 *       initialization, mode switching, ARP LOGEc, and state management.
 *       This layer sits between the HAL layer and the API layers,
 *       implementing the core SMBus protocol LOGEc.
 */
#include "common_defines.h"
#include "sbr_api.h"
#include "drv_smbus_dw.h"
#include "drv_smbus_dw_i2c.h"

/* ======================================================================== */
/*                    SMBus Initialization/Deinitialization                 */
/* ======================================================================== */

/* Forward declarations for modular initialization functions */
static S32 smbusAllocateDriverData(DevList_e devId, SmbusDrvData_s **ppDrvData);
static S32 smbusInitializeHardware(SmbusDrvData_s *pDrvData);
static void smbusClearInterrupts(volatile SmbusRegMap_s *regBase, U32 mask);
static S32 smbusAllocateMasterBuffers(SmbusDrvData_s *pDrvData);
static S32 smbusInitializeArpMaster(SmbusDrvData_s *pDrvData);
static void smbusFreeDriverData(SmbusDrvData_s *pDrvData);

/* ======================================================================== */
/*                    Core Utility Functions                                 */
/* ======================================================================== */

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
    S32 ret = 0;

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL, "pDrvData is NULL");

    /* Only initialize master buffers in master mode */
    if (pDrvData->sbrCfg.masterMode == SMBUS_MODE_MASTER) {
        /* Initialize master TX buffer array */
        memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
        pDrvData->pSmbusDev.masterTxBufLen = 0;
        LOGD("%s: Master TX buffer initialized (%u bytes)\n", __func__, sizeof(pDrvData->pSmbusDev.masterTxBuf));

        /* Note: master RX buffer is handled by pDrvData->rxBuffer in legacy mode */

        LOGD("%s: Master buffers initialized successfully\n", __func__);
    }

exit:
    return ret;
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
    S32 ret = EXIT_SUCCESS;
    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL, "pDrvData is NULL");

    LOGD("%s: Checking mode: masterMode=%d, SMBUS_MODE_SLAVE=%d\n",
         __func__, pDrvData->sbrCfg.masterMode, SMBUS_MODE_SLAVE);

    if (pDrvData->sbrCfg.masterMode == SMBUS_MODE_SLAVE) {
        LOGD("%s: Device is in SLAVE mode - allocating buffers\n", __func__);
        /* Only allocate if not already allocated */
        if (pDrvData->pSmbusDev.slaveRxBuf == NULL) {
            LOGD("%s: Allocating slave RX buffer\n", __func__);
            pDrvData->pSmbusDev.slaveRxBuf = (U8 *)malloc(SMBUS_SLAVE_BUF_LEN);
            if (pDrvData->pSmbusDev.slaveRxBuf == NULL) {
                LOGE("%s: Failed to allocate slave RX buffer\n", __func__);
                ret = -ENOMEM;
                goto exit;
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
                ret = -ENOMEM;
                goto exit;
            }
            pDrvData->pSmbusDev.slaveValidTxLen = 0;
            pDrvData->slaveTxIndex = 0;
        }
        LOGE("%s: Slave resources initialized successfully\n", __func__);
    }else {
        LOGD("%s: Device not in slave mode, skipping slave buffer allocation\n", __func__);
    }   
exit:
    return ret;
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
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL, "pDrvData is NULL");

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

    LOGE("%s: Slave resources freed successfully\n", __func__);
}

/**
 * @brief Copy UDID structure
 * @param dest Destination UDID
 * @param src Source UDID
 * [CORE] Core protocol layer - UDID copying
 */
__attribute((unused)) static void smbusUdidCopy(SmbusUdid_s *dest, const SmbusUdid_s *src)
{
    SMBUS_CHECK_PARAM_VOID(dest == NULL || src == NULL, "dest or src is NULL");

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
    LOGE("smbus: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
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
    SMBUS_CHECK_PARAM_RETURN(isDrvInit(devId) == false, -EINVAL, "Driver not initialized for devId %d", devId);
    SMBUS_CHECK_PARAM_RETURN(pCtrlReg == NULL, -EINVAL, "pCtrlReg is NULL");

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

    SMBUS_CHECK_PARAM_RETURN(ppDrvData == NULL, -EINVAL, "ppDrvData is NULL");
    LOGE("1\r\n");
    /* Allocate driver data structure */
    pDrvData = (SmbusDrvData_s *)calloc(1, sizeof(SmbusDrvData_s));
    if (pDrvData == NULL) {
        LOGE("%s: Failed to allocate driver data\n", __func__);
        return -ENOMEM;
    }

    /* Initialize master buffer variables */
    memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
    pDrvData->pSmbusDev.masterTxBufLen = 0;

    
    /* CRITICAL FIX: Initialize callback structure to prevent NULL pointer issues */
    pDrvData->callback.cb = NULL;           /* Initialize callback pointer to NULL */
    pDrvData->callback.userData = NULL;     /* Initialize user data to NULL */

    pDrvData->pSmbusDev.busId = devId - DEVICE_SMBUS0;  /* Set busId based on device ID */
    LOGD("%s: Allocated driver data for devId %d (busId %d)\n",
         __func__, devId, pDrvData->pSmbusDev.busId);
    /* Initialize basic fields - calloc already zeroed all other fields */
    pDrvData->devId = devId;

    /* Zero-initialize all numeric fields */
    memset(&pDrvData->pSmbusDev.status, 0,
           sizeof(pDrvData->pSmbusDev) - offsetof(SmbusDev_s, status));

    /* Set specific non-zero default values */
    pDrvData->pSmbusDev.mode = DW_SMBUS_MODE_MASTER;  /* Default to master mode */
    pDrvData->pSmbusDev.transferTimeout = SMBUS_DEFAULT_TIMEOUT_MS;  /* Default timeout */
    
    memset(pDrvData->pSmbusDev.msgs, 0, sizeof(SmbusMsg_s) *32);

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
    S32 ret = 0;

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL, "pDrvData is NULL");
    /* Configure SMBus device structure based on SBR settings */
        pDrvData->pSmbusDev = (SmbusDev_s){
        .regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr,
        .addrMode = pDrvData->sbrCfg.addrMode,
        .slaveAddr = pDrvData->sbrCfg.slaveAddrLow,
        .irq = pDrvData->sbrCfg.irqNo,
        .channelNum = pDrvData->devId,
        .workMode = (pDrvData->sbrCfg.interruptMode == 1) ? 0 : 1,
        .isSmbus = 1,
        .enabled = 1,
        .transferTimeout = SMBUS_TRANSFER_TIMEOUT_MS,
        .halOps = smbusGetHalOps()  /* Register HAL operations during initialization */
    };

    static const U32 sSmbusClkRateTable[SMBUS_SPEED_MODE_MAX] = {
        [SMBUS_SPEED_MODE_STANDARD]  = SMBUS_MAX_STANDARD_MODE_FREQ,   /* 100000U */
        [SMBUS_SPEED_MODE_FAST]      = SMBUS_MAX_FAST_MODE_FREQ,       /* 400000U */
        [SMBUS_SPEED_MODE_FAST_PLUS] = SMBUS_MAX_FAST_MODE_PLUS_FREQ   /* 1000000U */
    };

    /* Default to Fast mode if invalid speed setting */
    const U32 defaultClkRate = SMBUS_MAX_FAST_MODE_FREQ;

    pDrvData->pSmbusDev.clkRate = (pDrvData->sbrCfg.speed < SMBUS_SPEED_MODE_MAX)
                                 ? sSmbusClkRateTable[pDrvData->sbrCfg.speed]
                                 : defaultClkRate;

    /* Configure SMBus controller based on mode using I2C-compatible probe functions */
    if (pDrvData->sbrCfg.masterMode == SMBUS_MODE_MASTER) {
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
        /* Configure slave settings using HAL operations */
        if (pDrvData->pSmbusDev.halOps != NULL && pDrvData->pSmbusDev.halOps->configureSlave != NULL) {
            pDrvData->pSmbusDev.halOps->configureSlave(&pDrvData->pSmbusDev);
            LOGD("%s: Slave configuration completed via HAL operations\n", __func__);
        } else {
            LOGW("%s: HAL configureSlave function not available, using default configuration\n", __func__);
        }

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
    ret = smbusCreateSemaphore(&pDrvData->pSmbusDev, "Init");
    if (ret != 0) {
        return ret;  // Error already logged
    }

    /* FINAL SAFETY FIX: Force correct Slave interrupt mask if in Slave mode */
    if (pDrvData->sbrCfg.masterMode == SMBUS_MODE_SLAVE) {
        volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

        /* 改进：使用优化的初始化中断掩码，初始禁用TX_EMPTY以避免伪中断*/
        /* TX_EMPTY将在WR_REQ处理过程中根据需要动态启用 */
        regBase->icIntrMask.value = SMBUS_SLAVE_INIT_INTR_MASK;

        LOGE("FINAL OVERRIDE: Slave interrupt mask FORCED to 0x%08X (TX_EMPTY initially disabled)\n",
             regBase->icIntrMask.value);
        LOGD("Slave mode initialized with TX_EMPTY disabled - will be enabled on demand\n");
    }

exit:
    return ret;
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

/* 中断位到寄存器偏移的映射数组（按位索引） */
static volatile U32* const INTR_CLR_REGS[] = {
    [0]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRxUnder,    /* RX_UNDER */
    [1]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRxOver,     /* RX_OVER */
    [3]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrTxOver,     /* TX_OVER */
    [5]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRdReq,      /* RD_REQ */
    [6]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrTxAbrt,     /* TX_ABRT */
    [7]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRxDone,     /* RX_DONE */
    [8]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrActivity,   /* ACTIVITY */
    [9]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrStopDet,    /* STOP_DET */
    [10] = (volatile U32*)&((SmbusRegMap_s*)0)->icClrStartDet,   /* START_DET */
    [11] = (volatile U32*)&((SmbusRegMap_s*)0)->icClrGenCall,    /* GEN_CALL */
    [12] = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRestartDet, /* RESTART_DET */
};

static inline void clearInterrupts(volatile SmbusRegMap_s *regBase, U32 mask)
{
    U32 bit;
    volatile U32 *reg_ptr;
    uintptr_t base_addr, reg_offset;

    /* 使用位扫描快速定位 */
    while (mask != 0) {
        bit = __builtin_ctz(mask);  /* 获取最低置位位索引（GCC内置函数） */

        if (bit < ARRAY_SIZE(INTR_CLR_REGS) && INTR_CLR_REGS[bit] != NULL) {
            /* 计算实际寄存器地址 - 使用uintptr_t进行正确的指针运算 */
            base_addr = (uintptr_t)regBase;
            reg_offset = (uintptr_t)INTR_CLR_REGS[bit];
            reg_ptr = (volatile U32*)(base_addr + reg_offset);
            (void)(*reg_ptr);
        }

        mask &= ~(1U << bit);  /* 清除已处理的位 */
    }
}

static void smbusClearInterrupts(volatile SmbusRegMap_s *regBase, U32 mask)
{
    SMBUS_CHECK_PARAM_VOID(regBase == NULL, "regBase is NULL");

    /* Clear all interrupts if mask equals ALL mask */
    if (mask == 0xFFFFFFFF) { /* STARS_I2C_STATUS_INT_ALL equivalent */
        (void)regBase->icClrIntr;
        if (regBase->icClrSmbusIntr != 0) {
            (void)regBase->icClrSmbusIntr;  /* Dummy read to clear SMBus interrupts */
        }
        LOGD("%s: Cleared all interrupts\n", __func__);
        return;
    }

    /* 使用优化后的中断清除函数 */
    clearInterrupts(regBase, mask);

    /* Clear SMBus specific interrupts if available */
    if (regBase->icClrSmbusIntr != 0) {
        (void)regBase->icClrSmbusIntr;  /* Dummy read to clear SMBus interrupts */
    }

    LOGD("%s: Cleared interrupts with mask 0x%08X\n", __func__, mask);
}

/**
 * @brief Handle master mode interrupts (integrated from i2c_dw_isr)
 * @details Processes master mode interrupt events including TX abort,
 *          RX ready, TX ready. This function integrates I2C master ISR
 *          LOGEc with SMBus protocol handling.
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] intrStat I2C interrupt status
 * @param[in] smbusIntrStat SMBus interrupt status= * @return void
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

        /* Only process transfer LOGEc if actively transferring */
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
            regBase->icIntrMask.value = 0;

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
            LOGE("SMBus Async: Activity detected (0x%08X) and cleared\n", intrStat);
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

                U32 currentMask = regBase->icIntrMask.value;
                currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                regBase->icIntrMask.value = currentMask;

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
                U32 newMask = regBase->icIntrMask.value;
                LOGD("SMBus Async: TX_EMPTY processed, new mask=0x%08X, TX_EMPTY enabled=%d\n",
                     newMask, (newMask & SMBUS_IC_INTR_TX_EMPTY_MASK) ? 1 : 0);

                /* CRITICAL FIX: If smbusDwXferMsg completed all messages, ensure TX_EMPTY is disabled */
                if (dev->msgWriteIdx >= dev->msgsNum) {
                    U32 updatedMask = regBase->icIntrMask.value;
                    updatedMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                    regBase->icIntrMask.value = updatedMask;
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
            U32 abortMask = regBase->icIntrMask.value;
            abortMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            regBase->icIntrMask.value = abortMask;
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
                LOGE("SMBus Async: TX_EMPTY disabled due to STOP detection - mask=0x%08X\n", regBase->icIntrMask.value);

                /* Force clear any pending TX_EMPTY condition */
                if (dev->msgWriteIdx >= dev->msgsNum) {
                    //regBase->icIntrMask.value = 0;  /* Disable all interrupts on STOP */
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
                U32 currentMask = regBase->icIntrMask.value;
                currentMask &= ~(SMBUS_IC_INTR_TX_EMPTY_MASK | SMBUS_IC_INTR_RX_FULL_MASK |
                                SMBUS_IC_INTR_TX_ABRT_MASK | SMBUS_IC_INTR_STOP_DET_MASK |
                                SMBUS_IC_INTR_ACTIVITY_MASK);
                regBase->icIntrMask.value = currentMask;
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
                    U32 finalMask = regBase->icIntrMask.value;
                    finalMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
                    regBase->icIntrMask.value = finalMask;
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
                LOGD("SMBus Async: Transfer complete, releasing semaphore before exiting\n");
                smbusReleaseSemaphore(dev, "SMBus Async");

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
        regBase->icIntrMask.value = 0;
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
            U32 currentMask = regBase->icIntrMask.value;
            currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            regBase->icIntrMask.value = currentMask;
            LOGD("SMBus Master: TX_EMPTY disabled due to transfer completion\n");

            /* Clear device active status and release semaphore */
            LOGD("SMBus Master: Transfer completed, releasing semaphore\n");
            smbusReleaseSemaphore(&pDrvData->pSmbusDev, "SMBus Master");
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
    /* Clear all interrupt status bits using optimized macro approach */
    SMBUS_CLEAR_ALL_INTERRUPTS(regBase);
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

    SMBUS_CHECK_PARAM_VOID(master == NULL, "master is NULL");

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
    SMBUS_CHECK_PARAM_RETURN(master == NULL || udid == NULL, -EINVAL, "master or udid is NULL");

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
 */
__attribute((unused)) static int ArpDevInstallAuto(SmbusArpMaster_s *master, const SmbusUdid_s *udid,
                                                        U8 *assignedAddress)
{
    U8 address;
    SmbusArpDeviceNode_s *current = NULL;

    SMBUS_CHECK_PARAM_RETURN(master == NULL || udid == NULL || assignedAddress == NULL, -EINVAL, "master, udid, or assignedAddress is NULL");

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
        LOGE("master is NULL");
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

    SMBUS_CHECK_PARAM_RETURN(master == NULL || udid == NULL, -EINVAL, "master or udid is NULL");

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
 *          This function modularizes the ARP master initialization LOGEc.
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
    SMBUS_CHECK_PARAM_RETURN(pDrvData == NULL, -EINVAL, "pDrvData is NULL");

    /* Initialize ARP configuration with defaults using macro */
    pDrvData->arpConfig = (SmbusArpConfig_s)SMBUS_ARP_CONFIG_INIT;
    pDrvData->arpState = SMBUS_ARP_STATE_IDLE;
    pDrvData->retryCount = 0;

    /* Initialize ARP Master context using macro */
    pDrvData->arpMaster = (SmbusArpMaster_s)SMBUS_ARP_MASTER_INIT;
    pDrvData->arpMaster.busId = pDrvData->devId - DEVICE_SMBUS0;
    LOGD("%s: ARP Master busId set to %d\n", __func__, pDrvData->arpMaster.busId);

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
    if (pDrvData->enableArp) {
       smbusClearDevs(&pDrvData->arpMaster);
    }
    /* CRITICAL FIX: Free master RX buffer if allocated */
    if (pDrvData->rxBuffer != NULL) {
        free(pDrvData->rxBuffer);
        pDrvData->rxBuffer = NULL;
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
 * @note Integrates I2C slave ISR LOGEc with SMBus protocol handling
 * @note Uses SMBus callback functions for event notification
 * [CORE] Core protocol layer - slave interrupt handling
 */
static void smbusHandleSlaveInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                      U32 intrStat, U32 smbusIntrStat)
{
    /* ✓ 关键修复：优先处理 RD_REQ，顺序很重要！ */

    /* 基本参数验证 */
    if (pDrvData == NULL || regBase == NULL) {
        LOGE("SMBus Slave: Invalid parameters\n");
        return;
    }

    LOGI("SMBus Slave: Interrupt triggered - intrStat=0x%08X, smbusIntrStat=0x%08X\n", intrStat, smbusIntrStat);

    /* 1. 首先处理 WR_REQ（最高优先级）- Master 写入 Slave 时触发 */
    if (intrStat & SMBUS_IC_INTR_WR_REQ_MASK) {
        LOGI("SMBus Slave: WR_REQ - Master is writing to Slave (Priority 1)\n");

        /* 设置状态：正在进行写操作 */
        pDrvData->pSmbusDev.status |= SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
#if 0
        /* 关键修复：立即尝试读取FIFO数据以清除WR_REQ中断 */
        U32 rxValid = regBase->icRxflr;
        if (rxValid > 0) {
            LOGD("SMBus Slave: WR_REQ - Reading %u bytes from FIFO to clear interrupt\n", rxValid);

            /* 读取FIFO数据以清除WR_REQ中断 */
            U32 readCount = 0;
            while (readCount < rxValid && readCount < SMBUS_SLAVE_BUF_LEN) {
                U32 tmp = regBase->icDataCmd.value;
                U8 val = (U8)tmp;

                if (pDrvData->pSmbusDev.slaveValidRxLen < SMBUS_SLAVE_BUF_LEN && pDrvData->pSmbusDev.slaveRxBuf) {
                    /* 关键修复：添加DSB内存屏障确保数据存储完整性 */
                    __asm__ volatile("dsb sy" : : : "memory");

                    pDrvData->pSmbusDev.slaveRxBuf[pDrvData->pSmbusDev.slaveValidRxLen++] = val;

                    /* 再次添加DSB确保计数器更新完成 */
                    __asm__ volatile("dsb sy" : : : "memory");
                } else {
                    LOGE("SMBus Slave: WR_REQ - FAILED to store byte! len=%u, buf=%p\n",
                         pDrvData->pSmbusDev.slaveValidRxLen, pDrvData->pSmbusDev.slaveRxBuf);
                    /* 重置计数器以防数据损坏 */
                    pDrvData->pSmbusDev.slaveValidRxLen = 0;
                }
                readCount++;
                LOGD("SMBus Slave: WR_REQ - Read byte[%u] = 0x%02X\n", readCount, val);
            }

            LOGI("SMBus Slave: WR_REQ - Successfully read %u bytes, slaveValidRxLen = %u\n",
             readCount, pDrvData->pSmbusDev.slaveValidRxLen);
        } else {
            LOGD("SMBus Slave: WR_REQ - No data in FIFO, clearing interrupt by acknowledging\n");
            /* 如果没有数据，至少要 acknowledge 中断 */
        }
       #endif 
        /* 关键修复：WR_REQ表示Master写入Slave，不应该启用TX_EMPTY！ */
        /* TX_EMPTY只在RD_REQ(Master读取Slave)时才需要 */
        regBase->icIntrMask.value &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
        LOGD("SMBus Slave: WR_REQ - TX_EMPTY disabled (Master is writing, not reading) (mask=0x%08X)\n",
             regBase->icIntrMask.value);

        (void)regBase->icClrWrReq;  ///< 清除 WR_REQ 中断
    }

    /* 1.5 处理 SLV_ADDRx_TAG中断 - 地址标签中断 */
    if (intrStat & (SMBUS_IC_INTR_SLV_ADDR1_TAG_MASK | SMBUS_IC_INTR_SLV_ADDR2_TAG_MASK |
                   SMBUS_IC_INTR_SLV_ADDR3_TAG_MASK | SMBUS_IC_INTR_SLV_ADDR4_TAG_MASK)) {
        U32 addrTags = 0;
        if (intrStat & SMBUS_IC_INTR_SLV_ADDR1_TAG_MASK) addrTags |= 1;
        if (intrStat & SMBUS_IC_INTR_SLV_ADDR2_TAG_MASK) addrTags |= 2;
        if (intrStat & SMBUS_IC_INTR_SLV_ADDR3_TAG_MASK) addrTags |= 4;
        if (intrStat & SMBUS_IC_INTR_SLV_ADDR4_TAG_MASK) addrTags |= 8;

        LOGI("SMBus Slave: SLV_ADDR_TAG - Address tag detected (tags=0x%02X)\n", addrTags);

        /* 根据IP规范，地址标签中断通常不需要特殊处理，主要是用于调试 */
        (void)regBase->icClrSlvAddrTag;  ///< 清除 SLV_ADDR_TAG 中断
    }

    /* 2. 处理 RX_FULL（接收数据）- 第二优先级 */
    if (intrStat & SMBUS_IC_INTR_RX_FULL_MASK) {
        LOGI("SMBus Slave: RX_FULL - Receiving data from Master (Priority 2)\n");

        U32 rxValid = regBase->icRxflr;
        U32 readCount = 0;
        U8 val = 0;

        /* 读取FIFO数据 */
        while (readCount < rxValid && readCount < SMBUS_SLAVE_BUF_LEN) {
            U32 tmp = regBase->icDataCmd.value;
            if (tmp & SMBUS_IC_DATA_CMD_FIRST_DATA_BYTE_MASK) {
                smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_WRITE_REQ,
                                      &pDrvData->pSmbusDev.slaveRxBuf,
                                      pDrvData->pSmbusDev.slaveValidRxLen);
            }

            val = (U8)tmp;
            if (pDrvData->pSmbusDev.slaveValidRxLen < SMBUS_SLAVE_BUF_LEN) {                   
                pDrvData->pSmbusDev.slaveRxBuf[pDrvData->pSmbusDev.slaveValidRxLen++] = val;
                __asm__ volatile("dsb sy" : : : "memory");
            }
            readCount++;
        }

        ///< LOGD("SMBus Slave: Read %u bytes from FIFO,tmp:%d, rx_valid:%d\n", readCount, 
             ///< regBase->icDataCmd.value, rxValid);
    }

    /* 3. 处理 RD_REQ（Master 读取 Slave 时）- 第三优先级 */
    if (intrStat & SMBUS_IC_INTR_RD_REQ_MASK) {
        LOGD("SMBus Slave: RD_REQ - Master requests data\n");

        /* 立即填充 TX FIFO */
        U8 txData = 0xA5;
        if (pDrvData->pSmbusDev.slaveTxBuf != NULL &&
            pDrvData->pSmbusDev.slaveValidTxLen > 0) {
            static U32 txIndex = 0;
            txData = pDrvData->pSmbusDev.slaveTxBuf[txIndex % pDrvData->pSmbusDev.slaveValidTxLen];
            txIndex++;
        }
        /* 立即写入 */
        regBase->icDataCmd.value = (U32)txData;

        /* 清除 RD_REQ */
        (void)regBase->icClrRdReq;

        /* 关键修复：RD_REQ表示Master读取Slave，需要启用TX_EMPTY继续发送数据 */
        regBase->icIntrMask.value |= SMBUS_IC_INTR_TX_EMPTY_MASK;
        LOGD("SMBus Slave: RD_REQ - TX_EMPTY enabled for continued Master read (mask=0x%08X)\n",
             regBase->icIntrMask.value);

        LOGD("SMBus Slave: Sent 0x%02X\n", txData);

        /* 不要 return！继续处理其他中断 */
    }

    /* 4. 处理 TX_EMPTY（仅在 Master 读取 Slave 时，第五优先级） */
    if (intrStat & SMBUS_IC_INTR_TX_EMPTY_MASK) {
        /* 改进：增强TX_EMPTY处理逻辑，添加更严格的状态检查 */
        U32 status = regBase->icStatus.value;
        U32 slaveActivity = (status >> 6) & 1;  // SLV_ACTIVITY bit
        U32 rxFifoLevel = regBase->icRxflr;     // RX FIFO level
        U32 txFifoLevel = regBase->icTxflr;     // TX FIFO level

        /* 改进：只有在Master真正读取且无RX数据时才处理TX_EMPTY */
        if (slaveActivity && rxFifoLevel == 0) {
            /* Slave 正在被读取，填充 TX FIFO */
            LOGD("SMBus Slave: TX_EMPTY - refilling for Master read (rxLevel=%d, txLevel=%d)\n",
                 rxFifoLevel, txFifoLevel);

            U8 txData = 0xFF;
            if (pDrvData->pSmbusDev.slaveTxBuf != NULL &&
                pDrvData->pSmbusDev.slaveValidTxLen > 0) {
                static U32 txIndex = 0;
                txData = pDrvData->pSmbusDev.slaveTxBuf[txIndex % pDrvData->pSmbusDev.slaveValidTxLen];
                txIndex++;
            }

            regBase->icDataCmd.value = (U32)txData;
            LOGD("SMBus Slave: Refilled 0x%02X\n", txData);
        } else {
            /* Master 正在写入 Slave，TX_EMPTY 是伪中断 */
            LOGW("SMBus Slave: Spurious TX_EMPTY during Master write - ignoring (activity=%d, rxLevel=%d)\n",
                 slaveActivity, rxFifoLevel);

            /* 改进：更彻底的伪中断处理 */
            regBase->icIntrMask.value &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            LOGD("SMBus Slave: TX_EMPTY interrupt disabled and status cleared\n");
        }
    }

    /* 3. 处理 STOP_DET（事务结束，第四优先级） */
    if (intrStat & SMBUS_IC_INTR_STOP_DET_MASK) {
        LOGI("SMBus Slave: STOP_DET - Transaction completed (Priority 4)\n");

        /* 清除写状态并触发写完成事件 */
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;

            LOGD("SMBus Slave: STOP_DET - Transaction completed, %u bytes available\n",
                 pDrvData->pSmbusDev.slaveValidRxLen);

            /* 调试：显示缓冲区内容 */
            if (pDrvData->pSmbusDev.slaveValidRxLen > 0 && pDrvData->pSmbusDev.slaveRxBuf) {
                LOGD("SMBus Slave: Buffer data: [0x%02X, 0x%02X, 0x%02X, ...]\n",
                     pDrvData->pSmbusDev.slaveRxBuf[0],
                     pDrvData->pSmbusDev.slaveRxBuf[1] % (pDrvData->pSmbusDev.slaveValidRxLen > 1 ? pDrvData->pSmbusDev.slaveRxBuf[1] : 0),
                     pDrvData->pSmbusDev.slaveRxBuf[2] % (pDrvData->pSmbusDev.slaveValidRxLen > 2 ? pDrvData->pSmbusDev.slaveRxBuf[2] : 0));
            }

            smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_WRITE_REQ,
                                  &pDrvData->pSmbusDev.slaveRxBuf,
                                  pDrvData->pSmbusDev.slaveValidRxLen);
        }

        /* 清除读状态 */
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_READ_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
        }

        /* 改进：重新启用 TX_EMPTY以准备下一次读操作 */
        regBase->icIntrMask.value |= SMBUS_IC_INTR_TX_EMPTY_MASK;
        LOGD("SMBus Slave: STOP_DET - TX_EMPTY re-enabled (mask=0x%08X)\n", regBase->icIntrMask.value);

        /* 清除 STOP_DET 中断 */
        (void)regBase->icClrStopDet;
    }

    /* 4. 处理其他中断（较低优先级） */
    if (intrStat & SMBUS_IC_INTR_TX_ABRT_MASK) {
        LOGD("SMBus Slave: TX_ABRT\n");
        (void)regBase->icClrTxAbrt;

        /* 清除写状态 */
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
        }

        /* 触发完成事件，传递实际接收到的数据 */
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_DONE,
                              pDrvData->pSmbusDev.slaveRxBuf,
                              pDrvData->pSmbusDev.slaveValidRxLen);
        pDrvData->slaveTransferActive = 0;

        /* 释放信号量 */
        smbusReleaseSemaphore(&pDrvData->pSmbusDev, "SMBus Slave");
    }

    if (intrStat & SMBUS_IC_INTR_RX_DONE_MASK) {
        LOGD("SMBus Slave: RX_DONE\n");
        (void)regBase->icClrRxDone;
    }

    /* 6. 处理 SMBus 特定中断 */
    if (smbusIntrStat) {
        smbusHandleSlaveSpecificInterrupts(pDrvData, regBase, smbusIntrStat);
    }

    /* 7. 清除中断 */
    if (intrStat != 0) {
        //regBase->icIntrStat.value = intrStat;
        /* 改进：立即清除相关状态寄存器以防止连续伪中断 */
        (void)regBase->icIntrStat.value;  /* 读取中断状态寄存器以清除状态 */
    }
}

/* ======================================================================== */
/*                    Core Interrupt Handler                                 */
/* ======================================================================== */

/**
 * @brief SMBus interrupt service routine (unified I2C/SMBus handler)
 * @details Handles all SMBus and I2C interrupt events. This function provides
 *          a unified interrupt handler that supports both master and slave modes
 *          based on SmbusMode_e. It integrates I2C DW ISR LOGEc with SMBus
 *          protocol handling.
 * @param[in] arg Pointer to driver data structure
 * @return void
 *
 * @note This function is called from the interrupt context
 * @note Uses SmbusMode_e to differentiate between master/slave interrupt handling
 * @note Integrates I2C ISR LOGEc with SMBus protocol handling
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
    LOGE("SMBus ISR Entry: devId=%d, enabled=0x%08X, rawIntr=0x%08X\n",
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

    /* Handle interrupts based on current mode - FIXED: Use raw interrupt status */
    if (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) {
        LOGI("SMBus ISR: Handling MASTER mode interrupts\n");
        smbusHandleMasterInterrupt(pDrvData, regBase, rawIntr, smbusIntrStat.value);
    } else {
        LOGI("SMBus ISR: Handling SLAVE mode interrupts\n");
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
 * [CORE] Core protocol layer - initialization and core protocol LOGEc
 */
S32 smbusInit(DevList_e devId, SmbusUserConfigParam_s *config)
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
    if(config == NULL){
       ret = -EXDEV;
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
    pDrvData->sbrCfg.regAddr = config->base;
    pDrvData->sbrCfg.irqNo = config->irqNo;
    pDrvData->sbrCfg.irqPrio = config->irqPrio;
    pDrvData->sbrCfg.masterMode = config->masterMode;//DW_SMBUS_MODE_MASTER;
    pDrvData->sbrCfg.interruptMode = config->interruptMode;    
    pDrvData->sbrCfg.speed = config->busSpeedHz;
    pDrvData->sbrCfg.addrMode = config->addrMode;        
    pDrvData->sbrCfg.slaveAddrHigh = 0;
    pDrvData->sbrCfg.slaveAddrLow = config->slaveAddrLow;
    pDrvData->sbrCfg.enSmbus = 1;
    LOGD("%s: Using test configuration:%d\n", __func__, pDrvData->sbrCfg.masterMode);
#endif
    /* Step 2: Clear all pending interrupts before installing interrupt handler */
    smbusClearInterrupts((volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr, 0xFFFFFFFF);
    LOGD("%s: All interrupts cleared before handler installation\n", __func__);

    /* Step 3: Install interrupt handler (not modularized as requested) */
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

    /* Step 4: Enable peripheral clock if needed */
    ret = peripsClockEnable(devId);
    if (ret != EXIT_SUCCESS && ret != -EINVAL) {
        LOGE("%s: clock enable failed, ret=%d\n", __func__, ret);
        ret = -EIO;
        goto disable_vector;
    }
    /* Step 5: Perform peripheral reset FIRST - before any hardware configuration */
    /* This is critical because peripsReset clears all registers to default values */
    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGW("%s: reset failed\n", __func__);
        /* Continue even if reset fails */
    }

    /* Step 6: Initialize hardware using HAL probe functions AFTER reset */
    ret = smbusInitializeHardware(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Hardware initialization failed, ret=%d\n", __func__, ret);
        goto freeMem;
    }

    /* Step 7: Allocate slave buffers for slave mode */
    ret = smbusInitSlaveResources(pDrvData);
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
    if (config->isArpEnable) {
        ret = smbusInitializeArpMaster(pDrvData);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: ARP Master initialization failed, ret=%d\n", __func__, ret);
            goto disable_vector;
        }
    }
    /* Step 10: Install driver data with system */
    ret = drvInstall(devId, pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: drvInstall failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    LOGE("SMBus device %d initialized successfully in %s mode, semaphoreId=%d\n",
         devId, (pDrvData->sbrCfg.masterMode == 1) ? "master" : "slave",
         pDrvData->pSmbusDev.semaphoreId);

    /* DEBUG: Dump key registers before exit to help debug RX_FULL issues */
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    if (regBase != NULL) {
        LOGE("[DEBUG] SMBus Init Register Dump - Device %d:\n", devId);

        /* Dump IC_CON register */
        SmbusIcConReg_u icCon;
        icCon.value = regBase->icCon.value;
        LOGE("[DEBUG] IC_CON (0x%08X): masterMode=%u, speed=%u, ic10bitaddrSlave=%u, ic10bitaddrMaster=%u, icRestartEn=%u, icSlaveDisable=%u\n",
             icCon.value, icCon.fields.masterMode, icCon.fields.speed, icCon.fields.ic10bitaddrSlave,
             icCon.fields.ic10bitaddrMaster, icCon.fields.icRestartEn, icCon.fields.icSlaveDisable);

        /* Dump IC_SAR register (only relevant in slave mode) */
        SmbusIcSarReg_u icSar;
        icSar.value = regBase->icSar.value;
        LOGE("[DEBUG] IC_SAR (0x%08X): slaveAddress=0x%03X\n", icSar.value, icSar.fields.icSar);

        /* Dump IC_INTR_MASK register */
        SmbusIcIntrMask_u icIntrMask;
        icIntrMask.value = regBase->icIntrMask.value;
        LOGE("[DEBUG] IC_INTR_MASK (0x%08X): rxUnder=%u, rxOver=%u, rxFull=%u, txOver=%u, txEmpty=%u, rdReq=%u, txAbrt=%u, rxDone=%u\n",
             icIntrMask.value, icIntrMask.fields.rxUnder, icIntrMask.fields.rxOver, icIntrMask.fields.rxFull,
             icIntrMask.fields.txOver, icIntrMask.fields.txEmpty, icIntrMask.fields.rdReq,
             icIntrMask.fields.txAbrt, icIntrMask.fields.rxDone);
        LOGE("[DEBUG] IC_INTR_MASK continued: activity=%u, stopDet=%u, startDet=%u, genCall=%u, restartDet=%u\n",
             icIntrMask.fields.activity, icIntrMask.fields.stopDet, icIntrMask.fields.startDet,
             icIntrMask.fields.genCall, icIntrMask.fields.restartDet);

        /* Dump IC_STATUS register for additional context */
        SmbusIcStatusReg_u icStatus;
        icStatus.value = regBase->icStatus.value;
        LOGE("[DEBUG] IC_STATUS (0x%08X): activity=%u, tfnf=%u, tfe=%u, rfne=%u, rff=%u\n",
             icStatus.value, icStatus.fields.activity, icStatus.fields.tfnf, icStatus.fields.tfe,
             icStatus.fields.rfne, icStatus.fields.rff);

        /* Dump IC_ENABLE register */
        SmbusIcEnableReg_u icEnable;
        icEnable.value = regBase->icEnable.value;
        LOGE("[DEBUG] IC_ENABLE (0x%08X): enable=%u, abort=%u, icSarEn=%u\n",
             icEnable.value, icEnable.fields.enable, icEnable.fields.abort, icEnable.fields.icSarEn);

        LOGE("[DEBUG] End of Register Dump - Device %d\n", devId);
    } else {
        LOGE("[DEBUG] ERROR: regBase is NULL for device %d\n", devId);
    }

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
    SmbusDev_s *dev = NULL;

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
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
    dev = &pDrvData->pSmbusDev;
    /* Destroy semaphore if created */
    smbusDeleteSemaphore(&pDrvData->pSmbusDev, "Deinit");

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
        if (dev->halOps != NULL && dev->halOps->disable != NULL) {
            dev->halOps->disable(dev);
        }
    }

    /* Free driver data using centralized cleanup function */
    smbusFreeDriverData(pDrvData);
    /* Unregister driver data */
    drvUninstall(devId);

    LOGE("SMBus device %d deinitialized successfully\r\n", devId);

unlock:
    devUnlockByDriver(devId);
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
    SMBUS_CHECK_PARAM(param == NULL, -EINVAL, "param is NULL");

    /* Validate target mode parameter */
    SMBUS_CHECK_PARAM(param->targetMode != DW_SMBUS_MODE_MASTER && param->targetMode != DW_SMBUS_MODE_SLAVE,
                      -EINVAL, "Invalid target mode %d", param->targetMode);

    /* Validate mode-specific parameters */
    if (param->targetMode == DW_SMBUS_MODE_SLAVE) {
        /* Validate slave address - I2C addresses 0x00 and 0x78-0x7F are reserved */
        SMBUS_CHECK_PARAM(param->config.slaveConfig.slaveAddr == SMBUS_GENERAL_CALL_ADDR ||
                          (param->config.slaveConfig.slaveAddr >= SMBUS_RESERVED_ADDR_START &&
                           param->config.slaveConfig.slaveAddr <= SMBUS_RESERVED_ADDR_END),
                          -EINVAL, "Invalid slave address 0x%02X (reserved address range)",
                          param->config.slaveConfig.slaveAddr);

        /* Validate ARP enable flag */
        SMBUS_CHECK_PARAM(param->config.slaveConfig.enableArp > SMBUS_BOOL_MAX,
                          -EINVAL, "Invalid ARP enable flag %d (max: %d)",
                          param->config.slaveConfig.enableArp, SMBUS_BOOL_MAX);
    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        /* Validate address mode */
        SMBUS_CHECK_PARAM(param->config.masterConfig.addrMode > SMBUS_ADDR_MODE_10BIT,
                          -EINVAL, "Invalid master address mode %d (max: %d)",
                          param->config.masterConfig.addrMode, SMBUS_ADDR_MODE_10BIT);

        /* Validate master speed - should be one of the supported SMBus speeds */
        SMBUS_CHECK_PARAM(param->config.masterConfig.speed != SMBUS_SPEED_100KHZ &&
                          param->config.masterConfig.speed != SMBUS_SPEED_400KHZ &&
                          param->config.masterConfig.speed != SMBUS_SPEED_1MHZ,
                          -EINVAL, "Invalid master speed %u Hz. Supported speeds: %u Hz, %u Hz, %u Hz",
                          param->config.masterConfig.speed,
                          SMBUS_SPEED_100KHZ, SMBUS_SPEED_400KHZ, SMBUS_SPEED_1MHZ);
    }

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< Get driver data
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto exit;
    }

    ///< Get device pointer from driver data 
    dev = &pDrvData->pSmbusDev;
    if (dev == NULL) {
        LOGE("%s(): Device pointer is NULL\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    /* Update device configuration from switch parameters */
    /* Store timeout in transferTimeout field for temporary storage */
    dev->transferTimeout = param->timeout;  

    /* Copy mode-specific configuration and manage resources */
    if (param->targetMode == DW_SMBUS_MODE_SLAVE) {
        /* Copy slave configuration */
        dev->slaveAddr = param->config.slaveConfig.slaveAddr;
        LOGD("%s(): Slave config - addr=0x%02X, arp=%d\n", __func__,
             dev->slaveAddr, param->config.slaveConfig.enableArp);
        /* Note: enableArp could be stored in flags field if needed */
        /* Update driver configuration to reflect slave mode */
        pDrvData->sbrCfg.masterMode = SMBUS_MODE_SLAVE;  /* Set to slave mode */

        /* Initialize slave resources */
        ret = smbusInitSlaveResources(pDrvData);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s(): Failed to initialize slave resources, ret=%d\n", __func__, ret);
            goto exit;
        }

    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        /* Copy master configuration */
        dev->addrMode = param->config.masterConfig.addrMode;
        dev->clkRate = param->config.masterConfig.speed;
        /* Timing parameters will be configured in smbusConfigureMaster() */
        LOGD("%s(): Master config - addrMode=%d, speed=%u, clkRate=%u\n", __func__,
             dev->addrMode, param->config.masterConfig.speed, dev->clkRate);

        /* Free slave resources when switching to master mode */
        smbusFreeSlaveResources(pDrvData);

        /* Update driver configuration to reflect master mode */
        pDrvData->sbrCfg.masterMode = 1;  /* Set to master mode */
    }

    /* Get HAL operations for mode switching */
    if (dev->halOps == NULL || dev->halOps->modeSwitchCore == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto exit;
    }
    /* Call HAL layer mode switch core function */
    ret = dev->halOps->modeSwitchCore(dev, param->targetMode);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): HAL mode switch failed, ret=%d\n", __func__, ret);
        goto exit;
    }

    LOGD("%s(): Mode switching executed successfully\n", __func__);

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

exit:
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

        LOGE("%s(): ARP failure callback registered successfully for device %d\n",
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
 * @brief 注册SMBus设备通用事件回调函数
 * @param devId SMBus设备ID
 * @param callback 回调函数指针
 * @param userData 用户数据指针，会在回调时传回
 * @return 0 成功, 负值 失败
 */
S32 smbusRegisterCallback(DevList_e devId, SmbusCallback_t callback, void *userData)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;

    /* 参数有效性检查 */
    if (callback == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* 使用smbusDrvLockAndCheck统一检查驱动匹配、初始化状态和获取设备锁 */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* 获取驱动数据 */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    /* 注册回调函数和用户数据 */
    pDrvData->callback.cb = callback;
    pDrvData->callback.userData = (SmbusEventData_u*)userData;

    /* 释放设备锁 */
    devUnlockByDriver(devId);

    LOGI("%s(): SMBus回调函数注册成功，设备ID: %d\n", __func__, devId);
    goto exit;

unlock:
    /* 释放设备锁 */
    devUnlockByDriver(devId);

exit:
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): SMBus回调函数注册失败，ret=%d，设备ID: %d\n", __func__, ret, devId);
    }

    return ret;
}

/**
 * @brief 注销SMBus设备通用事件回调函数
 * @param devId SMBus设备ID
 * @param callback 回调函数指针
 * @return 0 成功, 负值 失败
 */
S32 smbusUnregisterCallback(DevList_e devId, SmbusCallback_t callback)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;

    /* 参数有效性检查 */
    if (callback == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* 使用smbusDrvLockAndCheck统一检查驱动匹配、初始化状态和获取设备锁 */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* 获取驱动数据 */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    /* 检查是否是同一个回调函数 */
    if (pDrvData->callback.cb == callback) {
        pDrvData->callback.cb = NULL;
        pDrvData->callback.userData = NULL;
        LOGI("%s(): SMBus回调函数注销成功，设备ID: %d\n", __func__, devId);
    } else {
        ret = -ENOENT;  /* 回调函数不匹配 */
        LOGW("%s(): SMBus回调函数注销失败，回调函数不匹配，设备ID: %d\n", __func__, devId);
    }

    /* 释放设备锁 */
    devUnlockByDriver(devId);
    goto exit;

unlock:
    /* 释放设备锁 */
    devUnlockByDriver(devId);

exit:
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): SMBus回调函数注销失败，ret=%d，设备ID: %d\n", __func__, ret, devId);
    }

    return ret;
}

/**
 * @brief Core ARP get UDID function
 * [CORE] Core protocol layer - initialization and core protocol LOGEc
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

    SMBUS_CHECK_PARAM(udid == NULL, -EINVAL, "udid is NULL");

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        LOGE("%s: getSmbusReg failed\n", __func__);
        ret = -ENODEV;
        goto exit;
    }

    LOGD("%s: Getting UDID from directed address 0x%02X\n", __func__, address);

    /* Step 1: Set target address to specified address */
    regBase->icTar.fields.icTar = (address & 0x7F);

    /* Step 2: Send Get UDID command */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL || halOps->checkTxReady == NULL) {
        LOGE("%s: HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto exit;
    }
    ret = halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for Get UDID command\n", __func__);
        ret = -ETIMEDOUT;
        goto exit;
    }
    regBase->icDataCmd.fields.dat = getUdidCmd;

    /* Step 3: Send Byte Count = 0 (Read command) */
    ret = halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for byte count\n", __func__);
        ret = -ETIMEDOUT;
        goto exit;
    }
    regBase->icDataCmd.value = (0 & 0xff) | (1 << 8) | (1 << 10); /* Read + STOP */

    /* Step 4: Read Byte Count */
    ret = halOps->checkRxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: RX ready timeout for byte count\n", __func__);
        ret = -ETIMEDOUT;
        goto exit;
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
            ret = -ETIMEDOUT;
            goto exit;
        }
        udidData[i] = (U8)regBase->icDataCmd.value;
    }

    /* Step 6: Read PEC */
    ret = halOps->checkRxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: RX ready timeout for PEC\n", __func__);
        ret = -ETIMEDOUT;
        goto exit;
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
        ret = -EIO;
        goto exit;
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

exit:
    return ret;
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
 */
U8 smbusPecPktConstruct(U8 addr7bitIn, bool isWrite, const U8 *pData, U32 count)
{
    if ((pData == NULL) && (count > 0)) {
        LOGE("pData is NULL but count > 0");
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

    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL,
                           "pDrvData is NULL or pDrvData->sbrCfg.regAddr is NULL");

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
    LOGE("SMBus Master timeout recovery completed successfully\n");

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

    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL,
                           "pDrvData is NULL or pDrvData->sbrCfg.regAddr is NULL");

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
    LOGE("SMBus Slave timeout recovery completed successfully\n");

    return;
}