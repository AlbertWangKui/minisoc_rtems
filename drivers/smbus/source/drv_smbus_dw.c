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
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
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
                ret = -ENOMEM;
                goto exit;
            }
            pDrvData->pSmbusDev.slaveValidTxLen = 0;
            pDrvData->pSmbusDev.txIndex = 0;

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
 * @brief 将软件 UDID 结构体写入硬件寄存器
 * @note 必须在使能 Slave 之前调用
 */
static void smbusWriteHwUdid(volatile SmbusRegMap_s *regBase, const SmbusUdid_s *udid)
{
    /* * SMBus UDID 顺序 (16 bytes):
     * Byte 0: Device Capabilities
     * Byte 1: Version Revision
     * Byte 2-3: Vendor ID
     * ...
     * * DW IP 寄存器通常按 Little Endian 填充：
     * Word0 [7:0]   = Byte 0
     * Word0 [15:8]  = Byte 1
     * Word0 [23:16] = Byte 2
     * Word0 [31:24] = Byte 3
     */
    const U8 *pBytes = (const U8 *)udid;
    /* Word 0: Bytes 0-3 */
    regBase->icSmbusUdidWord0.value = (U32)pBytes[0] | 
                                      ((U32)pBytes[1] << 8) | 
                                      ((U32)pBytes[2] << 16) | 
                                      ((U32)pBytes[3] << 24);

    /* Word 1: Bytes 4-7 */
    regBase->icSmbusUdidWord1.value = (U32)pBytes[4] | 
                                      ((U32)pBytes[5] << 8) | 
                                      ((U32)pBytes[6] << 16) | 
                                      ((U32)pBytes[7] << 24);

    /* Word 2: Bytes 8-11 */
    regBase->icSmbusUdidWord2.value = (U32)pBytes[8] | 
                                      ((U32)pBytes[9] << 8) | 
                                      ((U32)pBytes[10] << 16) | 
                                      ((U32)pBytes[11] << 24);

    /* Word 3: Bytes 12-15 */
    regBase->icSmbusUdidWord3.value = (U32)pBytes[12] | 
                                      ((U32)pBytes[13] << 8) | 
                                      ((U32)pBytes[14] << 16) | 
                                      ((U32)pBytes[15] << 24);
    
    LOGD("SMBus Hardware UDID Configured.\n");
    /* 增加调试打印，确认写入成功 */
    LOGD("[DEBUG] Configured HW UDID Registers:\n");
    LOGD("  UDID0: 0x%08X\n", regBase->icSmbusUdidWord0.value);
    LOGD("  UDID1: 0x%08X\n", regBase->icSmbusUdidWord1.value);
    LOGD("  UDID2: 0x%08X\n", regBase->icSmbusUdidWord2.value);
    LOGD("  UDID3: 0x%08X\n", regBase->icSmbusUdidWord3.value);
}
/**
 * @brief Get SMBus device configuration from SBR
 * @param devId Device identifier
 * @param sbrCfg Pointer to SBR configuration structure
 * @return EXIT_SUCCESS on success, negative error code on failure
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
 * @brief Convert I2C 7-bit address to read/write address byte
 * @details Converts a 7-bit I2C address to an 8-bit address byte with the
 *          appropriate read/write bit set.
 * @param[in] addrIn 7-bit I2C address (0-127)
 * @param[in] isWrite True for write operation, false for read operation
 * @return 8-bit address with R/W bit set
 *
 * @note Left-shifts address by 1 and adds R/W bit
 * @note Write operation: R/W bit = 0, Read operation: R/W bit = 1
 */
static const U8 i2cAddrConvert(U8 addrIn, bool isWrite)
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
static U8 smbusPecPktConstruct(U8 addr7bitIn, bool isWrite, const U8 *pData, U32 count)
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

/**
 * @brief Get SMBus register base address
 * @param devId Device identifier
 * @param pCtrlReg Pointer to register map pointer
 * @return EXIT_SUCCESS on success, negative error code on failure
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
    pDrvData->callback = NULL;           /* Initialize callback pointer to NULL */
    pDrvData->userData = NULL;         /* Initialize user data to dataNull */

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
 */
static S32 smbusInitializeHardware(SmbusDrvData_s *pDrvData)
{
    S32 ret = 0;
    U32 features = 0;
    
    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL, "pDrvData is NULL");
    SmbusDev_s *pDev = &pDrvData->pSmbusDev;
    features = pDrvData->sbrCfg.enSmbus;

    /* Configure SMBus device structure based on SBR settings */
        pDrvData->pSmbusDev = (SmbusDev_s){
        .regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr,
        .addrMode = pDrvData->sbrCfg.addrMode,
        .slaveAddr = pDrvData->sbrCfg.slaveAddrLow,
        .irq = pDrvData->sbrCfg.irqNo,
        .channelNum = pDrvData->devId,
        .workMode = (pDrvData->sbrCfg.interruptMode == 1) ? 0 : 1,
        .smbFeatures = {
            .pecEnb        = (features & SMBUS_FEATURE_PEC)         ? 1 : 0,
            .arpEnb        = (features & SMBUS_FEATURE_ARP)         ? 1 : 0,
            .smbAlertEnb   = (features & SMBUS_FEATURE_ALERT)       ? 1 : 0,
            .hostNotifyEnb = (features & SMBUS_FEATURE_HOST_NOTIFY) ? 1 : 0,
            .quickCmdEnb   = (features & SMBUS_FEATURE_QUICK_CMD)   ? 1 : 0,
            .reserved      = 0
        },
        .restartEnb = 1,             ///< requires Restart condition
        .transferTimeout = SMBUS_TRANSFER_TIMEOUT_MS,
        .halOps = smbusGetHalOps()  /* Register HAL operations during initialization */
    };
    pDrvData->enableArp = pDev->smbFeatures.arpEnb;
    static const U32 sSmbusClkRateTable[SMBUS_SPEED_MODE_MAX] = {
        [SMBUS_SPEED_MODE_STANDARD]  = SMBUS_MAX_STANDARD_MODE_FREQ,   /* 100000U */
        [SMBUS_SPEED_MODE_FAST]      = SMBUS_MAX_FAST_MODE_FREQ,       /* 400000U */
        [SMBUS_SPEED_MODE_FAST_PLUS] = SMBUS_MAX_FAST_MODE_PLUS_FREQ   /* 1000000U */
    };

    if (pDrvData->pSmbusDev.smbFeatures.arpEnb) {
        smbusWriteHwUdid(pDev->regBase, &pDrvData->udid);
    #if 1
        if (pDrvData->pSmbusDev.smbFeatures.quickCmdEnb) {
            LOGW("Conflict detected: ARP enabled, Auto-disabling Quick Command to ensure ARP works.\n");
            pDrvData->pSmbusDev.smbFeatures.quickCmdEnb = 0; /* 强制关闭，优先保证 ARP */
        }
    #endif
   
    }
    /* Default to Fast mode if invalid speed setting */
    const U32 defaultClkRate = SMBUS_MAX_FAST_MODE_FREQ;

    pDev->clkRate = (pDrvData->sbrCfg.speed < SMBUS_SPEED_MODE_MAX)
                                 ? sSmbusClkRateTable[pDrvData->sbrCfg.speed]
                                 : defaultClkRate;

    /* Configure SMBus controller based on mode using I2C-compatible probe functions */
    if (pDrvData->sbrCfg.masterMode == SMBUS_MODE_MASTER) {
        /* OPTIMIZATION: Allocate master RX buffer only if using legacy interrupt mode */
        if (pDrvData->sbrCfg.interruptMode == 1) {  
            pDrvData->rxBufferSize = SMBUS_MAX_BUFFER_SIZE;
            pDrvData->rxBuffer = (U8 *)malloc(pDrvData->rxBufferSize);
            memset(pDrvData->rxBuffer, 0, pDrvData->rxBufferSize);
            if (pDrvData->rxBuffer == NULL) {
                LOGE("%s: Failed to allocate master RX buffer of size %d\n", __func__, pDrvData->rxBufferSize);
                return -ENOMEM;
            }
            pDrvData->rxLength = 0;
            LOGD("%s: Master RX buffer allocated for legacy mode (size=%d)\n", __func__, pDrvData->rxBufferSize);
        } else {
            /* polling mode doesn't need global rxBuffer - data goes directly to message buffers */
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
    pDrvData->pSmbusDev.restartEnb = 1;

    /* Create semaphore for synchronization */
    ret = smbusCreateSemaphore(&pDrvData->pSmbusDev, "Init");
    if (ret != EXIT_SUCCESS) {
        return ret;  // Error already logged
    }
exit:
    return ret;
}

static inline void clearInterrupts(volatile SmbusRegMap_s *regBase, U32 mask)
{
    U32 bit;
    volatile U32 *reg_ptr;
    uintptr_t base_addr, reg_offset;

    /* 使用位扫描快速定位 */
    while (mask != 0) {
        bit = __builtin_ctz(mask);  /* 获取最低置位位索引 */

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
 */
static void smbusHandleMasterInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                       U32 intrStat, U32 smbusIntrStat)
{
    DevList_e devId = pDrvData->devId;
    SmbusDev_s *dev = &pDrvData->pSmbusDev;
    SmbusEventData_u eventData;
    Bool needCallback = false;
    Bool asyncComplete = false;
    Bool handled = false; /* 新增：标记中断是否已被 Async 逻辑处理 */

    /* ========== ASYNC MODE: Handle async interrupt transfers ========== */
    LOGD("SMBus Master: Checking async mode - interruptMode=%d, status=0x%08X, active=%d\n",
         pDrvData->sbrCfg.interruptMode, pDrvData->pSmbusDev.status,
         (pDrvData->pSmbusDev.status & SMBUS_STATUS_ACTIVE) ? 1 : 0);
    LOGD("SMBus Master: Device addresses - pDrvData=%p, pSmbusDev=%p, regBase=%p\n",
         (void*)pDrvData, (void*)&pDrvData->pSmbusDev, (void*)pDrvData->pSmbusDev.regBase);
    if (intrStat & SMBUS_INTR_ACTIVITY) {
        /* 1. 读取清除寄存器 */
        (void)regBase->icClrActivity;  
        /* 2. ！！！关键：在 Mask 中禁用它，防止风暴！！！ */
        U32 currentMask = regBase->icIntrMask.value;
        if (currentMask & SMBUS_INTR_ACTIVITY) {
            regBase->icIntrMask.value = currentMask & ~SMBUS_INTR_ACTIVITY;
            (void)regBase->icClrActivity;
            LOGW("SMBus Master: ACTIVITY interrupt storm detected - Disabled mask 0x100\n");
        }
    }
    if (intrStat & SMBUS_INTR_START_DET) { 
        (void)regBase->icClrStartDet;     
        LOGD("SMBus Master: START_DET cleared\n");
        handled = true;
    }
    if (pDrvData->sbrCfg.interruptMode == 1) {
        LOGD("SMBus Master: Async mode processing - status=0x%08X\n", pDrvData->pSmbusDev.status);

        /* Only process transfer LOGEc if actively transferring */
        if (pDrvData->pSmbusDev.status & (SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS)) {
            LOGD("SMBus Master: Active transfer detected - processing async interrupts\n");
            /* Async interrupt mode handling - similar to I2C implementation */
            pDrvData->txComplete = 0;

        /* Handle TX abort in async mode */
        if (intrStat & SMBUS_INTR_TX_ABRT) {
            handled = false;
        }

        /* Handle RX full in async mode - CRITICAL: Add missing RX handling */
        if (intrStat & SMBUS_INTR_RX_FULL) {
                SmbusHalOps_s *halOps = smbusGetHalOps();
                if (halOps != NULL && halOps->smbusDwRead != NULL) {
                    LOGD("SMBus Async: RX_FULL detected, rxOutstanding=%d\n", dev->rxOutstanding);
                    
                    /* 执行硬件读取，这通常会减少 dev->rxOutstanding */
                    halOps->smbusDwRead(pDrvData, pDrvData->rxBuffer, pDrvData->rxBufferSize);

                    /* [FIX 1] 补全逻辑：如果是读操作的最后一个字节，在这里结束传输！*/
                    if (dev->rxOutstanding == 0 && dev->msgWriteIdx >= dev->msgsNum) {
                        LOGI("SMBus Async: RX Complete (All bytes received). Ending transfer.\n");
                        dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
                        asyncComplete = true;
                    }
                } else {
                    /* HAL 缺失时的防御性代码，清空 FIFO 防止死循环 */
                    LOGE("SMBus Async: HAL missing, draining FIFO\n");
                    while (regBase->icRxflr > 0) (void)regBase->icDataCmd.value;
                    dev->rxOutstanding = 0;
                    dev->msgErr = -EIO;
                    asyncComplete = true; // 强制结束
                }
            handled = true;
            LOGD("SMBus Async: RX Full processed\n");
        }

            /* =================================================================================
             * 2. TX_EMPTY 处理
             * 核心修复：如果还有 RX 数据在路上，只关中断，不结束任务
             * ================================================================================= */
        if (intrStat & SMBUS_INTR_TX_EMPTY) {
                /* 先尝试发送更多数据 */
                if (dev->msgWriteIdx < dev->msgsNum) {
                    SmbusHalOps_s *halOps = smbusGetHalOps();
                    if (halOps && halOps->smbusDwXferMsg) {
                        halOps->smbusDwXferMsg(dev);
                    }
                }

                /* [FIX 2] 检查是否所有命令都发完了 */
                if (dev->msgWriteIdx >= dev->msgsNum) {
                    /* A. 必须操作：发送完了，必须禁用 TX_EMPTY 中断，否则会无限触发 */
                    U32 mask = regBase->icIntrMask.value;
                    if (mask & SMBUS_INTR_TX_EMPTY) {
                        regBase->icIntrMask.value = mask & ~SMBUS_INTR_TX_EMPTY;
                        LOGD("SMBus Async: All CMDs sent. Disabling TX_EMPTY IRQ.\n");
                    }

                    /* B. 关键判定：真的结束了吗？ */
                    if (dev->rxOutstanding > 0) {
                        /* * ！！！关键修复点！！！
                         * 还有数据没读回来 (Read 32 协议)，绝对不能设置 asyncComplete！
                         * 保持 Status = ACTIVE，直接退出，等待 RX_FULL 中断。
                         */
                        LOGD("SMBus Async: TX done, but waiting for %d RX bytes. Keeping ACTIVE.\n", dev->rxOutstanding);
                    } 
                    else {
                        /* 只有既没数据发，也没数据收，才是真的结束 */
                        LOGD("SMBus Async: Transfer fully complete (Write-only or Quick).\n");
                        dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
                        asyncComplete = true;
                    }
                }
                handled = true;
        }

        /* Check for async transfer completion */
        if (((intrStat & (SMBUS_INTR_TX_ABRT | SMBUS_INTR_STOP_DET)) || dev->msgErr)) {
            /* 只有当不需要等待更多 RX 数据，或者发生了严重错误时，才允许 STOP 结束传输 */
            /* 注意：正常的 Read 操作最后也会有 STOP，所以要在 rxOutstanding == 0 时才处理 STOP */
            if (dev->rxOutstanding == 0 || (intrStat & SMBUS_INTR_TX_ABRT) || dev->msgErr) {
                LOGD("SMBus Async: Stop/Abort detected. Finishing.\n");
                    
                /* 确保禁用 TX_EMPTY */
                U32 mask = regBase->icIntrMask.value;
                regBase->icIntrMask.value = mask & ~SMBUS_INTR_TX_EMPTY;

                if (intrStat & SMBUS_INTR_TX_ABRT) {
                    dev->xferStatus = -EIO;
                    dev->cmdErr = 1; 
                    (void)regBase->icClrTxAbrt;
                }
            /* 必须清除 STOP_DET */
            if (intrStat & SMBUS_INTR_STOP_DET) (void)regBase->icClrStopDet;
            dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
            asyncComplete = true;
            } else {
                /* 如果是 Read 操作中途的 STOP 信号（且无错误），清除标志但不结束传输（防止误判） */
                if (intrStat & SMBUS_INTR_STOP_DET) (void)regBase->icClrStopDet;
            }
            LOGE("SMBus Async: Transfer completed (abort/stop), releasing semaphore (semId=%d)\n", dev->semaphoreId);
            /* STOP检测到，说明传输完成 */
            if (dev->isQuick) {
            /* 检查是否有错误发生 (如 TX_ABRT) */
                if (intrStat & SMBUS_INTR_TX_ABRT) {
                    dev->xferStatus = -EIO; // 或者读取 icTxAbrtSource 获取详细错误
                    dev->cmdErr = 1; 
                    (void)regBase->icClrTxAbrt;
                } else {
                    dev->xferStatus = 0; // 成功
                }
                 dev->isQuick = 0; // 清除标记         
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
                currentMask &= ~(SMBUS_INTR_TX_EMPTY | SMBUS_INTR_RX_FULL |
                                SMBUS_INTR_TX_ABRT | SMBUS_INTR_STOP_DET |
                                SMBUS_INTR_ACTIVITY);
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
        if (intrStat & (SMBUS_INTR_TX_EMPTY | SMBUS_INTR_RX_FULL |
                       SMBUS_INTR_TX_ABRT | SMBUS_INTR_STOP_DET |
                       SMBUS_INTR_ACTIVITY)) {
            /* Clear interrupt bits that were handled */
            if (intrStat & SMBUS_INTR_TX_EMPTY) {
                /* CRITICAL FIX: TX_EMPTY doesn't have a dedicated clear register - it's cleared by writing data */
                /* But we need to ensure it's properly disabled to prevent continuous triggering */
                if (!asyncComplete && (dev->msgWriteIdx >= dev->msgsNum)) {
                    /* Force disable TX_EMPTY if all messages are processed but not marked complete yet */
                    U32 finalMask = regBase->icIntrMask.value;
                    finalMask &= ~SMBUS_INTR_TX_EMPTY;
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
                smbusReleaseSemaphore(dev, "SMBus Async");
                LOGD("SMBus Async: Transfer complete, clearing interrupts and exiting\n");
                return;
            }
        }
      }
    }

    /* ========== TX ABORT ========== */
    if (!handled && (intrStat & SMBUS_INTR_TX_ABRT)) {
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

        /* Prepare error callback data */
        eventData.error.errorCode = pDrvData->errorCode;
        eventData.error.statusReg = event.rawIntrStat;
        needCallback = true;

        /* Disable interrupts and mark transfer as failed */
        regBase->icIntrMask.value = 0;
        pDrvData->pSmbusDev.cmdErr = 1; 
        pDrvData->pSmbusDev.abortSource = abrtSource;
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_ACTIVE_MASK;

        if (pDrvData->sbrCfg.interruptMode == 1) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_ACTIVE;
            smbusReleaseSemaphore(&pDrvData->pSmbusDev, "Async Abort");
        }
        
        (void)regBase->icClrTxAbrt; /* 清除中断 */
        LOGE("SMBus Master TX Abort: source=0x%08X, reason=%d\n", abrtSource, event.reason);
    }

    /* ========== RX READY ========== */
    if (!handled && (intrStat & SMBUS_INTR_RX_FULL)) {
        /* CRITICAL FIX: Only execute Legacy mode when interruptMode = 0 */
        if (pDrvData->sbrCfg.interruptMode == 0) {
            LOGD("SMBus Master Legacy RX Ready - processing\n");
            smbusMasterReadData(pDrvData, regBase);

            /* Prepare RX done callback data */
            eventData.transfer.buffer = pDrvData->rxBuffer;
            eventData.transfer.len = pDrvData->rxLength;
            needCallback = true;

            LOGD("SMBus Master RX Ready: len=%d\n", pDrvData->rxLength);
        } else {
            /* In async mode, RX_FULL should be handled by the async handler above */
            LOGD("SMBus Master: RX_FULL in async mode - should be handled by async handler\n");
        }
        /* 防御性代码：非 Active 状态下如果还有 RX_FULL，说明是“幽灵数据”，必须清理 */
        if (intrStat & SMBUS_INTR_RX_FULL) {
             LOGW("SMBus Async: Ghost RX data detected while inactive. Draining...\n");
             while (regBase->icRxflr > 0) (void)regBase->icDataCmd.value;
             (void)regBase->icClrRxOver;
        }
    }
#if 1
    /* ========== TX READY ========== */
    if (!handled && (intrStat & SMBUS_INTR_TX_EMPTY)) {
        smbusMasterTransferData(pDrvData, regBase);

        /* Check if transfer is complete and handle completion */
        /* CRITICAL: Don't complete transfer if async operations are still active */
        if (pDrvData->txComplete &&
            !(pDrvData->pSmbusDev.status & (SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS))) {
            /* Transfer complete, disable TX_EMPTY interrupt */
            U32 currentMask = regBase->icIntrMask.value;
            currentMask &= ~SMBUS_INTR_TX_EMPTY;
            regBase->icIntrMask.value = currentMask;
            LOGD("SMBus Master: TX_EMPTY disabled due to transfer completion\n");
            
            if (dev->isQuick) {
                dev->xferStatus = eventData.error.errorCode; 
                dev->isQuick = 0;                // 清除标志
            }
            /* Clear device active status and release semaphore */
            LOGD("SMBus Master: Transfer completed, releasing semaphore\n");
            smbusReleaseSemaphore(&pDrvData->pSmbusDev, "SMBus Master");
        }       
        LOGD("SMBus Master TX Ready\n");
    }
#if 1
    /* Simplified STOP detection - ensure semaphore release for write operations */
    if (intrStat & SMBUS_INTR_STOP_DET || regBase->icRawIntrStat.fields.stopDet ) {
        dev->status &= ~(SMBUS_STATUS_ACTIVE | SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
        smbusReleaseSemaphore(dev, "SMBus Async");         
        /* 必须清除 STOP_DET */
        (void)regBase->icClrStopDet;
        handled = true;
    }
#endif
#endif
    /* ========== SMBus Specific Interrupts (Master) ========== */
    if (smbusIntrStat) {
        smbusHandleMasterSpecificInterrupts(pDrvData, regBase, smbusIntrStat);
    }

    /* Trigger unified callback if needed */
    if (needCallback && pDrvData->callback) {
        U32 eventType = (pDrvData->errorCode != 0) ? SMBUS_EVENT_ERROR : SMBUS_EVENT_RX_DONE;
        if (eventType == SMBUS_EVENT_RX_DONE && !eventData.transfer.buffer) {
             eventData.transfer.buffer = pDrvData->rxBuffer;
             eventData.transfer.len = pDrvData->rxLength;
        }
        pDrvData->callback(devId, eventType, &eventData, pDrvData->userData);
    }

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
 */
static S32 smbusInitializeArpMaster(SmbusDrvData_s *pDrvData)
{
    SMBUS_CHECK_PARAM_RETURN(pDrvData == NULL, -EINVAL, "pDrvData is NULL");

    /* Initialize ARP configuration with defaults using macro */
    pDrvData->arpState = SMBUS_ARP_STATE_IDLE;
    pDrvData->retryCount = 0;

    /* Initialize ARP Master context using macro */
    pDrvData->arpMaster = (SmbusArpMaster_s)SMBUS_ARP_MASTER_INIT;
    pDrvData->arpMaster.busId = pDrvData->devId - DEVICE_SMBUS0;
    LOGD("%s: ARP Master busId set to %d\n", __func__, pDrvData->arpMaster.busId);
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
    }

    /* Clear master TX buffer array - no need to free as it's a static array */
    memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
    pDrvData->pSmbusDev.masterTxBufLen = 0;
    LOGD("%s: Master TX buffer array cleared\n", __func__);
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
 */
static void smbusHandleSlaveInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                      U32 intrStat, U32 smbusIntrStat)
{
    /* 基本参数验证 */
    if (pDrvData == NULL || regBase == NULL) {
        LOGE("SMBus Slave: Invalid parameters\n");
        return;
    }
    LOGI("SMBus Slave: Interrupt triggered - intrStat=0x%08X, smbusIntrStat=0x%08X\n", intrStat, smbusIntrStat);

    /* 1. 首先处理 WR_REQ（最高优先级）- Master 写入 Slave 时触发 */
    if (intrStat & SMBUS_INTR_WR_REQ) {
        LOGI("SMBus Slave: WR_REQ - Master is writing to Slave (Priority 1) - INITIAL SETUP\n");

        /* 仅设置状态和清除中断，不在此处读取 FIFO */
        pDrvData->pSmbusDev.slaveValidRxLen = 0; // 重置计数器
        pDrvData->pSmbusDev.status |= SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
        
        /* 禁用 TX_EMPTY (保持不变，因为是 Master 写入) */
        regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;

        /* 必须最后清除 WR_REQ 中断 */
        (void)regBase->icClrWrReq; 
        LOGD("SMBus Slave: WR_REQ - TX_EMPTY disabled (Master is writing, not reading) (mask=0x%08X)\n",
             regBase->icIntrMask.value);
        ///< 立即返回，等待 RX_FULL 中断处理实际数据接收
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_WRITE_REQ,
                                pDrvData->pSmbusDev.slaveRxBuf,
                                pDrvData->pSmbusDev.slaveValidRxLen);

    }

    /* 1.5 处理 SLV_ADDRx_TAG中断 - 地址标签中断 */
    if (intrStat & (SMBUS_INTR_SLV_ADDR1_TAG | SMBUS_INTR_SLV_ADDR2_TAG |
                   SMBUS_INTR_SLV_ADDR3_TAG | SMBUS_INTR_SLV_ADDR4_TAG)) {
        U32 addrTags = 0;
        if (intrStat & SMBUS_INTR_SLV_ADDR1_TAG) addrTags |= 1;
        if (intrStat & SMBUS_INTR_SLV_ADDR2_TAG) addrTags |= 2;
        if (intrStat & SMBUS_INTR_SLV_ADDR3_TAG) addrTags |= 4;
        if (intrStat & SMBUS_INTR_SLV_ADDR4_TAG) addrTags |= 8;

        LOGI("SMBus Slave: SLV_ADDR_TAG - Address tag detected (tags=0x%02X)\n", addrTags);
        /* 根据IP规范，地址标签中断通常不需要特殊处理，主要是用于调试 */
        (void)regBase->icClrSlvAddrTag;  ///< 清除 SLV_ADDR_TAG 中断
    }

    /* 2. 处理 RX_FULL（接收数据）- 第二优先级 */
    if (intrStat & SMBUS_INTR_RX_FULL) {
        LOGI("SMBus Slave: RX_FULL - Receiving data from Master (Priority 2)\n");

        //U32 rxValid = regBase->icRxflr;
        //U32 readCount = 0;
        //U8 val = 0;
        U32 loopCount = 0;
        /* 检查 IC_STATUS 的 RFNE (Receive FIFO Not Empty) 位 */
        while ((regBase->icStatus.fields.rfne) && (loopCount < SMBUS_SLAVE_BUF_LEN)) {  
            U32 tmp = regBase->icDataCmd.value; // 读取数据会弹出 FIFO
            U8 val = (U8)tmp;
        
            if (tmp & SMBUS_IC_DATA_CMD_FIRST_DATA_BYTE_MASK) {
                LOGD("SMBus Slave: FIRST_DATA_BYTE detected: 0x%02X\n", val);
                // 如果是 ARP 命令，这可能是 Command Byte (0x01/0x02)
            }
            /* 存入缓冲区 */
            if (pDrvData->pSmbusDev.slaveValidRxLen < SMBUS_SLAVE_BUF_LEN) {                  
                pDrvData->pSmbusDev.slaveRxBuf[pDrvData->pSmbusDev.slaveValidRxLen++] = val;
                ///< __asm__ volatile("dsb st" : : : "memory");
            }   
            loopCount++;
        } 
        if (loopCount > 0) {
            LOGD("SMBus Slave: Drained %d bytes from FIFO in ISR\n", loopCount);
            /* 手动清除 RX_FULL 中断标志 (如果它被置位了的话) */
            if (intrStat & SMBUS_INTR_RX_FULL) {
                (void)regBase->icClrRxUnder; // 有些 IP 是读 Data 自动清，有些需要读 Clear 寄存器
            }
        }
    #if 0
        /* 读取FIFO数据 */
        while (readCount < rxValid && readCount < SMBUS_SLAVE_BUF_LEN) {
            U32 tmp = regBase->icDataCmd.value;
            if (tmp & SMBUS_IC_DATA_CMD_FIRST_DATA_BYTE_MASK) {
                LOGD("SMBus Slave: RX_FULL - Detected FIRST_DATA_BYTE in received data\n");
            }

            val = (U8)tmp;
            if (pDrvData->pSmbusDev.slaveValidRxLen < SMBUS_SLAVE_BUF_LEN) {                   
                pDrvData->pSmbusDev.slaveRxBuf[pDrvData->pSmbusDev.slaveValidRxLen++] = val;
                __asm__ volatile("dsb st" : : : "memory");
            }
            readCount++;
        }
        #endif
        LOGD("SMBus Slave: Read %u bytes from FIFO, rx_valid:%d\n", pDrvData->pSmbusDev.slaveValidRxLen, regBase->icRxflr);
    }

    /* 3. 处理 RD_REQ（Master 读取 Slave 时）- 第三优先级 */
    if (intrStat & SMBUS_INTR_RD_REQ) {
        LOGD("SMBus Slave: RD_REQ - Master requests data\n");
        /* 清除 RD_REQ */
        (void)regBase->icClrRdReq;
        U32 cmdStatus = regBase->icSmbusIntrStat.value;
        if (!cmdStatus) {
        /* 立即填充 TX FIFO */
         /* 只有当 txIndex 为 0 时，才认为是新的传输开始，需要准备数据 */
        if (pDrvData->pSmbusDev.txIndex == 0) {
            LOGI("SMBus Slave: RD_REQ - New transaction start, preparing data...\n");
            
            /* 调用回调准备数据，这会重置 txIndex = 0 并填充 Buffer */
            /* 假设 smbusSlaveSetResponse 在这个宏/函数内部 */
            smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_READ_REQ, 
                                   pDrvData->pSmbusDev.slaveTxBuf,
                                   pDrvData->pSmbusDev.slaveValidTxLen);
        } else {
            /* 如果 txIndex > 0，说明是数据传了一半 FIFO 空了触发的 RD_REQ */
            LOGD("SMBus Slave: RD_REQ - Continued transfer (refill), current index=%d\n", 
                 pDrvData->pSmbusDev.txIndex);
        }

        /* 3. 发送当前指针指向的数据 */
        U8 txData = 0xFF; // 默认填充
        
        ///< 检查是否有数据可发
        if (pDrvData->pSmbusDev.slaveTxBuf != NULL && 
            pDrvData->pSmbusDev.txIndex < pDrvData->pSmbusDev.slaveValidTxLen) {
            
            ///< 获取当前 txIndex 指向的字节
            txData = pDrvData->pSmbusDev.slaveTxBuf[pDrvData->pSmbusDev.txIndex];        
            ///< 写入 FIFO
            regBase->icDataCmd.value = (U32)txData;
            
            LOGD("SMBus Slave: RD_REQ - Sent Byte[%d] = 0x%02X\n", 
                 pDrvData->pSmbusDev.txIndex, txData);
            pDrvData->pSmbusDev.txIndex++; 
            
        } else {
            ///< 没有数据了，或者 Buffer 没准备好，发送 Dummy 防止总线挂死
            regBase->icDataCmd.value = 0xFF;
            LOGW("SMBus Slave: RD_REQ - No more data, sent dummy 0xFF\n");
        }
        /* 5. 启用 TX_EMPTY 以便发送剩余数据 */
        if (pDrvData->pSmbusDev.slaveValidTxLen > 1) {
            regBase->icIntrMask.value |= SMBUS_INTR_TX_EMPTY;
            LOGD("SMBus Slave: RD_REQ - TX_EMPTY enabled for remaining %d bytes\n", 
                 pDrvData->pSmbusDev.slaveValidTxLen - 1);
        } else {
            /* 如果只有1个字节，就不需要 TX_EMPTY 了 */
            regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
        }
      }
    }

    /* 4. 处理 TX_EMPTY（仅在 Master 读取 Slave 时，第五优先级） */
    if (intrStat & SMBUS_INTR_TX_EMPTY) {
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
            
            U32 txLen = pDrvData->pSmbusDev.slaveValidTxLen;
            
            while (pDrvData->pSmbusDev.txIndex < txLen) {
                /* 检查 FIFO 状态，防止溢出 */
                if (!(regBase->icStatus.fields.tfnf)) {
                    break; /* FIFO 满了，退出循环等待下一次中断 */
                }
                U8 txData = pDrvData->pSmbusDev.slaveTxBuf[pDrvData->pSmbusDev.txIndex];
                regBase->icDataCmd.value = (U32)txData;
                pDrvData->pSmbusDev.txIndex++;
            }
            if (pDrvData->pSmbusDev.txIndex >= txLen) {
                regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
                LOGD("SMBus Slave: All %u bytes queued in FIFO\n", txLen);
            }
        } else {
            /* Master 正在写入 Slave，TX_EMPTY 是伪中断 */
            LOGW("SMBus Slave: Spurious TX_EMPTY during Master write - ignoring (activity=%d, rxLevel=%d)\n",
                 slaveActivity, rxFifoLevel);

            /* 改进：更彻底的伪中断处理 */
            regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
           
            LOGD("SMBus Slave: TX_EMPTY interrupt disabled and status cleared\n");
        }
    }

    /* 3. 处理 STOP_DET（事务结束，第四优先级） */
    if (intrStat & SMBUS_INTR_STOP_DET) {
        LOGI("SMBus Slave: STOP_DET - Transaction completed (Priority 4)\n");
        /* 清除写状态并触发写完成事件 */
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;

            LOGD("SMBus Slave: STOP_DET - Transaction completed, %u bytes available\n",
                 pDrvData->pSmbusDev.slaveValidRxLen);
            
        if (pDrvData->pSmbusDev.slaveValidRxLen == 1) {
        // 触发回调，通知上层收到了 Send Byte
        // 注意：你可以定义一个新的事件类型，或者复用 WRITE_REQ
            smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLAVE_WRITE_REQ, 
                               pDrvData->pSmbusDev.slaveRxBuf, 
                               1); 
        } else if (pDrvData->pSmbusDev.slaveValidRxLen > 1) {
        ///< === 这是一个 Block Write 协议 ===        
            /* 调试：显示缓冲区内容 */
            if (pDrvData->pSmbusDev.slaveValidRxLen > 0 && pDrvData->pSmbusDev.slaveRxBuf) {
                LOGD("SMBus Slave: Buffer data: [0x%02X, 0x%02X, 0x%02X, ...]\n",
                pDrvData->pSmbusDev.slaveRxBuf[0],
                pDrvData->pSmbusDev.slaveValidRxLen > 1 ? pDrvData->pSmbusDev.slaveRxBuf[1] : 0,
                pDrvData->pSmbusDev.slaveValidRxLen > 2 ? pDrvData->pSmbusDev.slaveRxBuf[2] : 0);
            }

            smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_RX_DONE,
                                  pDrvData->pSmbusDev.slaveRxBuf,
                                  pDrvData->pSmbusDev.slaveValidRxLen);
        }else {
            LOGD("SMBus Slave: STOP_DET - No write operation in progress\n");
        }
    }
        /* 清除读状态 */
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_READ_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
        }

        /* 改进：重新启用 TX_EMPTY以准备下一次读操作 */
        regBase->icIntrMask.value |= SMBUS_INTR_TX_EMPTY;
        LOGD("SMBus Slave: STOP_DET - TX_EMPTY re-enabled (mask=0x%08X)\n", regBase->icIntrMask.value);
        pDrvData->pSmbusDev.txIndex = 0;
        /* 清除 STOP_DET 中断 */
        (void)regBase->icClrStopDet;
    }

    /* 4. 处理其他中断（较低优先级） */
    if (intrStat & SMBUS_INTR_TX_ABRT) {
        LOGD("SMBus Slave: TX_ABRT\n");
        (void)regBase->icClrTxAbrt;

        /* 清除写状态 */
        if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
            pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
        }

        /* 触发完成事件，传递实际接收到的数据 */
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_TX_DONE,
                               pDrvData->pSmbusDev.slaveTxBuf,
                               pDrvData->pSmbusDev.slaveValidTxLen);
        pDrvData->slaveTransferActive = 0;

        /* 释放信号量 */
        smbusReleaseSemaphore(&pDrvData->pSmbusDev, "SMBus Slave");
    }

    if (intrStat & SMBUS_INTR_RX_DONE) {
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
    LOGD("SMBus ISR Entry: devId=%d, enabled=0x%08X, rawIntr=0x%08X\n",
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
        smbusHandleMasterInterrupt(pDrvData, regBase, rawIntr, (intrStat & regBase->icIntrMask.value));
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
    pDrvData->sbrCfg.enSmbus = config->featureMap;
    pDrvData->udid.deviceCapabilities = EXTRACT_BYTE(config->udidWord0, 0);
    pDrvData->udid.versionRevision = EXTRACT_BYTE(config->udidWord0, 1);
    pDrvData->udid.vendorId = (U16)(config->udidWord0 >> 16); /* Bytes 2-3 */
    /* Word 1 -> Bytes 4-7 */
    pDrvData->udid.deviceId = (U16)(config->udidWord1 & 0xFFFF); /* Bytes 4-5 */
    pDrvData->udid.interface = (U16)(config->udidWord1 >> 16);    /* Bytes 6-7 */
    /* Word 2 -> Bytes 8-11 */
    pDrvData->udid.subsystemVendorId = (U16)(config->udidWord2 & 0xFFFF); /* Bytes 8-9 */
    pDrvData->udid.subsystemDeviceId = (U16)(config->udidWord2 >> 16);    /* Bytes 10-11 */
    /* Word 3 -> Bytes 12-15 */
    pDrvData->udid.vendorSpecificId = config->udidWord3;                 /* Bytes 12-15 */
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
        LOGD("%s: Interrupt handler removed successfully: %d\n", __func__, pDrvData->sbrCfg.irqNo);
    } else {
        LOGW("%s: Interrupt handler removal failed, ret=%d\n", __func__, irqRet);
    }
    LOGE("3\r\n");
    /* Disable SMBus controller */
    if (pDrvData->pSmbusDev.regBase != NULL) {
        /* Get HAL operations */
        if (dev->halOps != NULL && dev->halOps->disable != NULL) {
            dev->halOps->disable(dev);
        }
    }
    LOGE("4\r\n");
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
    pDrvData->callback = callback;
    pDrvData->userData = (SmbusEventData_u*)userData;

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
    if (pDrvData->callback == callback) {
        pDrvData->callback = NULL;
        pDrvData->userData = NULL;
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
 * @brief 通用 SMBus 传输接口 (Core API)
 * @details 处理所有 SMBus 协议细节：命令构建、Block 长度处理、PEC 计算与校验
 */
S32 smbusTransfer(DevList_e devId, SmbusXfer_s *xfer)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *pDev = NULL;

    /* ----------------------------------------------------------- */
    /* 1. 参数校验与锁保护 (Validation & Lock)                     */
    /* ----------------------------------------------------------- */
    if (xfer == NULL) {
        LOGE("%s: xfer is NULL\n", __func__);
        return -EINVAL;
    }

    /* 检查数据长度是否超限 */
    if (xfer->wLen > SMBUS_BLOCK_MAXLEN || xfer->rLen > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Length exceeds max limit %d\n", __func__, SMBUS_BLOCK_MAXLEN);
        return -EINVAL;
    }

    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        ret = -EINVAL;
        goto unlock;
    }
    
    pDev = &pDrvData->pSmbusDev;

    /* 检查 HAL 有效性 */
    if (pDev->halOps == NULL || pDev->halOps->i2cTransfer == NULL) {
        ret = -ENOTSUP;
        goto unlock;
    }

    /* ----------------------------------------------------------- */
    /* 2. 上下文准备 (Context Setup)                               */
    /* ----------------------------------------------------------- */
    /**
     * Buffer Strategy:
     * Tx: [Cmd(1)] + [Count(1)] + [Data(32)] + [PEC(1)] = Max 35 bytes
     * Rx: [Count(1)] + [Data(32)] + [PEC(1)] = Max 34 bytes
     */
    U8 txStackBuf[SMBUS_BLOCK_MAXLEN + 4] = {0};
    U8 rxStackBuf[SMBUS_BLOCK_MAXLEN + 4] = {0};

    U16 txPos = 0;
    U8 slaveAddr = xfer->addr & SMBUS_ADDRESS_MASK;
    
    /* 标志位解析 */
    bool pecEn   = (xfer->flags & SMBUS_FLAG_PEC_ENABLE) != 0;
    bool isBlock = (xfer->flags & SMBUS_FLAG_BLOCK_TRANSFER) != 0;
    bool noCmd   = (xfer->flags & SMBUS_FLAG_NO_COMMAND) != 0;
    bool isRead  = (xfer->flags & SMBUS_FLAG_READ) != 0; 
    bool isWriteOnly = (xfer->rLen == 0);

    /* ----------------------------------------------------------- */
    /* 3. 构建发送阶段 (Write Phase Construction)                  */
    /* ----------------------------------------------------------- */
    
    /* [A] 填充命令代码 (Command Code) */
    if (!noCmd) {
        txStackBuf[txPos++] = xfer->command;
    }

    /* [B] 填充数据与长度 (Data & Byte Count) */
    if (xfer->wLen > 0) {
        /* Block Write: 需显式发送 Byte Count */
        if (isBlock) {
            txStackBuf[txPos++] = (U8)xfer->wLen;
        }
        
        /* 拷贝写数据 */
        if (xfer->wBuf) {
            memcpy(&txStackBuf[txPos], xfer->wBuf, xfer->wLen);
            txPos += xfer->wLen;
        }
    }

    /* [C] 处理纯写操作的 PEC (Write Byte, Block Write, Send Byte) */
    /* 如果是读操作，PEC 将在读阶段结束后校验，此处不填充 */
    if (pecEn && isWriteOnly && txPos > 0) {
        /* 构造 PEC 输入流: Addr(W) + [Cmd] + [Count] + [Data] */
        U8 pecInput[40]; 
        U8 pIdx = 0;
        
        pecInput[pIdx++] = (U8)(slaveAddr << 1); // Addr + W
        memcpy(&pecInput[pIdx], txStackBuf, txPos);
        pIdx += txPos;

        /* 计算并追加 PEC */
        txStackBuf[txPos] = smbusPecPktConstruct(slaveAddr, TRUE, pecInput, pIdx); 
        txPos++;
    }

    /* ----------------------------------------------------------- */
    /* 4. 构建 I2C 消息 (Atomic Messages)                          */
    /* ----------------------------------------------------------- */
    SmbusMsg_s msgs[2];
    S32 msgCount = 0;

    /* Msg 0: Write Phase (Command + Data + [PEC if WriteOnly]) */
    /* Quick Command (Write/Read) 也是一个 Write Msg，只是 Len=0，buf=NULL(或指向空) */
    if (txPos > 0 || (noCmd && isWriteOnly)) {
        msgs[msgCount].addr  = slaveAddr;
        msgs[msgCount].flags = isRead ? SMBUS_M_RD : 0; /* 若是 QuickCmd Read，则标记为读 */
        msgs[msgCount].len   = txPos;
        msgs[msgCount].buf   = txStackBuf;
        msgCount++;
    }

    /* Msg 1: Read Phase (如果存在读请求) */
    U16 rxReqLen = 0;
    if (xfer->rLen > 0) {
        rxReqLen = xfer->rLen;
        
        /* Block Read: 硬件需多读 1 字节 (Byte Count) */
        if (isBlock) rxReqLen += 1; 
        
        /* PEC Enable: 硬件需多读 1 字节 (PEC) */
        if (pecEn)   rxReqLen += 1; 

        msgs[msgCount].addr  = slaveAddr;
        msgs[msgCount].flags = SMBUS_M_RD;
        if (isBlock) msgs[msgCount].flags |= SMBUS_M_RECV_LEN; /* 告诉 HAL 处理首字节长度 */
        if (pecEn) msgs[msgCount].flags |= SMBUS_M_PEC;
        msgs[msgCount].len   = rxReqLen;
        msgs[msgCount].buf   = rxStackBuf;
        msgCount++;
    }

    /* ----------------------------------------------------------- */
    /* 5. 执行传输 (Execute HAL)                                   */
    /* ----------------------------------------------------------- */
    ret = smbusExecuteTransfer(pDev, msgs, msgCount);

    if (ret < EXIT_SUCCESS) {
        LOGE("%s: Transfer failed (Addr:0x%02X, Cmd:0x%02X, Ret:%d)\n", 
             __func__, slaveAddr, xfer->command, ret);
        goto unlock;
    }

    /* ----------------------------------------------------------- */
    /* 6. 处理接收数据与 PEC 校验 (Post-Process)                   */
    /* ----------------------------------------------------------- */
    if (xfer->rLen > 0) {
        U8 *pDataStart = rxStackBuf;
        U8 actualDataLen = xfer->rLen;
        U8 receivedPec = 0;
        
        /* [A] 解析 Block Read 的长度字节 */
        if (isBlock) {
            U8 blockCount = rxStackBuf[0];
            /* 校验 Block Count 合法性 (SMBus Max 32) */
            if (blockCount > SMBUS_BLOCK_MAXLEN || blockCount == 0) {
                 LOGE("%s: Invalid Block Count %d\n", __func__, blockCount);
                 ret = -EPROTO;
                 goto unlock;
            }
            actualDataLen = blockCount;
            pDataStart = &rxStackBuf[1]; /* 数据偏移 1 字节 */
        }
        LOGD("RxStackBuf Dump (Len=%d):", xfer->rLen + 2); // Dump a safe amount
        for(U8 i=0; i < xfer->rLen + 2; i++) {
             LOGD(" %02X", rxStackBuf[i]); // Use printk or equivalent to stay on one line
        }
        LOGD("\r\n");
        /* [B] PEC 校验逻辑 */
        if (pecEn) {
            /* PEC 计算缓冲区：Addr(W)+Cmd + Addr(R)+[Count]+Data */
            U8 pecCalcBuf[72]; 
            U32 pIdx = 0;

            /* 1. 重构 Write 部分历史 (如果有) */
            if (msgCount == 2) { 
                /* Write-then-Read (例如 Read Word / Block Read) */
                pecCalcBuf[pIdx++] = (U8)(slaveAddr << 1);
                memcpy(&pecCalcBuf[pIdx], txStackBuf, txPos);
                pIdx += txPos;
                
                /* 2. 添加 Read 部分头部 */
                pecCalcBuf[pIdx++] = (U8)((slaveAddr << 1) | 1); // Addr + R
            } else {
                /* Receive Byte / Quick Read (直接读) */
                 pecCalcBuf[pIdx++] = (U8)((slaveAddr << 1) | 1); // Addr + R
            }
            
            /* 3. 添加接收到的数据 (包含 Block Count) */
            /* 这里的 payloadLen 是包含数据的总长度，但不包含 PEC 字节本身 */
            U16 payloadLen = isBlock ? (actualDataLen + 1) : actualDataLen;
            memcpy(&pecCalcBuf[pIdx], rxStackBuf, payloadLen);
            pIdx += payloadLen;

            LOGD("PEC Check: IsBlock=%d, ActualDataLen=%d, PayloadLen=%d\n", 
                 isBlock, actualDataLen, payloadLen);
            LOGD("PEC Calc Buffer (Size=%d):", pIdx);
            for(int i=0; i < pIdx; i++) {
                 LOGD(" %02X", pecCalcBuf[i]);
            }
            LOGD("\n");
            /* 4. 获取接收到的 PEC (位于有效负载之后) */
            receivedPec = rxStackBuf[payloadLen]; 

            /* 5. 计算并比对 */
            U8 calcPec = smbusPecPktConstruct(slaveAddr, FALSE, pecCalcBuf, pIdx);
            
            if (calcPec != receivedPec) {
                LOGE("%s: PEC Error (Calc:0x%02X, Recv:0x%02X)\n", __func__, calcPec, receivedPec);
                ret = -EIO; // 或定义为 -EBADMSG
                goto unlock;
            }
            LOGD("PEC Result: Calc=0x%02X, Recv=0x%02X at RxBuf[%d]\n", 
                 calcPec, receivedPec, payloadLen);
        }

        /* [C] 拷贝回用户缓冲区 */
        if (xfer->rBuf) {
            /* 安全拷贝：取 actual 和 用户请求长度 的较小值 */
            U32 copyLen = (actualDataLen > xfer->rLen) ? xfer->rLen : actualDataLen;
            memcpy(xfer->rBuf, pDataStart, copyLen);
            
            /* 回填实际长度 */
            if (xfer->actualRxLen) {
                *xfer->actualRxLen = copyLen;
            }
        }
    }
unlock:
    devUnlockByDriver(devId);
    return ret;
}
