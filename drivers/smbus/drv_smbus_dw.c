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
 * 2025/12/11   wangkui         correct the issue in AI review
 * 2025/12/15   wangkui         replace slave with target naming
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
 * @brief Initialize SMBus target resources only
 * @details Allocates target RX/TX buffers for devices that are being switched
 *          to target mode after initial initialization.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure:
 *         -EINVAL: NULL driver data
 *         -ENOMEM: Memory allocation failed
 * @note This function should only be called when switching TO target mode
 * @note Does NOT perform full device initialization, only target-specific resources
 */
static S32 smbusInitTargetResources(SmbusDrvData_s *pDrvData)
{
    S32 ret = EXIT_SUCCESS;
    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL, "pDrvData is NULL");

    LOGD("%s: Checking mode: masterMode=%d, SMBUS_MODE_TARGET=%d\n",
         __func__, pDrvData->sbrCfg.masterMode, SMBUS_MODE_TARGET);

    if (pDrvData->sbrCfg.masterMode == SMBUS_MODE_TARGET) {
        LOGD("%s: Device is in target mode - allocating buffers\n", __func__);
        /* Only allocate if not already allocated */
        if (pDrvData->pSmbusDev.targetRxBuf == NULL) {
            LOGD("%s: Allocating target RX buffer\n", __func__);
            pDrvData->pSmbusDev.targetRxBuf = (U8 *)malloc(SMBUS_TARGET_BUF_LEN);
            if (pDrvData->pSmbusDev.targetRxBuf == NULL) {
                LOGE("%s: Failed to allocate target RX buffer\n", __func__);
                ret = -ENOMEM;
                goto exit;
            }
            pDrvData->pSmbusDev.targetValidRxLen = 0;
        }

        if (pDrvData->pSmbusDev.targetTxBuf == NULL) {
            LOGD("%s: Allocating target TX buffer\n", __func__);
            pDrvData->pSmbusDev.targetTxBuf = (U8 *)malloc(SMBUS_TARGET_BUF_LEN);
            if (pDrvData->pSmbusDev.targetTxBuf == NULL) {
                LOGE("%s: Failed to allocate target TX buffer\n", __func__);
                ///< Clean up allocated RX buffer
                if (pDrvData->pSmbusDev.targetRxBuf != NULL) {
                    free(pDrvData->pSmbusDev.targetRxBuf);
                    pDrvData->pSmbusDev.targetRxBuf = NULL;
                    pDrvData->pSmbusDev.targetValidRxLen = 0;
                }
                ret = -ENOMEM;
                goto exit;
            }
            pDrvData->pSmbusDev.targetValidTxLen = 0;
            pDrvData->pSmbusDev.txIndex = 0;

        }
        LOGD("%s: target resources initialized successfully\n", __func__);
    }else {
        LOGD("%s: Device not in target mode, skipping target buffer allocation\n", __func__);
    }   
exit:
    return ret;
}

/**
 * @brief Free SMBus target resources only
 * @details Frees target RX/TX buffers for devices that are switching
 *          away from target mode.
 * @param[in] pDrvData Pointer to driver data structure
 * @return void
 * @note This function should only be called when switching FROM target mode
 */
static void smbusFreetargetResources(SmbusDrvData_s *pDrvData)
{
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL, "pDrvData is NULL");

    if (pDrvData->pSmbusDev.targetRxBuf != NULL) {
        LOGD("%s: Freeing target RX buffer\n", __func__);
        free(pDrvData->pSmbusDev.targetRxBuf);
        pDrvData->pSmbusDev.targetRxBuf = NULL;
        pDrvData->pSmbusDev.targetValidRxLen = 0;
    }

    if (pDrvData->pSmbusDev.targetTxBuf != NULL) {
        LOGD("%s: Freeing target TX buffer\n", __func__);
        free(pDrvData->pSmbusDev.targetTxBuf);
        pDrvData->pSmbusDev.targetTxBuf = NULL;
        pDrvData->pSmbusDev.targetValidTxLen = 0;
    }

    LOGD("%s: target resources freed successfully\n", __func__);
}

/**
 * @brief Write software UDID structure to hardware registers
 * @note Must be called before enabling target mode
 */
static void smbusWriteHwUdid(volatile SmbusRegMap_s *regBase, const SmbusUdid_s *udid)
{
    /* * SMBus UDID sequence (16 bytes):
     * Byte 0: Device Capabilities
     * Byte 1: Version Revision
     * Byte 2-3: Vendor ID
     * ...
     * * DW IP registers are typically filled in Little Endian:
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
    /* Add debug prints to confirm successful write */
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

    SMBUS_CHECK_PARAM(smbusSbrCfg == NULL, -EINVAL, "smbusSbrCfg is NULL");

    if (devSbrRead(devId, smbusSbrCfg, 0, sizeof(SbrI2cSmbusCfg_s)) != sizeof(SbrI2cSmbusCfg_s)) {
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGE("smbus: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
         smbusSbrCfg->regAddr, smbusSbrCfg->irqNo, smbusSbrCfg->irqPrio);
#endif

    SMBUS_CHECK_PARAM(smbusSbrCfg->irqNo == 0 || smbusSbrCfg->irqPrio == 0 || smbusSbrCfg->regAddr == NULL,
                    -EINVAL, "Invalid irqNo=0, irqPrio=0 or regAddr=NULL");

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
 * @note Uses CRC-8 with polynomial SMBUS_CRC8_POLYNOMIAL_VALUE
 * @note Processes entire buffer sequentially
 */
static U8 smbusCalcPEC(U8 pec, const U8 *buf, U32 len)
{
    for (U32 i = 0; i < len; ++i) {
        pec = smbusCrc8CalcOne(pec, buf[i]);
    }
    return pec;
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
    SMBUS_CHECK_PARAM_RETURN((pData == NULL) && (count > 0), 0xFF,
                            "pData is NULL but count > 0");

    if (count > SMBUS_MAX_BUFFER_SIZE) {
        return 0xFF;
    }
    U8 pec = 0;
    U8 addr = i2cAddrConvert(addr7bitIn, isWrite);
    U8 addrWithPec = smbusCalcPEC(pec, &addr, 1);
    return smbusCalcPEC(addrWithPec, pData, count);
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
 */
static S32 smbusAllocateDriverData(DevList_e devId, SmbusDrvData_s **ppDrvData)
{
    SmbusDrvData_s *pDrvData;

    SMBUS_CHECK_PARAM_RETURN(ppDrvData == NULL, -EINVAL, "ppDrvData is NULL");
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
    
    *ppDrvData = pDrvData;
    return EXIT_SUCCESS;
}

/**
 * @brief Initialize SMBus hardware using HAL probe functions
 * @details Configures hardware based on mode (master/target) using I2C-compatible
 *          probe functions from the HAL layer.
 * @param[in] pDrvData Pointer to driver data structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Uses smbusProbeMaster for master mode
 * @note Uses smbusProbeTarget for target mode
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
        .targetAddr = pDrvData->sbrCfg.slaveAddrLow,
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
            pDrvData->pSmbusDev.smbFeatures.quickCmdEnb = 0; /* Force disable, prioritize ARP */
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
            if (pDrvData->rxBuffer == NULL) {
                LOGE("%s: Failed to allocate master RX buffer of size %d\n", __func__, pDrvData->rxBufferSize);
                return -ENOMEM;
            }
            memset(pDrvData->rxBuffer, 0, pDrvData->rxBufferSize);
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
        /* Configure target settings using HAL operations */
        if (pDrvData->pSmbusDev.halOps != NULL && pDrvData->pSmbusDev.halOps->configureTarget != NULL) {
            pDrvData->pSmbusDev.halOps->configureTarget(&pDrvData->pSmbusDev);
            LOGD("%s: target configuration completed via HAL operations\n", __func__);
        } else {
            LOGW("%s: HAL configureTarget function not available, using default configuration\n", __func__);
        }

        ret = smbusProbeTarget(&pDrvData->pSmbusDev);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: smbusProbeTarget failed, ret=%d\n", __func__, ret);
            return ret;
        }
        pDrvData->pSmbusDev.mode = SMBUS_MODE_TARGET;
        LOGD("%s: target hardware initialization completed\n", __func__);
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
/**
 * @brief Clear specified I2C interrupt bits using optimized bit scanning
 * @details Clears individual interrupt bits by reading from corresponding
 *          clear registers. Uses __builtin_ctz for efficient bit position
 *          detection and array lookup for register mapping.
 * @param[in] regBase Pointer to SMBus register base address
 * @param[in] mask Bit mask of interrupts to clear (1=clear, 0=keep)
 * @return void
 *
 * @note Only clears bits present in INTR_CLR_REGS array
 * @note Uses dummy read operation to clear interrupts
 * @warning regBase must be valid, function does not perform NULL check
 */
static inline void clearInterrupts(volatile SmbusRegMap_s *regBase, U32 mask)
{
    U32 bit;
    volatile U32 *reg_ptr;
    uintptr_t base_addr, reg_offset;

    /* Use bit scanning for fast positioning */
    while (mask != 0) {
        bit = __builtin_ctz(mask);  /* Get lowest set bit index */

        if (bit < ARRAY_SIZE(INTR_CLR_REGS) && INTR_CLR_REGS[bit] != NULL) {
            /* Calculate actual register address - use uintptr_t for correct pointer arithmetic */
            base_addr = (uintptr_t)regBase;
            reg_offset = (uintptr_t)INTR_CLR_REGS[bit];
            reg_ptr = (volatile U32*)(base_addr + reg_offset);
            (void)(*reg_ptr);
        }

        mask &= ~(1U << bit);  /* Clear processed bit */
    }
}
/**
 * @brief Clear SMBus and I2C interrupts with comprehensive mask support
 * @details Clears both standard I2C interrupts and SMBus-specific interrupts.
 *          Supports clearing all interrupts (0xFFFFFFFF) or specific bits.
 *          Handles both regular and SMBus protocol interrupts.
 * @param[in] regBase Pointer to SMBus register base address
 * @param[in] mask Interrupt mask specifying which bits to clear:
 *             - 0xFFFFFFFF: Clear all interrupts
 *             - Other values: Clear specific interrupt bits
 * @return void
 *
 * @note For all-interrupt clear, reads both icClrIntr and icClrSmbusIntr
 * @note For specific-bit clear, delegates to clearInterrupts() function
 * @note SMBus-specific interrupts are cleared after standard I2C interrupts
 * @warning regBase cannot be NULL, validated with SMBUS_CHECK_PARAM_VOID
 */
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

    /* Use optimized interrupt clear function */
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
/**
 * @brief Handle master mode interrupts (Flattened & Robust Logic)
 * @details Optimized for Block Process Call and complex transfers.
 */
static void smbusHandleMasterInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                     U32 intrStat, U32 smbusIntrStat)
{
    SmbusDev_s *dev = &pDrvData->pSmbusDev;
    Bool isAsync = (pDrvData->sbrCfg.interruptMode == 1);
    Bool transferDone = false;
    Bool isError = false;
    SmbusEventData_u eventData;

    /* 1. Initial Interrupts (Clear Status) */
    if (intrStat & SMBUS_INTR_ACTIVITY) (void)regBase->icClrActivity;
    if (intrStat & SMBUS_INTR_START_DET) (void)regBase->icClrStartDet;

    /* 2. Error Handling: TX ABORT (Highest Priority) */
    if (intrStat & SMBUS_INTR_TX_ABRT) {
        U32 abrtSource = regBase->icTxAbrtSource.value;
        (void)regBase->icClrTxAbrt;

        LOGE("SMBus Master TX Abort: source=0x%08X\n", abrtSource);
        
        dev->abortSource = abrtSource;
        dev->cmdErr = 1;
        dev->status &= ~SMBUS_STATUS_ACTIVE_MASK;
        dev->xferStatus = -EIO; 
        
        /* Disable Interrupts immediately */
        regBase->icIntrMask.value = 0;

        if (isAsync) {
            smbusReleaseSemaphore(dev, "Async Abort");
        }
        
        eventData.error.errorCode = SMBUS_ERROR_TX_ABORT;
        eventData.error.statusReg = intrStat;
        isError = true;
        transferDone = true;
        goto callback_check;
    }

    /* 3. RX FULL Handling (Priority for Data) */
    if (intrStat & SMBUS_INTR_RX_FULL) {
        if (isAsync) {
            /* Async Mode: Drain FIFO via HAL */
            SmbusHalOps_s *halOps = smbusGetHalOps();
            if (halOps && halOps->smbusDwRead) {
                halOps->smbusDwRead(pDrvData, pDrvData->rxBuffer, pDrvData->rxBufferSize);
                
                /* ============================================================
                 * CRITICAL FIX: Re-sample TX_EMPTY status!
                 * smbusDwRead might have unmasked TX_EMPTY (Block Read Logic).
                 * We must catch this update immediately in the SAME ISR pass.
                 * ============================================================ */
                if ((regBase->icIntrMask.value & SMBUS_INTR_TX_EMPTY) && 
                    (regBase->icRawIntrStat.value & SMBUS_INTR_TX_EMPTY)) {
                    intrStat |= SMBUS_INTR_TX_EMPTY; /* Force TX handling below */
                }
            }
        } else {
            /* Legacy Mode */
            smbusMasterReadData(pDrvData, regBase);
            eventData.transfer.buffer = pDrvData->rxBuffer;
            eventData.transfer.len = pDrvData->rxLength;
            transferDone = true; 
        }
    }

    /* 4. TX EMPTY Handling (Refill FIFO) */
    if (intrStat & SMBUS_INTR_TX_EMPTY) {
        if (isAsync) {
            /* Async Mode: Refill FIFO */
            if (dev->msgWriteIdx < dev->msgsNum || (dev->status & SMBUS_STATUS_WRITE_IN_PROGRESS)) {
                SmbusHalOps_s *halOps = smbusGetHalOps();
                if (halOps && halOps->smbusDwXferMsg) {
                    halOps->smbusDwXferMsg(dev);
                }
            }
            /* Disable TX_EMPTY if no more messages to write */
            if (dev->msgWriteIdx >= dev->msgsNum && !(dev->status & SMBUS_STATUS_WRITE_IN_PROGRESS)) {
                regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
            }
        } else {
            /* Legacy Mode */
            smbusMasterTransferData(pDrvData, regBase);
            if (pDrvData->txComplete) {
                regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
                smbusReleaseSemaphore(dev, "SMBus Sync TX");
            }
        }
    }

    /* 5. STOP DETECT Handling (Completion Logic) */
    if ((intrStat & SMBUS_INTR_STOP_DET) || (dev->msgErr)) {
        (void)regBase->icClrStopDet;

        /* Quick Command Check */
        if (isAsync && dev->isQuick) {
            dev->xferStatus = (dev->cmdErr) ? -EIO : 0; 
            dev->isQuick = 0;
            regBase->icIntrMask.value = 0;
            smbusReleaseSemaphore(dev, "QuickCmd Stop");
            return; 
        }

        /* Async Completion Logic */
        if (isAsync) {
            Bool swWriteDone = (dev->msgWriteIdx >= dev->msgsNum);
            /* Check software read index */
            Bool swReadDone  = (dev->msgReadIdx >= dev->msgsNum);
            /* Double check hardware FIFO is empty */
            Bool hwRxEmpty   = (regBase->icRxflr == 0);
            
            /* Safety: If STOP detected but RX FIFO has data, drain it now */
            if (!hwRxEmpty) {
                 SmbusHalOps_s *halOps = smbusGetHalOps();
                 if (halOps && halOps->smbusDwRead) {
                     halOps->smbusDwRead(pDrvData, pDrvData->rxBuffer, pDrvData->rxBufferSize);
                     hwRxEmpty = (regBase->icRxflr == 0);
                     swReadDone = (dev->msgReadIdx >= dev->msgsNum);
                 }
            }

            /* Final Completion Check */
            if (swWriteDone && swReadDone && hwRxEmpty) {
                dev->status &= ~SMBUS_STATUS_ACTIVE_MASK;
                regBase->icIntrMask.value = 0; /* Disable all IRQs */
                smbusReleaseSemaphore(dev, "Async Complete");
            } else {
                /* Force completion if state is stuck but STOP happened */
                LOGW("SMBus: STOP detected but xfer incomplete (W:%d R:%d). Forcing Done.\n",
                     dev->msgWriteIdx, dev->msgReadIdx);
                dev->status &= ~SMBUS_STATUS_ACTIVE_MASK;
                regBase->icIntrMask.value = 0;
                smbusReleaseSemaphore(dev, "Async Force Complete");
            }
        } else {
            /* Legacy Mode */
            dev->status &= ~SMBUS_STATUS_ACTIVE_MASK;
            smbusReleaseSemaphore(dev, "Legacy Stop");
        }
    }

    /* 6. SMBus Specific Interrupts */
    if (smbusIntrStat) {
        smbusHandleMasterSpecificInterrupts(pDrvData, regBase, smbusIntrStat);
    }

callback_check:
    /* 7. Unified Callback */
    if ((transferDone || isError) && pDrvData->callback) {
        U32 type = isError ? SMBUS_EVENT_ERROR : SMBUS_EVENT_RX_DONE;
        pDrvData->callback(pDrvData->devId, type, &eventData, pDrvData->userData);
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
 *          target buffers, and ARP context.
 * @param[in] pDrvData Pointer to driver data structure
 * @return void
 *
 * @note Safe to call with NULL pointer
 * @note Handles both master and target mode cleanup
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

    /* Free target buffers if allocated */
    if (pDrvData->pSmbusDev.targetRxBuf != NULL) {
        free(pDrvData->pSmbusDev.targetRxBuf);
        pDrvData->pSmbusDev.targetRxBuf = NULL;
        pDrvData->pSmbusDev.targetValidRxLen = 0;
    }

    if (pDrvData->pSmbusDev.targetTxBuf != NULL) {
        free(pDrvData->pSmbusDev.targetTxBuf);
        pDrvData->pSmbusDev.targetTxBuf = NULL;
        pDrvData->pSmbusDev.targetValidTxLen = 0;
    }

    /* Clear master TX buffer array - no need to free as it's a static array */
    memset(pDrvData->pSmbusDev.masterTxBuf, 0, sizeof(pDrvData->pSmbusDev.masterTxBuf));
    pDrvData->pSmbusDev.masterTxBufLen = 0;
    LOGD("%s: Master TX buffer array cleared\n", __func__);
}

/* =========================================================================
 * Helper: Handle WR_REQ (Master -> target write transfer start)
 * ========================================================================= */
static void smbusTargetHandleWriteRequest(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase)
{
    LOGI("SMBus target: WR_REQ - Master writing (Priority 1)\n");

    /* Reset counters, set write status */
    pDrvData->pSmbusDev.targetValidRxLen = 0;
    pDrvData->pSmbusDev.status |= SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
    pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;

    /* Keep TX_EMPTY disabled as this is receive mode */
    regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;

    /* Clear WR_REQ */
    (void)regBase->icClrWrReq;

    LOGD("SMBus target: WR_REQ - Setup done, mask=0x%08X\n", regBase->icIntrMask.value);

    /* Trigger event to notify upper layer to prepare for reception */
    smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_TARGET_WRITE_REQ,
                           pDrvData->pSmbusDev.targetRxBuf,
                           pDrvData->pSmbusDev.targetValidRxLen);
}

/* =========================================================================
 * Helper: Handle RX_FULL (receive data FIFO)
 * ========================================================================= */
static inline void smbusTargetHandleRxData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase, U32 intrStat)
{
    LOGI("SMBus target: RX_FULL - Receiving data (Priority 2)\n");

    U32 loopCount = 0;
    /* Read FIFO until empty or limit reached */
    SmbusIcStatusReg_u statusReg;
    statusReg.value = smbusReadReg(&regBase->icStatus.value);
    while ((statusReg.fields.rfne) && (loopCount < SMBUS_TARGET_BUF_LEN)) {
        U32 tmp = smbusReadReg(&regBase->icDataCmd.value);
        U8 val = (U8)tmp;

        /* Re-read status for next iteration */
        statusReg.value = smbusReadReg(&regBase->icStatus.value);

        if (tmp & SMBUS_IC_DATA_CMD_FIRST_DATA_BYTE_MASK) {
            LOGD("SMBus target: FIRST_DATA_BYTE: 0x%02X\n", val);
        }

        if (pDrvData->pSmbusDev.targetValidRxLen < SMBUS_TARGET_BUF_LEN) {                  
            pDrvData->pSmbusDev.targetRxBuf[pDrvData->pSmbusDev.targetValidRxLen++] = val;
        }   
        loopCount++;
    } 

    if (loopCount > 0) {
        LOGD("SMBus target: Drained %d bytes, total valid: %d\n", loopCount, pDrvData->pSmbusDev.targetValidRxLen);
    }
}

/* =========================================================================
 * Helper: Handle RD_REQ (Master <- target read transfer request)
 * ========================================================================= */
static inline void smbusTargetHandleReadRequest(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase)
{
    LOGD("SMBus Slave: RD_REQ - Master requests data\n");
    (void)regBase->icClrRdReq;

    /* If this is a new transfer, prepare data */
    if (pDrvData->pSmbusDev.txIndex == 0) {
        LOGI("SMBus target: RD_REQ - New transaction start\n");
        smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_TARGET_READ_REQ,
                               pDrvData->pSmbusDev.targetTxBuf,
                               pDrvData->pSmbusDev.targetValidTxLen);
    } else {
        LOGD("SMBus target: RD_REQ - Continued transfer, idx=%d\n", pDrvData->pSmbusDev.txIndex);
    }

    /* Send current byte */
    U8 txData = SMBUS_DUMMY_DATA_BYTE;
    if (pDrvData->pSmbusDev.targetTxBuf != NULL &&
        pDrvData->pSmbusDev.txIndex < pDrvData->pSmbusDev.targetValidTxLen) {

        txData = pDrvData->pSmbusDev.targetTxBuf[pDrvData->pSmbusDev.txIndex];
        regBase->icDataCmd.value = (U32)txData;
        LOGD("SMBus target: RD_REQ - Sent Byte[%d] = 0x%02X\n", pDrvData->pSmbusDev.txIndex, txData);
        pDrvData->pSmbusDev.txIndex++;
    } else {
        smbusWriteReg(&regBase->icDataCmd.value, SMBUS_DUMMY_DATA_BYTE);
        LOGW("SMBus target: RD_REQ - No data/End of buf, sent dummy 0x%02X\n", SMBUS_DUMMY_DATA_BYTE);
    }

    /* If there is remaining data, enable TX_EMPTY to use FIFO interrupt for sending */
    if (pDrvData->pSmbusDev.targetValidTxLen > 1) {
        regBase->icIntrMask.value |= SMBUS_INTR_TX_EMPTY;
    } else {
        regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
    }
    
}

/* =========================================================================
 * Helper: Handle TX_EMPTY (FIFO filling)
 * ========================================================================= */
static inline void smbusTargetHandleTxEmpty(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase)
{
    U32 status = regBase->icStatus.value;
    U32 targetActivity = (status >> SMBUS_IC_CON_TARGET_DISABLE_BIT) & SMBUS_TARGET_ENABLED;
    U32 rxFifoLevel = regBase->icRxflr;
    
    if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
        LOGD("SMBus target: TX_EMPTY ignored during WRITE state\n");
        /* Should not fill data at this time, nor disable interrupt (may switch to Read soon), just return */
        /* Or: If interrupt fires too frequently affecting performance, can temporarily mask, reopen in RD_REQ */
        regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
        return;
    }
    /* Only handle when Master is actually reading and there is no RX data */
    if (targetActivity && rxFifoLevel == 0) {
        LOGD("SMBus target: TX_EMPTY - Refilling FIFO\n");
        U32 txLen = pDrvData->pSmbusDev.targetValidTxLen;
        if (txLen == 0) {
            regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
            return;
        }
        
        while (pDrvData->pSmbusDev.txIndex < txLen) {
            if (!(regBase->icStatus.fields.tfnf)) break; /* FIFO Full */

            U8 txData = pDrvData->pSmbusDev.targetTxBuf[pDrvData->pSmbusDev.txIndex];
            regBase->icDataCmd.value = (U32)txData;
            pDrvData->pSmbusDev.txIndex++;
        }
        
        if (pDrvData->pSmbusDev.txIndex >= txLen) {
            regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
            LOGD("SMBus target: All bytes queued, disabling TX_EMPTY\n");
        }
    } else {
        /* Spurious interrupt handling: Master is writing or bus is idle */
        LOGW("SMBus target: Spurious TX_EMPTY (act=%d, rxLvl=%d) - Disabling\n", targetActivity, rxFifoLevel);
        regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
    }
}

/* =========================================================================
 * Helper: Handle STOP_DET (transfer end)
 * ========================================================================= */
static inline void smbusTargetHandleStop(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase)
{
    LOGI("SMBus target: STOP_DET - Transaction completed\n");
    
    /* Case A: Write operation completed */
    if (pDrvData->pSmbusDev.status & SMBUS_STATUS_WRITE_IN_PROGRESS_MASK) {
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;

        if (pDrvData->pSmbusDev.targetValidRxLen == 1) {
            /* Send Byte protocol (or only wrote one byte) */
            smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_TARGET_WRITE_REQ,
                                   pDrvData->pSmbusDev.targetRxBuf, 1);
        } else if (pDrvData->pSmbusDev.targetValidRxLen > 1) {
            /* Block Write */
            smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_RX_DONE,
                                   pDrvData->pSmbusDev.targetRxBuf,
                                   pDrvData->pSmbusDev.targetValidRxLen);
        }
        LOGD("SMBus target: Write Done, len=%d\n", pDrvData->pSmbusDev.targetValidRxLen);
    }

    /* Case B: Read operation completed */
    if (pDrvData->pSmbusDev.status & SMBUS_STATUS_READ_IN_PROGRESS_MASK) {
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_READ_IN_PROGRESS_MASK;
    }

    /* Reset status to prepare for next possible read operation (IP feature requirement) */
    regBase->icIntrMask.value |= SMBUS_INTR_TX_EMPTY;
    pDrvData->pSmbusDev.txIndex = 0;
    (void)regBase->icClrStopDet;
}

/* =========================================================================
 * Helper: Handle exceptions and other simple interrupts (Tag, Abort, RX Done)
 * ========================================================================= */
static inline void smbusTargetHandleExceptions(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase, U32 intrStat)
{
    /* Address Tags */
    if (intrStat & (SMBUS_INTR_SLV_ADDR1_TAG | SMBUS_INTR_SLV_ADDR2_TAG |
                    SMBUS_INTR_SLV_ADDR3_TAG | SMBUS_INTR_SLV_ADDR4_TAG)) {
        LOGI("SMBus target: Address Tag detected\n");
        (void)regBase->icClrSlvAddrTag;
    }

    /* TX Abort */
    if (intrStat & SMBUS_INTR_TX_ABRT) {
        LOGD("SMBus target: TX_ABRT\n");
        (void)regBase->icClrTxAbrt;
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS_MASK;
        
        smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_TX_DONE,
                               pDrvData->pSmbusDev.targetTxBuf,
                               pDrvData->pSmbusDev.targetValidTxLen);
        pDrvData->targetTransferActive = 0;
        smbusReleaseSemaphore(&pDrvData->pSmbusDev, "SMBus target");
    }

    /* RX Done */
    if (intrStat & SMBUS_INTR_RX_DONE) {
        (void)regBase->icClrRxDone;
    }
}
/**
 * @brief Handle target mode interrupts (integrated from i2c_dw_isr_TARGET)
 * @details Processes target mode interrupt events including read/write requests,
 *          stop detection, and SMBus-specific target events.
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] intrStat I2C interrupt status
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 *
 * @note Integrates I2C target ISR LOGEc with SMBus protocol handling
 * @note Uses SMBus callback functions for event notification
 */
static void smbusHandleTargetInterrupt(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase,
                                      U32 intrStat, U32 smbusIntrStat)
{
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL || regBase == NULL,
                      "pDrvData or regBase is NULL");

    LOGI("SMBus target ISR: stat=0x%08X\n", intrStat);

    /* 1. WR_REQ (Priority 1) */
    if (intrStat & SMBUS_INTR_WR_REQ) {
        smbusTargetHandleWriteRequest(pDrvData, regBase);
    }

    /* 2. RX_FULL (Priority 2) */
    if (intrStat & SMBUS_INTR_RX_FULL) {
        smbusTargetHandleRxData(pDrvData, regBase, intrStat);
    }

    /* 3. RD_REQ (Priority 3) */
    if (intrStat & SMBUS_INTR_RD_REQ) {
        smbusTargetHandleReadRequest(pDrvData, regBase);
    }

    /* 4. TX_EMPTY (Priority 5 - but handled here) */
    if (intrStat & SMBUS_INTR_TX_EMPTY) {
        smbusTargetHandleTxEmpty(pDrvData, regBase);
    }

    /* 5. STOP_DET (Priority 4) */
    if (intrStat & SMBUS_INTR_STOP_DET) {
        smbusTargetHandleStop(pDrvData, regBase);
    }

    /* 6. Exceptions & Others AddrTags mask estimate */
    if (intrStat & (SMBUS_INTR_TX_ABRT | SMBUS_INTR_RX_DONE | 0xF00)) {
        smbusTargetHandleExceptions(pDrvData, regBase, intrStat);
    }

    /* 7. SMBus Specific */
    if (smbusIntrStat) {
        smbusHandleTargetSpecificInterrupts(pDrvData, regBase, smbusIntrStat);
    }
    
    /* 8. Clear Status to prevent re-trigger */
    if (intrStat != 0) {
        (void)regBase->icIntrStat.value;
    }
}

/**
 * @brief SMBus interrupt service routine (unified I2C/SMBus handler)
 * @details Handles all SMBus and I2C interrupt events. This function provides
 *          a unified interrupt handler that supports both master and target modes
 *          based on SmbusMode_e. It integrates I2C DW ISR LOGEc with SMBus
 *          protocol handling.
 * @param[in] arg Pointer to driver data structure
 * @return void
 *
 * @note This function is called from the interrupt context
 * @note Uses SmbusMode_e to differentiate between master/target interrupt handling
 * @note Integrates I2C ISR LOGEc with SMBus protocol handling
 * @note Calls SMBus callback functions for event notification
 */
static void smbusIsr(void *arg)
{
    SmbusDrvData_s *pDrvData = (SmbusDrvData_s *)arg;
    volatile SmbusRegMap_s *regBase;
    SmbusIntrStatReg_u smbusIntrStat;
    U32 intrStat, enabled;

    SMBUS_CHECK_PARAM_VOID((!pDrvData || !pDrvData->sbrCfg.regAddr),
                           "SMBus ISR: Invalid driver data or register address\n");

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
    SMBUS_CHECK_PARAM_VOID((!enabled || (rawIntr == 0) || (rawIntr == 0xFFFFFFFF)),
                           "SMBus ISR: enabled=0x%08X rawIntr=0x%08X intrStat=0x%08X\n",
                           enabled, rawIntr, intrStat);
    LOGD("SMBus IRQ: mode=%s, rawIntr=0x%08X, intrStat=0x%08X, smbus=0x%08X\n",
         (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "target",
         rawIntr, intrStat, smbusIntrStat.value);

    /* Handle interrupts based on current mode - FIXED: Use raw interrupt status */
    if (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) {
        LOGD("SMBus ISR: Handling MASTER mode interrupts\n");
        smbusHandleMasterInterrupt(pDrvData, regBase, rawIntr, smbusIntrStat.value);
    } else {
        LOGD("SMBus ISR: Handling target mode interrupts\n");
        smbusHandleTargetInterrupt(pDrvData, regBase, rawIntr, smbusIntrStat.value);
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
 *         -EBUSY: Device locked by another driver
 *         -EEXIST: Device already initialized
 *         -ENODEV: Device not found or driver mismatch
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
    Bool irqShared = false;
    SmbusDrvData_s *pDrvData = NULL;

    if (devLockByDriver(devId, SMBUS_LOCK_TIMEOUT_MS) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }
    /* Check if already initialized */
    if (isDrvInit(devId) == true) {
        ret = -EEXIST;
        goto unlock;
    }

    if (!isDrvMatch(devId, DRV_ID_DW_I2C)) {
        ret = -ENODEV;    ///< device not matched
        goto unlock;
    }
    if(config == NULL){
       ret = -EINVAL;
       goto unlock;
    }
    /* Step 1: Allocate driver data structure */
    ret = smbusAllocateDriverData(devId, &pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Driver data allocation failed, ret=%d\n", __func__, ret);
        goto unlock;
    }

#ifndef TEST_SUITS_1
    /* Step 2: Get SBR configuration (not modularized as requested) */
    if (smbusDevCfgGet(devId, &pDrvData->sbrCfg) != EXIT_SUCCESS) {
        LOGE("%s: get SBR failed\n", __func__);
        ret = -ENXIO;
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
    pDrvData->sbrCfg.slaveAddrLow = config->targetAddrLow;
    pDrvData->sbrCfg.enSmbus = config->featureMap;
#endif
    pDrvData->sbrCfg.masterMode = config->masterMode;
    pDrvData->udid.deviceCapabilities = EXTRACT_BYTE(config->udidWord0, 0);
    pDrvData->udid.versionRevision = EXTRACT_BYTE(config->udidWord0, 1);
    pDrvData->udid.vendorId = (U16)(config->udidWord0 >> SMBUS_BYTE_SHIFT_16); /* Bytes 2-3 */
    /* Word 1 -> Bytes 4-7 */
    pDrvData->udid.deviceId = (U16)(config->udidWord1 & SMBUS_16BIT_DATA_MASK); /* Bytes 4-5 */
    pDrvData->udid.interface = (U16)(config->udidWord1 >> SMBUS_BYTE_SHIFT_16);    /* Bytes 6-7 */
    /* Word 2 -> Bytes 8-11 */
    pDrvData->udid.subsystemVendorId = (U16)(config->udidWord2 & SMBUS_16BIT_DATA_MASK); /* Bytes 8-9 */
    pDrvData->udid.subsystemDeviceId = (U16)(config->udidWord2 >> SMBUS_BYTE_SHIFT_16);    /* Bytes 10-11 */
    /* Word 3 -> Bytes 12-15 */
    pDrvData->udid.vendorSpecificId = config->udidWord3;                 /* Bytes 12-15 */
    LOGD("%s: Using test configuration:%d\n", __func__, pDrvData->sbrCfg.masterMode);
    
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
        irqShared = true;
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
    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS) {
        LOGW("%s: reset failed\n", __func__);
        return ret;
    }

    /* Step 6: Initialize hardware using HAL probe functions AFTER reset */
    ret = smbusInitializeHardware(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Hardware initialization failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    /* Step 7: Allocate target buffers for target mode */
    ret = smbusInitTargetResources(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: target buffer allocation failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    /* Step 8: Allocate master buffers for master mode */
    ret = smbusAllocateMasterBuffers(pDrvData);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Master buffer allocation failed, ret=%d\n", __func__, ret);
        goto disable_vector;
    }

    /* Step 9: Initialize ARP Master context (modularized) */
    if (pDrvData->sbrCfg.enSmbus & SMBUS_FEATURE_ARP) {
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

    LOGI("SMBus device %d initialized successfully in %s mode, semaphoreId=%d\n",
         devId, (pDrvData->sbrCfg.masterMode == 1) ? "master" : "target",
         pDrvData->pSmbusDev.semaphoreId);
    goto unlock;

disable_vector:
    ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);
    if (!irqShared) ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, smbusIsr, pDrvData);
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

    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) return ret;

    dev = &pDrvData->pSmbusDev;
    /* Destroy semaphore if created */
    smbusDeleteSemaphore(&pDrvData->pSmbusDev, "Deinit");

    /* Disable interrupt vector */
    ospInterruptVectorDisable(pDrvData->sbrCfg.irqNo);

    /* Remove interrupt handler */
    ret = ospInterruptHandlerRemove(pDrvData->sbrCfg.irqNo, smbusIsr, pDrvData);
    if (ret != OSP_SUCCESSFUL) {
        LOGW("%s: Interrupt handler removal failed, ret=%d\n", __func__, ret);
        /* Continue with deinitialization even if interrupt removal fails */
        ret = EXIT_SUCCESS;  /* Reset to success since we continue */
    }

    /* Disable SMBus controller */
    if (pDrvData->pSmbusDev.regBase != NULL) {
        /* Get HAL operations */
        if (dev->halOps != NULL && dev->halOps->disable != NULL) {
            dev->halOps->disable(dev);
        }
    }

    /* Reset SMBus peripheral */
    ret = peripsReset(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO) {
        LOGE("%s: Peripheral reset failed, ret=%d\n", __func__, ret);
        ret = -EIO;
        goto cleanup;  /* Use goto for consistent error handling */
    }

    /* Disable SMBus peripheral clock */
    ret = peripsClockDisable(devId);
    if (ret != EXIT_SUCCESS && ret != -ENXIO) {
        LOGE("%s: Clock disable failed, ret=%d\n", __func__, ret);
        ret = -EIO;
        goto cleanup;
    }

    /* Free driver data using centralized cleanup function */
    smbusFreeDriverData(pDrvData);

    /* Unregister driver data */
    ret = drvUninstall(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Driver uninstall failed, ret=%d\n", __func__, ret);
        goto unlock;
    }

    LOGI("SMBus device %d deinitialized successfully\n", devId);
    ret = EXIT_SUCCESS;

unlock:
    funcRunEndHelper(devId);
    return ret;

cleanup:
    /* Ensure driver data is freed even if reset/clock fails */
    smbusFreeDriverData(pDrvData);
    /* Try to uninstall driver even if reset/clock failed */
    S32 uninstallRet = drvUninstall(devId);
    if (uninstallRet != EXIT_SUCCESS) {
        LOGE("%s: Driver uninstall failed during cleanup, ret=%d\n", __func__, uninstallRet);
    }
    goto unlock;
}

/**
 * @brief Master/target mode switch
 * @details Dynamically switches the SMBus controller between master and target
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
S32 smbusMasterTargetModeSwitch(DevList_e devId, SmbusSwitchParam_s *param)
{
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Parameter validation before any locking */
    SMBUS_CHECK_PARAM_RETURN(param == NULL, -EINVAL, "param is NULL");

    /* Validate target mode parameter */
    SMBUS_CHECK_PARAM_RETURN(param->targetMode != DW_SMBUS_MODE_MASTER && param->targetMode != DW_SMBUS_MODE_TARGET,
                            -EINVAL, "Invalid target mode %d", param->targetMode);

    /* Validate mode-specific parameters */
    if (param->targetMode == DW_SMBUS_MODE_TARGET) {
        /* Validate target address - I2C addresses 0x00 and 0x78-0x7F are reserved */
        SMBUS_CHECK_PARAM_RETURN(param->config.targetConfig.targetAddr == SMBUS_GENERAL_CALL_ADDR ||
                                (param->config.targetConfig.targetAddr >= SMBUS_RESERVED_ADDR_START &&
                                param->config.targetConfig.targetAddr <= SMBUS_RESERVED_ADDR_END),
                                -EINVAL, "Invalid target address 0x%02X (reserved address range)",
                                param->config.targetConfig.targetAddr);

        /* Validate ARP enable flag */
        SMBUS_CHECK_PARAM_RETURN(param->config.targetConfig.enableArp > SMBUS_BOOL_MAX,
                                -EINVAL, "Invalid ARP enable flag %d (max: %d)",
                                param->config.targetConfig.enableArp, SMBUS_BOOL_MAX);
    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        /* Validate address mode */
        SMBUS_CHECK_PARAM_RETURN(param->config.masterConfig.addrMode > SMBUS_ADDR_MODE_10BIT,
                                -EINVAL, "Invalid master address mode %d (max: %d)",
                                param->config.masterConfig.addrMode, SMBUS_ADDR_MODE_10BIT);

        /* Validate master speed - should be one of the supported SMBus speeds */
        SMBUS_CHECK_PARAM_RETURN(param->config.masterConfig.speed != SMBUS_SPEED_100KHZ &&
                                param->config.masterConfig.speed != SMBUS_SPEED_400KHZ &&
                                param->config.masterConfig.speed != SMBUS_SPEED_1MHZ,
                                -EINVAL, "Invalid master speed %u Hz. Supported speeds: %u Hz, %u Hz, %u Hz",
                                param->config.masterConfig.speed,
                                SMBUS_SPEED_100KHZ, SMBUS_SPEED_400KHZ, SMBUS_SPEED_1MHZ);
    }

    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) return ret;

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
    if (param->targetMode == DW_SMBUS_MODE_TARGET) {
        /* Copy target configuration */
        dev->targetAddr = param->config.targetConfig.targetAddr;
        LOGD("%s(): target config - addr=0x%02X, arp=%d\n", __func__,
             dev->targetAddr, param->config.targetConfig.enableArp);
        /* Note: enableArp could be stored in flags field if needed */
        /* Update driver configuration to reflect target mode */
        pDrvData->sbrCfg.masterMode = SMBUS_MODE_TARGET;  /* Set to target mode */

        /* Initialize target resources */
        ret = smbusInitTargetResources(pDrvData);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s(): Failed to initialize target resources, ret=%d\n", __func__, ret);
            goto exit;
        }

    } else if (param->targetMode == DW_SMBUS_MODE_MASTER) {
        /* Copy master configuration */
        dev->addrMode = param->config.masterConfig.addrMode;
        dev->clkRate = param->config.masterConfig.speed;
        /* Timing parameters will be configured in smbusConfigureMaster() */
        LOGD("%s(): Master config - addrMode=%d, speed=%u, clkRate=%u\n", __func__,
             dev->addrMode, param->config.masterConfig.speed, dev->clkRate);

        /* Free target resources when switching to master mode */
        smbusFreetargetResources(pDrvData);

        /* Update driver configuration to reflect master mode */
        pDrvData->sbrCfg.masterMode = SMBUS_MODE_MASTER;  /* Set to master mode */
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
    funcRunEndHelper(devId);
    return ret;
}

/* ======================================================================== */
/*                    Core ARP Functions                                    */
/* ======================================================================== */
/**
 * @brief Register SMBus device general event callback function
 * @param devId SMBus device ID
 * @param callback Pointer to callback function
 * @param userData Pointer to user data, will be passed back during callback
 * @return 0 on success, negative value on failure
 */
S32 smbusRegisterCallback(DevList_e devId, SmbusCallback_t callback, void *userData)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Parameter validity check */
    if (callback == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Use funcRunBeginHelper to uniformly check driver match, initialization status and acquire device lock */
    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Register callback function and user data */
    pDrvData->callback = callback;
    pDrvData->userData = (SmbusEventData_u*)userData;
    LOGI("%s(): SMBus callback function registered successfully, device ID: %d\n", __func__, devId);

    /* Release device lock */
    funcRunEndHelper(devId);

exit:
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): SMBus callback function registration failed, ret=%d, device ID: %d\n", __func__, ret, devId);
    }
    return ret;
}

/**
 * @brief Unregister SMBus device general event callback function
 * @param devId SMBus device ID
 * @param callback Pointer to callback function
 * @return 0 on success, negative value on failure
 */
S32 smbusUnregisterCallback(DevList_e devId, SmbusCallback_t callback)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Parameter validity check */
    if (callback == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Use funcRunBeginHelper to uniformly check driver match, initialization status and acquire device lock */
    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);

    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Check if it's the same callback function */
    if (pDrvData->callback == callback) {
        pDrvData->callback = NULL;
        pDrvData->userData = NULL;
        LOGI("%s(): SMBus callback function unregistered successfully, device ID: %d\n", __func__, devId);
    } else {
        ret = -ENOENT;  /* Callback function does not match */
        LOGW("%s(): SMBus callback function unregistration failed, callback function mismatch, device ID: %d\n", __func__, devId);
    }

    /* Release device lock */
    funcRunEndHelper(devId);

exit:
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): SMBus callback function unregistration failed, ret=%d, device ID: %d\n", __func__, ret, devId);
    }
    return ret;
}
static inline S32 smbusValidateXfer(const SmbusXfer_s *xfer)
{
    if (xfer == NULL) {
        LOGE("SMBus: xfer is NULL\n");
        return -EINVAL;
    }

    /* Check data length limit */
    if (xfer->wLen > SMBUS_BLOCK_MAXLEN || xfer->rLen > SMBUS_BLOCK_MAXLEN) {
        LOGE("SMBus: Length exceeds limit %d\n", SMBUS_BLOCK_MAXLEN);
        return -EINVAL;
    }

    /* [FIX Bug #3] Strictly check buffer pointers: if there is length, there must be a buffer */
    if ((xfer->wLen > 0 && xfer->wBuf == NULL) ||
        (xfer->rLen > 0 && xfer->rBuf == NULL)) {
        LOGE("SMBus: Invalid buffer (Len > 0 but Buf is NULL)\n");
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}

/* =========================================================================
 * Helper 2: Build TX Buffer and handle Write PEC
 * ========================================================================= */
static void smbusPrepareTxBuffer(const SmbusXfer_s *xfer, U8 *txBuf, U16 *pTxPos)
{
    U16 pos = 0;
    bool isBlock = (xfer->flags & SMBUS_FLAG_BLOCK_TRANSFER) != 0;
    bool noCmd   = (xfer->flags & SMBUS_FLAG_NO_COMMAND) != 0;
    bool pecEn   = (xfer->flags & SMBUS_FLAG_PEC_ENABLE) != 0;
    bool isWriteOnly = (xfer->rLen == 0);

    /* 1. Command Code */
    if (!noCmd) {
        txBuf[pos++] = xfer->command;
    }

    /* 2. Data & Byte Count */
    if (xfer->wLen > 0) {
        if (isBlock) {
            txBuf[pos++] = (U8)xfer->wLen;
        }
        memcpy(&txBuf[pos], xfer->wBuf, xfer->wLen);
        pos += xfer->wLen;
    }

    /* 3. Write-Only PEC Construction */
    if (pecEn && isWriteOnly && pos > 0) {
        /* [FIX Bug #10] Use sufficiently large buffer, or reuse stack variables (if size is controllable) */
        U8 pecCalcBuf[SMBUS_PEC_BUF_SIZE]; 
        U8 pIdx = 0;
        U8 targetAddr = xfer->addr & SMBUS_ADDRESS_MASK;

        /* Construct PEC stream: Addr(W) + [Cmd] + [Count] + [Data] */
        pecCalcBuf[pIdx++] = (U8)(targetAddr << 1); 
        
        /* Boundary check to prevent overflow */
        if (pos + 1 > SMBUS_PEC_BUF_SIZE) {
            LOGE("SMBus: PEC buffer overflow risk\n");
        } else {
            memcpy(&pecCalcBuf[pIdx], txBuf, pos);
            pIdx += pos;
            txBuf[pos++] = smbusPecPktConstruct(targetAddr, TRUE, pecCalcBuf, pIdx);
        }
    }

    *pTxPos = pos;
}

/* =========================================================================
 * Helper 3: Post-receive processing (PEC verification & data copy)
 * ========================================================================= */
static S32 smbusVerifyAndCopyRx(SmbusXfer_s *xfer, U8 *rxBuf, U8 *txBuf, U16 txLen)
{
    bool isBlock = (xfer->flags & SMBUS_FLAG_BLOCK_TRANSFER) != 0;
    bool pecEn   = (xfer->flags & SMBUS_FLAG_PEC_ENABLE) != 0;
    U8 *pDataStart = rxBuf;
    U8 actualDataLen = (U8)xfer->rLen;

    /* 1. Parse Block Length */
    if (isBlock) {
        U8 blockCount = rxBuf[0];
        if (blockCount > SMBUS_BLOCK_MAXLEN || blockCount == 0) {
            LOGE("SMBus: Invalid Block Count %d\n", blockCount);
            return -EPROTO;
        }
        actualDataLen = blockCount;
        pDataStart = &rxBuf[1]; /* Block read data offset by 1 byte */
    }

    /* 2. PEC verification */
    if (pecEn) {
        U8 pecCalcBuf[SMBUS_PEC_BUF_SIZE];
        U32 pIdx = 0;
        U8 targetAddr = xfer->addr & SMBUS_ADDRESS_MASK;
        
        /* Reconstruct Write Phase */
        if (txLen > 0 || (xfer->flags & SMBUS_FLAG_NO_COMMAND)) {
            pecCalcBuf[pIdx++] = (U8)(targetAddr << 1);
            memcpy(&pecCalcBuf[pIdx], txBuf, txLen);
            pIdx += txLen;
        }
        
        /* Read Header */
        pecCalcBuf[pIdx++] = (U8)((targetAddr << 1) | 1);

        /* Read Payload (Data + Count if block) */
        U16 payloadLen = isBlock ? (actualDataLen + 1) : actualDataLen;
        
        /* Check buffer boundary again */
        if (pIdx + payloadLen >= SMBUS_PEC_BUF_SIZE) {
            LOGE("SMBus: PEC Verify buffer too small\n");
            return -ENOMEM;
        }

        memcpy(&pecCalcBuf[pIdx], rxBuf, payloadLen);
        pIdx += payloadLen;

        U8 receivedPec = rxBuf[payloadLen];
        U8 calcPec = smbusPecPktConstruct(targetAddr, FALSE, pecCalcBuf, pIdx);

        if (calcPec != receivedPec) {
            LOGE("SMBus: PEC Error (Calc:0x%02X, Recv:0x%02X)\n", calcPec, receivedPec);
            return -EIO;
        }
    }

    /* 3. Copy to User Buffer */
    if (xfer->rBuf) {
        U32 copyLen = (actualDataLen > xfer->rLen) ? xfer->rLen : actualDataLen;
        memcpy(xfer->rBuf, pDataStart, copyLen);
        if (xfer->actualRxLen) {
            *xfer->actualRxLen = copyLen;
        }
    }
    return EXIT_SUCCESS;
}

/**
 * @brief General SMBus transfer interface (Core API)
 * @details Handle all SMBus protocol details: command construction, Block length processing, PEC calculation and verification
 */
S32 smbusTransfer(DevList_e devId, SmbusXfer_s *xfer)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *pDev = NULL;
    
    /* 1. Validation & Lock */
    ret = smbusValidateXfer(xfer);
    if (ret != EXIT_SUCCESS) return ret;

    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) return ret;

    pDev = &pDrvData->pSmbusDev;

    /* 2. Context Setup */
    U8 txStackBuf[SMBUS_STACK_BUF_SIZE] = {0};
    U8 rxStackBuf[SMBUS_STACK_BUF_SIZE] = {0};
    U16 txPos = 0;
    bool noCmd   = (xfer->flags & SMBUS_FLAG_NO_COMMAND) != 0;
    
    /* 3. Prepare TX Data */
    smbusPrepareTxBuffer(xfer, txStackBuf, &txPos);

    /* 4. Build Messages */
    SmbusMsg_s msgs[2];
    S32 msgCount = 0;
    U8 targetAddr = xfer->addr & SMBUS_ADDRESS_MASK;

    /* Msg 0: Write */
    if (txPos > 0 || (noCmd && xfer->rLen == 0)) {
        msgs[msgCount].addr  = targetAddr;
        msgs[msgCount].flags = (xfer->flags & SMBUS_FLAG_READ) ? SMBUS_M_RD : 0; 
        if (noCmd) msgs[msgCount].flags |= SMBUS_FLAG_QUICK_CMD;
        msgs[msgCount].len   = txPos;
        msgs[msgCount].buf   = txStackBuf;
        msgCount++;
    }

    /* Msg 1: Read */
    if (xfer->rLen > 0) {
        U16 rxReqLen = xfer->rLen;
        if (xfer->flags & SMBUS_FLAG_BLOCK_TRANSFER) rxReqLen += 1;
        if (xfer->flags & SMBUS_FLAG_PEC_ENABLE)     rxReqLen += 1;

        msgs[msgCount].addr  = targetAddr;
        msgs[msgCount].flags = SMBUS_M_RD;
        if (xfer->flags & SMBUS_FLAG_BLOCK_TRANSFER) msgs[msgCount].flags |= SMBUS_M_RECV_LEN;
        if (xfer->flags & SMBUS_FLAG_PEC_ENABLE)     msgs[msgCount].flags |= SMBUS_M_PEC;
        msgs[msgCount].len   = rxReqLen;
        msgs[msgCount].buf   = rxStackBuf;
        msgCount++;
    }

    /* 5. Execute */
    ret = smbusExecuteTransfer(pDev, msgs, msgCount);
    if (ret < EXIT_SUCCESS) {
        LOGE("SMBus: Transfer failed ret=%d\n", ret);
        goto unlock;
    }

    /* 6. Post Process RX */
    if (xfer->rLen > 0) {
        ret = smbusVerifyAndCopyRx(xfer, rxStackBuf, txStackBuf, txPos);
    }

unlock:
    funcRunEndHelper(devId);
    return ret;
}
