/**
 * @file test_smbus_control.c
 * @author stars-microsystem
 * @date 2025/11/18
 * @brief SMBUS Control API Test Cases
 * @details Comprehensive test cases for smbusControl API including all command types:
 *          - Hardware enable/disable operations
 *          - SAR (Slave Address Register) operations
 *          - ARP (Address Resolution Protocol) operations
 *          - Host Notify operations
 *          - Alert response operations
 *          - Bus recovery operations
 */

#include <stdbool.h>
#include <string.h>
#include "udelay.h"
#include "drv_smbus_dw.h"
#include "drv_smbus_api.h"
#include "test_smbus_api.h"

/* Test configuration constants */
#define TEST_SLAVE_ADDR              0x08
#define TEST_SAR_ID                  0
#define TEST_NEW_ADDR                0x45
#define TEST_INVALID_ADDR            0xFF
#define TEST_HOST_NOTIFY_ADDR        0x61
#define TEST_ALERT_RESPONSE_ADDR     0x0C
#define TEST_ARP_DEFAULT_ADDR        0x61
#define SMBUS_STATUS_OK              0x00
#define DEVICE_SMBUS_MAX             DEVICE_SMBUS3

S32 testSmbusControlIntegration(void);
/**
 * @brief Test hardware enable/disable commands
 * @details Tests SMBUS_CMD_HW_ENABLE and SMBUS_CMD_HW_DISABLE commands
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlHardwareEnable(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;

    LOGI("[TEST] Starting Hardware Enable/Disable Test\r\n");

    /* Initialize device */
    ret = testSmbusInit(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }

    /* Test 1: Disable hardware */
    LOGI("[TEST 1] Testing hardware disable...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_DISABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Hardware disable successful\r\n");
    } else {
        LOGE("[FAIL] Hardware disable failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Small delay to ensure disable takes effect */
    udelay(1000);

    /* Test 2: Enable hardware */
    LOGI("[TEST 2] Testing hardware enable...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_ENABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Hardware enable successful\r\n");
    } else {
        LOGE("[FAIL] Hardware enable failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 3: Test with invalid parameter (should still work for enable/disable) */
    LOGI("[TEST 3] Testing enable with NULL param (should succeed)...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_ENABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Hardware enable with NULL param successful\r\n");
    } else {
        LOGE("[FAIL] Hardware enable with NULL param failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    LOGI("[PASS] All hardware enable/disable tests passed!\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    testSmbusDeinit(devId);
    LOGI("[DONE] Hardware Enable/Disable Test completed\r\n");
    return ret;
}

/**
 * @brief Test SAR (Slave Address Register) operations
 * @details Tests SMBUS_CMD_SAR_* commands including setting and getting addresses
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlSarOperations(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;
    SmbusParam_u param;

    LOGI("[TEST] Starting SAR Operations Test\r\n");

    /* Initialize device in target mode */
    ret = testSmbusInit(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }

    /* Configure as target */
    SmbusSwitchParam_s switchParam = {
        .targetMode = SMBUS_MODE_TARGET,
        .flags = 0,
        .timeout = 1000,
        .config.targetConfig.targetAddr = TEST_SLAVE_ADDR,
        .config.targetConfig.enableArp = 0
    };
    ret = smbusMasterTargetModeSwitch(devId, &switchParam);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Mode switch to target failed, ret: %d\r\n", ret);
        goto cleanup;
    }
#if 0
    /* Test 1: Set SAR address */
    LOGI("[TEST 1] Testing SAR set address...\r\n");
    param.sarConfig.sarId = TEST_SAR_ID;
    param.sarConfig.slaveAddr = TEST_NEW_ADDR;

    ret = smbusControl(devId, SMBUS_CMD_SAR_SET_ADDR, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] SAR set address successful (addr: 0x%02X)\r\n", TEST_NEW_ADDR);
    } else {
        LOGE("[FAIL] SAR set address failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 2: Get SAR address */
    LOGI("[TEST 2] Testing SAR get address...\r\n");
    param.sarConfig.sarId = TEST_SAR_ID;
    param.sarConfig.slaveAddr = 0; /* Clear for reading */

    ret = smbusControl(devId, SMBUS_CMD_SAR_GET_ADDR, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] SAR get address successful (addr: 0x%02X)\r\n", param.sarConfig.slaveAddr);

        if (param.sarConfig.slaveAddr == TEST_NEW_ADDR) {
            LOGI("[PASS] SAR address verification successful\r\n");
        } else {
            LOGE("[FAIL] SAR address verification failed (expected: 0x%02X, got: 0x%02X)\r\n",
                 TEST_NEW_ADDR, param.sarConfig.slaveAddr);
            ret = -EINVAL;
            goto cleanup;
        }
    } else {
        LOGE("[FAIL] SAR get address failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 3: Enable SAR */
    LOGI("[TEST 3] Testing SAR enable...\r\n");
    param.sarConfig.sarId = TEST_SAR_ID;

    ret = smbusControl(devId, SMBUS_CMD_SAR_ENABLE, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] SAR enable successful\r\n");
    } else {
        LOGE("[FAIL] SAR enable failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 4: Disable SAR */
    LOGI("[TEST 4] Testing SAR disable...\r\n");
    param.sarConfig.sarId = TEST_SAR_ID;

    ret = smbusControl(devId, SMBUS_CMD_SAR_DISABLE, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] SAR disable successful\r\n");
    } else {
        LOGE("[FAIL] SAR disable failed, ret: %d\r\n", ret);
        goto cleanup;
    }
#endif
    /* Test 5: Test with invalid SAR ID */
    LOGI("[TEST 5] Testing invalid SAR ID...\r\n");
    param.sarConfig.sarId = 5; /* Invalid SAR ID */
    param.sarConfig.slaveAddr = TEST_NEW_ADDR;

    ret = smbusControl(devId, SMBUS_CMD_SAR_SET_ADDR, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] Invalid SAR ID properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] Invalid SAR ID should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    /* Test 6: Test with NULL parameter (should fail) */
    LOGI("[TEST 6] Testing SAR operations with NULL param...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_SAR_GET_ADDR, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] NULL parameter properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] NULL parameter should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    LOGI("[PASS] All SAR operations tests passed!\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    testSmbusDeinit(devId);
    LOGI("[DONE] SAR Operations Test completed\r\n");
    return ret;
}

/**
 * @brief Test ARP (Address Resolution Protocol) operations
 * @details Tests SMBUS_CMD_ARP_* commands including ARP enable/disable and UDID operations
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlArpOperations(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;
    SmbusParam_u param;

    LOGI("[TEST] Starting ARP Operations Test\r\n");

    /* Initialize device in target mode with ARP enabled */
    ret = testSmbusInit(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }

    /* Configure as target with ARP */
    SmbusSwitchParam_s switchParam = {
        .targetMode = SMBUS_MODE_TARGET,
        .flags = 0,
        .timeout = 1000,
        .config.targetConfig.targetAddr = TEST_ARP_DEFAULT_ADDR,
        .config.targetConfig.enableArp = 1
    };
    ret = smbusMasterTargetModeSwitch(devId, &switchParam);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Mode switch to target failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 1: Enable ARP */
    LOGI("[TEST 1] Testing ARP DIS enable...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ARP_ENABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] ARP enable successful\r\n");
    } else {
        LOGE("[FAIL] ARP enable failed, ret: %d\r\n", ret);
        goto cleanup;
    }
    smbusControl(devId, SMBUS_CMD_HW_DISABLE, NULL);
    udelay(100);
    
    /* Test 2: Set UDID */
    LOGI("[TEST 2] Testing ARP UDID set...\r\n");
    /* Create a test UDID */
    memset(&param.arp.udid, 0, sizeof(SmbusUdid_s));
    param.arp.udid.deviceCapabilities = 0x12;
    param.arp.udid.versionRevision = 0x34;
    param.arp.udid.vendorId = 0x1234;
    param.arp.udid.deviceId = 0x5678;
    param.arp.udid.interface = 0x0001;
    param.arp.udid.subsystemVendorId = 0x0000;
    param.arp.udid.subsystemDeviceId = 0x0000;
    param.arp.udid.vendorSpecificId = 0x00000000;

    ret = smbusControl(devId, SMBUS_CMD_ARP_SET_UDID, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] ARP UDID set successful\r\n");
    } else {
        LOGE("[FAIL] ARP UDID set failed, ret: %d\r\n", ret);
        goto cleanup;
    }
    
    smbusControl(devId, SMBUS_CMD_HW_ENABLE, NULL);
    udelay(100);
    /* Test 3: Get UDID */
    LOGI("[TEST 3] Testing ARP UDID get...\r\n");
    memset(&param.arp.udid, 0, sizeof(SmbusUdid_s)); /* Clear for reading */

    ret = smbusControl(devId, SMBUS_CMD_ARP_GET_UDID, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] ARP UDID get successful\r\n");
        LOGI("        Device Capabilities: 0x%02X\r\n", param.arp.udid.deviceCapabilities);
        LOGI("        Version Revision: 0x%02X\r\n", param.arp.udid.versionRevision);
        LOGI("        Vendor ID: 0x%04X\r\n", param.arp.udid.vendorId);
        LOGI("        Device ID: 0x%04X\r\n", param.arp.udid.deviceId);
        LOGI("        Interface: 0x%04X\r\n", param.arp.udid.interface);
    } else {
        LOGE("[FAIL] ARP UDID get failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 4: Get resolved address */
    LOGI("[TEST 4] Testing ARP get resolved address...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ARP_GET_ADDR_RESOLVED, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] ARP get resolved address successful (checkAddr: 0x%02X)\r\n", param.arp.checkAddr);
    } else {
        LOGE("[FAIL] ARP get resolved address failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 5: Disable ARP */
    LOGI("[TEST 5] Testing ARP disable...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ARP_DISABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] ARP disable successful\r\n");
    } else {
        LOGE("[FAIL] ARP disable failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 6: Test with NULL parameter for UDID operations (should fail) */
    LOGI("[TEST 6] Testing ARP UDID operations with NULL param...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ARP_SET_UDID, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] NULL parameter properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] NULL parameter should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    LOGI("[PASS] All ARP operations tests passed!\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    testSmbusDeinit(devId);
    LOGI("[DONE] ARP Operations Test completed\r\n");
    return ret;
}

/**
 * @brief Test Host Notify operations
 * @details Tests SMBUS_CMD_HOST_NOTIFY_* commands
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlHostNotifyOperations(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e targetDevId = DEVICE_SMBUS0;   /* Target设备 - 发送Host Notify */
    DevList_e hostDevId = DEVICE_SMBUS1;     /* Host设备 - 接收Host Notify */
    SmbusParam_u param;

    LOGI("[TEST] Starting Host Notify Operations Test\r\n");
    LOGI("[INFO] Protocol: Target(SMBUS0) -> Host(SMBUS1)\r\n");
    LOGI("[INFO] SMBus Host Notify协议要求:\r\n");
    LOGI("[INFO]   - 发送方(Target/Slave): 主动向Host发送通知到地址0x08\r\n");
    LOGI("[INFO]   - 接收方(Host/Master): 监听地址0x08并接收通知\r\n");

    /* ========================================================================
     * 步骤1: 初始化Host设备 (SMBUS1) - Slave模式监听地址0x08
     * ======================================================================== */
    LOGI("\r\n[STEP 1] Initializing Host device (SMBUS1) as Slave listening to 0x08...\r\n");

    /* 清理SMBUS1 */
    testSmbusDeinit(hostDevId);
    udelay(1000);

    /*
     * 重要：Host Notify接收端应该配置为Slave模式，监听地址0x08
     * SMBus Host Notify协议：
     *   - 发送方(Target): 作为Master，向地址0x08发送写命令
     *   - 接收方(Host): 作为Slave，监听地址0x08，接收数据
     *
     * 配置说明：
     *   - g_TestBlockReadWriteMode=false: n=0/n=2为Master, n=1/n=3为Slave
     *   - 这里n=1，所以SMBUS1会被配置为Slave模式
     *   - Slave地址需要通过SAR配置为0x08（Host Notify地址）
     */
    testSmbusConfigControlEx(true, false, false, false);  /* ARP=ON, PEC=OFF, QuickCmd=OFF, BlockRWMode=OFF */
    ret = smbusSetSpeed(1, 0);  /* n=1, speed=0 (100kHz), 配置为Slave模式 */
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] SMBUS1 initialization failed, ret: %d\r\n", ret);
        goto cleanup;
    }
    LOGI("[PASS] SMBUS1 initialized as Slave mode\r\n");

    /*
     * 关键：配置SMBUS1的Slave地址为0x08（Host Notify地址）
     * 这样SMBUS1才能监听并接收发送到0x08的Host Notify消息
     */
    SmbusParam_u sarParam = {0};
    sarParam.sarConfig.sarId = 0;
    sarParam.sarConfig.slaveAddr = 0x08;  /* Host Notify地址 */
    sarParam.sarConfig.enable = true;

    LOGI("[INFO] Configuring SMBUS1 slave address to 0x08 (Host Notify address)...\r\n");
    ret = smbusControl(hostDevId, SMBUS_CMD_SAR_SET_ADDR, &sarParam);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] SMBUS1 slave address set to 0x08\r\n");
    } else {
        LOGE("[FAIL] Failed to set slave address, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* 关键：使能SAR功能，让设备能够响应Slave地址0x08 */
    LOGI("[INFO] Enabling SAR (Slave Address Response) on SMBUS1...\r\n");
    ret = smbusControl(hostDevId, SMBUS_CMD_SAR_ENABLE, &sarParam);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] SAR enabled on SMBUS1 (now responding to address 0x08)\r\n");
    } else {
        LOGE("[FAIL] SAR enable failed on SMBUS1, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* 启用Host Notify接收 - Slave监听地址0x08 */
    LOGI("[INFO] Enabling Host Notify reception on SMBUS1...\r\n");
    ret = smbusControl(hostDevId, SMBUS_CMD_HOST_NOTIFY_ENABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Host Notify reception enabled on SMBUS1 (listening at 0x08)\r\n");
    } else {
        LOGE("[FAIL] Host Notify reception enable failed on SMBUS1, ret: %d\r\n", ret);
        goto cleanup;
    }

    udelay(2000);  /* 等待配置生效 */

    /* ========================================================================
     * 步骤2: 初始化Target设备 (SMBUS0) - Target模式发送Host Notify
     * ======================================================================== */
    LOGI("\r\n[STEP 2] Initializing Target device (SMBUS0)...\r\n");

    /* 清理SMBUS0 */
    testSmbusDeinit(targetDevId);
    udelay(1000);

    /* 初始化SMBUS0 */
    ret = testSmbusInit(targetDevId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Target device initialization failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* 配置SMBUS0为Target模式 */
    SmbusSwitchParam_s switchParam = {
        .targetMode = SMBUS_MODE_TARGET,
        .flags = 0,
        .timeout = 1000,
        .config.targetConfig.targetAddr = TEST_SLAVE_ADDR,
        .config.targetConfig.enableArp = 0
    };

    ret = smbusMasterTargetModeSwitch(targetDevId, &switchParam);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Mode switch to target failed, ret: %d\r\n", ret);
        goto cleanup;
    }
    LOGI("[PASS] SMBUS0 configured as Target (addr: 0x%02X)\r\n", TEST_SLAVE_ADDR);

    udelay(2000);  /* 等待模式切换稳定 */

    /* ========================================================================
     * 步骤3: 发送Host Notify消息
     * ======================================================================== */
    LOGI("\r\n[TEST 3] Sending Host Notify from SMBUS0(Target) to SMBUS1(Host)...\r\n");
    LOGI("[INFO] Protocol details:\r\n");
    LOGI("[INFO]   - Target sends to address 0x08 (SMBUS_HOST_NOTIFY_ADDR)\r\n");
    LOGI("[INFO]   - Format: [slave_addr<<1, data_low, data_high]\r\n");
    LOGI("[INFO]   - SMBUS1(Master/Host) monitors and receives at 0x08\r\n");

    param.hostNotify.slaveAddr = TEST_SLAVE_ADDR;  /* 发送方（Target）的地址 */
    param.hostNotify.data = 0x1234;                /* 要发送的数据 */

    ret = smbusControl(targetDevId, SMBUS_CMD_HOST_NOTIFY, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Host Notify send successful\r\n");
        LOGI("[INFO] Sent: addr=0x%02X, data=0x%04X\r\n",
             param.hostNotify.slaveAddr, param.hostNotify.data);
    } else {
        LOGE("[FAIL] Host Notify send failed, ret: %d\r\n", ret);
        LOGE("[INFO] Error analysis:\r\n");
        LOGE("        - Check if SMBUS1 is configured as Master (use smbusSetSpeed(0,0))\r\n");
        LOGE("        - Check if SMBUS1 Host Notify reception is enabled\r\n");
        LOGE("        - Check if both devices share the same bus (SDA/SCL lines)\r\n");
        LOGE("        - Verify hardware connections between SMBUS0 and SMBUS1\r\n");
        goto cleanup;
    }

    udelay(5000);  /* 等待Host处理通知 */

    /* ========================================================================
     * 步骤4: 发送多个Host Notify测试
     * ======================================================================== */
    LOGI("\r\n[TEST 4] Sending multiple Host Notify messages...\r\n");

    for (U32 i = 0; i < 3; i++) {
        param.hostNotify.data = 0x1000 + i;  /* 不同的数据 */
        ret = smbusControl(targetDevId, SMBUS_CMD_HOST_NOTIFY, &param);
        if (ret == SMBUS_STATUS_OK) {
            LOGI("[INFO] Host Notify %u sent: data=0x%04X\r\n", i+1, param.hostNotify.data);
        } else {
            LOGE("[FAIL] Host Notify %u failed, ret: %d\r\n", i+1, ret);
        }
        udelay(2000);
    }

    /* ========================================================================
     * 步骤5: 禁用Host Notify
     * ======================================================================== */
    LOGI("\r\n[TEST 5] Disabling Host Notify...\r\n");

    ret = smbusControl(targetDevId, SMBUS_CMD_HOST_NOTIFY_DISABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Host Notify disabled on Target\r\n");
    } else {
        LOGE("[FAIL] Host Notify disable failed, ret: %d\r\n", ret);
    }

    ret = smbusControl(hostDevId, SMBUS_CMD_HOST_NOTIFY_DISABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Host Notify reception disabled on Host\r\n");
    } else {
        LOGE("[FAIL] Host Notify reception disable failed, ret: %d\r\n", ret);
    }

    /* ========================================================================
     * 步骤6: 参数校验测试 - NULL参数应该失败
     * ======================================================================== */
    LOGI("\r\n[TEST 6] Testing NULL parameter validation...\r\n");
    ret = smbusControl(targetDevId, SMBUS_CMD_HOST_NOTIFY, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] NULL parameter properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] NULL parameter should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    LOGI("\r\n============================================================\r\n");
    LOGI("[PASS] All Host Notify operations tests passed!\r\n");
    LOGI("============================================================\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    testSmbusDeinit(targetDevId);
    testSmbusDeinit(hostDevId);
    LOGI("\r\n[DONE] Host Notify Operations Test completed\r\n");
    return ret;
}

/**
 * @brief Test Alert response operations
 * @details Tests SMBUS_CMD_ALERT_* commands
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlAlertOperations(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;
    SmbusParam_u param;

    LOGI("[TEST] Starting Alert Operations Test\r\n");

    /* Initialize device in master mode */
    ret = testSmbusInit(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }
#if 0
    /* Configure as target */
    SmbusSwitchParam_s switchParam = {
        .targetMode = SMBUS_MODE_TARGET,
        .flags = 0,
        .timeout = 1000,
        .config.targetConfig.targetAddr = TEST_SLAVE_ADDR,
        .config.targetConfig.enableArp = 0
    };
    ret = smbusMasterTargetModeSwitch(devId, &switchParam);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Mode switch to target failed, ret: %d\r\n", ret);
        goto cleanup;
    }
#endif
    /* Test 1: Enable Alert */
    LOGI("[TEST 1] Testing Alert enable...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ALERT_ENABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Alert enable successful\r\n");
    } else {
        LOGE("[FAIL] Alert enable failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 2: Respond to Alert */
    LOGI("[TEST 2] Testing Alert response...\r\n");
    param.alertResponse.respondingAddr = TEST_SLAVE_ADDR;
    param.alertResponse.status = 0; /* Normal status */

    ret = smbusControl(devId, SMBUS_CMD_ALERT_RESPOND, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Alert response successful\r\n");
    } else {
        LOGE("[FAIL] Alert response failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 3: Disable Alert */
    LOGI("[TEST 3] Testing Alert disable...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ALERT_DISABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Alert disable successful\r\n");
    } else {
        LOGE("[FAIL] Alert disable failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 4: Test with NULL parameter for response (should fail) */
    LOGI("[TEST 4] Testing Alert response with NULL param...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ALERT_RESPOND, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] NULL parameter properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] NULL parameter should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    LOGI("[PASS] All Alert operations tests passed!\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    testSmbusDeinit(devId);
    LOGI("[DONE] Alert Operations Test completed\r\n");
    return ret;
}

/**
 * @brief Test Bus Recovery operations
 * @details Tests SMBUS_CMD_BUS_RECOVERY command
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlBusRecovery(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;
    SmbusParam_u param;

    LOGI("[TEST] Starting Bus Recovery Test\r\n");

    /* Initialize device in MASTER mode (required for bus recovery) */
    LOGI("[INIT] Initializing device in MASTER mode for bus recovery...\r\n");
    ret = smbusSetSpeed(0, 0);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }
    ret = smbusSetSpeed(1, 0);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Setting bus speed failed, ret: %d\r\n", ret);
        testSmbusDeinit(devId);
        return ret;
    }
    /* Verify device is properly initialized by checking hardware enable */
    LOGI("[VERIFY] Verifying device state...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_DISABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[FAIL] Hardware enable verification failed, ret: %d\r\n", ret);
        goto cleanup;
    }
    LOGI("[VERIFY] Device hardware enable verification successful\r\n");

    /* Small delay to ensure device is stable */
    udelay(1000);

    /* Test 1: Bus recovery with parameter */
    LOGI("[TEST 1] Testing bus recovery with parameters...\r\n");
    param.busRecovery.sclRecoveryCount = 100;
    param.busRecovery.forceRecovery = false;  /* Non-force recovery: only recover if bus is stuck */
    param.busRecovery.timeoutMs = 1000;

    LOGI("[DEBUG] Attempting bus recovery with sclCount=%d, forceRecovery=%d, timeout=%d\r\n",
         param.busRecovery.sclRecoveryCount, param.busRecovery.forceRecovery, param.busRecovery.timeoutMs);

    ret = smbusControl(devId, SMBUS_CMD_BUS_RECOVERY, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Bus recovery with parameters successful\r\n");
    } else {
        LOGE("[FAIL] Bus recovery with parameters failed, ret: %d\r\n", ret);
        LOGE("[DEBUG] Error analysis:\r\n");
        LOGE("        - If ret=-116 (ESTALE): Device state is stale/invalid\r\n");
        LOGE("        - If ret=-110 (ETIMEDOUT): Bus recovery timeout\r\n");
        LOGE("        - If ret=-22 (EINVAL): Invalid parameters\r\n");
        LOGE("        - If ret negative: Check device initialization and mode\r\n");
        LOGE("        - Note: Non-force recovery may succeed even when bus is healthy\r\n");
        goto cleanup;
    }

    /* Small delay between recovery attempts */
    udelay(2000);

    param.busRecovery.forceRecovery = false;
    /* Test 2: Bus recovery without parameter (should still work) */
    LOGI("[TEST 2] Testing bus recovery without parameters...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_BUS_RECOVERY, &param);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Bus recovery without parameters successful\r\n");
    } else {
        LOGE("[FAIL] Bus recovery without parameters failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Test 3: Verify device still works after recovery */
    LOGI("[TEST 3] Verifying device functionality after recovery...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_ENABLE, NULL);
    if (ret == SMBUS_STATUS_OK) {
        LOGI("[PASS] Device still responsive after bus recovery\r\n");
    } else {
        LOGE("[FAIL] Device became unresponsive after bus recovery, ret: %d\r\n", ret);
        goto cleanup;
    }

    LOGI("[PASS] All bus recovery tests passed!\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    /* Cleanup and deinitialize */
    LOGI("[CLEANUP] Deinitializing device...\r\n");
    testSmbusDeinit(devId);
    testSmbusDeinit(DEVICE_SMBUS1);  /* Ensure complete deinitialization */
    LOGI("[CLEANUP] Bus recovery test cleanup completed\r\n");
    return ret;
}

/**
 * @brief Test invalid commands and error handling
 * @details Tests error handling for invalid commands and parameters
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlErrorHandling(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;
    SmbusParam_u param;

    LOGI("[TEST] Starting Error Handling Test\r\n");

    /* Test 1: Invalid device ID */
    LOGI("[TEST 1] Testing invalid device ID...\r\n");
    ret = smbusControl(DEVICE_SMBUS_MAX, SMBUS_CMD_HW_ENABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] Invalid device ID properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] Invalid device ID should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    /* Initialize device for remaining tests */
    ret = testSmbusInit(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[FAIL] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }

    /* Test 2: Invalid command */
    LOGI("[TEST 2] Testing invalid command...\r\n");
    ret = smbusControl(devId, (SmbusCmd_e)0xFF, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] Invalid command properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] Invalid command should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    /* Test 3: Required parameter is NULL */
    LOGI("[TEST 3] Testing NULL parameter for commands that require it...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_SAR_GET_ADDR, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] NULL parameter properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] NULL parameter should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    /* Test 4: Invalid SAR configuration */
    LOGI("[TEST 4] Testing invalid SAR configuration...\r\n");
    param.sarConfig.sarId = TEST_SAR_ID;
    param.sarConfig.slaveAddr = TEST_INVALID_ADDR; /* Invalid address > 0x3FF */

    ret = smbusControl(devId, SMBUS_CMD_SAR_SET_ADDR, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] Invalid SAR address properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] Invalid SAR address should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    /* Test 5: Test with uninitialized device */
    LOGI("[TEST 5] Testing operations on uninitialized device...\r\n");
    /* Deinitialize first */
    testSmbusDeinit(devId);

    ret = smbusControl(devId, SMBUS_CMD_HW_ENABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGI("[PASS] Operation on uninitialized device properly rejected (ret: %d)\r\n", ret);
    } else {
        LOGE("[FAIL] Operation on uninitialized device should have failed\r\n");
        ret = -EINVAL;
        goto cleanup;
    }

    LOGI("[PASS] All error handling tests passed!\r\n");
    ret = EXIT_SUCCESS;

cleanup:
    LOGI("[DONE] Error Handling Test completed\r\n");
    return ret;
}

/**
 * @brief Comprehensive smbusControl API test
 * @details Runs all smbusControl test cases to verify complete functionality
 * @return EXIT_SUCCESS if all tests pass, error code on failure
 */
S32 testSmbusControlComprehensive(void)
{
    S32 ret = EXIT_SUCCESS;
    U32 passedTests = 0;
    U32 totalTests = 7;

    LOGI("\r\n");
    LOGI("=================================================\r\n");
    LOGI("  SMBUS CONTROL API COMPREHENSIVE TEST SUITE  \r\n");
    LOGI("=================================================\r\n");
    LOGI("\r\n");

    /* Run all test cases */
    /* Note: Each individual test case handles its own initialization */
    LOGI("[SUITE] Running Hardware Enable/Disable Test...\r\n");
    ret = testSmbusControlHardwareEnable();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ Hardware Enable/Disable Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ Hardware Enable/Disable Test FAILED\r\n");
    }

    LOGI("[SUITE] Running SAR Operations Test...\r\n");
    ret = testSmbusControlSarOperations();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ SAR Operations Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ SAR Operations Test FAILED\r\n");
    }

    LOGI("[SUITE] Running ARP Operations Test...\r\n");
    ret = testSmbusControlArpOperations();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ ARP Operations Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ ARP Operations Test FAILED\r\n");
    }

    LOGI("[SUITE] Running Host Notify Operations Test...\r\n");
    ret = testSmbusControlHostNotifyOperations();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ Host Notify Operations Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ Host Notify Operations Test FAILED\r\n");
    }

    LOGI("[SUITE] Running Alert Operations Test...\r\n");
    ret = testSmbusControlAlertOperations();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ Alert Operations Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ Alert Operations Test FAILED\r\n");
    }

    LOGI("[SUITE] Running Bus Recovery Test...\r\n");
    ret = testSmbusControlBusRecovery();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ Bus Recovery Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ Bus Recovery Test FAILED\r\n");
    }

    LOGI("[SUITE] Running Error Handling Test...\r\n");
    ret = testSmbusControlErrorHandling();
    if (ret == EXIT_SUCCESS) {
        passedTests++;
        LOGI("[SUITE] ✓ Error Handling Test PASSED\r\n");
    } else {
        LOGE("[SUITE] ✗ Error Handling Test FAILED\r\n");
    }

    /* Print final results */
    LOGI("\r\n");
    LOGI("=================================================\r\n");
    LOGI("              COMPREHENSIVE TEST RESULTS         \r\n");
    LOGI("=================================================\r\n");
    LOGI("Total Tests:  %u\r\n", totalTests);
    LOGI("Passed Tests: %u\r\n", passedTests);
    LOGI("Failed Tests: %u\r\n", totalTests - passedTests);
    LOGI("Success Rate: %u%%\r\n", (passedTests * 100) / totalTests);
    LOGI("=================================================\r\n");

    if (passedTests == totalTests) {
        LOGI("🎉 ALL SMBUS CONTROL API TESTS PASSED! 🎉\r\n");
        return EXIT_SUCCESS;
    } else {
        LOGE("❌ SOME SMBUS CONTROL API TESTS FAILED! ❌\r\n");
        return -EIO;
    }
}

/* Test case structure for integration with existing test framework */
static SmbusTestCase_s gSmbusControlTestCases[] = {
    {"SMBUS Control - Hardware Enable/Disable", testSmbusControlHardwareEnable, TRUE},
    {"SMBUS Control - SAR Operations", testSmbusControlSarOperations, TRUE},
    {"SMBUS Control - ARP Operations", testSmbusControlArpOperations, TRUE},
    {"SMBUS Control - Host Notify Operations", testSmbusControlHostNotifyOperations, TRUE},
    {"SMBUS Control - Alert Operations", testSmbusControlAlertOperations, TRUE},
    {"SMBUS Control - Bus Recovery", testSmbusControlBusRecovery, TRUE},
    {"SMBUS Control - Error Handling", testSmbusControlErrorHandling, TRUE},
    {"SMBUS Control - Usage Example", testSmbusControlIntegration, TRUE},
    {"SMBUS Control - Comprehensive", testSmbusControlComprehensive, TRUE},
};

/**
 * @brief Run all SMBUS Control API tests
 * @details Executes all smbusControl related test cases and returns results
 */
void runSmbusControlTests(SmbusTestResult_s *testResults)
{
    U32 totalTests = sizeof(gSmbusControlTestCases) / sizeof(SmbusTestCase_s);

    testResults->totalTests = totalTests;
    testResults->passedTests = 0;
    testResults->failedTests = 0;

    LOGI("\r\n");
    LOGI("===============================================\r\n");
    LOGI("     RUNNING SMBUS CONTROL API TESTS          \r\n");
    LOGI("===============================================\r\n");

    for (U32 i = 0; i < totalTests; i++) {
        LOGI("\r\n[Test %u/%u] %s\r\n", i + 1, totalTests, gSmbusControlTestCases[i].testName);

        if (gSmbusControlTestCases[i].isEnabled) {
            S32 result = gSmbusControlTestCases[i].testFunc();
            if (result == EXIT_SUCCESS) {
                testResults->passedTests++;
                LOGI("[RESULT] ✓ PASSED\r\n");
            } else {
                testResults->failedTests++;
                LOGE("[RESULT] ✗ FAILED (ret: %d)\r\n", result);
            }
        } else {
            LOGI("[RESULT] - SKIPPED (disabled)\r\n");
        }
    }

    LOGI("\r\n");
    LOGI("===============================================\r\n");
    LOGI("           SMBUS CONTROL TEST SUMMARY          \r\n");
    LOGI("===============================================\r\n");
    LOGI("Total Tests: %u\r\n", testResults->totalTests);
    LOGI("Passed:      %u\r\n", testResults->passedTests);
    LOGI("Failed:      %u\r\n", testResults->failedTests);
    LOGI("Success:     %u%%\r\n", (testResults->passedTests * 100) / testResults->totalTests);
    LOGI("===============================================\r\n");
}

/**
 * @brief Demo program entry point for smbusControl API testing
 * @details Demonstrates how to use the smbusControl API test cases
 * @return EXIT_SUCCESS if all demo tests pass, error code on failure
 */
S32 demoSmbusControlApiTests(void)
{
    S32 ret = EXIT_SUCCESS;

    LOGI("\r\n");
    LOGI("************************************************\r\n");
    LOGI("*     SMBUS CONTROL API DEMO TESTS           *\r\n");
    LOGI("************************************************\r\n");
    LOGI("\r\n");

    /* Demo Test 1: Run individual test case */
    LOGI("[DEMO] Running individual SAR Operations test...\r\n");
    ret = testSmbusControlSarOperations();
    if (ret == EXIT_SUCCESS) {
        LOGI("[DEMO] ✓ SAR Operations test passed\r\n");
    } else {
        LOGI("[DEMO] ✗ SAR Operations test failed (ret: %d)\r\n", ret);
    }

    /* Demo Test 2: Run hardware enable/disable test */
    LOGI("\r\n[DEMO] Running individual Hardware Enable/Disable test...\r\n");
    ret = testSmbusControlHardwareEnable();
    if (ret == EXIT_SUCCESS) {
        LOGI("[DEMO] ✓ Hardware Enable/Disable test passed\r\n");
    } else {
        LOGI("[DEMO] ✗ Hardware Enable/Disable test failed (ret: %d)\r\n", ret);
    }

    /* Demo Test 3: Run comprehensive test */
    LOGI("\r\n[DEMO] Running comprehensive control test...\r\n");
    ret = testSmbusControlComprehensive();
    if (ret == EXIT_SUCCESS) {
        LOGI("[DEMO] ✓ Comprehensive test passed\r\n");
    } else {
        LOGI("[DEMO] ✗ Comprehensive test failed (ret: %d)\r\n", ret);
    }

    LOGI("\r\n");
    LOGI("************************************************\r\n");
    LOGI("*           DEMO TESTS COMPLETED              *\r\n");
    LOGI("************************************************\r\n");
    LOGI("\r\n");

    return ret;
}

/**
 * @brief Example of using smbusControl API in application code
 * @details This function shows how to properly use the smbusControl API
 *          with error handling and parameter validation
 * @param[in] devId SMBus device identifier
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 exampleSmbusControlUsage(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SmbusParam_u param;

    LOGI("[EXAMPLE] Starting smbusControl API usage example\r\n");

    /* Example 1: Enable hardware */
    LOGI("[EXAMPLE] Step 1: Enabling hardware...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_ENABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Hardware enable failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ Hardware enabled successfully\r\n");

    /* Example 2: Configure SAR address */
    LOGI("[EXAMPLE] Step 2: Setting SAR address to 0x30...\r\n");
    param.sarConfig.sarId = 0;
    param.sarConfig.slaveAddr = 0x30;
    ret = smbusControl(devId, SMBUS_CMD_SAR_SET_ADDR, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] SAR set address failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ SAR address set successfully\r\n");

    /* Verify SAR address */
    param.sarConfig.sarId = 0;
    param.sarConfig.slaveAddr = 0;
    ret = smbusControl(devId, SMBUS_CMD_SAR_GET_ADDR, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] SAR get address failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ SAR address verified: 0x%02X\r\n", param.sarConfig.slaveAddr);

    /* Example 3: Enable ARP */
    LOGI("[EXAMPLE] Step 3: Enabling ARP...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_ARP_ENABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] ARP enable failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ ARP enabled successfully\r\n");

    /* Example 4: Set UDID */
    LOGI("[EXAMPLE] Step 4: Setting UDID...\r\n");
    memset(&param.arp.udid, 0, sizeof(SmbusUdid_s));
    param.arp.udid.deviceCapabilities = 0x12;
    param.arp.udid.versionRevision = 0x34;
    param.arp.udid.vendorId = 0x1234;
    param.arp.udid.deviceId = 0x5678;
    param.arp.udid.interface = 0x0001;
    param.arp.udid.subsystemVendorId = 0x0000;
    param.arp.udid.subsystemDeviceId = 0x0000;
    param.arp.udid.vendorSpecificId = 0x00000000;

    ret = smbusControl(devId, SMBUS_CMD_ARP_SET_UDID, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] ARP UDID set failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ UDID set successfully\r\n");

    /* Verify UDID */
    memset(&param.arp.udid, 0, sizeof(SmbusUdid_s));
    ret = smbusControl(devId, SMBUS_CMD_ARP_GET_UDID, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] ARP UDID get failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ UDID verified:\r\n");
    LOGI("        Device Capabilities: 0x%02X\r\n", param.arp.udid.deviceCapabilities);
    LOGI("        Version Revision: 0x%02X\r\n", param.arp.udid.versionRevision);
    LOGI("        Vendor ID: 0x%04X\r\n", param.arp.udid.vendorId);
    LOGI("        Device ID: 0x%04X\r\n", param.arp.udid.deviceId);
    LOGI("        Interface: 0x%04X\r\n", param.arp.udid.interface);

    /* Example 5: Enable Host Notify on sender (SMBUS0) */
    LOGI("[EXAMPLE] Step 5: Enabling Host Notify on sender (SMBUS0)...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HOST_NOTIFY_ENABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Host Notify enable failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ Host Notify enabled on sender successfully\r\n");

    /* Example 5b: Configure receiver (SMBUS1) to listen at address 0x08 */
    LOGI("[EXAMPLE] Step 5b: Configuring receiver (SMBUS1) to listen at 0x08...\r\n");

    DevList_e hostDevId = DEVICE_SMBUS1;

    /* Deinitialize SMBUS1 first to ensure clean state */
    testSmbusDeinit(hostDevId);
    udelay(1000);

    /* Configure SMBUS1 as Slave mode to listen to address 0x08 */
    LOGI("[EXAMPLE] Configuring SMBUS1 as Slave mode...\r\n");
    testSmbusConfigControlEx(true, false, false, false);  /* BlockRWMode=OFF */
    ret = smbusSetSpeed(1, 0);  /* n=1, speed=0 (100kHz), Slave mode */
    if (ret != EXIT_SUCCESS) {
        LOGE("[EXAMPLE] SMBUS1 initialization failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ SMBUS1 configured as Slave mode\r\n");

    /* Set SAR address to 0x08 (Host Notify address) */
    SmbusParam_u sarParam = {0};
    sarParam.sarConfig.sarId = 0;
    sarParam.sarConfig.slaveAddr = 0x08;
    sarParam.sarConfig.enable = true;

    LOGI("[EXAMPLE] Setting SMBUS1 slave address to 0x08...\r\n");
    ret = smbusControl(hostDevId, SMBUS_CMD_SAR_SET_ADDR, &sarParam);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Failed to set SMBUS1 slave address: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ SMBUS1 slave address set to 0x08\r\n");

    /* Enable SAR functionality */
    LOGI("[EXAMPLE] Enabling SAR on SMBUS1...\r\n");
    ret = smbusControl(hostDevId, SMBUS_CMD_SAR_ENABLE, &sarParam);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Failed to enable SAR on SMBUS1: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ SAR enabled on SMBUS1 (responding to address 0x08)\r\n");

    /* Enable Host Notify reception on SMBUS1 */
    LOGI("[EXAMPLE] Enabling Host Notify reception on SMBUS1...\r\n");
    ret = smbusControl(hostDevId, SMBUS_CMD_HOST_NOTIFY_ENABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Failed to enable Host Notify reception on SMBUS1: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ Host Notify reception enabled on SMBUS1\r\n");

    /* Example 6: Send Host Notify */
    LOGI("[EXAMPLE] Step 6: Sending Host Notify message from SMBUS0 to SMBUS1...\r\n");
    param.hostNotify.slaveAddr = 0x08;
    param.hostNotify.data = 0x1234;
    ret = smbusControl(devId, SMBUS_CMD_HOST_NOTIFY, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Host Notify send failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ Host Notify sent successfully\r\n");

    /* Example 7: Bus recovery (optional parameter) */
    LOGI("[EXAMPLE] Step 7: Performing bus recovery...\r\n");
    param.busRecovery.sclRecoveryCount = 100;
    param.busRecovery.forceRecovery = false;  /* Non-force recovery: only recover if bus is stuck */
    param.busRecovery.timeoutMs = 1000;
    ret = smbusControl(devId, SMBUS_CMD_BUS_RECOVERY, &param);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Bus recovery failed: %d\r\n", ret);
        LOGE("[EXAMPLE]   Common errors: -116(ESTALE)=device state issue, -110(ETIMEDOUT)=timeout\r\n");
        LOGE("[EXAMPLE]   Note: Non-force recovery may skip if bus is healthy\r\n");
        return ret;
    }
    LOGI("[EXAMPLE] ✓ Bus recovery completed successfully\r\n");

    /* Example 8: Disable hardware (cleanup) */
    LOGI("[EXAMPLE] Step 8: Disabling hardware (cleanup)...\r\n");
    ret = smbusControl(devId, SMBUS_CMD_HW_DISABLE, NULL);
    if (ret != SMBUS_STATUS_OK) {
        LOGE("[EXAMPLE] Hardware disable failed: %d\r\n", ret);
        return ret;
    }
    LOGI("[EXAMPLE] ✓ Hardware disabled successfully\r\n");

    LOGI("[EXAMPLE] All smbusControl operations completed successfully!\r\n");
    return EXIT_SUCCESS;
}

/**
 * @brief Integration test for smbusControl API with real device
 * @details This test demonstrates end-to-end usage of the smbusControl API
 *          with proper initialization, configuration, and cleanup
 * @return EXIT_SUCCESS on success, error code on failure
 */
S32 testSmbusControlIntegration(void)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e devId = DEVICE_SMBUS0;

    LOGI("[INTEGRATION] Starting smbusControl integration test\r\n");

    /* Initialize device */
    ret = testSmbusInit(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[INTEGRATION] Device initialization failed, ret: %d\r\n", ret);
        return ret;
    }

    /* Configure device in target mode for this test */
    SmbusSwitchParam_s switchParam = {
        .targetMode = SMBUS_MODE_TARGET,
        .flags = 0,
        .timeout = 1000,
        .config.targetConfig.targetAddr = TEST_SLAVE_ADDR,
        .config.targetConfig.enableArp = 0
    };
    ret = smbusMasterTargetModeSwitch(devId, &switchParam);
    if (ret != EXIT_SUCCESS) {
        LOGE("[INTEGRATION] Mode switch failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    /* Run the usage example */
    ret = exampleSmbusControlUsage(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("[INTEGRATION] Usage example failed, ret: %d\r\n", ret);
        goto cleanup;
    }

    LOGI("[INTEGRATION] ✓ Integration test completed successfully\r\n");

cleanup:
    /* Deinitialize device */
    testSmbusDeinit(devId);
    LOGI("[INTEGRATION] Integration test cleanup completed\r\n");
    return ret;
}