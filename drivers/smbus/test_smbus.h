/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file test_smbus.h
 * @author stars-microsystem
 * @date 2025/11/18
 * @brief SMBUS test suite header file
 */

#ifndef __TEST_SMBUS_H__
#define __TEST_SMBUS_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include "osp_timer.h"
#include <stdbool.h>
#include <getopt.h>
#include "bsp_device.h"
#include "common_defines.h"
#include "drv_smbus_api.h"
#include "bsp_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "bsp_config.h"
#include "osp_common.h"

/* Test configuration constants */
#define TEST_SMBUS_DEVICE_ID       DEVICE_SMBUS0
#define TEST_SMBUS1_DEVICE_ID      DEVICE_SMBUS1
#define SMBUS_7BIT_ADDR            0
#define SMBUS_10BIT_ADDR           1
#define I2C_TESTSUITE_SLAVE_ADDR   0x54  /* Changed from 0x10 to avoid conflict with BMC at 0x10 */

/* Base addresses and offsets */
#ifndef CONFIG_TEST_I2C_SUIT
#define SMBUS_BASE_ADDR            (0xBE620000)
#define SMBUS_BASE_OFFSET          (0x1000)
#define SMBUS_BASE_ADDR_CAL(n)     (SMBUS_BASE_ADDR + ((n)*SMBUS_BASE_OFFSET))

/* System interrupt definitions */
#define SYS_INT_NUM_SMBUS0         60
#define SYS_INT_NUM_SMBUS1         61
#define SYS_INT_PRIORITY_SMBUS     127
#else
#define SMBUS_BASE_ADDR            (0xBE610000)
#define SMBUS_BASE_OFFSET          (0x1000)
#define SMBUS_BASE_ADDR_CAL(n)     (SMBUS_BASE_ADDR + ((n)*SMBUS_BASE_OFFSET))

#define SYS_INT_NUM_SMBUS0         55
#define SYS_INT_NUM_SMBUS1         56
#define SYS_INT_PRIORITY_SMBUS     127
#endif

/* SMBUS mode definitions */
#define SMBUS_MASTER_MODE          1U
#define SMBUS_SLAVE_MODE           0U

/* Test timeouts and counts */
#define TEST_TIMEOUT_MS            5000
#define TEST_STRESS_COUNT          1000
#define TEST_BUFFER_SIZE           256
#define TEST_MAX_DATA_LEN          32

/* Test result structure */
typedef struct {
    const char *suiteName;
    U32 totalTests;
    U32 passedTests;
    U32 failedTests;
    U32 skippedTests;
    double executionTime;
} SmbusTestResult_s;

/* Test suite structure */
typedef struct {
    const char *name;
    const char *description;
    void (*runner)(SmbusTestResult_s *);
    bool enabled;
} SmbusTestSuite_s;

/* Global test variables */
extern bool gSmbusSimHwErr;
extern bool gSmbusSimulateTimeout;
extern bool gSmbusSimulateOverrun;
extern U32 gSmbusInitDrvCount;
U32 gSmbusDeinitCount;
extern U32 gSmbusLockCount;
extern U32 gSmbusUnlockCount;
extern U32 gSmbusCallbackCount;

/* Enhanced SMBUS Test Assertion Macros */

/**
 * @brief Basic assertion macro - asserts condition is true
 * @param condition Boolean condition to test
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT(condition, message)                                                     \
    do {                                                                                      \
        if (!(condition)) {                                                                   \
            LOGE("[FAIL] Assertion failed: %s\r\n", message);                                 \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Condition: %s\r\n", #condition);                                     \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            /* Small delay after failure */                                                   \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that a condition is true (boolean)
 * @param condition Boolean condition to test
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_TRUE(condition, message)                                                \
    do {                                                                                      \
        if (!(condition)) {                                                                   \
            LOGE("[FAIL] Expected true, got false: %s\r\n", message);                        \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Condition: %s\r\n", #condition);                                     \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that two values are equal
 * @param value1 First value
 * @param value2 Second value (expected value)
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_EQUAL(value1, value2, message)                                          \
    do {                                                                                      \
        if ((value1) != (value2)) {                                                          \
            LOGE("[FAIL] Values not equal: %s\r\n", message);                                \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Expected: %d (0x%X), Got: %d (0x%X)\r\n",                           \
                   (int)(value2), (unsigned int)(value2),                                   \
                   (int)(value1), (unsigned int)(value1));                                  \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that a function return value is OK (0)
 * @param ret Return value from function
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_OK(ret, message)                                                        \
    do {                                                                                      \
        if ((ret) != 0) {                                                                     \
            LOGE("[FAIL] Function returned error: %s\r\n", message);                         \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Return value: %d (0x%X)\r\n", (int)(ret), (unsigned int)(ret));     \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that a function return value is not OK (non-zero)
 * @param ret Return value from function
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_NOT_OK(ret, message)                                                    \
    do {                                                                                      \
        if ((ret) == 0) {                                                                     \
            LOGE("[FAIL] Function unexpectedly succeeded: %s\r\n", message);                 \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Expected non-zero return value, got: 0\r\n");                      \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that a pointer is not NULL
 * @param ptr Pointer to test
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_NOT_NULL(ptr, message)                                                  \
    do {                                                                                      \
        if ((ptr) == NULL) {                                                                  \
            LOGE("[FAIL] Pointer is NULL: %s\r\n", message);                                 \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that a pointer is NULL
 * @param ptr Pointer to test
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_NULL(ptr, message)                                                       \
    do {                                                                                      \
        if ((ptr) != NULL) {                                                                  \
            LOGE("[FAIL] Pointer is not NULL: %s\r\n", message);                             \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Pointer value: %p\r\n", (void*)(ptr));                               \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that two memory blocks are equal
 * @param ptr1 First memory block
 * @param ptr2 Second memory block
 * @param size Size in bytes to compare
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_MEM_EQUAL(ptr1, ptr2, size, message)                                    \
    do {                                                                                      \
        if (memcmp((ptr1), (ptr2), (size)) != 0) {                                            \
            LOGE("[FAIL] Memory blocks not equal: %s\r\n", message);                         \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Size: %u bytes\r\n", (unsigned int)(size));                         \
            /* Print first few differing bytes */                                             \
            U8 *p1 = (U8*)(ptr1);                                                             \
            U8 *p2 = (U8*)(ptr2);                                                             \
            U32 min_print = (size) < 16 ? (size) : 16;                                         \
            for (U32 i = 0; i < min_print; i++) {                                             \
                if (p1[i] != p2[i]) {                                                        \
                    LOGE("[FAIL] First difference at offset %u: 0x%02X != 0x%02X\r\n",    \
                           i, p1[i], p2[i]);                                               \
                    break;                                                                  \
                }                                                                             \
            }                                                                                 \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/**
 * @brief Assert that an unsigned value is within a range (inclusive)
 * @param value Value to test
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @param message Error message to display on failure
 */
#define SMBUS_ASSERT_IN_RANGE(value, min, max, message)                                      \
    do {                                                                                      \
        if ((value) < (min) || (value) > (max)) {                                            \
            LOGE("[FAIL] Value out of range: %s\r\n", message);                             \
            LOGE("[FAIL] Location: %s:%d in %s()\r\n", __FILE__, __LINE__, __FUNCTION__);    \
            LOGE("[FAIL] Value: %u, Range: [%u, %u]\r\n",                                    \
                   (unsigned int)(value), (unsigned int)(min), (unsigned int)(max));        \
            /* Cleanup on failure */                                                          \
            smbusReset(TEST_SMBUS_DEVICE_ID);                                                 \
            udelay(1000);                                                                     \
            return EXIT_FAILURE;                                                              \
        }                                                                                     \
    } while (0)

/* Legacy macro for backward compatibility */
#define GPIO_ASSERT(condition, message) SMBUS_ASSERT(condition, message)

/* Legacy single-parameter macro for backward compatibility */
#define SMBUS_ASSERT_LEGACY(condition) SMBUS_ASSERT(condition, "Assertion failed")

/* ======================================================================== */
/*                            测试框架宏定义                                  */
/* ======================================================================== */

/** 测试用例结构体 */
typedef struct {
    const char *name;        ///< 测试用例名称
    S32 (*func)(void);       ///< 测试函数指针
    S32 result;              ///< 测试结果
} TestCase_s;

/** 测试结果 */
#define TEST_PASS            EXIT_SUCCESS
#define TEST_FAIL            (-1)

/** 断言宏 */
#define TEST_ASSERT(cond) do { \
    if (!(cond)) { \
        LOGE("[ASSERT FAIL] %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        return TEST_FAIL; \
    } \
} while(0)

#define TEST_ASSERT_EQ(a, b)  TEST_ASSERT((a) == (b))
#define TEST_ASSERT_NE(a, b)  TEST_ASSERT((a) != (b))
#define TEST_ASSERT_LT(a, b)  TEST_ASSERT((a) < (b))
#define TEST_ASSERT_LE(a, b)  TEST_ASSERT((a) <= (b))
#define TEST_ASSERT_GT(a, b)  TEST_ASSERT((a) > (b))
#define TEST_ASSERT_GE(a, b)  TEST_ASSERT((a) >= (b))

#define TEST_ASSERT_NULL(ptr)     TEST_ASSERT((ptr) == NULL)
#define TEST_ASSERT_NOT_NULL(ptr) TEST_ASSERT((ptr) != NULL)

/** 测试环境清理函数 */
extern void cleanupTestEnv(void);

/** 测试运行器 */
S32 runTestSuite(TestCase_s *tests, U32 count);

/** 测试设备ID (使用第一个SMBUS设备) */
#define TEST_DEV_ID          DEVICE_SMBUS0

/** 测试配置 */
extern SmbusUserConfigParam_s g_testConfig;

/* Helper functions */
/**
 * @brief Reset SMBUS test state
 */
static inline void resetSmbusTestState(void)
{
    gSmbusSimHwErr = false;
    gSmbusSimulateTimeout = false;
    gSmbusSimulateOverrun = false;
    gSmbusInitDrvCount = 0;
    gSmbusDeinitCount = 0;
    gSmbusLockCount = 0;
    gSmbusUnlockCount = 0;
    gSmbusCallbackCount = 0;
}

/**
 * @brief Wait for callback with timeout
 * @param timeoutMs Timeout in milliseconds
 * @return true if callback triggered, false on timeout
 */
static inline bool waitForSmbusCallback(U32 timeoutMs)
{
    U32 elapsed = 0;
    const U32 stepMs = 10;

    while (elapsed < timeoutMs) {
        if (gSmbusCallbackCount > 0) {
            return true;
        }
        udelay(stepMs * 1000);
        elapsed += stepMs;
    }
    return false;
}

/**
 * @brief Initialize SMBUS device for testing
 * @param devId Device ID
 * @return 0 on success, error code on failure
 */
static inline S32 testSmbusInit(DevList_e devId)
{
    SmbusUserConfigParam_s config = {
    #ifdef CONFIG_TEST_SUITS_1
        .base = (void *)SMBUS_BASE_ADDR_CAL(4),
        .busSpeedHz = 100000,
        .irqNo = SYS_INT_NUM_SMBUS0 + 4,
        .irqPrio = SYS_INT_PRIORITY_SMBUS,
        .masterMode = SMBUS_MASTER_MODE,
        .addrMode = SMBUS_7BIT_ADDR,
        .targetAddrLow = I2C_TESTSUITE_SLAVE_ADDR,
        .interruptMode = 1,
        .isArpEnable = false,
    #endif
        .udidWord0 = 0
    };

    S32 ret = smbusInit(devId, &config);
    if (ret == 0) {
        gSmbusInitDrvCount++;
    }
    return ret;
}

/**
 * @brief Deinitialize SMBUS device
 * @param devId Device ID
 * @return 0 on success, error code on failure
 */
static inline S32 testSmbusDeinit(DevList_e devId)
{
    S32 ret = smbusDeInit(devId);
    if (ret == 0) {
        gSmbusDeinitCount++;
    }
    return ret;
}

/**
 * @brief Get system time in microseconds
 * @return Current system time in microseconds
 */
static inline U64 getSystemTimeUs(void)
{
    /* Use OSP clock functions if available */
    return (U64)(ospClockGetTicksSinceBoot() * 1000000ULL / ospClockGetTicksPerSecond());
}

/* Function declarations for helper functions defined in test_smbus_api.c */
S32 smbusI2cWriteTest(DevList_e devId, U8 slaveAddr, U8 cmd, const U8 *data, U32 length);
S32 smbusI2cReadTest(DevList_e devId, U8 slaveAddr, U8 cmd, U8 *data, U32 length);
S32 smbusWriteByte(DevList_e devId, U8 slaveAddr, U8 cmd, U8 data);
S32 smbusBlockWrite(DevList_e devId, U8 slaveAddr, U8 cmd, const U8 *data, U8 count);
S32 smbusReadByte(DevList_e devId, U8 slaveAddr, U8 cmd, U8 *data);

/* isDrvInit is already declared in bsp_api.h */

/* Test framework type definitions */
typedef S32 (*SmbusTestFunc_t)(void);

/* SMBUS API test function declarations */
S32 testSmbus100kMasterWrite(void);
S32 testSmbus100kMasterRead(void);
S32 testSmbusQuickCmd(void);
S32 testLoopbackSendByte(void);
S32 testSmbusReceiveByte(void);
S32 testSmbusHostNotify(void);
S32 testSmbusArpPrepare(void);
S32 testSmbusArpResetGeneral(void);
S32 testSmbusArpGetUdid(void);
S32 testSmbusControllerReset(void);
S32 testSmbusPecControl(void);

S32 testSmbus100kLoopback(void);
S32 testSmbus400kLoopback(void);
S32 testSmbus1mLoopback(void);
/* BMC diagnosis test functions */
S32 testBmcProtocolDiagnosis(void);
S32 testBmcMaxLengthDiagnosis(void);

#endif /* __TEST_SMBUS_H__ */