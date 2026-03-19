/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file test_smbus_api.h
 * @author stars-microsystem
 * @date 2025/11/18
 * @brief SMBUS API Test Cases Header
 * @note This file is deprecated - use test_smbus.h instead
 */

#ifndef __TEST_SMBUS_API_H_
#define __TEST_SMBUS_API_H_

/* Include the main test header for backward compatibility */
#include "osp_timer.h"
#include "test_smbus.h"

#define TEST_I2C_SUIT   1
/* Test case structure */
typedef struct {
    const S8 *testName;
    SmbusTestFunc_t testFunc;
    Bool isEnabled;
} SmbusTestCase_s;

/* Legacy SMBUS Configuration Macros - kept for backward compatibility */
#define SMBUS_CONFIG_DEFAULT() { \
    .base = (void*)(SMBUS_BASE_ADDR + 0 * SMBUS_BASE_OFFSET), \
    .busSpeedHz = 100000, \
    .udidWord0 = 0, \
    .irqNo = SYS_INT_NUM_SMBUS0, \
    .irqPrio = SYS_INT_PRIORITY_SMBUS, \
    .masterMode = SMBUS_MASTER_MODE, \
    .addrMode = SMBUS_7BIT_ADDR, \
    .slaveAddrLow = I2C_TESTSUITE_SLAVE_ADDR, \
    .interruptMode = 1, \
    .isArpEnable = false \
}

#define SMBUS_CONFIG_400K() { \
    .base = (void*)(SMBUS_BASE_ADDR), \
    .busSpeedHz = 400000, \
    .udidWord0 = 0, \
    .irqNo = 45, \
    .irqPrio = 5, \
    .masterMode = 1, \
    .addrMode = 0, \
    .slaveAddrLow = 0x21, \
    .interruptMode = 1, \
    .isArpEnable = false \
}

#define SMBUS_CONFIG_1M() { \
    .base = (void*)(SMBUS_BASE_ADDR), \
    .busSpeedHz = 1000000, \
    .udidWord0 = 0, \
    .irqNo = 45, \
    .irqPrio = 5, \
    .masterMode = 1, \
    .addrMode = 0, \
    .slaveAddrLow = 0x21, \
    .interruptMode = 1, \
    .isArpEnable = false \
}

#ifdef TEST_I2C_SUIT
static const U8 smbusDeviceIds[] = {
        DEVICE_SMBUS0, // 索引 0
        DEVICE_SMBUS1, // 索引 1
        DEVICE_SMBUS2, // 索引 2
        DEVICE_SMBUS3  // 索引 3
    };
#else
static const U8 smbusDeviceIds[] = {
        DEVICE_I2C0, // 索引 0
        DEVICE_I2C1, // 索引 1
        DEVICE_I2C2, // 索引 2
        DEVICE_I2C3  // 索引 3
    };
#endif
/* Test function declarations */
S32 testSmbus100kMasterWrite(void);
S32 testSmbus100kMasterRead(void);
S32 testSmbus400kMasterWrite(void);
S32 testSmbus400kMasterRead(void);
S32 testSmbus1mMasterWrite(void);
S32 testSmbus1mMasterRead(void);
S32 testSmbus100kLoopback(void);
S32 testSmbus400kLoopback(void);
S32 testSmbus1mLoopback(void);
S32 testSmbusQuickCmd(void);
S32 testLoopbackSendByte(void);
S32 testSmbusReceiveByte(void);
S32 testSmbusWriteByte(void);
S32 testLoopbackSendByte(void);
S32 testSmbusReadByte(void);
S32 testSmbusWriteWord(void);
S32 testSmbusReadWord(void);
S32 testSmbusWrite32(void);
S32 testSmbusRead32(void);
S32 testSmbusWrite64(void);
S32 testSmbusRead64(void);
S32 testSmbusBlockWrite(void);
S32 testSmbusBlockRead(void);
S32 testSmbusProcessCall(void);
S32 testSmbusBlockProcessCall(void);
S32 testSmbusWriteByteProtocol(void);
S32 testSmbusWriteWordProtocol(void);
S32 testSmbusWrite32Protocol(void);
S32 testSmbusWrite64Protocol(void);
S32 testSmbusBlockWriteProtocol(void);
S32 testSmbusProcessCallProtocol(void);
S32 testSmbusBlockProcessCallProtocol(void);
S32 testSmbusBlockProcessCall(void);
S32 testSmbusHostNotify(void);
S32 testSmbusArpPrepare(void);
S32 testSmbusArpResetGeneral(void);
S32 testSmbusArpGetUdid(void);
//S32 testSmbusArpAssignAddress(void);
S32 testSmbusArpFullSequence(void);
S32 testSmbusControllerReset(void);
S32 testSmbusPecControl(void);

S32 smbusSetSpeed(U8 n, U32 speed);
S32 smbusI2cWriteTest(DevList_e devId, U8 slaveAddr, U8 cmd, const U8 *data, U32 length);
S32 smbusI2cReadTest(DevList_e devId, U8 slaveAddr, U8 cmd, U8 *data, U32 length);
S32 smbusQuickCmd(DevList_e devId, U8 slaveAddr, U8 rwBit);
S32 smbusSendByte(DevList_e devId, U8 slaveAddr, U8 data);
S32 smbusReceiveByte(DevList_e devId, SmbusXfer_s *xfer);
S32 smbusWriteByte(DevList_e devId, U8 slaveAddr, U8 cmd, U8 data);
S32 smbusReadByte(DevList_e devId, U8 slaveAddr, U8 cmd, U8 *data);
S32 smbusWriteWord(DevList_e devId, U8 slaveAddr, U8 cmd, U16 data);
S32 smbusReadWord(DevList_e devId, U8 slaveAddr, U8 cmd, U16 *data);
S32 smbusWrite32(DevList_e devId, U8 slaveAddr, U8 cmd, U32 data);
S32 smbusRead32(DevList_e devId, U8 slaveAddr, U8 cmd, U32 *data);
S32 smbusWrite64(DevList_e devId, U8 slaveAddr, U8 cmd, U64 data);
S32 smbusRead64(DevList_e devId, U8 slaveAddr, U8 cmd, U64 *data);
S32 smbusBlockWrite(DevList_e devId, U8 slaveAddr, U8 cmd, const U8 *data, U8 count);
S32 smbusBlockRead(DevList_e devId, U8 slaveAddr, U8 cmd, U8 *data, U8 *count);
S32 smbusProcessCall(DevList_e devId, U8 slaveAddr, U8 cmd, U16 writeData, U16 *readData);
S32 smbusBlockProcessCall(DevList_e devId, U8 slaveAddr, U8 cmd, const U8 *writeData, U8 writeCount, U8 *readData, U8 *readCount);
S32 smbusPecEnable(DevList_e devId, Bool enable);
S32 smbusTxFifoLevelGet(DevList_e devId, U32 *level);
S32 smbusRxFifoLevelGet(DevList_e devId, U32 *level);

/* ARP function declarations */
S32 smbusArpPrepare(DevList_e devId);
S32 smbusArpGetUdidGeneral(DevList_e devId, SmbusUdid_s *udid);

/* Additional API test functions */
S32 testSmbusArpFunction(void);
S32 testSmbusProtocolConsistency(void);
void runSmbusApiTests(SmbusTestResult_s *testSuites);

/* SMBus Slave test functions */
U32 testSmbusI2CReadSlaver(U8 slaveAddr, U8 *rxBuffer, U32 rxLen);
U32 testSmbusI2CWriteSlaver(U8 slaveAddr, const U8 *txData, U32 txLen);
S32 testSmbusSlaveModeWithBufferComparison(DevList_e devId);

/**
 * @brief Demo: 测试 SMBus Master 读 / Slave 写 场景（无参数接口）
 * @details 方便串口命令直接调用，使用默认配置
 */
void testSmbusSlaverWrite(void);

/* Test utility functions */
void smbusRunAllTests(void);
S32 cmdSmbusTest(int argc, char *argv[]);
void smbusCmdRegister(void);

/* BMC diagnosis test functions */
S32 testBmcProtocolDiagnosis(void);
S32 testBmcMaxLengthDiagnosis(void);
S32 testBmcDeviceExistenceCheck(void);
U8 testBmcFindSuggestedAddress(void);

void testSmbusConfigControlEx(bool arpEnable, bool pecEnable, bool quickCmd, bool blockReadWriteMode);

/**
 * @brief 执行主机模式ARP（地址解析协议）发现过程
 * @details 实现完整的SMBus ARP发现工作流程，用于枚举并为总线上的所有SMBus设备分配地址。
 *          此函数协调完整的ARP序列：准备 → 重置 → 发现循环（获取UDID → 安装 → 分配）。
 *          每次使用通用ARP地址（0x61）发现一个设备，在软件设备列表中注册，
 *          并从配置的地址池中分配唯一地址。
 *
 * @param[in] devId 来自DevList_e枚举的SMBus主机设备标识符
 * @param[in,out] masterInfo 指向ARP主机上下文结构的指针，包含：
 *                - addressPoolStart：分配的第一个可用地址（通常为0x08）
 *                - addressPoolEnd：分配的最后一个可用地址（通常为0x77）
 *                - deviceList：已发现设备的链表（在过程中修改）
 *                - deviceCount：已发现设备计数器（每次成功分配时递增）
 *
 * @return 如果ARP发现过程完成（即使没有发现设备），返回EXIT_SUCCESS (0)，
 *         关键失败时返回负错误码：
 *         -EINVAL：无效参数（NULL masterInfo，无效devId）
 *         -EBUSY：设备被其他操作锁定
 *         -EIO：关键通信错误（总线错误、超时、冲突）
 *         -ENOTSUP：所需的HAL操作不可用
 *
 * @note 此函数使用阻塞操作，可能需要较长时间
 * @note 发现循环持续直到没有更多设备响应（NACK或超时）
 * @note 失败的地址分配可能会部分填充设备列表
 * @note 每个设备被顺序发现 - 不支持并行发现
 * @note 设备必须处于ARP capable状态并在默认地址（0x61）监听
 * @note 地址池耗尽可能会阻止某些设备被分配
 *
 * @par ARP序列概览：
 * 1. **准备阶段**：发送Prepare to ARP命令（广播到0x61）
 *    - 所有ARP设备设置ARP标志并在默认地址监听
 *
 * 2. **重置阶段**：发送General Reset命令（广播到0x61）
 *    - 所有设备重置到默认状态和地址（0x61）
 *    - 清除任何现有的ARP分配
 *
 * 3. **发现循环**：重复直到总线空闲：
 *    - **获取UDID**：从0x61的第一个设备读取UDID
 *    - **安装**：在软件masterInfo->deviceList中注册设备
 *    - **分配**：从addressPoolStart向上分配唯一地址
 *    - 下次迭代发现下一个设备（第一个设备不再响应）
 *
 * @par 错误处理策略：
 * - **NACK响应**：正常的发现结束信号（所有设备已分配）
 * - **超时**：视为NACK（没有更多设备要发现）
 * - **安装错误**：记录并中断发现（可以使用不同策略重试）
 * - **分配错误**：中断发现（部分完整的设备列表）
 *
 * @par 设备寻址：
 * - 设备从addressPoolStart到addressPoolEnd递增分配
 * - 第一个设备获得addressPoolStart（通常为0x08）
 * - 每个后续设备获得下一个可用地址
 * - 如果地址池耗尽（nextAddr > addressPoolEnd），发现停止
 *
 * @warning 此函数不可重入 - 不要从多个线程调用
 * @warning 对系统初始化至关重要 - 失败可能使总线处于不一致状态
 * @warning 地址分配在硬件中持久 - 需要重置才能清除
 * @warning 在发现过程中不支持热插拔
 * @warning 假设所有设备使用兼容的SMBus ARP实现
 *
 * @par 使用示例：
 * @code
 * SmbusArpMaster_s arpMaster = {
 *     .addressPoolStart = 0x08,
 *     .addressPoolEnd = 0x77,
 *     .deviceList = NULL,
 *     .deviceCount = 0,
 *     // ... 其他字段 ...
 * };
 *
 * S32 ret = smbusMasterArpProcess(DEVICE_SMBUS0, &arpMaster);
 * if (ret == EXIT_SUCCESS) {
 *     printf("发现了 %d 个设备\n", arpMaster.deviceCount);
 *     // 遍历 arpMaster.deviceList 访问已分配的设备
 * }
 * @endcode
 *
 * @par 依赖关系：
 * - smbusArpPrepareToARP：发送Prepare to ARP广播命令
 * - smbusArpResetDevice：发送General Reset广播命令
 * - smbusArpGetUdidGeneral：在默认地址发现一个UDID
 * - ArpDevInstall：在软件列表中注册设备
 * - smbusArpAssignAddress：为设备分配硬件地址
 *
 * @par 线程安全性：
 * 不是线程安全的。如需要，应在调用方级别处理设备锁定。
 */
S32 smbusMasterArpProcess(DevList_e devId, SmbusArpMaster_s *masterInfo);

/**
 * @brief 设置 SMBUS 目标地址 (动态配置)
 * @param targetAddr 目标地址 (7位地址, 如 0x61)
 * @note 此设置将在下一次调用 smbusSetSpeed 时生效
 */
void testSmbusSetTargetAddr(U8 targetAddr);

/**
 * @brief 获取当前配置的 SMBUS 目标地址
 * @return 当前配置的目标地址
 */
U8 testSmbusGetTargetAddr(void);

/**
 * @brief 测试向 BMC Slave 设备发送多个数据包
 * @details 完整的测试流程：确保 Master 模式，发送 7 个数据包到外部 BMC Slave 设备
 *
 * @param[in] deviceId SMBus 设备 ID (如 DEVICE_SMBUS0, DEVICE_SMBUS1 等)
 * @param[in] slaveAddress 目标 BMC Slave 设备的地址 (7位地址，如 0x10)
 *
 * @return 0 成功，负值 失败错误码：
 *         -1: 参数错误
 *         -2: 切换到 Master 模式失败
 *         -3: 数据包发送失败
 *
 * @note 此函数不包含 smbusInit，假设设备已经初始化
 * @note 发送 7 个不同的数据包，每个包的数据内容和长度可自定义
 * @note 使用 400kHz 总线速度
 * @note ⚠️ 重要：此函数假设 BMC 是外部独立设备，不是本控制器自己
 * @note ⚠️ 修复：移除了"先切换为 Slave"的步骤，避免地址冲突
 */
S32 testBmcSendPacketsToSlave(DevList_e deviceId, U8 slaveAddress);

/**
 * @brief BMC 向 Slave 发送数据包的完整测试流程（带地址参数）
 * @details 这是增强版本，可以指定 BMC 地址，更灵活地测试
 *
 * @param[in] bmcAddr BMC 的地址（默认 0x10，可以改为其他地址如 0x21）
 *
 * @return 0 成功，负值失败
 *
 * @note 此函数会：
 *       1. 检查 BMC 是否存在
 *       2. 发送测试数据包到 Slave
 *       3. 运行诊断测试
 */
S32 testBmcSendPacketsToSlaveDefaultEx(U8 bmcAddr);

/**
 * @brief BMC 向 Slave 发送数据包的完整测试流程（默认地址）
 * @details 使用默认 BMC 地址 (0x10) 的便捷函数
 *
 * @return 0 成功，负值失败
 *
 * @note 此函数调用 testBmcSendPacketsToSlaveDefaultEx(0x10)
 */
S32 testBmcSendPacketsToSlaveDefault(void);

/**
 * @brief 诊断 SMBus 通信问题
 * @details 检查设备状态、FIFO 大小、中断状态等，帮助排查通信失败原因
 *
 * @param[in] deviceId SMBus 设备 ID
 *
 * @return 0 成功，负值 失败
 */
S32 testSmbusDiagnose(DevList_e deviceId);

/**
 * @brief 便捷函数：诊断 SMBUS0
 * @details 使用默认配置诊断 SMBUS0
 *
 * @return 0 成功，负值 失败
 */
S32 testSmbusDiagnoseDefault(void);

/**
 * @brief 扫描 I2C/SMBus 总线上的所有设备
 * @details 遍历所有可能的 I2C 地址（0x08-0x77），检测总线上的设备
 *
 * @param[in] deviceId SMBus 设备 ID
 *
 * @return 扫描到的设备数量
 *
 * @note 此函数会尝试向每个地址写入 0 字节来检测设备
 * @note 标准的 I2C 地址范围是 0x08-0x77
 * @note 扫描过程可能需要较长时间（每个地址约 1ms，总共约 120ms）
 */
U32 testSmbusScanI2CBus(DevList_e deviceId);

/**
 * @brief 便捷函数：扫描 SMBUS0 总线
 * @details 使用默认配置扫描 SMBUS0
 *
 * @return 扫描到的设备数量
 */
U32 testSmbusScanI2CBusDefault(void);
/* ======================================================================== */
/*                   测试辅助函数 API 接口                                   */
/* ======================================================================== */

/**
 * @brief 辅助函数：将SMBus设备切换到Slave模式并设置正确的地址
 * @param slaveIndex Slave设备索引 (0-3)
 * @param enableArp 是否启用ARP
 * @return 0 成功, 负值 失败
 *
 * @note 用于快速切换到Target模式进行测试
 * @note 支持动态启用/禁用ARP功能
 */
S32 smbusSwitchToSlaveMode(U8 slaveIndex, bool enableArp, U8 index);

/**
 * @brief 辅助函数：将SMBus设备切换到Master模式
 * @param masterIndex Master设备索引 (0-3)
 * @param speedMode 速度模式: 0=100kHz, 1=400kHz, 2=1MHz
 * @return 0 成功, 负值 失败
 *
 * @note 用于快速切换到Master模式进行测试
 * @note 默认使用7位地址模式
 */
S32 smbusSwitchToMasterMode(U8 masterIndex, U32 speedMode);

#endif /* __TEST_SMBUS_API_H_ */