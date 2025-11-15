/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_mpu.h
 * @author 
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */
 
#ifndef __BSP_MPU_H__
#define __BSP_MPU_H__

#ifdef __cplusplus
extern "C" {
#endif

//< MPU内存属性
#define MPU_SYSTEM_CONTROL_M    (0x00000001)
#define MPU_SYSTEM_CONTROL_C    (0x00000001 << 2)
#define MPU_SYSTEM_CONTROL_I    (0x00000001 << 12)
#define MPU_REGION_ENABLE       (0x00000001)

#define MPU_REGION_SIZE_32B         (0b00100 << 1)
#define MPU_REGION_SIZE_64B         (0b00101 << 1)
#define MPU_REGION_SIZE_128B        (0b00110 << 1)
#define MPU_REGION_SIZE_256B        (0b00111 << 1)
#define MPU_REGION_SIZE_512B        (0b01000 << 1)
#define MPU_REGION_SIZE_1K          (0b01001 << 1)
#define MPU_REGION_SIZE_2K          (0b01010 << 1)
#define MPU_REGION_SIZE_4K          (0b01011 << 1)
#define MPU_REGION_SIZE_8K          (0b01100 << 1)
#define MPU_REGION_SIZE_16K         (0b01101 << 1)
#define MPU_REGION_SIZE_32K         (0b01110 << 1)
#define MPU_REGION_SIZE_64K         (0b01111 << 1)
#define MPU_REGION_SIZE_128K        (0b10000 << 1)
#define MPU_REGION_SIZE_256K        (0b10001 << 1)
#define MPU_REGION_SIZE_512K        (0b10010 << 1)
#define MPU_REGION_SIZE_1M          (0b10011 << 1)
#define MPU_REGION_SIZE_2M          (0b10100 << 1)
#define MPU_REGION_SIZE_4M          (0b10101 << 1)
#define MPU_REGION_SIZE_8M          (0b10110 << 1)
#define MPU_REGION_SIZE_16M         (0b10111 << 1)
#define MPU_REGION_SIZE_32M         (0b11000 << 1)
#define MPU_REGION_SIZE_64M         (0b11001 << 1)
#define MPU_REGION_SIZE_128M        (0b11010 << 1)
#define MPU_REGION_SIZE_256M        (0b11011 << 1)
#define MPU_REGION_SIZE_512M        (0b11100 << 1)
#define MPU_REGION_SIZE_1G          (0b11101 << 1)
#define MPU_REGION_SIZE_2G          (0b11110 << 1)
#define MPU_REGION_SIZE_4G          (0b11111 << 1)

#define MPU_REGION_EXECUTE_NEVER    (0x1000)

#define MPU_REGION_AP_NO_ACCESS     (0x0000)
#define MPU_REGION_AP_FULL_ACCESS   (0x0300)
#define MPU_REGION_AP_READ_ONLY     (0x0600)

#define MPU_REGION_TYPE_STRONGLY_ORDERED_SHARED                 (0x00)
#define MPU_REGION_TYPE_DEVICE_NSHARED                          (0x10)
#define MPU_REGION_TYPE_DEVICE_SHARED                           (0x01)
#define MPU_REGION_TYPE_NORMAL_SHARED                           (0x04)
#define MPU_REGION_TYPE_NORMAL_NSHARED                          (0x00)
#define MPU_REGION_TYPE_NORMAL_WRITE_THROUGH_NO_WRITE_ALLOC     (0x02)
#define MPU_REGION_TYPE_NORMAL_WRITE_BACK_NO_WRITE_ALLOC        (0x03)
#define MPU_REGION_TYPE_NORMAL_WRITE_BACK_WRITE_ALLOC           (0x0b)
#define MPU_REGION_TYPE_NORMAL_NON_CACHEBLE                     (0x08)

/**
 * @brief dcache 使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheEnable(void);

/**
 * @brief icache 使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void icacheEnable(void);

/**
 * @brief dcache 去使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheDisable(void);

/**
 * @brief icache 去使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void icacheDisable(void);

/**
 * @brief icache 无效（全）
 * @param [in] none
 * @param [out] none
 * @return none
 */
void icacheAllInvalid(void);

/**
 * @brief dcache flush
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheAllFlush(void);

/**
 * @brief dcache 无效（全）
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheAllInvalid(void);

/**
 * @ brief 对指定地址进行dcache无效
 * @param [in] addr,待操作的地址
 * @param [in] size,待操作的大小
 * @param [out] none
 * @return none
 */
void dcacheInvalid(U32 addr, U32 size);

/**
 * @ brief 对指定地址进行dcache同步
 * @param [in] addr,待操作的地址
 * @param [in] size,待操作的大小
 * @param [out] none
 * @return none
 */
void dcacheClean(U32 addr, U32 size);

/**
 @ brief 对指定地址进行dcache刷新
 * @param [in] addr,待操作的地址
 * @param [in] size,待操作的大小
 * @param [out] none
 * @return none
 */
void dcacheFlush(U32 addr, U32 size);

/**
 * @brief 打开异常上报
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpsrAIFEnable(void);

/**
 * @brief 关闭异常上报
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpsrAIFDisable(void);

/**
 * @brief 打开中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuIrqEnable(void);

/**
 * @brief 关闭中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuIrqDisable(void);

/**
 * @brief 打开快速中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuFiqEnable(void);

/**
 * @brief 关闭快速中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuFiqDisable(void);

/**
 * @brief MPU默认配置
 * @param [in] none
 * @param [out] none
 * @return none
 */
void mpuInit(void);

/**
 * @brief MEM region 内存属性配置
 * @param [in] val,内存属性
 * @param [out] none
 * @return none
 */
void memAccessAttributeSet(U32 val);

/**
 * @brief MEM region 内存属性配置
 * @param [in] val,内存大小
 * @param [out] none
 * @return none
 */
void memRegionSizeSetAndEnable(U32 val);

/**
 * @brief MEM region 内存地址配置
 * @param [in] val,内存地址
 * @param [out] none
 * @return none
 */
void memRegionBaseSet(U32 addr);

/**
 * @brief MEM region 数配置
 * @param [in] val,region 数
 * @param [out] none
 * @return none
 */
void memRegionNumSet(U32 n);

/**
 * @brief MPU系统寄存器配置
 * @param [in] val,待配置的值
 * @param [out] none
 * @return none
 */
void mpuSystemCtrlRegSet(U32 val);

/**
 * @brief 获取MPU系统寄存器
 * @param [in] none
 * @param [out] none
 * @return MPU系统寄存器值
 */
U32  mpuSystemCtrlRegGet(void);

#ifdef __cplusplus
}
#endif

#endif