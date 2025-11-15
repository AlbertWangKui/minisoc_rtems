/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_api.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/05
 * @brief BSP API定义
 */

#ifndef __BSP_API_H__
#define __BSP_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtems/thread.h>
#include "bsp_config.h" ///< BSP配置头文件，包含全局配置和平台定义
#include "bsp_device.h" ///< 需要使用到各个bsp的设备枚举信息
#include "bsp_mpu.h"
#include "bsp_crg.h"
#include "bsp_drv_id.h"

/******** json api ********/
#ifdef CONFIG_CORE_JSON
#include "core_json.h"

#define JSON_VALUE_MAX_LEN         32 ///< json value的最大长度
#define JSON_KEY_MAX_LEN           32   ///< json key的最大长度
#define JSON_KEY_BIND_MAX_LEN      128  ///< json key bind的最大长度

typedef struct jsonValue {
    S8 jsonVal[JSON_VALUE_MAX_LEN];
} JsonVal_s;

/**
 * @brief 获取BSP的JSON字符串
 * @details 此函数用于获取BSP相关的JSON配置信息，返回JSON字符串的指针。
 * @return 指向JSON字符串的指针
 */
 S8* getBspJson(void);

 /**
  * @brief 获取BSP JSON的大小
  * @details 此函数返回BSP JSON字符串的大小
  * @return BSP JSON字符串的大小
  */
 U32 getBspJsonSize(void);

 /**
  * @brief 验证BSP JSON的有效性
  * @details 此函数验证BSP JSON字符串的格式是否正确
  * @return EXIT_SUCCESS 或 EXIT_FAILURE
  */
 S32 jsonValidate();

 /**
  * @brief 获取JSON值
  * @details 此函数根据给定的key获取JSON值
  * @param key JSON键
  * @param value 输出的JSON值
  * @return EXIT_SUCCESS 或 EXIT_FAILURE
  */
 S32 jsonGetValue(const char *key,JsonVal_s *value);

 /**
  * @brief 根据设备ID获取JSON值
  * @details 此函数根据设备ID、类别和键获取JSON值
  * @param devID 设备ID
  * @param pwd 设备所在的目录
  * @param key 键
  * @param value 输出的JSON值
  * @return EXIT_SUCCESS 或 EXIT_FAILURE
  * @note 如果pwd为NULL或空字符串，则只使用设备ID和键进行查找。
  *       这个函数仅仅是根据传入的devID来获得对应设备的设备名称，然后将他们以pwd.devName.key连接起来索引value。
  *       例1:假设，devName=uart0,索引顺序为perips.uart.uart0.baseAddr，则pwd==perips.uart，key==baseAddr。
  *       例2:假设，devName=pvt0,索引顺序为perips.pvt0.ch0.baseAddr,则pwd==perips，key==ch0.baseAddr。
  */
 S32 jsonGetValueByDevID(DevList_e devID,const char *pwd,const char *key,JsonVal_s *value);

 /**
  * @brief 获取JSON节点
  * @details 根据给定的key，从BSP的JSON配置中获取对应的节点内容和节点大小
  *          当json文件过大，而需要重复查找某个子节点时，可以先用这个函数找到子节点指针，
  *          然后用jsonGetValInNode函数获取值。
  * @param [in] key 需要查找的JSON键
  * @param [out] node 返回找到的JSON节点内容的指针
  * @param [out] nodeSize 返回节点内容的大小
  * @return EXIT_SUCCESS 表示成功，EXIT_FAILURE 表示失败
  */
 S32 jsonGetNode(char *key, char **node, U32 *nodeSize);

 /**
  * @brief 在指定的JSON节点中查找给定键的值。
  * @details 当json文件过大，而需要重复查找某个子节点时，可以先用jsonGetNode函数找到子节点指针，
  *          然后用jsonGetValInNode函数获取值。
  * @param node 指向包含JSON节点内容的缓冲区指针。
  * @param nodeSize 节点内容的字节大小。
  * @param key 需要查找的键名。
  * @param value 用于存放查找到的值的指针，结果通过该参数返回。
  * @return S32 返回操作结果，EXIT_SUCCESS表示成功，其他值表示失败。
  */
 S32 jsonGetValInNode(char *node, U32 nodeSize, const char *key, JsonVal_s *value);

 /**
 * @brief 通过名称获取设备ID
 * @details 此函数通过设备名称返回设备ID
 * @param [in] name 设备名称
 * @param [inout] devid 设备ID
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 getDevIDByName(char *name, U32 *devid);

/**
 * @brief 通过ID获取设备名称
 * @details 此函数通过设备ID返回设备名称
 * @param [in] devID 设备ID
 * @param [inout] name 设备名称
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 getDevName(DevList_e devID, const char *name);

#endif ///< CONFIG_CORE_JSON

/*
 * defines for devices
 */

typedef struct device {
    DevList_e devID;
    U16 drvID;
#ifdef CONFIG_CORE_JSON
    S8 name[JSON_KEY_MAX_LEN];
#endif
    void *driver;
    U32 stat; /* bit0 used for lock, bit1~bit31 for future use (expansion)*/
    rtems_recursive_mutex lock;
} Device_s;

#define DEVICE_CONSTRUCTOR(device, devName, driverID)    \
  {    \
    .devID = DEVICE_##device,    \
    .drvID = driverID,    \
    .driver = NULL,    \
    .lock = RTEMS_RECURSIVE_MUTEX_INITIALIZER("rMutex_" #devName),    \
    .stat = 0,    \
  }

typedef enum {
    DEV_LOCK_MANAGERED_BY_DRIVER = 0, ///< 设备锁由驱动程序控制
    DEV_LOCK_MANAGERED_BY_USER = 1,   ///< 设备锁由用户控制
    DEV_LOCK_MANAGERED_BY_NR
} DevLockManagedType_e;

/**
 * @brief 获取设备列表的大小
 * @details 此函数返回设备列表的大小
 * @return 设备列表的大小
 */
U32 getDevSize(void);

/**
 * @brief 通过ID获取设备
 * @details 此函数通过设备ID返回设备结构体
 * @param devID 设备ID
 * @return 指向设备结构体的指针，未找到返回NULL
 */
Device_s *getDevice(DevList_e devID);

/**
 * @brief 获取设备列表
 * @details 此函数返回设备列表
 * @return 指向设备列表的指针
 */
Device_s* getDevList(void);

/**
 * @brief 检查设备列表能否使能FAST_INDEXING
 * @details 如果sysDev[devID].devID == devID，则表示设备列表可以使用快速索引。
 * @return true 表示可以使用快速索引，false表示不可以
 */
bool isDevFastIdxCapable(void);

/**
 * @brief 通过ID获取设备驱动
 * @details 此函数通过设备ID返回设备驱动
 * @param [in] devID 设备ID
 * @param [inout] driver 设备驱动
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 getDevDriver(DevList_e devID, void **driver);

/**
 * @brief 检查设备驱动是否已初始化
 * @details 此函数检查设备驱动是否已初始化
 * @param [in] devID 设备ID
 * @return 已初始化返回true，否则返回false
 */
bool isDrvInit(DevList_e devID);

/**
 * @brief 安装设备驱动
 * @details 此函数安装设备驱动，设备锁、设备状态等会在此函数中初始化
 * @param [in] devID 设备ID
 * @param [inout] driver 设备驱动
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 drvInstall(DevList_e devID, void *driver);

/**
 * @brief 卸载设备驱动
 * @details 此函数卸载设备驱动，设备锁、设备状态等会在此函数中去初始化
 * @param [in] devID 设备ID
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 drvUninstall(DevList_e devID);

/**
 * @brief 获取设备锁
 * @details 此函数获取设备锁，为阻塞函数，会等待锁可用或超时
 * @param [in] devID 设备ID
 * @param [in] timeoutMS 超时时间（毫秒）
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 devLock(DevList_e devID, U32 timeoutMS);

/**
 * @brief 释放设备锁
 * @details 此函数释放设备锁
 * @param [in] devID 设备ID
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 devUnlock(DevList_e devID);

/**
 * @brief 设置设备锁模式
 * @details 此函数设置设备锁模式，可设置为用户管理或驱动管理
 * @param [in] devID 设备ID
 * @param [in] lockType 锁类型
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 */
S32 devLockModeSet(DevList_e devID,DevLockManagedType_e lockType);

/**
 * @brief 获取设备锁控制类型
 * @details 此函数获取设备锁的控制类型
 * @param [in] devID 设备ID
 * @return 设备锁控制类型
 */
DevLockManagedType_e devLockCtrlBy(DevList_e devID);

/**
 * @brief 用户管理设备锁
 * @details 此函数获取设备锁，等待指定时间
 * @param [in] devID 设备ID
 * @param [in] timeoutMS 超时时间（毫秒）
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 * @note 此函数仅由用户调用，驱动不可调用
 */
static inline S32 devLockByUser(DevList_e devID, U32 timeoutMS)
{
    if (devLockCtrlBy(devID) != DEV_LOCK_MANAGERED_BY_USER) {
        return EXIT_SUCCESS; ///< 如果设备锁由驱动管理，则直接返回成功
    } else {
        return devLock(devID, timeoutMS); ///< 用户管理锁，等待指定时间
    }
}

/**
 * @brief 驱动管理设备锁
 * @details 此函数获取设备锁，等待指定时间
 * @param [in] devID 设备ID
 * @param [in] timeoutMS 超时时间（毫秒）
 * @return EXIT_SUCCESS 或 EXIT_FAILURE
 * @note 此函数仅驱动调用，用户不可调用
 */
static inline S32 devLockByDriver(DevList_e devID, U32 timeoutMS)
{
    if (devLockCtrlBy(devID) != DEV_LOCK_MANAGERED_BY_DRIVER) {
        return EXIT_SUCCESS; ///< 如果设备锁由用户管理，则直接返回成功
    } else {
        return devLock(devID, timeoutMS); ///< 驱动管理锁，等待指定时间
    }
}

static inline S32 devUnlockByUser(DevList_e devID)
{
    if (devLockCtrlBy(devID) != DEV_LOCK_MANAGERED_BY_USER) {
        return EXIT_SUCCESS; ///< 如果设备锁由驱动管理，则直接返回成功
    } else {
        return devUnlock(devID); ///< 用户管理锁，直接释放
    }
}

static inline S32 devUnlockByDriver(DevList_e devID)
{
    if (devLockCtrlBy(devID) != DEV_LOCK_MANAGERED_BY_DRIVER) {
        return EXIT_SUCCESS; ///< 如果设备锁由用户管理，则直接返回成功
    } else {
        return devUnlock(devID); ///< 驱动管理锁，直接释放
    }
}

bool isDrvMatch(DevList_e devID, U16 drvID);

/**
 * @brief 运行检查辅助函数
 * @details 此函数封装了设备锁定、驱动匹配和驱动获取操作，提供统一的设备访问前检查
 * @param [in] devID 设备ID
 * @param [in] drvID 驱动ID
 * @param [out] driver 返回的驱动指针
 * @return EXIT_SUCCESS 操作成功
 * @return -EINVAL 参数无效或设备不存在
 * @return -EBUSY 设备锁定超时
 * @return -ENODEV 驱动不匹配或未初始化
 */
S32 funcRunBeginHelper(DevList_e devID, U16 drvID, void **driver);

/**
 * @brief 解锁辅助函数
 * @details 此函数封装了设备解锁操作，提供统一的设备访问后清理
 * @param [in] devID 设备ID
 * @return 无返回值
 */
void funcRunEndHelper(DevList_e devID);

#ifdef __cplusplus
}
#endif

#endif
