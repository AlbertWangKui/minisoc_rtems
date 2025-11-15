/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_api.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/05
 * @brief BSP API实现
 */

#include <bsp/linker-symbols.h>
#include <time.h>
#include <rtems/score/todimpl.h>
#include "common_defines.h"
#include "log_msg.h"
#include "bsp_api.h"

#ifdef CONFIG_CORE_JSON

#ifdef CONFIG_JSON_IN_SRAM
LINKER_SYMBOL(bsp_section_confscript_data_begin)
LINKER_SYMBOL(bsp_section_confscript_data_end)

/**
 * @brief Get BSP JSON configuration string
 * @details This function returns a pointer to the BSP JSON configuration
 *          data stored in the dedicated memory section.
 * @return Pointer to BSP JSON configuration string
 * @note The returned pointer points to read-only memory section
 */
S8 *getBspJson(void)
{
    return (S8 *)bsp_section_confscript_data_begin;
}

/**
 * @brief Get BSP JSON configuration size
 * @details This function calculates and returns the size of the BSP JSON
 *          configuration data in bytes.
 * @return Size of BSP JSON configuration in bytes
 * @return 0 if configuration section is invalid or empty
 */
U32 getBspJsonSize(void)
{
    U32 end_addr = (U32)bsp_section_confscript_data_end;
    U32 addr_input = (U32)bsp_section_confscript_data_begin;
    U32 size = 0;

    if (end_addr >= addr_input) {
        size = end_addr - addr_input;
    }
    return size;
}
#elif defined(CONFIG_JSON_IN_FLASH)
S8 *getBspJson(void)
{
    ///< todo: json file in flash.
    return NULL;
}

U32 getBspJsonSize(void)
{
    ///< todo: json file in flash.
    return 0;
}
#endif ///< CONFIG_JSON_IN_SRAM

/**
 * @brief Validate BSP JSON configuration
 * @details This function validates the syntax and structure of the BSP
 *          JSON configuration data using the core JSON parser.
 * @return EXIT_SUCCESS if JSON is valid
 * @return -EINVAL if JSON buffer is NULL or empty
 * @return -EXIT_FAILURE if JSON validation fails
 */
S32 jsonValidate()
{
    char *buf = getBspJson();
    uint32_t bufSize = getBspJsonSize();
    JSONStatus_t result;

    if (buf == NULL || bufSize == 0) {
        return -EINVAL;
    }

    result = JSON_Validate(buf, bufSize);
    if (result != JSONSuccess) {
        return -EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/**
 * @brief Get JSON value by key
 * @details This function searches for a specific key in the BSP JSON
 *          configuration and returns the corresponding value.
 * @param [in] key JSON key to search for
 * @param [out] value Pointer to store the found JSON value
 * @return EXIT_SUCCESS if value is found and retrieved
 * @return -EINVAL if any parameter is NULL or JSON buffer is empty
 * @return -EXIT_FAILURE if key is not found in JSON
 * @return -ENOMEM if value length exceeds JSON_VALUE_MAX_LEN-1
 */
S32 jsonGetValue(const S8 *key, JsonVal_s *value)
{
    S8 *jsonBuf = getBspJson();
    U32 fileSize = getBspJsonSize();
    JSONStatus_t result;
    size_t valueLength;
    char *valuePtr;

    if (jsonBuf == NULL|| key == NULL || value == NULL || fileSize == 0) {
        return -EINVAL;
    }

    /* clear buf */
    memset(value->jsonVal,0,sizeof(value->jsonVal));

    result = JSON_Search(jsonBuf, fileSize, key, strlen(key), &valuePtr, &valueLength);
    if (result != JSONSuccess) {
        return -EXIT_FAILURE;
    }

    if (valueLength > (JSON_VALUE_MAX_LEN-1)) {
        return -ENOMEM;
    }

    memcpy(value->jsonVal,valuePtr,valueLength);

    return EXIT_SUCCESS;
}

/**
 * @brief Get JSON value by device ID
 * @details This function retrieves a JSON value for a specific device by constructing
 *          a search key from device ID, optional path, and target key.
 * @param [in] devID Device identifier
 * @param [in] pwd Optional path prefix for hierarchical JSON structure
 * @param [in] key Target JSON key to search for
 * @param [out] value Pointer to store the found JSON value
 * @return EXIT_SUCCESS if value is found and retrieved
 * @return -EXIT_FAILURE if any parameter is NULL or invalid
 * @return -ENODEV if device ID is not found in device list
 * @return -EINVAL if key is empty or parameters are invalid
 * @return -ENOMEM if constructed key exceeds JSON_KEY_BIND_MAX_LEN
 * @note If pwd is NULL or empty, search format is "deviceName.key"
 *       If pwd is provided, search format is "pwd.deviceName.key"
 */
S32 jsonGetValueByDevID(DevList_e devID, const char *pwd, const char *key, JsonVal_s *value)
{
    char searchBind[JSON_KEY_BIND_MAX_LEN] = {0};
    S32 ret;
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
    U32 keyLen = 0;
#ifndef DEVICE_FAST_INDEXING
    U32 i;
#endif

    if (key == NULL || value == NULL || devList == NULL || devID >= devSize) {
        return -EXIT_FAILURE;
    }

    /* clear buf */
    memset(value->jsonVal, 0, sizeof(value->jsonVal));

#ifdef DEVICE_FAST_INDEXING
    if (devList[devID].devID != devID) {
        return -ENODEV; ///< device not found
    }
    if (pwd == NULL || strlen(pwd) == 0) {
        keyLen = snprintf(searchBind, JSON_KEY_BIND_MAX_LEN, "%s.%s", devList[devID].name, key);
    } else {
        keyLen = snprintf(searchBind, JSON_KEY_BIND_MAX_LEN, "%s.%s.%s", pwd, devList[devID].name, key);
    }
#else
    for (i = 0; i < devSize; i++) {
        if (devList[i].devID == devID) {
            break;
        }
    }
    if (i == devSize) {
        return -ENODEV;
    }
    if (pwd == NULL || strlen(pwd) == 0) {
        keyLen = snprintf(searchBind, JSON_KEY_BIND_MAX_LEN, "%s.%s", devList[i].name, key);
    } else {
        keyLen = snprintf(searchBind, JSON_KEY_BIND_MAX_LEN, "%s.%s.%s", pwd, devList[i].name, key);
    }
#endif

    if (keyLen < 0 || keyLen >= JSON_KEY_BIND_MAX_LEN) {
        return -ENOMEM; ///< key too long
    }
    if (keyLen == 0) {
        return -EINVAL; ///< key is empty
    }

    ret = jsonGetValue(searchBind, value);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Get JSON node by key
 * @details This function retrieves a JSON node (sub-object) by key, returning
 *          a pointer to the node content and its size for efficient parsing.
 * @param [in] key JSON key to search for
 * @param [out] node Pointer to store the found JSON node pointer
 * @param [out] nodeSize Pointer to store the node size in bytes
 * @return EXIT_SUCCESS if node is found and retrieved
 * @return -EINVAL if any parameter is NULL
 * @return -EXIT_FAILURE if key is not found in JSON
 * @note The returned node pointer points within the original JSON buffer
 */
S32 jsonGetNode(char *key, char **node, U32 *nodeSize)
{
    char *buf = getBspJson();
    U32 bufSize = getBspJsonSize();
    JSONStatus_t result;
    size_t nodeLength;
    char *nodePtr;

    if(key == NULL || node == NULL || nodeSize == NULL) {
        return -EINVAL;
    }

    result = JSON_Search(buf, bufSize, key, strlen(key), &nodePtr, &nodeLength);
    if(result != JSONSuccess) {
        return -EXIT_FAILURE;
    }

    *node = nodePtr;
    *nodeSize = nodeLength;

    return EXIT_SUCCESS;
}

/**
 * @brief Get JSON value within a specific node
 * @details This function searches for a key within a previously retrieved JSON node,
 *          providing efficient repeated searches within the same sub-object.
 * @param [in] node Pointer to JSON node content
 * @param [in] nodeSize Size of the JSON node in bytes
 * @param [in] key JSON key to search for within the node
 * @param [out] value Pointer to store the found JSON value
 * @return EXIT_SUCCESS if value is found and retrieved
 * @return -EINVAL if any parameter is NULL or nodeSize is 0
 * @return -EXIT_FAILURE if key is not found in the node
 * @return -ENOMEM if value length exceeds JSON_VALUE_MAX_LEN-1
 */
S32 jsonGetValInNode(char *node, U32 nodeSize, const char *key, JsonVal_s *value)
{
    JSONStatus_t result;
    size_t valueLength;
    char *valuePtr;

    if(node == NULL || nodeSize == 0 || key == NULL || value == NULL) {
        return -EINVAL;
    }
    /* clear buf */
    memset(value->jsonVal,0,sizeof(value->jsonVal));

    result = JSON_Search(node, nodeSize, key, strlen(key), &valuePtr, &valueLength);
    if(result != JSONSuccess) {
        return -EXIT_FAILURE;
    }

    if(valueLength > (JSON_VALUE_MAX_LEN-1)) {
        return -ENOMEM;
    }

    memcpy(value->jsonVal,valuePtr,valueLength);

    return EXIT_SUCCESS;
}

/**
 * @brief Get device ID by name
 * @details This function searches the device list for a device with the
 *          specified name and returns its device ID.
 * @param [in] name Device name to search for
 * @param [out] devID Pointer to store the found device ID
 * @return EXIT_SUCCESS if device is found and ID is returned
 * @return -EINVAL if any parameter is NULL
 * @return -ENODEV if device name is not found in device list
 */
S32 getDevIDByName(S8 *name,U32 *devID)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
    U32 i;

    if (name == NULL || devID == NULL || devList == NULL) {
        return -EINVAL;
    }

    for (i=0;i<devSize;i++) {
        if (strcmp(devList[i].name,name) == 0) {
#ifdef DEVICE_FAST_INDEXING
            if (devList[i].devID != i) {
                return -ENODEV; ///< device not found
            }
#endif
            *devID = devList[i].devID;
            return EXIT_SUCCESS;
        }
    }
    return -ENODEV;
}

/**
 * @brief Get device name by ID
 * @details This function retrieves the name of a device given its device ID.
 * @param [in] devID Device identifier
 * @param [out] name Pointer to buffer to store the device name
 * @return EXIT_SUCCESS if device name is retrieved
 * @return -EINVAL if any parameter is NULL or devID is invalid
 * @return -ENODEV if device ID is not found in device list
 * @warning Caller must ensure that name buffer is large enough to hold the device name
 */
S32 getDevName(DevList_e devID, const char *name)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
#ifndef DEVICE_FAST_INDEXING
    U32 i;
#endif

    if (name == NULL || devList == NULL || devID >= devSize) {
        return -EINVAL;
    }

#ifdef DEVICE_FAST_INDEXING
    if (devList[devID].devID != devID) {
        return -ENODEV; ///< device not found
    }
    strcpy((char *)name,devList[devID].name);
    return EXIT_SUCCESS;
#else
    for (i = 0; i < devSize; i++) {
        if (devList[i].devID == devID) {
            strcpy((char *)name, devList[i].name);///< todo:overflow
            return EXIT_SUCCESS;
        }
    }
#endif

    return -EXIT_FAILURE;
}

#endif ///< CONFIG_CORE_JSON

/**
 * @brief Get device structure by ID
 * @details This function retrieves the device structure for a given device ID
 *          from the device list using either fast indexing or linear search.
 * @param [in] devID Device identifier
 * @return Pointer to device structure if found
 * @return NULL if device is not found or parameters are invalid
 * @note Uses fast indexing if DEVICE_FAST_INDEXING is enabled
 */
Device_s *getDevice(DevList_e devID)
{
    Device_s *dev = NULL;
    Device_s *devList = NULL;
    devList = getDevList();
    U32 devSize = getDevSize();

    if (devList == NULL || devSize == 0 || devID >= DEVICE_NR) {
        return NULL; ///< device not found
    }
#ifndef DEVICE_FAST_INDEXING
   U32 loop = 0;
#endif

#ifdef DEVICE_FAST_INDEXING
    if (devList[devID].devID != devID) {
        return NULL; ///< device not found
    }
    dev = &devList[devID];
    return dev;
#else
    for (loop = 0; loop < devSize; loop++) {
        if (devList[loop].devID == devID) {
            dev = &devList[loop];
            break;
        }
    }
    return dev;
#endif
}

/**
 * @brief Check if device list supports fast indexing
 * @details This function verifies if the device list can use fast indexing
 *          by checking if devList[i].devID == i for all devices.
 * @return true if fast indexing is supported
 * @return false if fast indexing is not supported or list is not initialized
 */
bool isDevFastIdxCapable(void)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
    U32 i;

    if (devList == NULL || devSize == 0) {
        return false; ///< device list is not initialized
    }

    for (i = 0; i < devSize; i++) {
        if (devList[i].devID != i) {
            return false; ///< device list cannot use fast indexing
        }
    }
    return true; ///< device list can use fast indexing
}

/**
 * @brief Get device driver by device ID
 * @details This function retrieves the driver instance for a given device ID.
 * @param [in] devID Device identifier
 * @param [out] driver Pointer to store the driver instance
 * @return EXIT_SUCCESS if driver is retrieved
 * @return -EINVAL if any parameter is NULL or invalid
 * @return -ENODEV if device ID is not found
 * @return -EXIT_FAILURE if device is not found in linear search mode
 */
S32 getDevDriver(DevList_e devID, void **driver)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
#ifndef DEVICE_FAST_INDEXING
    U32 i;
#endif

    if (driver == NULL || devList == NULL || devSize == 0 || devID >= DEVICE_NR) {
        return -EINVAL;
    }

#ifdef DEVICE_FAST_INDEXING
    if (devList[devID].devID != devID) {
        return -ENODEV; ///< device not found
    }
    *driver = devList[devID].driver;
    return EXIT_SUCCESS;
#else
    for (i = 0; i < devSize; i++) {
        if (devList[i].devID == devID) {
            *driver = devList[i].driver;
            return EXIT_SUCCESS;
        }
    }
#endif
    return -EXIT_FAILURE;
}

/**
 * @brief Check if device driver is initialized
 * @details This function checks if a driver is installed for the given device ID.
 * @param [in] devID Device identifier
 * @return true if driver is initialized
 * @return false if driver is not initialized or device is not found
 */
bool isDrvInit(DevList_e devID)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
#ifndef DEVICE_FAST_INDEXING
    U32 i;
#endif

    if (devList == NULL || devSize == 0 || devID >= DEVICE_NR) {
        return false; ///< device list is not initialized
    }

#ifdef DEVICE_FAST_INDEXING

    if (devList[devID].devID != devID) {
        return false; ///< device not found
    }
    return devList[devID].driver != NULL;
#else
    for (i = 0; i < devSize; i++) {
        if (devList[i].devID == devID) {
            return devList[i].driver != NULL;
        }
    }
    return false;
#endif
}

/**
 * @brief Install device driver
 * @details This function installs a driver for a specific device ID and sets
 *          the device lock mode to driver-managed.
 * @param [in] devID Device identifier
 * @param [in] driver Pointer to driver instance
 * @return EXIT_SUCCESS if driver is installed
 * @return -ENODEV if device ID is not found
 * @return -EEXIST if driver is already installed
 * @return -EXIT_FAILURE if device is not found in linear search mode
 * @note Memory management: The framework takes ownership of the driver pointer
 */
S32 drvInstall(DevList_e devID, void *driver)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
#ifndef DEVICE_FAST_INDEXING
    U32 i;
#endif

    if (devList == NULL || devSize == 0 || devID >= DEVICE_NR) {
        return -ENODEV; ///< device not found
    }

#ifdef DEVICE_FAST_INDEXING
    if (devList[devID].devID != devID) {
        return -ENODEV; ///< device not found
    }
    if (devList[devID].driver != NULL) {
        return -EEXIST; ///< driver already installed
    }
    devList[devID].driver = driver;
    return EXIT_SUCCESS;
#else
    for (i = 0; i < devSize; i++) {
        if (devList[i].devID == devID) {
            if (devList[i].driver != NULL) {
                return -EEXIST;
            }
            devList[i].driver = driver;
            return EXIT_SUCCESS;
        }
    }
    return -EXIT_FAILURE;
#endif
}

/**
 * @brief Uninstall device driver
 * @details This function uninstalls a driver for a specific device ID and
 *          frees the allocated driver memory.
 * @param [in] devID Device identifier
 * @return EXIT_SUCCESS if driver is uninstalled
 * @return -ENODEV if device ID is not found or driver not installed
 * @return -EXIT_FAILURE if device is not found in linear search mode
 * @note Memory management: This function frees the driver memory
 */
S32 drvUninstall(DevList_e devID)
{
    Device_s *devList = getDevList();
    U32 devSize = getDevSize();
#ifndef DEVICE_FAST_INDEXING
    U32 i;
#endif

    if (devList == NULL || devSize == 0 || devID >= DEVICE_NR) {
        return -ENODEV; ///< device not found
    }

#ifdef DEVICE_FAST_INDEXING
    if (devList[devID].devID != devID) {
        return -ENODEV; ///< device not found
    }
    if (devList[devID].driver == NULL) {
        return -ENODEV; ///< driver not installed
    }
    free(devList[devID].driver);
    devList[devID].driver = NULL;
    devList[devID].stat = 0;
    return EXIT_SUCCESS;
#else
    for (i = 0; i < devSize; i++) {
        if (devList[i].devID == devID) {
            if (devList[i].driver != NULL) {
                free(devList[i].driver);
                devList[i].driver = NULL;
                devList[i].stat = 0;
            }
            return EXIT_SUCCESS;
        }
    }
    return -ENODEV;
#endif
}

/**
 * @brief Acquire device lock with timeout
 * @details This function acquires a recursive mutex lock for the specified device
 *          with a timeout period.
 * @param [in] devID Device identifier
 * @param [in] timeoutMS Timeout in milliseconds (0 for non-blocking)
 * @return EXIT_SUCCESS if lock is acquired
 * @return -ENODEV if device is not found
 * @return -EBUSY if timeout occurs while waiting for lock
 * @return -EXIT_FAILURE if lock acquisition fails for other reasons
 */
S32 devLock(DevList_e devID, U32 timeoutMS)
{
    Device_s *dev = getDevice(devID);
    struct timespec ts;

    _TOD_Get(&ts);

    ts.tv_sec += timeoutMS / 1000;
    ts.tv_nsec += (timeoutMS % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
        ++ts.tv_sec;
        ts.tv_nsec -= 1000000000;
    }

    if (dev == NULL) {
        return -ENODEV; ///<  device not found
    }

    switch (_Mutex_recursive_Acquire_timed(&dev->lock, &ts)) {
        case 0:
            return EXIT_SUCCESS;
        case -ETIMEDOUT:
            return -EBUSY;
        default:
            return -EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Release device lock
 * @details This function releases the recursive mutex lock for the specified device.
 * @param [in] devID Device identifier
 * @return EXIT_SUCCESS if lock is released
 * @return -ENODEV if device is not found
 */
S32 devUnlock(DevList_e devID)
{
    Device_s *dev = getDevice(devID);

    if (dev == NULL) {
        return -ENODEV;
    }

    _Mutex_recursive_Release(&dev->lock);

    return EXIT_SUCCESS;
}

/**
 * @brief Set device lock management mode
 * @details This function configures whether the device lock is managed by user
 *          or by the driver framework.
 * @param [in] devID Device identifier
 * @param [in] lockType Lock management type (user or driver)
 * @return EXIT_SUCCESS if lock mode is set
 * @return -ENODEV if device is not found
 * @return -EINVAL if lock type is invalid
 * @return -EBUSY if device lock is currently held
 */
S32 devLockModeSet(DevList_e devID, DevLockManagedType_e lockType)
{
    Device_s *dev = getDevice(devID);

    if (dev == NULL) {
        return -ENODEV;
    }

    if (lockType >= DEV_LOCK_MANAGERED_BY_NR) {
        return -EINVAL;
    }

    if (devLock(devID, 1000) != EXIT_SUCCESS) {
        return -EBUSY; ///< mode can not be changed when lock is taken
    }

    if (lockType == DEV_LOCK_MANAGERED_BY_USER) {
        SET_BIT(dev->stat, 0);
    } else if (lockType == DEV_LOCK_MANAGERED_BY_DRIVER) {
        CLR_BIT(dev->stat, 0);
    } else {
        /* nothing to do*/
    }

    devUnlock(devID);

    return EXIT_SUCCESS;
}

/**
 * @brief Get device lock management type
 * @details This function returns the current lock management type for the device.
 * @param [in] devID Device identifier
 * @return DEV_LOCK_MANAGERED_BY_DRIVER if driver manages the lock
 * @return DEV_LOCK_MANAGERED_BY_USER if user manages the lock
 * @return DEV_LOCK_MANAGERED_BY_NR if device is not found
 */
DevLockManagedType_e devLockCtrlBy(DevList_e devID)
{
    Device_s *dev = getDevice(devID);

    if (dev == NULL) {
        return DEV_LOCK_MANAGERED_BY_NR;
    }

    if (BIT_IS_ONE(dev->stat, 0)) {
        return DEV_LOCK_MANAGERED_BY_USER;
    } else {
        return DEV_LOCK_MANAGERED_BY_DRIVER;
    }
}

/**
 * @brief Check if driver matches device
 * @details This function verifies if the specified driver ID matches the driver
 *          installed for the given device.
 * @param [in] devID Device identifier
 * @param [in] drvID Driver identifier to match
 * @return true if driver ID matches
 * @return false if driver ID does not match or device is not found
 */
bool isDrvMatch(DevList_e devID, U16 drvID)
{
    Device_s *dev = getDevice(devID);

    if (dev == NULL) {
        return false;
    }

    return dev->drvID == drvID;
}

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
S32 funcRunBeginHelper(DevList_e devID, U16 drvID, void **driver)
{
    S32 ret;

    ///< 参数验证
    if (driver == NULL) {
        return -EINVAL;
    }

    ///< 设备锁定，超时1000ms
    ret = devLockByDriver(devID, 1000);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< 检查驱动是否匹配
    if (!isDrvMatch(devID, drvID)) {
        ret = -ENODEV;
        goto cleanup;
    }

    ///< 获取驱动指针
    ret = getDevDriver(devID, driver);
    if (ret != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto cleanup;
    }

    ///< 检查driver是否已初始化
    if (*driver == NULL) {
        ret = -ENODEV;
        goto cleanup;
    }

    return EXIT_SUCCESS;

cleanup:
    ///< 清理：释放设备锁
    devUnlockByDriver(devID);
    return ret;
}

/**
 * @brief 解锁辅助函数
 * @details 此函数封装了设备解锁操作，提供统一的设备访问后清理
 * @param [in] devID 设备ID
 * @return 无返回值
 */
void funcRunEndHelper(DevList_e devID)
{
    if (devUnlockByDriver(devID) != EXIT_SUCCESS) {
        LOGE("funcRunEndHelper: devUnlockByDriver failed for devID %d\n", devID);
    }
}
