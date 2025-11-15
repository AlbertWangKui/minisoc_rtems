/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 * @file drv_spi_device.h
 * @brief
 * @author tianye
 * @version
 * @date 2024-05-22
 */

#ifndef __DRV_SPI_DEVICE_H__
#define __DRV_SPI_DEVICE_H__

#include <stdbool.h>
#include <stdint.h>
#include "common_defines.h"
#include "bsp_device.h"

/*! \enum erase_type
 *
 *  擦除粒度
 */
enum erase_type {
    ERASE_NONE,
    ERASE_SECTOR,       ///< 4KByte
    ERASE_BLOCK,        ///< 64KByte
    ERASE_CHIP,         ///< all chip
    ERASE_BUTT,
};

struct channel_info {
    uint32_t channel_id;
};

#define SPI_NOR_MAX_ID_LEN      (6)

/*
 * 注意：正常场景芯片所接入的设备都需要在驱动的兼容列表。
 * 如果连接非兼容列表的device，驱动会使用比较通用的单线
 * 读/写命令访问设备，尽量支持设备访问。但该场景只适用于
 * 研发测试场景。
 * 若probe不在兼容列表的device：
 * type返回DEV_NONE
 * size返回0xFFFFFFFF
 */
/*! \enum dev_type
 *
 *  spi device type
 */
enum dev_type {
    DEV_NONE,
    DEV_FLASH,
    DEV_NVSRAM,
    DEV_BUTT,
};
struct spi_dev_info {
    uint8_t id[SPI_NOR_MAX_ID_LEN];
    uint32_t size;
    enum dev_type type;
};

struct spi_dev_config {
    bool support_wr_protect;    ///< 当前通道是否支持写保护，默认不支持
};

/**
 * @brief dev probe
 *
 * @Param chan_info
 * @Param dev_info
 *
 * @Returns  errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 * -EEXIST      设备已经probe
 *
 * @Notice 没有加锁，不能与remove/config/读/写擦等接口并发
 */
int32_t spi_dev_probe(struct channel_info *chan_info, struct spi_dev_info *dev_info);

/**
 * @brief dev config
 *
 * @Param chan_info
 * @Param dev_config
 *
 * @Returns  errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 *
 * @Notice 1 没有加锁，不能与probe/remove/读/写擦等接口并发
 *         2 probe成功后，读/写擦之前调用
 */
int32_t spi_dev_config(struct channel_info *chan_info, struct spi_dev_config *dev_config);

/**
 * @brief wait dev idle
 *
 * @Param chan_info
 * @Param seconds
 *
 * @Returns  errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 *
 * @Notice 1 有设备mutex锁，可以和读、写、擦互斥；
 *         2 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t spi_dev_wait_idle(struct channel_info *chan_info, uint32_t seconds);

/**
 * @brief wait dev remove
 *
 * @Param chan_info
 *
 * @Returns  errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 *
 * @Notice 没有加锁，不能与probe/config/读/写擦等接口并发
 */
int32_t spi_dev_remove(struct channel_info *chan_info);

/**
 * @brief flash read
 *
 * @Param chan_info
 * @Param buffer
 * @Param len
 * @Param base_addr
 *
 * @Returns errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 *
 * @Notice 1 有设备mutex锁，可以和读、写、擦互斥；
 *         2 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t flash_read(struct channel_info *chan_info, uint8_t *buffer, uint32_t len, uint32_t base_addr);

/**
 * @brief flash write
 *
 * @Param chan_info
 * @Param buffer
 * @Param len
 * @Param base_addr
 *
 * @Returns errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 *
 * @Notice 1 有设备mutex锁，可以和读、写、擦互斥；
 *         2 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t flash_write(struct channel_info *chan_info, uint8_t *buffer, uint32_t len, uint32_t base_addr);

/**
 * @brief flash erase by type
 *
 * @Param chan_info
 * @Param etype
 * @Param base_addr
 *
 * @Returns  errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 *
 * @Notice 1 有设备mutex锁，可以和读、写、擦互斥；
 *         2 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t flash_erase_by_type(struct channel_info *chan_info, enum erase_type etype, uint32_t base_addr);

/**
 * @brief flash erase by range
 *
 * @Param chan_info      通道信息
 * @Param offset         起始偏移地址
 * @Param len            擦除长度
 * @Param preserve_data  是否保护范围外数据：true-保护范围外数据，false-sector对齐擦除
 *
 * @Returns  errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 * -ENOMEM      内存不足（仅在preserve_data=true时）
 *
 * @Notice 1 当preserve_data=false时，函数执行sector对齐擦除，实际擦除范围可能大于请求范围
 *         2 当preserve_data=true时，函数会保护范围外数据，但需要额外的读写操作和内存
 *         3 有设备mutex锁，可以和读、写、擦互斥；
 *         4 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t flash_erase(struct channel_info *chan_info, uint32_t offset, uint32_t len, bool preserve_data);

/**
 * @brief nvsram read
 *
 * @Param chan_info
 * @Param buffer
 * @Param len
 * @Param base_addr
 *
 * @Returns errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 *
 * @Notice 1 有设备mutex锁，可以和读、写、擦互斥；
 *         2 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t nvsram_read(struct channel_info *chan_info, uint8_t *buffer, uint32_t len, uint32_t base_addr);

/**
 * @brief nvsram write
 *
 * @Param chan_info
 * @Param buffer
 * @Param len
 * @Param base_addr
 *
 * @Returns errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 *
 * @Notice 1 有设备mutex锁，可以和读、写、擦互斥；
 *         2 有通道mutex锁，可以和qspi不同通道互斥；
 */
int32_t nvsram_write(struct channel_info *chan_info, uint8_t *buffer, uint32_t len, uint32_t base_addr);

/**
 * @brief nvsram write try
 *
 * @Param chan_info
 * @Param buffer
 * @Param len
 * @Param base_addr
 *
 * @Returns errno.h定义的错误码
 * 0            成功
 * -EINVAL      无效参数
 * -ENODEV      无效通道（channel无效或者未probe）
 * -ETIMEDOUT   与设备通信超时或者DEVICE一直繁忙
 * -EAGAIN      获取设备锁或者控制器锁失败
 * @Notice 1 try设备mutex锁，可以和读、写、擦互斥；
 *         2 try通道mutex锁，可以和qspi不同通道互斥；
 *         3 trylock失败返回 -EAGAIN
 */
int32_t nvsram_write_try(struct channel_info *chan_info, uint8_t *buffer, uint32_t len, uint32_t base_addr);

///< 接了译码芯片有16个通道（0~15），否则只有4个通道（0~3）
#define CQSPI_MAX_CHIPSELECT                (16)
#define CQSPI_MAX_CHIPSELECT_NO_DECODED     (4)

struct spi_dev_pdata {
    uint32_t  clk_rate;     ///< 目标时钟频率
    uint32_t  read_delay;   ///< 读延时，单位为read_delay * qspi ref
    uint32_t  tshsl_ns;     ///< 参考devcie手册
    uint32_t  tsd2d_ns;     ///< 参考devcie手册
    uint32_t  tchsh_ns;     ///< 参考devcie手册
    uint32_t  tslch_ns;     ///< 参考devcie手册
};

#define INVALID_IO_WIRE_CNT                  (0xFF)
struct cqspi_ctr_usr_conf {
    uint8_t io_wire_cnt;                                ///< io线数 若要使用驱动默认值，则置INVALID_IO_WIRE_CNT
    bool is_decoded_cs;                                 ///< 是否连接译码器
    bool use_direct_mode;                               ///< 是否使用直接模式
    bool rd_no_irq;                                     ///< read是不使用中断
    bool wr_no_irq;                                     ///< write是不使用中断
    bool user_dev_conf;                                 ///< 如果置true，表示需要使用用户的配置，则驱动会解析结构体 spi_dev_pdata
    struct spi_dev_pdata pdata[CQSPI_MAX_CHIPSELECT_NO_DECODED];   ///< 用户设备参数配置
};

/**
 * @brief 修改默认cqspi控制器配置
 * 默认是4线、不接4-16译码芯片、不使用直接模式
 * 如果设置需要在调用flash和nvsram接口前配置
 *
 * @Param usr_conf
 *
 * @Returns
 * 0            成功
 * -EINVAL      无效参数
 *
 */
int32_t cqspi_controller_config(struct cqspi_ctr_usr_conf *usr_conf);

#define CQSPI_REG_CNT       (39)
#define SPI_DEV_REG_CNT     (1)
#endif
