/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    spi_private.h
 * @author  pengqianheng
 * @date    2020.09.11
 * @brief   QSPI Flash Controller Module
 * @note    基于Cadence的QSPI控制器IP
 */

#ifndef __QSPI_PRIVATE_H__
#define __QSPI_PRIVATE_H__

///< NOTE: To enable debugging output, delete the next line and uncomment the line after.
#define QSPI_DEBUG                      2
#if(1 == QSPI_DEBUG)
#define debugprintf_err(fmt, args...)   (void)printk("[QSPI ERR %s:%d]: " fmt, __FUNCTION__, __LINE__, ##args)
#define debugprintf_info(fmt, args...)  (void)printk("[QSPI INFO %s:%d]: " fmt, __FUNCTION__, __LINE__, ##args)
#elif(2 == QSPI_DEBUG)
#define debugprintf_err(fmt, args...)   (void)printk("[QSPI ERR %s:%d]: " fmt, __FUNCTION__, __LINE__, ##args)
#define debugprintf_info(...)
#else
#define debugprintf_err(...)
#define debugprintf_info(...)
#endif

///< Definitions of status codes returned by the QSPI Controller.
typedef int32_t QSPI_STATUS_CODE;
#define QSPI_E_SUCCESS              (0)     ///< The operation was successful.
#define QSPI_E_ERROR                (-1)    ///< The operation failed.
#define QSPI_E_FPGA_CFG             (-2)    ///< FPGA configuration error detected
#define QSPI_E_FPGA_CRC             (-3)    ///< FPGA CRC error detected.
#define QSPI_E_FPGA_CFG_STM         (-4)    ///< An error occurred on the FPGA configuration bitstream input source.
#define QSPI_E_FPGA_PWR_OFF         (-5)    ///< The FPGA is powered off.
#define QSPI_E_FPGA_NO_SOC_CTRL     (-6)    ///< The SoC does not currently control the FPGA.
#define QSPI_E_FPGA_NOT_USER_MODE   (-7)    ///< The FPGA is not in USER mode.
#define QSPI_E_ARG_RANGE            (-8)    ///< An argument violates a range constraint.
#define QSPI_E_BAD_ARG              (-9)    ///< A bad argument value was passed.
#define QSPI_E_BAD_OPERATION        (-10)   ///< The operation is invalid or illegal.
#define QSPI_E_INV_OPTION           (-11)   ///< An invalid option was selected.
#define QSPI_E_TMO                  (-12)   ///< An operation or response time_out period expired.
#define QSPI_E_RESERVED             (-13)   ///< The argument value is reserved or unavailable.
#define QSPI_E_BAD_CLK              (-14)   ///< A clock is not enabled or violates an operational constraint.
#define QSPI_E_BAD_VERSION          (-15)   ///< The version ID is invalid.
#define QSPI_E_BUF_OVF              (-20)   ///< The buffer does not contain enough free space for the operation.

void qspi_direct_disable(void);
void qspi_direct_enable(void);
void qspi_direct_read(unsigned char *buf, unsigned int size, unsigned int offset);
QSPI_STATUS_CODE qspi_wait_for_device_ready(uint32_t time_out);
QSPI_STATUS_CODE qspi_wait_for_device_ready_schedule(uint32_t time_out);

/**
 * This section provisions support for various flash devices.
 */
#define QSPI_PROVISION_WINBOND_W25Q_SUPPORT 1
#define QSPI_PROVISION_MICRON_N25Q_SUPPORT  0

#define QSPI_PAGE_ADDR_MSK      0xFFFFFF00
#define QSPI_PAGE_SIZE          0x00000100 // 256 B
#define QSPI_SUBSECTOR_ADDR_MSK 0xFFFFF000
#define QSPI_SUBSECTOR_SIZE     0x00001000 // 4096 B
#define QSPI_SECTOR_ADDR_MSK    0xFFFF0000
#define QSPI_SECTOR_SIZE        0x00010000 // 64 KiB
#define QSPI_BANK_ADDR_MSK      0xFE000000
#define QSPI_BANK_SIZE          0x02000000 // 32 MiB
#define QSPI_BANK_WIDTH         (25)
#if QSPI_PROVISION_WINBOND_W25Q_SUPPORT
#    define QSPI_W25Q_DIE_ADDR_MSK 0xFE000000
#    define QSPI_W25Q_DIE_SIZE     0x02000000 // 32 MiB
#endif

#define QSPI_DEVICE_SIZE_BYTE_TO_KB (10) // Byte -> KB
#define QSPI_DEVICE_SIZE_BYTE_TO_MB (20) // Byte -> MB
#define QSPI_DEVICE_SIZE_BYTE_TO_GB (30) // Byte -> GB

/////

// Default delay timing (in ns) for N25Q.
// These values are from the N25Q handbook. The timing correctness is difficult
// to test because the test setup does not feature mutliple chips.
#define QSPI_TSHSL_NS_DEF (50)
#define QSPI_TSD2D_NS_DEF (0)
#define QSPI_TCHSH_NS_DEF (4)
#define QSPI_TSLCH_NS_DEF (4)

/*
// Default delay timing (in ns)
#define QSPI_TSHSL_NS_DEF       (200)
#define QSPI_TSD2D_NS_DEF       (255)
#define QSPI_TCHSH_NS_DEF       (20)
#define QSPI_TSLCH_NS_DEF       (20)
*/

// Flash commands
#define QSPI_STIG_OPCODE_READ                   (0x03)
#define QSPI_STIG_OPCODE_4BYTE_READ             (0x13)
#define QSPI_STIG_OPCODE_FASTREAD               (0x0B)
#define QSPI_STIG_OPCODE_FASTREAD_4BYTE         (0x0C)
#define QSPI_STIG_OPCODE_FASTREAD_DUAL_OUTPUT   (0x3B)
#define QSPI_STIG_OPCODE_FASTREAD_DUAL_4BYTE    (0x3C)
#define QSPI_STIG_OPCODE_FASTREAD_QUAD_OUTPUT   (0x6B)
#define QSPI_STIG_OPCODE_FASTREAD_QUAD_4BYTE    (0x6C)
#define QSPI_STIG_OPCODE_FASTREAD_DUAL_IO       (0xBB)
#define QSPI_STIG_OPCODE_FASTREAD_QUAD_IO       (0xEB)
#define QSPI_STIG_OPCODE_FASTREAD_QUAD_IO_4BYTE (0xEC)
#define QSPI_STIG_OPCODE_PP                     (0x02)
#define QSPI_STIG_OPCODE_PP_4BYTE               (0x12)
#define QSPI_STIG_OPCODE_DUAL_PP                (0xA2)
#define QSPI_STIG_OPCODE_QUAD_PP                (0x32)
#define QSPI_STIG_OPCODE_QUAD_PP_4BYTE          (0x34)
#define QSPI_STIG_OPCODE_RDID                   (0x9F)
#define QSPI_STIG_OPCODE_WREN                   (0x06)
#define QSPI_STIG_OPCODE_WRDIS                  (0x04)
#define QSPI_STIG_OPCODE_RDSR                   (0x05)
#define QSPI_STIG_OPCODE_WRSR                   (0x01)
#define QSPI_STIG_OPCODE_RDSR_2                 (0x35)
#define QSPI_STIG_OPCODE_WRSR_2                 (0x31)
#define QSPI_STIG_OPCODE_RDSR_3                 (0x15)
#define QSPI_STIG_OPCODE_SEC_ERASE              (0x20)
#define QSPI_STIG_OPCODE_SEC_ERASE_4BYTE        (0x21)
#define QSPI_STIG_OPCODE_BLK_ERASE              (0xD8)
#define QSPI_STIG_OPCODE_BLK_ERASE_4BYTE        (0xDC)
#define QSPI_STIG_OPCODE_BULK_ERASE             (0xC7)
#define QSPI_STIG_OPCODE_DIE_ERASE              (0xC4)
#define QSPI_STIG_OPCODE_CHIP_ERASE             (0x60)
#define QSPI_STIG_OPCODE_RD_EXT_REG             (0xC8)
#define QSPI_STIG_OPCODE_WR_EXT_REG             (0xC5)
#define QSPI_STIG_OPCODE_RD_STAT_REG            (0x05)
#define QSPI_STIG_OPCODE_WR_STAT_REG            (0x01)
#define QSPI_STIG_OPCODE_ENTER_4BYTE_MODE       (0xB7)
#define QSPI_STIG_OPCODE_EXIT_4BYTE_MODE        (0xE9)
#define QSPI_STIG_OPCODE_RD_NVOL_CFG_REG        (0xB5)
#define QSPI_STIG_OPCODE_SET_DRV_STRENGTH       (0x11)

// Micron commands, for 512 Mib, 1 Gib (64 MiB, 128 MiB) parts.
#if QSPI_PROVISION_MICRON_N25Q_SUPPORT

#    define QSPI_STIG_OPCODE_RESET_EN     (0x66)
#    define QSPI_STIG_OPCODE_RESET_MEM    (0x99)
#    define QSPI_STIG_OPCODE_RDFLGSR      (0x70)
#    define QSPI_STIG_OPCODE_CLRFLGSR     (0x50)
#    define QSPI_STIG_OPCODE_DISCVR_PARAM (0x5A)

#endif

// Spansion commands
// #define OPCODE_ECRM                 (0xFF) // Exit continuous read mode

/** Winbond commands
*/
#if QSPI_PROVISION_WINBOND_W25Q_SUPPORT

#    define QSPI_STIG_OPCODE_RESET_EN  (0x66)
#    define QSPI_STIG_OPCODE_RESET_MEM (0x99)
#    define QSPI_STIG_OPCODE_SRWR_EN   (0x50) ///< Volatile SR Write Enable
#    define QSPI_STIG_OPCODE_DISCVR_PARAM                                      \
    (0x5A) ///< Discoverable Parameter (SFDP) register

#endif

#define QSPI_READ_CLK_MHZ     (50)
#define QSPI_FASTREAD_CLK_MHZ (100)

// Manufacturer ID
#define QSPI_STIG_RDID_JEDECID_MICRON   (0x20)
#define QSPI_STIG_RDID_JEDECID_NUMONYX  (0x20) // Same as Micron
#define QSPI_STIG_RDID_JEDECID_SPANSION (0xEF)
#define QSPI_STIG_RDID_JEDECID_WINBOND  (0xEF) // Same as Spansion
#define QSPI_STIG_RDID_JEDECID_MACRONIC (0xC2)
#define QSPI_STIG_RDID_JEDECID_ATMEL    (0x1F)
#define QSPI_STIG_RDID_JEDECID_GD       (0xC8)

#define QSPI_STIG_RDID_JEDECID_GET(value)    ((value >> 0) & 0xff)
#define QSPI_STIG_RDID_CAPACITYID_GET(value) ((value >> 16) & 0xff)

#define QSPI_STIG_FLAGSR_ERASEPROGRAMREADY_GET(value) ((value >> 7) & 0x1)
#define QSPI_STIG_FLAGSR_ERASEREADY_GET(value)        ((value >> 7) & 0x1)
#define QSPI_STIG_FLAGSR_PROGRAMREADY_GET(value)      ((value >> 5) & 0x1)
#define QSPI_STIG_FLAGSR_ERASEERROR_GET(value)        ((value >> 7) & 0x1)
#define QSPI_STIG_FLAGSR_PROGRAMERROR_GET(value)      ((value >> 4) & 0x1)
#define QSPI_STIG_FLAGSR_ADDRESSINGMODE_GET(value)    ((value >> 1) & 0x1)
#define QSPI_STIG_FLAGSR_PROTECTIONERROR_GET(value)   ((value >> 0) & 0x1)

#define QSPI_STIG_SR_BUSY_GET(value) ((value >> 0) & 0x1)
/**
 * The size of the onboard SRAM in bytes.
 */
#define QSPI_SRAM_FIFO_SIZE (1024)

/*
 * The size of the onboard SRAM in entries. Each entry is word (32-bit) sized.
 */
#define QSPI_SRAM_FIFO_ENTRY_COUNT (1024 / sizeof(uint32_t))

///< qspi_clk operating frequency range.
#define QSPI_CLK_FREQ_MIN ((uint32_t)0)
///< HZ单位时钟频率换算为MHZ单位比例
#define QSPI_CLOCK_MHZ_RATIO (1000000)
///< The set of all valid QSPI controller interrupt status mask values.
#define QSPI_INT_STATUS_ALL                                                                             \
    (QSPI_INT_STATUS_MODE_FAIL | QSPI_INT_STATUS_UFL | QSPI_INT_STATUS_IDAC_OP_COMPLETE |               \
     QSPI_INT_STATUS_IDAC_OP_REJECT | QSPI_INT_STATUS_WR_PROT_VIOL | QSPI_INT_STATUS_ILL_AHB_ACCESS |   \
     QSPI_INT_STATUS_IDAC_WTRMK_TRIG | QSPI_INT_STATUS_RX_OVF | QSPI_INT_STATUS_TX_FIFO_NOT_FULL |      \
     QSPI_INT_STATUS_TX_FIFO_FULL | QSPI_INT_STATUS_RX_FIFO_NOT_EMPTY | QSPI_INT_STATUS_RX_FIFO_FULL |  \
     QSPI_INT_STATUS_IDAC_RD_FULL | QSPI_IRQMSK_STIGCOMPL_SET_MSK)
///< 直接访问方式/间接访问方式
#define QSPI_DIRECT_ACCESS   (0)
#define QSPI_INDIRECT_ACCESS (1)
///< 间接访问FIFO地址（和硬件祝万超确认这个没有限制）
#if defined(PS3OS_HBA_R5)
#define QSPI_INDIRECT_ADDRESS_MAP (QSPI_FLASH_MAP)
#else
#define QSPI_INDIRECT_ADDRESS_MAP (QSPI_FLASH_MAP + (32 * 1024))
#endif

///< QSPI控制器访问超时时间(单位为us, 参考芯片DEMO代码)
#define QSPI_CTRL_ACCESS_TIMEOUT                        (10000)
#define QSPI_MICROSEC_TO_NANOSEC(micro_sec) (micro_sec * 1000)
#define QSPI_MICROSEC_TO_MILLISEC(micro_sec) (micro_sec / 1000)
#define QSPI_MIN(a, b) ((a) > (b) ? (b) : (a))

#if QSPI_PROVISION_WINBOND_W25Q_SUPPORT
///< QSPI控制器配置从设备寄存器超时时间(单位为us, 参考芯片DEMO代码，经验值)
#define QSPI_CONFIG_DEVICE_TIMEOUT                      (1000000)
/* 根据FLASH芯片（W25Q）手册，数据如下
 *                                      TYP                 MAX
 * Write Status Register Time           2ms                 30ms
 * Page Program Time                    0.8ms               5ms
 * Sector Erase Time                    50ms                400ms
 * Block Erase Time(32KB)               120ms               1600ms
 * Block Erase Time(64KB)               200ms               2000ms
 * Chip Erase Time                      90s                 400s
 * Reset                                --                  30us
 */
///< 相对于手册的典型时间，实际的超时时间有100倍左右放大
#define QSPI_WRITE_DEVICE_STATUS_REG_TIMEOUT            (30000000)
#define QSPI_READ_DEVICE_TIMEOUT                        (3000000)
#define QSPI_WRITE_DEVICE_TIMEOUT                       (5000000)
#define QSPI_RESET_DEVICE_TIMEOUT                       (3000000)
#define QSPI_ERASE_DEVICE_4K_TIMEOUT                    (400000000)
#define QSPI_ERASE_DEVICE_64K_TIMEOUT                   (2000000000)
#define QSPI_ERASE_DEVICE_ENTIRE_TIMEOUT                (400000000)
#endif

/******************************************************************************/
/**
 * This type enumerates the QSPI controller master baud rate divisor selections.
 */
typedef enum QSPI_BAUD_DIV_e {
    QSPI_BAUD_DIV_2  = 0x0, /**< Divide by 2 */
    QSPI_BAUD_DIV_4  = 0x1, /**< Divide by 4 */
    QSPI_BAUD_DIV_6  = 0x2, /**< Divide by 6 */
    QSPI_BAUD_DIV_8  = 0x3, /**< Divide by 8 */
    QSPI_BAUD_DIV_10 = 0x4, /**< Divide by 10 */
    QSPI_BAUD_DIV_12 = 0x5, /**< Divide by 12 */
    QSPI_BAUD_DIV_14 = 0x6, /**< Divide by 14 */
    QSPI_BAUD_DIV_16 = 0x7, /**< Divide by 16 */
    QSPI_BAUD_DIV_18 = 0x8, /**< Divide by 18 */
    QSPI_BAUD_DIV_20 = 0x9, /**< Divide by 20 */
    QSPI_BAUD_DIV_22 = 0xA, /**< Divide by 22 */
    QSPI_BAUD_DIV_24 = 0xB, /**< Divide by 24 */
    QSPI_BAUD_DIV_26 = 0xC, /**< Divide by 26 */
    QSPI_BAUD_DIV_28 = 0xD, /**< Divide by 28 */
    QSPI_BAUD_DIV_30 = 0xE, /**< Divide by 30 */
    QSPI_BAUD_DIV_32 = 0xF  /**< Divide by 32 */
} QSPI_BAUD_DIV_t;

/******************************************************************************/
/**
 * Device Size Configuration
 *
 * This type defines the structure used to specify flash device size and write
 * protect regions.
 */
typedef struct QSPI_DEV_SIZE_CONFIG_s {
    uint32_t block_size;         /**< Number of bytes per device block. The
                                  *   number is specified as a power of 2.
                                  *   That is 0 = 1 byte, 1 = 2 bytes, ...
                                  *   16 = 65535 bytes, etc.
                                  */
    uint32_t page_size;          /**< Number of bytes per device page.  This
                                  *   is required by the controller for
                                  *   performing flash writes up to and
                                  *   across page boundaries.
                                  */
    uint32_t addr_size;          /**< Number of bytes used for the flash
                                  *   address. The value is \e n + 1
                                  *   based. That is 0 = 1 byte, 1 = 2 bytes,
                                  *   2 = 3 bytes, 3 = 4 bytes.
                                  */
    uint32_t lower_wrprot_block; /**< The block number that defines the lower
                                  *   block in the range of blocks that is
                                  *   protected from writing. This field
                                  *   is ignored it write protection is
                                  *   disabled.
                                  */
    uint32_t upper_wrprot_block; /**< The block number that defines the upper
                                  *   block in the range of blocks that is
                                  *   protected from writing. This field
                                  *   is ignored it write protection is
                                  *   disabled.
                                  */
    bool wrprot_enable;          /**< The write region enable value. A value
                                  *   of \b true enables write protection
                                  *   on the region specified by the
                                  *   \e lower_wrprot_block and
                                  *   \e upper_wrprot_block range.
                                  */
} QSPI_DEV_SIZE_CONFIG_t;

/******************************************************************************/
/**
 * This type enumerates the operational modes the QSPI controller can be
 * configured for. It may apply to instruction, address, and/or data width
 * interactions between the QSPI controller and the flash device.
 */
typedef enum QSPI_MODE_e {
    QSPI_MODE_SINGLE = 0, /**< Use Standard Single SPI (SIO-SPI) mode (bits
                           *   always transferred into the device on DQ0
                           *   only). Supported by all SPI flash devices.
                           */
    QSPI_MODE_DUAL = 1,   /**< Use Dual SPI (DIO-SPI) SPI mode where bits are
                           *   transferred on DQ0 and DQ1.
                           */
    QSPI_MODE_QUAD = 2    /**< Use Dual SPI (QIO-SPI) SPI mode where bits are
                           *   transferred on DQ0, DQ1, DQ3, and DQ3.
                           */
} QSPI_MODE_t;

/******************************************************************************/
/**
 * Peripheral chip select lines
 * If bit 9 of this register = 0, ss[3:0] are output thus:
 * ss[3:0] n_ss_out[3:0]
 * 4'bxxx0 4'b1110
 * 4'bxx01 4'b1101
 * 4'bx011 4'b1011
 * 4'b0111 4'b0111
 * 4'b1111 4'b1111 (no peripheral selected)
 **/
typedef enum QSPI_CS_e {
    QSPI_CS_SELECT_BUS0 = 0xE, /**< b1110 */
    QSPI_CS_SELECT_BUS1 = 0xD, /**< b1101 */
    QSPI_CS_SELECT_BUS2 = 0xB, /**< b1011 */
    QSPI_CS_SELECT_BUS3 = 0x7, /**< b0111 */
    QSPI_CS_SELECT_BUS_NULL = 0xF  /**< b1110 */
} QSPI_CS_t;

/******************************************************************************/
/**
 * This type enumerates the mode configurations available for driving the
 * ss_n[3:0] device chip selects.  The chip selects may be controlled as either
 * in a '1 of 4' or '4 to 16 decode' mode.
 */
typedef enum QSPI_CS_MODE_e {
    QSPI_CS_MODE_SINGLE_SELECT = 0, /**< Select 1 of 4 chip select ss_n[3:0]
    */
    QSPI_CS_MODE_DECODE = 1         /**< Select external 4 to 16 decode of
                                     *   ss_n[3:0].
                                     */
} QSPI_CS_MODE_t;

/******************************************************************************/
/**
 * This type enumerates the QSPI clock phase activity options outside the SPI
 * word.
 */
typedef enum QSPI_CLK_PHASE_e {
    QSPI_CLK_PHASE_ACTIVE = 0,  /**< The SPI clock is active outside the
                                 *   word
                                 */
    QSPI_CLK_PHASE_INACTIVE = 1 /**< The SPI clock is inactive outside the
                                 *   word
                                 */
} QSPI_CLK_PHASE_t;

/******************************************************************************/
/**
 * This type enumerates the QSPI clock polarity options outside the SPI word.
 */
typedef enum QSPI_CLK_POLARITY_e {
    QSPI_CLK_POLARITY_LOW = 0, /**< SPI clock is quiescent low outside the
                                *   word.
                                */
    QSPI_CLK_POLARITY_HIGH = 1 /**< SPI clock is quiescent high outside the
                                *   word.
                                */
} QSPI_CLK_POLARITY_t;

/******************************************************************************/
/**
 * QSPI Controller Timing Configuration
 *
 * This type defines the structure used to configure timing paramaters used by
 * the QSPI controller to communicate with a target flash device.
 *
 * All timing values are defined in cycles of the SPI master ref clock.
 */
typedef struct QSPI_TIMING_CONFIG_s {
    QSPI_CLK_PHASE_t clk_phase; /**< Selects whether the clock is in an
                                 *   active or inactive phase outside the
                                 *   SPI word.
                                 */

    QSPI_CLK_POLARITY_t clk_pol; /**< Selects whether the clock is quiescent
                                  *   low or high outside the SPI word.
                                  */

    uint32_t cs_da;   /**< Chip Select De-Assert. Added delay in
                       *   master reference clocks for the length
                       *   that the master mode chip select
                       *   outputs are de-asserted between
                       *   transactions. If CSDA = \e X, then the
                       *   chip select de-assert time will be: 1
                       *   sclk_out + 1 ref_clk + \e X ref_clks.
                       */
    uint32_t cs_dads; /**< Chip Select De-Assert Different
                       *   Slaves. Delay in master reference
                       *   clocks between one chip select being
                       *   de-activated and the activation of
                       *   another. This is used to ensure a quiet
                       *   period between the selection of two
                       *   different slaves.  CSDADS is only
                       *   relevant when switching between 2
                       *   different external flash devices. If
                       *   CSDADS = \e X, then the delay will be:
                       *   1 sclk_out + 3 ref_clks + \e X
                       *   ref_clks.
                       */
    uint32_t cs_eot;  /**< Chip Select End Of Transfer. Delay in
                       *   master reference clocks between last
                       *   bit of current transaction and
                       *   de-asserting the device chip select
                       *   (n_ss_out). By default (when CSEOT=0),
                       *   the chip select will be de-asserted on
                       *   the last falling edge of sclk_out at
                       *   the completion of the current
                       *   transaction. If CSEOT = \e X, then chip
                       *   selected will de-assert \e X ref_clks
                       *   after the last falling edge of
                       *   sclk_out.
                       */
    uint32_t cs_sot;  /**< Chip Select Start Of Transfer. Delay in
                       *   master reference clocks between setting
                       *   n_ss_out low and first bit transfer. By
                       *   default (CSSOT=0), chip select will be
                       *   asserted half a SCLK period before the
                       *   first rising edge of sclk_out. If CSSOT
                       *   = \e X, chip select will be asserted
                       *   half an sclk_out period before the
                       *   first rising edge of sclk_out + \e X
                       *   ref_clks.
                       */

    uint32_t rd_datacap; /**< The additional number of read data
                          *   capture cycles (ref_clk) that should be
                          *   applied to the internal read data
                          *   capture circuit.  The large
                          *   clock-to-out delay of the flash memory
                          *   together with trace delays as well as
                          *   other device delays may impose a
                          *   maximum flash clock frequency which is
                          *   less than the flash memory device
                          *   itself can operate at. To compensate,
                          *   software should set this register to a
                          *   value that guarantees robust data
                          *   captures.
                          */
} QSPI_TIMING_CONFIG_t;

/******************************************************************************/
/**
 * Device Instruction Configuration
 *
 * This type defines a structure for specifying the optimal instruction set
 * configuration to use with a target flash device.
 */
typedef struct QSPI_DEV_INST_CONFIG_s {
    uint32_t op_code;           /**< The read or write op code to use
                                 *   for the device transaction.
                                 */
    QSPI_MODE_t inst_type;      /**< Instruction mode type for the
                                 *   controller to use with the
                                 *   device. The instruction type
                                 *   applies to all instructions
                                 *   (reads and writes) issued from
                                 *   either the Direct Access
                                 *   Controller or the Indirect
                                 *   Acces Controller.
                                 */
    QSPI_MODE_t addr_xfer_type; /**< Address transfer mode type. The
                                 *   value of this field is ignored
                                 *   if the \e inst_type data member
                                 *   is set to anything other than
                                 *  QSPI_MODE_SINGLE. In that
                                 *   case, the addr_xfer_type
                                 *   assumes the same mode as the \e
                                 *   inst_type.
                                 */
    QSPI_MODE_t data_xfer_type; /**< Data transfer mode type. The
                                 *   value of this field is ignored
                                 *   if the \e inst_type data member
                                 *   is set to anything other than
                                 *  QSPI_MODE_SINGLE. In that
                                 *   case, the data_xfer_type
                                 *   assumes the same mode as the \e
                                 *   inst_type.
                                 */
    uint32_t dummy_cycles;      /**< Number of dummy clock cycles
                                 *   required by device for a read
                                 *   or write instruction.
                                 */

} QSPI_DEV_INST_CONFIG_t;

/******************************************************************************/
/**
 * This type definition enumerates the interrupt status conditions for the QSPI
 * controller.
 *
 * The enumerations serve as masks for the QSPI controller events that can be
 * set when the designated conditions occur and the corresponding event is
 * enabled.  When any of these event source conditions are true, the
 * ALT_INT_INTERRUPT_QSPI_IRQ interrupt output is asserted high.
 *
 * Interrupt sources are cleared when software calls qspiInt_clear(). The
 * interrupt sources are individually maskable using qspi_int_disable() and
 * qspi_int_enable().
 */
typedef enum qspi_int_status_e {
    /**
     * Mode fail M - indicates the voltage on pin n_ss_in is inconsistent with
     * the SPI mode. Set = 1 if n_ss_in is low in master mode (multi-master
     * contention). These conditions will clear the spi_enable bit and disable
     * the SPI.
     *  * 0 = no mode fault has been detected.
     *  * 1 = a mode fault has occurred.
     */
    QSPI_INT_STATUS_MODE_FAIL = (0x1 << 0),

    /**
     * Underflow Detected.
     *  * 0 = no underflow has been detected.
     *  * 1 = underflow is detected and an attempt to transfer data is made
     *        when the small TX FIFO is empty. This may occur when AHB write
     *        data is being supplied too slowly to keep up with the requested
     *        write operation.
     */
    QSPI_INT_STATUS_UFL = (0x1 << 1),

    /**
     * Controller has completed last triggered indirect operation.
     */
    QSPI_INT_STATUS_IDAC_OP_COMPLETE = (0x1 << 2),

    /**
     * Indirect operation was requested but could not be accepted. Two indirect
     * operations already in storage.
     */
    QSPI_INT_STATUS_IDAC_OP_REJECT = (0x1 << 3),

    /**
     * Write to protected area was attempted and rejected.
     */
    QSPI_INT_STATUS_WR_PROT_VIOL = (0x1 << 4),

    /**
     * Illegal AHB Access Detected. AHB write wrapping bursts and the use of
     * SPLIT/RETRY accesses will cause this interrupt to trigger.
     */
    QSPI_INT_STATUS_ILL_AHB_ACCESS = (0x1 << 5),

    /**
     * Indirect Transfer Watermark Level Breached.
     */
    QSPI_INT_STATUS_IDAC_WTRMK_TRIG = (0x1 << 6),

    /**
     * Receive Overflow. This should only occur in Legacy SPI mode.
     *
     * Set if an attempt is made to push the RX FIFO when it is full. This bit
     * is reset only by a system reset and cleared only when this register is
     * read. If a new push to the RX FIFO occurs coincident with a register read
     * this flag will remain set.
     *  * 0 = no overflow has been detected.
     *  * 1 = an overflow has occurred.
     */
    QSPI_INT_STATUS_RX_OVF = (0x1 << 7),

    /**
     * Small TX FIFO not full (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO has >= THRESHOLD entries.
     *  * 1 = FIFO has < THRESHOLD entries.
     */
    QSPI_INT_STATUS_TX_FIFO_NOT_FULL = (0x1 << 8),

    /**
     * Small TX FIFO full (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO is not full.
     *  * 1 = FIFO is full.
     */
    QSPI_INT_STATUS_TX_FIFO_FULL = (0x1 << 9),

    /**
     * Small RX FIFO not empty (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO has < RX THRESHOLD entries.
     *  * 1 = FIFO has >= THRESHOLD entries.
     */
    QSPI_INT_STATUS_RX_FIFO_NOT_EMPTY = (0x1 << 10),

    /**
     * Small RX FIFO full (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO is not full.
     *  * 1 = FIFO is full.
     */
    QSPI_INT_STATUS_RX_FIFO_FULL = (0x1 << 11),

    /**
     * Indirect Read partition of SRAM is full and unable to immediately
     * complete indirect operation.
     */
    QSPI_INT_STATUS_IDAC_RD_FULL = (0x1 << 12)

} qspi_int_status_t;

///< qspi访问模式配置结构体
typedef struct qspi_access_cfg {
    uint32_t access_mode;
    uint32_t address_mode;
    uint32_t inst_mode;
    uint32_t addr_mode;
    uint32_t data_mode;
    uint32_t write_dummy_cycle;
    uint32_t read_dummy_cycle;
    uint32_t write_op_code;
    uint32_t read_op_code;
} qspi_access_config_t;

#endif // __QSPI_PRIVATE_H__
