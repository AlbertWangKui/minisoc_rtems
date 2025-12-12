/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 * @file drv_spi_device.c
 * @brief
 * @author tianye
 * @version
 * @date 2024-05-22
 */

#include <string.h>
#include <stdlib.h>
#include <rtems/score/sysstate.h>
#include <rtems/counter.h>
#include <bsp/drv_spi_device.h>
#include "drv_spi_device_inner.h"
#include "spi-cadence-quadspi.h"
#include "bsp_config.h"

///< 只打印warn和err uart log
u32 g_cqspi_log_level = 2;

/* Define max times to check status register before we give up. */

/*
 * For everything but full-chip erase; probably could be much smaller, but kept
 * around for safety for now
 */
#define DEFAULT_READY_WAIT_SECONDS                      (600)
#define DEFAULT_READY_WAIT_FOR_PP_SECONDS               (1)
#define DEFAULT_READY_WAIT_FOR_SECTOR_ERASE_SECONDS     (4)
#define DEFAULT_READY_WAIT_FOR_BLOCK_ERASE_SECONDS      (10)
#ifdef CONFIG_PLATFORM_EMU
#define DEFAULT_READY_WAIT_FOR_PROBE_SECONDS            (1)
#else
#define DEFAULT_READY_WAIT_FOR_PROBE_SECONDS            (10)
#endif
#define DEFAULT_RESET_WAIT_MICROSEC                     (5000)

#define SPI_NOR_MAX_ADDR_NBYTES 4

#define SPI_NOR_SRST_SLEEP_MIN 200
#define SPI_NOR_SRST_SLEEP_MAX 400

#define SECTOR_ADDR_MSK     (0xFFFFF000)
#define BLOCK_ADDR_MSK      (0xFFFF0000)

struct spi_nor g_spi_nor_device[CQSPI_MAX_CHIPSELECT];

static uint32_t g_device_status_reg_info[SPI_DEV_REG_CNT] = {0};

/**
 * spi_nor_spimem_setup_op() - Set up common properties of a spi-mem op.
 * @nor:        pointer to a 'struct spi_nor'
 * @op:         pointer to the 'struct spi_mem_op' whose properties
 *          need to be initialized.
 * @proto:      the protocol from which the properties need to be set.
 */
static void spi_nor_spimem_setup_op(const struct spi_nor *nor,
                 struct spi_mem_op *op,
                 const enum spi_nor_protocol proto)
{
    op->cmd.buswidth = spi_nor_get_protocol_inst_nbits(proto);
    if (op->addr.nbytes)
        op->addr.buswidth = spi_nor_get_protocol_addr_nbits(proto);
    if (op->dummy.nbytes)
        op->dummy.buswidth = spi_nor_get_protocol_addr_nbits(proto);
    if (op->data.nbytes)
        op->data.buswidth = spi_nor_get_protocol_data_nbits(proto);
}

/**
 * spi_nor_spimem_read_data() - read data from flash's memory region via
 *                              spi-mem
 * @nor:        pointer to 'struct spi_nor'
 * @from:       offset to read from
 * @len:        number of bytes to read
 * @buf:        pointer to dst buffer
 *
 * Return: number of bytes read successfully, -errno otherwise
 */
static u32 spi_nor_spimem_read_data(struct spi_nor *nor, u32 from,
                    u32 len, u8 *buf)
{
    struct spi_mem_op op =
        SPI_MEM_OP(SPI_MEM_OP_CMD(nor->read_opcode, 0),
               SPI_MEM_OP_ADDR(nor->addr_nbytes, from, 0),
               SPI_MEM_OP_DUMMY(nor->read_dummy, 0),
               SPI_MEM_OP_DATA_IN(len, buf, 0));
    u32 nbytes;
    int error;

    spi_nor_spimem_setup_op(nor, &op, nor->read_proto);
    /* convert the dummy cycles to the number of bytes */
    op.dummy.nbytes = (nor->read_dummy * op.dummy.buswidth) / 8;
    error = qspi_exec_mem_op(nor, &op);
    if (error) {
        return error;
    }
    nbytes = op.data.nbytes;
    return nbytes;
}

/**
 * spi_nor_read_data() - read data from flash memory
 * @nor:        pointer to 'struct spi_nor'
 * @from:       offset to read from
 * @len:        number of bytes to read
 * @buf:        pointer to dst buffer
 *
 * Return: number of bytes read successfully, -errno otherwise
 */
static u32 spi_nor_read_data(struct spi_nor *nor, u32 from, u32 len, u8 *buf)
{
    return spi_nor_spimem_read_data(nor, from, len, buf);
}

/**
 * spi_nor_spimem_write_data() - write data to flash memory via
 *                               spi-mem
 * @nor:        pointer to 'struct spi_nor'
 * @to:         offset to write to
 * @len:        number of bytes to write
 * @buf:        pointer to src buffer
 *
 * Return: number of bytes written successfully, -errno otherwise
 */
static u32 spi_nor_spimem_write_data(struct spi_nor *nor, u32 to, u32 len, const u8 *buf)
{
    struct spi_mem_op op =
        SPI_MEM_OP(SPI_MEM_OP_CMD(nor->program_opcode, 0),
               SPI_MEM_OP_ADDR(nor->addr_nbytes, to, 0),
               SPI_MEM_OP_NO_DUMMY,
               SPI_MEM_OP_DATA_OUT(len, buf, 0));
    u32 nbytes;
    int error;

    spi_nor_spimem_setup_op(nor, &op, nor->write_proto);
    error = qspi_exec_mem_op(nor, &op);
    if (error) {
        return error;
    }
    nbytes = op.data.nbytes;

    return nbytes;
}

/**
 * spi_nor_write_data() - write data to flash memory
 * @nor:        pointer to 'struct spi_nor'
 * @to:         offset to write to
 * @len:        number of bytes to write
 * @buf:        pointer to src buffer
 *
 * Return: number of bytes written successfully, -errno otherwise
 */
static u32 spi_nor_write_data(struct spi_nor *nor, u32 to, u32 len, const u8 *buf)
{
    return spi_nor_spimem_write_data(nor, to, len, buf);
}

/**
 * spi_nor_write_enable() - Set write enable latch with Write Enable command.
 * @nor:    pointer to 'struct spi_nor'.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_write_enable(struct spi_nor *nor)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_WREN_OP;

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    if (ret) {
        if (ret == -EAGAIN) {
            spi_info("retry %d on Write Enable\n", ret);
        } else {
            spi_err("error %d on Write Enable\n", ret);
        }
    }
    return ret;
}

/**
 * spi_nor_write_disable() - Send Write Disable instruction to the chip.
 * @nor:    pointer to 'struct spi_nor'.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_write_disable(struct spi_nor *nor)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_WRDI_OP;

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    if (ret) {
        spi_err("error %d on Write Disable\n", ret);
    }

    return ret;
}

/**
 * spi_nor_read_id() - Read the JEDEC ID.
 * @nor:    pointer to 'struct spi_nor'.
 * @naddr:  number of address bytes to send. Can be zero if the operation
 *      does not need to send an address.
 * @ndummy: number of dummy bytes to send after an opcode or address. Can
 *      be zero if the operation does not require dummy bytes.
 * @id:     pointer to a DMA-able buffer where the value of the JEDEC ID
 *      will be written.
 * @proto:  the SPI protocol for register operation.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_read_id(struct spi_nor *nor, u8 naddr, u8 ndummy, u8 *id,
            enum spi_nor_protocol proto)
{
    int ret;

    struct spi_mem_op op = SPI_NOR_READID_OP(naddr, ndummy, id, SPI_NOR_MAX_ID_LEN);
    spi_nor_spimem_setup_op(nor, &op, proto);
    ret = qspi_exec_mem_op(nor, &op);

    return ret;
}

/**
 * spi_nor_read_sr() - Read the Status Register.
 * @nor:    pointer to 'struct spi_nor'.
 * @sr:     pointer to a DMA-able buffer where the value of the
 *              Status Register will be written. Should be at least 2 bytes.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_read_sr(struct spi_nor *nor, u8 *sr)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_RDSR_OP(sr);

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    if (ret) {
        spi_err("error %d reading SR\n", ret);
    }
    return ret;
}

static void spi_nor_dev_reg_capture(struct spi_nor *nor)
{
    u8 sr;
    int ret;

    ///< 当前只获取状态寄存器，后续定位需要增加其他寄存器再适配
    ret = spi_nor_read_sr(nor, &sr);
    if (ret != 0) {
        spi_err("read dev sr failed(%d).\n", ret);
    } else {
        g_device_status_reg_info[0] = (uint32_t)sr;
        spi_err("dev sr:0x%08x.\n", g_device_status_reg_info[0]);
    }
}

#if 0
/**
 * spi_nor_read_cr() - Read the Configuration Register using the
 * SPINOR_OP_RDCR (35h) command.
 * @nor:    pointer to 'struct spi_nor'
 * @cr:     pointer to a DMA-able buffer where the value of the
 *              Configuration Register will be written.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_read_cr(struct spi_nor *nor, u8 *cr)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_RDCR_OP(cr);

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    if (ret) {
        spi_err("error %d reading CR\n", ret);
    }

    return ret;
}
#endif

/**
 * spi_nor_set_4byte_addr_mode_en4b_ex4b() - Enter/Exit 4-byte address mode
 *          using SPINOR_OP_EN4B/SPINOR_OP_EX4B. Typically used by
 *          Winbond and Macronix.
 * @nor:    pointer to 'struct spi_nor'.
 * @enable: true to enter the 4-byte address mode, false to exit the 4-byte
 *      address mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_set_4byte_addr_mode_en4b_ex4b(struct spi_nor *nor, bool enable)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_EN4B_EX4B_OP(enable);

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    if (ret) {
        spi_err("error %d setting 4-byte mode\n", ret);
    }
    return ret;
}

/**
 * spi_nor_set_4byte_addr_mode_wren_en4b_ex4b() - Set 4-byte address mode using
 * SPINOR_OP_WREN followed by SPINOR_OP_EN4B or SPINOR_OP_EX4B. Typically used
 * by ST and Micron flashes.
 * @nor:    pointer to 'struct spi_nor'.
 * @enable: true to enter the 4-byte address mode, false to exit the 4-byte
 *      address mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_set_4byte_addr_mode_wren_en4b_ex4b(struct spi_nor *nor, bool enable)
{
    int ret;

    ret = spi_nor_write_enable(nor);
    if (ret) {
        return ret;
    }
    ret = spi_nor_set_4byte_addr_mode_en4b_ex4b(nor, enable);
    if (ret) {
        return ret;
    }
    return spi_nor_write_disable(nor);
}

/**
 * spi_nor_sr_ready() - Query the Status Register to see if the flash is ready
 * for new commands.
 * @nor:    pointer to 'struct spi_nor'.
 *
 * Return: 1 if ready, 0 if not ready, -errno on errors.
 */
static int spi_nor_sr_ready(struct spi_nor *nor)
{
    int ret;
    u8 status;

    ret = spi_nor_read_sr(nor, &status);
    if(ret) {
        return ret;
    }
    return !(status & SR_WIP);
}

/**
 * spi_nor_ready() - Query the flash to see if it is ready for new commands.
 * @nor:    pointer to 'struct spi_nor'.
 *
 * Return: 1 if ready, 0 if not ready, -errno on errors.
 */
static int spi_nor_ready(struct spi_nor *nor)
{
    int ret;

    ret = spi_nor_sr_ready(nor);
    return ret;
}

/**
 * spi_nor_wait_till_ready_with_timeout() - Service routine to read the
 * Status Register until ready, or timeout occurs.
 * @nor:        pointer to "struct spi_nor".
 * @timeout_jiffies:    jiffies to wait until timeout.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_wait_till_ready_with_timeout(struct spi_nor *nor, unsigned long seconds)
{
    u32 timeout_ticks;
    u32 divisor;
    s32 ret;

    ret = rtems_configuration_get_milliseconds_per_tick();
    ret ? (divisor = ret) : (divisor = 1);
    timeout_ticks = (seconds * 1000) / divisor;
    while (timeout_ticks) {
        ret = spi_nor_ready(nor);
        if (ret < 0) {
            return ret;
        }
        if (ret) {
            return 0;
        }
        timeout_ticks--;
        if (_System_all_cpu_state_Shutdown_Get() == false) {
            rtems_task_wake_after(1);
        } else {
            rtems_counter_delay_nanoseconds(10 * 1000 * 1000);
        }
    }

    spi_err("flash operation timed out\n");
    return -ETIMEDOUT;
}

static int spi_nor_prep_and_lock_rd(struct spi_nor *nor)
{
    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_lock(&nor->lock);
    }
    return 0;
}

static void spi_nor_unlock_and_unprep_rd(struct spi_nor *nor)
{
    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_unlock(&nor->lock);
    }
}

static int spi_nor_global_lock(struct spi_nor *nor)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_GBULK_LOCK_OP;

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);

    return ret;
}

static int spi_nor_global_unlock(struct spi_nor *nor)
{
    int ret;
    struct spi_mem_op op = SPI_NOR_GBULK_UNLOCK_OP;

    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    return ret;
}

static int spi_nor_blk_sector_lock(struct spi_nor *nor, u32 offset)
{
    int ret;
    struct spi_mem_op op  = SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_BLK_SECTOR_LOCK, 0),
            SPI_MEM_OP_ADDR(nor->addr_nbytes, offset, 0),
            SPI_MEM_OP_NO_DUMMY,
            SPI_MEM_OP_NO_DATA);
    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    return ret;
}

static int spi_nor_blk_sector_unlock(struct spi_nor *nor, u32 offset)
{
    int ret;
    struct spi_mem_op op  = SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_BLK_SECTOR_UNLOCK, 0),
            SPI_MEM_OP_ADDR(nor->addr_nbytes, offset, 0),
            SPI_MEM_OP_NO_DUMMY,
            SPI_MEM_OP_NO_DATA);
    spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &op);
    return ret;
}

static bool spi_nor_sector_range(struct spi_nor *nor, u32 offset)
{
    return (offset < SZ_64K || offset >= (nor->info->size - SZ_64K)) ? true : false;
}

static int _spi_nor_writelock_lock(struct spi_nor *nor, u32 offset, u32 *locked)
{
    s32 ret = 0;
    *locked = 0;

    ret = spi_nor_write_enable(nor);
    if (ret) {
        goto write_enable_failed;
    }
    if (spi_nor_sector_range(nor, offset)) {
        ret = spi_nor_blk_sector_lock(nor, offset & (~(SZ_4K - 1)));
        if (ret) {
            goto gobal_lock_failed;
        }
        *locked = SZ_4K;
    } else {
        ret = spi_nor_blk_sector_lock(nor, offset & (~(SZ_64K - 1)));
        if (ret) {
            goto gobal_lock_failed;
        }
        *locked = SZ_64K;
    }
write_enable_failed:
gobal_lock_failed:
    return ret;
}

static u32 down_round_aligment(u32 offset, u32 alignment_msk)
{
    return (offset & alignment_msk);
}

static u32 up_round_aligment(u32 offset, u32 alignment_msk)
{

    return ((offset | (~alignment_msk) ) + 1);
}

static void adjustment_writelock_offset(struct spi_nor *nor, u32 offset, u32 size, u32 *new_offset, u32 *new_size)
{
    u32 offset_end = offset + size - 1;
    u32 new_offset_end = offset_end;

    if (spi_nor_sector_range(nor, offset)) {
        *new_offset = down_round_aligment(offset, (~(SZ_4K - 1)));
    } else {
        *new_offset = down_round_aligment(offset, (~(SZ_64K - 1)));
    }

    if (spi_nor_sector_range(nor, offset_end)) {
        new_offset_end = up_round_aligment(offset_end , (~(SZ_4K - 1)));
    } else {
        new_offset_end = up_round_aligment(offset_end , (~(SZ_64K - 1)));
    }
    *new_size = new_offset_end - *new_offset;
}

static int _spi_nor_writelock_unlock(struct spi_nor *nor, u32 offset, u32 *unlocked)
{

    int ret = 0;

    ret = spi_nor_write_enable(nor);
    if (ret) {
        goto write_enable_failed;
    }
    *unlocked = 0;
    if (spi_nor_sector_range(nor, offset)) {
        ret = spi_nor_blk_sector_unlock(nor, offset & (~(SZ_4K - 1)));
        if (ret) {
            goto gobal_unlock_failed;
        }
        *unlocked = SZ_4K;
    } else {
        ret = spi_nor_blk_sector_unlock(nor, offset & (~(SZ_64K - 1)));
        if (ret) {
            goto gobal_unlock_failed;
        }
        *unlocked = SZ_64K;
    }
write_enable_failed:
gobal_unlock_failed:
    return ret;
}

static int spi_nor_writelock_lock(struct spi_nor *nor, u32 start, u32 size)
{
    s32 ret = 0;
    u32 new_offset = 0;
    u32 new_size = 0;
    u32 lockd_size = 0;
    u32 lock_offset = 0;

    if (nor->support_wr_protect != true) {
        return 0;
    }

    adjustment_writelock_offset(nor, start, size, &new_offset, &new_size);
    lock_offset = new_offset;
    do
    {
        ret = _spi_nor_writelock_lock(nor, lock_offset, &lockd_size);
        if(ret != 0) {
            spi_err("writelock lock error addr = %x ,status = %u\n", lock_offset, ret);
            break;
        }
        lock_offset += lockd_size;
    }while(lock_offset < (new_size + new_offset));

    return ret;
}

static int spi_nor_writelock_unlock(struct spi_nor *nor, u32 start, u32 size)
{
    s32 ret = 0;
    u32 new_offset = 0;
    u32 new_size = 0;
    u32 lockd_size = 0;
    u32 lock_offset = 0;

    if (nor->support_wr_protect != true) {
        return 0;
    }

    adjustment_writelock_offset(nor, start, size, &new_offset, &new_size);
    lock_offset = new_offset;
    do
    {
        ret = _spi_nor_writelock_unlock(nor, lock_offset, &lockd_size);
        if(ret != 0) {
            spi_err("writelock unlock error addr = %x ,status = %u\n", lock_offset, ret);
            break;
        }
        lock_offset += lockd_size;
    }while(lock_offset < (new_size + new_offset));

    return ret;
}

static int spi_nor_writelock_global_lock(struct spi_nor *nor)
{
    int ret = 0;

    if (nor->support_wr_protect != true) {
        return 0;
    }

    ret = spi_nor_write_enable(nor);
    if (ret) {
        goto write_enable_failed;
    }
    ret = spi_nor_global_lock(nor);
    if (ret) {
        goto gobal_lock_failed;
    }
write_enable_failed:
gobal_lock_failed:
    return ret;

}

static int spi_nor_writelock_global_unlock(struct spi_nor *nor)
{
    int ret = 0;

    if (nor->support_wr_protect != true) {
        return 0;
    }

    ret = spi_nor_write_enable(nor);
    if (ret) {
        goto write_enable_failed;
    }
    ret = spi_nor_global_unlock(nor);
    if (ret) {
        goto gobal_unlock_failed;
    }
write_enable_failed:
gobal_unlock_failed:
    return ret;

}

static int spi_nor_prep_and_lock_pp(struct spi_nor *nor, u32 start, u32 len, bool trylock)
{
    int ret;

    if (_System_all_cpu_state_Shutdown_Get() == false) {
        if (trylock) {
            ret = _Mutex_Try_acquire(&nor->lock);
            if (ret != 0) {
                ret = -EAGAIN;
                goto exit;
            }
        } else {
            rtems_mutex_lock(&nor->lock);
        }
    }
    ret = spi_nor_writelock_unlock(nor, start, len);

exit:
    return ret;
}

static void spi_nor_unlock_and_unprep_pp(struct spi_nor *nor, u32 start, u32 len)
{
    (void)spi_nor_writelock_lock(nor, start, len);
    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_unlock(&nor->lock);
    }
}

static int spi_nor_prep_and_lock_erase(struct spi_nor *nor, u32 start, enum erase_type etype)
{
    u32 size = 0;
    int ret;

    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_lock(&nor->lock);
    }
    if (etype == ERASE_CHIP) {
        ret = spi_nor_writelock_global_unlock(nor);
    } else {
        if (etype == ERASE_SECTOR) {
            size = SZ_4K;
        } else {
            size = SZ_64K;
        }
        ret = spi_nor_writelock_unlock(nor, start, size);
    }

    return ret;
}

static void spi_nor_unlock_and_unprep_erase(struct spi_nor *nor, u32 start, enum erase_type etype)
{
    u32 size = 0;

    if (etype == ERASE_CHIP) {
        (void)spi_nor_writelock_global_lock(nor);
    } else {
        if (etype == ERASE_SECTOR) {
            size = SZ_4K;
        } else {
            size = SZ_64K;
        }
        (void)spi_nor_writelock_lock(nor, start, size);
    }
    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_unlock(&nor->lock);
    }
}

/* Manufacturer drivers. */
extern const struct spi_nor_manufacturer spi_nor_gigadevice;
extern const struct spi_nor_manufacturer spi_nor_macronix;
extern const struct spi_nor_manufacturer spi_nor_winbond;
extern const struct spi_nor_manufacturer spi_nor_xmc;
extern const struct spi_nor_manufacturer spi_cypress;
extern const struct spi_nor_manufacturer spi_mclogic;
extern const struct spi_nor_manufacturer spi_default;

static const struct spi_nor_manufacturer *manufacturers[] = {
    &spi_nor_gigadevice,
    &spi_nor_macronix,
    &spi_nor_winbond,
    &spi_nor_xmc,
    &spi_cypress,
    &spi_mclogic,
};

extern struct cqspi_controller g_cqspi_dev;

static const struct flash_info *spi_nor_match_id(struct spi_nor *nor,
                         const u8 *id)
{
    const struct flash_info *part;
    unsigned int i, j;

    for (i = 0; i < ARRAY_SIZE(manufacturers); i++) {
        for (j = 0; j < manufacturers[i]->nparts; j++) {
            part = &manufacturers[i]->parts[j];
            if (!memcmp(part->id.bytes, id, part->id.len)) {
                nor->manufacturer = manufacturers[i];
                return part;
            }
        }
    }

    ///< 不在兼容类别使用默认设备
    printk("Unrecongnized JEDEC id tytes:\n");
    printk("0x");
    for (int k = 0; k < SPI_NOR_MAX_ID_LEN; k++) {
        printk("%02x", nor->id[k]);
    }
    printk(".\n");

    printk("Use default config.\n");
    return &(spi_default.parts[0]);
}

static int spi_nor_read(struct channel_info *chan_info, u32 from, u32 len, u32 *retlen, u8 *buf)
{
    struct spi_nor *nor = &g_spi_nor_device[chan_info->channel_id];
    int ret;

    if ((nor->cqspi == NULL) || ((nor->cqspi->is_decoded_cs == false) && (chan_info->channel_id >= CQSPI_MAX_CHIPSELECT_NO_DECODED))) {
        ret = -ENODEV;
        goto params_err;
    }
    nor->channel_id = chan_info->channel_id;

    spi_debug("from 0x%08x, len %d\n", (u32)from, len);
    ret = spi_nor_prep_and_lock_rd(nor);
    if (ret) {
        goto lock_err;
    }

    while (len) {
        u32 addr = from;

        ret = spi_nor_read_data(nor, addr, len, buf);
        if (ret == 0) {
            /* We shouldn't see 0-length reads */
            ret = -EIO;
            goto read_err;
        }
        if (ret < 0) {
            goto read_err;
        }
        if (ret > len) {
            spi_warn("read ret(%d) big than len(%d).\n", ret, len);
        }
        *retlen += ret;
        buf += ret;
        from += ret;
        len -= ret;
    }
    ret = 0;
    spi_nor_unlock_and_unprep_rd(nor);
    return ret;

read_err:
lock_err:
    spi_nor_dev_reg_capture(nor);
    spi_nor_unlock_and_unprep_rd(nor);
params_err:
    return ret;
}

/*
 * Write an address range to the nor chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int spi_nor_write(struct channel_info *chan_info, u32 to, u32 len,
    u32 *retlen, const u8 *buf)
{
    struct spi_nor *nor = &g_spi_nor_device[chan_info->channel_id];
    int ret;
    u32 page_remain, i;
    u32 page_size = SPI_NOR_DEFAULT_PAGE_SIZE;

    if ((nor->cqspi == NULL) || ((nor->cqspi->is_decoded_cs == false) && (chan_info->channel_id >= CQSPI_MAX_CHIPSELECT_NO_DECODED))) {
        ret = -ENODEV;
        goto params_err;
    }
    nor->channel_id = chan_info->channel_id;

    spi_debug("to 0x%08x, len %d\n", to, len);
    ret = spi_nor_prep_and_lock_pp(nor, to, len, false);
    if (ret) {
        goto lock_err;
    }

    for (i = 0; i < len; ) {
        u32 written;
        u32 addr = to + i;
        u32 page_offset = addr & (page_size - 1);
        /* the size of data remaining on the first page */
        page_remain = min(page_size - page_offset, len - i);

        ret = spi_nor_write_enable(nor);
        if (ret) {
            goto write_err;
        }
        ret = spi_nor_write_data(nor, addr, page_remain, buf + i);
        if (ret < 0) {
            goto write_err;
        }
        written = ret;
        ret = spi_nor_wait_till_ready_with_timeout(nor, DEFAULT_READY_WAIT_FOR_PP_SECONDS);
        if (ret) {
            goto write_err;
        }
        *retlen += written;
        i += written;
    }
    spi_nor_unlock_and_unprep_pp(nor, to, len);
    return ret;

write_err:
lock_err:
    spi_nor_dev_reg_capture(nor);
    spi_nor_unlock_and_unprep_pp(nor, to, len);
params_err:
    return ret;
}

static int spi_nvsram_write(struct channel_info *chan_info, u32 to, u32 len,
    u32 *retlen, const u8 *buf, bool trylock)
{
    struct spi_nor *nor = &g_spi_nor_device[chan_info->channel_id];
    int ret;
    u32 i;

    if ((nor->cqspi == NULL) || ((nor->cqspi->is_decoded_cs == false) && (chan_info->channel_id >= CQSPI_MAX_CHIPSELECT_NO_DECODED))) {
        ret = -ENODEV;
        goto params_err;
    }
    nor->channel_id = chan_info->channel_id;

    spi_debug("to 0x%08x, len %d\n", to, len);
    ret = spi_nor_prep_and_lock_pp(nor, to, len, trylock);
    if (ret) {
        if (ret == -EAGAIN) {
            ///< device锁try失败，直接返回
            goto try_again_1;
        } else {
            goto lock_err;
        }
    }

    ///< 获取device锁成功，记录trylock标记到device结构体
    nor->trylock = trylock;
    for (i = 0; i < len; ) {
        u32 written;
        u32 addr = to + i;

        ret = spi_nor_write_enable(nor);
        if (ret) {
            ///< qspi控制器锁try失败，回退trylock标记并释放device锁
            if (ret == -EAGAIN) {
                goto try_again_2;
            } else {
                goto write_err;
            }
        }
        ret = spi_nor_write_data(nor, addr, len, buf);
        if (ret < 0) {
            if (ret == -EAGAIN) {
                ///< qspi控制器锁try失败，回退trylock标记并释放device锁
                goto try_again_2;
            } else {
                goto write_err;
            }
        }
        written = ret;
        *retlen += written;
        i += written;
        ret = 0;
    }
    nor->trylock = false;
    spi_nor_unlock_and_unprep_pp(nor, to, len);
    return ret;

write_err:
lock_err:
    spi_nor_dev_reg_capture(nor);
try_again_2:
    nor->trylock = false;
    spi_nor_unlock_and_unprep_pp(nor, to, len);
try_again_1:
params_err:
    return ret;
}

static int spi_nor_default_setup(struct spi_nor *nor)
{
    u8 io_wire;

    ///<  从当前flash和nvsram兼容列表分析，以下判断能够兼容，后续如果有需求在考虑自动探测

    ///< 1 型号不在兼容列表，使用最通用的命令保证尽量能访问
    if (nor->info->type == DEV_NONE) {
        nor->read_proto = SNOR_PROTO_1_1_1;
        nor->write_proto = SNOR_PROTO_1_1_1;
        nor->read_opcode = SPINOR_OP_READ_FAST_4B;
        nor->read_dummy = 8;
        nor->program_opcode = SPINOR_OP_PP_4B;
        return 0;
    }

    ///< 2 如果device设备配置为1线表示设备当前需要按1线访问
    ///< 3 否则安装qspi控制器的io线配置
    if (nor->info->io_wire != 0xFF) {
        io_wire = nor->info->io_wire;
    } else {
        io_wire = nor->cqspi->io_wire_cnt;
    }
    if (io_wire == 1) {
        nor->read_proto = SNOR_PROTO_1_1_1;
        nor->write_proto = SNOR_PROTO_1_1_1;
        if (nor->info->size <= SZ_16M) {
            nor->read_opcode = SPINOR_OP_READ_FAST;
            nor->read_dummy = 8;
            nor->program_opcode = SPINOR_OP_PP;
        } else {
            nor->read_opcode = SPINOR_OP_READ_FAST_4B;
            nor->read_dummy = 8;
            nor->program_opcode = SPINOR_OP_PP_4B;
        }
    } else if (io_wire == 2) {
        nor->read_proto = SNOR_PROTO_1_1_2;
        nor->write_proto = SNOR_PROTO_1_1_2;
        if (nor->info->size <= SZ_16M) {
            nor->read_opcode = SPINOR_OP_READ_1_1_2;
            nor->read_dummy = 8;
            nor->program_opcode = SPINOR_OP_PP;
        } else {
            nor->read_opcode = SPINOR_OP_READ_1_1_2_4B;
            nor->read_dummy = 8;
            nor->program_opcode = SPINOR_OP_PP_4B;
        }
    } else if (io_wire == 4) {
        nor->read_proto = SNOR_PROTO_1_1_4;
        nor->write_proto = SNOR_PROTO_1_1_4;
        if (nor->info->size <= SZ_16M) {
            nor->read_opcode = SPINOR_OP_READ_1_1_4;
            nor->read_dummy = 8;
            nor->program_opcode = SPINOR_OP_PP_1_1_4;
        } else {
            nor->read_opcode = SPINOR_OP_READ_1_1_4_4B;
            nor->read_dummy = 8;
            nor->program_opcode = SPINOR_OP_PP_1_1_4_4B;
        }
    } else {
        return -ENOENT;
    }
    return 0;
}

static int spi_nor_set_addr_nbytes(struct spi_nor *nor)
{
    if (nor->info->size > SZ_16M) {
        /* enable 4-byte addressing if the device exceeds 16MiB */
        nor->addr_nbytes = 4;
    } else {
        nor->addr_nbytes = 3;
    }
    return 0;
}

static int spi_nor_setup(struct spi_nor *nor)
{
    int ret;

    ret = spi_nor_default_setup(nor);
    if (ret)
        return ret;

    return spi_nor_set_addr_nbytes(nor);
}

static int spi_nor_init_params(struct spi_nor *nor)
{
    ///< linux解析sfdp注册不通的函数，我们应用简单直接注册
    nor->quad_enable = NULL;
    nor->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode_wren_en4b_ex4b;
    return 0;
}

/**
 * spi_nor_quad_enable() - enable Quad I/O if needed.
 * @nor:                pointer to a 'struct spi_nor'
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_quad_enable(struct spi_nor *nor)
{
    int ret = 0;

    /*
     * 1 写状态寄存器遇到掉电可能出现状态寄存器错乱问题
     * 2 正常情况下在生产阶段会将flash设置为quad状态。
     * 所以，这里os阶段暂时不设置，如果后续需要再支持。
     */
    if (nor->quad_enable != NULL) {
        ret = nor->quad_enable(nor, true);
    }
    return ret;
}

/**
 * spi_nor_set_4byte_addr_mode() - Set address mode.
 * @nor:                pointer to a 'struct spi_nor'.
 * @enable:             enable/disable 4 byte address mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_set_4byte_addr_mode(struct spi_nor *nor, bool enable)
{
    int ret;

    if (nor->set_4byte_addr_mode == NULL) {
        return -1;
    }
    ret = nor->set_4byte_addr_mode(nor, enable);
    if (ret && ret != -EOPNOTSUPP) {
        return ret;
    }
    return 0;
}

static int spi_nor_init(struct spi_nor *nor)
{
    int err;

    err = spi_nor_quad_enable(nor);
    if (err) {
        spi_err("quad mode not supported\n");
        return err;
    }

    if (nor->addr_nbytes == 4) {
        return spi_nor_set_4byte_addr_mode(nor, true);
    }

    return 0;
}

#if 0
static int spi_nor_reset_dev(struct spi_nor *nor)
{
    int ret;

    struct spi_mem_op srsten_op = SPINOR_SRSTEN_OP;
    spi_nor_spimem_setup_op(nor, &srsten_op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &srsten_op);
    if (ret) {
        spi_err("Rset enable failed(%d)", ret);
        goto reset_failed;
    }

    struct spi_mem_op srst_op = SPINOR_SRST_OP;
    spi_nor_spimem_setup_op(nor, &srst_op, nor->reg_proto);
    ret = qspi_exec_mem_op(nor, &srst_op);
    if (ret) {
        spi_err("Rset device failed(%d)", ret);
        goto reset_failed;
    }

    ///< 发送66h+99h复位序列，Winbound要求等待30us，其他device手册没有看到，这里统一等待5000us
    rtems_counter_delay_nanoseconds(DEFAULT_RESET_WAIT_MICROSEC);
    return 0;
reset_failed:
    return ret;
}
#endif

static int spi_nor_probe(struct channel_info *chan_info, struct spi_dev_info *dev_info)
{
    const struct flash_info *info;
    struct spi_nor *nor = &g_spi_nor_device[chan_info->channel_id];
    int ret;

    if (nor->cqspi != NULL) {
        ret = -EEXIST;
        goto probe_repeat;
    }

    nor->cqspi = &g_cqspi_dev;
    if ((nor->cqspi->is_decoded_cs == false) && (chan_info->channel_id >= CQSPI_MAX_CHIPSELECT_NO_DECODED)) {
        ret = -ENODEV;
        goto params_err;
    }
    rtems_mutex_init(&nor->lock, "spi lock");
    nor->channel_id = chan_info->channel_id;

    nor->reg_proto = SNOR_PROTO_1_1_1;
    nor->read_proto = SNOR_PROTO_1_1_1;
    nor->write_proto = SNOR_PROTO_1_1_1;
#if 0
    /*
     * 此段为特殊处理：
     * 场景1 preos跳转到os，要保证等flash ready。如果没有等，此处只等一个write的时间，超时后就reset flash；
     * 场景2 os业务正常运行时发生coredump，此处会等最后一个write时间，超过时间reset flash（此种场景flash接口并没有返回ok，理论上不应该有问题）。
     */
    ret = spi_nor_wait_till_ready_with_timeout(nor, DEFAULT_READY_WAIT_FOR_PP_SECONDS);
    if (ret) {
        spi_err("Wait device ready timeout(%d)", ret);
    }
    ret = spi_nor_reset_dev(nor);
    if (ret) {
        spi_err("Reset device failed(%d)", ret);
        goto probe_failed;
    }
#else
    ret = spi_nor_wait_till_ready_with_timeout(nor, DEFAULT_READY_WAIT_FOR_PROBE_SECONDS);
    if (ret) {
        spi_err("Wait device ready timeout(%d)\n", ret);
    }
#endif

    ret = spi_nor_read_id(nor, 0, 0, nor->id, nor->reg_proto);
    if (ret) {
        spi_err("Read jedec id failed(%d)\n", ret);
        goto probe_failed;
    }
    info = spi_nor_match_id(nor, nor->id);
    nor->info = info;

    ret = spi_nor_init_params(nor);
    if (ret) {
        goto init_failed;
    }
    ret = spi_nor_setup(nor);
    if (ret) {
        goto init_failed;
    }
    ret = spi_nor_init(nor);
    if (ret) {
        goto init_failed;
    }

    nor->support_wr_protect = false;    ///< 默认不支持写保护
    memcpy(dev_info->id, nor->id, SPI_NOR_MAX_ID_LEN);
    dev_info->size = nor->info->size;
    dev_info->type = nor->info->type;
    printk("Manufacturer and device ID:\n");
    printk("0x");
    for (int k = 0; k < nor->info->id.len; k++) {
        printk("%02x", nor->id[k]);
    }
    printk(".\n");
    return 0;
init_failed:
    nor->info = NULL;
probe_failed:
    spi_nor_dev_reg_capture(nor);
params_err:
    nor->cqspi = NULL;
probe_repeat:
    return ret;
}

int32_t spi_dev_config(struct channel_info *chan_info, struct spi_dev_config *dev_config)
{
    struct spi_nor *nor = NULL;
    int ret = 0;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT || dev_config == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    nor = &g_spi_nor_device[chan_info->channel_id];
    ///< 记录调用者的配置，如果有需要发送配置命令到flash
    ///< 写保护wps位当前方案是生产阶段打开，所以这里只记录配置不发送设置命令
    nor->support_wr_protect = dev_config->support_wr_protect;

exit:
    return ret;
}

static int spi_nor_erase_by_type(struct channel_info *chan_info, enum erase_type etype, u32 addr)
{
    struct spi_nor *nor = &g_spi_nor_device[chan_info->channel_id];
    int ret;
    u8 erase_opcode;
    u32 timeout;

    nor->channel_id = chan_info->channel_id;
    ret = spi_nor_prep_and_lock_erase(nor, addr, etype);
    if (ret) {
        goto lock_err;
    }
    ret = spi_nor_write_enable(nor);
    if (ret) {
        goto erase_err;
    }

    if (etype == ERASE_CHIP) {
        struct spi_mem_op op = SPI_NOR_DIE_ERASE_OP(SPINOR_OP_CHIP_ERASE, 0, 0, 0);
        spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
        ret = qspi_exec_mem_op(nor, &op);
        if (ret) {
            goto erase_err;
        }
        timeout = DEFAULT_READY_WAIT_SECONDS;
    } else {
        if (etype == ERASE_SECTOR) {
            if (nor->addr_nbytes == 4) {
                erase_opcode = SPINOR_OP_BE_4K_4B;
            } else {
                erase_opcode = SPINOR_OP_BE_4K;
            }
            timeout = DEFAULT_READY_WAIT_FOR_SECTOR_ERASE_SECONDS;
        } else {
            if (nor->addr_nbytes == 4) {
                erase_opcode = SPINOR_OP_SE_4B;
            } else {
                erase_opcode = SPINOR_OP_SE;
            }
            timeout = DEFAULT_READY_WAIT_FOR_BLOCK_ERASE_SECONDS;
        }
        struct spi_mem_op op  = SPI_NOR_SECTOR_ERASE_OP(erase_opcode, nor->addr_nbytes, addr);
        spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
        ret = qspi_exec_mem_op(nor, &op);
        if (ret) {
            goto erase_err;
        }
    }

    ret = spi_nor_wait_till_ready_with_timeout(nor, timeout);
    if (ret) {
        goto erase_err;
    }
    spi_nor_unlock_and_unprep_erase(nor, addr, etype);
    return ret;

erase_err:
lock_err:
    spi_nor_dev_reg_capture(nor);
    spi_nor_unlock_and_unprep_erase(nor, addr, etype);
    return ret;
}

int spi_dev_probe(struct channel_info *chan_info, struct spi_dev_info *dev_info)
{
   int ret = 0;

   if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT || dev_info == NULL) {
       ret = -EINVAL;
       goto exit;
   }
   ret = spi_nor_probe(chan_info, dev_info);

exit:
   return ret;
}

static int spi_nor_wait_idle(struct channel_info *chan_info, uint32_t seconds)
{
    struct spi_nor *nor = &g_spi_nor_device[chan_info->channel_id];
    int ret;

    if ((nor->cqspi == NULL) || ((nor->cqspi->is_decoded_cs == false) && (chan_info->channel_id >= CQSPI_MAX_CHIPSELECT_NO_DECODED))) {
        ret = -ENODEV;
        goto params_err;
    }
    nor->channel_id = chan_info->channel_id;

    if (seconds == 0) {
        seconds = DEFAULT_READY_WAIT_SECONDS;
    }
    ret = spi_nor_prep_and_lock_rd(nor);
    if (ret) {
        goto lock_err;
    }
    ret = spi_nor_wait_till_ready_with_timeout(nor, seconds);
    spi_nor_unlock_and_unprep_rd(nor);
lock_err:
params_err:
    return ret;
}

int spi_dev_wait_idle(struct channel_info *chan_info, uint32_t seconds)
{
   int ret = 0;

   if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT) {
       ret = -EINVAL;
       goto exit;
   }
   if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
       ret = -EINVAL;
       goto exit;
   }
   ret = spi_nor_wait_idle(chan_info, seconds);

exit:
   return ret;
}

int flash_read(struct channel_info *chan_info, unsigned char *buffer,
        unsigned int len, unsigned int base_addr)
{
    int ret = 0;
    u32 retlen = 0;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT || buffer == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((g_spi_nor_device[chan_info->channel_id].info->type != DEV_FLASH)
            && (g_spi_nor_device[chan_info->channel_id].info->type != DEV_NONE)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((base_addr >= g_spi_nor_device[chan_info->channel_id].info->size)
            || ((base_addr + len - 1) >= g_spi_nor_device[chan_info->channel_id].info->size)
            /* || (base_addr & 0x3) */) {
        ret = -EINVAL;
        goto exit;
    }
    ret = spi_nor_read(chan_info, base_addr, len, &retlen, buffer);

exit:
    return ret;
}

int flash_write(struct channel_info *chan_info, unsigned char *buffer,
        unsigned int len, unsigned int base_addr)
{

    int ret = 0;
    u32 retlen = 0;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT || buffer == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((g_spi_nor_device[chan_info->channel_id].info->type != DEV_FLASH)
            && (g_spi_nor_device[chan_info->channel_id].info->type != DEV_NONE)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((base_addr >= g_spi_nor_device[chan_info->channel_id].info->size)
            || ((base_addr + len - 1) >= g_spi_nor_device[chan_info->channel_id].info->size)
            /* || (base_addr & 0x3) */) {
        ret = -EINVAL;
        goto exit;
    }
    ret = spi_nor_write(chan_info, base_addr, len, &retlen, buffer);

exit:
    return ret;
}

int flash_erase_by_type(struct channel_info *chan_info, enum erase_type etype, unsigned int base_addr)
{
    int ret = 0;
    unsigned int convert_addr;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT
            || etype <= ERASE_NONE || etype >= ERASE_BUTT) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((g_spi_nor_device[chan_info->channel_id].info->type != DEV_FLASH)
            && (g_spi_nor_device[chan_info->channel_id].info->type != DEV_NONE)) {
        ret = -EINVAL;
        goto exit;
    }
    if (base_addr >= g_spi_nor_device[chan_info->channel_id].info->size) {
        ret = -EINVAL;
        goto exit;
    }
    if (etype == ERASE_SECTOR) {
        convert_addr = (base_addr & SECTOR_ADDR_MSK);
    } else if (etype == ERASE_BLOCK) {
        convert_addr = (base_addr & BLOCK_ADDR_MSK);
    } else {
        convert_addr = base_addr;
    }
    ret = spi_nor_erase_by_type(chan_info, etype, convert_addr);

exit:
    return ret;
}

int flash_erase(struct channel_info *chan_info, unsigned int offset, unsigned int len, bool preserve_data)
{
    int ret = 0;
    const unsigned int sector_size = 4096;  // 4KB sector size
    unsigned int current_offset;
    unsigned int end_offset;
    unsigned char *head_buffer = NULL;
    unsigned char *tail_buffer = NULL;
    unsigned int head_sector_start = 0;
    unsigned int tail_sector_start = 0;

    // Parameter validation
    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((g_spi_nor_device[chan_info->channel_id].info->type != DEV_FLASH)
            && (g_spi_nor_device[chan_info->channel_id].info->type != DEV_NONE)) {
        ret = -EINVAL;
        goto exit;
    }
    if (len == 0) {
        ret = 0;  // Nothing to erase
        goto exit;
    }
    if ((offset >= g_spi_nor_device[chan_info->channel_id].info->size)
            || ((offset + len - 1) >= g_spi_nor_device[chan_info->channel_id].info->size)) {
        ret = -EINVAL;
        goto exit;
    }

    // Calculate aligned erase range
    // Start from sector-aligned address (round down)
    current_offset = offset & SECTOR_ADDR_MSK;
    // End at sector-aligned address (round up)
    end_offset = (offset + len + sector_size - 1) & SECTOR_ADDR_MSK;

    if (preserve_data) {
        head_sector_start = current_offset;
        tail_sector_start = end_offset - sector_size;

        // just have one sector to erase and have data to retain.
        if (head_sector_start == tail_sector_start && len < sector_size) {
            head_buffer = (unsigned char *)malloc(sector_size);
            if (head_buffer == NULL) {
                ret = -ENOMEM;
                goto exit;
            }
        }
        // have more than one sector.
        else {
            // if it has data at the beginning of the first sector, we must malloc buffer for the first sector.
            if (offset > head_sector_start) {
                head_buffer = (unsigned char *)malloc(sector_size);
                if (head_buffer == NULL) {
                    ret = -ENOMEM;
                    goto exit;
                }
            }

            // if it has data at the end of the last sector, we must malloc buffer for the last sector.
            if ((offset + len) < end_offset) {
                tail_buffer = (unsigned char *)malloc(sector_size);
                if (tail_buffer == NULL) {
                    ret = -ENOMEM;
                    goto cleanup;
                }
            }
        }

        // in the first sector, data should be keep may exsit at both side of the sector.
        if (head_buffer != NULL) {
            ret = flash_read(chan_info, head_buffer, sector_size, head_sector_start);
            if (ret != 0) {
                goto cleanup;
            }
            U32 start_offset = offset & (sector_size - 1);
            memset(&head_buffer[start_offset], 0xFF,
                   (start_offset + len) >= sector_size ? (sector_size - start_offset) : len);
        }

        // in the last sector, data should be keep must only exsit at the end of the last sector.
        if (tail_buffer != NULL) {
            ret = flash_read(chan_info, tail_buffer, sector_size, tail_sector_start);
            if (ret != 0) {
                goto cleanup;
            }
            memset(tail_buffer, 0xFF, (offset + len) & (sector_size - 1));
        }
    }

    // Erase sectors one by one
    while (current_offset < end_offset) {
        ret = flash_erase_by_type(chan_info, ERASE_SECTOR, current_offset);
        if (ret != 0) {
            goto cleanup;
        }
        current_offset += sector_size;
    }

    if (preserve_data) {
        // Write back the first sector if we have backup data
        if (head_buffer != NULL) {
            ret = flash_write(chan_info, head_buffer, sector_size, head_sector_start);
            if (ret != 0) {
                goto cleanup;
            }
        }

        // Write back the last sector if we have backup data and it's different from first
        if (tail_buffer != NULL) {
            ret = flash_write(chan_info, tail_buffer, sector_size, tail_sector_start);
            if (ret != 0) {
                goto cleanup;
            }
        }
    }

cleanup:
    if (head_buffer != NULL) {
        free(head_buffer);
    }
    if (tail_buffer != NULL) {
        free(tail_buffer);
    }

exit:
    return ret;
}

int nvsram_read(struct channel_info *chan_info, unsigned char *buffer,
        unsigned int len, unsigned int base_addr)
{
    int ret = 0;
    u32 retlen = 0;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT || buffer == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((g_spi_nor_device[chan_info->channel_id].info->type != DEV_NVSRAM)
            && (g_spi_nor_device[chan_info->channel_id].info->type != DEV_NONE)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((base_addr >= g_spi_nor_device[chan_info->channel_id].info->size)
            || ((base_addr + len - 1) >= g_spi_nor_device[chan_info->channel_id].info->size)
            || (base_addr & 0x3)) {
        ret = -EINVAL;
        goto exit;
    }
    ret = spi_nor_read(chan_info, base_addr, len, &retlen, buffer);

exit:
    return ret;
}

static int nvsram_write_para_chk(struct channel_info *chan_info, unsigned char *buffer,
        unsigned int len, unsigned int base_addr)
{
    int ret = 0;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT || buffer == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if (g_spi_nor_device[chan_info->channel_id].info == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((g_spi_nor_device[chan_info->channel_id].info->type != DEV_NVSRAM)
            && (g_spi_nor_device[chan_info->channel_id].info->type != DEV_NONE)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((base_addr >= g_spi_nor_device[chan_info->channel_id].info->size)
            || ((base_addr + len - 1) >= g_spi_nor_device[chan_info->channel_id].info->size)
            || (base_addr & 0x3)) {
        ret = -EINVAL;
        goto exit;
    }
exit:
    return ret;
}

int nvsram_write(struct channel_info *chan_info, unsigned char *buffer,
        unsigned int len, unsigned int base_addr)
{

    int ret = 0;
    u32 retlen = 0;

    ret = nvsram_write_para_chk(chan_info, buffer, len, base_addr);
    if (ret != 0) {
        goto exit;
    }
    ret = spi_nvsram_write(chan_info, base_addr, len, &retlen, buffer, false);

exit:
    return ret;
}


int nvsram_write_try(struct channel_info *chan_info, unsigned char *buffer,
        unsigned int len, unsigned int base_addr)
{

    int ret = 0;
    u32 retlen = 0;

    ret = nvsram_write_para_chk(chan_info, buffer, len, base_addr);
    if (ret != 0) {
        goto exit;
    }
    ret = spi_nvsram_write(chan_info, base_addr, len, &retlen, buffer, true);

exit:
    return ret;
}

int spi_dev_remove(struct channel_info *chan_info)
{
    int ret = 0;
    struct spi_nor *nor = NULL;

    if (chan_info == NULL || chan_info->channel_id >= CQSPI_MAX_CHIPSELECT) {
        ret = -EINVAL;
        goto exit;
    }

    nor = &g_spi_nor_device[chan_info->channel_id];
    if (nor->cqspi == NULL) {
        ret = 0;
        goto exit;
    }

    rtems_mutex_destroy(&nor->lock);
    nor->cqspi = NULL;
    nor->info = NULL;
    memset(nor->id, 0, SPI_NOR_MAX_ID_LEN);

exit:
    return ret;
}
