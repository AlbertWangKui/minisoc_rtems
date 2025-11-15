/**
 * copyright (C), 2022, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_r5_op.c
 * @author taohb@starsmicrosystem.com
 * @date 2023-07-27
 * @brief rtems内核bsp接口重载bsp_r5_op
 * @note none
 * @version v1.0
 */

#include <bsp_common.h>
#include "common_defines.h"
#include "bsp_api.h"

/**
 * @brief dcache 使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheEnable(void)
{
    asm volatile (
        "MRC p15, 0, r0, c1, c0, 0;"
        "ORR r0, r0, #0x04;"
        "DSB;"
        "MCR p15, 0, r0, c1, c0, 0;"
        "ISB;"
        :
        :
        :"r0"
        );
}

/**
 * @brief icache 使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void icacheEnable(void)
{
    asm volatile (
        "MRC p15, 0, r0, c1, c0, 0;"
        "ORR r0, r0, #0x1000;"
        "DSB;"
        "MCR p15, 0, r0, c1, c0, 0;"
        "ISB;"
        :
        :
        :"r0"
        );
}

/**
 * @brief dcache 去使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheDisable(void)
{
    asm volatile (
        "MRC p15, 0, r0, c1, c0, 0;"
        "BIC r0, r0, #0x04;"
        "DSB;"
        "MCR p15, 0, r0, c1, c0, 0;"
        "ISB;"
        :
        :
        :"r0"
        );
}

/**
 * @brief icache 去使能
 * @param [in] none
 * @param [out] none
 * @return none
 */
void icacheDisable(void)
{
    asm volatile (
        "MRC p15, 0, r0, c1, c0, 0;"
        "BIC r0, r0, #0x1000;"
        "DSB;"
        "MCR p15, 0, r0, c1, c0, 0;"
        "ISB;"
        :
        :
        :"r0"
        );
}

/**
 * @brief dcache 无效（全）
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheAllInvalid(void)
{
  asm volatile (
        "DSB;"
        "MOV r0, #0;"
        "MCR p15, 0, r0, c15, c5, 0;"
        :
        :
        :"r0"
        );
}

/**
 * @brief icache 无效（全）
 * @param [in] none
 * @param [out] none
 * @return none
 */
void icacheAllInvalid(void)
{
    asm volatile (
        "DSB;"
        "MOV r0, #0;"
        "MCR p15, 0, r0, c7, c5, 0;"
        :
        :
        :"r0"
        );
}

/**
 * @brief 获取 cache line size
 * @param [in] none
 * @param [out] none
 * @return cache line size
 */
static U32 cacheLineSizeGet(void)
{
    U32 csselr, ccsidr, cacheLineSize;

    csselr = (1 << 1) | (0 << 0); // Level=0b001（L1）, Indx=0, Type=0（Data）
    __asm__ volatile (
        "MCR p15, 2, %0, c0, c0, 0\n"
        "ISB\n"
        : : "r" (csselr)
        : "memory"
    );

    __asm__ volatile (
        "MRC p15, 1, %0, c0, c0, 0\n"
        : "=r" (ccsidr)
        :
        : "memory"
    );

    cacheLineSize = (ccsidr & 0x7) + 4;

    return (1 << cacheLineSize);
}

/**
 * @ brief 对指定地址进行dcache无效
 * @param [in] addr,待操作的地址
 * @param [in] size,待操作的大小
 * @param [out] none
 * @return none
 */
void dcacheInvalid(U32 addr, U32 size)
{
    U32 cacheLineSize = 0;
    U32 end = addr + size;
    U32 tmp;

    cacheLineSize = cacheLineSizeGet();
    // printf("cacheline: %d \r\n",cacheLineSize);
    tmp = addr & ~(cacheLineSize - 1);

    for (; tmp < end; tmp += cacheLineSize) {
        __asm__ volatile (
            "MCR p15, 0, %0, c7, c6, 1"
            :
            : "r" (tmp)
            : "memory"
        );
    }

    __asm__ volatile ("DSB");
    __asm__ volatile ("ISB");
}

/**
 * @ brief 对指定地址进行dcache同步
 * @param [in] addr,待操作的地址
 * @param [in] size,待操作的大小
 * @param [out] none
 * @return none
 */
void dcacheClean(U32 addr, U32 size)
{
    U32 cacheLine;
    U32 start = 0;
    U32 end = 0;
    U32 line;

    cacheLine = cacheLineSizeGet();
    start = addr & ~(cacheLine - 1);
    end = addr + size;

    for (line = start; line < end; line += cacheLine) {
        __asm__ volatile (
            "MCR p15, 0, %0, c7, c10, 1"
            :
            : "r" (line)
            : "memory"
        );
    }

    __asm__ volatile ("DSB");
    __asm__ volatile ("ISB");
}

/**
 @ brief 对指定地址进行dcache刷新
 * @param [in] addr,待操作的地址
 * @param [in] size,待操作的大小
 * @param [out] none
 * @return none
 */
void dcacheFlush(U32 addr, U32 size)
{
    U32 line;
    uint32_t cacheLineSize = cacheLineSizeGet();
    U32 start = addr;
    U32 end;

    start = ALIGN_DOWN(start,cacheLineSize);
    end = addr + size;
    end = ALIGN_UP(end,cacheLineSize);

    for (line = start; line < end; line += cacheLineSize) {
        __asm__ volatile (
            "MCR p15, 0, %0, c7, c14, 1"
            :
            : "r" (line)
            : "memory"
        );
    }

    __asm__ volatile ("DSB");
    __asm__ volatile ("ISB");
}

/**
 @ brief 全部dcache flush
 * @param [in] none
 * @param [out] none
 * @return none
 */
void dcacheAllFlush(void)
{
    U32 csidReg, c7Reg;
    U32 cz, lz, ws;
    U32 way, wayId, set, setId, sets;
    U32 currmask;

    __asm__ __volatile__("mrs    %0, cpsr\n" : "=r" (currmask));
    __asm__ __volatile__("msr  cpsr,%0\n" : : "r" (currmask | 0xc0));

    __asm__ __volatile__("mcr " "p15, 2, %0,  c0,  c0, 0" "\n" : : "r" (0));
    __asm__ __volatile__("mrc " "p15, 1, %0,  c0,  c0, 0" "\n" : "=r" (csidReg));
    cz = (csidReg >> 13U) & 0x000001FFU;
    cz += 0x00000001U;
    cz *= (U32)128;  

    ws = (csidReg & 0x000003ffU) >> 3U;
    ws += 0x00000001U;

    lz = (csidReg & 0x00000007U) + 0x00000004U;

    sets = cz / ws;
    sets /= (0x00000001U << lz);

    way = 0U;
    set = 0U;

    for (wayId = 0U; wayId < ws; wayId++)
    {
        for (setId = 0U; setId < sets; setId++)
        {
            c7Reg = way | set;
            __asm__ __volatile__("mcr " "p15, 0, %0,  c7, c14, 2" :: "r" (c7Reg));

            set += (0x00000001U << lz);
        }
        set = 0U;
        way += 0x40000000U;
    }

    asm volatile ("dsb");
    __asm__ __volatile__("msr  cpsr,%0\n" : : "r" (currmask));
    asm volatile ("dsb");
}

/**
 * @brief 关闭快速中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuFiqDisable(void)
{
    asm volatile (
        "MRS r0, cpsr ;"
        "ORR r0, r0, #0x40 ;"
        "MSR cpsr, r0 ;"
        :
        :
        :"r0"
        );
}

/**
 * @brief 打开快速中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuFiqEnable(void)
{
    asm volatile (
        "MRS r0, cpsr ;"
        "bic r0, r0, #0x40;"
        "MSR cpsr, r0 ;"
        :
        :
        :"r0"
        );
}

/**
 * @brief 关闭中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuIrqDisable(void)
{
    asm volatile ("CPSID i;" ::: "memory");
}

/**
 * @brief 打开中断
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpuIrqEnable(void)
{
    asm volatile ("CPSIE i;" ::: "memory");
}

/**
 * @brief 打开异常上报
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpsrAIFEnable(void)
{
    asm volatile ("CPSIE aif;" ::: "memory");
}

/**
 * @brief 关闭异常上报
 * @param [in] none
 * @param [out] none
 * @return none
 */
void cpsrAIFDisable(void)
{
    asm volatile ("CPSID aif;" ::: "memory");
}

/**
 * @brief MPU系统寄存器配置
 * @param [in] val,待配置的值
 * @param [out] none
 * @return none
 */
void mpuSystemCtrlRegSet(U32 val)
{
    asm volatile (
        "mcr p15, 0, %0, c1, c0, 0;"
        :
        :"r"(val)
        :
        );
    asm volatile ("isb");
}

/**
 * @brief 获取MPU系统寄存器
 * @param [in] none
 * @param [out] none
 * @return MPU系统寄存器值
 */
U32 mpuSystemCtrlRegGet(void)
{
    U32 ret;

    asm volatile (
        "mrc p15, 0, %0, c1, c0, 0;"
        :"=r"(ret)
        :
        :"memory"
        );
    asm volatile ("dsb");
    return ret;
}

/**
 * @brief MPU region 数配置
 * @param [in] n, region 数
 * @param [out] none
 * @return none
 */
void memRegionNumSet(U32 n)
{
    asm volatile (
        "mcr p15, 0, %0, c6, c2, 0;"
        :
        :"r"(n)
        :
        );
    asm volatile ("isb");
}

/**
 * @brief MPU region 基地址配置
 * @param [in] addr,region 基地址
 * @param [out] none
 * @return none
 */
void memRegionBaseSet(U32 addr)
{
    asm volatile (
        "mcr p15, 0, %0, c6, c1, 0;"
        :
        :"r"(addr)
        :
        );
    asm volatile ("isb");
}

/**
 * @brief MPU region 大小配置
 * @param [in] val,region 大小
 * @param [out] none
 * @return none
 */
void memRegionSizeSetAndEnable(U32 val)
{
    asm volatile (
        "mcr p15, 0, %0, c6, c1, 2;"
        :
        :"r"(val)
        :
        );
    asm volatile ("isb");
}

/**
 * @brief MPU region 访问属性配置
 * @param [in] val,region 访问属性
 * @param [out] none
 * @return none
 */
void memAccessAttributeSet(U32 val)
{
    asm volatile (
        "mcr p15, 0, r0, c6, c1, 4;"
        :
        :"r"(val)
        :
        );
    asm volatile ("isb");
}

