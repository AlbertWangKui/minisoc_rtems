/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_atomic.h
 * @author  lichenxiang
 * @date    2020.12.28
 * @brief   NA
 * @note    NA
 */

#ifndef __OSP_ATOMIC_H
#define __OSP_ATOMIC_H
#include "osp_interrupt.h"
#include <osp_cpuopts.h>
#include <osp_cpu.h>
#include <osp_barrier.h>
#include <inner/osp_inner_cpustdatomic.h>
#include <bspopts.h>

#ifdef PS3OS_NIC_100G
#include <stdatomic.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ospAtomicTestAndSetBit
 * @param   bit , 比特位
 * @param   p [out], 数据的指针，该数据的第 @bit 比特会被置位
 * @return  执行结果
 * @warning 无
 * @note    无
 */
#define OSP_BITS_PER_BYTE      8
#define OSP_BITS_PER_LONG      (sizeof(unsigned long) * OSP_BITS_PER_BYTE)
#define OSP_BITS_PER_LONG_LONG (sizeof(unsigned long long) * OSP_BITS_PER_BYTE)

#define OSP_BIT_MASK(nr) (1UL << ((nr) & (OSP_BITS_PER_LONG - 1)))
#define OSP_BIT_WORD(nr) ((nr) / OSP_BITS_PER_LONG)

#define OSP_BIT_MASK64(nr) (1ULL << ((nr) & (OSP_BITS_PER_LONG_LONG - 1)))
#define OSP_BIT_WORD64(nr) ((nr) / OSP_BITS_PER_LONG_LONG)

#define OSP_ATOMIC_INIT(atomicValue, value)                                                                            \
    do {                                                                                                               \
        ACCESS_ONCE((atomicValue)->counter) = (value);                                                                 \
    } while(0)

static __inline unsigned int ospClz(unsigned int x)
{
    unsigned int ret;
    ARM_SWITCH_REGISTERS;

    __asm__ volatile(ARM_SWITCH_TO_ARM "clz    %0, %1\n" ARM_SWITCH_BACK
            : "=r"(ret) ARM_SWITCH_ADDITIONAL_OUTPUT
            : "r"(x));

    return ret;
}

static __inline int ospFls(int x)
{
    return 32 - ospClz(x);
}

static __inline int ospFfs(int x)
{
    return ospFls(x & -x);
}

static __inline unsigned long _ospFfs(unsigned long x)
{
    return ospFfs(x) - 1;
}

#define OSP_FFZ(x) _ospFfs(~(x))

#if !defined(PS3OS_SMP) && !defined(PS3OS_NIC_100G)

static __inline int ospAtomicTestAndSetBit(unsigned int bit, volatile unsigned long *p)
{
    unsigned int res;
    unsigned long mask = (1UL << ((bit) & ((sizeof(unsigned long) * OSP_BITS_PER_BYTE) - 1)));

    p += ((bit) / (sizeof(unsigned long) * OSP_BITS_PER_BYTE));

    OspInterruptLevel_t level = ospInterruptDisable( );

    res = *p;
    *p  = res | mask;

    ospInterruptEnable(level);

    return (res & mask) != 0;
}

static __inline void _ospAtomicInc(OSPAtomic_t *valAtomic, int val)
{
    OspInterruptLevel_t level = ospInterruptDisable( );
    valAtomic->counter += val;
    ospInterruptEnable(level);
}

static __inline void ospAtomicInc(OSPAtomic_t *valAtomic)
{
    _ospAtomicInc(valAtomic, 1);
}

static __inline void ospAtomicSetBit(unsigned int bit, volatile unsigned long *p)
{

    unsigned int mask = OSP_BIT_MASK(bit);
    p += OSP_BIT_WORD(bit);

    OspInterruptLevel_t level = ospInterruptDisable( );

    *p |= mask;

    ospInterruptEnable(level);
}

static __inline void ospAtomicClearBit(unsigned int bit, volatile unsigned long *p)
{
    unsigned int mask = OSP_BIT_MASK(bit);
    p += OSP_BIT_WORD(bit);

    OspInterruptLevel_t level = ospInterruptDisable( );

    *p &= ~mask;

    ospInterruptEnable(level);
}

static __inline int ospAtomicTestAndClearBit(unsigned int bit, volatile unsigned long *p)
{

    unsigned int res;

    unsigned int mask = OSP_BIT_MASK(bit);
    p += OSP_BIT_WORD(bit);

    OspInterruptLevel_t level = ospInterruptDisable( );

    res = *p;
    *p  = res & ~mask;

    ospInterruptEnable(level);

    return (res & mask) != 0;
}

static __inline int ospAtomicFindLastBitSet(unsigned int v)
{
    ARM_SWITCH_REGISTERS;
    if(0 == v) {
        return -1;
    }

    __asm__ volatile(ARM_SWITCH_TO_ARM "clz    %1, %0\n" ARM_SWITCH_BACK
            : "=r"(v) ARM_SWITCH_ADDITIONAL_OUTPUT
            : "Ir"(v));
    return (sizeof(v) << 3) - v - 1;
}

static __inline int ospAtomicFindFirstBitSet(unsigned int v)
{
    ARM_SWITCH_REGISTERS;
    if(0 == v) {
        return -1;
    }

    __asm__ volatile(ARM_SWITCH_TO_ARM "rbit   %1, %0\n"
                                       "clz    %1, %0\n" ARM_SWITCH_BACK
                     : "=r"(v) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "r"(v));
    return v;
}

static __inline int ospAtomic64FindFirstBitSet(unsigned long long v)
{
    ARM_SWITCH_REGISTERS;
    if(0 == v) {
        return -1;
    }

    int ret = 0;

    __asm__ volatile(ARM_SWITCH_TO_ARM "ldrexd  r0, r1, [%1]\n"
                                       "cmp    r0, #0\n"
                                       "beq    1f\n"
                                       "rbit   %0, r0\n"
                                       "clz    %0, %0\n"
                                       "b 2f\n"
                                       "1: rbit    r1, r1\n"
                                       "clz    r0, r1\n"
                                       "add    %0, r0, #32\n"
                                       "2: " ARM_SWITCH_BACK
                     : "=r"(ret) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "r"(&v)
                     : "r0", "r1", "memory");
    return ret;
}

static __inline long long ospAtomic64Read(OSPAtomic64_t *valAtomic)
{
    return ACCESS_ONCE(valAtomic->counter);
}

static __inline void ospAtomic64Set(OSPAtomic64_t *valAtomic, long long val)
{

    OspInterruptLevel_t level = ospInterruptDisable( );

    ACCESS_ONCE(valAtomic->counter) = val;

    ospInterruptEnable(level);
}

static __inline void ospAtomic64SetBit(unsigned long long bit, volatile unsigned long long *p)
{
    unsigned long long mask = OSP_BIT_MASK64(bit);
    p += OSP_BIT_WORD64(bit);
    OspInterruptLevel_t level = ospInterruptDisable( );

    *p |= mask;
    ospInterruptEnable(level);
}

static __inline void ospAtomic64ClearBit(unsigned long long bit, volatile unsigned long long *p)
{
    unsigned long long mask = OSP_BIT_MASK64(bit);
    p += OSP_BIT_WORD64(bit);

    OspInterruptLevel_t level = ospInterruptDisable( );

    *p &= ~mask;
    ospInterruptEnable(level);
}

static __inline bool ospCmpAndSet(unsigned int *mem, unsigned int oldVal, unsigned int newVal)
{
    OspInterruptLevel_t level = ospInterruptDisable( );

    if(RTEMS_PREDICT_TRUE(*mem == oldVal)) {
        *mem = newVal;
        ospInterruptEnable(level);
        return true;
    }
    ospInterruptEnable(level);
    return false;
}

static __inline int ospAtomicRead(OSPAtomic_t *valAtomic)
{
    return ACCESS_ONCE(valAtomic->counter);
}

static __inline void ospAtomicSet(OSPAtomic_t *valAtomic, int val)
{
    OspInterruptLevel_t level       = ospInterruptDisable( );
    ACCESS_ONCE(valAtomic->counter) = val;
    ospInterruptEnable(level);
}

static __inline void _ospAtomicDec(OSPAtomic_t *valAtomic, int val)
{
    OspInterruptLevel_t level = ospInterruptDisable( );
    valAtomic->counter -= val;
    ospInterruptEnable(level);
}

static __inline void ospAtomicDec(OSPAtomic_t *valAtomic)
{
    _ospAtomicDec(valAtomic, 1);
}

static __inline int _ospAtomicIncReturn(OSPAtomic_t *valAtomic, int val)
{
    int ret;
    OSP_COMPILER_MEMORY_BARRIER( );
    OSP_SMP_MB( );

    OspInterruptLevel_t level = ospInterruptDisable( );

    valAtomic->counter += val;
    ret = ACCESS_ONCE(valAtomic->counter);
    ospInterruptEnable(level);

    OSP_COMPILER_MEMORY_BARRIER( );
    OSP_SMP_MB( );
    return ret;
}

static __inline int ospAtomicIncReturn(OSPAtomic_t *valAtomic)
{
    return _ospAtomicIncReturn(valAtomic, 1);
}

static __inline int _ospAtomicDecReturn(OSPAtomic_t *valAtomic, int val)
{
    int ret;
    OSP_COMPILER_MEMORY_BARRIER( );
    OSP_SMP_MB( );

    OspInterruptLevel_t level = ospInterruptDisable( );

    valAtomic->counter -= val;
    ret = ACCESS_ONCE(valAtomic->counter);

    ospInterruptEnable(level);

    OSP_COMPILER_MEMORY_BARRIER( );
    OSP_SMP_MB( );

    return ret;
}

static __inline int ospAtomicDecReturn(OSPAtomic_t *valAtomic)
{
    return _ospAtomicDecReturn(valAtomic, 1);
}

static __inline bool ospAtomicIncAndTest(OSPAtomic_t *valAtomic)
{

    return (_ospAtomicIncReturn(valAtomic, 1) == 0);
}

static __inline bool ospAtomicDecAndTest(OSPAtomic_t *valAtomic)
{

    return (_ospAtomicDecReturn(valAtomic, 1) == 0);
}

static __inline int ospAtomicCmpChg(OSPAtomic_t *v, int old, int new)
{
    int ret;

    OspInterruptLevel_t level = ospInterruptDisable( );

    ret = v->counter;

    if(RTEMS_PREDICT_TRUE(ret == old)) {
        v->counter = new;
    }
    ospInterruptEnable(level);
    return ret;
}

static __inline int ospAtomicFetchAddUnless(OSPAtomic_t *v, int a, int u)
{
    int oldval, newval;

    OSP_SMP_MB( );

    oldval = ospAtomicRead(v);
    while((oldval != u) && ((newval = ospAtomicCmpChg((v), oldval, oldval + a)) != oldval)) {
        oldval = newval;
    }

    OSP_SMP_MB( );

    return oldval;
}

static __inline bool ospAtomicAddUnless(OSPAtomic_t *valAtomic, int a, int u)
{
    return ospAtomicFetchAddUnless(valAtomic, a, u) != u;
}

static __inline bool ospAtomicIncNotZero(OSPAtomic_t *valAtomic)
{
    return ospAtomicAddUnless(valAtomic, 1, 0);
}

#else

static __inline int ospAtomicTestAndSetBit(unsigned int bit, volatile unsigned long *p)
{
    unsigned int res;
    unsigned long mask = (1UL << ((bit) & ((sizeof(unsigned long) * OSP_BITS_PER_BYTE) - 1)));
    ARM_SWITCH_REGISTERS;

    p += ((bit) / (sizeof(unsigned long) * OSP_BITS_PER_BYTE));

    __asm__ volatile(ARM_SWITCH_TO_ARM "1:  ldrex r2, [ %[p] ]\n"
                                       "ands %[res], r2, %[mask]\n"
                                       "orreq r2, r2, %[mask]\n"
                                       "strex r4, r2, [ %[p] ]\n"
                                       "cmp r4, #0\n"
                                       "bne 1b\n"
                                       "dmb\n"
                                       "cmp %[res], #0\n"
                                       "movne %[res], #1\n" ARM_SWITCH_BACK
                     : [ res ] "=&r"(res) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : [ p ] "r"(p), [ mask ] "r"(mask)
                     : "memory", "r2", "r4", "cc");

    return res;
}

static __inline void ospAtomicSetBit(unsigned int bit, volatile unsigned long *p)
{

    unsigned int mask = OSP_BIT_MASK(bit);
    p += OSP_BIT_WORD(bit);

    unsigned int tmp;
    unsigned int result;
    ARM_SWITCH_REGISTERS;

    __asm__ volatile(ARM_SWITCH_TO_ARM "1: ldrex   %0, %2\n"
                                       "   orr     %0, %0, %3\n"
                                       "   strex   %1, %0, %2\n"
                                       "   teq     %1, #0\n"
                                       "   bne     1b\n" ARM_SWITCH_BACK
                     : "=&r"(result), "=&r"(tmp), "+Q"(*p) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "Ir"(mask)
                     : "cc", "memory");
}

static __inline void ospAtomicClearBit(unsigned int bit, volatile unsigned long *p)
{
    unsigned int mask = OSP_BIT_MASK(bit);
    p += OSP_BIT_WORD(bit);

    unsigned int tmp;
    unsigned int result;
    ARM_SWITCH_REGISTERS;

    mask = ~mask;
    __asm__ volatile(ARM_SWITCH_TO_ARM "1: ldrex   %0, %2\n"
                                       "   and     %0, %0, %3\n"
                                       "   strex   %1, %0, %2\n"
                                       "   teq     %1, #0\n"
                                       "   bne     1b\n" ARM_SWITCH_BACK
                     : "=&r"(result), "=&r"(tmp), "+Q"(*p) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "Ir"(mask)
                     : "cc", "memory");
}

static __inline int ospAtomicTestAndClearBit(unsigned int bit, volatile unsigned long *p)
{
    unsigned int mask = OSP_BIT_MASK(bit);
    p += OSP_BIT_WORD(bit);
    unsigned int tmp;
    unsigned int result;
    unsigned int ret;
    unsigned old = *p;
    ARM_SWITCH_REGISTERS;

    mask = ~mask;

    __asm__ volatile(ARM_SWITCH_TO_ARM "1: ldrex   %0, %2\n"
                                       "   mov     %3, %0\n"
                                       "   and     %0, %0, %4\n"
                                       "   strex   %1, %0, [%5]\n"
                                       "   teq     %1, #0\n"
                                       "   bne 1b\n" ARM_SWITCH_BACK
                     : "=&r"(result), "=&r"(tmp), "+Q"(*p), "=&r"(ret) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "Ir"(mask), "Ir"(p)
                     : "cc", "memory");

    if(0 != (old & ~mask)) {
        return 1;
    } else {
        return 0;
    }
}

static __inline int ospAtomicFindLastBitSet(unsigned int v)
{
    ARM_SWITCH_REGISTERS;
    if(0 == v) {
        return -1;
    }

    __asm__ volatile(ARM_SWITCH_TO_ARM "clz    %1, %0\n" ARM_SWITCH_BACK
            : "=r"(v) ARM_SWITCH_ADDITIONAL_OUTPUT
            : "Ir"(v));
    return (sizeof(v) << 3) - v - 1;
}

static __inline int ospAtomicFindFirstBitSet(unsigned int v)
{
    ARM_SWITCH_REGISTERS;
    if(0 == v) {
        return -1;
    }

    __asm__ volatile(ARM_SWITCH_TO_ARM "rbit   %1, %0\n"
                                       "clz    %1, %0\n" ARM_SWITCH_BACK
                     : "=r"(v) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "r"(v));
    return v;
}

static __inline int ospAtomic64FindFirstBitSet(unsigned long long v)
{
    if(0 == v) {
        return -1;
    }

    int ret = 0;
    ARM_SWITCH_REGISTERS;

    __asm__ volatile(ARM_SWITCH_TO_ARM "ldrexd  r0, r1, [%1]\n"
                                       "cmp    r0, #0\n"
                                       "beq    1f\n"
                                       "rbit   %0, r0\n"
                                       "clz    %0, %0\n"
                                       "b 2f\n"
                                       "1: rbit    r1, r1\n"
                                       "clz    r0, r1\n"
                                       "add    %0, r0, #32\n"
                                       "2: " ARM_SWITCH_BACK
                     : "=r"(ret) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "r"(&v)
                     : "r0", "r1", "memory", "cc");
    return ret;
}

static __inline long long ospAtomic64Read(OSPAtomic64_t *valAtomic)
{
    long long result;
    ARM_SWITCH_REGISTERS;

    __asm__ volatile(ARM_SWITCH_TO_ARM "@ ospAtomic64Read\n"
                                       "	ldrexd	%0, %H0, [%1]\n" ARM_SWITCH_BACK
                     : "=&r"(result) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "r"(&valAtomic->counter), "Qo"(valAtomic->counter));
    return result;
}

static __inline void ospAtomic64Set(OSPAtomic64_t *valAtomic, long long val)
{
    long long tmp;
    ARM_SWITCH_REGISTERS;

    __asm__ volatile(ARM_SWITCH_TO_ARM "@ ospAtomic64Set\n"
                                       "pld %a2\n"
                                       "1:	ldrexd	%0, %H0, [%2]\n"
                                       "	strexd	%0, %3, %H3, [%2]\n"
                                       "	teq	%0, #0\n"
                                       "	bne	1b\n" ARM_SWITCH_BACK
                     : "=&r"(tmp), "=Qo"(valAtomic->counter) ARM_SWITCH_ADDITIONAL_OUTPUT
                     : "r"(&valAtomic->counter), "r"(val)
                     : "cc");
}

static __inline void ospAtomic64SetBit(unsigned long long bit, volatile unsigned long long *p)
{
    unsigned long long mask = OSP_BIT_MASK64(bit);
    p += OSP_BIT_WORD64(bit);
    atomic64_or(mask, (OSPAtomic64_t *)p);
}

static __inline void ospAtomic64ClearBit(unsigned long long bit, volatile unsigned long long *p)
{
    unsigned long long mask = OSP_BIT_MASK64(bit);
    p += OSP_BIT_WORD64(bit);

    mask = ~mask;

    atomic64_and(mask, (OSPAtomic64_t *)p);
}

///< 参考linux atomic_cmpxchg_relaxed，但是由于想和业务定义为一致的，和标准linux实现有差异（当前hba业务也没使用这套接口）
static __inline bool ospCmpAndSet(unsigned int *mem, unsigned int oldVal, unsigned int newVal)
{
	unsigned int oldval;
	unsigned long res;
    ARM_SWITCH_REGISTERS;

    do {
        __asm__ volatile(ARM_SWITCH_TO_ARM "@ atomic_cmpxchg\n"
                "pld %a3\n"
                "ldrex	%1, [%3]\n"
                "mov	%0, #0\n"
                "teq	%1, %4\n"
                "strexeq %0, %5, [%3]\n" ARM_SWITCH_BACK
                : "=&r" (res), "=&r" (oldval), "+Qo" (*mem) ARM_SWITCH_ADDITIONAL_OUTPUT
                : "r" (mem), "Ir" (oldVal), "r" (newVal)
                : "cc","memory");
    } while (res);

    return (oldval == oldVal);
}

static __inline int ospAtomicRead(OSPAtomic_t *valAtomic)
{
    return ACCESS_ONCE(valAtomic->counter);
}

static __inline void ospAtomicSet(OSPAtomic_t *valAtomic, int val)
{
    ACCESS_ONCE(valAtomic->counter) = val;
    // atomic_store_explicit(valAtomic, val, ATOMIC_ORDER_RELAXED);
}

static __inline void _ospAtomicInc(OSPAtomic_t *valAtomic, int val)
{
    atomic_add(val, valAtomic);
}

static __inline void ospAtomicInc(OSPAtomic_t *valAtomic)
{
    _ospAtomicInc(valAtomic, 1);
}

static __inline void _ospAtomicDec(OSPAtomic_t *valAtomic, int val)
{
    atomic_sub(val, valAtomic);
}

static __inline void ospAtomicDec(OSPAtomic_t *valAtomic)
{
    _ospAtomicDec(valAtomic, 1);
}

static __inline int _ospAtomicIncReturn(OSPAtomic_t *valAtomic, int val)
{

    int ret = atomic_add_return_relaxed(val, valAtomic);

    return ret;
}

static __inline int ospAtomicIncReturn(OSPAtomic_t *valAtomic)
{
    return _ospAtomicIncReturn(valAtomic, 1);
}

static __inline int _ospAtomicDecReturn(OSPAtomic_t *valAtomic, int val)
{
    int ret = atomic_sub_return_relaxed(val, valAtomic);
    return ret;
}

static __inline int ospAtomicDecReturn(OSPAtomic_t *valAtomic)
{
    return _ospAtomicDecReturn(valAtomic, 1);
}

static __inline bool ospAtomicIncAndTest(OSPAtomic_t *valAtomic)
{

    return (_ospAtomicIncReturn(valAtomic, 1) == 0);
}

static __inline bool ospAtomicDecAndTest(OSPAtomic_t *valAtomic)
{

    return (_ospAtomicDecReturn(valAtomic, 1) == 0);
}

static __inline int ospAtomicFetchAddUnless(OSPAtomic_t *v, int a, int u)
{
    int oldval, newval;
    unsigned long tmp;
    ARM_SWITCH_REGISTERS;

    OSP_SMP_MB( );

    __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ ospAtomicFetchAddUnless\n"
                         "1:	ldrex	%0, [%4]\n"
                         "	teq	%0, %5\n"
                         "	beq	2f\n"
                         "	add	%1, %0, %6\n"
                         "	strex	%2, %1, [%4]\n"
                         "	teq	%2, #0\n"
                         "	bne	1b\n"
                         "2:\n" ARM_SWITCH_BACK
                         : "=&r"(oldval), "=&r"(newval), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT
                         : "r"(&(v->counter)), "r"(u), "r"(a)
                         : "cc");

    if(oldval != u)
        OSP_SMP_MB( );

    return oldval;
}

static __inline bool ospAtomicAddUnless(OSPAtomic_t *valAtomic, int a, int u)
{
    return ospAtomicFetchAddUnless(valAtomic, a, u) != u;
}

static __inline bool ospAtomicIncNotZero(OSPAtomic_t *valAtomic)
{
    return ospAtomicAddUnless(valAtomic, 1, 0);
}

#endif

#ifdef __cplusplus
}
#endif
#endif
