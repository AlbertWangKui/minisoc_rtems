/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_cpuopts.h
 * @author  lichenxiang
 * @date    2021.03.02
 * @brief   os inner config define
 * @note    NA
 */

#ifndef __OSP_INNER_CPUSTDATOMIC_H
#define __OSP_INNER_CPUSTDATOMIC_H

#ifndef __OSP_CPUOPTS_H
#error "include osp_cpuopts.h instead include this header file"
#endif

//#include <rtems/score/cpustdatomic.h>
#include <inner/osp_inner_cpuopts.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    int counter;
} OSPAtomic_t;
typedef struct {
    long long counter;
} OSPAtomic64_t;

typedef OSPAtomic_t atomic_t;
typedef OSPAtomic64_t atomic64_t;

#define OSP_LIKELY   RTEMS_PREDICT_TRUE
#define OSP_UNLIKELY RTEMS_PREDICT_FALSE

#define ATOMIC_OP(op, c_op, asm_op)                                                                                    \
    static __inline void atomic_##op(int i, atomic_t *v)                                                               \
    {                                                                                                                  \
        unsigned long tmp;                                                                                             \
        int result;                                                                                                    \
        ARM_SWITCH_REGISTERS;                                                                                          \
                                                                                                                       \
        __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ atomic_" #op "\n"                                                    \
                                               "pld %a3\n"                                                             \
                                               "1:	ldrex	%0, [%3]\n"                                                   \
                                               "	" #asm_op "	%0, %0, %4\n"                                          \
                                               "	strex	%1, %0, [%3]\n"                                                 \
                                               "	teq	%1, #0\n"                                                         \
                                               "	bne	1b\n" ARM_SWITCH_BACK                                          \
                             : "=&r"(result), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT               \
                             : "r"(&v->counter), "Ir"(i)                                                               \
                             : "cc");                                                                                  \
    }

#define ATOMIC_OP_RETURN(op, c_op, asm_op)                                                                             \
    static __inline int atomic_##op##_return_relaxed(int i, atomic_t *v)                                               \
    {                                                                                                                  \
        unsigned long tmp;                                                                                             \
        int result;                                                                                                    \
        ARM_SWITCH_REGISTERS;                                                                                          \
                                                                                                                       \
        __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ atomic_" #op "_return\n"                                             \
                                               "pld %a3\n"                                                             \
                                               "1:	ldrex	%0, [%3]\n"                                                   \
                                               "	" #asm_op "	%0, %0, %4\n"                                          \
                                               "	strex	%1, %0, [%3]\n"                                                 \
                                               "	teq	%1, #0\n"                                                         \
                                               "	bne	1b\n" ARM_SWITCH_BACK                                          \
                             : "=&r"(result), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT               \
                             : "r"(&v->counter), "Ir"(i)                                                               \
                             : "cc");                                                                                  \
                                                                                                                       \
        return result;                                                                                                 \
    }

#define ATOMIC_FETCH_OP(op, c_op, asm_op)                                                                              \
    static __inline int atomic_fetch_##op##_relaxed(int i, atomic_t *v)                                                \
    {                                                                                                                  \
        unsigned long tmp;                                                                                             \
        int result, val;                                                                                               \
        ARM_SWITCH_REGISTERS;                                                                                          \
                                                                                                                       \
        __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ atomic_fetch_" #op "\n"                                              \
                                               "pld %a4\n"                                                             \
                                               "1:	ldrex	%0, [%4]\n"                                                   \
                                               "	" #asm_op "	%1, %0, %5\n"                                          \
                                               "	strex	%2, %1, [%4]\n"                                                 \
                                               "	teq	%2, #0\n"                                                         \
                                               "	bne	1b\n" ARM_SWITCH_BACK                                          \
                             : "=&r"(result), "=&r"(val), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT   \
                             : "r"(&v->counter), "Ir"(i)                                                               \
                             : "cc");                                                                                  \
                                                                                                                       \
        return result;                                                                                                 \
    }

#define ATOMIC_OPS(op, c_op, asm_op)                                                                                   \
    ATOMIC_OP(op, c_op, asm_op)                                                                                        \
    ATOMIC_OP_RETURN(op, c_op, asm_op)                                                                                 \
    ATOMIC_FETCH_OP(op, c_op, asm_op)

ATOMIC_OPS(add, +=, add)
ATOMIC_OPS(sub, -=, sub)

#undef ATOMIC_OPS
#define ATOMIC_OPS(op, c_op, asm_op)                                                                                   \
    ATOMIC_OP(op, c_op, asm_op)                                                                                        \
    ATOMIC_FETCH_OP(op, c_op, asm_op)

ATOMIC_OPS(and, &=, and)
ATOMIC_OPS(andnot, &= ~, bic)
ATOMIC_OPS(or, |=, orr)
ATOMIC_OPS(xor, ^=, eor)

#undef ATOMIC_OPS
#undef ATOMIC_FETCH_OP
#undef ATOMIC_OP_RETURN
#undef ATOMIC_OP

#define ATOMIC64_OP(op, op1, op2)                                                                                      \
    static inline void atomic64_##op(long long i, atomic64_t *v)                                                       \
    {                                                                                                                  \
        long long result;                                                                                              \
        unsigned long tmp;                                                                                             \
        ARM_SWITCH_REGISTERS;                                                                                          \
                                                                                                                       \
        __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ atomic64_" #op "\n"                                                  \
                                               "pld %a3\n"                                                             \
                                               "1:	ldrexd	%0, %H0, [%3]\n"                                             \
                                               "	" #op1 " %Q0, %Q0, %Q4\n"                                          \
                                               "	" #op2 " %R0, %R0, %R4\n"                                          \
                                               "	strexd	%1, %0, %H0, [%3]\n"                                           \
                                               "	teq	%1, #0\n"                                                         \
                                               "	bne	1b\n" ARM_SWITCH_BACK                                          \
                             : "=&r"(result), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT               \
                             : "r"(&v->counter), "r"(i)                                                                \
                             : "cc");                                                                                  \
    }

#define ATOMIC64_OP_RETURN(op, op1, op2)                                                                               \
    static inline long long atomic64_##op##_return_relaxed(long long i, atomic64_t *v)                                 \
    {                                                                                                                  \
        long long result;                                                                                              \
        unsigned long tmp;                                                                                             \
        ARM_SWITCH_REGISTERS;                                                                                          \
                                                                                                                       \
        __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ atomic64_" #op "_return\n"                                           \
                                               "pld %a3\n"                                                             \
                                               "1:	ldrexd	%0, %H0, [%3]\n"                                             \
                                               "	" #op1 " %Q0, %Q0, %Q4\n"                                          \
                                               "	" #op2 " %R0, %R0, %R4\n"                                          \
                                               "	strexd	%1, %0, %H0, [%3]\n"                                           \
                                               "	teq	%1, #0\n"                                                         \
                                               "	bne	1b\n" ARM_SWITCH_BACK                                          \
                             : "=&r"(result), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT               \
                             : "r"(&v->counter), "r"(i)                                                                \
                             : "cc");                                                                                  \
                                                                                                                       \
        return result;                                                                                                 \
    }

#define ATOMIC64_FETCH_OP(op, op1, op2)                                                                                \
    static inline long long atomic64_fetch_##op##_relaxed(long long i, atomic64_t *v)                                  \
    {                                                                                                                  \
        long long result, val;                                                                                         \
        unsigned long tmp;                                                                                             \
        ARM_SWITCH_REGISTERS;                                                                                          \
                                                                                                                       \
        __asm__ __volatile__(ARM_SWITCH_TO_ARM "@ atomic64_fetch_" #op "\n"                                            \
                                               "pld %a4\n"                                                             \
                                               "1:	ldrexd	%0, %H0, [%4]\n"                                             \
                                               "	" #op1 " %Q1, %Q0, %Q5\n"                                          \
                                               "	" #op2 " %R1, %R0, %R5\n"                                          \
                                               "	strexd	%2, %1, %H1, [%4]\n"                                           \
                                               "	teq	%2, #0\n"                                                         \
                                               "	bne	1b\n" ARM_SWITCH_BACK                                          \
                             : "=&r"(result), "=&r"(val), "=&r"(tmp), "+Qo"(v->counter) ARM_SWITCH_ADDITIONAL_OUTPUT   \
                             : "r"(&v->counter), "r"(i)                                                                \
                             : "cc");                                                                                  \
                                                                                                                       \
        return result;                                                                                                 \
    }

#define ATOMIC64_OPS(op, op1, op2)                                                                                     \
    ATOMIC64_OP(op, op1, op2)                                                                                          \
    ATOMIC64_OP_RETURN(op, op1, op2)                                                                                   \
    ATOMIC64_FETCH_OP(op, op1, op2)

ATOMIC64_OPS(add, adds, adc)
ATOMIC64_OPS(sub, subs, sbc)

#undef ATOMIC64_OPS
#define ATOMIC64_OPS(op, op1, op2)                                                                                     \
    ATOMIC64_OP(op, op1, op2)                                                                                          \
    ATOMIC64_FETCH_OP(op, op1, op2)

ATOMIC64_OPS(and, and, and)
ATOMIC64_OPS(andnot, bic, bic)
ATOMIC64_OPS(or, orr, orr)
ATOMIC64_OPS(xor, eor, eor)

#undef ATOMIC64_OPS
#undef ATOMIC64_FETCH_OP
#undef ATOMIC64_OP_RETURN
#undef ATOMIC64_OP

#define ACCESS_ONCE(x) (*(volatile typeof(x) *)&(x))

#ifdef __cplusplus
}
#endif

#endif
