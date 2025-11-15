/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_barrier.h
 * @author  lichenxiang
 * @date    20201228
 * @brief   对外公共函数接口
 * @note    NA
 */

#ifndef __OSP_BARRIER_H
#define __OSP_BARRIER_H

#ifdef __cplusplus
extern "C" {
#endif

#define OSP_ISB      asm volatile("isb" : : : "memory")
#define OSP_DMB(opt) asm volatile("dmb " #opt : : : "memory")
#define OSP_DSB(opt) asm volatile("dsb " #opt : : : "memory")

#define OSP_MB( )  OSP_DSB(sy)
#define OSP_RMB( ) OSP_DSB(sy)
#define OSP_WMB( ) OSP_DSB(st)

#define OSP_DMA_RMB( ) OSP_DMB(osh)
#define OSP_DMA_WMB( ) OSP_DMB(oshst)

#define OSP_SMP_MB( )  OSP_DMB(ish)
#define OSP_SMP_RMB( ) OSP_DMB(ish)
#define OSP_SMP_WMB( ) OSP_DMB(ishst)

#define OSP_COMPILER_MEMORY_BARRIER( ) asm volatile("" ::: "memory")

#ifdef __cplusplus
}
#endif

#endif
