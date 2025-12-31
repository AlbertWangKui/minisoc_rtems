/**
 * copyright (C), 2020, WuXi Stars Micro System Technologies Co.,Ltd.
 *
 * @file common_defines.h
 * @author all
 * @date
 * @brief 常用宏定义
 * @note none
 * @version v1.0
 */

#ifndef __COMMON_DEFINES_H__
#define __COMMON_DEFINES_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>

/* define types according to the new code specification */
typedef unsigned char       U8;
typedef unsigned short      U16;
typedef unsigned int        U32;
typedef unsigned long long  U64;
typedef char                S8;
typedef short               S16;
typedef int                 S32;
typedef long long           S64;
typedef unsigned char       Bool;
typedef unsigned long       ULong;
typedef long                Long;

#define PLATFORM_KHZ  1000
#define PLATFORM_MHZ  1000000
#define PLATFORM_GHZ  1000000000
#define PLATFORM_KB   0x0400
#define PLATFORM_MB   0x00100000
#define PLATFORM_GB   0x40000000
#define PLATFORM_TB   0x0000010000000000UL
#define PLATFORM_PB   0x0004000000000000UL
#define PLATFORM_EB   0x1000000000000000UL

//< 大小端判断
#define IS_LITTLE_ENDIAN (*(U8*)&(U32){1} == 1)

//< 寄存器操作宏
#define reg8Read(addr)              (*(volatile U8*)(addr))
#define reg16Read(addr)             (*(volatile U16*)(addr))
#define reg32Read(addr)             (*(volatile U32*)(addr))
#define reg64Read(addr)             (*(volatile U64*)(addr))
#define reg8Write(addr,data)        *(volatile U8*)(addr)  = (data)
#define reg16Write(addr,data)       *(volatile U16*)(addr) = (data)
#define reg32Write(addr,data)       *(volatile U32*)(addr) = (data)
#define reg64Write(addr,data)       *(volatile U64*)(addr) = (data)

//< 计算数组元素的个数
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#endif

//< 基于基础宏效率考虑，宏参数的有效性不检查，使用者保证, 否则可能影响这种基础宏的效率
//< 位域操作宏: 对数据(值)
#define BIT_IS_ZERO(data,bit)           (((data) & (((typeof(data))1) << (bit))) == 0)
#define BIT_IS_ONE(data,bit)            (((data) & (((typeof(data))1) << (bit))) != 0)
#define CLR_BIT(data,bit)               ((data) &= ~((typeof(data))1 << (bit)))
#define SET_BIT(data,bit)               ((data) |= ((typeof(data))1 << (bit)))
#define FLIP_BIT(data, bit)             ((data) ^= ((typeof(data))1 << (bit)))
#define CLR_BITS(data, mask)            ((data) &= ~((typeof(data))(mask)))
#define SET_BITS(data, mask)            ((data) |= ((typeof(data))(mask)))
#define FLIP_BITS(data, mask)           ((data) ^= ((typeof(data))(mask)))

//< 位域操作宏: 对地址
#define SET_REG_BIT(addr, pos)          ((*(volatile U32*)addr) |=(0x00000001UL << (pos)))
#define CLR_REG_BIT(addr, pos)          ((*(volatile U32*)addr) &=(~(0x00000001UL << (pos))))
#define GET_REG_BIT(addr, pos)          (((*(volatile U32*)addr) & (1 << (pos))) >> pos)

#define GET_BIT_FIELD(data, offset, len) (((typeof(data))(data) >> (offset)) & \
                                ((typeof(data))((typeof(data))(1 << (len)) - 1)))
//< 字符串操作
#define STR1(str) #str
#define STR2(str) STR1(str)

//< 对齐判断
#define IS_POWER_OF_2(n)  (((n) != 0) && (((n) & ((n) - 1)) == 0))
#define IS_ALIGN(val, n)  (((val) & ((n) - 1)) == 0)

//< 向上或向下对齐
#define ALIGN_UP(val, n) (((val)+(n)-1)&(~((n)-1)))
#define ALIGN_DOWN(val, n) ((val)&(~((n)-1)))

//< 大小端反转，直接修改值本身
#define ENDIAN_SWAP(var) ({ \
    typeof(var) _##var = (var); \
    if (sizeof(_##var) == 1) { \
        var = _##var; \
    } else if (sizeof(_##var) == 2) { \
        var = ((_##var & 0xFF00) >> 8) | ((_##var & 0x00FF) << 8); \
    } else if (sizeof(_##var) == 4) { \
        var = \
        ((_##var & 0xFF000000) >> 24) | \
        ((_##var & 0x00FF0000) >> 8)  | \
        ((_##var & 0x0000FF00) << 8)  | \
        ((_##var & 0x000000FF) << 24); \
    } else if (sizeof(_##var) == 8) { \
        var = \
        ((_##var & 0xFF00000000000000ULL) >> 56) | \
        ((_##var & 0x00FF000000000000ULL) >> 40) | \
        ((_##var & 0x0000FF0000000000ULL) >> 24) | \
        ((_##var & 0x000000FF00000000ULL) >> 8)  | \
        ((_##var & 0x00000000FF000000ULL) << 8)  | \
        ((_##var & 0x0000000000FF0000ULL) << 24) | \
        ((_##var & 0x000000000000FF00ULL) << 40) | \
        ((_##var & 0x00000000000000FFULL) << 56); \
    } else { \
        var = _##var; \
    } \
})

#define ENDIAN_SWAP16(x) ( \
    (((U16)(x) & 0x00FFU) << 8) | \
    (((U16)(x) & 0xFF00U) >> 8) )

#define ENDIAN_SWAP32(x) ( \
    (((U32)(x) & 0x000000FFUL) << 24) | \
    (((U32)(x) & 0x0000FF00UL) << 8)  | \
    (((U32)(x) & 0x00FF0000UL) >> 8)  | \
    (((U32)(x) & 0xFF000000UL) >> 24) )

#define ENDIAN_SWAP64(x) ( \
    (((U64)(x) & 0x00000000000000FFULL) << 56) | \
    (((U64)(x) & 0x000000000000FF00ULL) << 40) | \
    (((U64)(x) & 0x0000000000FF0000ULL) << 24) | \
    (((U64)(x) & 0x00000000FF000000ULL) << 8)  | \
    (((U64)(x) & 0x000000FF00000000ULL) >> 8)  | \
    (((U64)(x) & 0x0000FF0000000000ULL) >> 24) | \
    (((U64)(x) & 0x00FF000000000000ULL) >> 40) | \
    (((U64)(x) & 0xFF00000000000000ULL) >> 56) )

#define CONCAT_U8_TO_U16(high, low)  (((U16)(high) <<  8) | (low))
#define CONCAT_U16_TO_U32(high, low) (((U32)(high) << 16) | (low))
#define CONCAT_U32_TO_U64(high, low) (((U64)(high) << 32) | (low))

static inline void dwMemCpy(volatile U32 *dest, volatile U32 *src,U32 nr_dwords)
{
    U32 m;

    if ((dest != NULL) && (src != NULL)) {
        for (m = 0; m < nr_dwords; m++) {
            dest[m] = src[m];
        }
    }
}

#if defined(__GNUC__)
  #define UNUSED_FUNC __attribute__((unused))
#else
  #define UNUSED_FUNC
#endif

#endif


