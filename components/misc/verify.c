/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file verify.c
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/01
 * @brief 常见校验算法
 */

#include "common_defines.h"
#include "verify.h"

/* CRC16多项式：0x1021 (x^16 + x^12 + x^5 + 1) - CCITT标准 */
/**
 * @brief CRC16查找表（64字节版本，使用6位索引）
 */
 static const U16 crc16Table[64] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC
};

/**
 * @brief 计算CRC16校验值（使用64字节查找表，高效实现）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC16校验值
 */
U16 crc16(const U8 *data, U32 length)
{
    U16 crc = 0xFFFF;  /* CRC16初始值 */
    U32 i;

    if (data == NULL) {
        return 0;
    }

    for (i = 0; i < length; i++) {
        /* 使用6位索引，每次处理6位数据 */
        U8 index = ((crc >> 10) ^ (data[i] >> 2)) & 0x3F;
        crc = ((crc << 6) ^ crc16Table[index]) & 0xFFFF;
    }

    return crc;
}

/**
 * @brief 计算16位累加和校验（checksum16）
 * @param data 数据指针
 * @param length 数据长度
 * @return 16位累加和校验值
 */
U16 checksum16(const U8 *data, U32 length)
{
    U32 sum = 0;
    U32 i;

    if (data == NULL) {
        return 0;
    }

    for (i = 0; i < length; i++) {
        sum += data[i];
    }

    /* 将sum折叠为16位 */
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return ~(U16)sum;
}


