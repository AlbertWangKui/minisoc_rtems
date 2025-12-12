/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_topcrg.c
 * @author huyl3@starsmicrosystem.com
 * @date 2025/09/26
 * @brief
 */

#include "common_defines.h"
#include "log_msg.h"

#define INFO_REG2_ADDR      0xB800CFF8

typedef enum
{
    HW_PLATFORM_ASIC,   /* 0 */
    HW_PLATFORM_EDA,
    HW_PLATFORM_S2C,
    HW_PLATFORM_HAPS100,
    HW_PLATFORM_HAPS200,
    HW_PLATFORM_UVHS_1_VP1902, /* 5 */
    HW_PLATFORM_UVHS_4_VP1902,
    HW_PLATFORM_EMU_PZ1,
    HW_PLATFORM_EMU_VELOCE,
    HW_PLATFORM_MAX,
} hwPlatformType;

// info_reg2[31:28] hwPlatformType
const U8 *getHwPlatformType(void)
{
    const char *hwName = NULL;
    U32 info = reg32Read(INFO_REG2_ADDR);

    info >>= 28;
    info &= 0x0F;

    switch(info)
    {
        case HW_PLATFORM_ASIC:
            hwName = "ASIC";
            break;
        case HW_PLATFORM_EDA:
            hwName = "EDA";
            break;
        case HW_PLATFORM_S2C:
            hwName = "S2C";
            break;
        case HW_PLATFORM_HAPS100:
            hwName = "HAPS100";
            break;
        case HW_PLATFORM_HAPS200:
            hwName = "HAPS200";
            break;
        case HW_PLATFORM_UVHS_1_VP1902:
            hwName = "UVHS_1*VP1902";
            break;
        case HW_PLATFORM_UVHS_4_VP1902:
            hwName = "UVHS_4*VP1902";
            break;
        case HW_PLATFORM_EMU_PZ1:
            hwName = "EMU(PZ1)";
            break;
        case HW_PLATFORM_EMU_VELOCE:
            hwName = "EMU(VELOCE)";
            break;
        default:
            LOGE("platform ID %d\r\n", info);
            hwName = "Unknown";
            break;
    }

    LOGI("hw platform %s\r\n", hwName);

    return (const U8 *)hwName;
}

// info_reg2[15:0] bcd code like 0723/0827
U32 getHwPlatformVersion(U8 *strBuffer, U32 strLen)
{
    U32 info = reg32Read(INFO_REG2_ADDR);
    U16 ver = (U16)(info & 0xFFFF);
    U8  verStr[5];

    if(strBuffer == NULL || strLen == 0)
    {
        return 0;
    }

    verStr[0] = ((ver >> 12) & 0x0F) + '0';
    verStr[1] = ((ver >> 8) & 0x0F) + '0';
    verStr[2] = ((ver >> 4) & 0x0F) + '0';
    verStr[3] = ((ver ) & 0x0F) + '0';
    verStr[4] = 0;

    LOGI("hw platform version %s\r\n", verStr);

    if(strLen > sizeof(verStr))
    {
        strLen = sizeof(verStr);
    }

    memcpy(strBuffer, verStr, strLen);

    return strLen;
}

