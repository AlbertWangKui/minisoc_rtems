/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file board_init.c
 * @author shaogl@starsmicrosystem.com
 * @date 2025/06/06
 * @brief 板级初始化实现
 */
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "board_init.h"
#include "iomux.h"
#include <bsp/linker-symbols.h>

__attribute__((weak)) void mpuInit(void)
{
    U32 temp;
    U32 i = 0;

    temp = mpuSystemCtrlRegGet();
    temp &= ~MPU_SYSTEM_CONTROL_C;
    temp &= ~MPU_SYSTEM_CONTROL_I;
    temp &= ~MPU_SYSTEM_CONTROL_M;
    mpuSystemCtrlRegSet(temp);

    /* back */
    memRegionNumSet(i++);
    memRegionBaseSet(0x00000000);
    memAccessAttributeSet(MPU_REGION_EXECUTE_NEVER |
        MPU_REGION_TYPE_NORMAL_WRITE_BACK_NO_WRITE_ALLOC |
        MPU_REGION_TYPE_NORMAL_NSHARED |
        MPU_REGION_AP_NO_ACCESS);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_4G | MPU_REGION_ENABLE);

    /* sram */
    memRegionNumSet(i++);
    memRegionBaseSet((U32)bsp_sram_os_used_begin);
    memAccessAttributeSet(MPU_REGION_TYPE_NORMAL_WRITE_BACK_WRITE_ALLOC |
        MPU_REGION_TYPE_NORMAL_NSHARED |
        MPU_REGION_AP_FULL_ACCESS);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_8M | MPU_REGION_ENABLE);

    /* sram non-cache */
    memRegionNumSet(i++);
    memRegionBaseSet((U32)bsp_sram_user_sbr_nocache_begin);
    memAccessAttributeSet(MPU_REGION_TYPE_NORMAL_NON_CACHEBLE |
        MPU_REGION_TYPE_NORMAL_NSHARED |
        MPU_REGION_AP_FULL_ACCESS);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_64K | MPU_REGION_ENABLE);

    /* rom */
    memRegionNumSet(i++);
    memRegionBaseSet(SYS_ROM_BASE_ADDR);
    memAccessAttributeSet(MPU_REGION_TYPE_NORMAL_NON_CACHEBLE |
        MPU_REGION_TYPE_NORMAL_NSHARED |
        MPU_REGION_AP_READ_ONLY);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_64K | MPU_REGION_ENABLE);

    /* flash */
    memRegionNumSet(i++);
    memRegionBaseSet(SYS_FLASH_BASE_ADDR);
    memAccessAttributeSet(MPU_REGION_EXECUTE_NEVER |
        MPU_REGION_TYPE_DEVICE_NSHARED |
        MPU_REGION_AP_FULL_ACCESS);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_512M | MPU_REGION_ENABLE);

    /* device register */
    memRegionNumSet(i++);
    memRegionBaseSet(SYS_DEVICE_MAP_ADDR);
    memAccessAttributeSet(MPU_REGION_EXECUTE_NEVER |
        MPU_REGION_TYPE_DEVICE_NSHARED |
        MPU_REGION_AP_FULL_ACCESS);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_1G | MPU_REGION_ENABLE);

    /* boot */
    memRegionNumSet(i++);
    memRegionBaseSet(0xFFFF0000);
    memAccessAttributeSet(MPU_REGION_TYPE_DEVICE_NSHARED |
        MPU_REGION_TYPE_NORMAL_WRITE_BACK_WRITE_ALLOC |
        MPU_REGION_AP_READ_ONLY);
    memRegionSizeSetAndEnable(MPU_REGION_SIZE_64K | MPU_REGION_ENABLE);

    temp = mpuSystemCtrlRegGet();
    //temp |= MPU_SYSTEM_CONTROL_C;
    temp |= MPU_SYSTEM_CONTROL_I;
    temp |= MPU_SYSTEM_CONTROL_M;
    mpuSystemCtrlRegSet(temp);
}

void exceptionAddressRemap(void)
{
    U32 regValue;

    regValue = reg32Read(SYS_SOC_CTRL_BASE_ADRS + REG_SOC_CTRL_NOCMAP_OFFSET);
    regValue |= 1;
    reg32Write(SYS_SOC_CTRL_BASE_ADRS + REG_SOC_CTRL_NOCMAP_OFFSET, regValue);
}

S32 boardInit(void)
{
    S32 ret = EXIT_SUCCESS;

    ///< gpio 复位
    CLR_REG_BIT(SYS_GPIO_RST_REG, SYS_GPIO_RST_BITS_NR);
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("DSB");
    SET_REG_BIT(SYS_GPIO_RST_REG, SYS_GPIO_RST_BITS_NR);

    /* pinmux init */
    //iomuxDefaultConfig();

    return ret;
}
