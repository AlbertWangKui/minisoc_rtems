/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_config.h
 * @author shaogl@starsmicrosystem.com
 * @date 2025.05.24
 * @brief none.
 */

#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

#include "kconfig.h"

/* 时钟定义 */
#define SYS_XTAL_CLK_FREQ               (25000000) ///< 系统晶振频率，单位Hz，必须定义

/* IP资源定义，如果没在配置文件中定义，可以定义在这里*/
#define SYS_TIMER0_BASE_ADRS            (0xBE660000)
#define SYS_TIMER1_BASE_ADRS            (0xBE661000)
#define SYS_UART0_BASE_ADRS             (0xBE640000)

#define SYS_INT_NUM_TIMER0              (81)
#define SYS_INT_NUM_TIMER1              (82)
#define SYS_INT_NUM_UART0               (98)

#define SYS_FUNCMUX_BASE_ADRS           (0xB8040000)
#define SYS_CRG_BASE_ADRS               (0xBE600000)
#define SYS_GIC_DIST_BASE_ADRS          (0xBE301000)
#define SYS_SOC_CTRL_BASE_ADRS          (0xBE100000)
#define REG_SOC_CTRL_NOCMAP_OFFSET      (0x04)
#define SYS_TOP_CRG_BASE_ADRS           (0xB8000000)
#define SYS_GIC_CPUIF_BASE_ADRS         (0xBE302000)

#define SYS_INT_NUM_QSPI                (49)
#define SYS_QSPI_BASE_ADRS              (0xBE410000)
#define SYS_QSPI_MAP_FLASH1             (0x60000000)

/*
 * TODO:sram大小，分区等的定义出处只能有一处，且要根据不同平台分别定义
 */

#define SYS_ROM_BASE_ADDR               (0x03000000)
#define SYS_ROM_SIZE                    (0x010000)

#define SYS_FLASH_BASE_ADDR             (0x60000000)
#define SYS_FLASH_SIZE                  (0x4000000)

#define SYS_DEVICE_MAP_ADDR             (0x80000000)
#define SYS_DEVICE_MAP_SIZE             (0x40000000)
/*
 * tick timer,high resolution timer,console 定义，这几项没有定义在配置文件中，所以定义在这里
 */
#define SYS_TICK_TIMER_BASE_ADRS        (SYS_TIMER0_BASE_ADRS)
#define SYS_HR_TIMER_BASE_ADRS          (SYS_TIMER1_BASE_ADRS)
#define SYS_TICK_TIMER_DEV              DEVICE_TIMER0
#define SYS_HR_TIMER_DEV                DEVICE_TIMER1

#define SYS_CONSOLE_UART_BASE_ADRS      (SYS_UART0_BASE_ADRS)
#define SYS_CONSOLE_UART_DEV            (DEVICE_UART0)
#define SYS_CONSOLE_UART_INT_NUM        (SYS_INT_NUM_UART0)

#endif
