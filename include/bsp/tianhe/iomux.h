/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file iomux.h
 * @author shaogl@starsmicrosystem.com
 * @date 2025.05.24
 * @brief none.
 */

#ifndef __IO_MUX_H__
#define __IO_MUX_H__

#define IOMUX_FUNC_BASE_REG (0xB8040218)
#define REG_IOMUX_BITS_WIDTH (4)
#define REG_BITS_LEN (32)
#define JASON_KEYVALUE_TOTAL (10)

typedef enum iomuxIoNum {
    IOMUX_GPIO0 = 0,
    IOMUX_GPIO1, IOMUX_GPIO2, IOMUX_GPIO3,
    IOMUX_GPIO4, IOMUX_GPIO5, IOMUX_GPIO6,
    IOMUX_GPIO7, IOMUX_GPIO8, IOMUX_GPIO9,
    IOMUX_GPIO10,
    IOMUX_GPIO11, IOMUX_GPIO12, IOMUX_GPIO13,
    IOMUX_GPIO14, IOMUX_GPIO15, IOMUX_GPIO16,
    IOMUX_GPIO17, IOMUX_GPIO18, IOMUX_GPIO19,
    IOMUX_GPIO20, IOMUX_GPIO21, IOMUX_GPIO22,
    IOMUX_GPIO23, IOMUX_GPIO24, IOMUX_GPIO25,
    IOMUX_GPIO26, IOMUX_GPIO27, IOMUX_GPIO28,
    IOMUX_GPIO29, IOMUX_GPIO30, IOMUX_GPIO31,
    IOMUX_GPIO32, IOMUX_GPIO33, IOMUX_GPIO34,
    IOMUX_GPIO35, IOMUX_GPIO36, IOMUX_GPIO37,
    IOMUX_GPIO38, IOMUX_GPIO39, IOMUX_GPIO40,
    IOMUX_GPIO41, IOMUX_GPIO42, IOMUX_GPIO43,
    IOMUX_GPIO44, IOMUX_GPIO45, IOMUX_GPIO46,
    IOMUX_GPIO47, IOMUX_GPIO48, IOMUX_GPIO49,
    IOMUX_GPIO50, IOMUX_GPIO51, IOMUX_GPIO52,
    IOMUX_GPIO53, IOMUX_GPIO54, IOMUX_GPIO55,
    IOMUX_GPIO56, IOMUX_GPIO57, IOMUX_GPIO58,
    IOMUX_GPIO59, IOMUX_GPIO60, IOMUX_GPIO61,
    IOMUX_GPIO62, IOMUX_GPIO63,
    IOMUX_TEST0 = 64, IOMUX_TEST1 = 65, IOMUX_TEST2 = 66,
    IOMUX_TEST3 = 67, IOMUX_TEST4 = 68, IOMUX_TEST5 = 69,
    IOMUX_TEST6 = 70, IOMUX_TEST7 = 71, IOMUX_TEST8 = 72,
    IOMUX_TEST9 = 73, IOMUX_TEST10 = 74, IOMUX_TEST11 = 75,
    IOMUX_TEST12 = 76, IOMUX_TMUX_EN0 = 77, IOMUX_TMUX_EN1 = 78,
    IOMUX_TMUX_EN2 = 79,
    IOMUX_TMUX_INVALID
} IomuxIoNum_e;

typedef enum iomuxFunc {
    IOMUX_DEFAULT_FUNC = 0,
    IOMUX_FUNC1,
    IOMUX_FUNC2,
    IOMUX_FUNC3,
    IOMUX_FUNC4,
    IOMUX_FUNC_INVALID,
} IomuxFunc_e;

typedef volatile union pinFuncReg {
    struct {
        U32 mode: 4; /* bit3 ~ bit0 */
        U32 : 12; /* bit15 ~ bit4 */
        U32 toCore : 1; /* bit16 */
        U32 toPad : 1; /* bit17 */
        U32 ie : 1; /* bit18 */
        U32 oe : 1; /* bit19 */
        U32 : 12; /* bit31 ~ bit20 */
    } fields;
    U32 dword;
} pinFuncReg_u;

typedef volatile struct iomuxReg {
    pinFuncReg_u pinFunc;
    U32 reserved;
} IomuxReg_s;

/**
 *
| Pin    | default | alternative func1 | alt func2      | alt func3 |
| ------ | ------- | ----------------- | -------------- | --------- |
| GPIO0  | GPIO0   | -                 | -              | mntbus_0  |
| GPIO1  | GPIO1   | -                 | -              | mntbus_1  |
| GPIO2  | GPIO2   | -                 | -              | mntbus_2  |
| GPIO3  | GPIO3   | -                 | -              | mntbus_3  |
| GPIO4  | GPIO4   | UART_1_RX         | -              | mntbus_4  |
| GPIO5  | GPIO5   | UART_1_TX         | -              | mntbus_5  |
| GPIO6  | GPIO6   | UART_1_CTS        | -              | mntbus_6  |
| GPIO7  | GPIO7   | UART_1_RTS        | -              | mntbus_7  |
| GPIO8  | GPIO8   | I2C_SCL_0         | -              | mntbus_8  |
| GPIO9  | GPIO9   | I2C_SDA_0         | -              | mntbus_9  |
| GPIO10 | GPIO10  | I2C_SCL_1         | -              | mntbus_10 |
| GPIO11 | GPIO11  | I2C_SDA_1         | -              | mntbus_11 |
| GPIO12 | GPIO12  | I2C_SCL_2         | -              | mntbus_12 |
| GPIO13 | GPIO13  | I2C_SDA_2         | -              | mntbus_13 |
| GPIO14 | GPIO14  | I2C_SCL_3         | -              | mntbus_14 |
| GPIO15 | GPIO15  | I2C_SDA_3         | -              | mntbus_15 |
| GPIO16 | GPIO16  | CPLD_QSPI_CLK     | -              | mntbus_16 |
| GPIO17 | GPIO17  | CPLD_QSPI_CS      | TRACE_CLK      | mntbus_17 |
| GPIO18 | GPIO18  | CPLD_QSPI_DIO0    | TRACE_CTL      | mntbus_18 |
| GPIO19 | GPIO19  | CPLD_QSPI_DIO1    | TRACE_DATA_0   | mntbus_19 |
| GPIO20 | GPIO20  | CPLD_QSPI_DIO2    | -              | mntbus_20 |
| GPIO21 | GPIO21  | CPLD_QSPI_DIO3    | TRACE_DATA_1   | mntbus_21 |
| GPIO22 | GPIO22  | CPLD_QSPI_RST     | TRACE_DATA_2   | mntbus_22 |
| GPIO23 | GPIO23  | CPLD_QSPI_INT     | TRACE_DATA_3   | mntbus_23 |
| GPIO24 | GPIO24  | JTAG_TCK_FUNC     | -              | mntbus_24 |
| GPIO25 | GPIO25  | JTAG_TDI_FUNC     | -              | mntbus_25 |
| GPIO26 | GPIO26  | JTAG_TDO_FUNC     | -              | mntbus_26 |
| GPIO27 | GPIO27  | JTAG_TMS_FUNC     | -              | mntbus_27 |
| GPIO28 | GPIO28  | JTAG_TRST_N_FUNC  | -              | mntbus_28 |
| GPIO29 | GPIO29  | -                 | -              | mntbus_29 |
| GPIO30 | GPIO30  | -                 | -              | mntbus_30 |
| GPIO31 | GPIO31  | -                 | TRACE_DATA_4   | mntbus_31 |
| GPIO32 | GPIO32  | -                 |                | -         |
| GPIO33 | GPIO33  | -                 | TRACE_DATA_5   | -         |
| GPIO34 | GPIO34  | -                 | TRACE_DATA_6   | -         |
| GPIO35 | GPIO35  | -                 | TRACE_DATA_7   | -         |
*/

/**
 * @brief 切换IO引脚的复用功能
 * @details none
 * @param [in] ioNum 表示支持复用功能的IO引脚编号
 * @param [in] funcSel 表示选择的复用功能
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 * @warning 不阻塞,可重入,OS启动前/后,可用于中断上下文,可用于线程上下文
 */
S32 ioFuncConfig(IomuxIoNum_e ioNum, IomuxFunc_e funcSel);

#endif /* __FUNC_MUX_H */
