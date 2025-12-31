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

#define IOMUX_FUNC_BASE_REG (0xB8040208)
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
    IOMUX_TEST9 = 73, IOMUX_TEST10 = 74, IOMUX_TMUX_INVALID
} IomuxIoNum_e;

typedef enum iomuxFunc {
    IOMUX_DEFAULT_FUNC = 0,
    IOMUX_FUNC1,
    IOMUX_FUNC2,
    IOMUX_FUNC3,
    IOMUX_FUNC4,
    IOMUX_FUNC5,
    IOMUX_FUNC6,
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
| Pin    | default func0   | alt func1          | alt func2            | alt func3         | alt func4     | alt func5      | alt func6 |
| ------ | -------------   | -----------------  | --------------       | ---------         | ---------     | ---------      | --------- |
| GPIO0  | GPIO            | LEDDATA0           | HP_PRSNT0_N          | LINKUP_LED0       | CLKREQ0_N     | SGPIO0_CLK     | TMUX_0    |
| GPIO1  | GPIO            | LEDDATA1           | HP_PRSNT1_N          | LINKUP_LED1       | CLKREQ1_N     | SGPIO0_LOAD    | TMUX_1    |
| GPIO2  | GPIO            | LEDDATA2           | HP_PRSNT2_N          | LINKUP_LED2       | CLKREQ2_N     | SGPIO0_DIN     | TMUX_2    |
| GPIO3  | GPIO            | LEDDATA3           | HP_PRSNT3_N          | LINKUP_LED3       | CLKREQ3_N     | SGPIO0_DOUT    | TMUX_3    |
| GPIO4  | GPIO            | LEDDATA4           | HP_PRSNT4_N          | LINKUP_LED4       | CLKREQ4_N     | SGPIO1_CLK     | TMUX_4    |
| GPIO5  | GPIO            | LEDDATA5           | HP_PRSNT5_N          | LINKUP_LED5       | CLKREQ5_N     | SGPIO1_LOAD    | TMUX_5    |
| GPIO6  | GPIO            | LEDDATA6           | HP_PRSNT6_N          | LINKUP_LED6       | CLKREQ6_N     | SGPIO1_DIN     | TMUX_6    |
| GPIO7  | GPIO            | LEDDATA7           | HP_PRSNT7_N          | LINKUP_LED7       | CLKREQ7_N     | SGPIO1_DOUT    | TMUX_7    |
| GPIO8  | GPIO            | LEDDATA8           | HP_PRSNT8_N          | LINKUP_LED8       | CLKREQ8_N     | I2C4_SCL       | TMUX_8    |
| GPIO9  | GPIO            | LEDDATA9           | HP_PRSNT9_N          | LINKUP_LED9       | CLKREQ9_N     | I2C4_SDA       | TMUX_9    |
| GPIO10 | GPIO            | LEDDATA10          | HP_PRSNT10_N         | LINKUP_LED10      | CLKREQ10_N    | I2C5_SCL       | TMUX_10   |
| GPIO11 | GPIO            | LEDDATA11          | HP_PRSNT11_N         | LINKUP_LED11      | CLKREQ11_N    | I2C5_SDA       | TMUX_11   |
| GPIO12 | GPIO            | LEDDATA12          | HP_PRSNT12_N         | LINKUP_LED12      | CLKREQ12_N    | PWM_OUT0       | TMUX_12   |
| GPIO13 | GPIO            | LEDDATA13          | HP_PRSNT13_N         | LINKUP_LED13      | CLKREQ13_N    | PWM_OUT1       | TMUX_13   |
| GPIO14 | GPIO            | LEDDATA14          | HP_PRSNT14_N         | LINKUP_LED14      | CLKREQ14_N    | TACH_IN0       | TMUX_14   |
| GPIO15 | GPIO            | LEDDATA15          | HP_PRSNT15_N         | LINKUP_LED15      | CLKREQ15_N    | TACH_IN1       | TMUX_15   |
| GPIO16 | GPIO            | LEDDATA16          | HP_PRSNT16_N         | LINKUP_LED16      |               | TRACE_CTL      | TMUX_16   |
| GPIO17 | GPIO            | LEDDATA17          | HP_PRSNT17_N         | LINKUP_LED17      |               | TRACE_CLK      | TMUX_17   |
| GPIO18 | GPIO            | LEDDATA18          | HP_PRSNT18_N         | LINKUP_LED18      |               | TRACE_DATA0    | TMUX_18   |
| GPIO19 | GPIO            | LEDDATA19          | HP_PRSNT19_N         | LINKUP_LED19      |               | TRACE_DATA1    | TMUX_19   |
| GPIO20 | GPIO            | LEDDATA20          | HP_PRSNT20_N         | LINKUP_LED20      |               | TRACE_DATA2    | TMUX_20   |
| GPIO21 | GPIO            | LEDDATA21          | HP_PRSNT21_N         | LINKUP_LED21      |               | TRACE_DATA3    | TMUX_21   |
| GPIO22 | GPIO            | LEDDATA22          | HP_PRSNT22_N         | LINKUP_LED22      |               | TRACE_DATA4    | TMUX_22   |
| GPIO23 | GPIO            | LEDDATA23          | HP_PRSNT23_N         | LINKUP_LED23      |               | TRACE_DATA5    | TMUX_23   |
| GPIO24 | GPIO            | LEDDATA24          | HP_PRSNT24_N         | LINKUP_LED24      |               | TRACE_DATA6    | TMUX_24   |
| GPIO25 | GPIO            | LEDDATA25          | HP_PRSNT25_N         | LINKUP_LED25      |               | TRACE_DATA7    | TMUX_25   |
| GPIO26 | GPIO            | LEDDATA26          | HP_PRSNT26_N         | LINKUP_LED26      |               | TRACE_DATA8    | TMUX_26   |
| GPIO27 | GPIO            | LEDDATA27          | HP_PRSNT27_N         | LINKUP_LED27      |               | TRACE_DATA9    | TMUX_27   |
| GPIO28 | GPIO            | LEDDATA28          | HP_PRSNT28_N         | LINKUP_LED28      |               | TRACE_DATA10   | TMUX_28   |
| GPIO29 | GPIO            | LEDDATA29          | HP_PRSNT29_N         | LINKUP_LED29      |               | TRACE_DATA11   | TMUX_29   |
| GPIO30 | GPIO            | LEDDATA30          | HP_PRSNT30_N         | LINKUP_LED30      |               | TRACE_DATA12   | TMUX_30   |
| GPIO31 | GPIO            | LEDDATA31          | HP_PRSNT31_N         | LINKUP_LED31      |               | TRACE_DATA13   | TMUX_31   |
| GPIO32 | GPIO            | LEDDATA32          | HP_PRSNT32_N         | LINKUP_LED32      | CLKREQ16_N    | TRACE_DATA14   | TMUX_32   |
| GPIO33 | GPIO            | LEDDATA33          | HP_PRSNT33_N         | LINKUP_LED33      | CLKREQ17_N    | TRACE_DATA15   | TMUX_33   |
| GPIO34 | GPIO            | LEDDATA34          | HP_PRSNT34_N         | LINKUP_LED34      | CLKREQ18_N    | -              | TMUX_34   |
| GPIO35 | GPIO            | LEDDATA35          | HP_PRSNT35_N         | LINKUP_LED35      | CLKREQ19_N    | -              | TMUX_35   |
| GPIO36 | GPIO            | LEDDATA36          | HP_PRSNT36_N         | LINKUP_LED36      | CLKREQ20_N    | -              | TMUX_36   |
| GPIO37 | GPIO            | LEDDATA37          | HP_PRSNT37_N         | LINKUP_LED37      | CLKREQ21_N    | -              | TMUX_37   |
| GPIO38 | GPIO            | LEDDATA38          | HP_PRSNT38_N         | LINKUP_LED38      | CLKREQ22_N    | -              | TMUX_38   |
| GPIO39 | GPIO            | LEDDATA39          | HP_PRSNT39_N         | LINKUP_LED39      | CLKREQ23_N    | -              | TMUX_39   |
| GPIO40 | GPIO            | LEDDATA40          | HP_PRSNT40_N         | LINKUP_LED40      | CLKREQ24_N    | -              | TMUX_40   |
| GPIO41 | GPIO            | LEDDATA41          | HP_PRSNT41_N         | LINKUP_LED41      | CLKREQ25_N    | -              | TMUX_41   |
| GPIO42 | GPIO            | LEDDATA42          | HP_PRSNT42_N         | LINKUP_LED42      | CLKREQ26_N    | -              | TMUX_42   |
| GPIO43 | GPIO            | LEDDATA43          | HP_PRSNT43_N         | LINKUP_LED43      | CLKREQ27_N    | -              | TMUX_43   |
| GPIO44 | GPIO            | LEDDATA44          | HP_PRSNT44_N         | LINKUP_LED44      | CLKREQ28_N    | -              | TMUX_44   |
| GPIO45 | GPIO            | LEDDATA45          | HP_PRSNT45_N         | LINKUP_LED45      | CLKREQ29_N    | -              | TMUX_45   |
| GPIO46 | GPIO            | LEDDATA46          | HP_PRSNT46_N         | LINKUP_LED46      | CLKREQ30_N    | -              | TMUX_46   |
| GPIO47 | GPIO            | LEDDATA47          | HP_PRSNT47_N         | LINKUP_LED47      | CLKREQ31_N    | -              | TMUX_47   |
| GPIO48 | GPIO            | LEDDATA48          | HP_PRSNT48_N         | LINKUP_LED48      | TACH_IN2      | -              | TMUX_48   |
| GPIO49 | GPIO            | LEDDATA49          | HP_PRSNT49_N         | LINKUP_LED49      | TACH_IN3      | -              | TMUX_49   |
| GPIO50 | GPIO            | LEDDATA50          | HP_PRSNT50_N         | LINKUP_LED50      | SGPIO2_CLK    | -              | TMUX_50   |
| GPIO51 | GPIO            | LEDDATA51          | HP_PRSNT51_N         | LINKUP_LED51      | SGPIO2_LOAD   | -              | TMUX_51   |
| GPIO52 | GPIO            | LEDDATA52          | HP_PRSNT52_N         | LINKUP_LED52      | SGPIO2_DIN    | -              | TMUX_52   |
| GPIO53 | GPIO            | LEDDATA53          | HP_PRSNT53_N         | LINKUP_LED53      | SGPIO2_DOUT   | -              | TMUX_53   |
| GPIO54 | GPIO            | LEDDATA54          | HP_PRSNT54_N         | LINKUP_LED54      | SGPIO3_CLK    | -              | TMUX_54   |
| GPIO55 | GPIO            | LEDDATA55          | HP_PRSNT55_N         | LINKUP_LED55      | SGPIO3_LOAD   | -              | TMUX_55   |
| GPIO56 | GPIO            | LEDDATA56          | HP_PRSNT56_N         | LINKUP_LED56      | SGPIO3_DIN    | -              | TMUX_56   |
| GPIO57 | GPIO            | LEDDATA57          | HP_PRSNT57_N         | LINKUP_LED57      | SGPIO3_DOUT   | -              | TMUX_57   |
| GPIO58 | GPIO            | LEDDATA58          | HP_PRSNT58_N         | LINKUP_LED58      | I2C6_SCL      | -              | TMUX_58   |
| GPIO59 | GPIO            | LEDDATA59          | HP_PRSNT59_N         | LINKUP_LED59      | I2C6_SDA      | -              | TMUX_59   |
| GPIO60 | GPIO            | LEDDATA60          | HP_PRSNT60_N         | LINKUP_LED60      | I2C7_SCL      | -              | TMUX_60   |
| GPIO61 | GPIO            | LEDDATA61          | HP_PRSNT61_N         | LINKUP_LED61      | I2C7_SDA      | -              | TMUX_61   |
| GPIO62 | GPIO            | LEDDATA62          | HP_PRSNT62_N         | LINKUP_LED62      | -             | -              | TMUX_62   |
| GPIO63 | GPIO            | LEDDATA63          | HP_PRSNT63_N         | LINKUP_LED63      | -             | -              | TMUX_63   |
| TEST0  |                 | SYS_ERROR_CODE_0   | SGPIO0_BPTYPE        | UART_RX_PHY       | -             | -              | -         |
| TEST1  |                 | SYS_ERROR_CODE_1   | SGPIO0_CTRL_TYPE     | UART_TX_PHY       | -             | -              | -         |
| TEST2  |                 | SYS_ERROR_CODE_2   |                      | PHY0_JTAG_TCK     | -             | -              | -         |
| TEST3  |                 | SYS_ERROR_CODE_3   | SGPIO1_BPTYPE        | PHY0_JTAG_TDI     | -             | -              | -         |
| TEST4  |                 | SYS_ERROR_CODE_4   | SGPIO1_CTRL_TYPE     | PHY0_JTAG_TDO     | -             | -              | -         |
| TEST5  |                 |                    |                      | PHY0_JTAG_TMS     | -             | -              | -         |
| TEST6  |                 |                    | SGPIO2_BPTYPE        | PHY0_JTAG_TRSTN   | -             | -              | -         |
| TEST7  |                 |                    | SGPIO2_CTRL_TYPE     | -                 | -             | -              | -         |
| TEST8  |                 |                    |                      | -                 | -             | -              | -         |
| TEST9  |                 |                    | SGPIO3_BPTYPE        | -                 | -             | -              | -         |
| TEST10 |                 |                    | SGPIO3_CTRL_TYPE     | -                 | -             | -              | -         |
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
