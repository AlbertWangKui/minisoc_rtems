/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_uart.h
 * @author taohb@starsmicrosystem.com)
 * @date 2025/06/07
 * @brief uart driver definitions.
 */

#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include <bsp/arm-gic-irq.h>
#include <bsp/irq-generic.h>
#include "common_defines.h"
#include "drv_uart_api.h"
#include "bsp_config.h"
#include "bsp_api.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

#define CONSOLE_USE_INTERRUPTS
#define UART_MAX_BAUD_RATE 4000000
#define UART_MIN_BAUD_RATE 1

typedef struct UartDrvData UartDrvData_s;

typedef struct UartCallbackOps {
    Bool isTxValid;
    UartTxCallback txCallback;

    Bool isRxValid;
    UartRxCallback rxCallback;
} UartCallbackOps_s;

typedef union UartCtrlReg {
    struct {
        U32 ctrlEnable : 1;
        U32 ctrlLoopback : 1;
        U32 flexiBaudOn : 1;
        U32 generateBreak : 1;
        U32 sizeSel : 2;
        U32 parityType : 2;
        U32 stopLength : 2;
        U32 dataLength : 4;
        U32 baudAdjustOn : 1;
        U32 : 1;
        U32 txIrqEn : 1;
        U32 rxIrqEn : 1;
        U32 rxBreakIrqEn : 1;
        U32 rxParityErrIrqEn : 1;
        U32 rxFctrlIrqEn : 1;
        U32 rxTimeoutIrqEn : 1;
        U32 rxOverrunEn: 1;
        U32 : 1;
        U32 rxTimeoutCount : 8;
    } fields;
    U32 dword;
} UartCtrlReg_u;

typedef union UartAuxReg {
    struct {
        U32 flexiBaud : 16;
        U32 : 16;
    } fields;
    U32 dword;
} UartAuxReg_u;

typedef union UartCompsReg {
    struct {
        U32 startBitMidpoint : 16;
        U32 baudCompensateValue : 4;
        U32 : 12;
    } fields;
    U32 dword;
} UartCompsReg_u;

typedef union UartFifoReg {
    struct {
        U32 rxIrqFifoLevel : 7;
        U32 : 1;
        U32 txIrqFifoLevel : 7;
        U32 : 1;
        U32 rxFifoLevel : 7;
        U32 : 1;
        U32 txFifoLevel : 7;
        U32 : 1;
    } fields;
    U32 dword;
} UartFifoReg_u;

typedef union UartFctrl0Reg {
    struct {
        U32 swFctrlRxFifoDown : 7;
        U32 : 1;
        U32 swFctrlRxFifoUp : 7;
        U32 : 1;
        U32 hwFctrlRtsEn : 1;
        U32 hwFctrlCtsEn : 1;
        U32 swFctrlTxXoffXonEn : 1;
        U32 swFctrlRxXoffXonEn : 1;
        U32 swFctrlRxXoffXonXdataEn : 1;
        U32 : 11;
    } fields;
    U32 dword;
} UartFctrl0Reg_u;

typedef union UartFctrl1Reg {
    struct {
        U32 swFctrlXoff : 8;
        U32 swFctrlXon : 8;
        U32 : 12;
    } fields;
    U32 dword;
} UartFctrl1Reg_u;

typedef union UartDebugReg {
    struct {
        U32 preadyEn : 1;
        U32 pslverrEn : 1;
        U32 : 30;
    } fields;
    U32 dword;
} UartDebugReg_u;

typedef union UartStatusReg {
    struct {
        U32 txIrq : 1;
        U32 rxIrq : 1;
        U32 rxBreakIrq : 1;
        U32 rxParityErrIrq : 1;
        U32 rxFctrlIrq : 1;
        U32 rxTimeoutIrq : 1;
        U32 rxOverRun : 1;
        U32 : 1;
        U32 rxState : 3;
        U32 : 1;
        U32 txState : 3;
        U32 : 17;
    } fields;
    U32 dword;
} UartStatusReg_u;

typedef volatile struct UartReg {
    U32 data;
    UartCtrlReg_u ctrl;
    UartAuxReg_u aux;
    UartCompsReg_u comps;
    UartFifoReg_u fifo;
    UartFctrl0Reg_u fctrl0;
    UartFctrl1Reg_u fctrl1;
    UartDebugReg_u debug;
    UartStatusReg_u status;
} UartReg_s;

typedef struct UartDrvData {
    DevList_e devId;
    UartReg_s *reg;
    SbrUartCfg_s sbrCfg;
    UartBasicSetting_s setting;

    UartTxCallback txCallback;
    UartRxCallback rxCallback;
} UartDrvData_s;

/**
 * @brief uartConosle初始化函数
 * @param [in] tty termios tty对象
 * @return EXIT_SUCCESS is success, negative return is failure
 */
S32 uartConsoleInit(void *tty);

/**
 * @brief uartConosle反初始化函数
 * @param [in] none
 * @param [out] none
 * @return none
 */
S32 uartConsoleDeinit(void);

/**
 * @brief console uart read
 * @param [in] none
 * @param [out] none
 * @return uart读的字符
 */
S32 uartConsoleRead(void);

/**
 * @brief console uart write
 * @param [in] buf uart写入的数据buf
 * @param [in] n uart写入的字符数
 * @param [out] none
 * @return none
 */
void uartConsoleWrite(const S8 *buf, size_t n);

/**
 * @brief printk uart写
 * @param [in] c printk写入的字符
 * @param [out] none
 * @return none
 */
void uartPrintkWrite(S8 c);

#endif /* __DRV_UART_H__ */
