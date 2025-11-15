/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_uart.c
 * @author taohb@starsmicrosystem.com
 * @date 2025/6/7
 * @brief uart driver
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <bsp/arm-gic-irq.h>
#include <bsp/irq-generic.h>
#include <rtems.h>
#include <rtems/termiostypes.h>
#include <rtems/console.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "osp_interrupt.h"
#include "log_msg.h"
#include "udelay.h"
#include "drv_uart.h"

#define UART_TX_FIFO_SIZE  64
#define UART_RX_FIFO_SIZE  64

static UartCtrlReg_u ctrlRegDefault = { \
    .fields = { \
        .ctrlEnable = 1, \
        .ctrlLoopback = 0, \
        .flexiBaudOn = 1, \
        .generateBreak = 0, \
        .sizeSel = 1, \
        .parityType = 0, \
        .stopLength = 1, \
        .dataLength = 8, \
        .baudAdjustOn = 0, \
        .txIrqEn = 0, \
        .rxIrqEn = 0, \
        .rxBreakIrqEn = 0, \
        .rxParityErrIrqEn = 0, \
        .rxFctrlIrqEn = 0, \
        .rxTimeoutIrqEn = 0, \
        .rxOverrunEn = 0, \
        .rxTimeoutCount = 0, \
    },
};

static SbrUartCfg_s uartConsoleBasicSetting = {
    .regAddr = (void *)SYS_CONSOLE_UART_BASE_ADRS,
    .irqNo = SYS_CONSOLE_UART_INT_NUM,
    .irqPrio = 127, /* default value */
    .baudRate = 115200,
    .dataBits = UART_DATA_LEN_8,
    .stopBits = UART_STOP_1,
    .parity = UART_PARITY_NONE,
    .console = 1,
    .txIntEn = 0,
#ifdef CONSOLE_USE_INTERRUPTS
    .rxIntEn = 1,
#else
    .rxIntEn = 0,
#endif
};

static rtems_termios_tty *ttyObj = NULL;

void ttyObjSet(rtems_termios_tty *tty)
{
    if (tty) {
        ttyObj = tty;
    }
}

rtems_termios_tty *ttyObjGet(void)
{
    return ttyObj;
}

static U32 uartConsoleDevCfgGet(DevList_e devId, SbrUartCfg_s *sbrCfg)
{
    memcpy(sbrCfg, &uartConsoleBasicSetting, sizeof(*sbrCfg));
    return EXIT_SUCCESS;
}

static S32 uartDevCfgGet(DevList_e devId, SbrUartCfg_s *sbrCfg)
{
    S32 ret = EXIT_SUCCESS;
    U32 readSize = 0;

    if (sbrCfg == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (devId == SYS_CONSOLE_UART_DEV) {
        return uartConsoleDevCfgGet(devId, sbrCfg); ///< overload uartDevCfgGet when it's console
    }

    readSize = devSbrRead(devId, (U8 *)sbrCfg, 0, sizeof(SbrUartCfg_s));
    if (readSize != sizeof(SbrUartCfg_s)) {
        LOGE("uart: failed to read UART config from SBR, readSize=%u, expected=%u\r\n",
             readSize, (U32)sizeof(SbrUartCfg_s));
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("uart: SBR config loaded - regAddr:%p, irq:%u, prio:%u, baud:%u, bits:%u:%u:%u\r\n",
         sbrCfg->regAddr, sbrCfg->irqNo, sbrCfg->irqPrio, sbrCfg->baudRate,
         sbrCfg->dataBits, sbrCfg->stopBits, sbrCfg->parity);
    LOGI("uart: SBR dump - console:%u, txIntEn:%u, rxIntEn:%u, reserved:%u\r\n",
         sbrCfg->console, sbrCfg->txIntEn, sbrCfg->rxIntEn, sbrCfg->reserved);
#endif

    if (sbrCfg->regAddr == NULL || sbrCfg->irqNo == 0 || sbrCfg->irqPrio == 0) {
        LOGE("uart: invalid SBR config - regAddr:%p, irqNo:%u, irqPrio:%u\r\n",
             sbrCfg->regAddr, sbrCfg->irqNo, sbrCfg->irqPrio);
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

static void uartIsr(void *arg)
{
    UartDrvData_s *drv = (UartDrvData_s *)arg;
    U8 recvData[UART_RX_FIFO_SIZE];
    U32 i, cnt;

    if (drv == NULL || drv->reg == NULL) {
        return;
    }

   if (drv->reg->status.fields.rxIrq) {
        drv->reg->status.fields.rxIrq = 1; ///< 清rx fifo irq中断
        while (drv->reg->fifo.fields.rxFifoLevel) {
            cnt = 0;
            for (i = 0; i < UART_RX_FIFO_SIZE; i++) {
                if (0 == drv->reg->fifo.fields.rxFifoLevel) {
                    break;
                }
                recvData[cnt++] = (U8)drv->reg->data;
            }
            if (drv->rxCallback) {
                drv->rxCallback(drv->devId, 0, recvData, cnt);
            }
        }
    }
    if (drv->reg->status.fields.txIrq) {
        drv->reg->status.fields.txIrq = 1;
        if (drv->txCallback) {
            drv->txCallback(drv->devId, 0);
        }
    }
}

static S32 uartBaudSet(UartDrvData_s *drv, U32 baud)
{
    S32 ret = EXIT_SUCCESS;
    float rate;
    U32 clk;
    U32 rateInt;
    U32 rateFrac;
    U32 startMidpoint;

    if (drv == NULL || drv->reg == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if (!baud) {
        ret = -EINVAL;
        goto exit;
    }

    if (peripsClockFreqGet(drv->devId, &clk) != EXIT_SUCCESS) {
        ret = -EIO;
        goto exit;
    }

    rateInt = clk / baud;
    rate = clk / (float)(baud);
    startMidpoint = (rate / 2) + 1;
    rate -= rateInt;
    rate *= 10;
    rateFrac = (U32)rate;

    drv->reg->aux.dword = rateInt;
    drv->reg->comps.fields.baudCompensateValue = rateFrac;
    drv->reg->comps.fields.startBitMidpoint = startMidpoint;

exit:
    return ret;
}

static S32 uartParamSet(UartDrvData_s *drv)
{
    S32 ret = EXIT_SUCCESS;

    if (drv == NULL || drv->reg == NULL) {
        ret = -EINVAL;
        goto exit;
    }
    if ((drv->setting.dataLen < UART_DATA_LEN_5) || (drv->setting.dataLen > UART_DATA_LEN_8)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((drv->setting.stopLen != UART_STOP_1) && (drv->setting.stopLen != UART_STOP_2)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((drv->setting.parity < UART_PARITY_NONE) || (drv->setting.parity > UART_PARITY_EVEN)) {
        ret = -EINVAL;
        goto exit;
    }
    if ((drv->setting.baud < UART_MIN_BAUD_RATE) || (drv->setting.baud > UART_MAX_BAUD_RATE)) {
        ret = -EINVAL;
        goto exit;
    }

    ///< 配置uart FIFO reg
    drv->reg->fifo.fields.rxIrqFifoLevel = 0;
    drv->reg->fifo.fields.txIrqFifoLevel = 1;

    ///< 配置uart ctrl reg - 使用默认参数
    drv->reg->ctrl.dword = ctrlRegDefault.dword;

    ///< 设置数据位
    drv->reg->ctrl.fields.dataLength = drv->setting.dataLen;

    ///< 设置停止位
    drv->reg->ctrl.fields.stopLength = drv->setting.stopLen;

    ///< 设置校验位
    drv->reg->ctrl.fields.parityType = drv->setting.parity;

    ///< 设置中断使能
    drv->reg->ctrl.fields.txIrqEn = drv->setting.enableTxInterrupt ? 1 : 0;
    drv->reg->ctrl.fields.rxIrqEn = drv->setting.enableRxInterrupt ? 1 : 0;

    ///< 设置波特率
    if (uartBaudSet(drv, drv->setting.baud) != EXIT_SUCCESS) {
        LOGE("uart: uartBaudSet failed\r\n");
        ret = -EIO;
        goto exit;
    }

exit:
    return ret;
}

S32 uartInitHelper(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (isDrvInit(devId)) {
        ret = -EBUSY;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (peripsClockEnable(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
        goto unlock;
    }

    drv = (UartDrvData_s *)calloc(1, sizeof(UartDrvData_s));
    if (drv == NULL) {
        ret = -ENOMEM;
        goto unlock;
    }

    if (uartDevCfgGet(devId, &drv->sbrCfg) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    drv->devId = devId;
    drv->reg = (UartReg_s *)drv->sbrCfg.regAddr;

    ///< 从SBR配置转换为内部配置
    drv->setting.baud = drv->sbrCfg.baudRate;
    drv->setting.dataLen = drv->sbrCfg.dataBits;
    drv->setting.stopLen = drv->sbrCfg.stopBits;
    drv->setting.parity = drv->sbrCfg.parity;
    drv->setting.enableTxInterrupt = drv->sbrCfg.txIntEn ? true : false;
    drv->setting.enableRxInterrupt = drv->sbrCfg.rxIntEn ? true : false;

    ///< 设置UART参数
    if (uartParamSet(drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto freeMem;
    }

    ///< 注册中断处理函数
    if (ospInterruptHandlerInstall(drv->sbrCfg.irqNo, "uart", OSP_INTERRUPT_UNIQUE, uartIsr, drv) != OSP_SUCCESSFUL) {
        ret = -EIO;
        goto freeMem;
    }

    if (arm_gic_irq_set_priority(drv->sbrCfg.irqNo, drv->sbrCfg.irqPrio) != 0) {
        ret = -EIO;
        goto removeIsr;
    }

    ospInterruptVectorEnable(drv->sbrCfg.irqNo);

    ///< 最后安装设备驱动
    if (drvInstall(devId, (void *)drv) != EXIT_SUCCESS) {
        ret = -EIO;
        goto disableVector;
    }

    ret = EXIT_SUCCESS;
    goto unlock;

disableVector:
    ospInterruptVectorDisable(drv->sbrCfg.irqNo);
removeIsr:
    ospInterruptHandlerRemove(drv->sbrCfg.irqNo, uartIsr, drv);
freeMem:
    free(drv);
unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 uartInit(DevList_e devId)
{
    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        return -EINVAL;
    }
    return uartInitHelper(devId);
}

S32 uartDeinit(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;

    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    ///< 禁用中断并移除处理函数
    ospInterruptVectorDisable(drv->sbrCfg.irqNo);
    if (ospInterruptHandlerRemove(drv->sbrCfg.irqNo, uartIsr, drv) != OSP_SUCCESSFUL) {
        LOGE("uart: ospInterruptHandlerRemove failed for uart:%d\r\n", devId);
        ret = -EIO;
    }

    ///< 复位设备
    if (peripsReset(devId) != EXIT_SUCCESS) {
        ret = -EIO;
    }

    ///< 卸载设备驱动
    if (drvUninstall(devId) != EXIT_SUCCESS) {
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 uartSetCfg(DevList_e devId, UartBasicSetting_s *cfg)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;

    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (!cfg) {
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    ///< 更新设置
    memcpy(&drv->setting, cfg, sizeof(drv->setting));

    ///< 重新配置UART参数
    if (uartParamSet(drv) != EXIT_SUCCESS) {
        ret = -EIO;
        LOGE("uart: uartParamSet failed\r\n");
    }

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 uartGetCfg(DevList_e devId, UartBasicSetting_s *cfg)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;

    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (!cfg) {
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    memcpy(cfg, &drv->setting, sizeof(drv->setting));

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

static U32 uartDeviceRead(UartDrvData_s *drv, void *buffer, size_t count, S32 timeout)
{
    U32 rxCnt = 0;
    U8 *rxBuf = buffer;
    U32 timeoutTicks, timerElapse, timerStart;
    Bool isWaitForEver = (timeout < 0) ? true : false;

    if (drv == NULL || drv->reg == NULL || buffer == NULL) {
        goto exit;
    }
    if (drv->setting.enableRxInterrupt) {
        LOGE("uart: rx interrupt is enabled, cannot use read function\r\n");
        goto exit;
    }
    if (!isWaitForEver) {
        ///< transfer ms to tick
        U32 msPerTick = rtems_configuration_get_milliseconds_per_tick();
        if (msPerTick == 0) {
            msPerTick = 1; ///< 防止除0错误
        }
        timeoutTicks = timeout / msPerTick;
        timerStart = sysTickGet();
    }
    while (1) {
        if (drv->reg->fifo.fields.rxFifoLevel) {
            if (rxCnt >= count) {
                break;
            }
            rxBuf[rxCnt++] = (U8)drv->reg->data;
        }
        if (!isWaitForEver) {
            timerElapse = sysTickGet() - timerStart;
            if (timerElapse > timeoutTicks) {
                break;
            }
        }
    }
exit:
    return rxCnt;
}

static S32 uartDeviceWrite(UartDrvData_s *drv, const void *buffer, size_t count, S32 timeout)
{
    S32 txCnt = 0;
    U8 *txBuf = (U8 *)buffer;
    U32 timeoutTicks, timerElapse, timerStart;
    U32 txFifoRemain;
    Bool isWaitForEver = (timeout < 0) ? true : false;

    if (drv == NULL || drv->reg == NULL || buffer == NULL) {
        goto exit;
    }
    if (!isWaitForEver) {
        ///< transfer ms to tick
        U32 msPerTick = rtems_configuration_get_milliseconds_per_tick();
        if (msPerTick == 0) {
            msPerTick = 1; ///< 防止除0错误
        }
        timeoutTicks = timeout / msPerTick;
        timerStart = sysTickGet();
    }
    ///< send bytes
    while (1) {
        if (!isWaitForEver) {
            timerElapse = sysTickGet() - timerStart;
            if (timerElapse > timeoutTicks) {
                break;
            }
        }
        txFifoRemain = UART_TX_FIFO_SIZE - drv->reg->fifo.fields.txFifoLevel;
        if (0 == txFifoRemain) {
            continue;
        }
        if (txCnt >= count) {
            break;
        }
        drv->reg->data = txBuf[txCnt++];
    }
exit:
    return txCnt;
}

S32 uartSend(DevList_e devId, U8 *buf, U32 size, S32 timeout)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;

    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (!buf || size == 0) {
        ret = -EINVAL;
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    if (size != uartDeviceWrite(drv, buf, size, timeout)) {
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 uartReceive(DevList_e devId, U8 *buf, U32 size, S32 timeout)
{
    S32 rxCnt = 0;
    UartDrvData_s *drv = NULL;

    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (!buf || size == 0) {
        goto exit;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        goto unlock;
    }

    if (!drv) {
        goto unlock;
    }

    rxCnt = uartDeviceRead(drv, buf, size, timeout);

unlock:
    devUnlockByDriver(devId);
exit:
    return rxCnt;
}

static S32 uartRegisterCallback(UartDrvData_s *drv, UartCallbackOps_s *cbOps)
{
    S32 ret = EXIT_SUCCESS;
    Bool uartEnable = false;

    if (drv == NULL || drv->reg == NULL || cbOps == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    uartEnable = (drv->reg->ctrl.fields.ctrlEnable) ? true : false;
    if (uartEnable) {
        drv->reg->ctrl.fields.ctrlEnable = 0;
    }

    if (cbOps->isTxValid) {
        drv->txCallback = cbOps->txCallback;
    }
    if (cbOps->isRxValid) {
        drv->rxCallback = cbOps->rxCallback;
    }

    if (uartEnable) {
        drv->reg->ctrl.fields.ctrlEnable = 1;
    }

exit:
    return ret;
}

S32 uartRegisterTxCallback(DevList_e devId, UartTxCallback txCallback)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;
    UartCallbackOps_s cbOps = { 0 };

    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        ret = -EINVAL;
        goto exit;
    }

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    cbOps.isTxValid = true;
    cbOps.txCallback = txCallback;
    if (uartRegisterCallback(drv, &cbOps) != EXIT_SUCCESS) {
        LOGE("uart: uartRegisterCallback failed\r\n");
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

static S32 uartRegisterRxCallbackHelper(DevList_e devId, UartRxCallback rxCallback)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;
    UartCallbackOps_s cbOps = { 0 };

    if (!isDrvMatch(devId, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (devLockByDriver(devId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    cbOps.isRxValid = true;
    cbOps.rxCallback = rxCallback;
    if (uartRegisterCallback(drv, &cbOps) != EXIT_SUCCESS) {
        LOGE("uart: uartRegisterCallback failed\r\n");
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

S32 uartRegisterRxCallback(DevList_e devId, UartRxCallback rxCallback)
{
    ///< 跳过console UART
    if (devId == SYS_CONSOLE_UART_DEV) {
        return -EINVAL;
    }
    return uartRegisterRxCallbackHelper(devId, rxCallback);
}

///< ==================== Console UART Functions ====================
#ifdef CONSOLE_USE_INTERRUPTS
void UartConsoleRxCallback(DevList_e devId, U32 state, U8 *buf, U32 size)
{
    rtems_termios_tty *tty = ttyObjGet();

    if ((NULL != tty) && (size > 0)) {
        rtems_termios_enqueue_raw_characters(tty, (const char *)buf, (S32)size);
    }
}

S32 uartConsoleInit(void *tty)
{
    S32 ret = EXIT_SUCCESS;
    DevList_e uartConsoleID = SYS_CONSOLE_UART_DEV;

    ttyObjSet(tty);
    if (EXIT_SUCCESS != uartInitHelper(uartConsoleID)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
    ///< Register Rx callback for console uart
    if (EXIT_SUCCESS != uartRegisterRxCallbackHelper(uartConsoleID, UartConsoleRxCallback)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
exit:
    return ret;
}
#else ///< CONSOLE_USE_INTERRUPTS
S32 uartConsoleInit(void *tty)
{
    DevList_e uartConsoleID = SYS_CONSOLE_UART_DEV;

    ttyObjSet(tty);
    return uartInitHelper(uartConsoleID);
}
#endif ///< CONSOLE_USE_INTERRUPTS

S32 uartConsoleDeinit(void)
{
    S32 ret = EXIT_SUCCESS;
    UartDrvData_s *drv = NULL;
    DevList_e uartConsoleID = SYS_CONSOLE_UART_DEV;

#ifdef CONSOLE_USE_INTERRUPTS
    ///< Unregister Rx callback for console uart
    if (EXIT_SUCCESS != uartRegisterRxCallbackHelper(uartConsoleID, NULL)) {
        return -EINVAL;
    }
#endif ///< CONSOLE_USE_INTERRUPTS
    if (!isDrvMatch(uartConsoleID, DRV_ID_STARS_UART)) {
        return -EINVAL;
    }

    if (devLockByDriver(uartConsoleID, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (getDevDriver(uartConsoleID, (void **)&drv) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    if (!drv) {
        ret = -ENODEV;
        goto unlock;
    }

    ///< 禁用中断并移除处理函数
    ospInterruptVectorDisable(drv->sbrCfg.irqNo);
    if (ospInterruptHandlerRemove(drv->sbrCfg.irqNo, uartIsr, drv) != OSP_SUCCESSFUL) {
        LOGE("uart: ospInterruptHandlerRemove failed for uart:%d\r\n", uartConsoleID);
        ret = -EIO;
    }

    ///< 复位设备
    if (peripsReset(uartConsoleID) != EXIT_SUCCESS) {
        ret = -EIO;
    }

    ///< 卸载设备驱动
    if (drvUninstall(uartConsoleID) != EXIT_SUCCESS) {
        ret = -EIO;
    }

unlock:
    devUnlockByDriver(uartConsoleID);
exit:
    return ret;
}

S32 uartConsoleRead(void)
{
    S32 c = -1;
    DevList_e devId = SYS_CONSOLE_UART_DEV;
    UartDrvData_s *drv = NULL;

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        goto exit;
    }
    if (drv == NULL || drv->reg == NULL) {
        goto exit;
    }
    if (drv->reg->fifo.fields.rxFifoLevel) {
        c = drv->reg->data;
    }
exit:
    return c;
}

static U32 uartTxFifoFreeSize(UartDrvData_s *drv)
{
  return UART_TX_FIFO_SIZE - drv->reg->fifo.fields.txFifoLevel;
}

static Bool uartConsoleIsTxReady(UartDrvData_s *drv)
{
  return uartTxFifoFreeSize(drv) > 16;
}

void uartConsoleWrite(const S8 *buf, size_t n)
{
    S32 i;
    DevList_e devId = SYS_CONSOLE_UART_DEV;
    UartDrvData_s *drv = NULL;
    rtems_termios_tty *tty = ttyObjGet();

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        goto exit;
    }
    if (drv == NULL || drv->reg == NULL || buf == NULL) {
        goto exit;
    }
    for (i = 0; i < (S32)n; i++) {
        ///< 将buf[i]写入设备寄存器
        while (!uartConsoleIsTxReady(drv));
        drv->reg->data = buf[i];
    }
    if ((NULL != tty) && (n > 0)) {
        rtems_termios_dequeue_characters(tty, n);
    }
exit:
    return;
}

void uartPrintkWrite(S8 c)
{
    DevList_e devId = SYS_CONSOLE_UART_DEV;
    UartDrvData_s *drv = NULL;

    if (getDevDriver(devId, (void **)&drv) != EXIT_SUCCESS) {
        goto exit;
    }
    if (drv == NULL || drv->reg == NULL) {
        goto exit;
    }
    while (!uartConsoleIsTxReady(drv));
    drv->reg->data = c;
exit:
    return;
}

