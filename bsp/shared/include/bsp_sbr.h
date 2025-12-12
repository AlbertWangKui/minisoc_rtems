#ifndef __BSP_SBR_H__
#define __BSP_SBR_H__

#include "common_defines.h"
#include "bsp_device.h"

/* UART配置结构体 */
typedef struct SbrUartCfg {
    void *regAddr;
    U16 irqNo;
    U16 irqPrio;
    U32 baudRate;
    U8 dataBits;
    U8 stopBits;  /* 停止位 */
    U8 parity;    /* 校验位 */
    U8 console;   /* 控制台标志 */
    U8 txIntEn;   /* 发送中断使能 */
    U8 rxIntEn;   /* 接收中断使能 */
    U8 reserved;  /* 保留字段 */
} __attribute__((packed)) SbrUartCfg_s;

/* I2C & SMBUS 配置结构体 */
typedef struct SbrI2cSmbusCfg {
    void *regAddr;
    U16 irqNo;
    U16 irqPrio;
    U8 masterMode;       /* 模式: 0=slave, 1=master */
    U8 interruptMode;    /* 工作模式: 0=poll, 1=interrupt */
    U8 speed;            /* 速度: 0=standard, 1=fast, 2=fast_plus, 3=high, 4=ultra */
    U8 addrMode;         /* 地址长度, 0=7bit, 1=10bit */
    U8 slaveAddrHigh;       /* 从机地址高8位 */
    U8 slaveAddrLow;        /* 从机地址低8位 */
    U8 enSmbus;          /* SMBus模式: 0=i2c, 1=smbus */
    U8 reserved;         /* 保留字段 */
} __attribute__((packed)) SbrI2cSmbusCfg_s;

typedef struct SbrGpioCfg {
    void *regAddr; /* 寄存器地址 */
    U16 irqNo; /* 中断号 */
    U8 irqPrio; /* 中断优先级 */
    U16 totalPinCnt; /* 芯片总引脚数 */
    U32 dir; /* 方向,0-input,1-output */
    U32 dataOut; /* 输出数据,0-low,1-high */
} __attribute__((packed)) SbrGpioCfg_s;

/* SGPIO配置结构体 */
typedef struct SbrSgpioCfg {
    void *regAddr;
    U8 mode;    /* 模式,0 == sff8485 normal, 1 == sff8485 gp, 2 == sta1005 */
    U8 blinkEn; /* blink使能,0 == enable, 1 == disable */
    U8 blinkMode; /* blink模式,0 == from external input trig, 1 == sio self trigger */
    U8 blinkFreq; /* blink频率, 单位Hz */
    U8 blinkOutSel; /* blink输出选择,0 == sgpio0, 1 == sgpio1, 2 == sgpio2, 3 == sgpio3 */
    U32 ActivityMasklow; /* Activity Mask low 32 bits, 0 == mask, 1 == unmask */
    U32 ActivityMaskhigh; /* Activity Mask high 32 bits, 0 == mask, 1 == unmask */
    U32 cfg1Timing;   /*  [31:28] stretch_activity_off, [27:24] stretch_activity_on, [23:20] force_activity_off，
                [19:16] force_activity_on, [15:12] pattern B blink freq, [11:8] pattern A blink freq, [7:0] rsv*/
    U32 maxClk;   /* 最大时钟,100000 单位Hz*/
    U32 cfgClk;   /* 配置时钟,10000 单位Hz*/
    U8 driveNum;    /* 驱动数量,81 */
    U8 irqEn;      /* 中断使能 */
    U16 irqNum;    /* 中断号 */
    U16 irqPrio;    /* 中断优先级 */
    U32 reserved;   /* 保留字段 */
} __attribute__((packed)) SbrSgpioCfg_s;

/* TRNG配置结构体 */
typedef struct {
    void *regAddr;
    U16 irqNo;
    U16 irqPrio;
    U32 reserved;
} __attribute__((packed)) SbrTrngCfg_s;

/* SM3 SBR Configuration Structure */
typedef struct SbrSm3Cfg {
    void *regAddr;      /* Register base address */
    U32 irqNo;          /* Interrupt number */
    U32 irqPrio;        /* Interrupt priority */
    U32 reserved;       /* Reserved for future use */
} __attribute__((packed)) SbrSm3Cfg_s;

/* WDT配置结构体 */
typedef struct SbrWdtCfg {
    void *regAddr;
    U16 irqNo;
    U8 irqPrio;
    U8 workMode;
    U16 feedTime;    ///< ms
    U16 maxFeedTime; ///< ms
    U16 div;
    U32 reserved;
} __attribute__((packed)) SbrWdtCfg_s;

typedef struct SbrEfuseCfg {
    U32 regAddr;
    U32 reserved;
} __attribute__((packed)) SbrEfuseCfg_s;

/* PVT配置结构体 */
typedef struct {
    U8 totalCh;        /* 总通道数 */
    U8 reserved[3];    /* 保留字段 */
} __attribute__((packed)) pvtCfg_s;

/* PVT通道配置结构体 */
typedef struct SbrPvtChCfg {
    void *regAddr;
    void *irqAddr;
    void *tmonThresholdIrqAddr;
    U16 irqNo;
    U16 irqPrio;
    U8 tmon;        /* TMON使能 */
    U8 dts;         /* DTS使能 */
    U8 dtsTempPoints; /* DTS温度点数 */
    U8 dtsVolPoints;  /* DTS电压点数 */
    U8 dtsCfgLatency; /* DTS配置延迟 */
    U8 tmonTempTh;     /* TMON温度阈值 */
    U16 dtsTempTh;     /* DTS温度阈值 */
} __attribute__((packed)) SbrPvtChCfg_s;

/* PWM配置结构体 */
typedef struct SbrPwmCfg {
    void *regAddr;
    U32 irqNo;
    U32 irqPrio;
    U8 chNum;          /* 通道数量 */
} __attribute__((packed)) SbrPwmCfg_s;

/* TACH配置结构体 */
typedef struct SbrTachCfg {
    void *regAddr;
    U32 irqNo;
    U32 irqPrio;
    U8 polarity;  ///< postive edge: 0, negative edge: 1
} __attribute__((packed)) SbrTachCfg_s;

typedef struct SbrTimerCfg {
    void *regAddr;
    U32 irqNo;
    U32 irqPrio;
    U32 intervalMs;
    U32 reserved;
} __attribute__((packed)) SbrTimerCfg_s;

/* IOMUX配置结构体 */
typedef struct SbrBrdCfg {
    U32 cfgNum;
    struct {
        U32 addr;
        U32 val;
    } brdCfg[16];
} __attribute__((packed)) SbrBrdCfg_s;

typedef struct SbrOcmEccCfg {
    U32 eccAddr;
    U16 irqNo;
    U8 irqPrio;
    U32 reserved;
} __attribute__((packed)) SbrOcmEccCfg_s;

typedef struct {
    void *regAddr;
    U32 irqNo;
    U32 irqPrio;
    U32 reserved;
} __attribute__((packed)) SbrSm2Cfg_s;

/* sbrData_s and sbrData_u are used to store the configuration data for the devices
 * and must be defined in the sbr.h file
 */
typedef struct {
    /* UART设备配置 */
    SbrUartCfg_s sbrUart0Cfg;       /* UART0配置 */
    SbrUartCfg_s sbrUart1Cfg;       /* UART1配置 */
    SbrUartCfg_s sbrUart2Cfg;       /* UART2配置 */
    SbrUartCfg_s sbrUart3Cfg;       /* UART3配置 */

    /* I2C设备配置 */
    SbrI2cSmbusCfg_s sbrI2c0Cfg;         /* I2C0配置 */
    SbrI2cSmbusCfg_s sbrI2c1Cfg;         /* I2C1配置 */
    SbrI2cSmbusCfg_s sbrI2c2Cfg;         /* I2C2配置 */
    SbrI2cSmbusCfg_s sbrI2c3Cfg;         /* I2C3配置 */
    SbrI2cSmbusCfg_s smbus0Cfg;     /* SMBUS0配置 */
    SbrI2cSmbusCfg_s smbus1Cfg;     /* SMBUS1配置 */

    /* GPIO设备配置 */
    SbrGpioCfg_s sbrGpio0Cfg;       /* GPIO0配置 */
    SbrGpioCfg_s sbrGpio1Cfg;       /* GPIO1配置 */

    /* 其他设备配置 */
    SbrSgpioCfg_s sgpioCfg;       /* SGPIO0配置 */
    SbrTrngCfg_s sbrTrngCfg;   /* TRNG0配置 */
    SbrSm3Cfg_s sbrSm3Cfg;     /* SM3配置 */
    SbrWdtCfg_s SbrWdt0Cfg;     /* WDT0配置 */
    SbrWdtCfg_s SbrWdt1Cfg;     /* WDT1配置 */
    SbrWdtCfg_s SbrWdt2Cfg;     /* WDT2配置 */
    SbrWdtCfg_s SbrWdt3Cfg;     /* WDT3配置 */
    SbrEfuseCfg_s SbrEfuseCfg; /* EFUSE0配置 */
    pvtCfg_s pvtCfg;           /* PVT0配置 */
    SbrPvtChCfg_s sbrPvtCh0Cfg;     /* PVT通道0配置 */
    SbrPvtChCfg_s sbrPvtCh1Cfg;     /* PVT通道1配置 */
    SbrPvtChCfg_s sbrPvtCh2Cfg;     /* PVT通道2配置 */
    SbrPvtChCfg_s sbrPvtCh3Cfg;     /* PVT通道3配置 */
    SbrPvtChCfg_s sbrPvtCh4Cfg;     /* PVT通道4配置 */
    SbrPvtChCfg_s sbrPvtCh5Cfg;     /* PVT通道5配置 */
    SbrPwmCfg_s sbrPwmCfg;           /* PWM0配置 */
    SbrTachCfg_s sbrTach0Cfg;           /* TACH0配置 */
    SbrTachCfg_s sbrTach1Cfg;           /* TACH0配置 */
    SbrTachCfg_s sbrTach2Cfg;           /* TACH0配置 */
    SbrTachCfg_s sbrTach3Cfg;           /* TACH0配置 */
    SbrBrdCfg_s brdCfg;       /* 通用配置 */
} __attribute__((packed)) sbrData_s;

typedef union {
    /* UART设备配置 */
    SbrUartCfg_s sbrUart0Cfg;       /* UART0配置 */
    SbrUartCfg_s sbrUart1Cfg;       /* UART1配置 */
    SbrUartCfg_s sbrUart2Cfg;       /* UART2配置 */
    SbrUartCfg_s sbrUart3Cfg;       /* UART3配置 */

    /* I2C设备配置 */
    SbrI2cSmbusCfg_s sbrI2c0Cfg;         /* I2C0配置 */
    SbrI2cSmbusCfg_s sbrI2c1Cfg;         /* I2C1配置 */
    SbrI2cSmbusCfg_s sbrI2c2Cfg;         /* I2C2配置 */
    SbrI2cSmbusCfg_s sbrI2c3Cfg;         /* I2C3配置 */
    SbrI2cSmbusCfg_s smbus0Cfg;     /* SMBUS0配置 */
    SbrI2cSmbusCfg_s smbus1Cfg;     /* SMBUS1配置 */

    /* GPIO设备配置 */
    SbrGpioCfg_s sbrGpio0Cfg;       /* GPIO0配置 */
    SbrGpioCfg_s sbrGpio1Cfg;       /* GPIO1配置 */

    /* 其他设备配置 */
    SbrSgpioCfg_s sgpioCfg;       /* SGPIO0配置 */
    SbrTrngCfg_s sbrTrngCfg;   /* TRNG0配置 */
    SbrSm3Cfg_s sbrSm3Cfg;     /* SM3配置 */
    SbrWdtCfg_s SbrWdtCfg;     /* WDT0配置 */
    SbrEfuseCfg_s SbrEfuseCfg; /* EFUSE0配置 */
    pvtCfg_s pvtCfg;           /* PVT0配置 */
    SbrPvtChCfg_s sbrPvtCh0Cfg;     /* PVT通道0配置 */
    SbrPvtChCfg_s sbrPvtCh1Cfg;     /* PVT通道1配置 */
    SbrPvtChCfg_s sbrPvtCh2Cfg;     /* PVT通道2配置 */
    SbrPvtChCfg_s sbrPvtCh3Cfg;     /* PVT通道3配置 */
    SbrPvtChCfg_s sbrPvtCh4Cfg;     /* PVT通道4配置 */
    SbrPvtChCfg_s sbrPvtCh5Cfg;     /* PVT通道5配置 */
    SbrPwmCfg_s sbrPwmCfg;           /* PWM0配置 */
    SbrTachCfg_s sbrTach0Cfg;           /* TACH0配置 */
    SbrTachCfg_s sbrTach1Cfg;           /* TACH0配置 */
    SbrTachCfg_s sbrTach2Cfg;           /* TACH0配置 */
    SbrTachCfg_s sbrTach3Cfg;           /* TACH0配置 */
    SbrTimerCfg_s sbrTimerCfg;       /* TIMER0配置 */
    SbrSm2Cfg_s sbrSm2Cfg;           /* SM2配置 */
    SbrBrdCfg_s brdCfg;       /* 通用配置 */
} __attribute__((packed)) sbrData_u;
typedef struct SbrSm4Cfg {
    void *regAddr; /* 寄存器地址 */
    U16  irqNo; /* 中断号 */
    U8   irqPrio; /* 中断优先级 */
} __attribute__((packed)) SbrSm4Cfg_s;
#endif /* __SBR_H__ */
