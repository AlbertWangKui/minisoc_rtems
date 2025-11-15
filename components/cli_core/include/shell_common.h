/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file    sw_cmd_common.h
 * @data    2021.11.30
 * @brief
 */

#ifndef __SHELL_COMMON_H__
#define __SHELL_COMMON_H__

#include "cli_core.h"

///< 记录寄存器命令输入的参数字符串
typedef struct swCliRegStr {
    S8 *pRegWidthStr;   ///< 寄存器命令参数width对应的字符串
    S8 *pRegTypeStr;    ///< 寄存器命令参数type对应的字符串
    S8 *pRegAddrStr;    ///< 寄存器命令参数addr对应的字符串
    S8 *pRegValueStr;   ///< 寄存器命令参数value对应的字符串
} swCliRegStr_s;

///< 记录reg mask命令输入的参数字符串
typedef struct swCliRegMaskStr {
    S8 *pRegMaskAddrStr;    ///< reg mask命令参数addr指定的字符串
    S8 *pRegMaskPosStr;     ///< reg mask命令参数pos指定的字符串
    S8 *pRegMaskBwStr;          ///< reg mask命令参数bw指定的字符串
    S8 *pRegMaskValueStr;       ///< reg mask命令参数value指定的字符串
} swCliRegMaskStr_s;

///< TODO:寄存器掩码设置，移植到片外mgl
typedef struct MglCtrlRegMsk {
    U32 addr;              ///< 起始地址
    U32 pos;               ///< 寄存器起始位
    U32 bw;                ///< 设置位宽
    U32 value;             ///< 寄存器起始位配值
} MglCtrlRegMsk_t;

typedef enum MglCtrlRegType {
    MglCtrlRegUnknown = 0,
    MglCtrlRegA,
    MglCtrlRegB,
    MglCtrlRegC,
    MglCtrlRegD,
} MglCtrlRegType_e;

typedef struct MglCtrlRegInfo {
    MglCtrlRegType_e type; ///< 寄存器类型
    U8  pad[4];
    U64 addr;              ///< 起始地址
    U32 width;             ///< 位宽
    U8  pad2[4];
    U64 value;             ///< 小端
} MglCtrlRegInfo_t;

/**
 * @brief  解析寄存器cli命令输入的字符串
 * @param  pRegInfo [out]   寄存器信息
 * @param  pRegStr  [out]   输入的寄存器命令的字符串
 * @param  pParam   [in]    命令参数结构体
 * @param  isSet    [in]    是否是写(若是写会多解析value值)
 * @return errno
 */
S32 swCliRegParamParse(MglCtrlRegInfo_t *pRegInfo, swCliRegStr_s *pRegStr,
        CliParam_s *pParam, Bool isSet);

/**
 * @brief        写寄存器命令回调
 * @param:       pReqBody      [in]      主机下发的req的消息体
 * @param:       reqBodyLen    [in]      主机下发的req的消息体长度
 * @param:       pRespBody     [in]      回复给主机的消息体
 * @param:       pRespBodyLen  [in/out]  回复给主机的消息体长度,传入的是期望长度,传出实际长度
 * @return       函数执行状态,成功返回0；失败返回错误码
 * @note
 */
S32 cliRegWrite(void *pReqBody, U32 reqBodyLen,  void *pRespBody, U32 *pRespBodyLen);

/**
 * @brief  解析寄存器mask配置cli命令输入的字符串
 * @param  pRegInfo [in] regMask设置信息
 * @param  pParam   [in] 参数
 * @return errno
 * @note: /comm set regMsk addr=xxx pos=xxx bw=xxx value=xxx
 */
S32 swCliRegMskParamParse(MglCtrlRegMsk_t *pRegInfo, swCliRegMaskStr_s *pCliRegStr, CliParam_s *pParam);

/**
 * @brief        写寄存器Msk命令回调
 * @param:       pReqBody      [in]      主机下发的req的消息体
 * @param:       reqBodyLen    [in]      主机下发的req的消息体长度
 * @param:       pRespBody     [in]      回复给主机的消息体
 * @param:       pRespBodyLen  [in/out]  回复给主机的消息体长度,传入的是期望长度,传出实际长度
 * @return       函数执行状态,成功返回0；失败返回错误码
 * @note
 */
S32 cliRegMskWrite(void *pReqBody, U32 reqBodyLen,  void *pRespBody, U32 *pRespBodyLen);

/**
 * @brief   获取cli总结构体指针
 * @param   void
 * @return  cli总结构体指针
 * @note    imt cmd中的handleCmd一律为该函数, 第一个参数的handleParam回调函数才为真正的命令处理函数
 * @note    而真正的参数均在第一个参数的followArgsList中,所以第一个参数的handleParam一定不能为空
 * @note    运行在cli cmd处理线程
 */
S32 swCliHandler(CmdPrivData_s *pPriv);

/**
 * @brief   通过key和参数数组来获取key对应的value
 * @param   pKey        [in]    key字符串
 * @param   pParamList  [in]    参数数组
 * @param   paramCount  [in]    参数数组的长度
 * @return  value对应的字符串 NULL表示不支持
 * @warn    调用者需保证paramCount要等于param数组的元素数量!!!!!!!!!!
 * @warn    可用于cli框架已经将参数解析并赋值到对应参数全局变量中
 */
S8 *swCliValueGetByKey(S8 *pKey, CliParam_s *pParamList, U8 paramCount);

#endif ///< __SHELL_COMMON_H__
