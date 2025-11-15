/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_core.c
 * @author  王殿卫
 * @data    2021.04.10
 * @brief   cli api
 */

#ifndef __CLI_API_H__
#define __CLI_API_H__

#include "cli_session.h"
#include "cli_porting.h"
#include "cli_command.h"
#include "cli_types.h"
#include "cli_argument.h"
#include "cli_errno.h"
#include "cli_std.h"
#include "cli_string.h"
#include "cli_normal_output.h"
#include "cli_level.h"
#include "cli_core_api.h"

/**
 * @brief   命令运行结束后回收资源
 * @param   pThreadInput 线程入参,该线程为NULL
 * @return  void
 */
void cliCmdRunFinish(CmdPrivData_s *pPriv, CliSessionObj_s *pSession);

/**
 * @brief   获取cli cfg
 * @param   由cfg读出的cli字段
 * @return  void
 */
void cliCfgSet(void* pCliCfg);

/**
 * @brief   cliCore的初始化
 * @param   void
 * @return  CommCliStatus
 */
S32 cliInit(void);

/**
 * @brief        cmd处理完成之后打印
 * @param        [in]   pPriv       命令私有信息
 * @param        [in]   cmdStatus   命令运行状态
 * @return       void
 * @warning
 * @note         2021.04.23  王殿卫  新生成函数
 * @note
 */
void cliCmdStatusPrint(CmdPrivData_s *pPriv, S32 cmdStatus);

/**
 * @brief   获取cli总结构体指针
 * @param   void
 * @return  cli总结构体指针
 */
S32 cliSessionReg(CliSessionDevIo_s *pSessionDevInfo);

/**
 * @brief   创建uart session
 * @param   void
 * @return  errno
 */
S32 cliUartSessionCreate( void );

/**
 * @brief   关闭该session
 * @param   sessionId   [in]
 * @return  errno
 * @note
 */
S32 cliSessionClose(U32 sessionId);

#endif ///< __CLI_API_H__
