/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_command.h
 * @author    xiezhj
 * @date      2021.04.07
 * @brief     cli command 头文件
 * @note
 */

#ifndef __CLI_COMMAND_H__
#define __CLI_COMMAND_H__

#include "cli_api.h"
#include "cli_types.h"

#define CLI_SEPARATOR "----------------------------------------------------------------------"

/**
 * @brief      命令注册函数
 * @param      [in]
 * @param      [out]
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note       2021.04.12  xiezhj
 */
void cliRegisterCmd(Command_s *pCmd);

/**
 * @brief        处理输入命令
 * @param        [in]   pArgument   输入命令
 * @param        [in]   pSession    session结构体
 * @return       EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note         2021.04.22  xiezhj
 * @note         2020.04.28  王殿卫 修改:将获取cmd private变量提到cliStringHandle函数中
 * @note         2020.09.24  王殿卫 修改:将入参sessionId改为pSession
 */
S32 cliHandleCommand(CmdPrivData_s *pPriv, CliSessionObj_s *pSession);

#define CONNECT(v1, v2) v1##v2
#define RAND_NAME(name, line) CONNECT(name, line)

#define REGISTER_CMD(val)                                                      \
    static void __attribute__((constructor))                                   \
        RAND_NAME(cliRegisterCmd, __LINE__)( )                                 \
    {                                                                          \
        cliRegisterCmd(&val);                                                  \
        return;                                                              \
    }

/**
 * @brief        打印help list
 * @param        [in]
 * @param        [out]
 * @return       EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note         2021.04.22  xiezhj
 */
void cliPrintHelpList(U32 sessionId, Command_s *list[], U32 count);

/**
 * @brief        打印详细帮助信息
 * @param        [in]
 * @param        [out]
 * @return       EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note         2021.04.22  xiezhj
 */
void cliPrintDetailHelp(U32 sessionId, Command_s *pCmd);

/**
 * @brief        打印usage
 * @param        [in]
 * @param        [out]
 * @return       EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note         2021.04.23  xiezhj
 */
void cliPrintUsage(U32 sessionId);

#endif ///< __CLI_COMMAND_H__
