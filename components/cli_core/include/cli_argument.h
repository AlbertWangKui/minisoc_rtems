/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_argument.h
 * @author    xiezhj
 * @date      2021.04.07
 * @brief     cli argument 头文件
 * @note
 */

#ifndef __CLI_ARGUMENT_H__
#define __CLI_ARGUMENT_H__

#include "cli_errno.h"
#include "cli_normal_output.h"
#include "cli_types.h"

#define PARAM_DELIMITER " "

#define PAGE_TOKEN              "page"
#define ALL_TOKEN               "all"
#define ALL_SIZE                3
#define PAGE_DEFAULT            3
#define OPTION_HELP_LONG        "help"
#define OPTION_HELP_SHORT       "-h"
#define OPTION_COMPLETE_LONG    "--complete-args"

/**
 * @brief   将命令字符串解析成argv
 * @param   pInputBuf [in] get到的字符串
 * @param   pInputBuf [in] session信息
 * @param   pArgc     [out] 解析到的参数个数
 * @param   argv      [out] 参数列表
 * @return  CommCliStatus          运行状态
 *          COMM_CLI_PARAM_EXCEED_NUM_LIMIT   参数数量超限
 *          COMM_CLI_PARAM_EXCEED_LEN_LIMIT   参数长度超限
 *          COMM_CLI_CMD_UNKNOWN              未知命令
 * @note    2021.04.08  xiezhj
 */
S32 cliStringParse(S8 *pInputBuf, S32 *pArgc, S8 *argv[], CmdPrivData_s *pPriv);

/**
 * @brief      获取参数列表中的额外参数，并从参数列表中删除额外参数
 * @param      [in] pArgument
 * @return     void
 * @warning
 * @note   额外参数为 -j json tree t d -h help
 * @note   从参数列表后面开始获取，遇到非额外参数时停止
 * @note       2021.04.14  xiezhj
 */
void cliGetExtraParameters(Argument_s *pArgument);

/**
 * @brief  获取输入参数列表的结构化对象
 * @param  argc   输入参数个数
 * @param  argv   输入参数列表
 * @return struct Argument*
 */
Argument_s *cliGetArgument(S32 argc, S8 *argv[],  CmdPrivData_s *pPriv);

/**
 * @brief   对用户输入的参数进行解析,输出到Param结构体数组中
 *          支持格式：key=value, key, key1|key2|key3
 * @param   argc, 参数个数
 * @param   argv [in], 参数字符串数组
 * @param   param [in/out], 结构体数组
 * @param   paramCount, Param结构体个数
 * @return  PS3_ERRNO_SUCCESS               成功
 * @warning 参数不包含"="时，将参数整体作为key，value为用户实际输入
 *          若param列表中包含'|'分隔符，则匹配到其中任意一项即为成功匹配
 * @note
 */
S32 cliParseParam(S32 argc, S8 *argv[], CliParam_s *pParamList, S32 paramCount);

/**
 * @brief      从param结构体中读取特定字段的值
 * @param      [in] paramList
 * @param      [in] paramCount
 * @param      [out] pKey
 * @return     CliParam_s
 * @warning
 * @note       2021.04.14  xiezhj
 */
CliParam_s *cliMatchParam(CliParam_s *pParamList, U32 paramCount, S8 *pKey);

/**
 * @brief      校验输入参数
 * @param      [in]
 * @param      [out]
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note       2021.04.10  xiezhj
 */
S32 cliCheckParam(CliParam_s *paramList, U32 paramCount, Node_s *pParent);

#endif ///< __CLI_ARGUMENT_H__