/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_core.c
 * @author  王殿卫
 * @data    2021.04.10
 * @brief   cli 的框架代码
 */

#ifndef __CLI_CORE_H__
#define __CLI_CORE_H__

#include <osp_common.h>
#include <osp_priority.h>
#include <osp_task.h>

#include "cli_std.h"
#include "cli_errno.h"
#include "cli_list.h"
#include "cli_session.h"
#include "cli_level.h"
#include "cli_argument.h"
#include "cli_command.h"
#include "cli_level.h"
#include "cli_session.h"
#include "cli_porting.h"

#define INPUT_MSG_QUEUE_DEPTH (3)
#define INPUT_MSG_QUEUE_MAX_SIZE (sizeof(ULong) * INPUT_MSG_QUEUE_DEPTH)
#define CLI_CMD_HANDLE_THREAD_NAME ( ospBuildName('C', 'L', 'I', 'S') )
#define CLI_MSG_QUEUE_WAIT_FOREVER (0x0)
#define CLI_THREAD_STACK_MINIMUM_SIZE (5 * 1024)

#define STRING_MATCH_SUCCESS (0)
#define CLI_INFO_LENGTH_COMPUTE(levelCount) (sizeof(CliInfo_s) + (sizeof(struct ListHead) * (levelCount)))

typedef S32 CommCliStatus;

typedef struct CliQueueInfo {
    U32 cliQueueKey;
    U32 cliQueueId;
} CliQueueInfo_s;

///< session info相关结构体
typedef struct CliInfo {
    intptr_t   cliCmdHandleThreadId;                ///< 命令处理线程id
    S32   cliCmdHandleThreadName;                   ///< 线程名称
    Bool  cliIsActive;                              ///< 是否一直运行
    U32   cliCmdRunCounter;                         ///< 运行命令计数器
    CliQueueInfo_s cliInputQueue;                   ///< 输入消息队列
    CliQueueInfo_s cliOutputQueue;                  ///< 输入消息队列
    CommandList_s cmdList;                          ///< cli cmd
} CliInfo_s;

/**
 * @brief   获取cli总结构体指针
 * @param   void
 * @return  cli总结构体指针
 */
CliInfo_s *cliInfoGet( void );

/**
 * @brief   获取cli cfg
 * @param   由cfg读出的cli字段
 * @return  void
 */
CliCfg_s *cliCfgGet(void);

/**
 * @brief   cliCore的初始化
 * @param   void
 * @return  errno
 */
S32 cliCoreInit(void);

#endif ///< __CLI_CORE_H__