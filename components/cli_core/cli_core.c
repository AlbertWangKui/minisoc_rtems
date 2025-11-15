/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_core.c
 * @author  王殿卫
 * @data    2021.04.10
 * @brief   cli 的框架代码
 */
#include "cli_core.h"
#include "cli_porting.h"
#include "osp_queue.h"
#include "osp_semaphore.h"
#include "cli_porting.h"

static CliCfg_s *sgpCliCfg = NULL;

static CliInfo_s sgCliInfo = { 0 };

/**
 * @brief   获取cli总结构体指针
 * @param   void
 * @return  cli总结构体指针
 */
CliInfo_s *cliInfoGet(void)
{
    return &sgCliInfo;
}

/**
 * @brief   获取cli cfg
 * @param   由cfg读出的cli字段
 * @return  void
 */
void cliCfgSet(void* pCliCfg)
{
    sgpCliCfg = pCliCfg;
}

/**
 * @brief   获取cli cfg
 * @param   由cfg读出的cli字段
 * @return  void
 */
CliCfg_s *cliCfgGet(void)
{
    return sgpCliCfg;
}

/**
 * @brief        创建打印父节点
 * @param        pPriv->pNode [out]   创建的父节点
 * @return       errno
 * @warning
 * @note         2021.04.23  wangdw
 */
static S32 cliCreatPrintParentNode(CmdPrivData_s *pPriv, CliSessionObj_s *pSession)
{
    S32 rc = -CLI_ERRNO_FAILED;
//    HalTimeOfDay_s timeInfo;
    Node_s *pPrintParent = NULL;

//    memset(&timeInfo, 0, sizeof(timeInfo));
    ///< 创建打印父节点
    pPrintParent = cliNodeCreatObject(NULL, "system");

    if (NULL == pPrintParent) {
        rc = -CLI_ERRNO_CREAT_NODE_FAIL;
        CLI_LOG_ERROR("Create node error, sessionId = %d, rc = %08x, cmd = %s",
                pPriv->sessionId, rc, pPriv->pCmd->name);
        goto lEnd;
    }

    cliNodeHideTitle(pPrintParent);

    ///< 创建打印版本(编译时间)
    cliNodeAddKeyStringF(pPrintParent, "Version", "%s", CLI_BUILD_TIME);

#if 0
    ///< 创建打印运行时间
    (void)halGetTimeOfDay(&timeInfo);
    cliNodeAddKeyStringF(pPrintParent, "System Time", "%.4d-%.2d-%.2d %.2d:%.2d:%.2d",
                        timeInfo.year, timeInfo.month, timeInfo.day,
                        timeInfo.hour, timeInfo.minute, timeInfo.second);
#endif

    ///< 创建打印命令运行状态
    cliNodeAddKeyStringF(pPrintParent, "Status", "Failed");
    ///< 创建打印命令运行状态描述
    cliNodeAddKeyStringF(pPrintParent, "Description", "common failed");

    pSession->pPrintParent = pPrintParent;
    ///< TODO 打印其他
    pPriv->pParent = pPrintParent;
    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief   cli 字符串解析线程处理函数
 * @param   pThreadInput 线程入参,该线程为NULL
 * @return  void
 */
static void cliStringHandle(void *pThreadInput)
{
    CliInfo_s *pCliInfo         = NULL;
    CliInputMsg_s *pInputMsg    = NULL;
    ULong inputMsgAddr          = 0;
    U32 inputlen                = sizeof(inputMsgAddr);
    S32 rc                      = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession   = NULL;
    S32 argc                    = 0;
    S8 *argv[CLI_MAX_PARAM_NUM] = {NULL};
    Argument_s *pArgument       = NULL;
    CmdPrivData_s *pPriv        = NULL;

    pCliInfo = cliInfoGet();

    do {
        ///< 初始化argv pInputMsg
        cliMemSet (argv, 0, (S32)(sizeof(S8 *) * ARRAY_SIZE(argv)));
        argc = 0;

        ///< 从消息队列中取输入的参数信息
        rc = ospMessageQueueReceive(pCliInfo->cliInputQueue.cliQueueId, &inputMsgAddr, &inputlen, OSP_DEFAULT_OPTIONS, \
                CLI_MSG_QUEUE_WAIT_FOREVER);
        if (rc != CLI_ERRNO_OK || inputlen != sizeof(inputMsgAddr)) {
            CLI_LOG_ERROR("Cli receive from msg queue error, rc = %08x", rc);
            goto lEnd;
        }

        pInputMsg = (CliInputMsg_s *)inputMsgAddr;

        rc = cliGetSessionById(pInputMsg->sessionId, &pSession);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Get session info error, sessionId = %d, rc = %08x", pInputMsg->sessionId, rc);
            cliCmdRunFinish(pPriv, pSession);
            continue;
        }

        CLI_LOG_TRACE("SessionId = %d, sessionType=%08x, str = %s",
                    pInputMsg->sessionId, pSession->devIo.type, pInputMsg->str);

        ///< 获取session private结构体
        rc = cliCmdPrivDataGet(pSession, &pPriv);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Get private date error, err = %08x, sessionId = %d", rc, pInputMsg->sessionId);
            goto lEnd;
        }

        pPriv->sessionId = pInputMsg->sessionId;
        rc = cliCreatPrintParentNode(pPriv, pSession);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("CliCreatPrintParentNode err:%d", rc);
            cliCmdRunFinish(pPriv, pSession);
            continue;
        }

        ///< 解析字符串
        rc = cliStringParse(pInputMsg->str, &argc, argv, pPriv);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Cli parse input string error, rc = %08x, str = %s", rc, pInputMsg->str);
            cliCmdStatusPrint(pPriv, rc);
            cliNodeAddStringVal(pPriv->pParent, "Please enter the correct cmd.\n");
            cliPrintUsage(pPriv->sessionId);
            cliCmdRunFinish(pPriv, pSession);
            continue;
        }

        ///< 获取argument结构体
        pArgument = cliGetArgument(argc, argv, pPriv);
        if (NULL == pArgument) {
            rc = -CLI_ERRNO_ARGUMENT_INVALID;
            CLI_LOG_ERROR("CiGetArgument failed, rc:%08x.", rc);
            cliNodeAddStringVal(pPriv->pParent, "Please enter the correct cmd.\n");
            cliPrintUsage(pPriv->sessionId);
            cliCmdStatusPrint(pPriv, rc);
            cliCmdRunFinish(pPriv, pSession);
            continue;
        }

        pPriv->pArg = pArgument;

        ///< 处理命令  分发各个模块, 并完成资源释放
        ///< cliCmdStatusPrint函数调用在此函数中,此处不再调用
        rc = cliHandleCommand(pPriv, pSession);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Cli handle command error, rc = %d", rc);
        }

    } while (true == pCliInfo->cliIsActive);

lEnd:
    pCliInfo->cliIsActive = false;
    ospTaskExit();
    return;
}

/**
 * @brief   cliCore的初始化
 * @param   void
 * @return  errno
 */
S32 cliCoreInit(void)
{
    S32 rc      = -CLI_ERRNO_FAILED;
    S32 status  = -CLI_ERRNO_FAILED;
    CliInfo_s *pCliInfo = NULL;
    U32 id = 0;

    pCliInfo = cliInfoGet();

    ///< 建队列
    pCliInfo->cliInputQueue.cliQueueKey = (U32)ospBuildName('C', 'L', 'I', 'C');
    rc = ospMessageQueueCreate(pCliInfo->cliInputQueue.cliQueueKey, INPUT_MSG_QUEUE_DEPTH, \
            INPUT_MSG_QUEUE_MAX_SIZE, OSP_DEFAULT_ATTRIBUTES, &(pCliInfo->cliInputQueue.cliQueueId));
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Message queue create error, rc = %d, key = %d, id = %d", rc, \
                pCliInfo->cliInputQueue.cliQueueKey, pCliInfo->cliInputQueue.cliQueueId);
        goto lEnd;
    }

    ///< 创建命令处理线程
    pCliInfo->cliCmdHandleThreadName = (S32)CLI_CMD_HANDLE_THREAD_NAME;
    if (OSP_SUCCESSFUL != ospTaskCreate((U32)pCliInfo->cliCmdHandleThreadName, USER_TASK_PRIORITY_NORMAL,
        CLI_THREAD_STACK_MINIMUM_SIZE, OSP_DEFAULT_MODES, OSP_DEFAULT_ATTRIBUTES, &id)) {
        CLI_LOG_ERROR("Create cmd handle thread error\r\n");
        goto lDestoryQueue;
    }
    if (OSP_SUCCESSFUL != ospTaskStart(id, (OspTaskEntry)cliStringHandle, (U32)NULL)) {
        CLI_LOG_ERROR("start cmd handle thread error\r\n");
        goto lDestoryQueue;
    }
    rc = CLI_ERRNO_OK;
    goto lEnd;

lDestoryQueue:
    status = ospMessageQueueDelete(pCliInfo->cliInputQueue.cliQueueId);
    if (status != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Delete cli queue error, rc = %d", status);
    }

lEnd:
    return rc;
}

/**
 * @brief   cliCore的初始化
 * @param   void
 * @return  CommCliStatus
 */
S32 cliInit(void)
{
    S32 rc = -CLI_ERRNO_FAILED;
    CliInfo_s *pCliInfo = NULL;
    pCliInfo = cliInfoGet();

    ///< 若没有初始化  进行初始化
    if (false == pCliInfo->cliIsActive) {

        cliMemSet(pCliInfo, 0, (S32)sizeof(*pCliInfo));
        ///< 初始化user
        rc = cliUserCfgInit();
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("CliUserCfgInit err:%08x", rc);
            goto lEnd;
        }

        ///< 初始化 core
        rc = cliCoreInit();
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("CliCoreInit err:%08x", rc);
            goto lEnd;
        }

        ///< TODO 注册部分命令

        ///< 初始化session
        cliSessionInit();

        pCliInfo->cliIsActive = true;
    } else {
        ///< 若已经初始化 打印log直接退出
        CLI_LOG_WARN("Cli init finished");
    }

    rc = CLI_ERRNO_OK;

lEnd:
    return rc;

}

/**
 * @brief   命令运行结束后回收资源
 * @param   pThreadInput 线程入参,该线程为NULL
 * @return  void
 */
void cliCmdRunFinish(CmdPrivData_s *pPriv, CliSessionObj_s *pSession)
{
    S32 status  = -CLI_ERRNO_FAILED;

    if (pSession == NULL) {
        CLI_LOG_WARN("session null");
        goto lEnd;
    }

    if (pPriv == NULL) {
        CLI_LOG_WARN("resource destroyed");
        goto lEnd;
    }

    if ((pPriv != NULL) && (pPriv->pArg != NULL)) {
        cliFree(pPriv->pArg);
        pPriv->pArg = NULL;
    }

    status = cliCmdPrivDataPut(pSession);
    if (status != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("CliCmdPrivDataPut err:%08x", status);
    }

lEnd:
    status = ospSemaphoreRelease(pSession->sessionSem);
    if (status != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("HalSemV err:%08x", status);
    }

    return;

}
