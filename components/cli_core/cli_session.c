/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_seesion.c
 * @author  王殿卫
 * @data    2021.04.07
 * @brief   cli session代码
 * @note    王殿卫新生成文件
 */

#include "cli_session.h"
#include "cli_porting.h"
#include "cli_core.h"
#include "cli_session.h"
#include "osp_queue.h"
#include "osp_semaphore.h"
#include "osp_timer.h"

static CliSessions_s sgCliSessions;
static Bool sIsLOGEnOpen = true;

/**
 * @brief   获取session信息
 * @param   void
 * @return  errno
 */
CliSessions_s *cliSessionsGet(void)
{
    return &sgCliSessions;
}

/**
 * @brief   获取是否开启登录功能状态
 * @param   void
 * @return  errno
 */
Bool cliLOGEnStatGet(void)
{
    return sIsLOGEnOpen;
}

/**
 * @brief   设置是否开启登录功
 * @param   void
 * @return  errno
 */
void cliLOGEnStatSet(Bool isOpen)
{
    sIsLOGEnOpen = isOpen;
}

/**
 * @brief   通过Id获取session结构体指针
 * @param   sessionId [in]
 * @param   ppSession [out] 读出的session结构体
 * @return  session结构体指针
 */
S32 cliGetSessionById(U32 sessionId, CliSessionObj_s **ppSession)
{
    CliSessions_s *pCliSessions = NULL;
    S32 rc = -CLI_ERRNO_FAILED;

    if (sessionId >= CLI_SESSION_MAX_COUNT) {
        rc = -CLI_ERRNO_SESSION_ID_UNKNOWN;
        CLI_LOG_ERROR("Cli session id is out of range, max is %d but input id is %d", \
                CLI_SESSION_MAX_COUNT, sessionId);
        goto lEnd;
    }

    pCliSessions = cliSessionsGet();
    if (sessionId < CLI_SESSION_MAX_COUNT) {
        *ppSession = &(pCliSessions->sessions[sessionId]);
    }

    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief   打印命令行的头
 * @param   sessionId [in]
 * @return  void
 */
void cliTitlePrint (CliSessionPutStringFunc pPutsFunc, U32 sessionId)
{
    cliSessionPrintf(sessionId, "\n" CLI_TITLE "\n\t\t\tPS3 CLI PROCESS \n" CLI_TITLE "\n");
}

/**
 * @brief   获取session状态
 * @param   sessionId       [in]    id
 * @param   status          [out]    要设置的状态
 * @return  errno
 */
SessionStatus_e sessionStatusGet(U32 sessionId)
{
    S32 rc = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;
    SessionStatus_e status;

    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session error, input session id = %d", sessionId);
        status = SESSION_STATUS_NR;
        goto lEnd;
    }

    status = pSession->sessionStatus;

lEnd:
    return status;
}

/**
 * @brief   设置session状态机
 * @param   sessionId       [in]    id
 * @param   status          [in]    要设置的状态
 * @return  errno
 */
void sessionStatusSet(U32 sessionId, SessionStatus_e status)
{
    S32 rc = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;

    if (status == SESSION_STATUS_NR) {
        CLI_LOG_ERROR("Session status unkown , sessionId = %d, status = %08x", sessionId, status);
        goto lEnd;
    }

    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session error rc = %08x, "
                      "input session id = %d", sessionId, rc);
        goto lEnd;
    }

    pSession->sessionStatus = status;

lEnd:
    return;
}

/**
 * @brief   获取session的私有数据buffer
 * @param   pSession        [in]   session 结构体
 * @param   ppSessionBuf    [out]  获取到的buffer
 * @return  errno
 */
S32 cliCmdPrivDataGet(CliSessionObj_s *pSession, CmdPrivData_s **ppCmdPriv)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if (NULL == pSession) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("pSession is null, rc:%08x", rc);
        goto lEnd;
    }

    if (pSession->cmdPrivData.callCount >= SESSION_PRIVATE_BUFFER_CALL_MAX) {
        rc = -CLI_ERRNO_SESSION_PRIVATE_DATA_UESED;
        CLI_LOG_ERROR("Session(id = %d) private data is in use, most number of user is %d, " \
                "now the num of user is %d", pSession->sessionId, SESSION_PRIVATE_BUFFER_CALL_MAX, \
                pSession->cmdPrivData.callCount);
        *ppCmdPriv = NULL;
        goto lEnd;
    }

    pSession->cmdPrivData.callCount++;
    *ppCmdPriv = &(pSession->cmdPrivData.data);
    (*ppCmdPriv)->sessionType = pSession->devIo.type;
    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief   session的私有数据buffer put
 * @param   pSession       [in]  session 结构体
 * @return  errno
 */
S32 cliCmdPrivDataPut(CliSessionObj_s *pSession)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if (NULL == pSession) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("NULL pointer, rc:%d", rc);
        goto lEnd;
    }

    if (pSession->cmdPrivData.callCount == SESSION_PRIVATE_BUFFER_CALL_NONE) {
        rc = -CLI_ERRNO_SESSION_PRIVATE_DATA_EMPTY;
        CLI_LOG_ERROR("Session(id = %d) private data is null", pSession->sessionId);
        goto lEnd;
    }

    pSession->cmdPrivData.callCount--;

    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief   获取空session
 * @param   void
 * @return  errno
 */
static S32 cliGetEmptySession(U32 *pSessionId)
{
    CliSessions_s *pCliSessions = NULL;
    U32 idx = 0;
    CliSessionObj_s *pSession = NULL;
    S32 rc = -CLI_ERRNO_FAILED;

    pCliSessions = cliSessionsGet();

    ospMutexLock(&(pCliSessions->sessionMutex));
    if (CLI_SESSION_MAX_COUNT <= pCliSessions->activeSessionCount) {
        CLI_LOG_ERROR("The session is full, active session num is %d, max num is %d", \
                pCliSessions->activeSessionCount, CLI_SESSION_MAX_COUNT);
        rc = -CLI_ERRNO_SESSION_FULL;
        *pSessionId = UNKNOWN_SESSION_ID;
        goto lEnd;
    }

    for (idx = 0; idx < CLI_SESSION_MAX_COUNT; idx++) {
        rc = cliGetSessionById(idx, &pSession);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Get session error, input session id = %d", idx);
            goto lEnd;
        }

        if (SESSION_STATUS_OFF == sessionStatusGet(idx)) {
            *pSessionId = idx;
            sessionStatusSet(idx, SESSION_STATUS_INIT);
            goto lEnd;
        }
    }

lEnd:
    ospMutexUnlock(&(pCliSessions->sessionMutex));
    return rc;
}

/**
 * @brief   创建session 线程的名字
 * @param   sessionId  [in]
 * @return  创建的session thread name
 */
static U32 cliBuildName(U32 sessionId)
{
    U32 sessionName;

    sessionName = ospBuildName(CHAR_C, CHAR_L, CHAR_I, CHAR_0 + sessionId);
    return sessionName;
}

/**
 * @brief   检查devInfo
 * @param   pSessionDevInfo  [in]
 * @return  是否符合要求
 */
static S32 cliCheckDevInfo(CliSessionDevIo_s *pSessionDevInfo)
{
    S32 rc = -CLI_ERRNO_FAILED;
    U32 idx;
    CliSessionObj_s *pSessionArr;

    if (NULL == pSessionDevInfo) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("The session is null");
        goto lEnd;
    }

    if (NULL == pSessionDevInfo->getCharFunc) {
        rc = -CLI_ERRNO_NO_GETCHAR_FUNC;
        CLI_LOG_ERROR("The session get char func is null");
        goto lEnd;
    }

    ///< 允许没有gets函数
    if (NULL == pSessionDevInfo->getsFunc) {
        CLI_LOG_DEBUG("The session gets func is null");
    }

    ///< 允许没有putchar函数
    if (NULL == pSessionDevInfo->putCharFunc) {
        CLI_LOG_DEBUG("The session putchar func is null");
    }

    if (NULL == pSessionDevInfo->putsFunc) {
        rc = -CLI_ERRNO_NO_PUTS_FUNC;
        CLI_LOG_ERROR("The session put char func is null");
        goto lEnd;
    }

    pSessionArr = sgCliSessions.sessions;
    switch (pSessionDevInfo->type) {
    case CLI_SESSION_TYPE_UART:
        for (idx = 0; idx < CLI_SESSION_MAX_COUNT; idx++) {
            if ((pSessionArr[idx].sessionStatus != SESSION_STATUS_OFF) && \
                (pSessionArr[idx].devIo.type == pSessionDevInfo->type)) {
                rc = -CLI_ERRNO_SESSION_TYPE_OCCUPIED;
                CLI_LOG_ERROR("Session type:%d occupied, rc:%08x", pSessionDevInfo->type, rc);
                goto lEnd;
            }
        }
        break;
    default:
        rc = -CLI_ERRNO_INVAL_SESSION_TYPE;
        CLI_LOG_ERROR("Unknow type:%d, rc:%08x", pSessionDevInfo->type, rc);
        goto lEnd;
    }

    rc = CLI_ERRNO_OK;
lEnd:
    return rc;
}

/**
 * @brief   记录session的数量
 * @param   void
 * @return  cli总结构体指针
 */
static void sessionCntInc()
{
    CliSessions_s *pCliSessions = NULL;

    pCliSessions = cliSessionsGet();
    ospMutexLock(&(pCliSessions->sessionMutex));
    pCliSessions->activeSessionCount++;
    (void)ospMutexUnlock(&(pCliSessions->sessionMutex));
}

/**
 * @brief      获取运行时间，单位ms
 * @param      存放运行时间的地址
 * @return     函数是否执行成功，失败返回hal_OS错误码
 * @warning    接口调用者保证pTimeMs不为NULL
 * @note:      xiaozhanhui
 */
static S32 cliSysUpTimeGet(U64 *pTimeMs)
{
    S32 ret = -CLI_ERRNO_FAILED;
    struct timespec tp = { 0 };

    ret = ospClockGetUptime(&tp);
    if (CLI_ERRNO_OK != ret) {
        perror("ospClockGetTimespec err");
        goto end;
    }
    *pTimeMs = tp.tv_sec * 1000LL; /* sec2ms */
    *pTimeMs += tp.tv_nsec / 1000000LL; /* ns2ms */

end:
    return ret;
}

/**
 * @brief        用户名密码检查
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2022.05.09  xiezhj
 */
S32 cliUartSessionLOGEn(CliSessionObj_s *pSession)
{
    S32             rc        = -CLI_ERRNO_FAILED;
    U64             nowStamp  = 0;
    U64             timeDiff  = 0;

    (void)cliSysUpTimeGet(&nowStamp);

    timeDiff = nowStamp - pSession->LOGEnStamp;
    if (timeDiff > CLI_UART_LOGEN_TIMEOUT) {         ///< 超过五分钟session无输入，则需要重新登录
        rc= cliUserLOGEn(pSession->sessionId);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("User LOGEn error.");
            goto lEnd;
        }
    }

    (void)cliSysUpTimeGet(&pSession->LOGEnStamp);

    rc = CLI_ERRNO_OK;
lEnd:
    return rc;
}

/**
 * @brief   session线程处理函数
 * @param   threadInput [in] 线程私有数据
 * @return  void
 * @note 可重入
 */
void cliSessionThreadHandler(void *pThreadInput)
{
    CliSessionObj_s *pSession   = NULL;
    U32 sessionId               = 0;
    CliInfo_s *pCliInfo         = NULL;
    S32 rc                      = -CLI_ERRNO_FAILED;
    CliInputMsg_s *pInputMsg    = NULL;
    InterationInfo_s interationInfo;
    U32  stringLen   = 0;

    (void)stringLen;

    pCliInfo = cliInfoGet();

    pInputMsg = cliMalloc((S32)sizeof(*pInputMsg), 0);
    if (NULL == pInputMsg) {
        CLI_LOG_ERROR("Cli malloc pInputMsg error.");
        goto lEnd;
    }

    cliMemSet(pInputMsg, 0, (S32)sizeof(*pInputMsg));
    pSession = (CliSessionObj_s *)pThreadInput;
    sessionId = pSession->sessionId;

    while (pSession->sessionStatus == SESSION_STATUS_RUNNING) {

        cliMemSet(pInputMsg->str, 0, (S32)sizeof(pInputMsg->str));
        stringLen = 0;

        ///< 接收输入
        prepareInteration(&interationInfo, sessionId, pInputMsg->str);

        rc = cliSessionGetInput(&interationInfo);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Get input error, sessionId = %d rc = %08x", sessionId, rc);
            continue;
        }

        if (pSession->devIo.LOGEnFunc != NULL) {
            ///< 用户登录检查
            rc = pSession->devIo.LOGEnFunc(pSession);
            if (rc != CLI_ERRNO_OK) {
                CLI_LOG_ERROR("UART LOGEn fail, rc=%08x.", rc);
                continue;
            }
        }

        ///< 只输入换行 不作处理
        if (interationInfo.recvLenReal == 0) {
            continue;
        }

        stringLen = interationInfo.recvLenReal;
        pInputMsg->sessionId = sessionId;

        CLI_LOG_DEBUG("pInputMsg.sessionId = %d", pInputMsg->sessionId);
        CLI_LOG_DEBUG("pInputMsg.str = %s", pInputMsg->str);

        pSession->inputMsgAddr = (ULong)pInputMsg;
        ///< 将输入放入 input msg que中
        rc = ospMessageQueueSend(pCliInfo->cliInputQueue.cliQueueId, &(pSession->inputMsgAddr), \
                sizeof(pSession->inputMsgAddr));
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("The session (Id = %d). send to queue fail, rc = %08x, queueId = %d, msgLen = %zd",  \
                    pSession->sessionId, rc, pCliInfo->cliInputQueue.cliQueueId,                             \
                    MSG_TO_MSGQUEUE_LENGTH(stringLen));
            continue;
        }

        ///< 阻塞等待命令完成
        rc = ospSemaphoreObtain(pSession->sessionSem, OSP_WAIT, 0);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("HalSemP error, rc:%08x", rc);
            goto lEnd;
        }

        ///< 打印命令运行结果并回收资源
        if (NULL == pSession->pPrintParent) {
            CLI_LOG_WARN("No outPut node, sessionId = %d", pInputMsg->sessionId);
            continue;
        }
        cliNodeShowNormal(pSession->sessionId, pSession->pPrintParent);
        cliDeleteNode(pSession->pPrintParent);
        pSession->devIo.isCmdFinish = true;
        pSession->pPrintParent = NULL;

        ///< 命令执行完成时，执行用户自定义的逻辑
        if (pSession->devIo.finshHandleFunc != NULL) {
            pSession->devIo.finshHandleFunc(pSession->devIo.type);
        }
    }

lEnd:
    cliFree(pInputMsg);
    rc = ospSemaphoreDelete(pSession->sessionSem);
    if (rc != OSP_SUCCESSFUL) {
        CLI_LOG_ERROR("HalSemDelete err:%08x", rc);
    }
    ospTaskExit();
    return;

}

/**
 * @brief   创建session线程
 * @param   pSession [in] session 结构体
 * @return  errno
 */
static S32 cliSessionCreate(CliSessionObj_s *pSession)
{
    S32 rc = -CLI_ERRNO_FAILED;
    U32 sessionName = 0;
    U32 id = 0;

    sessionName = cliBuildName(pSession->sessionId);
    ///< 打印头
    cliTitlePrint(pSession->devIo.putsFunc, pSession->sessionId);
    rc = ospSemaphoreCreate(ospBuildName('S', 'E', 'M', ' '), 0,
                            OSP_PRIORITY | OSP_NO_INHERIT_PRIORITY | OSP_NO_PRIORITY_CEILING |
                            OSP_LOCAL, USER_TASK_PRIORITY_HIGHEST, &pSession->sessionSem);
    if (rc != OSP_SUCCESSFUL) {
        CLI_LOG_ERROR("HalSemCreate, sessionId:%d rc:%08x", pSession->sessionId, rc);
        goto lEnd;
    }

    sessionStatusSet(pSession->sessionId, SESSION_STATUS_RUNNING);
    if (OSP_SUCCESSFUL != ospTaskCreate((U32)sessionName, USER_TASK_PRIORITY_NORMAL,
        CLI_THREAD_STACK_MINIMUM_SIZE, OSP_DEFAULT_MODES, OSP_DEFAULT_ATTRIBUTES, &id)) {
        CLI_LOG_ERROR("Cli session (Id = %d), Thread create error\r\n", \
                pSession->sessionId);
        goto lEnd;
    }
    if (OSP_SUCCESSFUL != ospTaskStart(id, (OspTaskEntry)cliSessionThreadHandler, (U32)pSession)) {
        CLI_LOG_ERROR("Cli session (Id = %d), Thread start error\r\n", \
                pSession->sessionId);
        goto lEnd;
    }
    rc = CLI_ERRNO_OK;
lEnd:
    return rc;
}

/**
 * @brief   获取cli总结构体指针
 * @param   void
 * @return  cli总结构体指针
 */
S32 cliSessionReg(CliSessionDevIo_s *pSessionDevInfo)
{
    S32 rc = -CLI_ERRNO_FAILED;
    U32 sessionId = UNKNOWN_SESSION_ID;
    CliSessionObj_s *pSession = NULL;

    rc = cliCheckDevInfo(pSessionDevInfo);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("The device info is invalid, rc = %08x", rc);
        goto lEnd;
    }

    ///< 获取关机的session
    rc = cliGetEmptySession(&sessionId);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Cli get session failed, rc = %08x", rc);
        goto lEnd;
    }

    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session error, input session id = %08x", sessionId);
        goto lEnd;
    }

    pSession->sessionId = sessionId;
    cliMemCpy(&(pSession->devIo), pSessionDevInfo, (S32)sizeof(*pSessionDevInfo));

    ///< 打开
    (void)cliSysUpTimeGet(&pSession->LOGEnStamp);
    pSession->LOGEnStamp -= CLI_UART_LOGEN_TIMEOUT;

    rc = cliSessionCreate(pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Cli creat session failed, rc = %08x", rc);
        goto lCloseSession;
    }

    sessionCntInc();
    goto lEnd;

lCloseSession:
    ///< 上面的锁可以保证其他线程访问不到该session,所以不需要加锁
    sessionStatusSet(sessionId, SESSION_STATUS_OFF);

lEnd:
    return rc;
}

/**
 * @brief   关闭该session
 * @param   sessionId   [in]
 * @return  errno
 * @note
 */
S32 cliSessionClose(U32 sessionId)
{
    S32 rc = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;

    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session error, input session id = %d", sessionId);
        goto lEnd;
    }

    rc = ospTaskDelete(pSession->cliSessionTID);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("HalThreadDelete err:%08x", rc);
    }
    sessionStatusSet(sessionId, SESSION_STATUS_OFF);
lEnd:
    return rc;
}

/**
 * @brief   获取输入
 * @param   pInputBuff [out] 接收到的输入biffer
 * @param   sessionId  [out] session id
 * @param   inputLen   [in/out] 传入最大长度, 传出获取的长度
 * @return  errno
 * @note 可重入
 */
S32 cliSessionGetInput(InterationInfo_s *pInteration)
{
    S32 rc = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;
    S8 userInput           = CLI_CHAR_NULL;
    U32 idx = 0;
    S8 *pOutPutString = NULL;

    if (NULL == pInteration) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("Input buffer is null, rc = %08x", rc);
        goto lEnd;
    }

    if (NULL == pInteration->recvString) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("Input buffer is null, rc = %08x", rc);
        goto lEnd;
    }

    rc = cliGetSessionById(pInteration->sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session error, sessionId should be 0-2, but input session id = %d, rc = %08x", \
                pInteration->sessionId, rc);
        goto lEnd;
    }

    ///< 打印行头
    (void)pSession->devIo.putsFunc(pInteration->lineTitle);

    ///< 初始化buffer
    cliMemSet(pInteration->recvString, 0, (S32)(pInteration->recvLenExpect + CLI_STRING_TAIL_LEN));
    idx = 0;
    pInteration->recvLenReal = 0;

    if (NULL != pSession->devIo.getsFunc) {
        (void)pSession->devIo.getsFunc(pInteration->recvString);
        pInteration->recvLenReal = cliStrLen(pInteration->recvString);
        CLI_LOG_DEBUG("Input str: %s", pInteration->recvString);

        if (CLI_CHAR_NULL == pInteration->outPutWord) {
            (void)pSession->devIo.putsFunc(pInteration->recvString);
            (void)pSession->devIo.putsFunc("\n");
        } else {
            pOutPutString = cliMalloc((S32)pInteration->recvLenReal + 1, 0);
            if (NULL == pOutPutString) {
                rc = -CLI_ERRNO_NO_MEMORY;
                CLI_LOG_ERROR("Out put string malloc error");
                goto lEnd;
            }

            cliMemSet(pOutPutString, (S32)pInteration->outPutWord, (S32)pInteration->recvLenReal);
            pOutPutString[pInteration->recvLenReal] = CLI_CHAR_NULL;
            (void)pSession->devIo.putsFunc(pOutPutString);
            (void)pSession->devIo.putsFunc("\n");
            cliFree(pOutPutString);
        }
    } else {

        ///< 循环取字符直到输入enter或长度超限,并将输入显示在命令行上
        do {
            ///< 从session中读取一个字节
            userInput = (S8)pSession->devIo.getCharFunc();

            if (((idx == 0) && (userInput == CLI_CHAR_BACKSPACE))) {
                ///< 如果命令行为空,忽视退格键
                continue;
            }

            ///< 发回命令行重定向的文件句柄中
            if (CLI_CHAR_NULL == pInteration->outPutWord) {
                (void)pSession->devIo.putCharFunc(userInput);
            } else {
                (void)pSession->devIo.putCharFunc(pInteration->outPutWord);
            }

            ///< 循环取字符,直到输入enter
            if ((userInput != CLI_CHAR_LF) && (idx < pInteration->recvLenExpect)) {
                if ((userInput == CLI_CHAR_BACKSPACE) && (idx > 0)) {
                    idx--;
                    pInteration->recvString[idx] = CLI_CHAR_NULL;

                    ///< 展示退格动作
                    (void)pSession->devIo.putCharFunc(CLI_CHAR_SPACE);
                    (void)pSession->devIo.putCharFunc(CLI_CHAR_BACKSPACE);
                } else {
                    ///< 保存用户的输入
                    pInteration->recvString[idx] = userInput;
                    idx++;
                }
            }
            ospTaskWakeAfter(0);        ///< 主动让出调度

        } while (userInput != CLI_CHAR_LF && idx < pInteration->recvLenExpect);

        pInteration->recvLenReal = idx;
    }

    ///< 判断是否超限
    if (pInteration->recvLenExpect <= pInteration->recvLenReal) {
        rc = -CLI_ERRNO_INPUT_BEYOND_LENGTH;
        CLI_LOG_ERROR("The session (Id = %d) input is beyond len. Input len is %d, "
                "max len allowed is %d", pSession->sessionId, idx, pInteration->recvLenExpect);
        goto lEnd;
    }

    ///< 一次输入结束加'\0'
    pInteration->recvString[pInteration->recvLenReal] = CLI_CHAR_NULL;

lEnd:
    return rc;
}

/**
 * @brief   填充用于接收用户输入的的数据结构
 * @param   pInterationInfo  [out] session id
 * @param   sessionId        [in]   传入最大长度, 传出获取的长度
 * @param   recvBuf          [in]   接收数据的buffer
 * @return  errno
 */
void prepareInteration(InterationInfo_s *pInterationInfo, U32 sessionId, S8 *recvBuf)
{
    cliMemSet(pInterationInfo, 0, (S32)sizeof(*pInterationInfo));
    ///< 准备和用户交互结构体
    cliMemCpy(pInterationInfo->lineTitle, CLI_LINE_HEADER, (S32)strlen(CLI_LINE_HEADER));
    pInterationInfo->recvLenExpect = CLI_MAX_LINE_LENGTH;
    pInterationInfo->sessionId = sessionId;
    pInterationInfo->recvString = recvBuf;
    pInterationInfo->recvLenReal = 0;
    pInterationInfo->outPutWord = CLI_CHAR_NULL;
}

/**
 * @brief   创建uart session
 * @param   void
 * @return  errno
 */
S32 cliUartSessionCreate(void)
{
    S32 rc = -CLI_ERRNO_FAILED;

    CliSessionDevIo_s sessionDevInfo;
    cliMemSet(&sessionDevInfo, 0, (S32)sizeof(CliSessionDevIo_s));

    sessionDevInfo.getCharFunc = getchar;
    sessionDevInfo.putCharFunc = putchar;
    sessionDevInfo.putsFunc = cliUartPutString;
    sessionDevInfo.LOGEnFunc = cliUartSessionLOGEn;
    sessionDevInfo.type = CLI_SESSION_TYPE_UART;

    rc = cliSessionReg(&sessionDevInfo);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Cli creat uart session errno, rc = %08x", rc);
        goto lEnd;
    }

lEnd:
    return rc;
}

/**
 * @brief   初始化全部session
 * @param
 * @return  errno
 */
void cliSessionInit()
{
    CliSessions_s *pCliSessions = NULL;

    pCliSessions = cliSessionsGet();

    cliMemSet(pCliSessions, 0, (S32)sizeof(*pCliSessions));

    ///< 初始化mutex 一定成功,不需要接收返回值
    (void)ospMutexInit(&(pCliSessions->sessionMutex), "cli_session");

    return;

}
