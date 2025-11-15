/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_command.c
 * @author    xiezhj
 * @date      2021.04.10
 * @brief     文件说明
 * @note
 */

#include "cli_api.h"
#include "cli_command.h"
#include "cli_core.h"

#define USAGE_STR       \
    "List of commands:\n"                                                               \
    "\n"                                                                                \
    "Commands   Description"

/**
 * @brief      命令注册函数
 * @param      [in]
 * @param      [out]
 * @return     CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note       2021.04.12  xiezhj
 */
void cliRegisterCmd(Command_s *pCmd)
{
    CliInfo_s *pCliInfo = NULL;
    CommandList_s *pCmdLst = NULL;

    ///< 入参检查
    if (NULL == pCmd) {
        CLI_LOG_ERROR("Input command is null.");
        goto lEnd;
    }

    pCliInfo = cliInfoGet();
    pCmdLst = &(pCliInfo->cmdList);

    if (pCmdLst->count >= (S64)ARRAY_SIZE(pCmdLst->list)) {
        CLI_LOG_ERROR("Command list beyond max size. gpCmdLst->count = %d, ARRAY_SIZE(pCmdLst->list) = %zd", \
                pCmdLst->count, ARRAY_SIZE(pCmdLst->list));
        goto lEnd;
    }

    pCmdLst->list[pCmdLst->count] = pCmd;
    pCmdLst->count++;

lEnd:
    return;
}

/**
 * @brief        命令析构函数
 * @param        [in] pCmd
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2023.05.11  xiezhj
 */
void cliDestructCmd(Command_s *pCmd)
{
    CliInfo_s     *pCliInfo = NULL;
    CommandList_s *pCmdLst  = NULL;
    U8            idx       = 0;
    U8            tempIdx   = 0;

    ///< 入参检查
    if (NULL == pCmd) {
        CLI_LOG_ERROR("Input command is null.");
        goto lEnd;
    }

    pCliInfo = cliInfoGet();
    pCmdLst = &(pCliInfo->cmdList);

    for (idx = 0; idx < pCmdLst->count; idx++) {
        if (cliStrCmp(pCmdLst->list[idx]->argument, pCmd->argument) == 0) {
            for (tempIdx = idx; tempIdx < pCmdLst->count - 1; tempIdx++) {
                pCmdLst->list[tempIdx] = pCmdLst->list[tempIdx + 1];
            }
            pCmdLst->list[--pCmdLst->count] = NULL;
            break;
        }
    }

lEnd:
    return ;
}

/**
 * @brief        判断标识符是否匹配
 * @param        [in]  pTemplate 模板
 * @param        [in]  pStr      标识符字符串
 * @param        [in]  pObj      标识符结构
 * @return       true/false
 * @warning
 * @note        /cx 匹配   /call /c1,2
 * @note        /cx 不匹配 /ca /ca1
 * @note         2021.04.21  xiezhj
 */
static Bool cliIsMatchIdentifier(S8 *pTemplate, S8 *pStr, Object_s *pObj)
{
    Bool ret       = false;
    U32  len       = 0;
    S8   lastChar  = 0;

    len = cliStrLen(pTemplate);
    S8   buff[len];

    memset(buff, 0, len);
    if ((NULL == pObj) || (NULL == pStr) || (NULL == pTemplate) || cliStrLen(pTemplate) == 0 || cliStrLen(pStr) == 0) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    lastChar = pTemplate[len - 1];
    if (cliToLower(lastChar) == 'x') {
        if (pObj->idList.count == 0) {
            goto lEnd;
        }
        (void)cliStrNCpy(buff, pTemplate + 1, (S32)len - 1); ///< +1: skip char '/' -1:skip last char 'x'
        ret = (cliStrCaseCmp(buff, pObj->name) == 0) ? true : false;
        goto lEnd;
    } else {
        ret = (cliStrCaseCmp(pTemplate, pStr) == 0) ? true : false;
        goto lEnd;
    }

lEnd:
        return ret;
}

/**
 * @brief        判断字符串是否匹配
 * @param        [in]   template 模板
 * @param        [in]   str      输入字符串
 * @param        [in]   object   标识符结构
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         show 匹配 show ShoW
 * @note         /cx  匹配 /c1 /call
 * @note         2021.04.21  xiezhj
 */
static Bool cliIsMatchString(S8 *pTemplate,S8 *pInput, Object_s *pObj)
{
    Bool ret            = false;
    S8   *pBuffInput    = NULL;
    S8   *pBuffTemplate = NULL;
    S8   *pTmp          = NULL;

    ///< 入参检查
    if ((NULL == pTemplate) || (NULL == pInput)) {
        CLI_LOG_DEBUG("Input para is null.");
        ret = false;
        goto lEnd;
    }

    if (pTemplate[0] == '/' && pInput[0] == '/') {
        ret = cliIsMatchIdentifier(pTemplate, pInput, pObj);
        goto lEnd;
    } else if (cliStrCmp(pTemplate, "...") == 0) {
        ret = true;
        goto lEnd;
    }

    ///< 非标识符匹配
    pBuffInput     = cliMalloc((S32)cliStrLen(pInput) + 1, 0);
    if (NULL == pBuffInput) {
        CLI_LOG_ERROR("Alloc pBuffInput failed, size=%d.", cliStrLen(pInput) + 1);
        goto lEnd;
    }
    pBuffTemplate  = cliMalloc((S32)cliStrLen(pTemplate) + 1, 0);
    if (NULL == pBuffTemplate) {
        CLI_LOG_ERROR("Alloc pTemplate failed, size=%d.", cliStrLen(pTemplate) + 1);
        goto lFreeInput;
    }

    pTmp = cliStrChr(pTemplate, '=');
    if (pTmp != NULL) { ///< 如果模板中有等号，对比等号前面的内容
        (void)cliMemCpy(pBuffTemplate, pTemplate, (pTmp - pTemplate));
        pTmp = cliStrChr(pInput, '=');
        if (pTmp != NULL) { ///< 如果输入有等号，取等号前面的内容进行对比
            (void)cliMemCpy(pBuffInput, pInput, (pTmp - pInput));
        } else {
            (void)cliMemCpy(pBuffInput, pInput, (S32)cliStrLen(pInput));
        }
    } else { ///< 如果模板中没有等号，对比所以内容
        (void)cliMemCpy(pBuffInput, pInput, (S32)cliStrLen(pInput));
        (void)cliMemCpy(pBuffTemplate, pTemplate, (S32)cliStrLen(pTemplate));
    }

    ret = (cliStrCaseCmp(pBuffTemplate, pBuffInput) == 0 ?  true : false);

    cliFree(pBuffTemplate);
lFreeInput:
    cliFree(pBuffInput);
lEnd:
    return ret;
}

/**
 * @brief        判断模板是否和输入参数匹配
 * @param        [in]   template 模板
 * @param        [in]   argument 输入参数
 * @param        [in]   deep     匹配深度
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         "/cx /vx show" 与 "/c1 show" 匹配深度为 1, 只匹配成功 /cx 一个字段
 * @note         2021.04.21  xiezhj
 */
static Bool cliIsMatchArgument(S8 *pTemplate, Argument_s *pArgument, S32 *pDeep)
{
    Bool     ret      = false;;
    S32      len      = (S32)cliStrLen(pTemplate) + 1;;
    S8       buf[len];
    S8       *pSave   = NULL;
    S8       *pToken  = NULL;
    S32      tmpIdx   = 0;
    Object_s *pObj    = NULL;

    ///< 入参检查
    if (NULL == pTemplate) {
        ((pArgument->argc == 0) ? (ret = true) : (ret = false));
        goto lEnd;
    }

    (void)cliStrNCpy(buf, pTemplate, (S32)sizeof(buf));

    pToken = cliStrtok(buf, " ", &pSave);
    while (pToken != NULL) {
        if (cliStrCmp(pToken, "...") == 0) {
            *pDeep = tmpIdx;
            ret = true;
            goto lEnd;
        }

        if (tmpIdx < pArgument->objectCount) {
            pObj = &pArgument->objectList[tmpIdx];
        }

        ret = cliIsMatchString(pToken, pArgument->argv[tmpIdx], pObj);
        if (false == ret) {
            *pDeep = tmpIdx;
            goto lEnd;
        }
        tmpIdx++;  ///< 未使用户输入与模板中所分析到的位置相匹配
        pToken = cliStrtok(NULL, " ", &pSave);
        if ((NULL == pToken) && (tmpIdx < pArgument->argc)) { ///< 参数个数不足
            *pDeep = tmpIdx;
            ret = false;
            goto lEnd;
        }
    }
    *pDeep = tmpIdx;
    ret = true;

lEnd:
    return ret;
}

/**
 * @brief        查找匹配的命令
 * @param        [in]   pArgument   输入命令行
 * @param        [out]  pHelpList   匹配层次最深的帮助列表（符合字段最多的命令列表）
 * @param        [out]  pMaxDeep    匹配的最大深度（匹配的最大参数个数，以此来决定帮助信息显示的层次）
 * @return       struct Command*
 * @warning
 * @note         2021.04.21  xiezhj
 */
static Command_s *cliFindCommand(Argument_s *pArgument, CommandList_s *pHelpList, S32 *pMaxDeep)
{
    Command_s  *pMatchCmd = NULL;
    S32        idx         = 0;
    Command_s  *pTmpCmd  = NULL;
    S32        deep      = 0;
    Bool       ret       = false;
    CliInfo_s *pCliInfo = NULL;
    CommandList_s *pCmdLst = NULL;

    if ((NULL == pArgument) || (NULL == pHelpList) || (NULL == pMaxDeep)) {
        CLI_LOG_ERROR("Input para is null, pArgument=null?:%d, pHelpList=null?:%d, pMaxDeep=null?:%d.",
                        (NULL == pArgument), (NULL == pHelpList), (NULL == pMaxDeep));
        goto lEnd;
    }

    pCliInfo = cliInfoGet();
    pCmdLst = &(pCliInfo->cmdList);

    *pMaxDeep  = 0;

    for (idx = 0; idx < pCmdLst->count; idx++) {
        pTmpCmd = pCmdLst->list[idx];
        deep    = 0;
        if (pTmpCmd->argument != NULL) {
            ret = cliIsMatchArgument(pTmpCmd->argument, pArgument, &deep);
            if (true == ret) {
                pMatchCmd = pTmpCmd;
                if ((false== pArgument->isHelp) && (false == pArgument->isComplete)) {
                    CLI_LOG_DEBUG("Cmd find success.");
                    goto lEnd;
                }
            }
        }

        if (deep > *pMaxDeep) {
            *pMaxDeep = deep;
            cliBZero(pHelpList, (S32)sizeof(*pHelpList));
            pHelpList->list[pHelpList->count] = pTmpCmd;
            pHelpList->count++;
        } else if (deep == *pMaxDeep) {
            pHelpList->list[pHelpList->count] = pTmpCmd;
            pHelpList->count++;
        }
    }

lEnd:
    return pMatchCmd;
}

/**
 * @brief        函数说明
 * @param        [in]
 * @param        [out]
 * @return       void
 * @warning
 * @note         2021.04.22  xiezhj
 */
void cliSetCmdPrivData(CmdPrivData_s *pPriv, U32 sessionId,
                             Argument_s *pArgument, Command_s *pCmd,
                             Node_s *pNode)
{

    ///< 填充私有数据
    pPriv->sessionId = sessionId;
    pPriv->pArg      = pArgument;
    pPriv->pCmd      = pCmd;
    pPriv->pParent   = pNode;
}

/**
 * @brief        cmd处理完成之后打印
 * @param        [in]   pPriv       命令私有信息
 * @param        [in]   cmdStatus   命令运行状态
 * @return       void
 * @warning
 * @note         2021.04.23  王殿卫  新生成函数
 * @note        只在函数CliCallCommand中使用  该函数中已经对pPriv进行了入参判断,所以次数无需入参判断
 */
void cliCmdStatusPrint(CmdPrivData_s *pPriv, S32 cmdStatus)
{

    if (cmdStatus == CLI_ERRNO_OK) {
        cliNodeModifyKeyStringF(pPriv->pParent, "Status", "Success");
        cliNodeModifyKeyStringF(pPriv->pParent, "Description", "None");
    } else {
        cliNodeModifyKeyStringF(pPriv->pParent, "Status", "Failure");
        ///< TODO:打印错误信息
        cliNodeModifyKeyStringF(pPriv->pParent, "Description",
            "Command execute failed(PS3CLI_ERRNO_COMMON 0x%08x[%d])", cmdStatus);
    }
}

/**
 * @brief        执行 Command 命令
 * @param        [in]   pArgument   参数信息
 * @param        [in]   pCmd        命令信息
 * @param        [in]   sessionId
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.21  xiezhj
 */
static S32 cliCallCommand(CmdPrivData_s *pPriv)
{

    S32        rc         = CLI_ERRNO_INVALID;
    S32        argc       = 0;
    S8         **ppArgv   = NULL;
    Argument_s *pArgument = NULL;
    Command_s  *pCmd      = NULL;

    ///< 入参检查
    if (NULL == pPriv) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("pPriv is null, rc = %d", rc);
        goto lEnd;
    }

    if (NULL == pPriv->pArg || NULL == pPriv->pCmd) {
        rc = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("pArgument or pCmd is null, rc = %d", rc);
        goto lEnd;
    }

    pArgument = pPriv->pArg;
    pCmd = pPriv->pCmd;

    ///< 该命令中有参数列表，则在此处解析，否则在具体回调里进行解析
    if (NULL != pCmd->pParam && pCmd->paramCnt != 0) {
        ///< 解析用户输入的参数
        argc = pArgument->argc;
        ppArgv = (S8 **)pArgument->argv;

        argc -= (S32)pCmd->argOffset;     ///< 跳过参数偏移
        ppArgv += pCmd->argOffset;
        rc = cliParseParam(argc, ppArgv, pCmd->pParam, (S32)pCmd->paramCnt);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Parse param error argc = %d, ppArgv[0] = %s, rc = %08x", argc, *ppArgv, rc);
            cliPrintDetailHelp(pPriv->sessionId, pCmd);
            goto lEnd;
        }

        ///< 检查参数
        rc = cliCheckParam(pCmd->pParam, pCmd->paramCnt, pPriv->pParent);
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("Check param error paramNum = %d, param[0] = %s, rc = %08x",
                    pCmd->paramCnt, pCmd->name, rc);
            goto lEnd;
        }

    }

    ///< 调用cmd回调
    if (pCmd->handleCmd != NULL) {
        rc = pCmd->handleCmd(pPriv);
        if (rc != CLI_ERRNO_OK) {
            ///< 如果不成功打印运行状态等信息
            CLI_LOG_ERROR("Call cmd handler error, rc:%d, sessionId:%d, cmd:%s", \
                    rc    , pPriv->sessionId, pCmd->name);
            goto lEnd;
        }
    }

    rc = CLI_ERRNO_OK;
lEnd:
    return rc;

}

/**
 * @brief        打印help list
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.22  xiezhj
 */
void cliPrintHelpList(U32 sessionId, Command_s *list[], U32 count)
{
    U32             loopCnti  = 0;
    Command_s       *pCmd     = NULL;
    S32             rc        = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;
    CmdPrivData_s   *pPriv    = NULL;

    ///< 根据sessionId 获取session私有数据结构
    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session info error, sessionId = %d, rc = %08x", sessionId, rc);
        goto lEnd;
    }
    pPriv = &(pSession->cmdPrivData.data);

    for (loopCnti = 0; loopCnti < count; loopCnti++) {
        pCmd = list[loopCnti];
        if (((pPriv->sessionType & pCmd->suppSession) == 0) && (pCmd->suppSession != 0)) {
           continue;            ///< session 不支持的command 不显示
        }
        if (pCmd->syntax != NULL) {
            cliNodeAddStringVal(pPriv->pParent, "%s", pCmd->syntax);
        } else if (pCmd->argument != NULL) {
            cliNodeAddStringVal(pPriv->pParent, "%s", pCmd->argument);
        }
    }
    cliNodeAddStringVal(pPriv->pParent, "%s", CLI_SEPARATOR);
lEnd:
    return;
}

/**
 * @brief        打印详细帮助信息
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.22  xiezhj
 */
void cliPrintDetailHelp(U32 sessionId, Command_s *pCmd)
{
    S32             rc        = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;
    CmdPrivData_s   *pPriv    = NULL;

    ///< 入参检查
    if (NULL == pCmd) {
        CLI_LOG_ERROR("input para pCmd is null.");
        goto lEnd;
    }

    ///< 根据sessionId 获取session私有数据结构
    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session info error, sessionId = %d, rc = %08x", sessionId, rc);
        goto lEnd;
    }
    pPriv = &(pSession->cmdPrivData.data);

    if (((pPriv->sessionType & pCmd->suppSession) == 0) && (pCmd->suppSession != 0)) {
            goto lEnd;
    }

    if (pCmd->name != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Name: %s",  pCmd->name);
    }

    if (pCmd->syntax != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Syntax: %s", pCmd->syntax);
    } else if (pCmd->argument != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Syntax: %s", pCmd->argument);
    }

    if (pCmd->description != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Description: %s", pCmd->description);
    }
    if (pCmd->option != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Option: %s", pCmd->option);
    }
    if (pCmd->convention != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Convention: %s", pCmd->convention);
    }
    if (pCmd->note != NULL) {
        cliNodeAddStringVal(pPriv->pParent, "Note: %s\n", pCmd->note);
    }
    cliNodeAddStringVal(pPriv->pParent, "%s", CLI_SEPARATOR);

lEnd:
    return;
}

/**
 * @brief        打印usage
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.23  xiezhj
 */
void cliPrintUsage(U32 sessionId)
{
    S32             idx       = 0;
    CliInfo_s       *pCliInfo = NULL;
    CommandList_s   *pCmdLst  = NULL;
    CmdPrivData_s   *pPriv    = NULL;
    S32             rc        = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;

    ///< 根据sessionId 获取session私有数据结构
    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session info error, sessionId = %d, rc = %08x", sessionId, rc);
        goto lEnd;
    }

    pPriv = &(pSession->cmdPrivData.data);

    cliNodeAddStringVal(pPriv->pParent, "%s", USAGE_STR);
    cliNodeAddStringVal(pPriv->pParent, "%s", CLI_SEPARATOR);

    pCliInfo = cliInfoGet();
    pCmdLst = &(pCliInfo->cmdList);
    for (idx = 0; idx < pCmdLst->count; idx++) {
        if (((pPriv->sessionType & pCmdLst->list[idx]->suppSession) != 0) ||
                (pCmdLst->list[idx]->suppSession == 0)) {
            cliNodeAddStringVal(pPriv->pParent, "%s", pCmdLst->list[idx]->syntax);
        }
    }
    cliNodeAddStringVal(pPriv->pParent, "%s", CLI_SEPARATOR);
lEnd:
    return;
}

/**
 * @brief        处理输入命令
 * @param        [in]   pArgument   输入命令
 * @param        [in]   pSession    session结构体
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.22  xiezhj
 * @note         2021.04.28  王殿卫 修改:将获取cmd private变量提到cliStringHandle函数中
 * @note         2021.09.24  王殿卫 修改:将入参sessionId改为pSession
 * @note         2021.11.25  王殿卫 修改:整理代码结构减少圈复杂度
 */
S32 cliHandleCommand(CmdPrivData_s *pPriv, CliSessionObj_s *pSession)
{
    S32    err              = CLI_ERRNO_INVALID;
    S32           maxDeep   = 0;
    CommandList_s helpList  = {};
    Command_s    *pCmd      = NULL;
    Argument_s *pArgument   = NULL;
    U8 level;
    U32 sessionId;

    ///< 入参检查
    if (NULL == pSession) {
        err = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("pSession is null, rc = %d", err);
        goto lEnd;
    }

    if (NULL == pPriv || NULL == pPriv->pArg) {
        err = -CLI_ERRNO_NULL_PTR;
        CLI_LOG_ERROR("pPriv is null?:%d, pPriv->pArg is null?:%d, err=%08x.",
            (pPriv == NULL), (pPriv->pArg == NULL),err);
        goto lEnd;
    }

    level       = pSession->user.level;
    sessionId = pSession->sessionId;
    pArgument   = pPriv->pArg;

    ///< 匹配命令
    pCmd = cliFindCommand(pArgument, &helpList, &maxDeep);
    pPriv->pCmd = pCmd;

    ///< 参数不支持打印帮助信息
    if ((true == pArgument->isHelp) && ((NULL == pCmd) || (false == pCmd->needHelp))) {
        if (helpList.count > 1) {    ///< 如果匹配到多个相近命令，显示命令列表
            cliPrintHelpList(sessionId, helpList.list, (U32)helpList.count);
        } else if (pCmd != NULL) {    ///< 如果匹配到精确命令，显示详细帮助
            cliPrintDetailHelp(sessionId, pCmd);
        } else if (helpList.count == 1) { ///< 如果匹配到一个相近命令，显示命令列表
            cliPrintHelpList(sessionId, helpList.list, (U32)helpList.count);
        } else {
            cliPrintUsage(sessionId);
        }
        err = -CLI_ERRNO_ARGUMENT_INVALID;
        CLI_LOG_ERROR("Arg invalid, rc:%d", err);
        goto lShow;
    }

    if (NULL == pCmd) {
        if (maxDeep >= 1) {
            cliPrintHelpList(sessionId, helpList.list, (U32)helpList.count);
        } else {
            cliPrintUsage(sessionId);
        }
        err = -CLI_ERRNO_INVAL_CMD;
        CLI_LOG_ERROR("Cmd invalid, deep=%d, rc:%d", maxDeep, err);
        goto lShow;
    }

    if (pCmd->suppSession != CLI_SESSION_TYPE_ALL) {
        if ((pCmd->suppSession & pPriv->sessionType) == 0) {
            err = -CLI_ERRNO_INVAL_CMD;
            CLI_LOG_WARN("This session not support this cmd, session type:%02x, cmd support" \
                "session type:%02x", pPriv->sessionType, pCmd->suppSession);
            cliNodeAddStringVal(pPriv->pParent, "This session not support this cmd.");
            goto lShow;
        }
    }

    if (level > pCmd->cmdLevel) {            ///< level 越低，优先级越高
        err = -CLI_ERRNO_LEVEL_ERROR;
        cliNodeAddStringVal(pPriv->pParent, "You do not have authority to operate this cmd.");
        goto lShow;
    }

    err = cliCallCommand(pPriv);
    if (err != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Call cmd error, rc = %08x", err);
        goto lShow;
    }

    ///< 同步情况下，在此释放资源；异步情况下,且线程间通信成功，在执行命令的线程内释放资源
    if (pCmd->isSync == false) {
        err = CLI_ERRNO_OK;
        goto lEnd;
    }

    err = CLI_ERRNO_OK;

lShow:
    cliCmdStatusPrint(pPriv, err);
    cliCmdRunFinish(pPriv, pSession);
lEnd:
    return err;
}
