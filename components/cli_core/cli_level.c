/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_level.c
 * @author  王殿卫
 * @data    2021.04.10
 * @brief   cli level代码
 * @note    王殿卫新生成文件
 */

#include "cli_core.h"

CliUsers_s cliUsers;
static S8 spPassWd[CLI_PASSWORD_LEN_MAX] = CLI_DEFAULT_PASSWORD;

/**
 * @brief   获取user信息
 * @param   void
 * @return  errno
 */
CliUsers_s *cliUsersGet(void)
{
    return &cliUsers;
}

/**
 * @brief   cliCore的初始化
 * @param   pCliUserCfg [in] 用户配置
 * @param   userCount   [in] 用户数量
 * @return  errno
 */
S32 cliUserCfgInit(void)
{
    S32 rc = -CLI_ERRNO_FAILED;

    CliUsers_s *pCliUsers = NULL;
    CliUserObj_s *pCliUserNode = NULL;
    S32 idx = 0;
    CliCfg_s *pCliCfg = NULL;
    CliUserObj_s *pCliUserNextNode = NULL;

    pCliUsers = cliUsersGet();
    pCliCfg = cliCfgGet();
    initListHead(&(pCliUsers->cliUserList));

    if (NULL == pCliCfg) {
        CLI_LOG_DEBUG("Cli cfg was not specified, and use the default cfg, only root user can be used");

        pCliUserNode = (CliUserObj_s *)cliMalloc((S32)sizeof(*pCliUserNode), 0);
        if (NULL == pCliUserNode) {
            rc = -CLI_ERRNO_NO_MEMORY;
            CLI_LOG_ERROR("User info malloc error, rc = %08x", rc);
            goto lEnd;
        }

        pCliUsers->cliCmdLevelCount = CLI_LEVEL_0;
        pCliUsers->cliUserCount = DEFAULT_USER_NUM;

        cliStrNCpy(pCliUserNode->name, "root", SESSION_USERNAME_MAX_REAL_LEN);
        cliStrNCpy(pCliUserNode->passWord, spPassWd, SESSION_PASSWORD_MAX_REAL_LEN);
        pCliUserNode->level = CLI_LEVEL_0;
        listAdd(&(pCliUserNode->listNode), &(pCliUsers->cliUserList));

    } else {
        pCliUsers->cliCmdLevelCount = pCliCfg->cliCount.levelCount;
        pCliUsers->cliUserCount = pCliCfg->cliCount.userCount;

        initListHead(&(pCliUsers->cliUserList));
        for (idx = 0; idx < pCliUsers->cliUserCount; idx++) {
            pCliUserNode = (CliUserObj_s *)cliMalloc((S32)sizeof(*pCliUserNode), 0);
            if (NULL == pCliUserNode) {
                rc = -CLI_ERRNO_NO_MEMORY;
                CLI_LOG_ERROR("%dth user info malloc error, rc = %08x", idx, rc);
                goto lFreeUserInfo;
            }
            pCliUserNode->level = pCliCfg->cliUser[idx].level;
            cliStrNCpy(pCliUserNode->name, pCliCfg->cliUser[idx].name, SESSION_USERNAME_MAX_REAL_LEN);
            cliStrNCpy(pCliUserNode->passWord, pCliCfg->cliUser[idx].passWd, SESSION_PASSWORD_MAX_REAL_LEN);
            listAdd(&(pCliUserNode->listNode), &(pCliUsers->cliUserList));
        }
    }

    rc = CLI_ERRNO_OK;
    goto lEnd;

lFreeUserInfo:
    pCliUserNode = LIST_FIRST_ENTRY(&(pCliUsers->cliUserList), CliUserObj_s, listNode);
    LIST_FOR_EACH_ENTRY_SAFE(pCliUserNode, pCliUserNextNode, &(pCliUsers->cliUserList), listNode) {
        listDel(&(pCliUserNode->listNode));
        cliFree(pCliUserNode);
    }

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
///< static
void cliPrepareGetUsername(InterationInfo_s *pInterationInfo,
        U32 sessionId, S8 *recvBuf)
{
    cliMemCpy(pInterationInfo->lineTitle, CLI_LOGIN_USERNAME, (S32)strlen(CLI_LOGIN_USERNAME));
    pInterationInfo->recvLenExpect = SESSION_USERNAME_MAX_REAL_LEN;
    pInterationInfo->sessionId = sessionId;
    pInterationInfo->recvLenReal = 0;
    pInterationInfo->outPutWord = CLI_CHAR_NULL;
    pInterationInfo->recvString = recvBuf;
}

/**
 * @brief   填充用于接收用户输入的的数据结构
 * @param   pInterationInfo  [out] session id
 * @param   sessionId        [in]   传入最大长度, 传出获取的长度
 * @param   recvBuf          [in]   接收数据的buffer
 * @return  errno
 */
static void cliPrepareGetPassWord(InterationInfo_s *pInterationInfo,
        U32 sessionId, S8 *recvBuf)
{
    cliMemCpy(pInterationInfo->lineTitle, CLI_LOGIN_PASSWORD, (S32)strlen(CLI_LOGIN_PASSWORD));
    pInterationInfo->recvLenExpect = SESSION_PASSWORD_MAX_REAL_LEN;
    pInterationInfo->sessionId = sessionId;
    pInterationInfo->recvLenReal = 0;
    pInterationInfo->outPutWord = CHAR_REPLACE_PASSWORD;
    pInterationInfo->recvString = recvBuf;
}

/**
 * @brief   cliCore的初始化
 * @param   sessionId [in]
 * @return  errno
 */
S32 cliUserLogin(U32 sessionId)
{

    S32 rc = -CLI_ERRNO_FAILED;

    CliSessionObj_s *pSession = NULL;
    CliUserObj_s *pCliUserInfo = NULL;
    CliUsers_s *pCliUsers = NULL;
    CliUserObj_s *pCliUserNode = NULL;

    InterationInfo_s interationInfo;
    ///< S8 userName[CLI_USERNAME_MAX_LEN];
    S8 passWord[SESSION_PASSWORD_MAX_LEN];
    U32 idx = 0;
    Bool findFlag = false;
    pCliUsers = cliUsersGet();

    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("Get session error, rc = %08x, sessionId = %d", rc, sessionId);
        goto lEnd;
    }

    pCliUserInfo = &(pSession->user);

    ///< 每次输入username 有三次输入密码的机会,若三次全部错误 需重新输入username
    while (true) {
        ///< 依次对比用户链表的username
        LIST_FOR_EACH_ENTRY(pCliUserNode, &(pCliUsers->cliUserList), listNode) {
            rc = cliStrNCmp(pCliUserNode->name, "root", SESSION_USERNAME_MAX_REAL_LEN);

            ///< 成功打断循环
            if (STRING_MATCH_SUCCESS == rc) {
                findFlag = true;
                break;
            }
        }

        ///< 若没有对比成功  重新接收username
        if (false == findFlag) {
            (void)pSession->devIo.putsFunc(STRING_UNKOWN_USERNAME);
            continue;
        }

        ///< 与用户交互 获取passwd
        for (idx = 0; idx < IDENTIFY_MAX_TIMES; idx++) {
            findFlag = false;

            ///< 初始化 interationInfo 与 passWord
            cliMemSet(&interationInfo, 0, (S32)sizeof(interationInfo));
            cliMemSet(passWord, 0, (S32)sizeof(passWord));

            ///< 用户交互 获取输入的password
            cliPrepareGetPassWord(&interationInfo, sessionId, passWord);
            rc = cliSessionGetInput(&interationInfo);
            if (rc != CLI_ERRNO_OK) {
                CLI_LOG_ERROR("get password error, rc = %d, sessionId = %d", rc, sessionId);
                goto lEnd;
            }

            ///< 对比密码
            rc = cliPassWdMatch(pCliUserNode->passWord, passWord);

            ///< 成功保存用户信息
            if (STRING_MATCH_SUCCESS == rc) {
                CLI_LOG_DEBUG("Identify success.");
                (void)pSession->devIo.putsFunc(STRING_WELCOME);
                cliMemCpy(pCliUserInfo, pCliUserNode, (S32)sizeof(*pCliUserInfo));
                findFlag = true;
                break;
            } else {
                (void)pSession->devIo.putsFunc(STRING_TRY_AGAIN);
            }
        }

        if (true == findFlag) {
            break;
        } else {
            (void)pSession->devIo.putsFunc(STRING_BYONG_RETRY_NUM);
            continue;
        }
    }

    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief        密码校验
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2022.05.11  xiezhj
 */
S32 cliPassWdMatch(S8 *pPassWd, S8 *pInput)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if (!pInput) {
        goto lEnd;
    }
    if (0 != strcmp(pInput, pPassWd)) {
        goto lEnd;
    }
    rc = CLI_ERRNO_OK;
lEnd:
    return rc;
}

S32 cliPassWdGet(S8 *pPassWd)
{
    S32 rc = -EXIT_FAILURE;
    CliUsers_s *pCliUsers = cliUsersGet();
    CliUserObj_s *pCliUserNode = NULL;

    if (!pCliUsers || !pPassWd) {
        goto lEnd;
    }
    LIST_FOR_EACH_ENTRY(pCliUserNode, &(pCliUsers->cliUserList), listNode) {
        rc = cliStrNCmp(pCliUserNode->name, "root", SESSION_USERNAME_MAX_REAL_LEN);
        if (STRING_MATCH_SUCCESS != rc) {
            continue;
        }
        ///< 成功
        cliStrNCpy(pPassWd, pCliUserNode->passWord, SESSION_PASSWORD_MAX_REAL_LEN);
        rc = EXIT_SUCCESS;
        break;
    }
lEnd:
    return rc;
}

S32 cliPassWdSet(S8 *pPassWd)
{
    S32 rc = -EXIT_FAILURE;
    CliUsers_s *pCliUsers = cliUsersGet();
    CliUserObj_s *pCliUserNode = NULL;

    if (!pCliUsers || !pPassWd) {
        goto lEnd;
    }
    LIST_FOR_EACH_ENTRY(pCliUserNode, &(pCliUsers->cliUserList), listNode) {
        rc = cliStrNCmp(pCliUserNode->name, "root", SESSION_USERNAME_MAX_REAL_LEN);
        if (STRING_MATCH_SUCCESS != rc) {
            continue;
        }
        ///< 成功
        cliStrNCpy(pCliUserNode->passWord, pPassWd, SESSION_PASSWORD_MAX_REAL_LEN);
        rc = EXIT_SUCCESS;
        break;
    }
lEnd:
    return rc;
}