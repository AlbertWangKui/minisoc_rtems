/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_level.h
 * @author  王殿卫
 * @data    2021.04.08
 * @brief   cli level代码
 * @note    王殿卫新生成文件
 */

#ifndef __CLI_LEVEL_H__
#define __CLI_LEVEL_H__

#include "cli_std.h"
#include "cli_list.h"
#include "cli_session.h"

#define UNKOWN_LEVEL_ID (0xFF)
#define DEFAULT_LEVEL_COUNT (3)

#define CHAR_REPLACE_PASSWORD '*'
#define IDENTIFY_MAX_TIMES (3)
#define STRING_WELCOME "\nWelcome\n\0"
#define STRING_TRY_AGAIN "Password wrong, Try again\n\0"
#define STRING_UNKOWN_USERNAME "Unkown username, please input again\n\0"
#define STRING_BYONG_RETRY_NUM "\nIdentify failed\n\0"
#define DEFAULT_USER_NUM (1)
#define CLI_PASSWD_OFFSET 5

typedef struct CliUsers {
    U8  cliUserCount;                               ///< 用户数量
    U8  cliCmdLevelCount;                           ///< cli cmd 等级数量
    struct ListHead cliUserList;                    ///< user 链表 CliUserObj_s
} CliUsers_s;

typedef struct CliCountCfg {
    U8        levelCount;
    U8        userCount;
    U8        reserved[2];
} CliCountCfg_s;

typedef struct CliUserCfg {
    S8        name[CLI_USERNAME_MAX_LEN];
    S8        passWd[SESSION_PASSWORD_MAX_LEN];
    U8        level   ;
    U8        reserved_2[3];
} CliUserCfg_s;

typedef struct CliCfg {
    CliCountCfg_s     cliCount;
    CliUserCfg_s      cliUser[];
} CliCfg_s;

/**
 * @brief   cliCore的初始化
 * @param   pCliUserCfg [in] 用户配置
 * @param   userCount   [in] 用户数量
 * @return  errno
 */
S32 cliUserCfgInit(void);

/**
 * @brief   cliCore的初始化
 * @param   sessionId [in]
 * @return  errno
 */
S32 cliUserLOGEn(U32 sessionId);

/**
 * @brief   获取user信息
 * @param   void
 * @return  errno
 */
CliUsers_s *cliUsersGet( void );

/**
 * @brief        密码校验
 * @param        [in]
 * @param        [out]
 * @return       EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note         2022.05.11  xiezhj
 */
S32 cliPassWdMatch(S8 *pPassWd, S8 *pInput);

#endif ///< __CLI_LEVEL_H__

