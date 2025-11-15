/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_seesion.c
 * @author  王殿卫
 * @data    2021.04.07
 * @brief   cli session代码
 * @note    王殿卫新生成文件
 */

#ifndef __CLI_SESSION_H__
#define __CLI_SESSION_H__

#include "cli_list.h"
#include "cli_api.h"
#include "cli_types.h"

#define UNKNOWN_SESSION_ID    (0xFF)

///< 用于求session thread id
enum {
    CHAR_0 = '0',
    CHAR_C = 'C',
    CHAR_L = 'L',
    CHAR_I = 'I',
};

///< 特殊字符
typedef enum CommCliChar {
    CLI_CHAR_SPACE     = 0X20, ///< 空格
    CLI_CHAR_BACKSPACE = 0X08, ///< 退格
    CLI_CHAR_CR        = 0X0D, ///< carriage return 回车
    CLI_CHAR_SOH       = 0X01, ///< start of headline 标题开始
    CLI_CHAR_NULL      = 0X00, ///< NULL
    CLI_CHAR_LF        = '\n', ///< 换行
} CommCliChar_e;

#define PARAM_DELIMITER       " "
#define CLI_TITLE "======================================================================"

#define CLI_LINE_HEADER "PS3> \0"
#define CLI_LOGIN_USERNAME "PS3> Please input your username:\0"
#define CLI_LOGIN_PASSWORD "PS3> Please input your password:\0"

#define SESSION_OUTPUT_MSG_QUE_LEN      (sizeof(ULong))
#define SESSION_OUTPUT_MSG_QUE_CNT      (1)

#define MSG_TO_MSGQUEUE_LENGTH_WITHOUT_STR ( sizeof(U32) )
#define MSG_TO_MSGQUEUE_LENGTH( strLen ) ( MSG_TO_MSGQUEUE_LENGTH_WITHOUT_STR + strLen )

///< 该session私有数据buffer申请的次数
enum {
    SESSION_PRIVATE_BUFFER_CALL_NONE = 0,   ///< 暂时没有被申请
    SESSION_PRIVATE_BUFFER_CALL_MAX = 1,    ///< 申请已满
};

/**
 * @brief   获取是否开启登录功能状态
 * @param   void
 * @return  errno
 */
Bool cliLoginStatGet( void );

/**
 * @brief   设置是否开启登录功
 * @param   void
 * @return  errno
 */
void cliLoginStatSet( Bool isOpen );

/**
 * @brief   通过Id获取session结构体指针
 * @param   sessionId [in]
 * @param   ppSession [out] 读出的session结构体
 * @return  session结构体指针
 */
S32 cliGetSessionById( U32 sessionId, CliSessionObj_s **ppSession);

/**
 * @brief   获取session的私有数据buffer
 * @param   pSession        [in]   session 结构体
 * @param   ppSessionBuf    [out]  获取到的buffer
 * @return  errno
 */
S32 cliCmdPrivDataGet(CliSessionObj_s *pSession, CmdPrivData_s **ppCmdPriv);

/**
 * @brief   session的私有数据buffer put
 * @param   pSession       [in]  session 结构体
 * @return  errno
 */
S32 cliCmdPrivDataPut(CliSessionObj_s *pSession);

/**
 * @brief   获取输入
 * @param   pInputBuff [out] 接收到的输入biffer
 * @param   sessionId  [out] session id
 * @param   inputLen   [in/out] 传入最大长度, 传出获取的长度
 * @return  errno
 * @note 可重入
 */
S32 cliSessionGetInput( InterationInfo_s *pInteration );

/**
 * @brief   填充用于接收用户输入的的数据结构
 * @param   pInterationInfo  [out] session id
 * @param   sessionId        [in]   传入最大长度, 传出获取的长度
 * @param   recvBuf          [in]   接收数据的buffer
 * @return  errno
 */
void prepareInteration(InterationInfo_s *pInterationInfo, U32 sessionId, S8 *recvBuf);

/**
 * @brief   获取session信息
 * @param   void
 * @return  errno
 */
CliSessions_s *cliSessionsGet( void );

/**
 * @brief   获取session状态
 * @param   sessionId       [in]    id
 * @param   status          [out]    要设置的状态
 * @return  errno
 */
SessionStatus_e sessionStatusGet(U32 sessionId);

/**
 * @brief   设置session状态机
 * @param   sessionId       [in]    id
 * @param   status          [in]    要设置的状态
 * @return  errno
 */
void sessionStatusSet(U32 sessionId, SessionStatus_e status);

/**
 * @brief   打印命令行的头
 * @param   sessionId [in]
 * @return  void
 */
void cliTitlePrint (CliSessionPutStringFunc pPutsFunc, U32 sessionId);

/**
 * @brief   初始化全部session
 * @param
 * @return  errno
 */
void cliSessionInit();

#endif ///< __CLI_SESSION_H__
