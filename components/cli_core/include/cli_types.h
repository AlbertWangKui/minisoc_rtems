/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_types.h
 * @author  王殿卫
 * @data    2021.04.10
 * @brief   cli api type
 */

#ifndef __CLI_TYPES_H__
#define __CLI_TYPES_H__

#include "cli_list.h"
#include <osp_types.h>
#include <osp_mutex.h>
#include "cli_core_api.h"
#include "cli_porting.h"

#define CLI_UART_LOGIN_TIMEOUT 300000   ///<  UART登录超时时间
#define COMMAND_COUNT 128   ///< 最大命令个数

typedef S32(*CliCmdCallFunc) (Command_s *pCmd);

/**
 * @brief   向session 打印一个字符 函数回调,用法与putchar() 一致
 * @param   ch  [in]  向session打印的字符
 * @return  返回值为ch转换为ASCii码, 错误返回EOF
 */
typedef S32 (*CliSessionPutCharFunc)(S32 ch);

/**
 * @brief   在session 获取一个字符 函数回调,用法与getchar() 一致
 * @param   void
 * @return  session输入的无符号char强转为S32 读错误返回EOF
 */
typedef S32 (*CliSessionGetCharFunc)(void);

/**
 * @brief   在session 获取一个字符串 函数回调,用法与gets() 一致
 * @param   str  [out] 存储的字符串的buffer
 * @return  成功返回接收的字符串 失败返回EOF
 * @note    字符串末尾加'\0'
 */
typedef S8 *(*CliSessionGetStringFunc)(S8 *str);

/**
 * @brief   在session 输出一个字符串 函数回调,用法与puts() 一致
 * @param   str 输出的字符串
 * @return  成功返回字符串长度 失败返回EOF
 * @note    不会输出'\0'
 */
typedef S32 (*CliSessionPutStringFunc)(const S8 *str);

/**
 * @brief   在session 命令执行完成时，自定义操作函数
 * @param   pSession [in] session信息
 * @return
 * @note
 */
typedef void (*CliSessionFinishHandleFunc)(CliSessionType_e type);

typedef struct CliSessionObj CliSessionObj_s;
/**
 * @brief   session 登录回调函数
 * @param   pSession [in] session信息
 * @return
 * @note
 */
typedef S32 (*CliSessionLoginFunc)(CliSessionObj_s *pSession);

///< session信息
typedef struct CliSessionDevInfo{
    CliSessionGetStringFunc      getsFunc;
    CliSessionPutStringFunc      putsFunc;
    CliSessionPutCharFunc        putCharFunc;
    CliSessionGetCharFunc        getCharFunc;
    CliSessionFinishHandleFunc   finshHandleFunc;
    CliSessionLoginFunc          loginFunc;
    CliSessionType_e type;
    Bool isCmdFinish;
} CliSessionDevIo_s;

typedef enum CliMax {
    CLI_MAX_LINE_LENGTH           = 1023,                           ///< 命令行一行最大长度为80,加上NULL共81.
    CLI_STRING_TAIL_LEN           = 1,                              ///< 尾部'\0'的长度
    CLI_MAX_LINE_LENGTH_WITH_TAIL = CLI_MAX_LINE_LENGTH + CLI_STRING_TAIL_LEN,
    CLI_MAX_PARAM_LEN             = 64,                             ///< 一个参数最大长度为64
    CLI_MAX_PARAM_NUM             = 64,                             ///< 最多参数个数
    CLI_MAX_RESULT_LEN            = 1024,                           ///< 执行结果的最大长度
    CLI_MAX_BUF_SIZE              = 256,                           ///< buffer 最大长度
    CLI_BUF_SIZE_DWORD            = 32,
    CLI_KEY_LENGTH                = 64,                             ///< key -value key最大长度
} CommCliMax_e;

/**
 *******************************************************************
 *************************  command  *******************************
 *******************************************************************
 */
typedef struct CommandList {
    S32 count;            ///< 实际命令个数
    Command_s *list[COMMAND_COUNT];
} CommandList_s;

/**
 *******************************************************************
 **************************  user  *********************************
 *******************************************************************
 */
enum {
    CLI_USERNAME_MAX_LEN = 32,
    SESSION_USERNAME_MAX_REAL_LEN = CLI_USERNAME_MAX_LEN - 1, ///< 减去'\0'长度
    SESSION_PASSWORD_MAX_LEN = CLI_PASSWORD_LEN_MAX,
    SESSION_PASSWORD_MAX_REAL_LEN = SESSION_PASSWORD_MAX_LEN - 1, ///< 减去'\0'长度
};

typedef struct CliUserObj {
    S8 name[CLI_USERNAME_MAX_LEN];
    S8 passWord[SESSION_PASSWORD_MAX_LEN];
    U8 level;
    U8 reserved[3];
    struct ListHead listNode;
} CliUserObj_s;

/**
 *******************************************************************
 **************************  session  ******************************
 *******************************************************************
 */
#define CLI_SESSION_MAX_COUNT (3)
#define SESSION_INPUT_LINE_TILE_LEN     (64)

/**
 *                              状态机简易流程
 *
 *    (OFF)--注册-->(INITTING)--初始化-+->(READY)--获取到字符串-->(RUNNING)
 *                                     |                              |
 *                                     +--------------运行------------+
 *
 */
typedef enum SessionStatus {
    SESSION_STATUS_OFF,             ///< session关闭
    SESSION_STATUS_INIT,            ///< session正在初始化
    SESSION_STATUS_READY,           ///< session 就绪
    SESSION_STATUS_RUNNING,         ///< session正在工作
    SESSION_STATUS_NR,
} SessionStatus_e;

///< session向各个模块分发时使用的私有数据信息
typedef struct sessionCmdPrivData {
    U32 callCount;                ///< 引用计数
    CmdPrivData_s data;
} sessionCmdPrivData_s;

///< session信息
struct CliSessionObj {
    CliSessionDevIo_s devIo;                        ///< 设备信息
    CliUserObj_s user;                              ///< 用户
    U32 sessionId;                                  ///< ID
    SessionStatus_e sessionStatus;                  ///< session状态
    sessionCmdPrivData_s cmdPrivData;               ///< 发给各个命令的私有数据
    intptr_t cliSessionTID;                         ///< session的线程ID
    OspID sessionSem;                            ///< 信号量
    Node_s *pPrintParent;                           ///< 输出父节点
    ULong  inputMsgAddr;                            ///< 输入的字符串及信息的地址,会取CliInputMsg_s的地址
    Bool   isOpenLogin;                             ///< 该session是否开启用户登录功能
    U64    loginStamp;                              ///< 最近登录时间戳
};

///< 用于和用户交互获取字符串的结构体
typedef struct InterationInfo {
    U32 sessionId;                                  ///< sessionId
    U32 recvLenExpect;                              ///< 期望获取的长度
    U32 recvLenReal;                                ///< 实际获取的长度
    S8  lineTitle[SESSION_INPUT_LINE_TILE_LEN];     ///< 行首打印的字符串
    S8  *recvString;                                ///< 获取的字符串
    S8  outPutWord;                                 ///< 向外发送的字符, 若等于0, 则获取到什么字符 发送什么字符
} InterationInfo_s;

///< session向消息队列中放入的数据结构
typedef struct CliInputMsg {
    U32 sessionId;
    S8  str[CLI_MAX_LINE_LENGTH_WITH_TAIL];
} CliInputMsg_s;

typedef struct CliSessions{
    OspMutex_t sessionMutex;                                ///< sessions 锁
    CliSessionObj_s sessions[CLI_SESSION_MAX_COUNT];        ///< sessions 数组
    U8  activeSessionCount;                                 ///< sessions 数量
} CliSessions_s;

#endif  ///< __CLI_TYPES_H__
