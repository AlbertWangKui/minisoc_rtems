/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_core.c
 * @author  王殿卫
 * @data    2021.04.10
 * @brief   cli api
 */

#ifndef __CLI_CORE_API_H__
#define __CLI_CORE_API_H__

#include "common_defines.h"

#define ID_LIST_MAX_COUNT       64
#define OBJECT_MAX_COUNT        3   ///< 标识符最大个数:
#define OBJECT_NAME_MAX_LENGTH  16  ///< 最大标识符名称长度

#define CLI_PASSWORD_LEN_MAX (32)

typedef struct CliParam CliParam_s;
typedef struct Command Command_s;
typedef struct CmdPrivData CmdPrivData_s;

enum {
    CLI_LEVEL_0,
    CLI_LEVEL_1,
    CLI_LEVEL_2,
    CLI_LEVEL_3,
    CLI_LEVEL_4,
    CLI_LEVEL_5,
    CLI_LEVEL_LOWEST = 0xFF,
};

enum {
    ARG_OFFSET_0 = 0,
    ARG_OFFSET_1 = 1,
    ARG_OFFSET_2 = 2,
    ARG_OFFSET_3 = 3,
    ARG_OFFSET_4 = 4,
    ARG_OFFSET_5 = 5,
    ARG_OFFSET_6 = 6,
};

/**
 *******************************************************************
 **************************  out put  ******************************
 *******************************************************************
 */
typedef enum NodeType {
    CLI_NODE_TYPE_UNKNOWN,
    CLI_NODE_TYPE_OBJ,    ///< object
    CLI_NODE_TYPE_TAB,    ///< table
    CLI_NODE_TYPE_ARR,    ///< array
    CLI_NODE_TYPE_KV,     ///< key-value
    CLI_NODE_TYPE_VALUE,     ///< value
} NodeType_e;

typedef enum OutputType {
    OUTPUT_NORMAL,
    OUTPUT_JSON,
    OUTPUT_TREE,
    OUTPUT_DUMP,
    OUTPUT_MENU,
} OutputType_e;

typedef struct TableRow {
    S8 ** pStrings;
    U8 count;
} TableRow_s;

typedef struct TableRowList {
    TableRow_s     row;
    struct TableRowList *pNext;
}TableRowList_s;

typedef struct Note { ///< 注释结构
    S8 *pKey;        ///< 注释名
    S8 *pValue;      ///< 注释内容
} Note_s;

typedef struct NoteList {
    Note_s      note;
    struct NoteList *pNext;
} NoteList_s;

typedef struct Table {
    S8 **        titles;         ///< 表头
    TableRowList_s      *pRowList;      ///< 行列表
    NoteList_s          *pNoteList;     ///< 备注列表
    U8                  colCount;       ///< 列数
} Table_s;

typedef struct Node {
    NodeType_e      type;                   ///< 节点类型
    S8              *pName;                 ///< 节点名
    Bool            hideName;               ///< 是否隐藏name字段，隐藏后，Normal模式不显示节点名
    S8              *pValue;                ///< 节点值
    Table_s         *pTable;                ///< 表格
    struct Node     *pParent;               ///< 父节点
    struct Node     *pChild;                ///< 子节点
    struct Node     *pNext;                 ///< 链表
    S32             maxLengthInSubK;        ///< 子节点中所有KV对，Key的最大值，用于对齐输出
    Bool            close;                  ///< 关闭，不显示子节点内容
    U32             alignLen;               ///< 对齐长度
} Node_s;

///< session对应的设备类型
///< 若修改session type需要对应修改cli_session.c中 cliCheckDevInfo 函数中的switch case
typedef enum CliSessionType {
    CLI_SESSION_TYPE_ALL,    ///< 该枚举类型用于命令默认支持所有session
    CLI_SESSION_TYPE_UART   = 0x01,
    CLI_SESSION_TYPE_NR,
} CliSessionType_e;

/**
 *******************************************************************
 *************************  argument  ******************************
 *******************************************************************
 */
typedef struct IdList {
    S32 count; ///< -1:all
    S64 values[ID_LIST_MAX_COUNT];
} IdList_t;

/**
 * @brief  标识符结构
 */
typedef struct Object {
    S8     name[OBJECT_NAME_MAX_LENGTH]; ///< 标识符名称
    IdList_t idList;                       ///< 标识符范围
} Object_s;

/**
 * @brief  参数描述结构
 */
typedef struct Argument {
    S32               page;                         ///< 分页， 0 为不分页
    OutputType_e      outputType;                   ///< 输出格式
    Bool              isHelp;                       ///< 是否为帮助信息
    Bool              isComplete;                   ///< 是否为完善命令参数命令
    Bool              gitVersion;                   ///< 是否显示git分支
    Bool              showTraceId;                  ///< 是否显示错误的traceId
    U8                objectCount;                  ///< 标识符个数
    Object_s          objectList[OBJECT_MAX_COUNT]; ///< 标识符 ( '/' 起始的字符串为标识符)
    S8                *cur;                         ///< 命令补全时,当前光标所在的参数
    S32               argc;                         ///< 输入参数个数
    S8                *argv[];                      ///< 输入参数列表
} Argument_s;

///< 分发时使用的私有数据
struct CmdPrivData {
    U32 sessionId;
    Argument_s *pArg;
    Command_s *pCmd;
    Node_s *pParent;
    CliSessionType_e sessionType;
};

/**
 * @brief   单个参数结构体
 *          key=value格式,匹配到则value为"value"
 *          key格式,匹配到则value为"key"
 *          key1|key2|key3,匹配到key2,则value为"key2"
 *          没有匹配到key字段，value为NULL
 */
struct CliParam {
    S8          *key;                   ///< 参数字段名称
    S8          *allow;                 ///< 允许的参数范围,若不校验则为NULL
    S8          *allowEnum;             ///< allow对应的枚举值, 仅当allow不为空时生效
    S8          *depend;                ///< 该参数的依赖项,若不依赖其他参数,则为NULL
    S8          *value;                 ///< 解析到的参数值
    S8          *displayName;           ///< 打印到终端输出的字段名称
    S8          *minValue;              ///< 数值参数最小值
    S8          *maxValue;              ///< 数值参数最大值
    S8          *moduleName;            ///< 命令的模块名
    Bool        isNecessary;            ///< 该参数是否是必选参数
    Bool        noEqualSign;            ///< 是否不包含等号
    U8          followArgsNum;          ///< 该参数跟随的参数个数
    CliParam_s  *followArgsList;        ///< 跟随的参数列表

    S32        (*paramCheck)(CliParam_s *, Node_s *pNode);             ///< 参数检查回调
    S32        (*paramCheckCtrlSet)(CliParam_s *);      ///< 参数检查回调
    S32        (*getParamValue)(CliParam_s*, U32 *len, U8 **value);
    S32        (*getParamValueCheck)(CliParam_s*, U32 *len, U8 **value);
    S32        (*handleParam)(Node_s*, CliParam_s*);   ///< 处理该参数的回调
};

/**
 * @brief  命令
 */
struct Command {
    S8          *argument;                      ///< 命令模板
    U8          cmdLevel;                       ///< 命令等级
    U8          suppSession;                    ///< 命令支持的session
    S8          *name;                          ///< 命令名称
    S8          *syntax;                        ///< 命令语法
    S8          *description;                   ///< 命令描述
    S8          *option;                        ///< 命令参数描述
    S8          *convention;                    ///< 命令惯例
    S8          *note;                          ///< 命令提示
    Bool        needHelp;                       ///< 命令自行解析help参数
    U32         argOffset;                      ///< 参数偏移
    U32         paramCnt;                       ///< 参数个数
    CliParam_s *pParam;                         ///< 参数列表
    Bool        isSync;                         ///< 命令是否同步运行
    S32 (*handleCmd)(CmdPrivData_s *);          ///< 命令回调函数
};

/**
 * @brief      命令注册函数
 * @param      [in]
 * @param      [out]
 * @return     void
 * @warning    阻塞；不可重入；OS启动后， cli_core初始化后；不可用于中断上下文；可以用于线程上下文
 * @note       2021.04.12  xiezhj
 */
void cliRegisterCmd(Command_s *pCmd);

/**
 * @brief      命令注册函数
 * @param      [out] pPassWd 密码字符串，最大程度为：CLI_PASSWORD_LEN_MAX
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning    阻塞；不可重入；OS启动后， cli_core初始化后；不可用于中断上下文；可以用于线程上下文
 * @note       2025.07.07  taohb
 */
S32 cliPassWdGet(S8 *pPassWd);

/**
 * @brief      登录密码设置
 * @param      [in] pPassWd 密码字符串，最大程度为：CLI_PASSWORD_LEN_MAX
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning    阻塞；不可重入；OS启动后， cli_core初始化后；不可用于中断上下文；可以用于线程上下文
 * @note       2025.07.07  taohb
 */
S32 cliPassWdSet(S8 *pPassWd);

/**
 * @brief   cli shell初始化封装
 * @param   void
 * @return  EXIT_SUCCESS/-EXIT_FAILURE
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 * @note    2025.07.07  taohb
 */
S32 clishellInit(void);

#endif ///< __CLI_CORE_API_H__
