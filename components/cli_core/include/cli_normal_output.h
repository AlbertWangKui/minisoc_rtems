/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_normal_output.h
 * @author    xiezhj
 * @date      2021.04.12
 * @brief     文件说明
 * @note
 */

#ifndef __CLI_NORMAL_OUTPUT_H__
#define __CLI_NORMAL_OUTPUT_H__

#include "cli_string.h"
#include "cli_std.h"
#include "cli_printf.h"
#include "cli_types.h"

#define INDENT_LENGTH 4
#define STR_MAX_LEN 256

#define INVALID_CODE_U8     0XFF
#define INVALID_CODE_U16    0XFFFF
#define INVALID_CODE_U32    0XFFFFFFFF
#define INVALID_CODE_U64    0XFFFFFFFFFFFFFFFF
#define INVALID_CODE_S8     0X7F
#define INVALID_CODE_S16    0X7FFF
#define INVALID_CODE_S32    0X7FFFFFFF
#define INVALID_CODE_S64    0X7FFFFFFFFFFFFFFF
#define INVALID_CODE_PTR    (NULL)

/**
 * @brief   释放节点中的指定name子节点
 * @param   node, 节点
 * @param   name, 名称
 * @warning 节点中允许重名，该函数删除节点下所有名称符合要求的子节点
 */
void cliNodeDeleteChild(Node_s *pParent, S8 *pName);

/**
 * @brief  将 child 添加到 parent 的子节点中
 * @param  parent
 * @param  child
 */
void cliNodeAddChild(Node_s *pParent, Node_s *pChild);

/**
 * @brief  创建节点
 * @param  parent  父节点
 * @param  title   标题
 * @note   Title XXXXXXXX :
 * @note   ==============
 * @return Node_t*
 */
Node_s *cliNodeCreatObject(Node_s *pParent, S8 *pName);

/**
 * @brief  创建节点
 * @param  parent 父节点
 * @param  format 标题格式
 * @param  ...    标题参数
 * @return Node_t*
 */
Node_s *cliNodeCreatObjectF(Node_s *pParent, S8 *format, ...);

/**
 * @brief  隐藏节点的名称
 * @param  node
 * @note   只在 Normal 模式下不显示名称
 */
void cliNodeHideTitle(Node_s *pNode);

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeAddKeyString(Node_s *pParent, S8 *pKey, S8 *pValue);

/**
 * @brief  添加Value内容
 * @param  parent 父节点
 * @param  value 添加内容
 */
void cliNodeAddStringValue(Node_s *pParent, S8 *pValue);

/**
 * @brief  无需key直接添加Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddStringVal(Node_s *pParent, S8 *format, ...);

/**
 * @brief  添加 Key - Value,并按照len长度进行换行
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeAddKeyStringCRLF(Node_s *pParent, U32 alignLen, S8 *pKey, S8 *pValue);

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = 12345
 */
void cliNodeAddKeyValue(Node_s *pParent, S8 *k, S32 v);

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddKeyStringF(Node_s *parent, S8 *pKey, S8 *format, ...);

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddKeyStringFCRLF(Node_s *pParent, U32 alignLen, S8 *pKey, S8 *format, ...);

/**
 * @brief  添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliNodeAddKeyBuff(Node_s *pParent, S8 *pKey, S8 *pValue, S8 len);

/**
 * @brief  修改父节点下所有名称为 Key 的KV一级子节点的 Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeModifyKeyString(Node_s *pParent, S8 *pKey, S8 *pValue);

/**
 * @brief  修改父节点下所有名称为 Key 的KV一级子节点的 Value
 * @param  parent 父节点
 * @param  k      key
 * @param  v      value
 * @note   kkkkkk = vvvvvvvvvvv
 */
void cliNodeModifyKeyStringF(Node_s *pParent, S8 *pKey, S8 *format, ...);

/**
 * @brief   查找同名表格是否存在
 * @param   parent, 父节点
 * @param   name, 表格名称
 * @return  bool
 */
Bool cliNodeTableExist(Node_s *pParent, S8 *name);

/**
 * @brief  创建表格,如果同名表格已存在,直接返回
 * @param  parent   父节点
 * @param  name     表格名
 * @param  title    表头
 * @note    ----------------------------------------------------------------------
 * @note    Title1  Title2  Title3  Title4  Title5  Title6  Title7  Title8  Title9
 * @note    ----------------------------------------------------------------------
 * @return Table_t*
 */
Node_s *cliNodeCreatTable(Node_s *pParent, S8 *name, S8 *titles[],
        Note_s *pNotes, S32 noteSize);

/**
 * @brief  创建表格的一行
 * @param  table   表格节点
 * @note   Value1  Value2  Value3  Value4  Value5  Value6  Value7  Value8  Value9
 * @return TableRow_t*
 */
TableRow_s *cliNodeCreatTableRow(Node_s *pNode);

/**
 * @brief  表格行内添加一个元素
 * @param  tableRow 表格行
 * @param  str      元素
 */
void cliTableRowAddString(TableRow_s *pTableRow, S8 *pStr);

/**
 * @brief  表格行内添加一个元素
 * @param  tableRow 表格行
 * @param  format   元素格式
 * @param  ...      元素参数
 */
void cliTableRowAddStringF(TableRow_s *pTableRow, S8 *format, ...);

/**
 * @brief  表内添加 Key - Value
 * @param  parent 父节点
 * @param  k      key
 * @param  format value 格式
 * @param  ...    value 参数
 */
void cliTableRowAddBuff(TableRow_s *pTableRow, U8 *v, S32 len);

/**
 * @brief  表格添加备注
 * @param  table  表格节点
 * @param  size   备注个数
 */
void cliNodeAddTableNotes(Table_s *pTable, Note_s *pNotes, S32 size);

/**
 * @brief  表格添加备注
 * @param  table  表格节点
 * @param  k      备注 key
 * @param  v      备注 value
 */
void cliNodeAddTableNote(Table_s *pTable, S8 *pKey, S8 *pValue);

/**
 * @brief  创建数组
 * @param  parent 父节点
 * @param  name   数组名
 * @return Node_t*
 */
Node_s *cliNodeCreatArray(Node_s *pParent, S8 *pName);

/**
 * @brief  同 nodeCreatArray, name可格式化
 * @param  parent 父节点
 * @param  format 格式化字符串
 * @param  ...    格式化参数
 * @return Node_t*
 */
Node_s *cliNodeCreatArrayF(Node_s *pParent, S8 *format, ...);

/**
 * @brief      函数说明
 * @param      [in]
 * @param      [out]
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note       2021.04.12  xiezhj
 */
void cliNodeShowNormal(U32 sessionId, Node_s *pParent);

/**
 * @brief      普通格式显示节点信息
 * @param      [in] sessionId  session Id
 * @param      [in] pNode      要显示的节点
 * @return     void
 * @warning
 * @note       2021.04.12  xiezhj
 */
void cliShowNormal(U32 SessionId, Node_s *pNode);

/**
 * @brief  释放节点空间
 * @param  node   节点
 */
void cliDeleteNode(Node_s *pNode);

#endif ///< __CLI_NORMAL_OUTPUT_H__
