/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_string.h
 * @author    xiezhj
 * @date      2021.04.08
 * @brief     文件说明
 * @note
 */

#ifndef __CLI_STRING_H__
#define __CLI_STRING_H__

#include "cli_types.h"

typedef struct StrPool {   ///< 字符串池
    S32     count;         ///< 字符串个数
    S8      **list;        ///< 字符串列表
    S32     arrayCount;    ///< 字符串数组个数
    S8      ***listArray;  ///< 字符串数组列表
} StrPool_s;

/**
 * @brief   获取字符串str中指定字符c出现的次数
 * @param   str in 字符串
 * @param   c   in 字符
 * @return  个数
 * @warning 不可重入、阻塞等属性特殊说明，以及其它特别提示等
 * @note    重大修改等
 */
S32 cliGetCharCount(S8 *str, S8 c);

/**
 * @brief   获取字符串str中指定位置的字符
 * @param   str     in 字符串
 * @param   index   in 位置
 * @return  字符
 * @warning 不可重入、阻塞等属性特殊说明，以及其它特别提示等
 * @note    超出范围返回'\0'
 * @note    index非负,从头部计数; index为负数,从尾部开始计数
 */
S8 cliStrGetChar(S8 *str, S32 charIndex);

/**
 * @brief  判断字符串是否全部为数字
 * @param  str 输入字符串
 * @return
 */
Bool cliIsAllDigit(S8 *str);

/**
 * @brief  申请字符串池
 * @return StrPool_t*
 */
StrPool_s *cliStrPoolInit(void);

/**
 * @brief  在字符串池中构造新字符串
 * @param  pool
 * @param  format
 * @param  ...
 * @return S8 *
 */
S8 *cliStrPoolNew(StrPool_s *pool, S8 *format, ...);

/**
 * @brief  子串的个数
 * @param  str  字符串
 * @param  sub  子串
 * @note cliStrPoolCountOf("123123", "12") = 2
 * @note cliStrPoolCountOf("123123", "34") = 0
 */
S32 cliStrPoolCountOf(S8 *str, S8 *sub);

S32 cliStrPoolCountOfCase(S8 *str, S8 *sub);

/**
 * @brief  分割字符串
 * @param  pool     字符串池
 * @param  str      字符串
 * @param  delim    分隔符
 * @note  cliStrPoolSplit(pool, "abc,123,ABC", ",") = {"abc", "123", "ABC", NULL);
 * @note  cliStrPoolSplit(pool, "abc123abc123", "ab") = {"", "c123", "c123", NULL);
 * @note  cliStrPoolSplit(pool, "abcdef", "123") = {"abcdef", NULL);
 */
S8 ** _strPoolSplit(StrPool_s *pool, S8 *str,
                           S8 *delim, Bool ignoreCase);

/**
 * @brief  释放字符串池
 * @param  pool
 */
void cliStrPoolDestroy(StrPool_s *pool);

/**
 * @brief      函数说明
 * @param      [in]
 * @param      [out]
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note       2021.04.10  xiezhj
 */
S8 ** cliStrPoolSplit(StrPool_s *pool, S8 *str, S8 *delim);

/**
 * @brief      判断字符串是否相等
 * @param      [in]
 * @param      [out]
 * @return     EXIT_SUCCESS/-EXIT_FAILURE
 * @warning
 * @note       2021.04.12  xiezhj
 */
Bool cliStrPoolCmpCase(S8 *str1, S8 *str2);

#endif ///< __CLI_STRING_H__
