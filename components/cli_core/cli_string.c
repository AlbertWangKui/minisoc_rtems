/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_string.c
 * @author    xiezhj
 * @date      2021.04.08
 * @brief     字符串处理接口
 * @note
 */

#include "cli_string.h"
#include "cli_std.h"

#define STR_POOL_BUFF_BLOCK 16                                                         ///< 字符串池每次申请的指针数
#define STR_POOL_BUFF_COUNT(n) (((n) / STR_POOL_BUFF_BLOCK + 1) * STR_POOL_BUFF_BLOCK) ///< 字符串池全部的指针数

/**
 * @brief   获取字符串str中指定字符c出现的次数
 * @param   str in 字符串
 * @param   c   in 字符
 * @return  个数
 * @warning 不可重入、阻塞等属性特殊说明，以及其它特别提示等
 * @note    重大修改等
 */
S32 cliGetCharCount(S8 *str, S8 c)
{
    S32 count = 0;
    S32 i     = 0;
    if (str == NULL) {
        goto lEnd;
    }

    for (i = 0; str[i] != '\0'; i++) {
        if (str[i] == c) {
            count++;
        }
    }

lEnd:
    return count;
}

/**
 * @brief   获取字符串str中指定位置的字符
 * @param   str     in 字符串
 * @param   index   in 位置
 * @return  字符
 * @warning 不可重入、阻塞等属性特殊说明，以及其它特别提示等
 * @note    超出范围返回'\0'
 * @note    index非负,从头部计数; index为负数,从尾部开始计数
 */
S8 cliStrGetChar(S8 *str, S32 charIndex)
{
    S8  rc  = '\0';
    S32 len = 0;
    S32 offset = 0;

    if (str == NULL) {
        rc = '\0';
        goto lEnd;
    }
    len = (S32)cliStrLen(str);
    if (charIndex >= 0) {
        rc = (charIndex < len ? str[charIndex] : '\0');
    } else {
        charIndex = -charIndex;
        offset = (len - charIndex);
        rc = (charIndex <= len ? str[offset] : '\0');
    }

lEnd:
    return rc;
}

/**
 * @brief  判断字符串是否全部为数字
 * @param  str 输入字符串
 * @return
 */
Bool cliIsAllDigit(S8 *str)
{
    Bool ret = false;
    S32  i   = 0;
    if (str == NULL || cliStrLen(str) == 0) {
        goto lEnd;
    }
    for (i = 0; str[i] != '\0'; i++) {
        if (cliIsDigit(str[i]) == false) {
            goto lEnd;
        }
    }
    ret = true;

lEnd:
    return ret;
}