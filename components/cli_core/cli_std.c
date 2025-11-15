/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_std.c
 * @author    xiezhj
 * @date      2021.04.07
 * @brief     封装clib库基础接口
 * @note
 */

#include "cli_std.h"
#include "cli_std.h"
#include "cli_porting.h"
#include <string.h>
#include "osp_task.h"
#include "cli_errno.h"

#define YEAR_SINCE  (1900)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/**
 * @brief        cli malloc封装
 * @param        [in] size      需要malloc的内存大小
 * @param        [in] flag      内存类型flag
 * @return       内存指针/NULL
 * @warning
 * @note         2021.09.10  xiezhj
 */
void *cliMalloc(S32 size, U32 flag)
{
    void *ptr = NULL;

    ptr = malloc(size);
    if (NULL != ptr) {
        memset(ptr, 0, (U32)size);
    }
    return ptr;
}

/**
 * @brief        cli free封装
 * @param        [in] pUser   需要释放的内存指针
 * @return       void
 * @warning
 * @note         2021.09.10  xiezhj
 */
void cliFree(void *pUser)
{
    if (pUser != NULL) {
        free(pUser);
    }
    return;
}

/**
 * @brief   通过hal层从uart读取一个字符
 * @param   void
 * @return  读取到的字符
 * @note    由于OS封的getchar接口对字符"退格"未做处理,并且会把退格接收到buffer中 所以暂时直接使用BSP的接口
 */
S32 cliUartGetChar(void)
{
    S32 tempCh = 0;

    ///< 循环读取uart,直到获得一个有意义的字符
    while (1) {
        tempCh = BSP_poll_char();

        if (tempCh >= 0) {
            break;
        }

        ospTaskWakeAfter(1);
    }
    return (tempCh);
}

/**
 * @brief   通过hal层从uart写入一个字符
 * @param   void
 * @return  读取到的字符
 * @note    os封的putchar会写入buffer中, 去除buffer版本会出core  所以暂时直接使用BSP的接口
 */
S32 CliUartPutChar(S32 ch)
{
    BSP_output_char(ch);
    return CLI_ERRNO_OK;
}

/**
 * @brief   向默认io打印格式化字符串
 * @param   fmt 格式化字符串
 * @return  void
 * @note    os封的puts函数会写入buffer中, 去除buffer版本会出core  所以暂时直接使用BSP的接口
 */
S32 cliUartPutString(const S8 *str)
{
    return printf("%s", str);
}

/**
 * @brief        查找原字符串中字串出现位置(不区分大小写)
 * @param        [in] haystack
 * @param        [in] needle
 * @return       第一次出现字串的位置，未匹配返回null
 * @warning
 * @note         2021.04.27  xiezhj
 */
S8 *cliStrCaseStr(const S8 *pHaystack, const S8 *pNeedle)
{
    return (S8 *)strstr(pHaystack, pNeedle);
}

/**
 * @brief        切分字符串
 * @param        [in]  pstr      原字符串
 * @param        [in]  pDelim    分割字符串
 * @param        [out] ppSaveptr 保存剩余串
 * @return       且分出的字符串，失败返回NULL
 * @warning
 * @note         2021.04.27  xiezhj
 */
S8 *cliStrtok(S8 *pStr, const S8 *pDelim, S8 **ppSaveptr)
{
    S8 *p = NULL;
    p = strtok_r(pStr, pDelim, ppSaveptr);
    return p;
}

/**
 * @brief        格式化输出到stdout
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliPrintf(const S8 *format, ...)
{
    va_list ap;
    va_start(ap, format);
    S32 ret = vprintf(format, ap);
    va_end(ap);
    return ret;
}

/**
 * @brief        格式化输出到buffer
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliSNPrintf(S8 *str, S32 size, const S8 *format, ...)
{
    va_list ap;
    va_start(ap, format);
    S32 ret = vsnprintf(str, (U32)size, format, ap);
    va_end(ap);
    return ret;
}

/**
 * @brief        格式化输出到buffer
 * @param        [in]
 * @param        [out]
 * @return       实际输出长度
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliVSNPrintf(S8 *str, S32 size, const S8 *format, cliVaList apList)
{
    return vsnprintf(str, (U32)size, format, apList);
}

/**
 * @brief        将字符串前n个字节清零
 * @param        [in] pStr 原串
 * @param        [in] n    清零个数
 * @return       void
 * @warning
 * @note         2021.04.27  xiezhj
 */
void cliBZero(void *pStr, S32 n)
{
    if (NULL == pStr) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    bzero(pStr, (U32)n);
lEnd:
    return;
}

/**
 * @brief        比较两个字符串前n的字节是否相等(不区分大小写)
 * @param        [in]
 * @param        [out]
 * @return       0 相等，非0 不相等
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliStrNCaseCmp(const S8 *s1, const S8 *s2, S32 n)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if ((NULL == s1) || (NULL == s2)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    rc = strncasecmp(s1, s2, (U32)n);
lEnd:
    return rc;
}

/**
 * @brief        比较两个字符串是否相等(不区分大小写)
 * @param        [in]
 * @param        [out]
 * @return       0 相等，非0 不相等
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliStrCaseCmp(const S8 *s1, const S8 *s2)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if ((NULL == s1) || (NULL == s2)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    rc = strcasecmp(s1, s2);
lEnd:
    return rc;
}

/**
 * @brief        copy
 * @param        [in] dest目标地址
 * @param        [in] src 源地址
 * @param        [in] n   copy字节大小
 * @return
 * @warning
 * @note         2021.04.27  xiezhj
 */
void cliMemCpy(void *dest, const void *src, S32 n)
{
    if ((NULL == dest) || (NULL == src)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    (void)memcpy(dest, src, (U32)n);
lEnd:
    return;
}

/**
 * @brief        比较两个空间前n字节
 * @param        [in]
 * @param        [out]
 * @return       0 相等； >0 str1>str2
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliMemCmp(const void *str1, const void *str2, U32 n)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if ((NULL == str1) || (NULL == str2)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    rc = memcmp(str1, str2, n);
lEnd:
    return rc;
}

/**
 * @brief        空间s前n个字节置为c
 * @param        [in]
 * @param        [out]
 * @return
 * @warning
 * @note         2021.04.27  xiezhj
 */
void cliMemSet(void *s, S32 c, S32 n)
{
    if (NULL == s) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    memset(s, c, (U32)n);
lEnd:
    return;
}

/**
 * @brief        计算字符串长度
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
U32 cliStrLen(const S8 *s)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if (NULL == s) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    rc = (S32)strlen(s);
lEnd:
    return (U32)rc;
}

/**
 * @brief        字符串拼接
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S8 *cliStrcat(S8 *dest, const S8 *src)
{
    S8 *pRet = NULL;

    if ((NULL == dest) || (NULL == src)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    pRet = strcat(dest, src);
lEnd:
    return pRet;
}

/**
 * @brief        将src的前n个字节追加到dest末尾
 * @param        [in]
 * @param        [out]
 * @return
 * @warning
 * @note         2021.04.27  xiezhj
 */
S8 *cliStrNCat(S8 *dest, const S8 *src, S32 n)
{
    S8 *pRet = NULL;

    if ((NULL == dest) || (NULL == src)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    pRet = strncat(dest, src, (U32)n);
lEnd:
    return pRet;
}

/**
 * @brief        比较两个字符串是否相等
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliStrCmp(const S8 *s1, const S8 *s2)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if ((NULL == s1) || (NULL == s2)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    rc = strcmp(s1, s2);
lEnd:
    return rc;
}

/**
 * @brief        字符串比较(区分大小写)
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliStrNCmp(const S8 *s1, const S8 *s2, S32 n)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if ((NULL == s1) || (NULL == s2)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    rc = strncmp(s1, s2, (U32)n);
lEnd:
    return rc;
}

/**
 * @brief        copy字符串前n个字节
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
void cliStrNCpy(S8 *dest, const S8 *src, S32 n)
{
    if ((NULL == dest) || (NULL == src)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    if (n > 0) {
        (void)strncpy(dest, src, (U32)n);
        dest[n - 1] = 0;
    }
lEnd:
    return;
}

/**
 * @brief        查找字符串出现位置(区分大小写)
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S8 *cliStrStr(const S8 *haystack, const S8 *needle)
{
    S8 *pRet = NULL;

    if ((NULL == haystack) || (NULL == needle)) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    pRet = (S8 *)strstr(haystack, needle);
lEnd:
    return pRet;
}

/**
 * @brief        查找字符出现位置(区分大小写)
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S8 *cliStrChr(const S8 *s, S32 c)
{
    S8 *pRet = NULL;

    if (NULL == s) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    pRet = (S8 *)strchr(s, c);
lEnd:
    return pRet;
}

/**
 * @brief        小写转大写
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliToUpper(S32 c)
{
    return toupper(c);
}

/**
 * @brief        大写转小写
 * @param        [in]
 * @param        [out]
 * @return       CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliToLower(S32 c)
{
    return tolower(c);
}

/**
 * @brief        字符转S32
 * @param        [in] nptr 需要转换字符串
 * @param        [in] endptr 转换后剩余字符串
 * @return       转换结果
 * @warning
 * @note         2021.04.27  xiezhj
 */
S32 cliStrToS32(const S8 *pNptr, S8 **endptr, S32 base)
{
    return strtol(pNptr, endptr, base);;
}

/**
 * @brief        将字符串转换为U32类型
 * @param        [in]   nptr 需要转换字符串
 * @param        [in]   endptr 转换后剩余字符串
 * @return       转换结果
 * @warning
 * @note         2021.05.07  xiezhj
 */
U32 cliStrToU32(const S8 *pNptr, S8 **endptr, S32 base)
{
    return strtoul(pNptr, endptr, base);
}

/**
 * @brief        字符转U64
 * @param        [in] nptr 需要转换字符串
 * @param        [in] endptr 转换后剩余字符串
 * @return       转换结果
 * @warning
 * @note         2021.04.27  xiezhj
 */
U64 cliStrToU64(const S8 *pNptr, S8 **endptr, S32 base)
{
    return strtoull(pNptr, endptr, base);;
}

/**
 * @brief        字符转S64
 * @param        [in] nptr 需要转换字符串
 * @param        [in] endptr 转换后剩余字符串
 * @return       转换结果
 * @warning
 * @note         2021.04.27  xiezhj
 */
S64 cliStrToS64(const S8 *pNptr, S8 **endptr, S32 base)
{
    return strtoll(pNptr, endptr, base);
}

/**
 * @brief        判断是否是10进制
 * @param        [in]
 * @return       true/false
 * @warning
 * @note         2021.04.27  xiezhj
 */
Bool cliIsDigit(S32 c)
{
    Bool rc = false;
    rc = (isdigit(c) ? true : false);
    return rc;
}

/**
 * @brief        判断是否是字符
 * @param        [in]
 * @return       true/false
 * @warning
 * @note         2021.04.27  xiezhj
 */
Bool cliIsAlpha(S32 c)
{
    Bool rc = false;
    rc = (isalpha(c) ? true : false);
    return rc;
}
