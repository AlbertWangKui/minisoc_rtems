/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_argument.c
 * @author    xiezhj
 * @date      2021.04.07
 * @brief     cli argument相关解析及参数检查等接口
 * @note
 */

#include "cli_argument.h"
#include "cli_std.h"
#include "cli_string.h"
#include "cli_porting.h"
#include "cli_core.h"

///< 适配gcc 8.4编译问题，添加内联函数声明
static void maxSwap(S32 *pMax, S32 *pMin);
static Bool isConnector(const S8 c);
static Bool startWithAll(const S8 *str);

/**
 * @brief   将命令字符串解析成argv
 * @param   pInputBuf [in] get到的字符串
 * @param   pArgc     [out] 解析到的参数个数
 * @param   argv      [out] 参数列表
 * @return  CliErrno错误码
 * @note    2021.04.08  xiezhj
 */
S32 cliStringParse(S8 *pInputBuf, S32 *pArgc, S8 *argv[], CmdPrivData_s *pPriv)
{
    S32 argc       = 0;
    U32 paramLen   = 0;
    S8  *pParamStr = { 0 };
    S8  *sp        = NULL;
    S32 rc  = CLI_ERRNO_INVALID;

    ///< 参数检查
    if (NULL == pInputBuf) {
        rc = -CLI_ERRNO_PARAM_NULL;
        CLI_LOG_ERROR("input para is null, rc = %08x.", rc);
        goto lEnd;
    }

    ///< 以空格为分隔符切割字符串
    pParamStr = cliStrtok(pInputBuf, PARAM_DELIMITER, &sp);

    ///< 输入字符串解析
    while (pParamStr != NULL) {
        paramLen = cliStrLen(pParamStr);                        ///< 切分出的字符串长度

        if (paramLen >= CLI_MAX_PARAM_LEN) {               ///< 字符串长度超过最大值 64bytes
            rc = -CLI_ERRNO_INPUT_BEYOND_LENGTH;
            CLI_LOG_ERROR("The input is beyond len. Input len is %d, max len allowed is %d"
                           "rc = %08x", paramLen, CLI_MAX_PARAM_LEN, rc);
            goto lEnd;
        } else if (argc >= CLI_MAX_PARAM_NUM) {            ///< 参数个数超过限制64个
            rc = -CLI_ERRNO_PARAM_NUM_BEYOND_LIMIT;
            CLI_LOG_ERROR("The num of param is beyond. Input num is %d, max num allowed is %d"
                           "rc = %08x", argc, CLI_MAX_PARAM_NUM, rc);
            goto lEnd;
        } else {                                                ///< 参数符合规范
            argv[argc] = pParamStr;
            argc++;
        }
        ///< 继续切分字符串
        pParamStr = cliStrtok(NULL, PARAM_DELIMITER, &sp);
    }

    *pArgc = argc;
    rc = CLI_ERRNO_OK;
lEnd:
    return rc;
}

/**
 * @brief        函数说明
 * @param        [in]
 * @param        [out]
 * @return       void
 * @warning
 * @note         2021.04.21  xiezhj
 */
static inline void maxSwap(S32 *pMax, S32 *pMin)
{
    S32 tmp = 0;
    if (*pMax < *pMin) {
        tmp = *pMin;
        *pMin = *pMax;
        *pMax = tmp;
    }
    return;
}

/**
 * @brief      内联函数，判断字符是否为连接符
 * @param      [in] c 输入字符
 * @return     true/false
 * @warning    连接符:',' \ '-' \ ':';
 * @note       2021.04.14  xiezhj
 */
static inline Bool isConnector(const S8 c)
{
    Bool rc = false;

    if ((c == ',') || (c == '-') || (c == ':')) {
        rc = true;
    }
    return rc;
}

/**
 * @brief      内联函数，判断字符串是否以"all"开头
 * @param      [in]   str   输入字符串
 * @return     true/false
 * @warning    不区分大小写
 * @note       2021.04.14  xiezhj
 */
static inline Bool startWithAll(const S8 *str)
{
    Bool ret = false;

    if (str == NULL) {
        ret = false;
        goto lEnd;
    }

    ///< 判断是否all开头;
    if ((str[0] == 'a' || str[0] == 'A') && (str[1] == 'l' || str[1] == 'L') &&
        (str[2] == 'l' || str[2] == 'L') && (str[3] == '/' || str[3] == '\0')) {
        ret = true;
    }

lEnd:
    return ret;
}

/**
 * @brief      将字符串拼接到当前参数的最后面
 * @param      [in] pArgument
 * @param      [in] str
 * @param      [in] maxArgumentCount
 * @return     void
 * @warning
 * @note       2021.04.14  xiezhj
 */
static void cliAddArgument(Argument_s *pArgument, const S8 *pStr, S32 maxArgumentCount)
{
    S32 tmpIndex = pArgument->argc - 1;

    ///< 去const
    if (pArgument->argv[tmpIndex] == NULL) {
        if (tmpIndex == 0) {
            *(void **)&pArgument->argv[tmpIndex] = (S8 *)(pArgument->argv + maxArgumentCount + 1);
        } else {
            *(void **)&pArgument->argv[tmpIndex] = (S8 *)(pArgument->argv[tmpIndex - 1]
                                                         + cliStrLen(pArgument->argv[tmpIndex - 1]) + 1);
        }
    }
    ///< 拼接字符串;
    (void)cliStrcat(*(void **)&pArgument->argv[tmpIndex], pStr);
}

/**
 * @brief      合并参数， /c 0 -> /c0;  /c 1 , 2 - 4 -> /c1,2-4;   /c 1 all -> /c1all
 * @param      [in] pArgument 被修改的参数结构
 * @param      [in] pStr      输入参数
 * @param      [in] len       输入参数长度
 * @param      [in] maxArgumentCount 最大的参数个数
 * @return     void
 * @warning
 * @note       2021.04.14  xiezhj
 */
static void cliMergeArgument(Argument_s *pArgument, S8 *pStr, S32 len, S32 maxArgumentCount)
{
    S8 *pre = NULL;
    S8 last = 0;
    S8 next = 0;

    ///< 判断合并参数长度，如果<0，则合并整个字符出，否则合并pStr的前len个字符串
    if (len < 0) {
        len = (S32)cliStrLen(pStr);
    }

    ///< 定义局部数组
    S8 cur[len + 1];

    ///< 入参检查
    if ((NULL == pArgument) || (NULL == pStr)) {
        CLI_LOG_ERROR(
            "Input para is null, pArgument=NULL?:%d, pStr=NULL?:%d, rc=%08x.",
            (NULL == pArgument), (NULL == pStr), CLI_ERRNO_PARAM_NULL);
            goto lEnd;
    }

    (void)cliStrNCpy(cur, pStr, (S32)sizeof(cur));

    if (pArgument->argc == 0) {                         ///< argv中无内容，直接添加
        pArgument->argc++;
        cliAddArgument(pArgument, cur, maxArgumentCount);
        goto lEnd;
    } else {
        pre  = pArgument->argv[pArgument->argc - 1];    ///< 当前处理的参数
        last = cliStrGetChar(pre, -1);                  ///< 上一个参数的最后一个字符
        next = cliStrGetChar(cur, 0);                   ///< 当前参数的第一个字符

        if (cliStrGetChar(pre, 0) == '/') {                                                     ///< /xxx
            if (((last == '/') && cliIsAlpha(next))                                               ///< /xx/ cxx
                || (isConnector(last) && cliIsDigit(next))                                      ///< /xx- 1xx
                || (cliIsDigit(last) && isConnector(next) && cliIsDigit(cliStrGetChar(cur, 1))) ///< /xx1 -1x
                || (cliIsAlpha(last) && cliIsDigit(next))                                       ///< /xxc 1xx
                || (cliIsAlpha(last) && startWithAll(cur))                                      ///< /xxa all
                || (isConnector(last))                              ///< /xx- xxx 只要前面是连接符，就合并
                || (isConnector(next) && (cliStrLen(cur) == 1))       ///< /xx - 单独的 -, 合并
                ///< || (isConnector(next) && cliIsDigit(cliStrGetChar(cur, 1))) ///< /xxx -1xx 只要后面是连接符+数字，就合并
            ) {
                cliAddArgument(pArgument, cur, maxArgumentCount);
                goto lEnd;
            }
        } else if (((last == '=') || (next == '='))                     ///< a = or = b
            && (cliStrCaseCmp(cur, OPTION_COMPLETE_LONG) != 0)) {     ///< 参数为命令行补全关键字时不拼接
            cliAddArgument(pArgument, cur, maxArgumentCount);
            goto lEnd;
        }

        pArgument->argc++;
        cliAddArgument(pArgument, cur, maxArgumentCount);
    }
lEnd:
    return;
}

/**
 * @brief      分割参数 /c0/v0 -> /c0 /v0
 * @param      [in] argument    被修改的参数结构
 * @param      [in] argc        输入参数个数
 * @param      [in] argv        输入参数列表
 * @param      [in] maxArgumentCount    最大参数个数
 * @return     void
 * @warning
 * @note       2021.04.14  xiezhj
 */
static void cliSplitArgument(Argument_s *pArgument, S32 argc, S8 *argv[], S32 maxArgumentCount)
{
   Bool stopObjectAnalyze = false;  ///< 不再继续拆分 '/' 起始的object,false 继续拆分
   S8   *str              = NULL;
   S8   *head             = NULL;
   S8   next              = 0;
   S32  idx               = 0;
   U32  idj               = 0;
   S32  size              = 0;
   U32  len               = 0;
   U64  nextIdx           = 0;

   for (idx = 0; idx < argc; idx++) {
        str = argv[idx];
        if ((stopObjectAnalyze == false) && (cliGetCharCount(str, '/') > 0)) {  ///< 参数中含有 /
            len  = cliStrLen(str);
            head = str;
            size = 0;
            for (idj = 0; idj < len; idj++) {
                size++;
                nextIdx = (idj + 1);
                next = str[nextIdx];
                if ((next == '/') || (next == '\0')) {
                    cliMergeArgument(pArgument, head, size, maxArgumentCount);  ///< 将 / 前面的参数合并到当前参数
                    nextIdx = (idj + 1);
                    head = &str[nextIdx];
                    size = 0;
                }
            }
        } else {
            cliMergeArgument(pArgument, str, -1, maxArgumentCount);
            if ((pArgument->argc - 1 >= 0)
                && (pArgument->argv[pArgument->argc - 1] != NULL)
                && (pArgument->argv[pArgument->argc - 1][0] != '/')) {  ///< 判断前一个参数不是 / 起始，后面不再拆分
                stopObjectAnalyze = true;               ///< 不再继续拆分 '/' 起始的object, 可能是文件的绝对路径
            }
        }
    };

    return;
}

/**
 * @brief      获取参数列表中的额外参数，并从参数列表中删除额外参数
 * @param      [in] pArgument
 * @return     void
 * @warning
 * @note       额外参数为 -j json tree t d -h help
 * @note       从参数列表后面开始获取，遇到非额外参数时停止
 * @note       2021.04.14  xiezhj
 */
void cliGetExtraParameters(Argument_s *pArgument)
{
    S8 *pLast  = NULL;

    ///< 入参检查
    if (NULL == pArgument) {
        CLI_LOG_ERROR("Input para is null, rc=%08x", CLI_ERRNO_PARAM_NULL);
        goto lEnd;
    }

    ///< 从最后一个参数开始解析
    while (pArgument->argc != 0) {
        pLast = pArgument->argv[pArgument->argc - 1];                       ///< 从最后参数向前一次解析
        if ((cliStrCaseCmp(pLast, OPTION_HELP_SHORT) == 0) ||
            (cliStrCaseCmp(pLast, OPTION_HELP_LONG) == 0)) {                ///< help 信息
            pArgument->isHelp = true;
        } else {
            break; ///<  只从后面分析参数，中间的不分析，不符合以上几种情况退出循环
        }
        pArgument->argc--;                                  ///< 删除额外参数
        *(void **)&pArgument->argv[pArgument->argc] = NULL; ///< 去const
    }
lEnd:
    return;
}

/**
 * @brief      获取字符串中的标识名
 * @param      [in] pStr        输入字符串
 * @param      [out] pObject    返回的标识符结构体
 * @return     true/false
 * @warning
 * @note       2021.04.14  xiezhj
 */
static Bool cliFillObjectName(S8 *pStr, Object_s *pObject)
{
    Bool ret   = false;
    S8   *pAll = NULL;
    S32  idx   = 0;

    ///<  入参检查
    if ((NULL == pStr) || (NULL == pObject)) {
        CLI_LOG_ERROR(
            "Input para is null, pStr=null?:%d, pObject=null:?%d, rc=%08x.",
            (NULL == pStr), (NULL == pObject), CLI_ERRNO_PARAM_NULL);
        goto lEnd;
    }

    pAll = cliStrCaseStr(pStr, ALL_TOKEN);
    if (pAll != NULL) {
        if (cliStrCaseCmp(pAll, ALL_TOKEN) != 0) { ///<  all 后面还有其他字符
            CLI_LOG_ERROR("some char after 'all'.");
            ret = false;
            goto lEnd;
        }

        (void)cliStrNCpy(pObject->name, pStr, (S32)(cliStrLen(pStr) - ALL_SIZE + 1));   ///< +1 '\0'
        pObject->idList.count = -1;
        ret = true;
        goto lEnd;
    }
    for (idx = 0; idx < (S32)(ARRAY_SIZE(pObject->name)) - 1; idx++) {
        if (cliIsAlpha(pStr[idx])) {
            pObject->name[idx] = pStr[idx];
        } else {
            break;
        }
    }
    ret = true;
lEnd:
    return ret;
}

/**
 * @brief      添加数字范围到object
 * @param      [in] pObject     标识符
 * @param      [in] min         最小值，闭区间
 * @param      [in] max         最大值，闭区间
 * @return     true/false
 * @warning
 * @note       2021.04.14  xiezhj
 */
static Bool cliAddObjectNum(Object_s *pObject, S32 min, S32 max)
{
    S32  count  = 0;
    S32  idx    = 0;
    S32  idj    = 0;
    S64  valNum = 0;
    Bool ret    = false;

    ///< 入参检查
    if (NULL == pObject) {
        CLI_LOG_ERROR("Input para is null,pObject=null, rc=%08x.", CLI_ERRNO_PARAM_NULL);
        goto lEnd;
    }

    maxSwap(&max, &min);

    idx = pObject->idList.count;
    count = max - min + 1;
    pObject->idList.count += count;
    if ((U32)pObject->idList.count >= ARRAY_SIZE(pObject->idList.values)) {
        CLI_LOG_ERROR("Invalid Object Count, Cannot exceed 64.");
        ret = false;
        goto lEnd;
    }
    for (idj = 0; idj < count; idj++) {
        valNum = idx + idj;
        pObject->idList.values[valNum] = min + idj;
    }
    ret = true;
lEnd:
    return ret;
}

/**
 * @brief      获取字符串中的标识范围
 * @param      [in]     str     输入字符串
 * @param      [out]    pObject 标识符结构体
 * @return     true/false
 * @warning
 * @note       /vall => -1   /v1,2 => 1,2  /v1-3 => 1,2,3
 * @note       2021.04.14  xiezhj
 */
static Bool cliFillObjectNum(S8 *pStr, Object_s *pObject)
{
    U32  len     = 0;
    Bool ret     = false;
    S8   *pSave  = NULL;       ///< 保存cliStrtok函数中间过程值
    S8   *pToken = NULL;
    S32  value   = 0;
    U32  tmpLen  = 0;
    S8   *s      = NULL;       ///< cliStrtok 函数需要一个临时变量保存中间过程值
    S8   *t1     = NULL;
    S8   *t2     = NULL;
    S32  v1      = 0;
    S32  v2      = 0;

    pStr += cliStrLen(pObject->name);
    len = cliStrLen(pStr) + 1;
    S8      buf[len];

    ///< 入参检查
    if ((NULL == pStr) || (NULL == pObject)) {
        CLI_LOG_ERROR(
            "Input para is null, pStr=null?:%d, pObject=null?:%d, rc=%08x.",
            (NULL == pStr), (NULL == pObject), CLI_ERRNO_PARAM_NULL);
            goto lEnd;
    }

    (void)cliStrNCpy(buf, pStr, (S32)sizeof(buf));

    pToken = cliStrtok(buf, ",", &pSave); ///< 按 "," 分割
    while (pToken != NULL) {
        ret = cliIsAllDigit(pToken);
        if (true == ret) { ///< 全数字
            value = cliStrToS32(pToken, NULL, 10);
            ret = cliAddObjectNum(pObject, value, value);
            if (false == ret) {
                CLI_LOG_ERROR("Add object num fail,ret=%08x.", ret);
                goto lEnd;
            }
        } else if (cliGetCharCount(pToken, '-') == 1) {     ///< 包含 "-" 符号
            tmpLen = cliStrLen(pToken) + 1;                 ///< 因为下面逻辑会导致 token 更改
            S8 tmpBuf[tmpLen];                              ///< 所以构造一个临时字符串
            (void)cliStrNCpy(tmpBuf, pToken, (S32)sizeof(tmpBuf));                     ///< 存储token
            t1 = cliStrtok(tmpBuf, "-", &s);                ///< 取出 "-" 前的字符串
            t2 = cliStrtok(NULL, "-", &s);                  ///< 取出 "-" 后的字符串
            if (t2 == NULL) {                               ///< "-" 后没有字符串 错误
                CLI_LOG_ERROR("Parse object num fail,ret=%08x.", ret);
                ret = false;
                goto lEnd;
            }
            if (cliIsAllDigit(t1) && cliIsAllDigit(t2)) { ///< "-" 前后应该为纯数字
                v1 = cliStrToS32(t1, NULL, 10);
                v2 = cliStrToS32(t2, NULL, 10);
                ret = cliAddObjectNum(pObject, v1, v2);
                if (false == ret) {
                    CLI_LOG_ERROR("Add object num fail, ret=%08x.", ret);
                    goto lEnd;
                }
            } else {
                CLI_LOG_ERROR("Object num is invalid.");
                ret = false;
                goto lEnd;
            }

        } else if (pObject->idList.count < 0) { ///< 小于0 是 all
            ///< Nothing to do
        } else {
            CLI_LOG_ERROR("Object is invalid.");
            ret = false;
            goto lEnd;
        }
        pToken = cliStrtok(NULL, ",", &pSave);
    }
    ret = true;
lEnd:
    return ret;
}

/**
 * @brief      获取字符串描述的标识符
 * @param      [out]     pObject 标识符
 * @param      [in]      str     目标字符串
 * @return     true/false
 * @warning
 * @note       str  -> name : value
 * @note       2021.04.14  xiezhj
 */
static Bool cliFillObject(Object_s *pObject, S8 *pStr)
{
    Bool ret = false;

    ///< 入参检查
    if ((NULL == pObject) || (NULL == pStr)) {
        CLI_LOG_ERROR("Input para is null, pObject=null?:%d, pStr=null?:%d.",
                      (NULL == pObject), (NULL == pStr));
        goto lEnd;
    }

    if (cliStrGetChar(pStr, 0) != '/') {    ///< 标识符必须 '/' 起始
        CLI_LOG_ERROR("Object not begin with \'/\'");
        ret = false;
        goto lEnd;
    }

    pStr++;        ///< 跳过'/'
    ret = cliFillObjectName(pStr, pObject);
    if (false == ret) {
        CLI_LOG_ERROR("%s name error, ret=%08x", pStr, ret);
        goto lEnd;
    }

    ret = cliFillObjectNum(pStr, pObject);
    if (false == ret) {
        CLI_LOG_ERROR("%s number error, ret=%08x.", pStr, ret);
        goto lEnd;
    }
    ret = true;
lEnd:
    return ret;
}

/**
 * @brief      获取 argument 中的标识符object信息
 * @param      [in] pArgument
 * @return     true/false
 * @warning
 * @note       2021.04.14  xiezhj
 */
static Bool cliGetObjectArgument(Argument_s *pArgument, CmdPrivData_s *pPriv)
{
    S32  idx   = 0;
    S8   *pStr = NULL;
    Bool ret   = false;

    ///< 入参检查
    if (NULL == pArgument) {
        CLI_LOG_ERROR("Input para is null, rc=%08x.", -CLI_ERRNO_PARAM_NULL);
        goto lEnd;
    }

    pArgument->objectCount = 0;
    for (idx = 0; (idx < pArgument->argc) &&(pArgument->argv[idx][0] == '/'); idx++) {
        pStr = pArgument->argv[idx];
        if (pArgument->objectCount >= ARRAY_SIZE(pArgument->objectList)) {
            CLI_LOG_ERROR("Too many object, real size=%d, max size=%zd.",
                            pArgument->objectCount, ARRAY_SIZE(pArgument->objectList));
            cliNodeAddStringVal(pPriv->pParent, "Object num must no greater than %d.",
                            ARRAY_SIZE(pArgument->objectList));
            ret = false;
            goto lEnd;
        }

        ret = cliFillObject(&pArgument->objectList[pArgument->objectCount], pStr);
        if (false == ret) {
            CLI_LOG_ERROR("Fill object fail.");
            cliNodeAddStringVal(pPriv->pParent, "Object [%s] is invalid.", pStr);
            goto lEnd;
        }
        pArgument->objectCount++;
    }
    ret = true;
lEnd:
    return ret;
}

/**
 * @brief      获取输入参数列表的结构化对象
 * @param      [in] argc   参数个数
 * @param      [in] argv   参数列表
 * @return     pArgument
 * @warning
 * @note       2021.04.14  xiezhj
 */
Argument_s *cliGetArgument(S32 argc, S8 *argv[], CmdPrivData_s *pPriv)
{
    S32        maxArgumentLength = 0;          ///< 参数的最大长度
    S32        maxArgumentCount  = argc;       ///< 参数的最大个数
    Argument_s *pArgument        = NULL;
    S32        i                 = 0;
    S32        count             = 0;
    Bool       ret               = false;

    if (NULL == argv) {
        CLI_LOG_ERROR("Input para argv is null.");
        goto lEnd;
    }

    for (i = 0; i < argc; i++) {
        count = cliGetCharCount(argv[i], '/') + 1; ///< 当前参数拆分后的参数个数
        maxArgumentLength += (S32)(cliStrLen(argv[i]) + count * sizeof('\0'));
        maxArgumentCount += count;
    }

    pArgument = cliMalloc((S32)(sizeof(Argument_s)
                                + (maxArgumentCount + 1) * sizeof(S8 *) + maxArgumentLength), 0);
    if (NULL == pArgument) {
        CLI_LOG_ERROR("pArgument alloc fail.");
        goto lEnd;
    }

    cliSplitArgument(pArgument, argc, argv, maxArgumentCount);  ///< 分割操作对象 /c1/v1 -> /c1 /v1

    cliGetExtraParameters(pArgument);

    ret = cliGetObjectArgument(pArgument, pPriv);
    if (false == ret) {
        CLI_LOG_ERROR("Get object fail.");
        cliFree(pArgument);
        pArgument = NULL;
        goto lEnd;
    }

lEnd:
    return pArgument;
}

/**
 * @brief      对用户输入的参数进行解析,输出到Param结构体数组中
 * @param      [in] argc        参数个数
 * @param      [in] argv        参数列表
 * @param      [out] paramList  解析生成的参数结构体列表
 * @param      [in] paramCount  参数结构体个数
 * @return     处理的参数个数
 * @warning
 * @note        参数不包含"="时，将参数整体作为key，value为用户实际输入
 * @note        若param列表中包含'|'分隔符，则匹配到其中任意一项即为成功匹配
 * @note       2021.04.14  xiezhj
 */
static S32 cliParseFollowParam(S32 argc, S8 *argv[], CliParam_s *pParamList, S32 paramCount)
{
    S32  handleArgsNum          = 0;
    S8   key[CLI_KEY_LENGTH]    = { 0 };
    S8   tmpKey[CLI_KEY_LENGTH] = { 0 };
    S8   *pValue                = NULL;
    S32  idx                    = 0;
    S32  idj                    = 0;
    Bool findFlag               = false;
    Bool noEqualSign            = false;  ///< 是否包含等号
    S8   *pTokStr               = NULL;
    S8   *sp                    = NULL;

    if ((NULL == argv) || (NULL == pParamList)) {
        CLI_LOG_ERROR("Input para is null, argv=null?:%d, pParamList=null?:%d.",
                      (NULL == argv), (NULL == pParamList));
        goto lEnd;
    }

    ///< 静态变量 清空参数value值
    for (idx = 0; idx < paramCount; idx++) {
        pParamList[idx].value = NULL;
    }

    for (idx = 0; idx < argc; idx++) {         ///< 遍历用户输入的参数
        findFlag = false;
        (void)cliMemSet(key, 0, (S32)sizeof(key));
        pValue = cliStrChr(argv[idx], '=');
        if (pValue != NULL) {                 ///< key=value格式
            noEqualSign = false;
            (void)cliMemCpy(key, argv[idx], pValue - argv[idx]);
            pValue++;
        } else {                            ///< 不包含等号
            noEqualSign = true;
            (void)cliMemCpy(key, argv[idx], (S32)cliStrLen(argv[idx]));
            pValue = argv[idx];
        }

        for (idj = 0; idj < paramCount; idj++) {               ///< 遍历该命令可接收的参数列表
            cliMemSet(tmpKey, 0, (S32)sizeof(tmpKey));
            cliMemCpy(tmpKey, pParamList[idj].key, (S32)strlen(pParamList[idj].key));
            pTokStr = cliStrtok(tmpKey, "|", &sp);
            while (pTokStr != NULL) {
                if (cliStrCaseCmp(key, pTokStr) == 0) {
                    findFlag = true;
                    if (pParamList[idj].noEqualSign != noEqualSign) {
                        handleArgsNum = idx;
                        goto lEnd;
                    }
                    if (pParamList[idj].value == NULL) {    ///< 从未解析到该参数
                        pParamList[idj].value = pValue;
                    } else {        ///< 参数重复
                        CLI_LOG_ERROR("Parse Param %s Failed: Repeated Param.\n", key);
                        handleArgsNum = idx;
                        goto lEnd;
                    }
                    break;
                }
                pTokStr = cliStrtok(NULL, "|", &sp);
            }
        }

        if (findFlag == false) {
            handleArgsNum = idx;
            goto lEnd;
        } else {
            handleArgsNum = idx + 1;
        }
    }

lEnd:
    return handleArgsNum;
}

/**
 * @brief      对用户输入的参数进行解析,输出到Param结构体数组中,
 * @brief      支持格式：key=value, key, key1|key2|key3
 * @param      [in] argc        参数个数
 * @param      [in] argv        参数列表
 * @param      [out] paramList  解析生成的参数结构体列表
 * @param      [in] paramCount  参数结构体个数
 * @return     处理的参数个数
 * @warning
 * @note        参数不包含"="时，将参数整体作为key，value为用户实际输入
 * @note        若param列表中包含'|'分隔符，则匹配到其中任意一项即为成功匹配
 * @note       2021.04.14  xiezhj
 */
S32 cliParseParam(S32 argc, S8 *argv[], CliParam_s *pParamList, S32 paramCount)
{
    S32        ret                    = CLI_ERRNO_INVALID;
    S8         key[CLI_KEY_LENGTH];
    S8         tmpKey[CLI_KEY_LENGTH];
    S8         *pValue                = NULL;
    S32        loopCnti               = 0;
    S32        loopCntj               = 0;
    S32        argcRemain             = 0;
    S32        followArgsNum          = 0;
    S32        argcTmp                = 0;
    S8         **argvTmp              = NULL;
    S32        handleArgsNum          = 0;                  ///< 处理的参数个数
    Bool       findFlag               = false;
    Bool       noEqualSign            = false;          ///< 是否匹配到
    CliParam_s *pParaMatched          = NULL;               ///< 匹配到的param
    CliParam_s *followParamList       = NULL;
    S8         *pTokStr               = NULL;
    S8         *sp                    = NULL;

    if ((NULL == argv) || (NULL == pParamList)) {
        CLI_LOG_ERROR("Input para is null, argv=null?:%d, pParamList=null?:%d.",
                      (NULL == argv), (NULL == pParamList));
        goto lEnd;
    }

    ///< 静态变量 清空参数value值
    for (loopCnti = 0; loopCnti < paramCount; loopCnti++) {
        pParamList[loopCnti].value = NULL;
    }

    for (loopCnti = 0; loopCnti < argc; loopCnti++) {         ///< 遍历用户输入的参数
        findFlag = false;
        (void)cliMemSet(key, 0, (S32)sizeof(key));
        pValue = cliStrChr(argv[loopCnti], '=');

        if (pValue != NULL) {                 ///< key=value格式
            noEqualSign = false;
            (void)cliMemCpy(key, argv[loopCnti], pValue - argv[loopCnti]);
            pValue++;
        } else {                            ///< 不包含等号
            noEqualSign = true;
            (void)cliMemCpy(key, argv[loopCnti], (S32)strlen(argv[loopCnti]));
            pValue = argv[loopCnti];
        }

        for (loopCntj = 0; loopCntj < paramCount; loopCntj++) {               ///< 遍历该命令可接收的参数列表
            cliMemSet(tmpKey, 0, (S32)sizeof(tmpKey));
            cliMemCpy(tmpKey, pParamList[loopCntj].key, (S32)strlen(pParamList[loopCntj].key));
            pTokStr = cliStrtok(tmpKey, "|", &sp);
            while (pTokStr != NULL) {
                if (cliStrCaseCmp(key, pTokStr) == 0) {
                    findFlag = true;
                    pParaMatched = &pParamList[loopCntj];

                    if (pParamList[loopCntj].noEqualSign != noEqualSign) {
                        CLI_LOG_ERROR("Parse Param %s Failed: Wrong Equal Sign.", key);
                        ret = -CLI_ERRNO_ARGUMENT_INVALID;
                        goto lEnd;
                    }
                    if (pParamList[loopCntj].value == NULL) {    ///< 从未解析到该参数
                        pParamList[loopCntj].value = pValue;
                    } else {        ///< 参数重复
                        CLI_LOG_ERROR("Parse Param %s Failed: Repeated Param.", key);
                        ret = -CLI_ERRNO_ARGUMENT_INVALID;
                        goto lEnd;
                    }

                    break;
                }
                pTokStr = cliStrtok(NULL, "|", &sp);
            }
        }

        if (findFlag == false) {
            CLI_LOG_ERROR("Parse Param Failed: Param %s Invalid.", key);
            ret = -CLI_ERRNO_ARGUMENT_INVALID;
            goto lEnd;
        } else {    ///< 解析参数成功,判断此参数是否存在跟随的参数
            handleArgsNum = 0;  ///< 处理的参数个数
            if (pParaMatched == NULL) {
                CLI_LOG_ERROR("Parse Param %s Failed: Never Get Here.", key);
                ret = -CLI_ERRNO_ARGUMENT_INVALID;
                goto lEnd;
            }

            if ((pParaMatched != NULL) && (pParaMatched->followArgsNum != 0)) {
                argcRemain = argc - loopCnti - 1;               ///< 剩余未处理的参数个数
                followArgsNum = pParaMatched->followArgsNum;    ///< 跟随的参数个数
                argcTmp = (argcRemain < followArgsNum) ? argcRemain : followArgsNum;
                argvTmp = argv + loopCnti + 1;                  ///< 剩余未处理的参数
                followParamList = pParaMatched->followArgsList;

                handleArgsNum = cliParseFollowParam(argcTmp, argvTmp, followParamList, followArgsNum);

                loopCnti += handleArgsNum;
            }
        }
    }
    ret = CLI_ERRNO_OK;
lEnd:
    return ret;
}

/**
 * @brief      检查必选参数是否存在
 * @param      [in] pParam
 * @return     CliErrNo_e
 * @warning
 * @note       2021.04.10  xiezhj
 */
static S32 cliCheckParamNecessary(CliParam_s *pParam)
{
    S32 ret = CLI_ERRNO_INVALID;

    if (NULL == pParam) {
        CLI_LOG_ERROR("Input para is null.");
        goto lEnd;
    }

    if (pParam->isNecessary == true && ((pParam->value == NULL) || (strlen(pParam->value) == 0))) {
        ret = -CLI_ERRNO_ARGUMENT_INVALID;
        CLI_LOG_ERROR("Param [%s] is necessary but not found.", pParam->key);
        goto lEnd;
    }

    ret = CLI_ERRNO_OK;
lEnd:
    return ret;
}

/**
 * @brief      从param结构体中读取特定字段的值
 * @param      [in] paramList
 * @param      [in] paramCount
 * @param      [out] pKey
 * @return     CliParam_s
 * @warning
 * @note       2021.04.14  xiezhj
 */
CliParam_s *cliMatchParam(CliParam_s *pParamList, U32 paramCount, S8 *pKey)
{
    CliParam_s *pParam = NULL;
    U32        i      = 0;

    if ((NULL == pParamList) || (NULL == pKey)) {
        CLI_LOG_ERROR("Input para is null, pParamList=null?:%d, pKey=null?:%d.",
                      (NULL == pParamList), (NULL == pKey));
        goto lEnd;
    }

    for (i = 0; i < paramCount; i++) {
        if (cliStrCaseCmp(pKey, pParamList[i].key) == 0) {
            pParam = &pParamList[i];
            break;
        }
    }
lEnd:
    return pParam;
}

/**
 * @brief      检查参数是否依赖其他参数
 * @param      [in] pParam
 * @param      [in] pParamList
 * @param      [in] paramCount
 * @return     CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note       2021.04.10  xiezhj
 */
static S32 cliCheckParamDepend(CliParam_s *pParam, CliParam_s *pParamList, U32 paramCount)
{
    S32        ret                       = CLI_ERRNO_INVALID;
    CliParam_s *pParamDepend             = NULL;
    S8         *pTokStr                  = NULL;
    S8         *sp                       = NULL;
    S8         tmpDepend[CLI_KEY_LENGTH] = { 0 };

    ///< 入参检查
    if ((NULL == pParam) || (NULL == pParamList)) {
        ret = -CLI_ERRNO_PARAM_NULL;
        CLI_LOG_ERROR("Input para is null, pParamList=null?:%d, pParam=null?:%d, ret=%08x.",
                      (NULL == pParamList), (NULL == pParam), ret);
        goto lEnd;
    }

    ///< 检查该参数是否依赖其他参数
    if (pParam->depend != NULL) {
        cliMemSet(tmpDepend, 0, (S32)sizeof(tmpDepend));
        cliMemCpy(tmpDepend, pParam->depend, (S32)strlen(pParam->depend));
        pTokStr = cliStrtok(tmpDepend, "|", &sp);
        while (pTokStr != NULL) {
            pParamDepend = cliMatchParam(pParamList, paramCount, pTokStr);
            if (pParamDepend == NULL) {
                ret = -CLI_ERRNO_ARGUMENT_INVALID;
                CLI_LOG_ERROR("Param [%s] depend on [%s], not found.\n", pParam->key, pTokStr);
                goto lEnd;
            }
            if (pParamDepend->value == NULL) {
                ret = -CLI_ERRNO_ARGUMENT_INVALID;
                CLI_LOG_ERROR("Param [%s] depend on [%s], not found.\n", pParam->key, pTokStr);
                goto lEnd;
            }
            pTokStr = cliStrtok(NULL, "|", &sp);
        }
    }
    ret = CLI_ERRNO_OK;
lEnd:
    return ret;
}

/**
 * @brief      检查参数值是否被允许
 * @param      [in] pParam
 * @return     CliErrNo_e
 * @warning
 * @note       2021.04.12  xiezhj
 */
static S32 cliCheckParamAllow(CliParam_s *pParam)
{
    S32  ret                    = CLI_ERRNO_INVALID;
    Bool matchFlag              = false;
    S8   *pTokStr               = NULL;
    S8   *sp                    = NULL;
    S8   tmpAll[CLI_MAX_BUF_SIZE] = { 0 };

    ///< 入参检查
    if (NULL == pParam) {
        ret = -CLI_ERRNO_PARAM_NULL;
        CLI_LOG_ERROR("Input para pParam is null, ret=%08x.", ret);
        goto lEnd;
    }

    ///< 检查该参数值是否被允许
    if (pParam->allow != NULL) {
        cliMemCpy(tmpAll, pParam->allow, (S32)strlen(pParam->allow));
        if (cliStrCaseCmp(tmpAll, "") != 0) { ///< allow为""表示不限制
            pTokStr = cliStrtok(tmpAll, "|", &sp);
            while (pTokStr!=NULL) {
                if (cliStrCaseCmp(pTokStr, pParam->value) == 0) {
                    matchFlag = true;
                    break;
                }
                pTokStr = cliStrtok(NULL, "|", &sp);
            }
            if (matchFlag != true) {
                ret = -CLI_ERRNO_ARGUMENT_INVALID;
                CLI_LOG_ERROR("Uknown param: %s.\n", pParam->value);
                goto lEnd;
            }
        }
    }
    ret = CLI_ERRNO_OK;
lEnd:
    return ret;
}

/**
 * @brief      检查follow param
 * @param      [in] pParamList
 * @param      [in] paramCount
 * @return     CliErrNo_e
 * @warning
 * @note       2021.04.12  xiezhj
 */
static S32 cliCheckFollowParam(CliParam_s *pParamList, U32 paramCount, Node_s *pParent)
{
    S32 ret = CLI_ERRNO_INVALID;
    U32        idx = 0;

    ///< 入参检查
    if (NULL == pParamList) {
        ret = -CLI_ERRNO_PARAM_NULL;
        CLI_LOG_ERROR("Input para pParamList is null, ret=%08x.", ret);
        goto lEnd;
    }

    for (idx = 0; idx < paramCount; idx++) {
        if (pParamList[idx].value == NULL) {
            if (pParamList[idx].isNecessary == true) {
                CLI_LOG_ERROR("Param %s is necessary.", pParamList[idx].key);
                ret = -CLI_ERRNO_ARGUMENT_INVALID;
                goto lEnd;
            }
            continue;
        }

        ///< 检查该参数是否依赖其他参数
        ret = cliCheckParamDepend(&pParamList[idx], pParamList, paramCount);
        if (ret != CLI_ERRNO_OK) {
            cliNodeAddStringVal(pParent, "Param %s is depend on param %s.",
                pParamList[idx].key, pParamList[idx].depend);
            cliNodeAddStringVal(pParent, "You can enter -h or help to get cmd list.");
            goto lEnd;
        }

        ///< 检查必选参数是否都存在
        ret = cliCheckParamNecessary(&pParamList[idx]);
        if (ret != CLI_ERRNO_OK) {
            cliNodeAddStringVal(pParent, "Please enter correct param.");
            cliNodeAddStringVal(pParent, "You can enter -h or help to get cmd list.");
            goto lEnd;
        }

        ///< 检查该参数值是否被允许
        ret = cliCheckParamAllow(&pParamList[idx]);
        if (ret != CLI_ERRNO_OK) {
            cliNodeAddStringVal(pParent, "Please enter param: %s.", pParamList[idx].key);
            cliNodeAddStringVal(pParent, "You can enter -h or help to get cmd list.");
            goto lEnd;
        }
    }
    ret = CLI_ERRNO_OK;
lEnd:
    return ret;
}

/**
 * @brief      校验输入参数
 * @param      [in]
 * @param      [out]
 * @return     CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note       2021.04.10  xiezhj
 */
S32 cliCheckParam(CliParam_s *pParamList, U32 paramCount, Node_s *pParent)
{
    S32 ret = CLI_ERRNO_INVALID;
    U32        i   = 0;

    ///< 入参检查
    if (NULL == pParamList) {
        ret = -CLI_ERRNO_PARAM_NULL;
        CLI_LOG_ERROR("Input para pParamList is null, ret=%08x.", ret);
        goto lEnd;
    }

    for (i = 0; i < paramCount; i++) {
        if (pParamList[i].value == NULL && pParamList[i].isNecessary != true) {
            continue;
        }

        ///< 检查必选参数是否都存在
        ret = cliCheckParamNecessary(&pParamList[i]);
        if (ret != CLI_ERRNO_OK) {
            cliNodeAddStringVal(pParent, "Please enter correct param.");
            cliNodeAddStringVal(pParent, "You can enter -h or help to get cmd list.");
            goto lEnd;
        }

        ///< 检查该参数是否依赖其他参数
        ret = cliCheckParamDepend(&pParamList[i], pParamList, paramCount);
        if (ret != CLI_ERRNO_OK) {
            cliNodeAddStringVal(pParent, "Param %s is depend on param %s.",
                pParamList[i].key, pParamList[i].depend);
            cliNodeAddStringVal(pParent, "You can enter -h or help to get cmd list.");
            goto lEnd;
        }

        ///< 检查该参数值是否被允许
        ret = cliCheckParamAllow(&pParamList[i]);
        if (ret != CLI_ERRNO_OK) {
            cliNodeAddStringVal(pParent, "Please enter correct param value, %s: %s.",
                pParamList[i].key, pParamList[i].allow);
            goto lEnd;
        }

        ///< 参数检查
        if (pParamList[i].paramCheck != NULL) {
            ret = pParamList[i].paramCheck(&pParamList[i], pParent);
            if (ret != CLI_ERRNO_OK) {
                cliNodeAddStringVal(pParent, "Please enter correct param %s.", pParamList[i].key);
                goto lEnd;
            }
        }
        ///< 检查fllow参数是否被允许
        if (pParamList[i].followArgsNum != 0) {
            ret = cliCheckFollowParam(pParamList[i].followArgsList, pParamList[i].followArgsNum, pParent);
            if (ret != CLI_ERRNO_OK) {
                CLI_LOG_ERROR("Check follow param failed.\n");
                goto lEnd;
            }
        }

    }
    ret = CLI_ERRNO_OK;

lEnd:
    return ret;
}
