#include "common_defines.h"
#include "shell_common.h"
#include "cli_core.h"
#include "cli_std.h"
#include "log_msg.h"
#include "cli_common.h"

#define MGL_CTRL_REG_NR (MglCtrlRegD + 1)

#define MAX_BIT8  (0xFFULL)
#define MAX_BIT16 (0xFFFFULL)
#define MAX_BIT32 (0xFFFFFFFFULL)
#define MAX_BIT64 (0xFFFFFFFFFFFFFFFFULL)
/**
 * 地址是否字节对齐
 * addr为需要校验的地址, bit为以多少位对齐, 1字节-->bit=8  2字节-->bit=16 4字节-->bit=32 8字节-->bit=64
 */
#define ADDR_IS_ALIGNED(addr, bit) ((addr) & (((bit) >> 3) - 1) ? false:true)
#define FW_BITS_MSK(n)             ((1ULL << (n)) - 1ULL)
#define FW_BITS_MSK2(m, n)         (FW_BITS_MSK((m) - (n) + 1ULL) << (n))
#define FW_BITS_SET(var, mask)     do {var |=  (mask);} while (0)
#define FW_BITS_CLR(var, mask)     do {var &= ~(mask);} while (0)
#define FW_BITS_REV(var, mask)     do {var ^=  (mask);} while (0)

#define FW_BITF_GET(data, pos, bw)      (((data) >> (pos)) & FW_BITS_MSK(bw))
#define FW_BITF_CLR(data, pos, bw)      do {data &= ~(FW_BITS_MSK(bw) << (pos));} while (0)
#define FW_BITF_SET(data, pos, bw, val) do {FW_BITF_CLR(data,pos,bw); \
                                           data |= ((val) & FW_BITS_MSK(bw)) << (pos); \
                                        } while (0)

///< 求4byte对齐地址
#define DOWN_ALIGN(x, byte) ((x) & (~((byte) - 1)))
#define DOWN_ALIGN_OFFSET(x, byte) ((x) & ((byte) - 1))
#define DOWN_ALIGN_BIT_OFFSET(x, byte) (DOWN_ALIGN_OFFSET(x, byte) << 3)
#define DWORD_WIDTH (4)
///< 下一个32位寄存器
#define NEXT_32BIT_REG(addr) ((addr) + 4)

enum {
    BIT_WIDTH_8  = 8,
    BIT_WIDTH_16 = 16,
    BIT_WIDTH_32 = 32,
    BIT_WIDTH_64 = 64,
};

U32 gSessionId = 0;

/**
 * @brief   获取sessionId
 * @param  void
 * @return  sessionId
 */
U32 swCliGetSessionId(void)
{
    return gSessionId;
}

/**
 * @brief        以32位读寄存器
 * @param:       addr       [in]   地址
 * @param:       pRegValue  [out]  寄存器的值
 * @return       errno
 * @note         为方便打桩再封一次
 */
U32 halRegRead32(ULong addr)
{
    return (*((volatile U32 *)(ULong)(addr)));
}

/**
 * @brief        以32位读寄存器
 * @param:       addr    [in]  地址
 * @param:       val8    [in]  寄存器值
 * @return       errno
 * @note         为方便打桩再封一次
 */
void halRegWrite32(ULong addr, U32 val32)
{
    (*((volatile U32 *)(ULong)(addr)) = (val32));
}

/**
 * @brief        写32位写寄存器(指定类型和offset的方式)
 * @param:       type    [in]  寄存器类型
 * @param:       addr    [in]  地址
 * @param:       val32   [in]  寄存器值
 * @return       void
 * @note         王殿卫 新生成的函数
 */
static void cliRegWrite64ByType(MglCtrlRegType_e type, ULong offset, U64 val64)
{
    ULong addr = 0;

    ///< 计算实际地址
    //addr = halRegBaseGet(type) + offset;
    addr = offset;

    halRegWrite32(addr, (U32)(val64 & MAX_BIT32));
    ///< 写下一个U32
    halRegWrite32(NEXT_32BIT_REG(addr), (U32)((val64 >> BIT_WIDTH_32) & MAX_BIT32));

    CLI_LOG_DEBUG("write reg type:%d, offset:%#lx, addr:%#lx, width:64, val:%#llx", \
            (U32)type, offset, addr, val64);
    return;
}

/**
 * @brief        写16位写寄存器(指定类型和offset的方式)
 * @param:       type    [in]  寄存器类型
 * @param:       addr    [in]  地址
 * @param:       val16   [in]  寄存器值
 * @return       void
 * @note         王殿卫 新生成的函数
 */
static void cliRegWrite16ByType(MglCtrlRegType_e type, ULong offset, U16 val16)
{
    ULong addr = 0;
    U32 regValue = 0;
    ULong alignOffset = 0;
    ULong alignAddr = 0;

    ///< 计算实际地址
    //addr = halRegBaseGet(type) + offset;
    addr = offset;

    alignAddr = DOWN_ALIGN(addr, DWORD_WIDTH);
    alignOffset = DOWN_ALIGN_BIT_OFFSET(addr, DWORD_WIDTH);

    ///< 读出寄存器值
    regValue = halRegRead32(alignAddr);

    ///< 更改寄存器值
    regValue &= ~(MAX_BIT16 << alignOffset);
    regValue |= (val16 << alignOffset);

    ///< 写入寄存器值
    halRegWrite32(alignAddr, regValue);

    CLI_LOG_DEBUG("write reg type:%d, offset:%#lx, addr:%#lx, width:16, val:%#04x", \
            (U32)type, offset, addr, val16);

    return;
}

/**
 * @brief        写8位写寄存器(指定类型和offset的方式)
 * @param:       type    [in]  寄存器类型
 * @param:       addr    [in]  地址
 * @param:       val8    [in]  寄存器值
 * @return       void
 * @note         王殿卫 新生成的函数
 */
static void cliRegWrite8ByType(MglCtrlRegType_e type, ULong offset, U8 val8)
{
    ULong addr = 0;
    U32 regValue = 0;
    ULong alignOffset = 0;
    ULong alignAddr = 0;

    ///< 计算实际地址
    //addr = halRegBaseGet(type) + offset;
    addr = offset;

    alignAddr = DOWN_ALIGN(addr, DWORD_WIDTH);
    alignOffset = DOWN_ALIGN_BIT_OFFSET(addr, DWORD_WIDTH);

    ///< 读出寄存器值

    regValue = halRegRead32(alignAddr);

    ///< 更改寄存器值
    regValue &= ~(MAX_BIT8 << alignOffset);
    regValue |= (val8 << alignOffset);

    ///< 写入寄存器值
    halRegWrite32(alignAddr, regValue);

    CLI_LOG_DEBUG("write reg type:%d, offset:%#lx, addr:%#lx, width:8, val:%#02x", \
            (U32)type, offset, addr, val8);
    return;
}

/**
 * @brief  将输入的寄存器名称(字符串) 转换为枚举
 * @param  pParent [in] 打印父节点
 * @param  pParam  [in] 参数
 * @return errno
 */
static S32 swCliRegTypeGet(S8 *pTypeStr, MglCtrlRegType_e *pType)
{
    U32 idx;
    S32 rc = -CLI_ERRNO_FAILED;
    const S8 *typeList[] = {
        [MglCtrlRegA] = "A",
        [MglCtrlRegB] = "B",
        [MglCtrlRegC] = "C",
        [MglCtrlRegD] = "D",
    };

    *pType = MglCtrlRegUnknown;
    for (idx = 0; idx < ARRAY_SIZE(typeList); idx++) {
        if (NULL == typeList[idx]) {
            continue;
        }
        if (cliStrCaseCmp(pTypeStr, typeList[idx]) == CLI_ERRNO_OK) {
            *pType = (MglCtrlRegType_e)idx;
            rc = CLI_ERRNO_OK;
            goto lEnd;
        }
    }

    rc = -(S32)1;

lEnd:
    return rc;
}

/**
 * @brief        字符串转U64,判断是否超限
 * @param        [in] pNptr  需要转换字符串
 * @param        [in] base   以多少进制转换, 如16以16进制转换, 0表示自适应
 * @param        [in] pData  转换的结果
 * @return       转换结果
 * @warning
 * @note         2022.01.06
 */
S32 swCliStrToU64Edge(const S8 *pNptr, S32 base, U64 *pData)
{
    S8 *pStrEnd     = NULL;
    U64 tempData;
    S32 rc;

    errno = 0;
    tempData = strtoull(pNptr, &pStrEnd, base);
    if (((errno == ERANGE) && (tempData == U64_MAX)) || ((errno != 0) && (tempData == 0))) {
        rc = -(S32)1;
        CLI_LOG_ERROR("digit:%s over %llu, err:%d", pNptr, U64_MAX, rc);
        goto lEnd;
    }

    if (*pStrEnd != '\0') {
        rc = -(S32)1;
        CLI_LOG_ERROR("str \"%s\" not digit, err:%d", pNptr, rc);
        goto lEnd;
    }

    *pData = tempData;
    rc = CLI_ERRNO_OK;
lEnd:
    return rc;
}

/**
 * @brief        写32位写寄存器(指定类型和offset的方式)
 * @param:       type    [in]  寄存器类型
 * @param:       addr    [in]  地址
 * @param:       val32   [in]  寄存器值
 * @return       void
 * @note         王殿卫 新生成的函数
 */
static void cliRegWrite32ByType(MglCtrlRegType_e type, ULong offset, U32 val32)
{
    ULong addr = 0;

    ///< 计算实际地址
    ///< addr = halRegBaseGet(type) + offset;
    addr = 0 + offset;

    halRegWrite32(addr, val32);
    CLI_LOG_DEBUG("write reg type:%d, offset:%#lx, addr:%#lx, width:32, val:%#x", \
            (U32)type, offset, addr, val32);
    return;
}

/**
 * @brief        以32位读寄存器(指定类型和offset的方式)
 * @param:       type       [in]  寄存器类型
 * @param:       addr       [in]   地址
 * @return       寄存器的值
 * @note         王殿卫 新生成的函数
 */
static U32 cliRegRead32ByType(MglCtrlRegType_e type, ULong offset)
{
    ULong addr = 0;
    U32 val = 0;

    ///< 计算实际地址并读取
    addr = offset;
    val = halRegRead32(addr);

    CLI_LOG_DEBUG("read reg type:%d, offset:%#lx, addr:%#lx, width:32, value:%#x", \
            (U32)type, offset, addr, val);

    return val;

}


/**
 * @brief        写寄存器Msk命令参数检查
 * @param:       pRegMsk [in]  寄存器mask 设置模式信息
 * @return       成功返回0；失败小于0
 * @note
 */
static S32 cliRegMskInfoCheck(MglCtrlRegMsk_t *pRegMsk)
{
    S32 ret;

    ///< 地址必须是4字节对齐输入
    if (true != ADDR_IS_ALIGNED(pRegMsk->addr, BIT_WIDTH_32)) {
        ret = -(S32)1;
        CLI_LOG_ERROR("reg_addr:%#x invalid, err:%d", pRegMsk->addr, ret);
        goto end;
    }
    /**
     * BITS pos: bit位置从0开始编号[0,31]
     * BITS  bw: 位宽从1开始编号, 位宽需要小于等于pos起始位留下的位宽
     * 31            16 15            7               4 3             0
     * +---------------+---------------+---------------+---------------+
     * |                                                               |
     * +---------------+---------------+---------------+---------------+
     *                       ^
     *                       |_pos
     *              ^        ^
     *              |        |
     *              |__ bw __|
     *              |_value _|
     */

    if (pRegMsk->pos > 31 || pRegMsk->bw > 32 || pRegMsk->bw == 0 ||
        pRegMsk->bw > (31 - pRegMsk->pos + 1) || (~0ULL >> pRegMsk->pos) < pRegMsk->value) {
        ret = -(S32)1;
        CLI_LOG_ERROR("pos:%d bw:%d value:%#x invalid, err:%d",
            pRegMsk->pos, pRegMsk->bw, pRegMsk->value, ret);
        goto end;
    }

    ret = CLI_ERRNO_OK;

end:
    return ret;
}

/**
 * @brief        写寄存器Msk命令回调
 * @param:       pReqBody      [in]      主机下发的req的消息体
 * @param:       reqBodyLen    [in]      主机下发的req的消息体长度
 * @param:       pRespBody     [in]      回复给主机的消息体
 * @param:       pRespBodyLen  [in/out]  回复给主机的消息体长度,传入的是期望长度,传出实际长度
 * @return       函数执行状态,成功返回0；失败返回错误码
 * @note
 */
S32 cliRegMskWrite(void *pReqBody, U32 reqBodyLen,  void *pRespBody, U32 *pRespBodyLen)
{
    MglCtrlRegMsk_t *pRegMsk = NULL;
    U32 regV;
    S32 ret = CLI_ERRNO_FAILED;

    ///< 入参检查
    if (!pReqBody || !pRespBodyLen) {
        goto end;
    }
    pRegMsk = (MglCtrlRegMsk_t *)pReqBody;

    ret = cliRegMskInfoCheck(pRegMsk);
    if (CLI_ERRNO_OK > ret) {
        CLI_LOG_ERROR("cliRegMskInfoCheck failed, err:%d", ret);
        goto end;
    }

    ///< TODO: 寄存器堵路类型默认为TPYE_A,给定寄存器范围后删除
    regV = cliRegRead32ByType(MglCtrlRegA, pRegMsk->addr);

    FW_BITF_SET(regV, pRegMsk->pos,  pRegMsk->bw, pRegMsk->value);

    cliRegWrite32ByType(MglCtrlRegA, pRegMsk->addr, regV);

    ret = CLI_ERRNO_OK;

end:
    *pRespBodyLen = 0;
    return ret;
}

/**
 * @brief  解析寄存器mask配置cli命令输入的字符串
 * @param  pRegInfo [in] regMask设置信息
 * @param  pParam   [in] 参数
 * @return errno
 * @note: /comm set regMsk addr=xxx pos=xxx bw=xxx value=xxx
 */
S32 swCliRegMskParamParse(MglCtrlRegMsk_t *pRegInfo, swCliRegMaskStr_s *pCliRegStr, CliParam_s *pParam)
{
    S8 *pStrEnd     = NULL;
    S32 ret;

    ///< 解析addr
    pCliRegStr->pRegMaskAddrStr = swCliValueGetByKey("addr", pParam->followArgsList, pParam->followArgsNum);
    pCliRegStr->pRegMaskBwStr = swCliValueGetByKey("bw", pParam->followArgsList, pParam->followArgsNum);
    pCliRegStr->pRegMaskPosStr = swCliValueGetByKey("pos", pParam->followArgsList, pParam->followArgsNum);
    pCliRegStr->pRegMaskValueStr = swCliValueGetByKey("value", pParam->followArgsList, pParam->followArgsNum);
    if (NULL == pCliRegStr->pRegMaskAddrStr || NULL == pCliRegStr->pRegMaskBwStr || \
            NULL == pCliRegStr->pRegMaskPosStr || NULL == pCliRegStr->pRegMaskValueStr) {
        ret = -(S32)1;
        CLI_LOG_ERROR("param null, err:%d", ret);
        goto lEnd;
    }

    pRegInfo->addr = cliStrToU32(pCliRegStr->pRegMaskAddrStr, &pStrEnd, 0);
    if ('\0' != *pStrEnd) {
        ret = -(S32)1;
        CLI_LOG_ERROR("addr \"%s\" invalid err:%d", pCliRegStr->pRegMaskAddrStr, ret);
        goto lEnd;
    }

    pRegInfo->pos = cliStrToU32(pCliRegStr->pRegMaskPosStr, &pStrEnd, 0);
    if ('\0' != *pStrEnd) {
        ret = -(S32)1;
        CLI_LOG_ERROR("pos \"%s\" invalid err:%d", pCliRegStr->pRegMaskPosStr, ret);
        goto lEnd;
    }

    ///< 解析bw 位宽
    pRegInfo->bw = cliStrToU32(pCliRegStr->pRegMaskBwStr, &pStrEnd, 0);
    if ('\0' != *pStrEnd) {
        ret = -(S32)1;
        CLI_LOG_ERROR("bw \"%s\" invalid err:%d", pCliRegStr->pRegMaskBwStr, ret);
        goto lEnd;
    }

    ///< 解析value
    pRegInfo->value = cliStrToU32(pCliRegStr->pRegMaskValueStr, &pStrEnd, 0);
    if ('\0' != *pStrEnd) {
        ret = -(S32)1;
        CLI_LOG_ERROR("value \"%s\" invalid err:%d", pCliRegStr->pRegMaskValueStr, ret);
        goto lEnd;
    }

    ret = CLI_ERRNO_OK;

lEnd:
    return ret;
}

/* @brief        检查寄存器数据是否合法
 * @param:       type   [in]  寄存器类型
 * @return       函数执行状态,合法返回0；不合法返回错误码
 * @note         王殿卫 新生成的函数
 */
static S32 cliRegInfoCheck(MglCtrlRegInfo_t *pRegInfo, Bool isRead)
{
    S32 rc = -CLI_ERRNO_FAILED;

    ///< 判断寄存器类型
    if ((MglCtrlRegUnknown == (U32)pRegInfo->type) || (MGL_CTRL_REG_NR == (U32)pRegInfo->type)) {
        rc = -(S32)1;
        CLI_LOG_ERROR("regInfo type invalid, type:%d, rc:%d", (U32)pRegInfo->type, rc);
        goto lEnd;
    }

    ///< 判断要写入的寄存器数值是否超限
    switch (pRegInfo->width) {
    case BIT_WIDTH_8:
        if (pRegInfo->value > MAX_BIT8) {
            rc = -(S32)1;
            CLI_LOG_ERROR("value:%llu over U8_MAX, err:%d", pRegInfo->value, rc);
            goto lEnd;
        }
        break;
    case BIT_WIDTH_16:
        if (pRegInfo->value > MAX_BIT16) {
            rc = -(S32)1;
            CLI_LOG_ERROR("value:%llu over U16_MAX, err:%d", pRegInfo->value, rc);
            goto lEnd;
        }
        break;
    case BIT_WIDTH_32:
        if (pRegInfo->value > MAX_BIT32) {
            rc = -(S32)1;
            CLI_LOG_ERROR("value:%llu over U32_MAX, err:%d", pRegInfo->value, rc);
            goto lEnd;
        }
        break;
    case BIT_WIDTH_64:
        ///< 64位时 传输过来的数据一定小于64位所容纳的最大的数据,此时有片外防守
        break;
    default:
        rc = -(S32)1;
        CLI_LOG_ERROR("reg width %d invalid, rc:%d", pRegInfo->width, rc);
        goto lEnd;
    }

    ///< 判断地址是否对齐
    if (false == ADDR_IS_ALIGNED(pRegInfo->addr, pRegInfo->width)) {
        rc = -(S32)1;
        CLI_LOG_ERROR("not aligned, addr:%#llx, width:%d, err:%d",
                pRegInfo->addr, pRegInfo->width, rc);
        goto lEnd;
    }
    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief        写寄存器命令回调
 * @param:       pReqBody      [in]      主机下发的req的消息体
 * @param:       reqBodyLen    [in]      主机下发的req的消息体长度
 * @param:       pRespBody     [in]      回复给主机的消息体
 * @param:       pRespBodyLen  [in/out]  回复给主机的消息体长度,传入的是期望长度,传出实际长度
 * @return       函数执行状态,成功返回0；失败返回错误码
 * @note
 */
S32 cliRegWrite(void *pReqBody, U32 reqBodyLen,  void *pRespBody, U32 *pRespBodyLen)
{
    MglCtrlRegInfo_t *pRegInfo = NULL;
    S32 rc = -CLI_ERRNO_FAILED;

    ///< 入参检查
    if (!pReqBody) {
        goto lEnd;
    }
    pRegInfo = (MglCtrlRegInfo_t *)pReqBody;

    rc = cliRegInfoCheck(pRegInfo, false);
    if (rc != CLI_ERRNO_OK) {
        ///< 具体报错信息会在调用的函数
        CLI_LOG_ERROR("reg info invalid, rc = %d", rc);
        goto lEnd;
    }

    ///< 写寄存器
    switch (pRegInfo->width) {
    case BIT_WIDTH_8:
        cliRegWrite8ByType(pRegInfo->type, pRegInfo->addr, (U8)(pRegInfo->value));
        break;
    case BIT_WIDTH_16:
        cliRegWrite16ByType(pRegInfo->type, pRegInfo->addr, (U16)(pRegInfo->value));
        break;
    case BIT_WIDTH_32:
        cliRegWrite32ByType(pRegInfo->type, pRegInfo->addr, (U32)(pRegInfo->value));
        break;
    case BIT_WIDTH_64:
        cliRegWrite64ByType(pRegInfo->type, pRegInfo->addr, pRegInfo->value);
        break;
    default:
        rc = -(S32)1;
        CLI_LOG_ERROR("reg width %u invalid", pRegInfo->width);
        goto lEnd;
    }

    CLI_LOG_DEBUG("reqest reg info: addr = %#llx, type = %d, width = %d, value = %#llx", \
            pRegInfo->addr, (U32)pRegInfo->type, pRegInfo->width, pRegInfo->value);
    rc = CLI_ERRNO_OK;

lEnd:
    if (pRespBodyLen) {
        *pRespBodyLen = 0;
    }
    return rc;
}

/**
 * @brief   通过key和参数数组来获取key对应的value
 * @param   pKey        [in]    key字符串
 * @param   pParamList  [in]    参数数组
 * @param   paramCount  [in]    参数数组的长度
 * @return  value对应的字符串 NULL表示不支持
 * @warn    调用者需保证paramCount要等于param数组的元素数量!!!!!!!!!!
 * @warn    可用于cli框架已经将参数解析并赋值到对应参数全局变量中
 */
S8 *swCliValueGetByKey(S8 *pKey, CliParam_s *pParamList, U8 paramCount)
{
    S8 *pValue = NULL;
    U8 idx;

    for (idx = 0; idx < paramCount; idx++) {
        if (CLI_ERRNO_OK == cliStrNCaseCmp(pKey, pParamList[idx].key, cliStrLen(pParamList[idx].key))) {
            pValue = pParamList[idx].value;
            goto lEnd;
        }
    }

lEnd:
    return pValue;
}

/**
 * @brief  解析寄存器cli命令输入的字符串
 * @param  pRegInfo [out]   寄存器信息
 * @param  pRegStr  [out]   输入的寄存器命令的字符串
 * @param  pParam   [in]    命令参数结构体
 * @param  isSet    [in]    是否是写(若是写会多解析value值)
 * @return errno
 */
S32 swCliRegParamParse(MglCtrlRegInfo_t *pRegInfo, swCliRegStr_s *pRegStr,
        CliParam_s *pParam, Bool isSet)
{
    S8 *pStrEnd     = NULL;
    S32 rc          = -CLI_ERRNO_FAILED;

    ///< 获取命令输入的各个字符串
    pRegStr->pRegTypeStr = swCliValueGetByKey("type", pParam->followArgsList, pParam->followArgsNum);
    pRegStr->pRegWidthStr = swCliValueGetByKey("width", pParam->followArgsList, pParam->followArgsNum);
    pRegStr->pRegAddrStr = swCliValueGetByKey("addr", pParam->followArgsList, pParam->followArgsNum);
    if (true == isSet) {
        pRegStr->pRegValueStr = swCliValueGetByKey("value", pParam->followArgsList, pParam->followArgsNum);
        if (NULL == pRegStr->pRegValueStr) {
            rc = -(S32)1;
            CLI_LOG_ERROR("pRegValueStr:%p, err:%d", pRegStr->pRegValueStr, rc);
            goto lEnd;
        }
    }

    ///< 校验必要参数是否全部指定
    if (NULL == pRegStr->pRegTypeStr || NULL == pRegStr->pRegWidthStr || NULL == pRegStr->pRegAddrStr) {
        rc = -(S32)1;
        CLI_LOG_ERROR("pRegTypeStr:%p pRegWidthStr:%p pRegAddrStr:%p err:%d",
            pRegStr->pRegTypeStr, pRegStr->pRegWidthStr, pRegStr->pRegAddrStr, rc);
        goto lEnd;
    }

    ///< 解析type
    rc = swCliRegTypeGet(pRegStr->pRegTypeStr, &(pRegInfo->type));
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("swCliRegTypeGet failed err:%d", rc);
        goto lEnd;
    }

    ///< 解析宽度
    pRegInfo->width = cliStrToU32(pRegStr->pRegWidthStr, &pStrEnd, 0);
    if ('\0' != *pStrEnd) {
        rc = -(S32)1;
        CLI_LOG_ERROR("width \"%s\" not digit, err:%d", pStrEnd, rc);
        goto lEnd;
    }
    ///< 片内cli框架 cliRegRead中会均校验width, 此处不做校验

    ///< 解析addr
    pRegInfo->addr = cliStrToU64(pRegStr->pRegAddrStr, &pStrEnd, 0);
    if ('\0' != *pStrEnd || pRegInfo->addr > MAX_BIT32) {
        rc = -(S32)1;
        CLI_LOG_ERROR("addr \"%s\" not digit err:%d", pRegStr->pRegAddrStr, rc);
        goto lEnd;
    }
    ///< cliRegRead/cliRegWrite均会校验字节对齐,此处不做校验

    ///< 解析输入的value值, 只有写寄存器需要解析, 读不需要
    if (true == isSet) {
        ///< 解析value
        ///< value值是否大于width的限制会在cliRegRead函数中校验,此处不做校验
        rc = swCliStrToU64Edge(pRegStr->pRegValueStr, 0, &(pRegInfo->value));
        if (rc != CLI_ERRNO_OK) {
            CLI_LOG_ERROR("swCliStrToU64Edge failed err:%d", rc);
            goto lEnd;
        }
    }
    ///< 无需else,若是读寄存器 不会输入value值, 所以无需解析

lEnd:
    return rc;
}

/**
 * @brief   检查私有数据
 * @param   pPriv [in] 需要检查的私有数据
 * @return  errno
 */
static S32 swCliCheckPrivData(CmdPrivData_s *pPriv)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if (NULL == pPriv->pCmd) {
        rc = -(S32)1;
        CLI_LOG_ERROR("Cmd is null, err:%d", rc);
        goto lEnd;
    }

    if (NULL == pPriv->pCmd->handleCmd) {
        CLI_LOG_DEBUG("cmd (name:%s) have no handle func, assign func " \
            "\"swCliHandler\"", pPriv->pCmd->name);
        pPriv->pCmd->handleCmd = swCliHandler;
    }

    if (NULL == pPriv->pCmd->pParam) {
        rc = -(S32)1;
        CLI_LOG_ERROR("cmd:%s no param, err:%d", pPriv->pCmd->name, rc);
        goto lEnd;
    }

    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}

/**
 * @brief  执行 Command 命令
 * @param  argument 参数信息
 * @param  cmd      命令信息
 * @return Ps3Errno
 */
static S32 swCliCompleteFunc(CmdPrivData_s *pPriv)
{
    ///< 返回给cli的返回值
    S32 rc = -CLI_ERRNO_FAILED;
    CliSessionObj_s *pSession = NULL;
    Command_s *pCmd = NULL;
    U32 idx = 0;
    CliParam_s *pParam = NULL;
    S32 status = -CLI_ERRNO_FAILED;

    ///<  入参检查
    if (NULL == pPriv) {
        rc = -(S32)1;
        CLI_LOG_ERROR("obPrivate null, err:%d", rc);
        goto lEnd;
    }
    if (NULL == pPriv->pParent) {
        rc = -(S32)1;
        CLI_LOG_ERROR("Private:%p, err:%d", pPriv, rc);
        goto lEnd;
    }

    gSessionId = pPriv->sessionId;
    ///< 获取session
    rc = cliGetSessionById(pPriv->sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("cliGetSessionById failed, err:%d", rc);
        goto lEnd;
    }

    rc = swCliCheckPrivData(pPriv);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("swCliCheckPrivData failed, err:%d", status);
        goto lEnd;
    }

    pCmd = pPriv->pCmd;
    ///< 找参数   cliParseParam 函数中已经解析参数  并将其处理后放在pParam->value中
    for (idx = 0; idx < pCmd->paramCnt; idx++) {
        pParam = &(pCmd->pParam[idx]);
        if (NULL != pParam->value) {
            CLI_LOG_DEBUG("cmd \"%s\" param found, first param = \"%s\"", pCmd->name, pParam->displayName);
            if (NULL == pParam->handleParam) {
                status = -(S32)1;
                CLI_LOG_ERROR("Cmd \"%s\" param \"%s\" no cb, err:%d", \
                        pCmd->name, pParam->displayName, status);
                goto lResp;
            }

            ///< 执行回调函数
            status = pParam->handleParam(pPriv->pParent, pParam);
            rc = CLI_ERRNO_OK;
            goto lResp;
        }
    }
    ///< 参数没有找到 设置返回值, 并且打印帮助信息
    status = -(S32)1;
    rc = CLI_ERRNO_OK;

lResp:
    if (status != CLI_ERRNO_OK) {
        cliPrintDetailHelp(pPriv->sessionId, pCmd);
    }
    cliCmdStatusPrint(pPriv, status);
    cliCmdRunFinish(pPriv, pSession);
lEnd:
    return rc;
}

/**
 * @brief   获取cli总结构体指针
 * @param   void
 * @return  cli总结构体指针
 * @note    imt cmd中的handleCmd一律为该函数, 第一个参数的handleParam回调函数才为真正的命令处理函数
 * @note    而真正的参数均在第一个参数的followArgsList中,所以第一个参数的handleParam一定不能为空
 * @note    运行在cli cmd处理线程
 */
S32 swCliHandler(CmdPrivData_s *pPriv)
{
    S32 rc = -CLI_ERRNO_FAILED;

    if (NULL == pPriv) {
        rc = -(S32)1;
        CLI_LOG_ERROR("no priv data, err:%d", rc);
        goto lEnd;
    }
    rc = swCliCompleteFunc(pPriv);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("create imt job failed, err:%d", rc);
        goto lEnd;
    }

    rc = CLI_ERRNO_OK;

lEnd:
    return rc;
}