/**
 * copyright (C), 2021, Start Micro System Technologies Co. Ltd.
 *
 * @file        shell_command.c
 * @author      taohb
 * @date        2025/04/08
 * @brief       rtems shell command
 * @note        NULL
 */

#include "cli_core_api.h"
#include "shell_common.h"

#define REG_INFO_TITLE "Reg Type", "Addr", "width", "Value", "ErrCode", "ErrMsg", NULL

/**
 * @brief  寄存器写cli命令回调
 * @param  pParent [in] 打印父节点
 * @param  pParam  [in] 参数
 * @return errno
 */
static S32 swCliRegSet(Node_s *pParent, CliParam_s *pParam)
{
    S32 rc              = -CLI_ERRNO_FAILED;
    MglCtrlRegInfo_t regInfo;
    U32 valueLen        = sizeof(regInfo.value);
    Node_s *pNode       = NULL;
    TableRow_s *pRow    = NULL;
    S8  *pRegTitle[] = {REG_INFO_TITLE};
    swCliRegStr_s regStr     = { 0 };

    ///< 入参检查
    if (NULL == pParent || NULL == pParam) {
        rc = -(S32)1;
        CLI_LOG_ERROR("null ptr, err:%d", rc);
        goto lEnd;
    }

    pNode = cliNodeCreatTable(pParent, "register set", pRegTitle, NULL, 0);
    if (NULL == pNode) {
        rc = -(S32)1;
        CLI_LOG_ERROR("NO MEM, err:%d", rc);
        goto lEnd;
    }

    pRow  = cliNodeCreatTableRow(pNode);
    if (NULL == pRow) {
        rc = -(S32)1;
        CLI_LOG_ERROR("NO MEM, err:%d", rc);
        goto lEnd;
    }

    memset(&regInfo, 0, sizeof(regInfo));
    rc = swCliRegParamParse(&regInfo, &regStr, pParam, true);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("swCliRegParamParse failed err:%d", rc);
        goto lResp;
    }

    rc = cliRegWrite(&regInfo, sizeof(regInfo), NULL, &valueLen);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("cliRegRead failed err:%d", rc);
    }

lResp:
    cliTableRowAddStringF(pRow, "%s", regStr.pRegTypeStr);
    cliTableRowAddStringF(pRow, "%s", regStr.pRegAddrStr);
    cliTableRowAddStringF(pRow, "%s", regStr.pRegWidthStr);
    cliTableRowAddStringF(pRow, "%s", regStr.pRegValueStr);
    cliTableRowAddStringF(pRow, "%d", rc);
    cliTableRowAddStringF(pRow, "To Be Done");
lEnd:
    return rc;
}

/**
 * @brief  寄存器msk写cli命令回调
 * @param  pParent [in] 打印父节点
 * @param  pParam  [in] 参数
 * @return errno
 */
static S32 swCliRegMskSet(Node_s *pParent, CliParam_s *pParam)
{
    MglCtrlRegMsk_t regInfo;
    S32 ret;
    Node_s *pNode       = NULL;
    TableRow_s *pRow    = NULL;
    U32 valueLen        = sizeof(regInfo.value);
    swCliRegMaskStr_s cliRegStr = { 0 };
    S8 *pTitle[]  = {"addr" , "pos", "bw", "value", "ErrCd", "ErrMsg", NULL};

    if (NULL == pParent || NULL == pParam) {
        ret = -(S32)1;
        CLI_LOG_ERROR("pParent:%p pParam:%p err:%d", pParent, pParam,ret);
        goto lEnd;
    }

    pNode = cliNodeCreatTable(pParent, "regMsk set", pTitle, NULL, 0);
    if (NULL == pNode) {
        ret = -(S32)1;
        CLI_LOG_ERROR("cliNodeCreatTable failed, err:%d", ret);
        goto lEnd;
    }

    pRow = cliNodeCreatTableRow(pNode);
    if (NULL == pRow) {
        ret = -(S32)1;
        CLI_LOG_ERROR("cliNodeCreatTableRow failed, err:%d", ret);
        goto lEnd;
    }

    ret = swCliRegMskParamParse(&regInfo, &cliRegStr, pParam);
    if (CLI_ERRNO_OK > ret) {
        CLI_LOG_ERROR("swCliRegMskParamParse failed, err:%d", ret);
        goto lResp;
    }

    ret = cliRegMskWrite(&regInfo, sizeof(regInfo), NULL, &valueLen);
    if (CLI_ERRNO_OK > ret) {
        CLI_LOG_ERROR("cliRegMskWrite failed, err:%d", ret);
    }

lResp:
    cliTableRowAddStringF(pRow, "%s", cliRegStr.pRegMaskAddrStr);
    cliTableRowAddStringF(pRow, "%s", cliRegStr.pRegMaskPosStr);
    cliTableRowAddStringF(pRow, "%s", cliRegStr.pRegMaskBwStr);
    cliTableRowAddStringF(pRow, "%s", cliRegStr.pRegMaskValueStr);
    cliTableRowAddStringF(pRow, "%d", ret);
    cliTableRowAddStringF(pRow, "To Be Done");

lEnd:
    return ret;
}

static CliParam_s sgSwRegSetParaLst[] = {
    {
        .key            = "type",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "reg_type",
        .isNecessary    = true,
        .allow          = "A|B|C|D",
    },
    {
        .key            = "addr",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "reg_addr",
        .isNecessary    = true,
    },
    {
        .key            = "width",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "reg_addr",
        .isNecessary    = true,
        .allow          = "8|16|32|64"
    },
    {
        .key            = "value",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "reg_val",
        .isNecessary    = true,
    },
};

///< 写寄存器msk 后续参数
static CliParam_s sgSwRegMskSetParaLst[] = {
    {
        .key            = "addr",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "reg_addr",
        .isNecessary    = true,
    },
    {
        .key            = "pos",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "bit_pos",
        .isNecessary    = true,
    },
    {
        .key            = "bw",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "bit_witch",
        .isNecessary    = true,
    },
    {
        .key            = "value",
        .value          = NULL,
        .noEqualSign    = false,
        .displayName    = "bit_msk_val",
        .isNecessary    = true,
    },
};

///< comm set参数
static CliParam_s sgSwCommSetParaLst[] = {
    {
        .key            = "reg",
        .value          = NULL,
        .noEqualSign    = true,
        .displayName    = "reg_write",
        .handleParam    = swCliRegSet,
        .followArgsList = sgSwRegSetParaLst,
        .followArgsNum  = ARRAY_SIZE(sgSwRegSetParaLst),
    },
    {
        .key            = "regMsk",
        .value          = NULL,
        .noEqualSign    = true,
        .displayName    = "regMsk_write",
        .handleParam    = swCliRegMskSet,
        .followArgsList = sgSwRegMskSetParaLst,
        .followArgsNum  = ARRAY_SIZE(sgSwRegMskSetParaLst),
    },
};

///< comm set命令
Command_s gSwCliCommSetCmd = {
    .argument       = "/comm set ...",
    .cmdLevel       = CLI_LEVEL_LOWEST,
    .isSync         = false,
    .handleCmd      = swCliHandler,
    .name           = "comm set",
    .syntax         = "/comm set ...",
    .description    = "\n"
                    "    /comm set reg type=A|B|C|D addr=xxx width=8|16|32|64 value=xxx \n"
                    "    /comm set regMsk addr=xxx pos=xxx bw=xxx value=xxx \n",
    .pParam         = sgSwCommSetParaLst,
    .paramCnt       = ARRAY_SIZE(sgSwCommSetParaLst),
    .argOffset      = ARG_OFFSET_2,
};

///< 保存所有cmd的数组  若新增cli命令需要修改此处
static Command_s *sgImtCmd[] = {
    &gSwCliCommSetCmd,
};

/**
 * @brief   imt 命令注册
 * @param   void
 * @return  errno
 * @note
 */
void swCmdRegister(Command_s **pImtCmd, U32 cmdNum)
{
    U32 idx = 0;

    for (idx = 0; idx < cmdNum; idx++) {
        cliRegisterCmd(pImtCmd[idx]);
    }
}

/**
 * @brief   imt 命令初始化
 * @param   void
 * @return  errno
 * @note
 */
void swCmdInit(void)
{
    swCmdRegister(sgImtCmd, ARRAY_SIZE(sgImtCmd));
}

/**
 * @brief   cli shell初始化封装
 * @param   void
 * @return  EXIT_SUCCESS/-EXIT_FAILURE
 * @warning 阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 * @note    2025.07.07  taohb
 */
S32 clishellInit(void)
{
    S32 ret = EXIT_SUCCESS;

    ///< cli模块初始化
    if (cliInit() != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("cliInit failed\r\n");
        ret = -EXIT_FAILURE;
        goto lEnd;
    }

    cliLoginStatSet(false);
    if (cliUartSessionCreate() != CLI_ERRNO_OK) {
        CLI_LOG_ERROR("cliUartSessionCreate failed\r\n");
        ret = -EXIT_FAILURE;
        goto lEnd;
    }
    ///< register common cmds.
    swCmdInit();
lEnd:
    return ret;
}
