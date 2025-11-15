/**
 * Copyright (C), 2020, Starsmicrosystem Technologies Co.,Ltd
 *
 * @file    cli_core.c
 * @author  王殿卫
 * @data    2021.04.12
 * @brief   cli打印代码
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "cli_session.h"
#include "cli_printf.h"
#include "cli_errno.h"
#include "cli_porting.h"

/**
 * @brief  打印到缓存
 * @param  format   格式
 * @param  list     格式参数列表
 * @note       2021.04.20  xiezhj
 */
static void cliPrintVa(U32 sessionId, S8 *format, cliVaList list)
{
    cliVaList va;
    S32                rc        = -CLI_ERRNO_FAILED;
    S32                size      = 0;
    CliSessionObj_s *pSession = NULL;

    memset(&va, 0, sizeof(va));
    cliVaCopy(va, list);

    size = cliVSNPrintf(NULL, 0, format, va);
    S8 buf[size+1];

    rc = cliGetSessionById(sessionId, &pSession);
    if (rc != CLI_ERRNO_OK) {
        CLI_LOG_DEBUG("get session error, rc = %d, sessionId = %d", rc, sessionId);
        goto lEnd;
    }

    (void)cliVSNPrintf(buf, (S32)sizeof(buf), format, list);
    buf[size]='\0';

    (void)pSession->devIo.putsFunc(buf);

lEnd:
    return;
}

/**
 * @brief   根据不同session打印
 * @param   sessionId [in]
 * @param   format [in]  打印的格式
 * @return  void
 * @note       2021.04.20  xiezhj
 */
void cliSessionPrintf(U32 sessionId, S8 *format, ...)
{
    cliVaList list;
    cliVaStart(list, format);
    cliPrintVa(sessionId, format, list);
    cliVaEnd(list);

    return;
}

/**
 * @brief      根据session带颜色打印
 * @param      [in]
 * @param      [out]
 * @return     CLI_ERRNO_OK/-CLI_ERRNO_FAILED
 * @warning
 * @note       2021.04.20  xiezhj
 */
void cliSessionPrintfColor(U8 color, S8 *format, ...)
{
    return;
}
