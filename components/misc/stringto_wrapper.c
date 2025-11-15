
/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file stringto_wrapper.h
 * @author taohb@starsmicrosystem.com
 * @date 2025/05/21
 * @brief wrapper for strtoxxx, for example strtol/strtoul.
 */

#include "stringto_wrapper.h"
#include <rtems/score/percpu.h>
#include <sys/reent.h>
#include <rtems/score/thread.h>
#include <errno.h>

struct _reent *__real___getreent(void);

struct _reent *__wrap___getreent(void)
{
    if (_Thread_Get_executing()) {
        return __real___getreent();
    } else {
        return _GLOBAL_REENT;
    }
}

S32 str2L(const S8 *str, S32 *out)
{
    S32 ret = EXIT_SUCCESS;
    S32 result;
    S8 *endptr;

    errno = 0; /* To distinguish success/failure after call */
    result = strtol(str, &endptr, 0);
    if (errno != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    if (endptr == str) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    *out = result;
exit:
    return ret;
}

S32 str2Ul(const S8 *str, U32 *out)
{
    S32 ret = EXIT_SUCCESS;
    S32 result;
    S8 *endptr;

    errno = 0; /* To distinguish success/failure after call */
    result = strtoul(str, &endptr, 0);
    if (errno != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    if (endptr == str) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    *out = result;
exit:
    return ret;
}
