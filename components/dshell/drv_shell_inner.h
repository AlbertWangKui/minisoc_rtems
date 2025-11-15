#ifndef __DRV_SHELL_INNER_H__
#define __DRV_SHELL_INNER_H__

#ifdef __cplusplus
    extern "C" {
#endif

#include "menuconfig.h"

#define MAX_CMDS        (CONFIG_MAX_USHELL_CMD_NUM)
#define CMD_NAME_SIZE   64

typedef struct {
    S8 cmdName[CMD_NAME_SIZE];
    void (*cmdFunc)(S32, S8**);
} CmdEntry_s;

#ifdef __cplusplus
}
#endif

#endif
