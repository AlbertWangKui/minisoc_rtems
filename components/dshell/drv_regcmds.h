#ifndef __DRV_REGCMDS_H__
#define __DRV_REGCMDS_H__

#include <stdint.h>
#include "common_defines.h"
#include "log_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BYTES_PER_LINE 16

typedef enum {
    WIDTH_INVALID = 0,
    WIDTH_8 = 1,
    WIDTH_16 = 2,
    WIDTH_32 = 4,
    WIDTH_64 = 8,
} DataWidth_t;

int parseNumber(const char* str, U64* result);
int checkAlignment(U64 addr, DataWidth_t width);
DataWidth_t parseWidthOption(const char* arg);
int isValidBitPosition(int bit, DataWidth_t width);

void cmdMd(S32 argc, S8 **argv);
void cmdMm(S32 argc, S8 **argv);
void cmdMdr(S32 argc, S8 **argv);
void cmdMmr(S32 argc, S8 **argv);
void cmdSb(S32 argc, S8 **argv);
void cmdCb(S32 argc, S8 **argv);

void drvRegcmdsInit(void);

#ifdef __cplusplus
}
#endif

#endif