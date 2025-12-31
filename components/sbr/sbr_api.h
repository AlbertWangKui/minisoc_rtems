#ifndef __SBR_API_H__
#define __SBR_API_H__

#include "common_defines.h"
#include "bsp_config.h"
#include "bsp_sbr.h"

#define MAX_SBR_BUF_SIZE                128
#define MAX_CHUNK_SIZE                  32

#define SBR_PRJ_CODE_TIANLONG_CS        0x20
#define SBR_PRJ_CODE_SHESHOU            0x21
#define SBR_PRJ_CODE_TIANHE             0x22
#define SBR_PRJ_CODE_JUXIE              0x23
#define SBR_PRJ_CODE_MOJIE              0x24
#define HEADER_OFFSET_SBR               0x140

#if defined(CONFIG_BSP_SHESHOU)
#define SBR_PRJ_CODE SBR_PRJ_CODE_SHESHOU
#elif defined(CONFIG_BSP_TIANHE)
#define SBR_PRJ_CODE SBR_PRJ_CODE_TIANHE
#elif defined(CONFIG_BSP_TIANLONG_CS)
#define SBR_PRJ_CODE SBR_PRJ_CODE_TIANLONG_CS
#elif defined(CONFIG_BSP_JUXIE)
#define SBR_PRJ_CODE SBR_PRJ_CODE_JUXIE
#elif defined(CONFIG_BSP_MOJIE)
#define SBR_PRJ_CODE SBR_PRJ_CODE_MOJIE
#else
#error "No SBR project code defined"
#endif

#define SBR_MAGIC       0x5342
#define SBR_VERSION     0x10
#define SBR_HEADER_ID   ((SBR_PRJ_CODE << 8) | SBR_VERSION)

typedef struct {
    U16 magic;
    U16 hdrID;
    U32 size; /* size of entries+data, not include header */
    U16 entryCount;
    U16 sbrMetaSize; /* sizeof(sbrHeader_s) + entryCount * sizeof(sbrEntry_s) */
} __attribute__((packed)) sbrHeader_s;

typedef struct {
    U16 devID: 14;
    U16 valid: 2;
    U32 size;   /* 配置数据大小 */
    U32 offset; /* 配置数据在sbrData_s中的偏移 */
} __attribute__((packed)) sbrEntry_s;

typedef struct {
    sbrHeader_s header;
    sbrEntry_s entries[1024];/* 用于占位，实际使用时会根据entryCount分配 */
    sbrData_s data;
} __attribute__((packed)) sbrImage_s;

S32 sbrValidate(void);
U32 devSbrRead(DevList_e devID, void *CfgBuf, U32 offset, U32 len);
U32 devSbrWrite(DevList_e devID, void *CfgBuf, U32 offset, U32 len);
U32 usrSettingGet(U32 offset, U8 *buf, U8 len);
U32 usrSettingSet(U32 offset, U8 *buf, U8 len);
S32 usrBitMapReset(void);

#endif
