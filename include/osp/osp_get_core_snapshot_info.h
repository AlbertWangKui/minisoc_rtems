#ifndef __OSP_GET_CORE_SNAPSHOT_INFO_H__
#define __OSP_GET_CORE_SNAPSHOT_INFO_H__

#include <osp_cli.h>
#include <osp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _QueryThreadInfo
{
    OspID queryThreadID;
    int64_t switchInTime;
    int64_t switchOutTime;
}QueryThreadInfo;

typedef struct _QueryStackInfo
{
    unsigned char *stack;
    int length;
}QueryStackInfo;

typedef struct _CoreSnapshotInfo {

    QueryThreadInfo queryThreadInfo;
    QueryStackInfo queryStackInfo;

}CoreSnapshotInfo_s;

/**
 * @brief   打印 coreid 对应core 的调试信息
 *              中断信息
 *              cpu 超时信息
 *              是否有中断风暴
 *          返回业务最后执行任务的栈信息和任务切换时间信息
 * @param   coreID [in],  指定需要打印信息的coreid
 * @param   returnInfo [out], 指向获取任务切换时间和栈信息的存放地址
 *                            这里返回的是出先wdt时检测到coreid 指定的core上最后执行的任务的相关信息
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e  ospGetCoreSnapshotInfo(unsigned int coreID , CoreSnapshotInfo_s * returnInfo);


#ifdef __cplusplus
}
#endif
#endif
