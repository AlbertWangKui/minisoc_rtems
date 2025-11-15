#ifndef __OSP_MONITOR_H__
#define __OSP_MONITOR_H__

#include <osp_status.h>
#include <osp_types.h>

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief   进入monitor模式
 * @param   originTaskID [in], 进入monitor模式的调用者任务id（一般为业务cli任务）
 * @param   *monitorTaskID [out], 函数返回的monitor任务id
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospEnterMonitorMode(OspID originTaskID, OspID *monitorTaskID);

/**
 * @brief   退出monitor模式
 * @param   void
 * @return  void
 * @warning 无
 * @note    无
 */
void ospExitMonitorMode(void);
#ifdef __cplusplus
}
#endif
#endif
