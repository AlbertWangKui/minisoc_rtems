/**
 * Copyright (C), 2021, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_monitor.c
 * @author  tianye
 * @date    2021.12.28
 * @brief   封装rtems monitor相关接口
 * @note    NA
 */

#include <rtems.h>
#include <rtems/monitor.h>
#include <rtems/capture-cli.h>
#include <osp_common.h>
#include <osp_task.h>

///< 全局变量保存业务cli任务id，便于暂停后恢复
static OspID gOriginTaskID = 0;
///< 业务注册的monitor退出命令（恢复业务cli任务并退出monitor任务）
static void ospStopMonitor(int argc, char **argv,
        const rtems_monitor_command_arg_t* command_arg, bool verbose)
{
    argc = argc;
    argv = argv;
    command_arg = command_arg;
    verbose = verbose;

    ospTaskResume(gOriginTaskID);
    rtems_monitor_kill();
}
static rtems_monitor_command_entry_t capture_cmds[] =
{
    {
        "stopmonitor",
        "usage: \n",
        0,
        ospStopMonitor,
        { 0 },
        0
    }
};

/**
 * @brief   进入monitor模式
 * @param   originTaskID [in], 进入monitor模式的调用者任务id（一般为业务cli任务）
 * @param   *monitorTaskID [out], 函数返回的monitor任务id
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospEnterMonitorMode(OspID originTaskID, OspID *monitorTaskID)
{
    size_t cmd;
    OspStatusCode_e status;

    if (monitorTaskID == NULL) {
        status = OSP_INVALID_ADDRESS;
        goto exit;
    }

    gOriginTaskID = originTaskID;
    ///< osp封装接口初始化monitor
    rtems_monitor_init (0);
    ///< osp封装接口注册capture
    rtems_capture_cli_init (0);
    ///< 业务向monitor框架注册退出monitor名
    for (cmd = 0; cmd < sizeof (capture_cmds) / sizeof (rtems_monitor_command_entry_t); cmd++) {
        rtems_monitor_insert_cmd (&capture_cmds[cmd]);
    }
    ///< 获取刚创建的monitor任务id，返回给调用者并由其设置合适的优先级和属性
    status = ospTaskIdent(RTEMS_MONITOR_NAME, OSP_SEARCH_ALL_NODES, monitorTaskID);
    if (status != OSP_SUCCESSFUL) {
        rtems_monitor_kill();
        goto exit;
    }

exit:
    return status;
}

/**
 * @brief   退出monitor模式
 * @param   void
 * @return  void
 * @warning 无
 * @note    无
 */
void ospExitMonitorMode(void)
{
    rtems_monitor_kill();
}
