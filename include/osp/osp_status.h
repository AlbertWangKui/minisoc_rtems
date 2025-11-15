/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_status.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting status
 * @note    NA
 */

#ifndef __OSP_STATUS_H__
#define __OSP_STATUS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @brief Classic API Status
 *
 *  This enumerates the possible status values returned b
 *  Classic API directives.
 */
typedef enum OspStatusCode {
    /**
     *  This is the status to indicate successful completion.
     */
    OSP_SUCCESSFUL = 0,
    /**
     *  This is the status to indicate that a thread exited.
     */
    OSP_TASK_EXITTED = 1,
    /**
     *  This is the status to indicate multiprocessing is not configured.
     */
    OSP_MP_NOT_CONFIGURED = 2,
    /**
     *  This is the status to indicate that the object name was invalid.
     */
    OSP_INVALID_NAME = 3,
    /**
     *  This is the status to indicate that the object Id was invalid.
     */
    OSP_INVALID_ID = 4,
    /**
     *  This is the status to indicate you have attempted to create too many
     *  instances of a particular object class.
     */
    OSP_TOO_MANY = 5,
    /**
     *  This is the status to indicate that a blocking directive timed out.
     */
    OSP_TIMEOUT = 6,
    /**
     *  This is the status to indicate the the object was deleted
     *  while the task was blocked waiting.
     */
    OSP_OBJECT_WAS_DELETED = 7,
    /**
     *  This is the status to indicate that the specified size was invalid.
     */
    OSP_INVALID_SIZE = 8,
    /**
     *  This is the status to indicate that the specified address is invalid.
     */
    OSP_INVALID_ADDRESS = 9,
    /**
     *  This is the status to indicate that the specified number was invalid.
     */
    OSP_INVALID_NUMBER = 10,
    /**
     *  This is the status to indicate that the item has not been initialized.
     */
    OSP_NOT_DEFINED = 11,
    /**
     *  This is the status to indicate that the object still has
     *  resources in use.
     */
    OSP_RESOURCE_IN_USE = 12,
    /**
     *  This is the status to indicate that the request was not satisfied.
     */
    OSP_UNSATISFIED = 13,
    /**
     *  This is the status to indicate that a thread is in wrong state
     *  was in the wrong execution state for the requested operation.
     */
    OSP_INCORRECT_STATE = 14,
    /**
     *  This is the status to indicate thread was already suspended.
     */
    OSP_ALREADY_SUSPENDED = 15,
    /**
     *  This is the status to indicate that the operation is illegal
     *  on calling thread.
     */
    OSP_ILLEGAL_ON_SELF = 16,
    /**
     *  This is the status to indicate illegal for remote object.
     */
    OSP_ILLEGAL_ON_REMOTE_OBJECT = 17,
    /**
     *  This is the status to indicate that the operation should not be
     *  called from from this excecution environment.
     */
    OSP_CALLED_FROM_ISR = 18,
    /**
     *  This is the status to indicate that an invalid thread priority
     *  was provided.
     */
    OSP_INVALID_PRIORITY = 19,
    /**
     *  This is the status to indicate that the specified date/time was invalid.
     */
    OSP_INVALID_CLOCK = 20,
    /**
     *  This is the status to indicate that the specified node Id was invalid.
     */
    OSP_INVALID_NODE = 21,
    /**
     *  This is the status to indicate that the directive was not configured.
     */
    OSP_NOT_CONFIGURED = 22,
    /**
     *  This is the status to indicate that the caller is not the
     *  owner of the resource.
     */
    OSP_NOT_OWNER_OF_RESOURCE = 23,
    /**
     *  This is the status to indicate the the directive or requested
     *  portion of the directive is not implemented.  This is a hint
     *  that you have stumbled across an opportunity to submit code
     *  to the osp Project.
     */
    OSP_NOT_IMPLEMENTED = 24,
    /**
     *  This is the status to indicate that an internal osp inconsistency
     *  was detected.
     */
    OSP_INTERNAL_ERROR = 25,
    /**
     *  This is the status to indicate that the directive attempted to allocate
     *  memory but was unable to do so.
     */
    OSP_NO_MEMORY = 26,
    /**
     *  This is the status to indicate an driver IO error.
     */
    OSP_IO_ERROR = 27,
    /**
     *  This is the status used internally to indicate a blocking device
     *  driver call has been interrupted and should be reflected to the
     *  called as an INTERRUPTED.
     */
    OSP_INTERRUPTED = 28,
    /**
     *  This is the status is used internally to osp when performing
     *  operations on behalf of remote tasks.  This is referred to as
     *  proxying operations and this status indicates that the operation
     *  could not be completed immediately and the "proxy is blocking."
     *
     *  @note This status will @b NOT be returned to the user.
     */
    OSP_PROXY_BLOCKING = 29
} OspStatusCode_e;

#define OSP_STATUS_CODES_LAST OSP_PROXY_BLOCKING

#ifdef __cplusplus
}
#endif

#endif
