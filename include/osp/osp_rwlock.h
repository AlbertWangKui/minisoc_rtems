 /**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_rwlock.h
 * @author  hulong
 * @date    2021.04.21
 * @brief   porting rtems rwlock
 * @note    NA
 */

#ifndef __OSP_RWLOCK_H
#define __OSP_RWLOCK_H

#include <osp_status.h>
#include <osp_types.h>
#include <sys/lock.h>

//__BEGIN_DECLS
#ifdef __cplusplus
extern "C" {
#endif

#define RWLOCK_MAGIC 0x9621dabdUL

typedef struct {
   unsigned long _flags;
   struct _Thread_queue_Queue _Queue;
   unsigned int _current_state;
   unsigned int _number_of_readers;
} OspRWLock_t;

/**
 * @brief   Allocate resources to use the read-write lock and Initialize it
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockInit(OspRWLock_t *_rwlock);

/**
 * @brief   Destroy a RWLock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockDestroy(OspRWLock_t *_rwlock);

/**
 * @brief   Obtain a Read Lock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockRDLock(OspRWLock_t *_rwlock);

/**
 * @brief    Attempt to Obtain a Read Lock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTryRDLock(OspRWLock_t *_rwlock);

/**
 * @brief   Attempt to Obtain a Read Lock on a RWLock Instance
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTimedRDLock(OspRWLock_t *_rwlock, const struct timespec *abstime);

/**
 * @brief   Obtain a Write Lock on a RWlock Instance
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockWRLock(OspRWLock_t *_rwlock);

/**
 * @brief    Attempt to Obtain a Write Lock on a RWLock Instance
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTryWRLock(OspRWLock_t *_rwlock);

/**
 * @brief   Function applies a Write lock to RWLock referenced by rwlock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTimedWRLock(OspRWLock_t *_rwlock, const struct timespec *abstime);


/**
 * @brief   Function Releases a lock held on RWLock object referenced by rwlock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockUnlock(OspRWLock_t *_rwlock);


#ifdef __cplusplus
}
#endif
//__END_DECLS
#endif
