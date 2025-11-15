/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_rwlock.c
 * @author  hulong
 * @date    2021.04.21
 * @brief   封装rtems操作系统rwlock对外接口
 * @note    NA
 */

#include <string.h>
#include <osp_rwlock.h>
#include <rtems/score/corerwlockimpl.h>

typedef struct {
  unsigned long flags;
    CORE_RWLock_Control RWLock;
    } InnerOspRWLock_t;

#define RWLOCK_VALIDATE_OBJECT( rw ) \
  do { \
    if ( ( rw ) == NULL ) { \
      return OSP_INVALID_ADDRESS; \
    } \
    if ( ( (uintptr_t) ( rw ) ^ RWLOCK_MAGIC ) != ( rw )->flags ) { \
      if ( !RWLockAutoInitialization( rw ) ) { \
        return OSP_INVALID_NUMBER; \
      } \
    } \
  } while ( 0 )


bool RWLockAutoInitialization( InnerOspRWLock_t *rwlock )
{
  InnerOspRWLock_t zero;

  memset( &zero, 0, sizeof( zero ) );

  if ( memcmp( rwlock, &zero, sizeof( *rwlock ) ) != 0 ) {
    return false;
  }

  rwlock->flags = (uintptr_t) rwlock ^ RWLOCK_MAGIC;
  return true;
}


/**
 * @brief   Allocate resources to use the read-write lock and Initialize it
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockInit(OspRWLock_t *_rwlock)
{
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	if ( rwlock == NULL )
	{
		return OSP_INVALID_ADDRESS;
	}

	rwlock->flags = (uintptr_t) rwlock ^ RWLOCK_MAGIC;
	_CORE_RWLock_Initialize(&rwlock->RWLock);
	return OSP_SUCCESSFUL;

}

/**
 * @brief   Destroy a RWLock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockDestroy(OspRWLock_t *_rwlock)
{
	Thread_queue_Context queue_context;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );

	(void)_CORE_RWLock_Acquire(&rwlock->RWLock, &queue_context);

	/*
	 *	If there is at least one thread waiting, then do not delete it.
	 */

	if ( !_Thread_queue_Is_empty(&rwlock->RWLock.Queue.Queue) ) {
	  _CORE_RWLock_Release(&rwlock->RWLock, &queue_context);
	  return OSP_RESOURCE_IN_USE;
	}

	/*
	 *doesn't require behavior when it is locked.
	 */
	rwlock->flags = ~rwlock->flags;
	_CORE_RWLock_Release(&rwlock->RWLock, &queue_context);
	return OSP_SUCCESSFUL;

}

/**
 * @brief   Obtain a Read Lock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockRDLock(OspRWLock_t *_rwlock)
{
	Thread_queue_Context queue_context;
	Status_Control		  status;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );
	_Thread_queue_Context_initialize(&queue_context);
	_Thread_queue_Context_set_enqueue_do_nothing_extra(&queue_context);
    status = _CORE_RWLock_Seize_for_reading(
		            &rwlock->RWLock,
				    true,                 /* we are willing to wait forever */
				    &queue_context
			  );

	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));
}

/**
 * @brief    Attempt to Obtain a Read Lock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTryRDLock(OspRWLock_t *_rwlock)
{
    Thread_queue_Context  queue_context;
	Status_Control 	   status;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );

	_Thread_queue_Context_initialize( &queue_context );
	status = _CORE_RWLock_Seize_for_reading(
	   &rwlock->RWLock,
	   false,				   /* do not wait for the rwlock */
	   &queue_context
	);
	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));
}

/**
 * @brief   Attempt to Obtain a Read Lock on a RWLock Instance
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTimedRDLock(OspRWLock_t *_rwlock, const struct timespec *abstime)
{
	Thread_queue_Context  queue_context;
	Status_Control		  status;
	InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );

	_Thread_queue_Context_initialize( &queue_context );
	_Thread_queue_Context_set_enqueue_timeout_realtime_timespec(
	  &queue_context,
	  abstime
	);
	status = _CORE_RWLock_Seize_for_reading(
	  &rwlock->RWLock,
	  true,
	  &queue_context
	);
	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));
}

/**
 * @brief   Obtain a Write Lock on a RWlock Instance
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockWRLock(OspRWLock_t *_rwlock)
{
	Thread_queue_Context  queue_context;
	Status_Control		status;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT(rwlock);

	_Thread_queue_Context_initialize( &queue_context );
	_Thread_queue_Context_set_enqueue_do_nothing_extra( &queue_context );
	status = _CORE_RWLock_Seize_for_writing(
		&rwlock->RWLock,
		true,		   /* do not timeout -- wait forever */
		&queue_context
		);

	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));

}

/**
 * @brief    Attempt to Obtain a Write Lock on a RWLock Instance
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTryWRLock(OspRWLock_t *_rwlock)
{
	Thread_queue_Context  queue_context;
	Status_Control		  status;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );

	_Thread_queue_Context_initialize( &queue_context );
	status = _CORE_RWLock_Seize_for_writing(
	  &rwlock->RWLock,
	  false,				 /* we are not willing to wait */
	  &queue_context
	);
	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));
}

/**
 * @brief   Function applies a Write lock to RWLock referenced by rwlock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockTimedWRLock(OspRWLock_t *_rwlock, const struct timespec *abstime)
{
	Thread_queue_Context  queue_context;
	Status_Control		  status;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );

	_Thread_queue_Context_initialize( &queue_context );
	_Thread_queue_Context_set_enqueue_timeout_realtime_timespec(
	  &queue_context,
	  abstime
	);
	status = _CORE_RWLock_Seize_for_writing(
	  &rwlock->RWLock,
	  true,
	  &queue_context
	);
	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));
}

/**
 * @brief   Function Releases a lock held on RWLock object referenced by rwlock
 * @param   rwlock [in/out]
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospRWLockUnlock(OspRWLock_t *_rwlock)
{
	Status_Control		  status;
    InnerOspRWLock_t *rwlock =(InnerOspRWLock_t *) _rwlock;

	RWLOCK_VALIDATE_OBJECT( rwlock );

	status = _CORE_RWLock_Surrender( &rwlock->RWLock );
	return (OspStatusCode_e)(STATUS_GET_CLASSIC(status));
}

