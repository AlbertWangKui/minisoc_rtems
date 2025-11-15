/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_errno.h
 * @author  tianye
 * @date    2020.10.27
 * @brief   osp模块对外错误码定义和解析函数声明(移植ps3库上的ps3_errno_base.h)
 * @note    NA
 */

#ifndef _OSP_ERRNO_H_
#define _OSP_ERRNO_H_

#ifdef __cplusplus
extern "C" {
#endif
/**
 * 32位错误码bit位图
 *  31 |30~27 |26  -  20 |19  -  12|11     -     0|
 *  -   ----   -------    ---- ---- ---- ---- ----
 * flag|module|sub module|extraid  |    errno     |
 */
#define OSP_ABS(_v)               (((_v) < 0) ? (0 - (_v)) : (_v)) ///< 取绝对值
#define OSP_MODULE_SYSTEM         (1)  ///< PS3库给操作系统和基础模块分配的模块id为1
#define OSP_ERR_MODULE_OFFSET     (27) ///< 错误码位图中模块偏移
#define OSP_ERR_SUB_MODULE_OFFSET (20) ///< 错误码位图中子模块偏移
#define OSP_ERR_EXTRA_OFFSET                                                                                           \
    (12) ///< 错误码伪终中错误详细描述偏移：为了详细描述错误，供子模块使用的附加信息ID，可以做时间编号，也可以不填
#define OSP_ERR_BASE_OFFSET (0)          ///< 错误码位图中错误码偏移
#define OSP_ERR_SYSTEM_NUM  (540)        ///< 系统错误码最大枚举值
#define OSP_MODULE_MASK     (0x78000000) ///< 模块ID掩码
#define OSP_SUB_MODULE_MASK (0x07F00000) ///< 子模块ID掩码
#define OSP_EXTRA_MASK      (0x000FF000) ///< 扩展掩码
#define OSP_BASE_MASK       (0x00000FFF) ///< 错误码ID掩码

///< OSP子模块枚举定义，当前有两个子模块：KERNEL和FLASH_REGION，后续如果需要再新增
typedef enum OspSubModule {
    OSP_SUB_MODULE_KERNEL = 1,
    OSP_SUB_MODULE_FLASH_REGION,
    OSP_SUB_MODULE_PROFILING_PMU,
    OSP_SUB_MODULE_BUTT,
} OspSubModule_e;

///< OSP系统错误码定义，参考linux errno.h和errno-base.h，同ps3_errno_base.h的Ps3SysError_t
typedef enum OspSysErrno {
    OSP_EPERM = 1,    ///< 1,     Operation not permitted
    OSP_ENOENT,       ///< 2,     No such file or directory
    OSP_ESRCH,        ///< 3,     No such process
    OSP_EINTR,        ///< 4,     Interrupted system call
    OSP_EIO,          ///< 5,     I/O error
    OSP_ENXIO,        ///< 6,     No such device or address
    OSP_E2BIG,        ///< 7,     Argument list too long
    OSP_ENOEXEC,      ///< 8,     Exec format error
    OSP_EBADF,        ///< 9,     Bad file number
    OSP_ECHILD,       ///< 10,    No child processes
    OSP_EAGAIN,       ///< 11,    Try again
    OSP_ENOMEM,       ///< 12,    Out of memory
    OSP_EACCES,       ///< 13,    Permission denied
    OSP_EFAULT,       ///< 14,    Bad address
    OSP_ENOTBLK,      ///< 15,    Block device required
    OSP_EBUSY,        ///< 16,    Device or resource busy
    OSP_EEXIST,       ///< 17,    File exists
    OSP_EXDEV,        ///< 18,    Cross-device link
    OSP_ENODEV,       ///< 19,    No such device
    OSP_ENOTDIR,      ///< 20,    Not a directory
    OSP_EISDIR,       ///< 21,    Is a directory
    OSP_EINVAL,       ///< 22,    Invalid argument
    OSP_ENFILE,       ///< 23,    File table overflow
    OSP_EMFILE,       ///< 24,    Too many open files
    OSP_ENOTTY,       ///< 25,    Not a typewriter
    OSP_ETXTBSY,      ///< 26,    Text file busy
    OSP_EFBIG,        ///< 27,    File too large
    OSP_ENOSPC,       ///< 28,    No space left on device
    OSP_ESPIPE,       ///< 29,    Illegal seek
    OSP_EROFS,        ///< 30,    Read-only file system
    OSP_EMLINK,       ///< 31,    Too many links
    OSP_EPIPE,        ///< 32,    Broken pipe
    OSP_EDOM,         ///< 33,    Math argument out of domain of func
    OSP_ERANGE,       ///< 34,    Math result not representable
    OSP_EDEADLK,      ///< 35,    Resource deadlock would occur
    OSP_ENAMETOOLONG, ///< 36,    File name too long
    OSP_ENOLCK,       ///< 37,    No record locks available
    /**
     * This error code is special: arch syscall entry code will return
     * -ENOSYS if users try to call a syscall that doesn't exist.  To keep
     * failures of syscalls that really do exist distinguishable from
     * failures due to attempts to use a nonexistent syscall, syscall
     * implementations should refrain from returning -ENOSYS.
     */
    OSP_ENOSYS,          ///< 38,    Invalid system call number
    OSP_ENOTEMPTY,       ///< 39,    Directory not empty
    OSP_ELOOP,           ///< 40,    Too many symbolic links encountered
    OSP_EWOULDBLOCK,     ///< 41,    OSP_EAGAIN, Operation would block
    OSP_ENOMSG,          ///< 42,    No message of desired type
    OSP_EIDRM,           ///< 43,    Identifier removed
    OSP_ECHRNG,          ///< 44,    Channel number out of range
    OSP_EL2NSYNC,        ///< 45,    Level 2 not synchronized
    OSP_EL3HLT,          ///< 46,    Level 3 halted
    OSP_EL3RST,          ///< 47,    Level 3 reset
    OSP_ELNRNG,          ///< 48,    Link number out of range
    OSP_EUNATCH,         ///< 49,    Protocol driver not attached
    OSP_ENOCSI,          ///< 50,    No CSI structure available
    OSP_EL2HLT,          ///< 51,    Level 2 halted
    OSP_EBADE,           ///< 52,    Invalid exchange
    OSP_EBADR,           ///< 53,    Invalid request descriptor
    OSP_EXFULL,          ///< 54,    Exchange full
    OSP_ENOANO,          ///< 55,    No anode
    OSP_EBADRQC,         ///< 56,    Invalid request code
    OSP_EBADSLT,         ///< 57,    Invalid slot
    OSP_EDEADLOCK,       ///< 58,    OSP_EDEADLK
    OSP_EBFONT,          ///< 59,    Bad font file format
    OSP_ENOSTR,          ///< 60,    Device not a stream
    OSP_ENODATA,         ///< 61,    No data available
    OSP_ETIME,           ///< 62,    Timer expired
    OSP_ENOSR,           ///< 63,    Out of streams resources
    OSP_ENONET,          ///< 64,    Machine is not on the network
    OSP_ENOPKG,          ///< 65,    Package not installed
    OSP_EREMOTE,         ///< 66,    Object is remote
    OSP_ENOLINK,         ///< 67,    Link has been severed
    OSP_EADV,            ///< 68,    Advertise error
    OSP_ESRMNT,          ///< 69,    Srmount error
    OSP_ECOMM,           ///< 70,    Communication error on send
    OSP_EPROTO,          ///< 71,    Protocol error
    OSP_EMULTIHOP,       ///< 72,    Multihop attempted
    OSP_EDOTDOT,         ///< 73,    RFS specific error
    OSP_EBADMSG,         ///< 74,    Not a data message
    OSP_EOVERFLOW,       ///< 75,    Value too large for defined data type
    OSP_ENOTUNIQ,        ///< 76,    Name not unique on network
    OSP_EBADFD,          ///< 77,    File descriptor in bad state
    OSP_EREMCHG,         ///< 78,    Remote address changed
    OSP_ELIBACC,         ///< 79,    Can not access a needed shared library
    OSP_ELIBBAD,         ///< 80,    Accessing a corrupted shared library
    OSP_ELIBSCN,         ///< 81,    .lib section in a.out corrupted
    OSP_ELIBMAX,         ///< 82,    Attempting to link in too many shared libraries
    OSP_ELIBEXEC,        ///< 83,    Cannot exec a shared library directly
    OSP_EILSEQ,          ///< 84,    Illegal byte sequence
    OSP_ERESTART,        ///< 85,    Interrupted system call should be restarted
    OSP_ESTRPIPE,        ///< 86,    Streams pipe error
    OSP_EUSERS,          ///< 87,    Too many users
    OSP_ENOTSOCK,        ///< 88,    Socket operation on non-socket
    OSP_EDESTADDRREQ,    ///< 89,    Destination address required
    OSP_EMSGSIZE,        ///< 90,    Message too long
    OSP_EPROTOTYPE,      ///< 91,    Protocol wrong type for socket
    OSP_ENOPROTOOPT,     ///< 92,    Protocol not available
    OSP_EPROTONOSUPPORT, ///< 93,    Protocol not supported
    OSP_ESOCKTNOSUPPORT, ///< 94,    Socket type not supported
    OSP_EOPNOTSUPP,      ///< 95,    Operation not supported on transport endpoint
    OSP_EPFNOSUPPORT,    ///< 96,    Protocol family not supported
    OSP_EAFNOSUPPORT,    ///< 97,    Address family not supported by protocol
    OSP_EADDRINUSE,      ///< 98,    Address already in use
    OSP_EADDRNOTAVAIL,   ///< 99,    Cannot assign requested address
    OSP_ENETDOWN,        ///< 100,   Network is down
    OSP_ENETUNREACH,     ///< 101,   Network is unreachable
    OSP_ENETRESET,       ///< 102,   Network dropped connection because of reset
    OSP_ECONNABORTED,    ///< 103,   Software caused connection abort
    OSP_ECONNRESET,      ///< 104,   Connection reset by peer
    OSP_ENOBUFS,         ///< 105,   No buffer space available
    OSP_EISCONN,         ///< 106,   Transport endpoint is already connected
    OSP_ENOTCONN,        ///< 107,   Transport endpoint is not connected
    OSP_ESHUTDOWN,       ///< 108,   Cannot send after transport endpoint shutdown
    OSP_ETOOMANYREFS,    ///< 109,   Too many references: cannot splice
    OSP_ETIMEDOUT,       ///< 110,   Connection timed out
    OSP_ECONNREFUSED,    ///< 111,   Connection refused
    OSP_EHOSTDOWN,       ///< 112,   Host is down
    OSP_EHOSTUNREACH,    ///< 113,   No route to host
    OSP_EALREADY,        ///< 114,   Operation already in progress
    OSP_EINPROGRESS,     ///< 115,   Operation now in progress
    OSP_ESTALE,          ///< 116,   Stale file handle
    OSP_EUCLEAN,         ///< 117,   Structure needs cleaning
    OSP_ENOTNAM,         ///< 118,   Not a XENIX named type file
    OSP_ENAVAIL,         ///< 119,   No XENIX semaphores available
    OSP_EISNAM,          ///< 120,   Is a named type file
    OSP_EREMOTEIO,       ///< 121,   Remote I/O error
    OSP_EDQUOT,          ///< 122,   Quota exceeded
    OSP_ENOMEDIUM,       ///< 123,   No medium found
    OSP_EMEDIUMTYPE,     ///< 124,   Wrong medium type
    OSP_ECANCELED,       ///< 125,   Operation Canceled
    OSP_ENOKEY,          ///< 126,   Required key not available
    OSP_EKEYEXPIRED,     ///< 127,   Key has expired
    OSP_EKEYREVOKED,     ///< 128,   Key has been revoked
    OSP_EKEYREJECTED,    ///< 129,   Key was rejected by service
    ///< for robust mutexes
    OSP_EOWNERDEAD,      ///< 130,   Owner died
    OSP_ENOTRECOVERABLE, ///< 131,   State not recoverable
    OSP_ERFKILL,         ///< 132,   Operation not possible due to RF-kill
    OSP_EHWPOISON,       ///< 133,   Memory page has hardware error
    /**
     * These should never be seen by user programs.  To return one of ERESTART*
     * codes, signal_pending() MUST be set.  Note that ptrace can observe these
     * at syscall exit tracing, but they will never be left for the debugged user
     * process to see.
     */
    OSP_ERESTARTSYS,           ///< 512,
    OSP_ERESTARTNOINTR,        ///< 513,
    OSP_ERESTARTNOHAND,        ///< 514,   restart if no handler..
    OSP_ENOIOCTLCMD,           ///< 515,   No ioctl command
    OSP_ERESTART_RESTARTBLOCK, ///< 516, restart by calling sys_restart_syscall
    OSP_EPROBE_DEFER,          ///< 517,   Driver requests probe retry
    OSP_EOPENSTALE,            ///< 518,   open found a stale dentry
    OSP_ENOPARAM,              ///< 519,   Parameter not supported
    ///< Defined for the NFSv3 protocol
    OSP_EBADHANDLE,      ///< 521,   Illegal NFS file handle
    OSP_ENOTSYNC,        ///< 522,   Update synchronization mismatch
    OSP_EBADCOOKIE,      ///< 523,   Cookie is stale
    OSP_ENOTSUPP,        ///< 524,   Operation is not supported
    OSP_ETOOSMALL,       ///< 525,   Buffer or request is too small
    OSP_ESERVERFAULT,    ///< 526,   An untranslatable error occurred
    OSP_EBADTYPE,        ///< 527,   Type not supported by server
    OSP_EJUKEBOX,        ///< 528,   Request initiated, but will not complete before timeout
    OSP_EIOCBQUEUED,     ///< 529,   iocb queued, will get completion event
    OSP_ERECALLCONFLICT, ///< 530,   conflict with recalled state
    OSP_SYS_ERRNO_MAXN,  ///< 系统错误码最大个数
} OspSysErrno_e;

///< OSP的KERNEL子模块自定义错误码基地址
#define OSP_KERNEL_ERR_BASE                                                                                            \
    ((OSP_MODULE_SYSTEM << OSP_ERR_MODULE_OFFSET) | (OSP_SUB_MODULE_KERNEL << OSP_ERR_SUB_MODULE_OFFSET) |             \
     OSP_ERR_SYSTEM_NUM)
///< OSP的flash region子模块自定义错误码基地址
#define OSP_FLASH_ERR_BASE                                                                                             \
    ((OSP_MODULE_SYSTEM << OSP_ERR_MODULE_OFFSET) | (OSP_SUB_MODULE_FLASH_REGION << OSP_ERR_SUB_MODULE_OFFSET) |       \
     OSP_ERR_SYSTEM_NUM)

///< OSP的profiling pmu子模块自定义错误码基地址
#define OSP_PROFILING_ERR_BASE                                                                                         \
    ((OSP_MODULE_SYSTEM << OSP_ERR_MODULE_OFFSET) | (OSP_SUB_MODULE_PROFILING_PMU << OSP_ERR_SUB_MODULE_OFFSET) |      \
     OSP_ERR_SYSTEM_NUM)

/**
 * OSP模块对外错误码表
 * 总共分成如下几段：
 * 1、0                         通用执行成功返回值
 * 2、0x0810 0000~0x0810 021B   540个系统错误码在KERNEL子模块中的映射
 * 3、0x0810 021C               OSP的KERNEL子模块错误码基地址
 * 4、0x0810 021D~0x0810 0FFF   OSP的KERNEL子模块可自定义错误码区域
 * 5、0x0820 0000~0x0820 021B   540个系统错误码在FLASH REGION子模块中的映射
 * 6、0x0820 021C OSP的FLASH    REGION子模块错误码基地址
 * 7、0x0820 021D~0x0820 0FFF   OSP的FLASH REGION子模块可自定义错误码区域
 */
typedef enum OspErrno {
    OSP_OK,                                            ///< 0,执行成功
    OSP_SELF_DEFINE_KERNEL_BASE = OSP_KERNEL_ERR_BASE, ///< 0x0810 021C,内核子模块自定义错误码基地址
    OSP_SELF_DEFINE_FLASH_BASE  = OSP_FLASH_ERR_BASE,  ///< 0x0820 021C,flash子模块自定义错误码基地址
    OSP_FLASH_REG_TABLE_INVA,     ///< 自定义错误码：分区表crc校验不过或者分区表内容非法
    OSP_FLASH_BAK_REG_TABLE_INVA, ///< 自定义错误码：备份分区表crc校验不过或者分区表内容非法
    OSP_FLASH_GET_SIZE_FAILED,    ///< 自定义错误码：获取FLASH大小失败
    OSP_FLASH_READ_FAILED,        ///< 自定义错误码：读FLASH内容失败
    OSP_FLASH_WRITE_FAILED,       ///< 自定义错误码：写FLASH失败
    OSP_FLASH_ERASE_FAILED,       ///< 自定义错误码：擦除FLASH失败
    OSP_FLASH_TABLE_NOT_FOUND,    ///< 自定义错误码：分区表未找到
    OSP_FLASH_OP_MISMATCH,        ///< 自定义错误码：对FLASH的操作与FLASH分区属性不匹配
    OSP_FLASH_OP_OVER_BOUNDARY,   ///< 自定义错误码：越界操作FLASH分区
    OSP_FLASH_DRIVER_INIT_FAILED, ///< 自定义错误码：FLASH REGION模块初始化失败
    OSP_SELF_DEFINE_PROFILING_BASE = OSP_PROFILING_ERR_BASE, ///< 0x0830 021C, profiling 子模块自定义错误码基地址
    OSP_PROFILING_QUEUE_LEN_IVNA,                            ///< 自定义错误码：待测的事件数量无效
    OSP_PROFILING_QUEUE_EMPTY,                               ///< 自定义错误码：待测的事件队列为空
    OSP_PROFILING_PROCESSING,                                ///< 自定义错误码：正在进行PMU监测
    OSP_SELF_DEFINE_END,                                     ///< 自定义错误码：OSP自定义错误码结束
} OspErrno_e;

///< 获取错误码模块ID
#define OSP_ERR_MODULE_GET(errno) ((OSP_ABS(errno) & OSP_MODULE_MASK) >> OSP_ERR_MODULE_OFFSET)
///< 获取错误码子模块ID
#define OSP_ERR_SUB_MODULE_GET(errno) ((OSP_ABS(errno) & OSP_SUB_MODULE_MASK) >> OSP_ERR_SUB_MODULE_OFFSET)
///< 获取错误码事件ID
#define OSP_ERR_EXTRA_GET(errno) ((OSP_ABS(errno) & OSP_EXTRA_MASK) >> OSP_ERR_EXTRA_OFFSET)
///< 获取错误码ID
#define OSP_ERR_NUM_GET(errno) ((OSP_ABS(errno) & OSP_BASE_MASK) >> OSP_ERR_BASE_OFFSET)
///< 构建模块错误码编号
#define OSP_ERR_NO(module, subModule, cause)                                                                           \
    ((module << OSP_ERR_MODULE_OFFSET) | (subModule << OSP_ERR_SUB_MODULE_OFFSET) | (cause << OSP_ERR_EXTRA_OFFSET))

/**
 * @brief   将系统错误码映射到本模块的子模块错误码域中
 * @return  映射后的错误码
 * @warning NA
 * @note    NA
 */
static inline int ospBuildErrnoWithSysErr(unsigned int subModule, unsigned int cause, unsigned int sysErr)
{
    int err = -1;

    if((subModule < OSP_SUB_MODULE_BUTT) && (sysErr < OSP_SYS_ERRNO_MAXN)) {
        err = OSP_ERR_NO(OSP_MODULE_SYSTEM, subModule, cause) + sysErr;
    } else {
        ///< TODO
        // BUGON
    }
    return err;
}

/**
 * @brief   将系统错误码映射到OSP的KERNEL子模块的错误码域中
 * @return  映射后的错误码
 * @warning NA
 * @note    NA
 */
static inline int ospKernelErrnoWithSysErr(int sysNo)
{
    int errNo = 0;

    errNo = ospBuildErrnoWithSysErr(OSP_SUB_MODULE_KERNEL, 0, sysNo);
    return errNo;
}

/**
 * @brief   将系统错误码映射到OSP的FLASH子模块的错误码域中
 * @return  映射后的错误码
 * @warning NA
 * @note    NA
 */
static inline int ospFlashErrnoWithSysErr(int sysNo)
{
    int errNo = 0;

    errNo = ospBuildErrnoWithSysErr(OSP_SUB_MODULE_FLASH_REGION, 0, sysNo);
    return errNo;
}

///< 映射系统错误码：定义错误码时，如果可以选择OspSysErrno_e定义的标准系统错误码，可以使用该宏将系统错误码构建成PS3标准的错误码，
///< 而不用在OspErrno_e中新定义。
#define OSP_SYSNO(type, sysNo) (int)osp##type##ErrnoWithSysErr(sysNo)

#ifdef __cplusplus
}
#endif

#endif
