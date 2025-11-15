/*
 * header for drv_shell.c
 */
#include <stdio.h>
#include <stdbool.h>

#ifndef _DRV_SHELL_H
#define _DRV_SHELL_H

#define DRV_SHELL_MAX_ARGV	      17 /* 0 for func name */
#define RTEMS_SHELL_MAXIMUM_ARGUMENTS (128)
#define RTEMS_SHELL_CMD_SIZE		  (256)
#define RTEMS_SHELL_CMD_COUNT		  (32)
#define RTEMS_SHELL_CMD_SIZE_COUNT	  RTEMS_SHELL_CMD_SIZE* RTEMS_SHELL_CMD_COUNT

/*
 * Some key labels to define special keys.
 */
#define RTEMS_SHELL_KEYS_EXTENDED    (0x8000)
#define RTEMS_SHELL_KEYS_NORMAL_MASK (0x00ff)
#define RTEMS_SHELL_KEYS_INS         (0)
#define RTEMS_SHELL_KEYS_DEL         (1)
#define RTEMS_SHELL_KEYS_UARROW      (2)
#define RTEMS_SHELL_KEYS_DARROW      (3)
#define RTEMS_SHELL_KEYS_LARROW      (4)
#define RTEMS_SHELL_KEYS_RARROW      (5)
#define RTEMS_SHELL_KEYS_HOME        (6)
#define RTEMS_SHELL_KEYS_END         (7)
#define RTEMS_SHELL_KEYS_F1          (8)
#define RTEMS_SHELL_KEYS_F2          (9)
#define RTEMS_SHELL_KEYS_F3          (10)
#define RTEMS_SHELL_KEYS_F4          (11)
#define RTEMS_SHELL_KEYS_F5          (12)
#define RTEMS_SHELL_KEYS_F6          (13)
#define RTEMS_SHELL_KEYS_F7          (14)
#define RTEMS_SHELL_KEYS_F8          (15)
#define RTEMS_SHELL_KEYS_F9          (16)
#define RTEMS_SHELL_KEYS_F10         (17)

/** This mode constant is used to indicate preemption is enabled. */
#define RTEMS_PREEMPT      0x00000000
/** This mode constant is used to indicate preemption is disabled. */
#define RTEMS_NO_PREEMPT   0x00000100

/** This mode constant is used to indicate timeslicing is disabled. */
#define RTEMS_NO_TIMESLICE 0x00000000
/** This mode constant is used to indicate timeslicing is enabled. */
#define RTEMS_TIMESLICE    0x00000200

/** This mode constant is used to indicate signal processing is enabled. */
#define RTEMS_ASR          0x00000000
/** This mode constant is used to indicate signal processing is disabled. */
#define RTEMS_NO_ASR       0x00000400


#define RTEMS_MINIMUM_STACK_SIZE (1024 * 4)

typedef int (*OSHLL_EXEC_CMD)(const char *cmd, int argc, char  *argv[]);

typedef int (*DSHLL_EXEC_CMD)(int argc, char  *argv[]);

typedef int (*PFUNC)(int arg0, int arg1, int arg2, int arg3,
		     int arg4, int arg5, int arg6, int arg7,
		     int arg8, int arg9, int arg10, int arg11,
		     int arg12, int arg13, int arg14, int arg15);

extern unsigned int drv_rtems_shell_getchar(FILE *in);

extern int drv_rtems_shell_make_args(
  char  *commandLine,
  int   *argc_p,
  char **argv_p,
  int    max_args
);

extern int rtems_shell_execute_cmd(
  const char *cmd, int argc, char *argv[]
);

#if __has_attribute(__fallthrough__)
# define fallthrough                    __attribute__((__fallthrough__))
#else
# define fallthrough                    do {} while (0)  /* fallthrough */
#endif

int driver_shell_init(bool dshell_task);

int ushellCmdRegister(const char *name, void (*func)(int, char**));
int ushellCmdUnRegister(const char *name);
int ushellCmdExec(const char *name, int argc, char **argv);
void userCmdsList();
void r5ToolEntry(int argc, char **argv);

#endif /* _DRV_SHELL_H */