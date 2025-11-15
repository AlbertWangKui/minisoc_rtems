/*
 *  Time Shell Command Implmentation
 *
 *  Author: Chris Johns <chrisj@rtems.org>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "drv_shell_cmd.h"

extern int drv_shell_execute_cmd(int argc, char **argv);

/* #define PRId32 "d" */
/* #define PRId64 "lld" */
#define __RTEMS_SIZEOF_TIME_T__ 8
/** Helper macro to print "time_t" in decimal */
#if __RTEMS_SIZEOF_TIME_T__ == 8
#define PRIdtime_t PRId64
#elif __RTEMS_SIZEOF_TIME_T__ == 4
#define PRIdtime_t PRId32
#else
#error "PRIdtime_t: unsupported size of time_t"
#endif

/**
 * @brief Obtain the System Uptime
 *
 * This directive returns the system uptime.
 *
 * @param[in] uptime is a pointer to the time structure
 *
 * @retval This method returns RTEMS_SUCCESSFUL if there was not an
 *         error. Otherwise, a status code is returned indicating the
 *         source of the error. If successful, the @a uptime will be
 *         filled in.
 */
extern int rtems_clock_get_uptime(
  struct timespec *uptime
);

static int rtems_shell_main_time(
  int   argc,
  char *argv[]
)
{
  int                errorlevel = 0;
  struct timespec    start;
  struct timespec    end;
  struct timespec    period;
  int                sc;

  argc--;

  sc = rtems_clock_get_uptime(&start);
  if (sc != 0)
    fprintf(stderr, "error: cannot read time\n");

  if (argc) {
    errorlevel = drv_shell_execute_cmd(argc, &argv[1]);
  }

  sc = rtems_clock_get_uptime(&end);
  if (sc != 0)
    fprintf(stderr, "error: cannot read time\n");

  period.tv_sec = end.tv_sec - start.tv_sec;
  period.tv_nsec = end.tv_nsec - start.tv_nsec;
  if (period.tv_nsec < 0)
  {
    --period.tv_sec;
    period.tv_nsec += 1000000000UL;
  }

  fprintf(stderr, "time: %" PRIdtime_t ":%02" PRIdtime_t ":%02" PRIdtime_t ".%03li\n",
         period.tv_sec / 3600,
         period.tv_sec / 60, period.tv_sec % 60,
         period.tv_nsec / 1000000);

  return errorlevel;
}

int timex(void)
{
	int    argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return rtems_shell_main_time(argc, argv);
}

struct drv_shell_cmd dsh_TIME_Command = {
  .name  = "timex",
  .usage = "timex command [arguments...]",
};
