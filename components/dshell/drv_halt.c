/*
 *  Halt Command Implementation
 *
 *  COPYRIGHT (c) 1989-2008.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <stdlib.h>
#include "drv_shell_cmd.h"

static int rtems_shell_main_shutdown(
  int   argc RTEMS_UNUSED,
  char *argv[] RTEMS_UNUSED
)
{
  fprintf(stdout, "System shutting down at user request\n");
  exit(0);
  return 0;
}

__attribute__((weak)) int shutdown(void)
{
	int    argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return rtems_shell_main_shutdown(argc, argv);
}

struct drv_shell_cmd dsh_SHUTDOWN_Command = {
  "shutdown",                                /* name */
  "shutdown",                                /* usage */
};
