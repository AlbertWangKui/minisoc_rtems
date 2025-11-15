/*
 *  TOP Command Implementation
 *
 *  COPYRIGHT (c) 2014.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <string.h>
#include "drv_shell_cmd.h"

/**
 *   @brief CPU usage Top plugin
 *
 *   Report CPU Usage in top format to
 *   to a print plugin.
 */
void rtems_cpu_usage_top_with_plugin(const rtems_printer *printer);

/**
 *  @brief Reset CPU usage.
 *
 *  CPU Usage Reporter
 */

void rtems_cpu_usage_reset(void);

static int rtems_shell_main_top(int argc, char *argv[])
{
	/*
	 *  When invoked with no arguments, print the report.
	 */
	if (argc == 1) {
		rtems_printer printer;
		rtems_print_printer_fprintf(&printer, stdout);
		rtems_cpu_usage_top_with_plugin(&printer);
		return 0;
	}

	/*
	 *  When invoked with the single argument -r, reset the statistics.
	 */
	if (argc == 2 && !strcmp(argv[1], "-r")) {
		printf("Resetting CPU Usage information\n");
		rtems_cpu_usage_reset();
		return 0;
	}

	/*
	 *  OK.  The user did something wrong.
	 */
	fprintf(stderr, "%s: [-r]\n", argv[0]);
	return -1;
}

int top(void)
{
	int    argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return rtems_shell_main_top(argc, argv);
}

struct drv_shell_cmd dsh_TOP_Command = {
  "top",                                      /* name */
  "[-r] print or reset per thread cpu usage", /* usage */
};