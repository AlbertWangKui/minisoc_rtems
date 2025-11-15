/*
 *  stackuse Command Implementation
 *
 *  COPYRIGHT (c) 1989-2008.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include "drv_shell_cmd.h"

/**
 * @brief Print the stack usage report using caller's routine.
 *
 * This method prints a stack usage report for the curently executing
 * task.
 *
 * @param[in] context is the context to pass to the print handler
 * @param[in] print is the print handler
 *
 * @note It uses the caller's routine to print the report.
 */
void rtems_stack_checker_report_usage_with_plugin(const rtems_printer *printer);

static int rtems_shell_main_stackuse(void/*int argc, char *argv[]*/)
{
	rtems_printer printer;
	rtems_print_printer_printf(&printer);
	rtems_stack_checker_report_usage_with_plugin(&printer);
	return 0;
}

int stackuse(void)
{
	int    argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return rtems_shell_main_stackuse(/*argc, argv*/);
}

struct drv_shell_cmd dsh_STACKUSE_Command = {
  "stackuse",                                 /* name */
  "print per thread stack usage",             /* usage */
};