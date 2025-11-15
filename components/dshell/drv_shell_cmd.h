#ifndef _DRV_SHELL_CMD_H
#define _DRV_SHELL_CMD_H

#include <stdio.h>

struct drv_shell_cmd {
  const char    *name;
  const char    *usage;
};

/**
 *  Instructs the compiler that a specific variable is deliberately unused.
 *  This can occur when reading volatile device memory or skipping arguments
 *  in a variable argument method.
 */
#if defined(__GNUC__)
  #define RTEMS_UNUSED __attribute__((__unused__))
#else
  #define RTEMS_UNUSED
#endif

extern void drv_shell_cmd_load(int *argc, char ***argv);

/**
 * Type definition for function which can be plugged in to certain reporting
 * routines to redirect the output.
 *
 * Use the RTEMS Print interface to call these functions. Do not directly use
 * them.
 *
 * If the user provides their own printer, then they may redirect those reports
 * as they see fit.
 */
typedef int (*rtems_print_printer)(void *, const char *format, va_list ap);

/**
 * Type definition for the printer structure used to access the kernel print
 * support.
 */
struct rtems_printer {
	void	       *context;
	rtems_print_printer printer;
};

typedef struct rtems_printer rtems_printer;

/**
 * @brief Initializes the printer to print via fprintf() using the specified
 * file stream.
 *
 * @param[in] printer Pointer to the printer structure.
 */
extern void rtems_print_printer_fprintf(rtems_printer *printer, FILE *file);

/**
 * @brief Initializes the printer to print via printf().
 *
 * @param[in] printer Pointer to the printer structure.
 */
void rtems_print_printer_printf(rtems_printer *printer);

#endif