/*
 *  MALLOC_INFO Shell Command Implmentation
 *
 *  COPYRIGHT (c) 1989-2008.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <inttypes.h>
#include <string.h>
#include <stdbool.h>
#include "heapinfo.h"
#include "drv_shell_cmd.h"

/**
 * @brief Malloc walk.
 */
extern bool malloc_walk(int source, bool printf_enabled);

extern void rtems_shell_print_unified_work_area_message(void);

/**
 * @brief Get malloc status information.
 * 
 * Find amount of free heap remaining.
 */
extern int malloc_info(Heap_Information_block *the_info);

extern void rtems_shell_print_heap_info(
  const char             *c,
  const Heap_Information *h
);

extern void rtems_shell_print_heap_stats(
  const Heap_Statistics *s
);

static int rtems_shell_main_malloc_info(
  int   argc,
  char *argv[]
)
{
  if ( argc == 2 && strcmp( argv[ 1 ], "walk" ) == 0 ) {
    malloc_walk( 0, true );
  } else {
    Heap_Information_block info;

    rtems_shell_print_unified_work_area_message();
    malloc_info( &info );
    rtems_shell_print_heap_info( "free", &info.Free );
    rtems_shell_print_heap_info( "used", &info.Used );
    rtems_shell_print_heap_stats( &info.Stats );
  }

  return 0;
}

int mallocinfo(void)
{
	int	   argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return rtems_shell_main_malloc_info(argc, argv);
}

struct drv_shell_cmd dsh_MALLOC_INFO_Command = {
  "mallocinfo",                               /* name */
  "mallocinfo [walk]",                        /* usage */
};