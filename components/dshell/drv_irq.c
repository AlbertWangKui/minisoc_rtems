/**
 * @file
 *
 * @ingroup bsp_interrupt
 *
 * @brief Generic BSP interrupt shell implementation.
 */

/*
 * Copyright (c) 2009
 * embedded brains GmbH
 * Obere Lagerstr. 30
 * D-82178 Puchheim
 * Germany
 * <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include "drv_shell_cmd.h"

/**
 * @brief Prints interrupt information via the printk plugin @a print with the
 * context @a context.
 */
void bsp_interrupt_report_with_plugin(
  const rtems_printer *printer
);

static int bsp_interrupt_shell_main(int argc __attribute__((__unused__)),
                                    char **argv __attribute__((__unused__)))
{
  rtems_printer printer;
  rtems_print_printer_printf(&printer);
  bsp_interrupt_report_with_plugin(&printer);

  return 0;
}

int irq(void)
{
	int    argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return bsp_interrupt_shell_main(argc, argv);
}

struct drv_shell_cmd dsh_IRQ_command = {
  .name    = "irq",
  .usage   = "Prints interrupt information",
};
