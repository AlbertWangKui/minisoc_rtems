/*
 *
 *  Shell Help Command
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "drv_shell_cmd.h"

extern struct drv_shell_cmd dsh_TIME_Command;
extern struct drv_shell_cmd dsh_CPUUSE_Command;
extern struct drv_shell_cmd dsh_MALLOC_INFO_Command;
extern struct drv_shell_cmd dsh_IRQ_command;
extern struct drv_shell_cmd dsh_CONFIG_Command;
extern struct drv_shell_cmd dsh_ITASK_Command;
extern struct drv_shell_cmd dsh_TASK_Command;
extern struct drv_shell_cmd dsh_QUEUE_Command;
extern struct drv_shell_cmd dsh_SEMA_Command;
extern struct drv_shell_cmd dsh_REGION_Command;
extern struct drv_shell_cmd dsh_PART_Command;
extern struct drv_shell_cmd dsh_OBJECT_Command;
extern struct drv_shell_cmd dsh_CALL_Command;
extern struct drv_shell_cmd dsh_SHUTDOWN_Command;

struct drv_shell_cmd dsh_HELP_Command  =  {
  .name = "h",
  .usage = "list of usage of commands",
};

struct drv_shell_cmd *dsh_cmds[] = {
  &dsh_HELP_Command,
  &dsh_TIME_Command,
  &dsh_CPUUSE_Command,
  &dsh_MALLOC_INFO_Command,
  &dsh_IRQ_command,
  &dsh_CONFIG_Command,
  &dsh_ITASK_Command,
  &dsh_TASK_Command,
  &dsh_QUEUE_Command,
  &dsh_SEMA_Command,
  &dsh_REGION_Command,
  &dsh_PART_Command,
  &dsh_OBJECT_Command,
  &dsh_CALL_Command,
  &dsh_SHUTDOWN_Command,
};

int dsh_cmds_count = sizeof(dsh_cmds) / sizeof(dsh_cmds[0]);

/*
 * show the help for one command.
 */
static int rtems_shell_help_cmd(
  const struct drv_shell_cmd *shell_cmd
)
{
  const char * pc;
  int    col,line;

  printf("%-12.12s - ",shell_cmd->name);
  col = 14;
  line = 1;
  if (shell_cmd->usage) {
    pc = shell_cmd->usage;
    while (*pc) {
      switch(*pc) {
        case '\r':
          break;
        case '\n':
          putchar('\n');
          col = 0;
          break;
        default:
          putchar(*pc);
          col++;
          break;
      }
      pc++;
      if (col>78) { /* What daring... 78?*/
        if (*pc) {
          putchar('\n');
          col = 0;
        }
      }
      if (!col && *pc) {
        printf("            ");
        col = 12;line++;
      }
    }
  }
  puts("");
  return line;
}

/*
 * show the help. The first command implemented.
 * Can you see the header of routine? Known?
 * The same with all the commands....
 */
static int rtems_shell_help(
  int argc __attribute__((__unused__)), 
  char * argv[] __attribute__((__unused__))
)
{
  int line = 0;
  int lines = 16;
  int i;

  printf("available shell command list\n");
  line++;

  for (i = 0; i < dsh_cmds_count; i++) {
    line+= rtems_shell_help_cmd(dsh_cmds[i]);
    if (lines && (line > lines)) {
      printf("Press any key to continue...");
      getchar();
      printf("\n");
      line = 0;
    }
  }

  puts("");
  return 0;
}

int h(void)
{
	int    argc;
	char **argv;

	drv_shell_cmd_load(&argc, &argv);

	return rtems_shell_help(argc, argv);
}
