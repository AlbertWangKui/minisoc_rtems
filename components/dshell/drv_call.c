#include <stdio.h>
#include "drv_shell.h"
#include "drv_shell_cmd.h"

int call(uintptr_t addr /*argv0*/,
	 int argv1, int argv2, int argv3, int argv4,
	 int argv5, int argv6, int argv7, int argv8,
	 int argv9, int argv10, int argv11, int argv12,
	 int argv13, int argv14, int argv15, int argv16)
{
	PFUNC pfunc = (PFUNC)addr;

	if (pfunc == NULL) {
		printf("illegal addr\n");
		return -1;
	}

	return pfunc(argv1, argv2, argv3, argv4,
		     argv5, argv6, argv7, argv8,
		     argv9, argv10, argv11, argv12,
		     argv13, argv14, argv15, argv16);
}

struct drv_shell_cmd dsh_CALL_Command = {
  "call",              /* name */
  "Call a symbol",     /* usage */
};
