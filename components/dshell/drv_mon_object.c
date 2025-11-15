#include <stdbool.h>
#include "drv_shell_cmd.h"

typedef enum {
  OBJECTS_CLASSIC_NO_CLASS = 0,

  /* Must be one, see __Thread_Get_objects_information() */
  OBJECTS_RTEMS_TASKS = 1,

  OBJECTS_RTEMS_TIMERS,
  OBJECTS_RTEMS_SEMAPHORES,
  OBJECTS_RTEMS_MESSAGE_QUEUES,
  OBJECTS_RTEMS_PARTITIONS,
  OBJECTS_RTEMS_REGIONS,
  OBJECTS_RTEMS_PORTS,
  OBJECTS_RTEMS_PERIODS,
  OBJECTS_RTEMS_EXTENSIONS,
  OBJECTS_RTEMS_BARRIERS
} Objects_Classic_API;

/** This macro is used to generically specify the last API index. */
#define OBJECTS_RTEMS_CLASSES_LAST OBJECTS_RTEMS_BARRIERS

typedef enum {
    RTEMS_MONITOR_OBJECT_INVALID   =  OBJECTS_CLASSIC_NO_CLASS,
    RTEMS_MONITOR_OBJECT_TASK      =  OBJECTS_RTEMS_TASKS,
    RTEMS_MONITOR_OBJECT_EXTENSION =  OBJECTS_RTEMS_EXTENSIONS,
    RTEMS_MONITOR_OBJECT_QUEUE     =  OBJECTS_RTEMS_MESSAGE_QUEUES,
    RTEMS_MONITOR_OBJECT_SEMAPHORE =  OBJECTS_RTEMS_SEMAPHORES,
    RTEMS_MONITOR_OBJECT_PARTITION =  OBJECTS_RTEMS_PARTITIONS,
    RTEMS_MONITOR_OBJECT_REGION    =  OBJECTS_RTEMS_REGIONS,
    RTEMS_MONITOR_OBJECT_PORT      =  OBJECTS_RTEMS_PORTS,

    /* following monitor objects are not known to RTEMS, but
     * we like to have "types" for them anyway */

    RTEMS_MONITOR_OBJECT_DRIVER    =  OBJECTS_RTEMS_CLASSES_LAST+1,
    RTEMS_MONITOR_OBJECT_DNAME,
    RTEMS_MONITOR_OBJECT_CONFIG,
    RTEMS_MONITOR_OBJECT_INIT_TASK,
    RTEMS_MONITOR_OBJECT_MPCI,
    RTEMS_MONITOR_OBJECT_SYMBOL,
    RTEMS_MONITOR_OBJECT_PTHREAD
} rtems_monitor_object_type_t;

typedef struct {
	int monitor_object;
} rtems_monitor_command_arg_t;

extern void
rtems_monitor_object_cmd(
	int								   argc,
	char **							   argv,
	const rtems_monitor_command_arg_t *command_arg,
	bool							   verbose);

int drv_rtems_monitor_object_cmd(rtems_monitor_object_type_t type)
{
	int							argc;
	char **						argv;
	rtems_monitor_command_arg_t command_arg;
	command_arg.monitor_object = type;

	drv_shell_cmd_load(&argc, &argv);

	rtems_monitor_object_cmd(argc, argv, &command_arg, false);

	return 0;
}

int task(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_TASK);
}

int config(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_CONFIG);
}

int itask(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_INIT_TASK);
}

#if 0
int mpci(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_MPCI);
}
#endif

int queue(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_QUEUE);
}

int sema(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_SEMAPHORE);
}

int region(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_REGION);
}

int part(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_PARTITION);
}

int object(void)
{
	return drv_rtems_monitor_object_cmd(RTEMS_MONITOR_OBJECT_INVALID);
}

#if 0
int config(void)
{
	int							argc;
	char **						argv;
	rtems_monitor_command_arg_t command_arg;
	command_arg.monitor_object = RTEMS_MONITOR_OBJECT_CONFIG;

	drv_shell_cmd_load(&argc, &argv);

	rtems_monitor_object_cmd(argc, argv, &command_arg, false);

	return 0;
}
#endif

/*
 * The top-level commands
 */
struct drv_shell_cmd dsh_CONFIG_Command = {
  "config",                                      /* name */
  "Show the system configuration."               /* usage */
};

struct drv_shell_cmd dsh_ITASK_Command = {
  "itask",                                      /* name */
  "List init tasks for the system."             /* usage */
};

struct drv_shell_cmd dsh_TASK_Command = {
  "task",                                       /* name */
  "Display information about the specified tasks. "
  "Default is to display information about all tasks on this node.\n"
  "  task [id [id ...] ]",                      /* usage */
};

struct drv_shell_cmd dsh_QUEUE_Command = {
  "queue",                                       /* name */
  "Display information about the specified message queues. "
  "Default is to display information about all queues on this node.\n"
  "  queue [id [id ... ] ]",                     /* usage */
};

struct drv_shell_cmd dsh_SEMA_Command = {
  "sema",                                       /* name */
  "sema [id [id ... ] ]\n"                      /* usage */
  "  display information about the specified semaphores\n"
  "  Default is to display information about all semaphores on this node\n",
};

struct drv_shell_cmd dsh_REGION_Command = {
  "region",                                     /* name */
  "region [id [id ... ] ]\n"                    /* usage */
  "  display information about the specified regions\n"
  "  Default is to display information about all regions on this node\n",
};

struct drv_shell_cmd dsh_PART_Command = {
  "part",                                     /* name */
  "part [id [id ... ] ]\n"                    /* usage */
  "  display information about the specified partitions\n"
  "  Default is to display information about all partitions on this node\n"
};

struct drv_shell_cmd dsh_OBJECT_Command = {
  "object",                                     /* name */
  "Display information about specified RTEMS objects. "
  "Object id's must include 'type' information. "
  "(which may normally be defaulted)\n"
  "  object [id [id ...] ]",                    /* usage */
};
