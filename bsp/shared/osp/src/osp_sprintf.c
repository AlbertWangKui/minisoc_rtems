#include <osp_print.h>
#include <stdarg.h>
#include <rtems/bspIo.h>


int ospSprintf(char *str, const char *format, ...)
{
    va_list ap;
    int len;
    va_start(ap, format);
    len = vsprintk(str, format, ap);
    va_end(ap);

    return len;
}
