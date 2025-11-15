#ifndef _UNIFY_UDELAY_H__
#define _UNIFY_UDELAY_H__

#include <stdint.h>
__attribute__((weak)) void minisoc_port_udelay(uint32_t usecs);

#endif // _UNIFY_UDELAY_H