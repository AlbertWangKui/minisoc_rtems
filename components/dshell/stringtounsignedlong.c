/**
 * @file
 *
 * @ingroup libmisc_conv_help Conversion Helpers
 *
 * @brief Convert String to Unsigned Long Long (with validation)
 */

/*
 *  COPYRIGHT (c) 2009.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  Copyright (c) 2011  Ralf Corsépius, Ulm, Germany.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */


#include <errno.h>
#include <stdlib.h>
#include <limits.h>
#include "osp_status.h"

/*
 *  Instantiate an error checking wrapper for strtoul (unsigned long)
 */

OspStatusCode_e drv_rtems_string_to_unsigned_long (
  const char *s,
  unsigned long *n,
  char **endptr,
  int base
)
{
  unsigned long result;
  char *end;

  if ( !n )
    return OSP_INVALID_ADDRESS;

  errno = 0;
  *n = 0;

  result = strtoul( s, &end, base );

  if ( endptr )
    *endptr = end;

  if ( end == s )
    return OSP_NOT_DEFINED;

  if ( ( errno == ERANGE ) &&
    (( result == 0 ) || ( result == ULONG_MAX )))
      return OSP_INVALID_NUMBER;

  *n = result;

  return OSP_SUCCESSFUL;
}
