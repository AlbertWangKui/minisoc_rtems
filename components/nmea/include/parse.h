/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: parse.h 4 2007-08-27 13:11:03Z xtimor $
 *
 */

#ifndef __NMEA_PARSE_H__
#define __NMEA_PARSE_H__

#include "sentence.h"

#ifdef  __cplusplus
extern "C" {
#endif

/************************************************************************/
/* modified by SYAKYOU, Fri Jun  4 10:03:09 CST 2021                    */
/************************************************************************/
typedef int (*pfn_nmea_parse)(const char *buff, int buff_sz, void *pack, const char* phead);
typedef void (*pfn_nmea_info)(void *pack, nmeaINFO *info);

typedef struct {
  const char* phead;
  int ptype;
  int pack_size;
  pfn_nmea_parse pfn_parse;
  pfn_nmea_info pfn_info;
} nmea_lookup_t;

nmea_lookup_t* nmea_lookuptab(const char *buff, int buff_sz);

int nmea_find_tail(const char *buff, int buff_sz, int *res_crc);

int nmea_parse_ZDA(const char *buff, int buff_sz, void *mem, const char* phead);

#ifdef  __cplusplus
}
#endif

#endif /* __NMEA_PARSE_H__ */
