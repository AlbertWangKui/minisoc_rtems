
/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_asan.c
 * @author  lichenxiang
 * @date    2021.11.15
 * @brief
 * @note
 */
#ifdef CFG_ASAN_SHADOW_OFFSET

#include <osp_asan.h>
#include <stdio.h>


void ospAsanTagAccess(const void *begin, const void *end)
{
    asan_tag_access(begin,end);

}

static bool _ospRedZoneSupported(unsigned char  value)
{
    if (value == KASAN_GLOBAL_REDZONE ||
            value == KASAN_KMALLOC_REDZONE ||
            value == KASAN_KMALLOC_FREE ||
            value == KASAN_PARTITION_FREE ||
            value == KASAN_PARTITION_REDZONE ||
            value == KASAN_STACK_LEFT ||
            value == KASAN_STACK_MID ||
            value == KASAN_STACK_RIGHT ||
            value == KASAN_STACK_PARTIAL ||
            value == KASAN_USE_AFTER_SCOPE )
    {
        return false;
    }
    return true;
}

void ospAsanTagNoAccessWithRedzone(const void *begin, const void *end,unsigned char  value)
{
    if (!_ospRedZoneSupported(value))
    {
        printf("Warning : osp interface may not use OS reserved redZone [0x%x] !!!!!\n",value);
    }

    asan_tag_no_access_with_readzone(begin,end,value);
}

bool ospAsanCheckAccessRange(void* addr, size_t size,bool isWrite)
{

    return check_access_range(addr,size,isWrite);
}

void ospAsanPanicOn()
{
    asan_panic_on();

}

void ospAsanPanicOff()
{
    asan_panic_off();
}


#endif
