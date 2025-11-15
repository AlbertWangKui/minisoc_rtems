/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_appinit.h
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   提供初始化入口(APP使用)
 * @note    NA
 */
#ifndef __OSP_APPINIT_H
#define __OSP_APPINIT_H

#include <inner/osp_inner_linkersets.h>

#ifdef __cplusplus
extern "C" {
#endif

///< 高优先级应该预留给os使用，普通APP使用MIDDLE 或者 LOW 或者 LAST
#define OSP_APPINIT_HIGH   000100
#define OSP_APPINIT_MIDDLE 000140
#define OSP_APPINIT_LOW    000180
#define OSP_APPINIT_LAST   ffffff

#define OSP_APPINIT_ORDER_FIRST   00
#define OSP_APPINIT_ORDER_SECOND  01
#define OSP_APPINIT_ORDER_THIRD   02
#define OSP_APPINIT_ORDER_FOURTH  03
#define OSP_APPINIT_ORDER_FIFTH   04
#define OSP_APPINIT_ORDER_SIXTH   05
#define OSP_APPINIT_ORDER_SEVENTH 06
#define OSP_APPINIT_ORDER_EIGHTH  07
#define OSP_APPINIT_ORDER_NINETH  08
#define OSP_APPINIT_ORDER_TENTH   09
#define OSP_APPINIT_ORDER_LAST    ff

typedef void (*osp_appinit_handler)(void);

typedef struct {
    osp_appinit_handler handler;
} osp_appinit_item;

#define OSP_LINKER_ROSET(set, type) OSP_INNER_LINKER_ROSET(set, type)

///< The enum helps to detect typos in the module and order parameters
#define _OSP_APPINIT_INDEX_ITEM(handler, index)                                                                        \
    enum { _Appinit_##handler = index };                                                                               \
    OSP_LINKER_ROSET_ITEM_ORDERED(_Appinit, osp_appinit_item, handler, index) = {handler}

///< Create index from module and order
#define _OSP_APPINIT_ITEM(handler, module, order) _OSP_APPINIT_INDEX_ITEM(handler, 0x##module##order)

///< Perform parameter expansion
#define OSP_APPINIT_ITEM(handler, module, order) _OSP_APPINIT_ITEM(handler, module, order)

#ifdef __cplusplus
}
#endif

#endif
