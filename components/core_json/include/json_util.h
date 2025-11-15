/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file fdt_util.h
 * @author taohb@starsmicrosystem.com
 * @date 2025.03.26
 * @brief fdt uitls.
 */

#ifndef __JSON_UTIL_H__
#define __JSON_UTIL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "core_json.h"

typedef struct __json_node_info {
    const char *name;
    bool has_id;
    uint32_t id;
    /* fdt_propery_info propery; */
    char *node_ptr;
} json_node_info;

typedef struct __json_node_search_info {
    void *json; /* start address of json */
    uint32_t json_size; /* size of json */
    char *parent_path; /* parent node path */
    uint32_t max_matched_node_num; /* the max number of of matched nodes */
} json_node_search_info;

/**
 * @brief get root address of config script
 * @details null
 * @param [in] none
 * @param [out] none
 * @return root address of fdt.
 */
void *confscript_root_get(void);

/**
 * @brief get root size of conf script
 * @details null
 * @param [in] none
 * @param [out] none
 * @return the size of json.
 */
uint32_t confscript_size_get(void);

/**
 * @brief get root address of fdt
 * @details null
 * @param [in] search_info search information.
 * @param [inout] matched_node_offset the node list of matched.
 * @param [inout] matched_node_number the number of matched nodes.
 * @return null.
 */
int32_t json_search_all_nodes_by_property_string(json_node_search_info *search_info, json_node_info *matched_node,
    uint32_t *matched_node_number);

/**
 * @brief sort nodes by id
 * @details null
 * @param [inout] node the node list to be sorted.
 * @param [in] node_number the number of nodes.
 * @param [in] is_ascend true is ascend, false is descend.
 * @return null.
 */
void sort_nodes_by_id(json_node_info *node, uint32_t node_number, bool is_ascend);

/**
 * @brief  get json node key name("keyname" : "value")
 * @details 
 * @param [in] node
 * @param [out] outname outname buffer
 * @param [in] outlenmax outname buf length
 */
void json_get_key_name(json_node_info *node, char *outname, uint32_t outlenmax);

/**
 * @brief get unsigned int value from json file by key name.
 * @details null
 * @param [in] node_ptr the node list to be sorted.
 * @param [in] key the key's name.
 * @param [out] out unsigned int value from json file by key name.
 * @return null.
 */
void json_get_value_u32(void *node_ptr, char *key, uint32_t *out);
void json_get_value_string(void *node_ptr, char *key, char *out,uint32_t buf_size);

/**
 * @brief get unsigned int value array from json file by key name.
 * @details null
 * @param [in] node_ptr the node list to be sorted.
 * @param [in] key the key's name.
 * @param [in] max_array_len max array len.
 * @param [out] array_len array len.
 * @param [out] out unsigned int value array from json file by key name.
 * @return null.
 */
void json_getprpop_value_u32_array(void *node_ptr, char *key, uint32_t max_array_len, uint32_t *array_len, uint32_t *out);
#endif

