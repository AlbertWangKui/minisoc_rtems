/**
 * Copyright (C), 2021, Starsmicrosystem Technologies Co., Ltd.
 *
 * @file      cli_printf.h
 * @author    xiezhj
 * @date      2021.04.20
 * @brief     文件说明
 * @note
 */

#ifndef __CLI_PRINTF_H__
#define __CLI_PRINTF_H__

#define CLI_PRINT_BUFFER_LEN (1024)
#define CLI_PRINT_BUFFER_LEN_WITHOUT_TAIL (CLI_PRINT_BUFFER_LEN - (1))

///< 定义颜色
#define COLOR_NONE         "\033[m"
#define COLOR_RED          "\033[0;32;31m"
#define COLOR_LIGHT_RED    "\033[1;31m"
#define COLOR_GREEN        "\033[0;32;32m"
#define COLOR_LIGHT_GREEN  "\033[1;32m"
#define COLOR_BLUE         "\033[0;32;34m"
#define COLOR_LIGHT_BLUE   "\033[1;34m"
#define COLOR_DARY_GRAY    "\033[1;30m"
#define COLOR_CYAN         "\033[0;36m"
#define COLOR_LIGHT_CYAN   "\033[1;36;43m"
#define COLOR_PURPLE       "\033[0;35m"
#define COLOR_LIGHT_PURPLE "\033[1;35m"
#define COLOR_BROWN        "\033[0;33m"
#define COLOR_YELLOW       "\033[1;33m"
#define COLOR_LIGHT_GRAY   "\033[0;37m"
#define COLOR_WHITE        "\033[1;37m"

typedef enum CliColor {
    CLI_COLOR_NONE,
    CLI_COLOR_RED,
    CLI_COLOR_LIGHT_RED,
    CLI_COLOR_GREEN,
    CLI_COLOR_LIGHT_GREEN,
    CLI_COLOR_BLUE,
    CLI_COLOR_LIGHT_BLUE,
    CLI_COLOR_DARY_GRAY,
    CLI_COLOR_CYAN,
    CLI_COLOR_LIGHT_CYAN,
    CLI_COLOR_PURPLE,
    CLI_COLOR_LIGHT_PURPLE,
    CLI_COLOR_BROWN,
    CLI_COLOR_YELLOW,
    CLI_COLOR_LIGHT_GRAY,
    CLI_COLOR_WHITE,

    CLI_COLOR_TITLE   = CLI_COLOR_LIGHT_GREEN,
    CLI_COLOR_CONTENT = CLI_COLOR_LIGHT_RED,
    CLI_COLOR_KEY     = CLI_COLOR_CYAN,
    CLI_COLOR_VALUE   = CLI_COLOR_PURPLE,
    CLI_COLOR_ERROR   = CLI_COLOR_RED,
    CLI_COLOR_TABLE   = CLI_COLOR_LIGHT_BLUE,
    CLI_COLOR_HELP    = CLI_COLOR_BROWN,
}CliColor_e;

/**
 * @brief   根据不同session打印
 * @param   pSession    [in] session 结构体
 * @param   format      [in] 打印的格式
 * @return  void
 */
void cliSessionPrintf(U32 sessionId, S8 *format, ...);

#endif ///< __CLI_PRINTF_H__
