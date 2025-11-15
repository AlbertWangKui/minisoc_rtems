#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <termios.h>
#include <unistd.h>
#include "stringto.h"
#include "osp_mutex.h"
#include "osp_common.h"
#include "osp_task.h"
#include "osp_attr.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "log_msg.h"
#include "drv_shell_inner.h"
#include "drv_shell.h"
#include "drv_symbols.h"
#include "drv_regcmds.h"

#define DRV_SHELL_MAX_PROMPT_LEN 24
#define AUTO_INFO_SESSION_SIZE   200

extern struct drv_shell_cmd g_drv_cmd_tbl[];
extern unsigned long		g_drv_cmd_tbl_size;

extern int mem_main_entry(int argc, char **argv);
int drv_shell_execute_cmd(int argc, char **argv);
extern void uartPrintkWrite(S8 c);

static char g_prompt[DRV_SHELL_MAX_PROMPT_LEN] = "[dshell]# ";
static char			 g_cmds_buf[RTEMS_SHELL_CMD_SIZE_COUNT];
static char			 g_cmd_argv_buf[RTEMS_SHELL_CMD_SIZE];
static int			 g_argc;
static uintptr_t	 g_argv;
static OSHLL_EXEC_CMD 		outer_shell_exec_cmd;
static DSHLL_EXEC_CMD 		dshell_exec_cmd = drv_shell_execute_cmd;

void drv_shell_reg_outer_exec_cmd(OSHLL_EXEC_CMD shll_exec_cmd)
{
	if (shll_exec_cmd == NULL) {
		printf("illegal shll_exec_cmd\n");
		return;
	}

	outer_shell_exec_cmd = shll_exec_cmd;
}

void drv_shell_unreg_outer_exec_cmd(void)
{
	outer_shell_exec_cmd = NULL;
}

void drv_shell_reg_custom_exec_cmd(DSHLL_EXEC_CMD shll_exec_cmd)
{
	if (shll_exec_cmd == NULL) {
		printf("illegal shll_exec_cmd\n");
		return;
	}

	dshell_exec_cmd = shll_exec_cmd;
}

void drv_shell_unreg_custom_exec_cmd(void)
{
	dshell_exec_cmd = drv_shell_execute_cmd;
}

static void drv_shell_cmd_store(int argc, char **argv)
{
	g_argc = argc;
	g_argv = (uintptr_t)argv;
}

void drv_shell_cmd_load(int *argc,char ***argv)
{
	*argc = g_argc;
	*argv = (char **)g_argv;
}

void drv_shell_cmd_tbl_show(void)
{
	unsigned int i;

	for (i = 0; i < g_drv_cmd_tbl_size; i++) {
		printf("%d %p %s\n", i, g_drv_cmd_tbl[i].addr, g_drv_cmd_tbl[i].name);
	}
}

int drv_shell_set_prompt(const char *prompt)
{
	int len = strlen(prompt);

	if (len > DRV_SHELL_MAX_PROMPT_LEN - 1) {
		printf("prompt too long\n");
		return -1;
	}

	memcpy(g_prompt, prompt, len);
	g_prompt[len] = '\0';

	return 0;
}

static int drv_shell_cmd_compare(const void *data1, const void *data2)
{
	struct drv_shell_cmd *cmd1 = (struct drv_shell_cmd *)data1;
	struct drv_shell_cmd *cmd2 = (struct drv_shell_cmd *)data2;

	return strcmp(cmd1->name, cmd2->name);
}

int drv_shell_execute_cmd(int argc, char **argv)
{
	int		      input[DRV_SHELL_MAX_ARGV] = { 0 };
	int					  array_size				= g_drv_cmd_tbl_size;
	struct drv_shell_cmd  key;
	struct drv_shell_cmd *match;
	int		     *value;
	PFUNC		      pfunc = NULL;
	OspStatusCode_e	      status;
	int		      ret = 0;

	if (argc > DRV_SHELL_MAX_ARGV) {
		printf("argc illlegal %d \n", DRV_SHELL_MAX_ARGV - 1);
		return -1;
	}

	key.name = argv[0];
	match	 = bsearch(&key, g_drv_cmd_tbl, array_size, sizeof(struct drv_shell_cmd), drv_shell_cmd_compare);
	if (match == NULL) {
		if (outer_shell_exec_cmd != NULL) {
			return outer_shell_exec_cmd(argv[0], argc, argv);
		} else {
			printf("%s: command not found\n", argv[0]);
		}
		return -1;
	}

	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			if (*(&argv[i][0] - 1) == '"') {
				input[i - 1] = (intptr_t)argv[i];
				continue;
			}

			status = drv_rtems_string_to_int(argv[i], &input[i - 1], NULL, 0);
			if (status == OSP_INVALID_NUMBER) {
				if (drv_rtems_string_to_unsigned_long(argv[i], (unsigned long *)&input[i - 1], NULL, 0)) {
					printf("Length argument (%s) is not a number\n", argv[i]);
					return -1;
				}
			}
		}
	}

	switch (match->type) {
		case SYM_BSS:
		case SYM_DATA:
		case SYM_COMM:
			value = (int *)match->addr;
			if (argc == 1) {
				printf("%s = %p: value = %d = 0x%x\n",
				       match->name, match->addr, *value, *value);
				return 0;
			} else if (argc == 3 && strcmp(argv[1], "=") == 0) {
				*value = input[1];
				printf("%s = %p: value = %d = 0x%x\n",
				       match->name, match->addr, *value, *value);
				return 0;
			} else {
				printf("undefined operation\n");
				return -1;
			}

		case SYM_TEXT:
			drv_shell_cmd_store(argc, argv);

			pfunc = (PFUNC)match->addr;
			ret   = pfunc(input[0], input[1], input[2], input[3],
				      input[4], input[5], input[6], input[7],
				      input[8], input[9], input[10], input[11],
				      input[12], input[13], input[14], input[15]);

			printf("value = %d = 0x%x\n", ret, ret);
			return 0;

		default:
			printf("undefined symbol type(%d)\n", match->type);
			break;
	}

	return -1;
}

static void drv_shell_cmd_init(void)
{
	/* 借助编译服务器强大的计算能力，直接在编译时完成排序，缩短shell启动时间 */
#if 0
	qsort(g_drv_cmd_tbl, g_drv_cmd_tbl_size, sizeof(struct drv_shell_cmd), drv_shell_cmd_compare);
#endif
	drvRegcmdsInit();
	printf("driver shell cmd init complete, %lu cmd add to shell\n", g_drv_cmd_tbl_size);
}

static int drv_shell_str_common_len(const char *str1, const char *str2)
{
	const char *str = str1;
	while ((*str != 0) && (*str2 != 0) && (*str == *str2)) {
		str ++;
		str2 ++;
	}

	return (str - str1);
}

static unsigned int drv_shell_max_width_find(const struct drv_shell_cmd *begin,
		const struct drv_shell_cmd *end)
{
	unsigned int current_width, max_width = 0;

	if (!begin || !end)
		return 0;

	for (; begin <= end; begin++) {
		current_width = strlen(begin->name);
		max_width = (current_width > max_width) ? current_width : max_width;
	}

	return max_width;
}

static inline int drv_shell_common_beginning_find(const struct drv_shell_cmd *begin,
		const struct drv_shell_cmd *end)
{
	if (!begin || !end)
		return 0;

	return drv_shell_str_common_len(begin->name, end->name);
}

#define SEARCH_BEGIN 1
#define SEARCH_END 0

static struct drv_shell_cmd *drv_shell_command_bsearch_bound(const char *key,
		const struct drv_shell_cmd *base, size_t num, int isbegin)
{
	const struct drv_shell_cmd *mid;
	int result, key_len;

	if (!key)
		return NULL;

	if (!base || !num)
		return NULL;

	key_len = strlen(key);
	while (num > 0) {
		mid = base + (num >> 1);
		result = strncmp(key, mid->name, key_len);

		if (result == 0) {
			/* 找到相同元素 判断是否查找完成 */
			if (isbegin) {
				/*
				 * 如果剩下一个元素并且相等，那么一定是左边界
				 * 否则在比较右边元素是否相等，如果不等表示找到边界
				 */
				if (num == 1 || strncmp(key, (mid - 1)->name, key_len))
					return (struct drv_shell_cmd *)mid;

				result = -1;
			} else {
				/*
				 * 如果剩下一个或者俩个元素并且相等，那么一定是右边界
				 * 否则在比较右边元素是否相等，如果不等表示找到边界
				 */
				if (num == 1 || num == 2 || strncmp(key, (mid + 1)->name, key_len))
					return (struct drv_shell_cmd *)mid;

				result = 1;
			}
		}

		/**
		 * 向右查找
		 *
		 * base                            mid             key         end   num = 16
		 * |                               |               |           |
		 * 01  02  03  03  03  06  07  08  09  10  12  12  12  14  15  16
		 *                                     |                       |
		 *                                  new base                   end   num = 7;
		 *
		 * base                            mid             key             end   num = 17
		 * |                               |               |               |
		 * 01  02  03  03  03  06  07  08  09  10  12  12  12  14  15  16  17
		 *                                     |                           |
		 *                                  new base                       end   num = 8;
		 *
		 * 向左查找
		 *
		 * base                            mid                         end   num = 16
		 * |                               |                            |
		 * 01  02  03  03  03  06  07  08  09  10  12  12  12  14  15  16
		 *             |               |                                |
		 *             key             new end                         end   num = 8;
		 *
		 * base                            mid             key            end   num = 17
		 * |                               |               |               |
		 * 01  02  03  03  03  06  07  08  09  10  12  12  12  14  15  16  17
		 *             |               |                                   |
		 *             key             new end                            end   num = 8;
		 */
		if (result > 0) {
			base = mid + 1;
			num = (num - 1) >> 1;
		} else if (result < 0) {
			base = base;
			num = num >> 1;
		}
	}

	return NULL;
}

static void drv_shell_display_matches(const struct drv_shell_cmd *begin,
		const struct drv_shell_cmd *end, int max_len)
{
	int screen_width = 90;
	int space = 2;
	int i = 0;
	int cols;
	int temp;
	int len;

	if (!begin || !end) {
		return;
	}

	cols = (screen_width + space) / (max_len + space);

	if (cols > 1)
		temp = (screen_width - max_len * cols) / (cols - 1);
	else
		temp = 0;

	if (space < temp)
		space = temp;
#if 0
	for (; begin <= end; begin++)  {
		printf("%-*s", max_len + space, begin->name);

		i++;
		if (i >= cols || begin == end) {
			i = 0;
			printf("\n");
		}
	}
#else
	for (i = 1; begin <= end; begin++, i++) {
		len = printf("%s", begin->name);

		if (i < cols)
			printf("%*s", max_len + space - len, "");
		else {
			i = 0;
			printf("\n");
		}
	}

	if (i > 1)
		printf("\n");
#endif

	return;
}

static void drv_rtems_shell_complete(
		const char *prompt,
		char *      line)
{
	uint32_t common_len, max_len;
	const char *common_ptr;
	struct drv_shell_cmd *begin, *end;

	if (*line == '\0')
		return;

	common_len = 0;
	common_ptr = NULL;

	begin = drv_shell_command_bsearch_bound(line, g_drv_cmd_tbl, g_drv_cmd_tbl_size, SEARCH_BEGIN);
	if(!begin)
		return;

	end = drv_shell_command_bsearch_bound(line, g_drv_cmd_tbl, g_drv_cmd_tbl_size, SEARCH_END);
	if(!end)
		return;

	common_len = drv_shell_common_beginning_find(begin, end);
	common_ptr = begin->name;

	max_len = drv_shell_max_width_find(begin, end);

	/* auto complete string */
	if (common_ptr != NULL) {
		printf("\n");

		drv_shell_display_matches(begin, end, max_len);

		strncpy(line, common_ptr, common_len);
		line[common_len] = '\0';

		printf("%s%s", prompt, line);
	}
}

/*
 *  Get a line of user input with modest features
 */
static int drv_rtems_shell_line_editor(
	char *		cmds[],
	int			count,
	int			size,
	const char *prompt,
	FILE *		in,
	FILE *		out)
{
	unsigned int extended_key;
    int          c, i;
	char         str;
	int			 col;
	int			 last_col;
	int			 output;
	char		 line[size];
	char		 new_line[size];
	int			 up;
	int			 cmd		= -1;
	int			 inserting	= 1;
	int			 in_fileno	= fileno(in);
	int			 out_fileno = fileno(out);

	/*
   * Only this task can use this file descriptor because calling
   * fileno will block if another thread call made a call on this
   * descriptor.
   */
	output = (out && isatty(in_fileno));

	col = last_col = 0;

	tcdrain(in_fileno);
	if (out)
		tcdrain(out_fileno);

	if (output && prompt)
		fprintf(out, "\r%s", prompt);

	line[0] = 0;
	new_line[0] = 0;

	for (;;) {

		if (output)
			fflush(out);

		extended_key = drv_rtems_shell_getchar(in);

		if (extended_key == (unsigned int)EOF)
			return -2;

		c = extended_key & RTEMS_SHELL_KEYS_NORMAL_MASK;

		/*
     * Make the extended_key usable as a boolean.
     */
		extended_key &= ~RTEMS_SHELL_KEYS_NORMAL_MASK;

		up = 0;

		if (extended_key) {
			switch (c) {
				case RTEMS_SHELL_KEYS_END:
					if (output)
						fprintf(out, "%s", line + col);
					col = (int)strlen(line);
					break;

				case RTEMS_SHELL_KEYS_HOME:
					if (output) {
						if (prompt)
							fprintf(out, "\r%s", prompt);
					}
					col = 0;
					break;

				case RTEMS_SHELL_KEYS_LARROW:
					c			 = 2;
					extended_key = 0;
					break;

				case RTEMS_SHELL_KEYS_RARROW:
					c			 = 6;
					extended_key = 0;
					break;

				case RTEMS_SHELL_KEYS_UARROW:
					c			 = 16;
					extended_key = 0;
					break;

				case RTEMS_SHELL_KEYS_DARROW:
					c			 = 14;
					extended_key = 0;
					break;

				case RTEMS_SHELL_KEYS_DEL:
					if (line[col] != '\0') {
						int end;
						int bs;
						strcpy(&line[col], &line[col + 1]);
						if (output) {
							fprintf(out, "\r%s%s ", prompt, line);
							end = (int)strlen(line);
							for (bs = 0; bs < ((end - col) + 1); bs++)
								fputc('\b', out);
						}
					}
					break;

				case RTEMS_SHELL_KEYS_INS:
					inserting = inserting ? 0 : 1;
					break;
			}
		}
		if (!extended_key) {
			switch (c) {
				case 1: /*Control-a*/
					if (output) {
						if (prompt)
							fprintf(out, "\r%s", prompt);
					}
					col = 0;
					break;

				case 2: /* Control-B */
					if (col > 0) {
						col--;
						if (output)
							fputc('\b', out);
					}
					break;
#if 0 /* Disable Control-D */
				case 4: /* Control-D */
					if (strlen(line)) {
						if (col < (int)strlen(line)) {
							strcpy(line + col, line + col + 1);
							if (output) {
								int bs;
								fprintf(out, "%s \b", line + col);
								for (bs = 0; bs < ((int)strlen(line) - col); bs++)
									fputc('\b', out);
							}
						}
						break;
					}
					/* Fall through */
#endif
				case EOF:
					if (output)
						fputc('\n', out);
					return -2;

				case 5: /*Control-e*/
					if (output)
						fprintf(out, "%s", line + col);
					col = (int)strlen(line);
					break;

				case 6: /* Control-F */
					if ((col < size) && (line[col] != '\0')) {
						if (output)
							fputc(line[col], out);
						col++;
					}
					break;

				case 7: /* Control-G */
					if (output) {
						/*
						 * The (int) cast is needed because the width specifier (%*)
						 * must be an int, but strlen() returns a size_t. Without
						 * the case, the result is a printf() format warning.
						 */
						fprintf(out, "\r%s%*c", prompt, (int)strlen(line), ' ');
						fprintf(out, "\r%s\x7", prompt);
					}
					memset(line, '\0', strlen(line));
					col = 0;
					break;

				case 11: /*Control-k*/
					if (line[col]) {
						if (output) {
							int end = strlen(line);
							int bs;
							fprintf(out, "%*c", end - col, ' ');
							for (bs = 0; bs < (end - col); bs++)
								fputc('\b', out);
						}
						line[col] = '\0';
					}
					break;

				case '\f':
					if (output) {
						int end;
						int bs;
						fputc('\f', out);
						fprintf(out, "\r%s%s", prompt, line);
						end = (int)strlen(line);
						for (bs = 0; bs < (end - col); bs++)
							fputc('\b', out);
					}
					break;

				case '\b':
				case '\x7f':
					if (col > 0) {
						int bs;
						col--;
						strcpy(line + col, line + col + 1);
						if (output) {
							fprintf(out, "\b%s \b", line + col);
							for (bs = 0; bs < ((int)strlen(line) - col); bs++)
								fputc('\b', out);
						}
					}
					break;

				case '\n':
				case '\r': {
					/*
					 * Process the command.
					 */
					if (output)
						fprintf(out, "\n");

					/*
					 * Only process the command if we have a command and it is not
					 * repeated in the history.
					 */
					if (strlen(line) == 0) {
						cmd = -1;
                    } else {
                        char *cmds0 = NULL;
                        int line_len = strlen(line);
                        line_len = line_len < size - 1 ? line_len : size - 1;

                        if ((cmd < 0) || (line_len != strlen(cmds[cmd]))
                            || (strcmp(line, cmds[cmd]) != 0)) {
                            if (count > 1) {
                                cmds0 = cmds[count - 1];
                                for (i = count - 1; i > 0; i--) {
                                    cmds[i] = cmds[i - 1];
                                }
                                cmds[0] = cmds0;
                            }
                            memcpy(cmds[0], line, line_len);
                            cmds[0][line_len] = 0;
							cmd = 0;
						} else {
                            if ((cmd > 1) && (line_len == strlen(cmds[cmd]))
                                && (strcmp(line, cmds[cmd]) == 0)) {
                                cmds0 = cmds[cmd];
                                for (i = cmd; i > 0; i--) {
                                    cmds[i] = cmds[i - 1];
                                }
                                cmds[0] = cmds0;
                                memcpy(cmds[0], line, line_len);
                                cmds[0][line_len] = 0;
								cmd = 0;
							}
						}
					}
				}
					return cmd;

				case 16: /* Control-P */
					if ((cmd >= (count - 1)) || (strlen(cmds[cmd + 1]) == 0)) {
						if (output) {
							str = '\x7';
							uartPrintkWrite(str);
						}
						break;
					}

					up = 1;
					/* drop through */
					fallthrough;

				case 14: /* Control-N */
				{
					int last_cmd = cmd;
					int clen	 = strlen(line);

					if (prompt)
						clen += strlen(prompt);

					if (up) {
						cmd++;
					} else {
						if (cmd < 0) {
							if (output)
								fprintf(out, "\x7");
							break;
						} else
							cmd--;
					}

					if ((last_cmd < 0) || (strcmp(cmds[last_cmd], line) != 0))
						memcpy(new_line, line, size);

					if (cmd < 0)
						memcpy(line, new_line, size);
					else
						memcpy(line, cmds[cmd], size);

					col = strlen(line);

					if (output) {
						fprintf(out, "\r%s%*c", prompt, clen, ' ');
						fprintf(out, "\r%s%s", prompt, line);
					}
				} break;

				case 20: /* Control-T */
					if (col > 0) {
						char tmp;
						if (col == (int)strlen(line)) {
							col--;
							if (output)
								fprintf(out, "\b");
						}
						tmp			  = line[col];
						line[col]	  = line[col - 1];
						line[col - 1] = tmp;
						if (output)
							fprintf(out, "\b%c%c", line[col - 1], line[col]);
						col++;
					} else {
						if (output) {
							str = '\x7';
							uartPrintkWrite(str);
						}
					}
					break;

				case 21: /* Control-U */
					if (col > 0) {
						int clen = strlen(line);

						strcpy(line, line + col);
						if (output) {
							fprintf(out, "\r%s%*c", prompt, clen, ' ');
							fprintf(out, "\r%s%s", prompt, line);
						}
						col = 0;
					}
					break;

				case '\t': /* tab key */
					drv_rtems_shell_complete(prompt, line);
					col = strlen(line);
					break;

				default:
					if ((col < (size - 1)) && (c >= ' ') && (c <= '~')) {
						int end = strlen(line);
						if (inserting && (col < end) && (end < size)) {
							int ch, bs;
							for (ch = end + 1; ch > col; ch--)
								line[ch] = line[ch - 1];
							if (output) {
								fprintf(out, "%s", line + col);
								for (bs = 0; bs < (end - col + 1); bs++) {
									str = '\b';
									uartPrintkWrite(str);
								}

							}
						}
						line[col++] = c;
						if (col > end)
							line[col] = '\0';
						if (output) {
							str = c;
							uartPrintkWrite(str);
						}
					}
					break;
			}
		}
	}
	return -2;
}

static void drv_shell_main_loop(void)
{
	struct termios term;
	struct termios previous_term;
	const char *   prompt = g_prompt;
	int			   cmd;
	int			   cmd_count = RTEMS_SHELL_CMD_COUNT;
	char *		   cmds[RTEMS_SHELL_CMD_COUNT];
	char *		   cmd_argv = g_cmd_argv_buf;
	int			   argc;
	char *		   argv[RTEMS_SHELL_MAXIMUM_ARGUMENTS];
	bool		   exit_shell = false;
	bool		   result	  = true;
	int			   line		  = 0;
	int			   ret;
	const char *   c;

	/* Make a raw terminal, Linux Manuals */
    if (tcgetattr(fileno(stdin), &previous_term) >= 0) {
      term = previous_term;
      term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
      term.c_oflag &= ~OPOST;
      term.c_oflag |= (OPOST|ONLCR); /* But with cr+nl on output */
      term.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
      term.c_cflag |= CLOCAL | CREAD;
      term.c_cc[VMIN]  = 1;
      term.c_cc[VTIME] = 0;
      if (tcsetattr (fileno(stdin), TCSADRAIN, &term) < 0) {
        fprintf(stderr,
                "shell: cannot set terminal attributes\n");
      }
    }

	/* Do not buffer if interactive else leave buffered */
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);

    memset(g_cmds_buf, 0, sizeof(g_cmds_buf));

    for (cmd = 0; cmd < cmd_count; cmd++) {
        cmds[cmd] = &g_cmds_buf[cmd * RTEMS_SHELL_CMD_SIZE];
    }

	for (;;) {
		/* getcmd section */
		cmd = drv_rtems_shell_line_editor(cmds, cmd_count,
						  RTEMS_SHELL_CMD_SIZE, prompt,
						  stdin, stdout);

		if (cmd == -1)
			continue; /* empty line */

		if (cmd == -2) {
			result = false;
			(void)result;
			break; /*EOF*/
		}

		line++;
		// fprintf(stdout, "%d: %s\n", line, cmds[cmd]);

		/* evaluate cmd section */
		c = cmds[cmd];
		while (*c) {
			if (!isblank((unsigned char)*c))
				break;
			c++;
		}

		if (*c == '\0') /* empty line */
			continue;

		if (*c == '#') { /* comment character */
			cmds[cmd][0] = 0;
			continue;
		}

		if (!strcmp(cmds[cmd], "bye") || !strcmp(cmds[cmd], "exit")) {
			fprintf(stdout, "Shell exiting\n");
			break;
		}

		/* exec cmd section */
		/* TODO:
           *  To avoid user crash catch the signals.
           *  Open a new stdio files with posibility of redirection *
           *  Run in a new shell task background. (unix &)
           *  Resuming. A little bash.
           */
		memcpy(cmd_argv, cmds[cmd], RTEMS_SHELL_CMD_SIZE);
		if (!drv_rtems_shell_make_args(cmd_argv, &argc, argv,
									   RTEMS_SHELL_MAXIMUM_ARGUMENTS)) {
			if (ushellCmdExec(argv[0],argc, argv) != 0) {
				if (!strcmp(argv[0], "mem")) {
					ret = mem_main_entry(argc, argv);
				} else {
					ret = drv_shell_execute_cmd(argc, argv);
				}
				(void)ret;
			}
		}

		/* end exec cmd section */
		if (exit_shell)
			break;
	}

	fflush(stdout);
	fflush(stderr);

	if (tcsetattr(fileno(stdin), TCSADRAIN, &previous_term) < 0) {
		fprintf(stderr, "shell: cannot reset terminal attributes\n");
	}

	return;
}

void drv_shell_print_logo_discard(void)
{
	printf("--------------------------------------------------\n");
	printf("-                                                -\n");
	printf("-         Welcome to Driver Debug Shell          -\n");
	printf("-                                                -\n");
	printf("--------------------------------------------------\n");
}

static void drv_shell_print_logo(void)
{
	printf("\n"
	       "\t       __       __           __ __\n"
	       "\t  ____/ /_____ / /_   ___   / // /\n"
	       "\t / __  // ___// __ \\ / _ \\ / // / \n"
	       "\t/ /_/ /(__  )/ / / //  __// // /  \n"
	       "\t\\__,_//____//_/ /_/ \\___//_//_/    1.0.1\n\n");
}

void driver_shell_exit(void);
static void driver_shell_run(OspTaskArgument data)
{
	(void)data;

	drv_shell_cmd_init();

	drv_shell_print_logo();

	drv_shell_main_loop();

	driver_shell_exit();
}

void driver_shell_exit(void)
{
	OspName name = ospBuildName('D', 'S', 'H', 'L');
	OspID	id   = 0;

	ospTaskIdent(name, 0, &id);
	if (id != 0) {
		/* fprintf(stdout, "Shell exiting\n");
		fflush(stdout);
		fflush(stderr); */
		ospTaskDelete(id);
	}
}

int driver_shell_init(bool dshell_task)
{
	OspStatusCode_e status;
	OspID			tid;

	if (!dshell_task) {
		driver_shell_run(0);
		return 0;
	}

	status = ospTaskCreate(ospBuildName('D', 'S', 'H', 'L'),
			       USER_TASK_PRIORITY_HIGHEST,   /* USER_TASK_PRIORITY_NORMAL, */
			       RTEMS_MINIMUM_STACK_SIZE * 4, /* like rtems_shell_init */
			       RTEMS_PREEMPT | RTEMS_TIMESLICE | RTEMS_NO_ASR,
			       OSP_LOCAL | OSP_FLOATING_POINT, &tid);
	if (status != OSP_SUCCESSFUL) {
		printf("ospTaskCreate fail, error code %d\n", status);
		return status;
	}

	/* start Task */
	status = ospTaskStart(tid, driver_shell_run, 0);
	if (status != OSP_SUCCESSFUL) {
		printf("ospTaskStart fail, error code %d\n", status);
		ospTaskDelete(tid);
		return status;
	}

	return 0;
}

int history(void)
{
	int   cmd_count = RTEMS_SHELL_CMD_COUNT;
	char *cmds[RTEMS_SHELL_CMD_COUNT];
	int   h = 1;
	int   i;

	cmds[0] = g_cmds_buf;
	for (i = 1; i < cmd_count; i++) {
		cmds[i] = cmds[i - 1] + RTEMS_SHELL_CMD_SIZE;
	}

	for (i = cmd_count - 1; i >= 0; i--) {
		if (strlen(cmds[i]) == 0)
			continue;

		printf("%3d %s\n", h++, cmds[i]);
	}

	return 0;
}

static CmdEntry_s gCmdTable[MAX_CMDS];
static S32 cmdCnt = 0;

OspMutex_t   gLockAt;
static char *gAutoInfo;
static char  gAutoInfoArr[AUTO_INFO_SESSION_SIZE];

typedef void (*FuncPtr)(S32, char **argv);

void r5ToolEntry(S32 argc, char **argv)
{
    S32 flag = 0;

    if (argc == 1) {
        printf("usage: sys\n");
        return ;
    }

    if (argc == 2) {
        printf("choice: call auto\n");
        return;
    }

    gAutoInfo = gAutoInfoArr;
    if (strcmp(argv[2], "auto") == 0) {
        if (argc < 10) {
            printf("usage: r5_tool sys auto {srcId} {dstId} {caseId} {sn} {timeStamp} call [para]\n");
            return;
        }
        sprintf(gAutoInfo,"AT+%s+%s+%s+%s+%s+%s+%s+data:", argv[1],
                argv[3], argv[4], argv[5], argv[6], argv[7], argv[9]);
        ospMutexLock(&gLockAt);
        printf("%s", gAutoInfo);

        for (S32 i = 0; i < sizeof(gCmdTable) / sizeof(gCmdTable[0]); i++) {
            if (!gCmdTable[i].cmdName) {
                break;
            }
            if (!strcmp(argv[9], gCmdTable[i].cmdName)) {
                ((FuncPtr)(gCmdTable[i].cmdFunc))(argc - 9, &argv[9]);
                flag = 1;
                break;
            }
        }

        if (flag == 0) {
            if (!strcmp(argv[1], "sys")) {
                drv_shell_execute_cmd(argc - 9, &argv[9]);
            }
        }

        printf("+@@\n");
        memset(gAutoInfo, 0, AUTO_INFO_SESSION_SIZE);
        ospMutexUnlock(&gLockAt);
    } else if (strcmp(argv[2], "call") == 0) {
        for (S32 i = 0; i < sizeof(gCmdTable) / sizeof(gCmdTable[0]); i++) {
            if (!gCmdTable[i].cmdName) {
                break;
            }
            if (!strcmp(argv[3], gCmdTable[i].cmdName)) {
                ((FuncPtr)(gCmdTable[i].cmdFunc))(argc - 3, &argv[3]);
                flag = 1;
                break;
            }
        }

        if (flag == 0) {
            if (!strcmp(argv[1], "sys")) {
                drv_shell_execute_cmd(argc - 3, &argv[3]);
            }
        }
    }
}

static S32 findCmdIdx(const S8 *name, S32 *isExact)
{
    S32 left = 0;
    S32 right = cmdCnt - 1;
    S32 mid = 0;

    if (cmdCnt == 0) {
        *isExact = 0;
        return 0;
    }

    *isExact = 0;

    while (left <= right) {
        mid = (left + right) / 2;
        S32 cmp = strcmp(name, gCmdTable[mid].cmdName);

        if (cmp == 0) {
            *isExact = 1;
            return mid;
        } else if (cmp < 0) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }

    *isExact = 0;
    return (strcmp(name, gCmdTable[mid].cmdName) < 0 ? mid : mid + 1);
}

void userCmdsList()
{
    S32 i;
    printf("Reged cmds (%d):\n", cmdCnt);
    for (i = 0; i < cmdCnt; i++) {
        printf("  %-20s [%p]\n", gCmdTable[i].cmdName, gCmdTable[i].cmdFunc);
    }
    printf("\n");
}

S32 ushellCmdRegister(const S8 *name, void (*func)(S32, S8**))
{
    if (cmdCnt >= MAX_CMDS) {
        fprintf(stderr, "Cmd table full! Can't reg '%s'\n", name);
        return -1;
    }

    S32 isExact;
    S32 idx = findCmdIdx(name, &isExact);

    if (isExact) {
        fprintf(stderr, "Cmd '%s' already reged!\n", name);
        return -2;
    }

    if (idx < cmdCnt) {
        memmove(&gCmdTable[idx + 1], &gCmdTable[idx],
               (cmdCnt - idx) * sizeof(CmdEntry_s));
    }

    strncpy(gCmdTable[idx].cmdName, name, sizeof(gCmdTable[idx].cmdName) - 1);
    gCmdTable[idx].cmdName[sizeof(gCmdTable[idx].cmdName) - 1] = '\0';
    gCmdTable[idx].cmdFunc = func;
    cmdCnt++;

    return 0;
}

S32 ushellCmdUnRegister(const S8 *name)
{
    S32 isExact;
    S32 idx = findCmdIdx(name, &isExact);

    if (cmdCnt == 0) {
        fprintf(stderr, "Cmd table empty!\n");
        return -1;
    }

    if (!isExact) {
        fprintf(stderr, "Cmd '%s' not found!\n", name);
        return -2;
    }

    if (idx < cmdCnt - 1) {
        memmove(&gCmdTable[idx], &gCmdTable[idx + 1],
               (cmdCnt - idx - 1) * sizeof(CmdEntry_s));
    }

    cmdCnt--;

    memset(&gCmdTable[cmdCnt], 0, sizeof(CmdEntry_s));

    return 0;
}

S32 ushellCmdExec(const S8 *name, S32 argc, S8 **argv)
{
    S32 isExact;
    S32 idx = findCmdIdx(name, &isExact);

    if (!isExact) {
        return -1;
    }

    gCmdTable[idx].cmdFunc(argc, argv);
    return 0;
}


#if 0
#if 1
int driver_shell_init(void)
{
	OspStatusCode_e status;
	OspID			tid;

	status = ospTaskCreate(ospBuildName('D', 'S', 'H', 'L'),
						   USER_TASK_PRIORITY_NORMAL,   /* USER_TASK_PRIORITY_HIGH, */
						   RTEMS_MINIMUM_STACK_SIZE *4, /* like u*/
						   RTEMS_PREEMPT | RTEMS_TIMESLICE | RTEMS_NO_ASR,
						   OSP_LOCAL | OSP_FLOATING_POINT,
						   &tid);
	if (status != OSP_SUCCESSFUL) {
		printf("ospTaskCreate fail, error code %d\n", status);
		return status;
	}

	/* start Task */
	status = ospTaskStart(tid, driver_shell_run, 0);
	if (status != OSP_SUCCESSFUL) {
		printf("ospTaskStart fail, error code %d\n", status);
		ospTaskDelete(tid);
		return status;
	}

	return 0;
}
#else
int driver_shell_init(void)
{
	drv_shell_cmd_init();

	drv_shell_print_logo();

	drv_shell_main_loop();

	return 0;
}
#endif
#endif

