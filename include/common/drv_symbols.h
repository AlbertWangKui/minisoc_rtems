#define drv_add_shell_cmd(func) {#func, (char *)func}

typedef unsigned int	SYM_TYPE;

#define SYM_UNDF        0x0     /* undefined (lowest 8 bits only) */
#define SYM_GLOBAL      0x1     /* global (external) */
#define SYM_ABS         0x2     /* absolute */
#define SYM_TEXT        0x4     /* text */
#define SYM_DATA        0x8     /* data */
#define SYM_BSS        0x10     /* bss */
#define SYM_COMM       0x20     /* common symbol */
#define SYM_LOCAL      0x40     /* local */
#define SYM_THUMB      0x80     /* Thumb function */

struct drv_shell_cmd {
	const char *name;
	char       *addr;
	SYM_TYPE	type;
};
