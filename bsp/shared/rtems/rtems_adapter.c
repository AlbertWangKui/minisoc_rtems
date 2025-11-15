#include <string.h>
#include <bsp/linker-symbols.h>
#include <bsp_common.h>
#include "bsp_config.h"
#include "uart/drv_uart.h"

#define BSP_HIGH_EXP_VEC 0XFFFF0000
#define BSP_LOW_EXP_VEC 0x00000000
extern char bsp_int_vec_overlay_start[];

int need_vec_exp_remap(void);

///< 重载内核函数（GIC）
uint32_t bsp_gic_dist_base_get(void)
{
    return SYS_GIC_DIST_BASE_ADRS;
}

uint32_t bsp_gic_cpuif_base_get(void)
{
    return SYS_GIC_CPUIF_BASE_ADRS;
}


///< 重载内核函数（vector remap）
uint32_t global_overlay_target_address_start =
                                 (uintptr_t)bsp_int_vec_overlay_start;
int need_vec_exp_remap(void)
{
    unsigned int cp15SCTLR = 0xFFFFFFFF;

    asm volatile (
            "MRC p15, 0, %0, c1, c0, 0\n"
            :"=r"(cp15SCTLR)
            :
            :"memory");

    ///< 0xFFFF0000-0xFFFF001C SCTLR.V = 1 , 0x00000000-0x0000001C SCTLR.V = 0
    unsigned int startAddr = ((cp15SCTLR >> 13) & 0x00000001) ? BSP_HIGH_EXP_VEC : BSP_LOW_EXP_VEC;

    void *need_remap_ptr;
    need_remap_ptr = &bsp_start_vector_table_begin;
    unsigned int need_remap_int = (unsigned int) need_remap_ptr;

    asm volatile ("":::"memory");

    if (need_remap_int != startAddr){
        return 1;
    }
    return 0;
}

void vec_exp_remap(void)
{
    if (need_vec_exp_remap() != 0) {
        uint32_t vec_overlay_start = global_overlay_target_address_start;
        memcpy((void*)vec_overlay_start, &bsp_start_vector_table_begin, 64);
    }
}

///< 重载内核函数（printk support）
void bsp_uart_init(void)
{
}

void bsp_uart_put_char(char c)
{
    uartPrintkWrite(c);
}

int bsp_uart_get_char(void)
{
    rtems_interrupt_level level;
    int c = -1;

    rtems_interrupt_disable(level);
    c = uartConsoleRead();
    rtems_interrupt_enable(level);
    return c;
}

///< console init
static bool bspUartFirstOpen(rtems_termios_tty *tty, rtems_termios_device_context *ctx,
    struct termios *term, rtems_libio_open_close_args_t *args)
{
    bool ret = true;

    if (EXIT_SUCCESS != uartConsoleInit(tty)) {
        ret = false;
        goto exit;
    }
    rtems_termios_set_best_baud(term, 115200);
exit:
    return ret;
}

static void bspUartLastClose(rtems_termios_tty *tty,
    rtems_termios_device_context *base, rtems_libio_open_close_args_t *args)
{
}

#ifndef CONSOLE_USE_INTERRUPTS
static int bspUartRead(rtems_termios_device_context *base)
{
    return uartConsoleRead();
}
#endif

static void bspUartWrite(rtems_termios_device_context *base, const char *buf, size_t n)
{
    uartConsoleWrite(buf, n);
    return;
}

static bool bspUartSetAttributes(rtems_termios_device_context *base, const struct termios *t)
{
    return true;
}
#ifdef CONSOLE_USE_INTERRUPTS
const rtems_termios_device_handler bsp_uart_device_handler = {
    .first_open = bspUartFirstOpen,
    .last_close = bspUartLastClose,
    .poll_read = NULL,
    .write = bspUartWrite,
    .set_attributes = bspUartSetAttributes,
    .mode = TERMIOS_IRQ_DRIVEN
};
#else
const rtems_termios_device_handler bsp_uart_device_handler = {
    .first_open = bspUartFirstOpen,
    .last_close = bspUartLastClose,
    .poll_read = bspUartRead,
    .write = bspUartWrite,
    .set_attributes = bspUartSetAttributes,
    .mode = TERMIOS_POLLED
};
#endif

rtems_device_driver console_initialize(rtems_device_major_number major,
    rtems_device_minor_number minor, void *arg)
{
    static rtems_termios_device_context base = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("bsp_uart");
    rtems_termios_initialize();
    rtems_termios_device_install("/dev/console", &bsp_uart_device_handler, NULL, &base);
    return RTEMS_SUCCESSFUL;
}

///< rtems startup hooks
static void bsp_r5_actrl_config(void)
{
    asm volatile (
        "MRC p15, 0, r0, c1, c0, 1;"
        "ORR r0, r0, #0x40;"
        "ORR r0, r0, #0x30000;"
        "BIC r0, r0, #0x8000;"
        "MCR p15, 0, r0, c1, c0, 1;"
        "ISB;"
        :
        :
        :"r0"
        );
}

BSP_START_TEXT_SECTION static void  bsp_open_serror(void)
{
    /* clear Asynchronous exception mask bits */
    uint32_t level;
    ARM_SWITCH_REGISTERS;
    __asm__ volatile (
            ARM_SWITCH_TO_ARM
            "mrs %[level], cpsr\n"
            "bic  %[level], %[level], #0x100\n"
            "msr cpsr_x, %[level]\n"
            ARM_SWITCH_BACK
            : [level] "=&r" (level)   ARM_SWITCH_ADDITIONAL_OUTPUT
            );
}

BSP_START_TEXT_SECTION void bsp_start_hook_0( void )
{
    cpsrAIFDisable();
    icacheDisable();
    dcacheDisable();
    icacheAllInvalid();
    dcacheAllInvalid();
    bsp_open_serror();
}

BSP_START_TEXT_SECTION void bsp_start_hook_1( void )
{
#ifndef CONFIG_PLATFORM_EMU
    bsp_start_clear_bss();
#endif
    vec_exp_remap();
    bsp_r5_actrl_config();
    mpuInit();
    pllInit();
    exceptionAddressRemap();
}

