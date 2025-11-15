#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtems/shell.h>

#ifndef PAGE_SIZE
#define PAGE_SIZE (1 << 12)
#endif
#define reg_read(addr)  (*((volatile uint32_t *)(addr)))
#define reg_write(addr,data)  {*((volatile uint32_t *)(addr)) = (data);}
#define ONE_LINE_BYTES_LEN  16
void pr_usage(char **argv)
{
    printf("dshell memtool usages:\n" \
        "%s [0x]phyaddr\n" \
        "%s [0x]phyaddr [0x]len\n" \
        "%s [0x]phyaddr [0x]1 [0x]data\n" \
        "%s [0x]phyaddr [0x]2 [0x]data\n" \
        "%s [0x]phyaddr [0x]4 [0x]data\n" \
        "%s [0x]phyaddr [0x]8 [0x]data\n" \
        "%s [0x]phyaddr bit_high:bit_low\n" \
        "%s [0x]phyaddr bit_high:bit_low [0x]data\n",
        argv[0], argv[0], argv[0], argv[0], argv[0], argv[0], argv[0], argv[0]);

}
unsigned int readValue(unsigned long phyAddr)
{
    unsigned int data;

    data = *(unsigned int *)phyAddr;

    return data;
}

void print_data(unsigned long phyAddr, int len)
{
    int i;
    int N;

    if (len < 4)
        return ;
    if (len < ONE_LINE_BYTES_LEN) {
        printf("%.8lx: ", phyAddr);
        for (i = 0; i < len; i += 4) {
            printf("%.8x    ",*(unsigned int *)(phyAddr + i));
        }
        printf("\n");
    } else {
        N = len / ONE_LINE_BYTES_LEN;
        for (i = 0; i < N; i++) {
            printf("%.8lx: %.8x    %.8x    %.8x    %.8x\n",
                phyAddr + i * ONE_LINE_BYTES_LEN,
                *(unsigned int *)(phyAddr + i * ONE_LINE_BYTES_LEN),
                *(unsigned int *)(phyAddr + i * ONE_LINE_BYTES_LEN + 4),
                *(unsigned int *)(phyAddr + i * ONE_LINE_BYTES_LEN + 8),
                *(unsigned int *)(phyAddr + i * ONE_LINE_BYTES_LEN + 12)
                );
        }
        len = len - N * ONE_LINE_BYTES_LEN;
        if (len == 0)
            return ;
        printf("%.8lx: ", phyAddr + N * ONE_LINE_BYTES_LEN);
        for (i = 0; i < len; i += 4)
            printf("%.8x    ",*(unsigned int *)(phyAddr + N * ONE_LINE_BYTES_LEN + i));
		printf("\n");
    }
}

void writeValue(unsigned long phyAddr, unsigned long bits, unsigned long data)
{
    unsigned int tmp_data;

    if ((bits != 1) && (bits != 2) && (bits != 4) && (bits != 8))
    {
        printf("invalid length, should be 1 or 2 or 4 or 8 bytes\n");
        return ;
    }
    tmp_data = reg_read(phyAddr);

    if (bits == 1) {
        tmp_data = (tmp_data & ~0xFF) | (data & 0xFF); ///< 保留高位，更新低8位
        reg_write(phyAddr, tmp_data);
    } else if(bits == 2) {
        tmp_data = (tmp_data & ~0xFFFF) | (data & 0xFFFF); ///< 保留高位，更新低16位
        reg_write(phyAddr, tmp_data);
    } else if (bits == 4)
        *(unsigned int *)phyAddr = data;
    else if(bits == 8)
        *(unsigned long *)phyAddr = data;
}

int mem_main_entry(int argc, char **argv)
{
    int val;
    unsigned long phyAddr;
    unsigned long data;
    unsigned long len;
    unsigned long bits;
    unsigned int  i;
    unsigned long len2;
    unsigned int bit_hi;
    unsigned int bit_low;
    unsigned int bit_len;
    unsigned int tmp_data;
    char *p = NULL;

    if ((argc < 2) || (argc > 5))
    {
        printf("invalid para\n");
        pr_usage(argv);
        return -1;
    }
    if ((argv[1][0] == '0') && (argv[1][1] == 'x')) {
        phyAddr = strtoul(&argv[1][2], NULL, 16);
    } else {
        phyAddr = strtoul(&argv[1][0], NULL, 16);
    }
    phyAddr = phyAddr & (~0x3); ///< align to 4 bytes

    switch (argc)
    {
    case 2:
        rtems_cache_invalidate_multiple_data_lines((const void *)phyAddr, 4); ///< cache invalid
        val = readValue(phyAddr);
        printf("%x: %x\n", (unsigned int)phyAddr, val);
        break;

    case 3:
        if ((p = strchr(argv[2], ':'))) {
            int i;
            for (i = 0; i < strlen(argv[2]); i++) {
                if (!((argv[2][i] == ':') || ((argv[2][i] >= '0') && (argv[2][i] <= '9')))) {
                    pr_usage(argv);
                    return -1;
                }
            }

            bit_hi = atoi(argv[2]);
            bit_low = atoi(p + 1);
            tmp_data = reg_read(phyAddr);
            bit_len = bit_hi - bit_low + 1;
            tmp_data = (tmp_data & (((1 << bit_len) - 1) << bit_low)) >> bit_low;
            printf("%08lx[%d:%d]: %x\n", phyAddr, bit_hi, bit_low, tmp_data);
        } else {
            if ((argv[2][0] == '0') && (argv[2][1] == 'x')) {
                len = strtoul(&argv[2][2], NULL, 16);
            } else {
                len = strtoul(&argv[2][0], NULL, 16);
            }
            if (len > 0x1000)
                len = 0x1000;
            rtems_cache_invalidate_multiple_data_lines((const void *)phyAddr, len); ///< cahce invalid
            print_data(phyAddr, len);
        }
        break;

    case 4:
        if ((argv[3][0] == '0') && (argv[3][1] == 'x')) {
            data = strtoul(&argv[3][2], NULL, 16);
        } else {
            data = strtoul(&argv[3][0], NULL, 16);
        }

        if ((p = strchr(argv[2], ':'))) {
            int i;
            for (i = 0; i < strlen(argv[2]); i++) {
                if (!((argv[2][i] == ':') || ((argv[2][i] >= '0') && (argv[2][i] <= '9')))) {
                    pr_usage(argv);
                    return -1;
                }
            }

            bit_hi = atoi(argv[2]);
            bit_low = atoi(p + 1);
            tmp_data = reg_read(phyAddr);
            bit_len = bit_hi - bit_low + 1;
            tmp_data = (tmp_data & ~(((1 << bit_len) - 1) << bit_low)) | (data << bit_low);
            reg_write(phyAddr, tmp_data);

        } else {
            if ((argv[2][0] == '0') && (argv[2][1] == 'x')) {
                bits = strtoul(&argv[2][2], NULL, 16);
            } else {
                bits = strtoul(&argv[2][0], NULL, 16);
            }
            writeValue(phyAddr, bits, data);
            rtems_cache_flush_multiple_data_lines((const void *)phyAddr, bits);
        }

        break;

    case 5:
        if ((argv[2][0] == '0') && (argv[2][1] == 'x')) {
            bits = strtoul(&argv[2][2], NULL, 16);
        } else {
            bits = strtoul(&argv[2][0], NULL, 16);
        }

        if ((argv[3][0] == '0') && (argv[3][1] == 'x')) {
            data = strtoul(&argv[3][2], NULL, 16);
        } else {
            data = strtoul(&argv[3][0], NULL, 16);
        }

        if ((argv[4][0] == '0') && (argv[4][1] == 'x')) {
            len2 = strtoul(&argv[4][2], NULL, 16);
        } else {
            len2 = strtoul(&argv[4][0], NULL, 16);
        }

        for (i = 0; i < len2 / 4; i++) {
            writeValue(phyAddr + i * 4, 4, data);
        }
        rtems_cache_flush_multiple_data_lines((const void *)phyAddr, len2);
        break;

    default:
        return -1;
    }

    return 0;
}

