#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include "common_defines.h"
#include "drv_regcmds.h"
#include "drv_shell.h"
#include "log_msg.h"

int parseNumber(const char* str, U64* result) {
    if (!str || !result) {
        return -1;
    }

    char* endptr;
    errno = 0;

    if (strncmp(str, "0x", 2) == 0 || strncmp(str, "0X", 2) == 0) {
        *result = strtoull(str, &endptr, 16);
    } else {
        *result = strtoull(str, &endptr, 10);
    }

    if (errno != 0 || *endptr != '\0') {
        LOGE("Invalid number format: %s\n", str);
        return -1;
    }

    return 0;
}

int checkAlignment(U64 addr, DataWidth_t width) {
    if (addr % width != 0) {
        LOGE("Address 0x%llx is not aligned to %d-byte boundary\n", (unsigned long long)addr, width);
        return -1;
    }
    return 0;
}

DataWidth_t parseWidthOption(const char* arg) {
    if (strcmp(arg, "-8") == 0) {
        return WIDTH_8;
    } else if (strcmp(arg, "-16") == 0) {
        return WIDTH_16;
    } else if (strcmp(arg, "-32") == 0) {
        return WIDTH_32;
    } else if (strcmp(arg, "-64") == 0) {
        return WIDTH_64;
    }
    return WIDTH_INVALID;
}

int isValidBitPosition(int bit, DataWidth_t width) {
    int maxBits = width * 8 - 1;
    if (bit < 0 || bit > maxBits) {
        LOGE("Invalid bit position %d for %d-byte operation (valid range: 0-%d)\n",
                bit, width, maxBits);
        return 0;
    }
    return 1;
}

void cmdMd(S32 argc, S8 **argv) {
    if (argc < 2) {
        LOGE("Usage: md [-8|-16|-32|-64] addr [count]\n");
        return;
    }

    DataWidth_t width = WIDTH_32;
    U64 addr, count = 1;
    int argIdx = 1;

    if (argv[1][0] == '-') {
        width = parseWidthOption(argv[1]);
        if (width == WIDTH_INVALID) {
            LOGE("Invalid width option: %s\n", argv[1]);
            return;
        }
        argIdx = 2;
        if (argc < 3) {
            LOGE("Usage: md [-8|-16|-32|-64] addr [count]\n");
            return;
        }
    }

    if (parseNumber(argv[argIdx], &addr) != 0) {
        return;
    }

    if (checkAlignment(addr, width) != 0) {
        return;
    }

    if (argc > argIdx + 1) {
        if (parseNumber(argv[argIdx + 1], &count) != 0) {
            return;
        }
    }

    U64 startAddr = (addr / BYTES_PER_LINE) * BYTES_PER_LINE;
    U64 endAddr = addr + count * width;

    for (U64 lineAddr = startAddr; lineAddr < endAddr; lineAddr += BYTES_PER_LINE) {
        LOGE("%08llx: ", (unsigned long long)lineAddr);

        for (int i = 0; i < BYTES_PER_LINE; i++) {
            U64 byteAddr = lineAddr + i;

            if (byteAddr < addr || byteAddr >= endAddr) {
                LOGE("   ");
            } else {
                U8 val = reg8Read((U32)byteAddr);
                LOGE("%02x ", val);
            }

            if (i == 7) LOGE(" ");
        }

        LOGE(" |");
        for (int i = 0; i < BYTES_PER_LINE; i++) {
            U64 byteAddr = lineAddr + i;

            if (byteAddr < addr || byteAddr >= endAddr) {
                LOGE(" ");
            } else {
                U8 val = reg8Read((U32)byteAddr);
                LOGE("%c", (val >= 32 && val <= 126) ? val : '.');
            }
        }
        LOGE("|\n");
    }
}

void cmdMm(S32 argc, S8 **argv) {
    if (argc < 3) {
        LOGE("Usage: mm [-8|-16|-32|-64] addr val\n");
        return;
    }

    DataWidth_t width = WIDTH_32;
    U64 addr, val;
    int argIdx = 1;

    if (argv[1][0] == '-') {
        width = parseWidthOption(argv[1]);
        if (width == WIDTH_INVALID) {
            LOGE("Invalid width option: %s\n", argv[1]);
            return;
        }
        argIdx = 2;
        if (argc < 4) {
            LOGE("Usage: mm [-8|-16|-32|-64] addr val\n");
            return;
        }
    }

    if (parseNumber(argv[argIdx], &addr) != 0) {
        return;
    }

    if (checkAlignment(addr, width) != 0) {
        return;
    }

    if (parseNumber(argv[argIdx + 1], &val) != 0) {
        return;
    }

    U64 maxVal = (1ULL << (width * 8)) - 1;
    if (val > maxVal) {
        LOGE("Value 0x%llx exceeds maximum for %d-byte operation (max: 0x%llx)\n",
                (unsigned long long)val, width, (unsigned long long)maxVal);
        return;
    }

    switch (width) {
        case WIDTH_8:
            reg8Write((U32)addr, (U8)val);
            break;
        case WIDTH_16:
            reg16Write((U32)addr, (U16)val);
            break;
        case WIDTH_32:
            reg32Write((U32)addr, (U32)val);
            break;
        case WIDTH_64:
            reg64Write((U32)addr, (U64)val);
            break;
        default:
            return;
    }
}

void cmdMdr(S32 argc, S8 **argv) {
    if (argc < 4) {
        LOGE("Usage: mdr [-8|-16|-32|-64] addr msb lsb\n");
        return;
    }

    DataWidth_t width = WIDTH_32;
    U64 addr, msb, lsb;
    int argIdx = 1;

    if (argv[1][0] == '-') {
        width = parseWidthOption(argv[1]);
        if (width == WIDTH_INVALID) {
            LOGE("Invalid width option: %s\n", argv[1]);
            return;
        }
        argIdx = 2;
        if (argc < 5) {
            LOGE("Usage: mdr [-8|-16|-32|-64] addr msb lsb\n");
            return;
        }
    }

    if (parseNumber(argv[argIdx], &addr) != 0 ||
        parseNumber(argv[argIdx + 1], &msb) != 0 ||
        parseNumber(argv[argIdx + 2], &lsb) != 0) {
        return;
    }

    if (checkAlignment(addr, width) != 0) {
        return;
    }

    U64 maxBit = width * 8 - 1;
    if (msb > maxBit || lsb > maxBit || msb < lsb) {
        LOGE("Invalid bit range [%llu:%llu] for %d-byte operation (valid range: [0:%llu])\n",
                (unsigned long long)msb, (unsigned long long)lsb, width, (unsigned long long)maxBit);
        return;
    }

    U64 regVal;

    switch (width) {
        case WIDTH_8:
            regVal = reg8Read((U32)addr);
            break;
        case WIDTH_16:
            regVal = reg16Read((U32)addr);
            break;
        case WIDTH_32:
            regVal = reg32Read((U32)addr);
            break;
        case WIDTH_64:
            regVal = reg64Read((U32)addr);
            break;
        default:
            return;
    }

    U64 mask = ((1ULL << (msb - lsb + 1)) - 1);
    U64 bitfield = (regVal >> lsb) & mask;

    LOGE("0x%llx\n", (unsigned long long)bitfield);
}

void cmdMmr(S32 argc, S8 **argv) {
    if (argc < 5) {
        LOGE("Usage: mmr [-8|-16|-32|-64] addr msb lsb val\n");
        return;
    }

    DataWidth_t width = WIDTH_32;
    U64 addr, msb, lsb, val;
    int argIdx = 1;

    if (argv[1][0] == '-') {
        width = parseWidthOption(argv[1]);
        if (width == WIDTH_INVALID) {
            LOGE("Invalid width option: %s\n", argv[1]);
            return;
        }
        argIdx = 2;
        if (argc < 6) {
            LOGE("Usage: mmr [-8|-16|-32|-64] addr msb lsb val\n");
            return;
        }
    }

    if (parseNumber(argv[argIdx], &addr) != 0 ||
        parseNumber(argv[argIdx + 1], &msb) != 0 ||
        parseNumber(argv[argIdx + 2], &lsb) != 0 ||
        parseNumber(argv[argIdx + 3], &val) != 0) {
        return;
    }

    if (checkAlignment(addr, width) != 0) {
        return;
    }

    U64 maxBit = width * 8 - 1;
    if (msb > maxBit || lsb > maxBit || msb < lsb) {
        LOGE("Invalid bit range [%llu:%llu] for %d-byte operation (valid range: [0:%llu])\n",
                (unsigned long long)msb, (unsigned long long)lsb, width, (unsigned long long)maxBit);
        return;
    }

    U64 maxFieldVal = (1ULL << (msb - lsb + 1)) - 1;
    if (val > maxFieldVal) {
        LOGE("Value 0x%llx exceeds maximum for bitfield [%llu:%llu] (max: 0x%llx)\n",
                (unsigned long long)val, (unsigned long long)msb, (unsigned long long)lsb, (unsigned long long)maxFieldVal);
        return;
    }

    U64 regVal;

    switch (width) {
        case WIDTH_8:
            regVal = reg8Read((U32)addr);
            break;
        case WIDTH_16:
            regVal = reg16Read((U32)addr);
            break;
        case WIDTH_32:
            regVal = reg32Read((U32)addr);
            break;
        case WIDTH_64:
            regVal = reg64Read((U32)addr);
            break;
        default:
            return;
    }

    U64 mask = ((1ULL << (msb - lsb + 1)) - 1) << lsb;
    regVal = (regVal & ~mask) | ((val << lsb) & mask);

    switch (width) {
        case WIDTH_8:
            reg8Write((U32)addr, (U8)regVal);
            break;
        case WIDTH_16:
            reg16Write((U32)addr, (U16)regVal);
            break;
        case WIDTH_32:
            reg32Write((U32)addr, (U32)regVal);
            break;
        case WIDTH_64:
            reg64Write((U32)addr, (U64)regVal);
            break;
        default:
            return;
    }
}

void cmdSb(S32 argc, S8 **argv) {
    if (argc < 3) {
        LOGE("Usage: sb [-8|-16|-32|-64] addr bit\n");
        return;
    }

    DataWidth_t width = WIDTH_32;
    U64 addr, bit;
    int argIdx = 1;

    if (argv[1][0] == '-') {
        width = parseWidthOption(argv[1]);
        if (width == WIDTH_INVALID) {
            LOGE("Invalid width option: %s\n", argv[1]);
            return;
        }
        argIdx = 2;
        if (argc < 4) {
            LOGE("Usage: sb [-8|-16|-32|-64] addr bit\n");
            return;
        }
    }

    if (parseNumber(argv[argIdx], &addr) != 0 ||
        parseNumber(argv[argIdx + 1], &bit) != 0) {
        return;
    }

    if (checkAlignment(addr, width) != 0) {
        return;
    }

    if (!isValidBitPosition(bit, width)) {
        return;
    }

    U64 regVal;

    switch (width) {
        case WIDTH_8:
            regVal = reg8Read((U32)addr);
            break;
        case WIDTH_16:
            regVal = reg16Read((U32)addr);
            break;
        case WIDTH_32:
            regVal = reg32Read((U32)addr);
            break;
        case WIDTH_64:
            regVal = reg64Read((U32)addr);
            break;
        default:
            return;
    }

    regVal |= (1ULL << bit);

    switch (width) {
        case WIDTH_8:
            reg8Write((U32)addr, (U8)regVal);
            break;
        case WIDTH_16:
            reg16Write((U32)addr, (U16)regVal);
            break;
        case WIDTH_32:
            reg32Write((U32)addr, (U32)regVal);
            break;
        case WIDTH_64:
            reg64Write((U32)addr, (U64)regVal);
            break;
        default:
            return;
    }
}

void cmdCb(S32 argc, S8 **argv) {
    if (argc < 3) {
        LOGE("Usage: cb [-8|-16|-32|-64] addr bit\n");
        return;
    }

    DataWidth_t width = WIDTH_32;
    U64 addr, bit;
    int argIdx = 1;

    if (argv[1][0] == '-') {
        width = parseWidthOption(argv[1]);
        if (width == WIDTH_INVALID) {
            LOGE("Invalid width option: %s\n", argv[1]);
            return;
        }
        argIdx = 2;
        if (argc < 4) {
            LOGE("Usage: cb [-8|-16|-32|-64] addr bit\n");
            return;
        }
    }

    if (parseNumber(argv[argIdx], &addr) != 0 ||
        parseNumber(argv[argIdx + 1], &bit) != 0) {
        return;
    }

    if (checkAlignment(addr, width) != 0) {
        return;
    }

    if (!isValidBitPosition(bit, width)) {
        return;
    }

    U64 regVal;

    switch (width) {
        case WIDTH_8:
            regVal = reg8Read((U32)addr);
            break;
        case WIDTH_16:
            regVal = reg16Read((U32)addr);
            break;
        case WIDTH_32:
            regVal = reg32Read((U32)addr);
            break;
        case WIDTH_64:
            regVal = reg64Read((U32)addr);
            break;
        default:
            return;
    }

    regVal &= ~(1ULL << bit);

    switch (width) {
        case WIDTH_8:
            reg8Write((U32)addr, (U8)regVal);
            break;
        case WIDTH_16:
            reg16Write((U32)addr, (U16)regVal);
            break;
        case WIDTH_32:
            reg32Write((U32)addr, (U32)regVal);
            break;
        case WIDTH_64:
            reg64Write((U32)addr, (U64)regVal);
            break;
        default:
            return;
    }
}

void drvRegcmdsInit(void) {
    ushellCmdRegister("md", cmdMd);
    ushellCmdRegister("mm", cmdMm);
    ushellCmdRegister("mdr", cmdMdr);
    ushellCmdRegister("mmr", cmdMmr);
    ushellCmdRegister("sb", cmdSb);
    ushellCmdRegister("cb", cmdCb);

    LOGE("mem rw commands initialized: md, mm, mdr, mmr, sb, cb\n");
}