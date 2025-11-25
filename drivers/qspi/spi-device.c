/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 * @file spi-device.c
 * @brief
 * @author tianye
 * @version
 * @date 2024-05-23
 */

#include <bsp/drv_spi_device.h>
#include "drv_spi_device_inner.h"
#include "common_defines.h"

static const struct flash_info gigadevice_nor_parts[] = {
    {
        .id = {
            .bytes = {0xc8, 0x67, 0x1a},
            .len = 3,
        },
        .size = SZ_64M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
    {
        .id = {
            .bytes = {0xc8, 0x60, 0x1a},
            .len = 3,
        },
        .size = SZ_64M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
    {
        .id = {
            .bytes = {0xc8, 0x67, 0x19},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
    {
        .id = {
            .bytes = {0xc8, 0x68, 0x19},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 1,
    },
    {
        .id = {
            .bytes = {0xc8, 0x60, 0x19},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
};
const struct spi_nor_manufacturer spi_nor_gigadevice = {
    .parts = gigadevice_nor_parts,
    .nparts = ARRAY_SIZE(gigadevice_nor_parts),
};

static const struct flash_info winbond_nor_parts[] = {
    {
        .id = {
            .bytes = {0xef, 0x60, 0x20},
            .len = 3,
        },
        .size = SZ_64M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
    {
        .id = {
            .bytes = {0xef, 0x80, 0x20},
            .len = 3,
        },
        .size = SZ_64M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
    {
        .id = {
            .bytes = {0xef, 0x60, 0x19},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
    {
        .id = {
            .bytes = {0xef, 0x80, 0x19},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
};
const struct spi_nor_manufacturer spi_nor_winbond = {
    .parts = winbond_nor_parts,
    .nparts = ARRAY_SIZE(winbond_nor_parts),
};

static const struct flash_info macronix_nor_parts[] = {
    {
        .id = {
            .bytes = {0xc2, 0x25, 0x39},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
};
const struct spi_nor_manufacturer spi_nor_macronix = {
    .parts = macronix_nor_parts,
    .nparts = ARRAY_SIZE(macronix_nor_parts),
};

static const struct flash_info xmc_nor_parts[] = {
    {
        .id = {
            .bytes = {0x20, 0x41, 0x19},
            .len = 3,
        },
        .size = SZ_32M,
        .type = DEV_FLASH,
        .io_wire = 0xFF,
    },
};
const struct spi_nor_manufacturer spi_nor_xmc = {
    .parts = xmc_nor_parts,
    .nparts = ARRAY_SIZE(xmc_nor_parts),
};

static const struct flash_info cypress_parts[] = {
    {
        .id = {
            .bytes = {0x06, 0x81, 0x88, 0xa1},
            .len = 4,
        },
        .size = SZ_128K,
        .type = DEV_NVSRAM,
        .io_wire = 0x1,
    },
};
const struct spi_nor_manufacturer spi_cypress = {
    .parts = cypress_parts,
    .nparts = ARRAY_SIZE(cypress_parts),
};

static const struct flash_info mcLOGEc_parts[] = {
    {
        .id = {
            .bytes = {0xae, 0x60, 0x84, 0xb1},
            .len = 4,
        },
        .size = SZ_128K,
        .type = DEV_NVSRAM,
        .io_wire = 0x1,
    },
};
const struct spi_nor_manufacturer spi_mcLOGEc = {
    .parts = mcLOGEc_parts,
    .nparts = ARRAY_SIZE(mcLOGEc_parts),
};

///< 默认设备型号
static const struct flash_info default_parts[] = {
    {
        .id = {
            .bytes = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            .len = 6,
        },
        .size = 0xFFFFFFFF,
        .type = DEV_NONE,
        .io_wire = 0x1,
    },
};
const struct spi_nor_manufacturer spi_default = {
    .parts = default_parts,
    .nparts = ARRAY_SIZE(default_parts),
};
