
#include "log_msg.h"
#include "bsp_api.h"
#include "bsp_drv_id.h"
#include "bsp_sbr.h"
#include "sbr_api.h"

static S32 devCfgGet(DevList_e devID, SbrBrdCfg_s *cfg)
{
    S32 ret;

    if (cfg == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    if (devSbrRead(devID, cfg, 0, sizeof(SbrBrdCfg_s)) != sizeof(SbrBrdCfg_s)) {
        ret = -EIO;
        goto exit;
    }

#ifdef CONFIG_DUMP_SBR
    LOGE("brd_init: SBR dump - cfgNum:%u\r\n", cfg->cfgNum);
    for (int i = 0; i < cfg->cfgNum && i < 16; i++) {
        LOGE("brd_init: SBR dump - brdCfg[%d] addr:0x%08x, val:0x%08x\r\n",
             i, cfg->brdCfg[i].addr, cfg->brdCfg[i].val);
    }
#endif

    if (cfg->cfgNum == 0) {
        ret = -EINVAL;
        goto exit;
    }

exit:
    return ret;
}

S32 doBrdCfg(DevList_e devID)
{
    SbrBrdCfg_s cfg;
    int i = 0;

    /* Check driver match */
    if (!isDrvMatch(devID, DRV_ID_BRD_INIT)) {
        return -EINVAL;
    }

    if (devCfgGet(devID, &cfg) != EXIT_SUCCESS) {
        return -EIO;
    }

    for (i = 0; i < cfg.cfgNum; i++) {
        if (cfg.brdCfg[i].addr == 0) {
            break;
        }
        reg32Write(cfg.brdCfg[i].addr, cfg.brdCfg[i].val);
    }

    return EXIT_SUCCESS;
}