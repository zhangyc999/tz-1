#include "addr.h"
#include "type.h"
#include "vx.h"

extern MSG_Q_ID msg_pwr;
extern MSG_Q_ID msg_swh;

MSG_Q_ID remap_addr_msg(u8 addr)
{
        switch (addr) {
        case ADDR_GEND:
        case ADDR_GENS:
        case ADDR_PSU:
                return msg_pwr;
        case ADDR_SWH0:
        case ADDR_SWH1:
        case ADDR_SWH2:
        case ADDR_SWH3:
                return msg_swh;
        case ADDR_RSE0:
        case ADDR_RSE1:
        case ADDR_RSE2:
        case ADDR_RSE3:
                return 0;
        case ADDR_SWV0:
        case ADDR_SWV1:
        case ADDR_SWV2:
        case ADDR_SWV3:
                return 0;
        case ADDR_PRP0:
        case ADDR_PRP1:
        case ADDR_PRP2:
        case ADDR_PRP3:
                return 0;
        case ADDR_FX:
        case ADDR_FY0:
        case ADDR_FY1:
        case ADDR_FZ:
        case ADDR_VSLF:
                return 0;
        case ADDR_BX:
        case ADDR_BY0:
        case ADDR_BY1:
        case ADDR_BZ:
        case ADDR_VSLB:
                return 0;
        case ADDR_LVL0:
        case ADDR_LVL1:
                return 0;
        case ADDR_MOM0:
        case ADDR_MOM1:
        case ADDR_MOM2:
        case ADDR_MOM3:
                return 0;
        case ADDR_SDT:
        case ADDR_SDS0:
        case ADDR_SDS1:
        case ADDR_SDS2:
        case ADDR_SDS3:
        case ADDR_SDF0:
        case ADDR_SDF1:
        case ADDR_SDF2:
        case ADDR_SDF3:
        case ADDR_SDB0:
        case ADDR_SDB1:
        case ADDR_SDB2:
        case ADDR_SDB3:
                return 0;
        default:
                return 0;
        }
}
