#include "j1939.h"
#include "type.h"
#include "vx.h"

extern MSG_Q_ID msg_gen;
extern MSG_Q_ID msg_psu;
extern MSG_Q_ID msg_swh;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_swv;
extern MSG_Q_ID msg_prp;

MSG_Q_ID remap_addr_msg(u8 addr)
{
        switch (addr) {
        case J1939_ADDR_GEND:
        case J1939_ADDR_GENS:
                return msg_gen;
        case J1939_ADDR_PSU:
                return msg_psu;
        case J1939_ADDR_SWH0:
        case J1939_ADDR_SWH1:
        case J1939_ADDR_SWH2:
        case J1939_ADDR_SWH3:
                return msg_swh;
        case J1939_ADDR_RSE0:
        case J1939_ADDR_RSE1:
        case J1939_ADDR_RSE2:
        case J1939_ADDR_RSE3:
                return msg_rse;
        case J1939_ADDR_SWV0:
        case J1939_ADDR_SWV1:
        case J1939_ADDR_SWV2:
        case J1939_ADDR_SWV3:
                return msg_swv;
        case J1939_ADDR_PRP0:
        case J1939_ADDR_PRP1:
        case J1939_ADDR_PRP2:
        case J1939_ADDR_PRP3:
                return msg_prp;
        case J1939_ADDR_FX:
        case J1939_ADDR_FY0:
        case J1939_ADDR_FY1:
        case J1939_ADDR_FZ:
        case J1939_ADDR_VSLF:
                return 0;
        case J1939_ADDR_BX:
        case J1939_ADDR_BY0:
        case J1939_ADDR_BY1:
        case J1939_ADDR_BZ:
        case J1939_ADDR_VSLB:
                return 0;
        case J1939_ADDR_LVL0:
        case J1939_ADDR_LVL1:
                return 0;
        case J1939_ADDR_MOM0:
        case J1939_ADDR_MOM1:
        case J1939_ADDR_MOM2:
        case J1939_ADDR_MOM3:
                return 0;
        case J1939_ADDR_SDT:
        case J1939_ADDR_SDS0:
        case J1939_ADDR_SDS1:
        case J1939_ADDR_SDS2:
        case J1939_ADDR_SDS3:
        case J1939_ADDR_SDF0:
        case J1939_ADDR_SDF1:
        case J1939_ADDR_SDF2:
        case J1939_ADDR_SDF3:
        case J1939_ADDR_SDB0:
        case J1939_ADDR_SDB1:
        case J1939_ADDR_SDB2:
        case J1939_ADDR_SDB3:
                return 0;
        default:
                return 0;
        }
}

int remap_form_index(u8 form)
{
        switch (form) {
        case 0xC0:
                return 0;
        case 0xC3:
                return 1;
        case 0xC6:
                return 2;
        default:
                return -1;
        }
}
