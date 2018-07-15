#include "j1939.h"
#include "type.h"
#include "vx.h"

extern MSG_Q_ID msg_lvl;
extern MSG_Q_ID msg_vsl;
extern MSG_Q_ID msg_gen;
extern MSG_Q_ID msg_psu;
extern MSG_Q_ID msg_mom;
extern MSG_Q_ID msg_swh;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_swv;
extern MSG_Q_ID msg_prp;
extern MSG_Q_ID msg_top;
extern MSG_Q_ID msg_shd;
extern MSG_Q_ID msg_x;
extern MSG_Q_ID msg_y;
extern MSG_Q_ID msg_z;

MSG_Q_ID remap_addr_msg(u8 addr)
{
        switch (addr) {
        case J1939_ADDR_LVL0:
        case J1939_ADDR_LVL1:
                return msg_lvl;
        case J1939_ADDR_VSLF:
        case J1939_ADDR_VSLB:
                return msg_vsl;
        case J1939_ADDR_GEND:
        case J1939_ADDR_GENS:
                return msg_gen;
        case J1939_ADDR_PSU:
                return msg_psu;
        case J1939_ADDR_MOM0:
        case J1939_ADDR_MOM1:
        case J1939_ADDR_MOM2:
        case J1939_ADDR_MOM3:
                return msg_mom;
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
        case J1939_ADDR_TOP:
                return msg_top;
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
                return msg_shd;
        case J1939_ADDR_FX:
        case J1939_ADDR_BX:
                return msg_x;
        case J1939_ADDR_FY0:
        case J1939_ADDR_FY1:
        case J1939_ADDR_BY0:
        case J1939_ADDR_BY1:
                return msg_y;
        case J1939_ADDR_FZ:
        case J1939_ADDR_BZ:
                return msg_z;
        default:
                return 0;
        }
}

int remap_addr_index(u8 addr)
{
        switch (addr) {
        case J1939_ADDR_VSLF:
                return 0;
        case J1939_ADDR_VSLB:
                return 1;
        case J1939_ADDR_LVL0:
                return 2;
        case J1939_ADDR_LVL1:
                return 3;
        case J1939_ADDR_FZ:
                return 4;
        case J1939_ADDR_BZ:
                return 5;
        case J1939_ADDR_FY0:
                return 6;
        case J1939_ADDR_FY1:
                return 7;
        case J1939_ADDR_BY0:
                return 8;
        case J1939_ADDR_BY1:
                return 9;
        case J1939_ADDR_FX:
                return 10;
        case J1939_ADDR_BX:
                return 11;
        case J1939_ADDR_PRP0:
                return 12;
        case J1939_ADDR_PRP1:
                return 13;
        case J1939_ADDR_PRP2:
                return 14;
        case J1939_ADDR_PRP3:
                return 15;
        case J1939_ADDR_RSE0:
                return 16;
        case J1939_ADDR_RSE1:
                return 17;
        case J1939_ADDR_RSE2:
                return 18;
        case J1939_ADDR_RSE3:
                return 19;
        case J1939_ADDR_SWV0:
                return 20;
        case J1939_ADDR_SWV1:
                return 21;
        case J1939_ADDR_SWV2:
                return 22;
        case J1939_ADDR_SWV3:
                return 23;
        case J1939_ADDR_SWH0:
                return 24;
        case J1939_ADDR_SWH1:
                return 25;
        case J1939_ADDR_SWH2:
                return 26;
        case J1939_ADDR_SWH3:
                return 27;
        case J1939_ADDR_GEND:
                return 28;
        case J1939_ADDR_GENS:
                return 29;
        case J1939_ADDR_PSU:
                return 30;
        case J1939_ADDR_MOM0:
                return 32;
        case J1939_ADDR_MOM1:
                return 33;
        case J1939_ADDR_MOM2:
                return 34;
        case J1939_ADDR_MOM3:
                return 35;
        case J1939_ADDR_TOP:
                return 36;
        case J1939_ADDR_SDS0:
                return 37;
        case J1939_ADDR_SDS1:
                return 38;
        case J1939_ADDR_SDS2:
                return 39;
        case J1939_ADDR_SDS3:
                return 40;
        case J1939_ADDR_SDF0:
                return 41;
        case J1939_ADDR_SDF1:
                return 42;
        case J1939_ADDR_SDF2:
                return 43;
        case J1939_ADDR_SDF3:
                return 44;
        case J1939_ADDR_SDB0:
                return 45;
        case J1939_ADDR_SDB1:
                return 46;
        case J1939_ADDR_SDB2:
                return 47;
        case J1939_ADDR_SDB3:
                return 48;
        default:
                return -1;
        }
}

int remap_addr_period(u8 addr)
{
        switch (addr) {
        case J1939_ADDR_LVL0:
        case J1939_ADDR_LVL1:
        case J1939_ADDR_VSLF:
        case J1939_ADDR_VSLB:
        case J1939_ADDR_GEND:
        case J1939_ADDR_GENS:
        case J1939_ADDR_PSU:
                return sysClkRateGet();
        case J1939_ADDR_MOM0:
        case J1939_ADDR_MOM1:
        case J1939_ADDR_MOM2:
        case J1939_ADDR_MOM3:
                return sysClkRateGet() / 10;
        case J1939_ADDR_SWH0:
        case J1939_ADDR_SWH1:
        case J1939_ADDR_SWH2:
        case J1939_ADDR_SWH3:
                return sysClkRateGet() / 25;
        case J1939_ADDR_RSE0:
        case J1939_ADDR_RSE1:
        case J1939_ADDR_RSE2:
        case J1939_ADDR_RSE3:
                return sysClkRateGet() / 10;
        case J1939_ADDR_SWV0:
        case J1939_ADDR_SWV1:
        case J1939_ADDR_SWV2:
        case J1939_ADDR_SWV3:
                return sysClkRateGet() / 25;
        case J1939_ADDR_PRP0:
        case J1939_ADDR_PRP1:
        case J1939_ADDR_PRP2:
        case J1939_ADDR_PRP3:
                return sysClkRateGet() / 10;
        case J1939_ADDR_TOP:
                return sysClkRateGet() / 10;
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
                return sysClkRateGet() / 10;
        case J1939_ADDR_FX:
        case J1939_ADDR_BX:
        case J1939_ADDR_FY0:
        case J1939_ADDR_FY1:
        case J1939_ADDR_BY0:
        case J1939_ADDR_BY1:
                return sysClkRateGet() / 25;
        case J1939_ADDR_FZ:
        case J1939_ADDR_BZ:
                return sysClkRateGet() / 10;
        default:
                return -1;
        }
}
