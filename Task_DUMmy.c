#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define POS_ORIG_SWH0 -500
#define POS_ORIG_SWH1 -500
#define POS_ORIG_SWH2 -500
#define POS_ORIG_SWH3 -500
#define POS_ORIG_SWV0 -500
#define POS_ORIG_SWV1 -500
#define POS_ORIG_SWV2 -500
#define POS_ORIG_SWV3 -500
#define POS_ORIG_FX   0
#define POS_ORIG_BX   40000

extern MSG_Q_ID remap_addr_msg(u8 addr);
extern RING_ID rng_can[];
extern RING_ID rng_udp;
extern RING_ID rng_dbg;

static struct frame_can rx;
static struct frame_can tx[50];
static int pos[50];
static int vel[50];
static int ampr[50];
static int period[50];
static MSG_Q_ID msg;
static int i;
static int j;

void t_dum(void)
{
        pos[20] = -1000;
        pos[21] = 2000;
        for (;;) {
                taskDelay(1);
                for (i = 0; i < 2; i++) {
                        if (sizeof(rx) != rngBufGet(rng_can[i], (char *)&rx, sizeof(rx)))
                                continue;
                        switch (rx.dest) {
                        case J1939_ADDR_SWH0:
                                j = 0;
                                period[j] = 8;
                                break;
                        case J1939_ADDR_SWH1:
                                j = 1;
                                period[j] = 8;
                                break;
                        case J1939_ADDR_SWH2:
                                j = 2;
                                period[j] = 8;
                                break;
                        case J1939_ADDR_SWH3:
                                j = 3;
                                period[j] = 8;
                                break;
                        case J1939_ADDR_RSE0:
                                j = 4;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_RSE1:
                                j = 5;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_RSE2:
                                j = 6;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_RSE3:
                                j = 7;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SWV0:
                                j = 8;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SWV1:
                                j = 9;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SWV2:
                                j = 10;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SWV3:
                                j = 11;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_PRP0:
                                j = 12;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_PRP1:
                                j = 13;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_PRP2:
                                j = 14;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_PRP3:
                                j = 15;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_FY0:
                                j = 16;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_FY1:
                                j = 17;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_BY0:
                                j = 18;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_BY1:
                                j = 19;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_FX:
                                j = 20;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_BX:
                                j = 21;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_FZ:
                                j = 22;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_BZ:
                                j = 23;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_MOM0:
                                j = 24;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_MOM1:
                                j = 25;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_MOM2:
                                j = 26;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_MOM3:
                                j = 27;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDT:
                                j = 28;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDS0:
                                j = 29;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDS1:
                                j = 30;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDS2:
                                j = 31;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDS3:
                                j = 32;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDF0:
                                j = 33;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDF1:
                                j = 34;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDF2:
                                j = 35;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDF3:
                                j = 36;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDB0:
                                j = 37;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDB1:
                                j = 38;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDB2:
                                j = 39;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_SDB3:
                                j = 40;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_PSU:
                                j = 41;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_VSLF:
                                j = 42;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_VSLB:
                                j = 43;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_LVL0:
                                j = 44;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_LVL1:
                                j = 45;
                                period[j] = 20;
                                break;
                        case J1939_ADDR_GEND:
                                j = 46;
                                period[j] = 200;
                                break;
                        case J1939_ADDR_GENS:
                                j = 47;
                                period[j] = 200;
                                break;
                        default:
                                break;
                        }
                        switch (rx.dest) {
                        case J1939_ADDR_SWH0:
                        case J1939_ADDR_SWH1:
                        case J1939_ADDR_SWH2:
                        case J1939_ADDR_SWH3:
                        case J1939_ADDR_RSE0:
                        case J1939_ADDR_RSE1:
                        case J1939_ADDR_RSE2:
                        case J1939_ADDR_RSE3:
                        case J1939_ADDR_SWV0:
                        case J1939_ADDR_SWV1:
                        case J1939_ADDR_SWV2:
                        case J1939_ADDR_SWV3:
                        case J1939_ADDR_PRP0:
                        case J1939_ADDR_PRP1:
                        case J1939_ADDR_PRP2:
                        case J1939_ADDR_PRP3:
                        case J1939_ADDR_FY0:
                        case J1939_ADDR_FY1:
                        case J1939_ADDR_BY0:
                        case J1939_ADDR_BY1:
                        case J1939_ADDR_FX:
                        case J1939_ADDR_BX:
                        case J1939_ADDR_FZ:
                        case J1939_ADDR_BZ:
                        case J1939_ADDR_MOM0:
                        case J1939_ADDR_MOM1:
                        case J1939_ADDR_MOM2:
                        case J1939_ADDR_MOM3:
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
                                switch (rx.form) {
                                case 0xA5:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC6;
                                        tx[j].prio = 0x14;
                                        vel[j] = *(s16 *)&rx.data[2];
                                        pos[j] += vel[j] * period[j] / sysClkRateGet();
                                        ampr[j] = *(s16 *)&rx.data[4];
                                        *(s16 *)&tx[j].data[0] = (s16)(pos[j] / 20);
                                        *(s16 *)&tx[j].data[2] = (s16)vel[j];
                                        *(s16 *)&tx[j].data[4] = (s16)ampr[j];
                                        tx[j].data[6] = 0x00;
                                        if (rx.data[7] == 0xC3)
                                                tx[j].data[7] = 0x1C;
                                        else if (rx.data[7] == 0x3C)
                                                tx[j].data[7] = 0x0C;
                                        msg = remap_addr_msg(rx.dest);
                                        msgQSend(msg, (char *)&tx[j], sizeof(tx[j]), NO_WAIT, MSG_PRI_NORMAL);
                                        rngBufPut(rng_udp, (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg, (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case J1939_ADDR_PSU:
                                break;
                        case J1939_ADDR_GEND:
                                break;
                        case J1939_ADDR_GENS:
                                break;
                        case J1939_ADDR_VSLF:
                                break;
                        case J1939_ADDR_VSLB:
                                break;
                        case J1939_ADDR_LVL0:
                                break;
                        case J1939_ADDR_LVL1:
                                break;
                        default:
                                break;
                        }
                }
        }
}
