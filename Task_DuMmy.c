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
extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_udp;
extern MSG_Q_ID msg_dbg;

static struct frame_can rx;
static struct frame_can tx[50];
static int pos[50];
static int vel[50];
static int ampr[50];
static int period[50];
static MSG_Q_ID msg;
static int i;

void t_dm0(void)
{
        pos[20] = -1000;
        pos[21] = 2000;
        for (;;) {
                msgQReceive(msg_can[0], (char *)&rx, sizeof(rx), WAIT_FOREVER);
                switch (rx.dest) {
                case J1939_ADDR_SWH0:
                        i = 0;
                        period[i] = 8;
                        break;
                case J1939_ADDR_SWH1:
                        i = 1;
                        period[i] = 8;
                        break;
                case J1939_ADDR_SWH2:
                        i = 2;
                        period[i] = 8;
                        break;
                case J1939_ADDR_SWH3:
                        i = 3;
                        period[i] = 8;
                        break;
                case J1939_ADDR_RSE0:
                        i = 4;
                        period[i] = 20;
                        break;
                case J1939_ADDR_RSE1:
                        i = 5;
                        period[i] = 20;
                        break;
                case J1939_ADDR_RSE2:
                        i = 6;
                        period[i] = 20;
                        break;
                case J1939_ADDR_RSE3:
                        i = 7;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV0:
                        i = 8;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV1:
                        i = 9;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV2:
                        i = 10;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV3:
                        i = 11;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP0:
                        i = 12;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP1:
                        i = 13;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP2:
                        i = 14;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP3:
                        i = 15;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FY0:
                        i = 16;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FY1:
                        i = 17;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BY0:
                        i = 18;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BY1:
                        i = 19;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FX:
                        i = 20;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BX:
                        i = 21;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FZ:
                        i = 22;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BZ:
                        i = 23;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM0:
                        i = 24;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM1:
                        i = 25;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM2:
                        i = 26;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM3:
                        i = 27;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDT:
                        i = 28;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS0:
                        i = 29;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS1:
                        i = 30;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS2:
                        i = 31;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS3:
                        i = 32;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF0:
                        i = 33;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF1:
                        i = 34;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF2:
                        i = 35;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF3:
                        i = 36;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB0:
                        i = 37;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB1:
                        i = 38;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB2:
                        i = 39;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB3:
                        i = 40;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PSU:
                        i = 41;
                        period[i] = 20;
                        break;
                case J1939_ADDR_VSLF:
                        i = 42;
                        period[i] = 20;
                        break;
                case J1939_ADDR_VSLB:
                        i = 43;
                        period[i] = 20;
                        break;
                case J1939_ADDR_LVL0:
                        i = 44;
                        period[i] = 20;
                        break;
                case J1939_ADDR_LVL1:
                        i = 45;
                        period[i] = 20;
                        break;
                case J1939_ADDR_GEND:
                        i = 46;
                        period[i] = 200;
                        break;
                case J1939_ADDR_GENS:
                        i = 47;
                        period[i] = 200;
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
                                tx[i].src = rx.dest;
                                tx[i].dest = rx.src;
                                tx[i].form = 0xC6;
                                tx[i].prio = 0x14;
                                vel[i] = *(s16 *)&rx.data[2];
                                pos[i] += vel[i] * period[i] / sysClkRateGet();
                                ampr[i] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[i].data[0] = (s16)(pos[i] / 20);
                                *(s16 *)&tx[i].data[2] = (s16)vel[i];
                                *(s16 *)&tx[i].data[4] = (s16)ampr[i];
                                tx[i].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[i].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[i].data[7] = 0x0C;
                                msg = remap_addr_msg(rx.dest);
                                msgQSend(msg, (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_udp, (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_dbg, (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
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

void t_dm1(void)
{
        for (;;) {
                msgQReceive(msg_can[1], (char *)&rx, sizeof(rx), WAIT_FOREVER);
                switch (rx.dest) {
                case J1939_ADDR_SWH0:
                        i = 0;
                        period[i] = 8;
                        break;
                case J1939_ADDR_SWH1:
                        i = 1;
                        period[i] = 8;
                        break;
                case J1939_ADDR_SWH2:
                        i = 2;
                        period[i] = 8;
                        break;
                case J1939_ADDR_SWH3:
                        i = 3;
                        period[i] = 8;
                        break;
                case J1939_ADDR_RSE0:
                        i = 4;
                        period[i] = 20;
                        break;
                case J1939_ADDR_RSE1:
                        i = 5;
                        period[i] = 20;
                        break;
                case J1939_ADDR_RSE2:
                        i = 6;
                        period[i] = 20;
                        break;
                case J1939_ADDR_RSE3:
                        i = 7;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV0:
                        i = 8;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV1:
                        i = 9;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV2:
                        i = 10;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SWV3:
                        i = 11;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP0:
                        i = 12;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP1:
                        i = 13;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP2:
                        i = 14;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PRP3:
                        i = 15;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FY0:
                        i = 16;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FY1:
                        i = 17;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BY0:
                        i = 18;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BY1:
                        i = 19;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FX:
                        i = 20;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BX:
                        i = 21;
                        period[i] = 20;
                        break;
                case J1939_ADDR_FZ:
                        i = 22;
                        period[i] = 20;
                        break;
                case J1939_ADDR_BZ:
                        i = 23;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM0:
                        i = 24;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM1:
                        i = 25;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM2:
                        i = 26;
                        period[i] = 20;
                        break;
                case J1939_ADDR_MOM3:
                        i = 27;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDT:
                        i = 28;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS0:
                        i = 29;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS1:
                        i = 30;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS2:
                        i = 31;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDS3:
                        i = 32;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF0:
                        i = 33;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF1:
                        i = 34;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF2:
                        i = 35;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDF3:
                        i = 36;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB0:
                        i = 37;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB1:
                        i = 38;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB2:
                        i = 39;
                        period[i] = 20;
                        break;
                case J1939_ADDR_SDB3:
                        i = 40;
                        period[i] = 20;
                        break;
                case J1939_ADDR_PSU:
                        i = 41;
                        period[i] = 20;
                        break;
                case J1939_ADDR_VSLF:
                        i = 42;
                        period[i] = 20;
                        break;
                case J1939_ADDR_VSLB:
                        i = 43;
                        period[i] = 20;
                        break;
                case J1939_ADDR_LVL0:
                        i = 44;
                        period[i] = 20;
                        break;
                case J1939_ADDR_LVL1:
                        i = 45;
                        period[i] = 20;
                        break;
                case J1939_ADDR_GEND:
                        i = 46;
                        period[i] = 200;
                        break;
                case J1939_ADDR_GENS:
                        i = 47;
                        period[i] = 200;
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
                                tx[i].src = rx.dest;
                                tx[i].dest = rx.src;
                                tx[i].form = 0xC6;
                                tx[i].prio = 0x14;
                                vel[i] = *(s16 *)&rx.data[2];
                                pos[i] += vel[i] * period[i] / sysClkRateGet();
                                ampr[i] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[i].data[0] = (s16)(pos[i] / 20);
                                *(s16 *)&tx[i].data[2] = (s16)vel[i];
                                *(s16 *)&tx[i].data[4] = (s16)ampr[i];
                                tx[i].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[i].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[i].data[7] = 0x0C;
                                msg = remap_addr_msg(rx.dest);
                                msgQSend(msg, (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_udp, (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_dbg, (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
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
