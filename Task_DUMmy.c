#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

MSG_Q_ID remap_addr_msg(u8 addr);
int remap_addr_index(u8 addr);
int remap_addr_period(u8 addr);

extern RING_ID rng_can_slow[];
extern RING_ID rng_can_fast[];
extern RING_ID rng_udp[];
extern RING_ID rng_dbg[];

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
        pos[11] = 2000;
        pos[13] = 2000;
        for (;;) {
                taskDelay(1);
                for (i = 0; i < 2; i++) {
                        if (sizeof(rx) != rngBufGet(rng_can_fast[i], (char *)&rx, sizeof(rx))) {
                                if (sizeof(rx) != rngBufGet(rng_can_slow[i], (char *)&rx, sizeof(rx)))
                                        continue;
                        }
#if 0
                        if (i == 0)
                                printf("\033[25;1HCAN0:%8d", rngNBytes(rng_can_fast[0]));
                        if (i == 1)
                                printf("\033[25;16HCAN1:%8d", rngNBytes(rng_can_fast[1]));
#endif
                        j = remap_addr_index(rx.dest);
                        period[j] = remap_addr_period(rx.dest);
                        switch (rx.dest) {
                        case J1939_ADDR_LVL0:
                        case J1939_ADDR_LVL1:
                                switch (rx.form) {
                                case 0x5C:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC3;
                                        tx[j].prio = 0x14;
                                        *(s16 *)&tx[j].data[0] = 3000;
                                        *(s16 *)&tx[j].data[2] = 3000;
                                        msg = remap_addr_msg(rx.dest);
                                        msgQSend(msg, (char *)&tx[j], sizeof(tx[j]), NO_WAIT, MSG_PRI_NORMAL);
                                        rngBufPut(rng_udp[0], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[0], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case J1939_ADDR_VSLF:
                        case J1939_ADDR_VSLB:
                                break;
                        case J1939_ADDR_GEND:
                                break;
                        case J1939_ADDR_GENS:
                                break;
                        case J1939_ADDR_PSU:
                                switch (rx.form) {
                                case 0x5C:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC0;
                                        tx[j].prio = 0x18;
                                        *(u32 *)&tx[j].data[0] = *(u32 *)&rx.data[0];
                                        *(u16 *)&tx[j].data[4] = *(u16 *)&rx.data[4];
                                        msg = remap_addr_msg(rx.dest);
                                        msgQSend(msg, (char *)&tx[j], sizeof(tx[j]), NO_WAIT, MSG_PRI_NORMAL);
                                        rngBufPut(rng_udp[0], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[0], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                case 0xA0:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC0;
                                        tx[j].prio = 0x14;
                                        *(u32 *)&tx[j].data[0] = *(u32 *)&rx.data[0];
                                        *(u16 *)&tx[j].data[4] = *(u16 *)&rx.data[4];
                                        msg = remap_addr_msg(rx.dest);
                                        msgQSend(msg, (char *)&tx[j], sizeof(tx[j]), NO_WAIT, MSG_PRI_NORMAL);
                                        rngBufPut(rng_udp[0], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[0], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case J1939_ADDR_SWH0:
                        case J1939_ADDR_SWH1:
                        case J1939_ADDR_SWV0:
                        case J1939_ADDR_SWV1:
                        case J1939_ADDR_PRP0:
                        case J1939_ADDR_PRP1:
                        case J1939_ADDR_FY0:
                        case J1939_ADDR_FY1:
                        case J1939_ADDR_FX:
                        case J1939_ADDR_FZ:
                                switch (rx.form) {
                                case 0x5C:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC6;
                                        tx[j].prio = 0x14;
                                        *(s16 *)&tx[j].data[0] = (s16)(pos[j] / 20);
                                        *(s16 *)&tx[j].data[2] = (s16)vel[j];
                                        *(s16 *)&tx[j].data[4] = (s16)ampr[j];
                                        msg = remap_addr_msg(rx.dest);
                                        msgQSend(msg, (char *)&tx[j], sizeof(tx[j]), NO_WAIT, MSG_PRI_NORMAL);
                                        rngBufPut(rng_udp[0], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[0], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                case 0xA5:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC6;
                                        tx[j].prio = 0x14;
                                        vel[j] = *(s16 *)&rx.data[2];
                                        pos[j] += vel[j] * period[j] / sysClkRateGet();
                                        if (pos[j] < -100)
                                                ampr[j] = (s16)pos[j];
                                        else
                                                ampr[j] = 0;
                                        if (vel[j] == 0)
                                                ampr[j] = 0;
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
                                        rngBufPut(rng_udp[0], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[0], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case J1939_ADDR_SWH2:
                        case J1939_ADDR_SWH3:
                        case J1939_ADDR_RSE0:
                        case J1939_ADDR_RSE1:
                        case J1939_ADDR_RSE2:
                        case J1939_ADDR_RSE3:
                        case J1939_ADDR_SWV2:
                        case J1939_ADDR_SWV3:
                        case J1939_ADDR_PRP2:
                        case J1939_ADDR_PRP3:
                        case J1939_ADDR_BY0:
                        case J1939_ADDR_BY1:
                        case J1939_ADDR_BX:
                        case J1939_ADDR_BZ:
                        case J1939_ADDR_MOM0:
                        case J1939_ADDR_MOM1:
                        case J1939_ADDR_MOM2:
                        case J1939_ADDR_MOM3:
                        case J1939_ADDR_TOP:
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
                                case 0x5C:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC6;
                                        tx[j].prio = 0x14;
                                        *(s16 *)&tx[j].data[0] = (s16)(pos[j] / 20);
                                        *(s16 *)&tx[j].data[2] = (s16)vel[j];
                                        *(s16 *)&tx[j].data[4] = (s16)ampr[j];
                                        msg = remap_addr_msg(rx.dest);
                                        msgQSend(msg, (char *)&tx[j], sizeof(tx[j]), NO_WAIT, MSG_PRI_NORMAL);
                                        rngBufPut(rng_udp[1], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[1], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                case 0xA5:
                                        tx[j].src = rx.dest;
                                        tx[j].dest = rx.src;
                                        tx[j].form = 0xC6;
                                        tx[j].prio = 0x14;
                                        vel[j] = *(s16 *)&rx.data[2];
                                        pos[j] += vel[j] * period[j] / sysClkRateGet();
                                        if (pos[j] < -100)
                                                ampr[j] = (s16)pos[j];
                                        else
                                                ampr[j] = 0;
                                        if (vel[j] == 0)
                                                ampr[j] = 0;
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
                                        rngBufPut(rng_udp[1], (char *)&tx[j], sizeof(tx[j]));
                                        rngBufPut(rng_dbg[1], (char *)&tx[j], sizeof(tx[j]));
                                        break;
                                default:
                                        break;
                                }
                                break;
                        default:
                                break;
                        }
                }
        }
}
