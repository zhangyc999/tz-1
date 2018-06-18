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

extern MSG_Q_ID remap_addr_msg(u8 addr);
extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_udp;
extern MSG_Q_ID msg_dbg;

void t_dm0(void)
{
        struct frame_can rx;
        struct frame_can tx[4];
        int pos[4] = {POS_ORIG_SWH0, POS_ORIG_SWH1, POS_ORIG_SWV0, POS_ORIG_SWV1};
        int vel[4] = {0};
        int ampr[4] = {0};
        MSG_Q_ID msg;
        bzero((char *)&tx[0], sizeof(tx[0]));
        bzero((char *)&tx[1], sizeof(tx[1]));
        bzero((char *)&tx[2], sizeof(tx[2]));
        bzero((char *)&tx[3], sizeof(tx[3]));
        *(s16 *)&tx[0].data[0] = (s16)(pos[0] / 20);
        *(s16 *)&tx[1].data[0] = (s16)(pos[1] / 20);
        *(s16 *)&tx[2].data[0] = (s16)(pos[2] / 20);
        *(s16 *)&tx[3].data[0] = (s16)(pos[3] / 20);
        tx[0].data[6] = 0x00;
        tx[1].data[6] = 0x00;
        tx[2].data[6] = 0x00;
        tx[3].data[6] = 0x00;
        tx[0].data[7] = 0x0C;
        tx[1].data[7] = 0x0C;
        tx[2].data[7] = 0x0C;
        tx[3].data[7] = 0x0C;
        for (;;) {
                msgQReceive(msg_can[0], (char *)&rx, sizeof(rx), WAIT_FOREVER);
                switch (rx.dest) {
                case J1939_ADDR_SWH0:
                        tx[0].src = rx.dest;
                        tx[0].dest = rx.src;
                        tx[0].form = 0xC6;
                        tx[0].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[0] = *(s16 *)(rx.data + 2);
                                pos[0] += vel[0] / 25;
                                ampr[0] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[0].data[0] = (s16)(pos[0] / 20);
                                *(s16 *)&tx[0].data[2] = (s16)vel[0];
                                *(s16 *)&tx[0].data[4] = (s16)ampr[0];
                                tx[0].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[0].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[0].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[0], sizeof(tx[0]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[0], sizeof(tx[0]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[0], sizeof(tx[0]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                case J1939_ADDR_SWH1:
                        tx[1].src = rx.dest;
                        tx[1].dest = rx.src;
                        tx[1].form = 0xC6;
                        tx[1].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[1] = *(s16 *)(rx.data + 2);
                                pos[1] += vel[1] / 25;
                                ampr[1] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[1].data[0] = (s16)(pos[1] / 20);
                                *(s16 *)&tx[1].data[2] = (s16)vel[1];
                                *(s16 *)&tx[1].data[4] = (s16)ampr[1];
                                tx[0].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[1].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[1].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[1], sizeof(tx[1]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[1], sizeof(tx[1]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[1], sizeof(tx[1]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                case J1939_ADDR_SWV0:
                        tx[2].src = rx.dest;
                        tx[2].dest = rx.src;
                        tx[2].form = 0xC6;
                        tx[2].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[2] = *(s16 *)(rx.data + 2);
                                pos[2] += vel[2] / 10;
                                ampr[2] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[2].data[0] = (s16)(pos[2] / 20);
                                *(s16 *)&tx[2].data[2] = (s16)vel[2];
                                *(s16 *)&tx[2].data[4] = (s16)ampr[2];
                                tx[2].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[2].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[2].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[2], sizeof(tx[2]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[2], sizeof(tx[2]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[2], sizeof(tx[2]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                case J1939_ADDR_SWV1:
                        tx[3].src = rx.dest;
                        tx[3].dest = rx.src;
                        tx[3].form = 0xC6;
                        tx[3].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[3] = *(s16 *)(rx.data + 2);
                                pos[3] += vel[3] / 10;
                                ampr[3] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[3].data[0] = (s16)(pos[3] / 20);
                                *(s16 *)&tx[3].data[2] = (s16)vel[3];
                                *(s16 *)&tx[3].data[4] = (s16)ampr[3];
                                tx[0].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[3].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[3].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[3], sizeof(tx[3]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[3], sizeof(tx[3]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[3], sizeof(tx[3]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                default:
                        break;
                }
                taskDelay(1);
        }
}

void t_dm1(void)
{
        struct frame_can rx;
        struct frame_can tx[4];
        int pos[4] = {POS_ORIG_SWH2, POS_ORIG_SWH3, POS_ORIG_SWV2, POS_ORIG_SWV3};
        int vel[4] = {0};
        int ampr[4] = {0};
        MSG_Q_ID msg;
        bzero((char *)&tx[0], sizeof(tx[0]));
        bzero((char *)&tx[1], sizeof(tx[1]));
        bzero((char *)&tx[2], sizeof(tx[2]));
        bzero((char *)&tx[3], sizeof(tx[3]));
        *(s16 *)&tx[0].data[0] = (s16)(pos[0] / 20);
        *(s16 *)&tx[1].data[0] = (s16)(pos[1] / 20);
        *(s16 *)&tx[2].data[0] = (s16)(pos[2] / 20);
        *(s16 *)&tx[3].data[0] = (s16)(pos[3] / 20);
        tx[0].data[6] = 0x00;
        tx[1].data[6] = 0x00;
        tx[2].data[6] = 0x00;
        tx[3].data[6] = 0x00;
        tx[0].data[7] = 0x0C;
        tx[1].data[7] = 0x0C;
        tx[2].data[7] = 0x0C;
        tx[3].data[7] = 0x0C;
        for (;;) {
                msgQReceive(msg_can[1], (char *)&rx, sizeof(rx), WAIT_FOREVER);
                switch (rx.dest) {
                case J1939_ADDR_SWH2:
                        tx[0].src = rx.dest;
                        tx[0].dest = rx.src;
                        tx[0].form = 0xC6;
                        tx[0].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[0] = *(s16 *)(rx.data + 2);
                                pos[0] += vel[0] / 25;
                                ampr[0] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[0].data[0] = (s16)(pos[0] / 20);
                                *(s16 *)&tx[0].data[2] = (s16)vel[0];
                                *(s16 *)&tx[0].data[4] = (s16)ampr[0];
                                tx[0].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[0].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[0].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[0], sizeof(tx[0]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[0], sizeof(tx[0]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[0], sizeof(tx[0]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                case J1939_ADDR_SWH3:
                        tx[1].src = rx.dest;
                        tx[1].dest = rx.src;
                        tx[1].form = 0xC6;
                        tx[1].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[1] = *(s16 *)(rx.data + 2);
                                pos[1] += vel[1] / 25;
                                ampr[1] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[1].data[0] = (s16)(pos[1] / 20);
                                *(s16 *)&tx[1].data[2] = (s16)vel[1];
                                *(s16 *)&tx[1].data[4] = (s16)ampr[1];
                                tx[0].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[1].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[1].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[1], sizeof(tx[1]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[1], sizeof(tx[1]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[1], sizeof(tx[1]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                case J1939_ADDR_SWV2:
                        tx[2].src = rx.dest;
                        tx[2].dest = rx.src;
                        tx[2].form = 0xC6;
                        tx[2].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[2] = *(s16 *)(rx.data + 2);
                                pos[2] += vel[2] / 10;
                                ampr[2] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[2].data[0] = (s16)(pos[2] / 20);
                                *(s16 *)&tx[2].data[2] = (s16)vel[2];
                                *(s16 *)&tx[2].data[4] = (s16)ampr[2];
                                tx[2].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[2].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[2].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[2], sizeof(tx[2]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[2], sizeof(tx[2]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[2], sizeof(tx[2]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                case J1939_ADDR_SWV3:
                        tx[3].src = rx.dest;
                        tx[3].dest = rx.src;
                        tx[3].form = 0xC6;
                        tx[3].prio = 0x14;
                        switch (rx.form) {
                        case 0x5C:
                                break;
                        case 0xA3:
                                break;
                        case 0xA5:
                                vel[3] = *(s16 *)(rx.data + 2);
                                pos[3] += vel[3] / 10;
                                ampr[3] = *(s16 *)&rx.data[4];
                                *(s16 *)&tx[3].data[0] = (s16)(pos[3] / 20);
                                *(s16 *)&tx[3].data[2] = (s16)vel[3];
                                *(s16 *)&tx[3].data[4] = (s16)ampr[3];
                                tx[0].data[6] = 0x00;
                                if (rx.data[7] == 0xC3)
                                        tx[3].data[7] = 0x1C;
                                else if (rx.data[7] == 0x3C)
                                        tx[3].data[7] = 0x0C;
                                break;
                        default:
                                break;
                        }
                        msg = remap_addr_msg(rx.dest);
                        msgQSend(msg, (char *)&tx[3], sizeof(tx[3]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_udp, (char *)&tx[3], sizeof(tx[3]), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_dbg, (char *)&tx[3], sizeof(tx[3]), NO_WAIT, MSG_PRI_NORMAL);
                        break;
                default:
                        break;
                }
                taskDelay(1);
        }
}
