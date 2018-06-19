#include "define.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define FLAG_GEN 0x00000001
#define FLAG_PSU 0x00000002
#define FLAG_SWH 0x00000004
#define FLAG_RSE 0x00000008
#define FLAG_SWV 0x00000010
#define FLAG_PRP 0x00000020

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_gen;
extern MSG_Q_ID msg_psu;
extern MSG_Q_ID msg_swh;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_swv;
extern MSG_Q_ID msg_prp;
extern MSG_Q_ID msg_x;

void t_main(void)
{
        struct main rx;
        struct main tx;
        u32 ot = 0;
        int tmp;
        int flag = 0;
        int verify = CMD_IDLE;
        for (;;) {
                if (sizeof(rx) != msgQReceive(msg_main, (char *)&rx, sizeof(rx), 20)) {
                        ot++;
                        /* TODO: Deal with child task overtime. */
                        continue;
                }
                tmp = 0;
                switch (rx.type & UNMASK_TASK_NOTIFY) {
                case TASK_NOTIFY_GEN:
                        tmp = FLAG_GEN;
                        break;
                case TASK_NOTIFY_PSU:
                        tmp = FLAG_PSU;
                        break;
                case TASK_NOTIFY_SWH:
                        tmp = FLAG_SWH;
                        break;
                case TASK_NOTIFY_RSE:
                        tmp = FLAG_RSE;
                        break;
                case TASK_NOTIFY_SWV:
                        tmp = FLAG_SWV;
                        break;
                case TASK_NOTIFY_PRP:
                        tmp = FLAG_PRP;
                        break;
                default:
                        break;
                }
                if (tmp) {
                        if ((rx.type & UNMASK_TASK_STATE) == TASK_STATE_OK)
                                flag |=  tmp;
                        else
                                flag &= ~tmp;
                }
                switch (verify & UNMASK_CMD_ACT) {
                case CMD_IDLE:
                case CMD_ACT_GEND:
                case CMD_ACT_GENS:
                case CMD_ACT_PSU_24:
                case CMD_ACT_SWH:
                case CMD_ACT_RSE:
                case CMD_ACT_SWV:
                case CMD_ACT_PRP:
                case CMD_ACT_X:
                        switch (rx.type & UNMASK_CMD_ACT) {
                        case CMD_IDLE:
                                verify = rx.type;
                                break;
                        case CMD_ACT_GEND:
                        case CMD_ACT_GENS:
                                verify = rx.type;
                                tx.type = rx.type;
                                tx.data = rx.data;
                                msgQSend(msg_gen, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_PSU_24:
                                verify = rx.type;
                                tx.type = rx.type;
                                tx.data = rx.data;
                                msgQSend(msg_psu, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SWH:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_swh, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_RSE:
                                verify = rx.type;
                                tx.type = rx.type;
                                tx.data = rx.data;
                                msgQSend(msg_rse, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SWV:
                                verify = rx.type;
                                tx.type = rx.type;
                                tx.data = rx.data;
                                msgQSend(msg_swv, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_PRP:
                                verify = rx.type;
                                tx.type = rx.type;
                                tx.data = rx.data;
                                msgQSend(msg_prp, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_X:
                                verify = rx.type;
                                tx.type = rx.type;
                                tx.data = rx.data;
                                msgQSend(msg_x, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
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
