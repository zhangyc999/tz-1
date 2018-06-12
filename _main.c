#include "define.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define FLAG_PWR 0x00000001
#define FLAG_SWH 0x00000002

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_pwr;
extern MSG_Q_ID msg_swh;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_swv;
extern MSG_Q_ID msg_prp;

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
                case TASK_NOTIFY_PWR:
                        tmp = FLAG_PWR;
                        break;
                case TASK_NOTIFY_SWH:
                        tmp = FLAG_SWH;
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
                case CMD_ACT_PSU:
                case CMD_ACT_SWH:
                case CMD_ACT_RSE:
                case CMD_ACT_SWV:
                case CMD_ACT_PRP:
                        switch (rx.type & UNMASK_CMD_ACT) {
                        case CMD_IDLE:
                                verify = rx.type;
                                break;
                        case CMD_ACT_GEND:
                        case CMD_ACT_GENS:
                        case CMD_ACT_PSU:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_pwr, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SWH:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_swh, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_RSE:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_rse, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SWV:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_swv, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_PRP:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_prp, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
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
