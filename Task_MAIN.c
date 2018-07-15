#include "define.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define FLAG_LVL  0x00000001
#define FLAG_VSL  0x00000002
#define FLAG_GEN  0x00000004
#define FLAG_PSU  0x00000008
#define FLAG_MOM  0x00000010
#define FLAG_SWH  0x00000020
#define FLAG_RSE  0x00000040
#define FLAG_SWV  0x00000080
#define FLAG_PRP  0x00000100
#define FLAG_SDT  0x00000200
#define FLAG_SDS  0x00000400
#define FLAG_SDFB 0x00000800
#define FLAG_X    0x00001000
#define FLAG_Y    0x00002000
#define FLAG_Z    0x00004000

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_gen;
extern MSG_Q_ID msg_psu;
extern MSG_Q_ID msg_mom;
extern MSG_Q_ID msg_swh;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_swv;
extern MSG_Q_ID msg_prp;
extern MSG_Q_ID msg_sdt;
extern MSG_Q_ID msg_sds;
extern MSG_Q_ID msg_sdfb;
extern MSG_Q_ID msg_x;
extern MSG_Q_ID msg_y;
extern MSG_Q_ID msg_z;

static struct main rx;
static struct main tx;
static struct main verify = {CMD_IDLE, 0};
static int overtime;
static int tmp;
static int fault;

void t_main(void)
{
        for (;;) {
                if (sizeof(rx) != msgQReceive(msg_main, (char *)&rx, sizeof(rx), 20)) {
                        overtime++;
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
                case TASK_NOTIFY_MOM:
                        tmp = FLAG_MOM;
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
                case TASK_NOTIFY_SDT:
                        tmp = FLAG_SDT;
                        break;
                case TASK_NOTIFY_SDS:
                        tmp = FLAG_SDS;
                        break;
                case TASK_NOTIFY_SDFB:
                        tmp = FLAG_SDFB;
                        break;
                case TASK_NOTIFY_X:
                        tmp = FLAG_X;
                        break;
                case TASK_NOTIFY_Y:
                        tmp = FLAG_Y;
                        break;
                case TASK_NOTIFY_Z:
                        tmp = FLAG_Z;
                        break;
                case TASK_NOTIFY_LVL:
                        tmp = FLAG_LVL;
                        break;
                default:
                        break;
                }
                if (tmp) {
                        if ((rx.type & UNMASK_TASK_STATE) == TASK_STATE_FAULT)
                                fault &= ~tmp;
                        else
                                fault |= tmp;
                }
                switch (verify.type & UNMASK_CMD_ACT) {
                case CMD_IDLE:
                case CMD_ACT_GEND:
                case CMD_ACT_GENS:
                case CMD_ACT_PSU:
                case CMD_ACT_MOM:
                case CMD_ACT_SWH:
                case CMD_ACT_RSE:
                case CMD_ACT_SWV:
                case CMD_ACT_PRP:
                case CMD_ACT_SDT:
                case CMD_ACT_SDS:
                case CMD_ACT_SDFB:
                case CMD_ACT_X:
                case CMD_ACT_Y:
                case CMD_ACT_Z:
                        switch (rx.type & UNMASK_CMD_ACT) {
                        case CMD_IDLE:
                                verify = rx;
                                break;
                        case CMD_ACT_GEND:
                        case CMD_ACT_GENS:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_gen, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_PSU:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_psu, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_MOM:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_mom, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SWH:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_swh, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_RSE:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_rse, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SWV:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_swv, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_PRP:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_prp, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SDT:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_sdt, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SDS:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_sds, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_SDFB:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_sdfb, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_X:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_x, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_Y:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_y, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                break;
                        case CMD_ACT_Z:
                                verify = rx;
                                tx = verify;
                                msgQSend(msg_z, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
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
