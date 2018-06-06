#include "struct.h"
#include "type.h"
#include "vx.h"

#define UMASK_TASK_NOTIFY 0xFFFF0000
#define TASK_NOTIFY_UDP   0x5A010000
#define TASK_NOTIFY_PWR   0x5A020000
#define TASK_NOTIFY_SWH   0x5A100000

#define UMASK_TASK_STATE 0x0000FFFF
#define TASK_STATE_FAULT 0x000000E0
#define TASK_STATE_OK    0x000000D7

#define UMASK_CMD_ACT   0xFF00F00F
#define UMASK_CMD_MOD   0x00F00000
#define UMASK_CMD_DIR   0x000F0000
#define CMD_IDLE        0x00000000
#define CMD_ACT_GEND    0xEC001001
#define CMD_ACT_GENS    0xEC001002
#define CMD_ACT_PSU     0xEC001003
#define CMD_ACT_SWH     0xEC002001
#define CMD_MODE_AUTO   0x00000000
#define CMD_MODE_MANUAL 0x00E00000
#define CMD_DIR_POSI    0x00000000
#define CMD_DIR_NEGA    0x000C0000

#define FLAG_PWR 0x00000001
#define FLAG_SWH 0x00000002

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_pwr;
extern MSG_Q_ID msg_swh;

void t_main(void)
{
        struct main rx;
        struct main tx;
        u32 ot = 0;
        int tmp;
        int flag = 0;
        int verify = 0;
        for (;;) {
                if (sizeof(rx) != msgQReceive(msg_main, (char *)&rx, sizeof(rx), 20)) {
                        ot++;
                        /* TODO: Deal with child task overtime. */
                        continue;
                }
                tmp = 0;
                switch (rx.type & UMASK_TASK_NOTIFY) {
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
                        if ((rx.type & UMASK_TASK_STATE) == TASK_STATE_OK)
                                flag |=  tmp;
                        else
                                flag &= ~tmp;
                }
                if ((rx.type & UMASK_TASK_NOTIFY) != TASK_NOTIFY_UDP)
                        continue;
                switch (verify & UMASK_CMD_ACT) {
                case CMD_IDLE:
                        switch (rx.type & UMASK_CMD_ACT) {
                        case CMD_IDLE:
                        case CMD_ACT_GEND:
                        case CMD_ACT_GENS:
                        case CMD_ACT_PSU:
                        case CMD_ACT_SWH:
                                verify = rx.type;
                                break;
                        default:
                                break;
                        }
                        break;
                case CMD_ACT_GEND:
                case CMD_ACT_GENS:
                case CMD_ACT_PSU:
                        switch (rx.type & UMASK_CMD_ACT) {
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
                                break;
                        default:
                                break;
                        }
                        break;
                case CMD_ACT_SWH:
                        switch (rx.type & UMASK_CMD_ACT) {
                        case CMD_IDLE:
                        case CMD_ACT_GEND:
                        case CMD_ACT_GENS:
                        case CMD_ACT_PSU:
                                verify = rx.type;
                                break;
                        case CMD_ACT_SWH:
                                verify = rx.type;
                                tx.type = rx.type;
                                msgQSend(msg_swh, (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
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
