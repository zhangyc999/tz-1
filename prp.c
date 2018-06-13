#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200
#define PERIOD_FAST 8

#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_IO      0x000000FF
#define UNMASK_RESULT_FAULT   0x0000FF00
#define UNMASK_RESULT_RUNNING 0x00FF0000
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_POS      0x00000400
#define RESULT_FAULT_VEL      0x00000800
#define RESULT_FAULT_AMPR     0x00001000
#define RESULT_FAULT_SYNC     0x00002000
#define RESULT_FAULT_COMM     0x00004000
#define RESULT_RUNNING        0x00FF0000

#define MAX_LEN 20000 /* 200mm */
#define MAX_ACC 100   /* 1mm/s^2 */

#define PLAN_VEL_LOW  100  /* 1mm/s */
#define PLAN_VEL_HIGH 1500 /* 15mm/s */
#define PLAN_LEN_LOW  500  /* 5mm */
#define PLAN_LEN_AMPR 200  /* 2mm */

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_prp;

extern int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
extern s16 plan_vel(int vel_cur, int len_pass, int len_remain, int period);
extern int max_of_n(int *buf, int n);
extern int min_of_n(int *buf, int n);
extern struct frame_can *can_cllst_init(struct frame_can buf[], int len);

void t_prp(void) /* Task: PRoP */
{
        int period = PERIOD_SLOW;
        u32 prev;
        int len;
        int tmp[sizeof(struct frame_can)];
        struct main *cmd;
        struct main state;
        struct main state_old;
        struct frame_can *can;
        struct frame_can rx[4][3][MAX_LEN_CLLST];
        FRAME_RX *p[4][3];
        FRAME_TX tx[4];
        int verify = CMD_IDLE;
        int has_received[4] = {0};
        int n = 2;
        int i;
        int j;
        int max_form = 3;
        int addr[4] = {J1939_ADDR_PRP0, J1939_ADDR_PRP1, J1939_ADDR_PRP2, J1939_ADDR_PRP3};
        int cable[4] = {0, 0, 1, 1};
        int cur_vel[4] = {0};
        int sum_pos[4] = {0};
        int sum_vel[4] = {0};
        int sum_ampr[4] = {0};
        int avg_pos[4] = {0};
        int avg_vel[4] = {0};
        int avg_ampr[4] = {0};
        int fault_old[4] = {0};
        int io_old[4] = {0};
        int min_pos[4] = {0};
        int min_vel[4] = {-1000, -1000, -1000, -1000};
        int min_ampr[4] = {-1000, -1000, -1000, -1000};
        int max_pos[4] = {1000, 1000, 1000, 1000};
        int max_vel[4] = {1000, 1000, 1000, 1000};
        int max_ampr[4] = {1000, 1000, 1000, 1000};
        int safe_pos[4] = {10000, 10000, 10000, 10000};
        int err_sync = 20;
        int ctr_ok_pos[4] = {0};
        int ctr_ok_vel[4] = {0};
        int ctr_ok_ampr[4] = {0};
        int ctr_ok_sync = 0;
        int ctr_ok_stop[4] = {0};
        int ctr_err_pos[4] = {0};
        int ctr_err_vel[4] = {0};
        int ctr_err_ampr[4] = {0};
        int ctr_err_sync = 0;
        int ctr_err_stop[4] = {0};
        int ctr_fault[4] = {0};
        int ctr_io[4] = {0};
        int ctr_comm[4] = {0};
        int result[4] = {0};
        int tmp_pos;
        int tmp_vel;
        int tmp_ampr;
        int tmp_running;
        int tmp_sync;
        int sub;
        int len_remain = 0;
        int len_pass = 0;
        int max_len_posi = MAX_LEN;
        int max_len_nega = MAX_LEN;
        for (i = 0; i < n; i++) {
                for (j = 0; j < max_form; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_prp, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = (struct main *)&tmp;
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                switch (cmd->type) {
                                case CMD_IDLE:
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd->type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_POSI:
                                switch (cmd->type) {
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd->type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                switch (cmd->type) {
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd->type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                switch (cmd->type) {
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd->type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                switch (cmd->type) {
                                case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd->type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        default:
                                break;
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = (struct frame_can *)&tmp;
                        switch (can->src) {
                        case J1939_ADDR_PRP0:
                                i = 0;
                                break;
                        case J1939_ADDR_PRP1:
                                i = 1;
                                break;
                        case J1939_ADDR_PRP2:
                                i = 2;
                                break;
                        case J1939_ADDR_PRP3:
                                i = 3;
                                break;
                        default:
                                break;
                        }
                        has_received[i] = 1;
                        j = remap_form_index(can->form);
                        switch (j) {
                        case 0:
                        case 1:
                                break;
                        case 2:
                                p[i][j] = p[i][j]->next;
                                sum_pos[i] -= p[i][j]->data.state.pos;
                                sum_vel[i] -= p[i][j]->data.state.vel;
                                sum_ampr[i] -= p[i][j]->data.state.ampr;
                                fault_old[i] = p[i][j]->data.state.fault;
                                io_old[i] = p[i][j]->data.state.io;
                                p[i][j]->data.state.pos = ((FRAME_RX *)can)->data.state.pos;
                                p[i][j]->data.state.vel = ((FRAME_RX *)can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)can)->data.state.io;
                                cur_vel[i] = ((FRAME_RX *)can)->data.state.vel;
                                sum_pos[i] += p[i][j]->data.state.pos;
                                sum_vel[i] += p[i][j]->data.state.vel;
                                sum_ampr[i] += p[i][j]->data.state.ampr;
                                avg_pos[i] = sum_pos[i] / MAX_LEN_CLLST;
                                avg_vel[i] = sum_vel[i] / MAX_LEN_CLLST;
                                avg_ampr[i] = sum_ampr[i] / MAX_LEN_CLLST;
                                if (fault_old[i] == p[i][j]->data.state.fault) {
                                        if (ctr_fault[i] < 10)
                                                ctr_fault[i]++;
                                } else {
                                        ctr_fault[i] = 0;
                                }
                                switch (p[i][j]->data.state.fault) {
                                case 0x00:
                                case 0x03:
                                        if (ctr_fault[i] < 5)
                                                break;
                                        result[i] &= ~RESULT_FAULT_GENERAL;
                                        result[i] &= ~RESULT_FAULT_SERIOUS;
                                        break;
                                case 0x0C:
                                        if (ctr_fault[i] < 3)
                                                break;
                                        result[i] |= RESULT_FAULT_GENERAL;
                                        break;
                                case 0xF0:
                                        result[i] |= RESULT_FAULT_SERIOUS;
                                        break;
                                default:
                                        break;
                                }
                                if (io_old[i] == p[i][j]->data.state.io) {
                                        if (ctr_io[i] < 10)
                                                ctr_io[i]++;
                                } else {
                                        ctr_io[i] = 0;
                                }
                                if (ctr_io[i] > 5)
                                        result[i] = result[i] & ~UNMASK_RESULT_IO | p[i][j]->data.state.io;
                                tmp_pos = judge_filter(&ctr_ok_pos[i], &ctr_err_pos[i], avg_pos[i], min_pos[i], max_pos[i], 10);
                                tmp_vel = judge_filter(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], 10);
                                tmp_ampr = judge_filter(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], 10);
                                tmp_running = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, 10);
                                if (tmp_pos == -1)
                                        result[i] |= RESULT_FAULT_POS;
                                else if (tmp_pos == 1)
                                        result[i] &= ~RESULT_FAULT_POS;
                                if (tmp_vel == -1)
                                        result[i] |= RESULT_FAULT_VEL;
                                else if (tmp_vel == 1)
                                        result[i] &= ~RESULT_FAULT_VEL;
                                if (tmp_ampr == -1)
                                        result[i] |= RESULT_FAULT_AMPR;
                                else if (tmp_ampr == 1)
                                        result[i] &= ~RESULT_FAULT_AMPR;
                                if (tmp_running == -1)
                                        result[i] |= RESULT_RUNNING;
                                else if (tmp_running == 1)
                                        result[i] &= ~RESULT_RUNNING;
                                break;
                        default:
                                break;
                        }
                        sub = max_of_n(avg_pos, n) - min_of_n(avg_pos, n);
                        tmp_sync = judge_filter(&ctr_ok_sync, &ctr_err_sync, sub, -err_sync, err_sync, 10);
                        if (tmp_sync == -1) {
                                result[0] |= RESULT_FAULT_SYNC;
                                result[1] |= RESULT_FAULT_SYNC;
                                result[2] |= RESULT_FAULT_SYNC;
                                result[3] |= RESULT_FAULT_SYNC;
                        } else if (tmp_sync == 1) {
                                result[0] &= ~RESULT_FAULT_SYNC;
                                result[1] &= ~RESULT_FAULT_SYNC;
                                result[2] &= ~RESULT_FAULT_SYNC;
                                result[3] &= ~RESULT_FAULT_SYNC;
                        }
                        period -= tickGet() - prev;
                        break;
                default:
#if 0
                        for (i = 0; i < n; i++) {
                                if (has_received[i]) {
                                        has_received[i] = 0;
                                        if (ctr_comm[i] < 0)
                                                ctr_comm[i] = 0;
                                        if (ctr_comm[i] < 10)
                                                ctr_comm[i]++;
                                        if (ctr_comm[i] == 10)
                                                result[i] &= ~RESULT_FAULT_COMM;
                                } else {
                                        if (ctr_comm[i] > 0)
                                                ctr_comm[i] = 0;
                                        if (ctr_comm[i] > -10)
                                                ctr_comm[i]--;
                                        if (ctr_comm[i] == -10)
                                                result[i] |= RESULT_FAULT_COMM;
                                }
                        }
                        state.type = TASK_NOTIFY_PRP;
                        state.data = 0;
                        for (i = 0; i < n; i++) {
                                if (result[i] & UNMASK_RESULT_FAULT)
                                        break;
                        }
                        if (i != n) {
                                state.type |= TASK_STATE_FAULT;
                                verify = verify & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type |= TASK_STATE_OK;
                        }
                        for (i = 0; i < n; i++) {
                                if (avg_pos[i] < safe_pos[i])
                                        break;
                        }
                        if (i != n)
                                state.type |= TASK_STATE_DANGER;
                        else
                                state.type |= TASK_STATE_SAFE;
                        if (state_old.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_URGENT);
                        state_old = state;
#endif
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                max_len_posi = MAX_LEN - min_of_n(avg_pos, n) * 20;
                                max_len_nega = max_of_n(avg_pos, n) * 20;
                                break;
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                len_remain = MAX_LEN - min_of_n(avg_pos, n) * 20;
                                len_pass = max_len_posi - len_remain;
                                break;
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                len_remain = max_of_n(avg_pos, n) * 20;
                                len_pass = max_len_nega - len_remain;
                                break;
                        default:
                                break;
                        }
                        switch (verify) {
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0;
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        if ((result[i] & UNMASK_RESULT_RUNNING) == 0)
                                                tx[i].data.cmd.enable = J1939_SERVO_DISABLE;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                for (i = 0; i < n; i++) {
                                        if (tx[i].data.cmd.enable != J1939_SERVO_DISABLE)
                                                break;
                                }
                                if (i == 4)
                                        period = PERIOD_SLOW;
                                else
                                        period = PERIOD_FAST;
                                break;
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = plan_vel(abs(cur_vel[i]), len_pass, len_remain, PERIOD_FAST);
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = -plan_vel(abs(cur_vel[i]), len_pass, len_remain, PERIOD_FAST);
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                period = PERIOD_FAST;
                                break;
                        default:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_QUERY;
                                        tx[i].prio = J1939_PRIO_QUERY;
                                        tx[i].data.query[0] = 0x00;
                                        tx[i].data.query[1] = 0x11;
                                        tx[i].data.query[2] = 0x22;
                                        tx[i].data.query[3] = 0x33;
                                        tx[i].data.query[4] = 0x44;
                                        tx[i].data.query[5] = 0x55;
                                        tx[i].data.query[6] = 0x66;
                                        tx[i].data.query[7] = 0x77;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_NORMAL);
                                }
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}
