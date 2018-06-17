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
#define RESULT_FAULT_IO       0x00008000
#define RESULT_RUNNING        0x00FF0000

#define POS_IO_NEGA 5    /* 1mm */
#define POS_IO_POSI 2000 /* 400mm */

#define MAX_LEN 20000 /* 200mm */
#define MAX_ACC 100   /* 1mm/s^2 */

#define PLAN_VEL_LOW  100  /* 1mm/s */
#define PLAN_VEL_HIGH 1500 /* 15mm/s */
#define PLAN_LEN_LOW  1000 /* 10mm */
#define PLAN_LEN_AMPR 500  /* 5mm */

#define OVERSHOOT 1 /* 0.2mm */

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swh;

int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
int plan_vel(int vel_cur, int len_pass, int len_remain, int period);
int max_pos_of_n(int pos_cur[], int n, int overshoot);
int min_pos_of_n(int pos_cur[], int n, int overshoot);
struct frame_can *can_cllst_init(struct frame_can buf[], int len);

void t_swh(void) /* Task: SWing arm of Horizontal */
{
        int period = PERIOD_SLOW;
        u32 prev;
        int len;
        u8 tmp[sizeof(struct frame_can)];
        struct main cmd;
        struct main state;
        struct main state_old;
        struct frame_can can;
        struct frame_can rx[4][3][MAX_LEN_CLLST];
        FRAME_RX *p[4][3];
        FRAME_TX tx[4];
        int verify = CMD_IDLE;
        int has_received[4] = {0};
        int n = 4;
        int i;
        int j;
        int max_form = 3;
        int addr[4] = {J1939_ADDR_SWH0, J1939_ADDR_SWH1, J1939_ADDR_SWH2, J1939_ADDR_SWH3};
        int cable[4] = {0, 0, 1, 1};
        int pos_cur[4] = {0};
        int vel_cur[4] = {0};
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
        int err_sync_01 = 10;
        int err_sync_23 = 10;
        int err_sync_0123 = 20;
        int ctr_ok_pos[4] = {0};
        int ctr_ok_vel[4] = {0};
        int ctr_ok_ampr[4] = {0};
        int ctr_ok_sync_01 = 0;
        int ctr_ok_sync_23 = 0;
        int ctr_ok_sync_0123 = 0;
        int ctr_ok_stop[4] = {0};
        int ctr_err_pos[4] = {0};
        int ctr_err_vel[4] = {0};
        int ctr_err_ampr[4] = {0};
        int ctr_err_sync_01 = 0;
        int ctr_err_sync_23 = 0;
        int ctr_err_sync_0123 = 0;
        int ctr_err_stop[4] = {0};
        int ctr_fault[4] = {0};
        int ctr_io[4] = {0};
        int ctr_comm[4] = {0};
        int result[4] = {0};
        int tmp_pos;
        int tmp_vel;
        int tmp_ampr;
        int tmp_running;
        int tmp_sync_01;
        int tmp_sync_23;
        int tmp_sync_0123;
        int sub_01;
        int sub_23;
        int sub_0123;
        int len_remain = 0;
        int len_pass = 0;
        int max_len_posi = MAX_LEN;
        int max_len_nega = MAX_LEN;
        int slowest;
        for (i = 0; i < n; i++) {
                for (j = 0; j < max_form; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_swh, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                switch (cmd.type) {
                                case CMD_IDLE:
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
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
                        can = *(struct frame_can *)tmp;
                        switch (can.src) {
                        case J1939_ADDR_SWH0:
                                i = 0;
                                break;
                        case J1939_ADDR_SWH1:
                                i = 1;
                                break;
                        case J1939_ADDR_SWH2:
                                i = 2;
                                break;
                        case J1939_ADDR_SWH3:
                                i = 3;
                                break;
                        default:
                                break;
                        }
                        has_received[i] = 1;
                        j = remap_form_index(can.form);
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
                                p[i][j]->data.state.pos = ((FRAME_RX *)&can)->data.state.pos;
                                p[i][j]->data.state.vel = ((FRAME_RX *)&can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                pos_cur[i] = ((FRAME_RX *)&can)->data.state.pos;
                                vel_cur[i] = ((FRAME_RX *)&can)->data.state.vel;
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
                                if (avg_pos[i] > POS_IO_POSI && (result[i] & 0x0000000C) == 0x00000008
                                    || avg_pos[i] < POS_IO_NEGA && (result[i] & 0x0000000C) == 0x00000004
                                    || avg_pos[i] > POS_IO_NEGA + 10 && avg_pos[i] < POS_IO_POSI - 10 && (result[i] & 0x0000000C) != 0x0000000C)
                                        result[i] |= RESULT_FAULT_IO;
                                else
                                        result[i] &= ~RESULT_FAULT_IO;
                                if (tmp_running == -1)
                                        result[i] |= RESULT_RUNNING;
                                else if (tmp_running == 1)
                                        result[i] &= ~RESULT_RUNNING;
                                break;
                        default:
                                break;
                        }
                        sub_01 = avg_pos[0] - avg_pos[1];
                        sub_23 = avg_pos[2] - avg_pos[3];
                        sub_0123 = (avg_pos[0] - avg_pos[3] + avg_pos[1] - avg_pos[2]) / 2;
                        tmp_sync_01 = judge_filter(&ctr_ok_sync_01, &ctr_err_sync_01, sub_01, -err_sync_01, err_sync_01, 10);
                        tmp_sync_23 = judge_filter(&ctr_ok_sync_23, &ctr_err_sync_23, sub_23, -err_sync_23, err_sync_23, 10);
                        tmp_sync_0123 = judge_filter(&ctr_ok_sync_0123, &ctr_err_sync_0123, sub_0123, -err_sync_0123, err_sync_0123, 10);
                        if (tmp_sync_01 == -1) {
                                result[0] |= RESULT_FAULT_SYNC;
                                result[1] |= RESULT_FAULT_SYNC;
                        } else if (tmp_sync_01 == 1) {
                                result[0] &= ~RESULT_FAULT_SYNC;
                                result[1] &= ~RESULT_FAULT_SYNC;
                        }
                        if (tmp_sync_23 == -1) {
                                result[2] |= RESULT_FAULT_SYNC;
                                result[3] |= RESULT_FAULT_SYNC;
                        } else if (tmp_sync_23 == 1) {
                                result[2] &= ~RESULT_FAULT_SYNC;
                                result[3] &= ~RESULT_FAULT_SYNC;
                        }
                        if (tmp_sync_0123 == -1) {
                                result[0] |= RESULT_FAULT_SYNC;
                                result[1] |= RESULT_FAULT_SYNC;
                                result[2] |= RESULT_FAULT_SYNC;
                                result[3] |= RESULT_FAULT_SYNC;
                        } else if (tmp_sync_0123 == 1) {
                                result[0] &= ~RESULT_FAULT_SYNC;
                                result[1] &= ~RESULT_FAULT_SYNC;
                                result[2] &= ~RESULT_FAULT_SYNC;
                                result[3] &= ~RESULT_FAULT_SYNC;
                        }
                        period -= tickGet() - prev;
                        break;
                default:
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
                        state.type = TASK_NOTIFY_SWH;
                        state.data = 0;
                        for (i = 0; i < n; i++) {
                                if (result[i] & UNMASK_RESULT_FAULT)
                                        break;
                        }
                        if (i != n) {
                                state.type |= TASK_STATE_FAULT;
                                /* verify = verify & ~UNMASK_CMD_DIR | CMD_DIR_STOP; */
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
                        switch (verify) {
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                max_len_posi = MAX_LEN - pos_cur[min_pos_of_n(pos_cur, n, OVERSHOOT)] * 20;
                                max_len_nega = pos_cur[max_pos_of_n(pos_cur, n, OVERSHOOT)] * 20;
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
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                slowest = min_pos_of_n(pos_cur, n, OVERSHOOT);
                                len_remain = MAX_LEN - pos_cur[slowest] * 20;
                                len_pass = max_len_posi - len_remain;
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (pos_cur[i] > pos_cur[slowest])
                                                tx[i].data.cmd.vel = 0;
                                        else
                                                tx[i].data.cmd.vel =
                                                        (s16)(pos_cur[slowest] * 20  * sysClkRateGet() / PERIOD_FAST
                                                              + plan_vel(abs(vel_cur[i]), len_pass, len_remain, PERIOD_FAST)
                                                              - pos_cur[slowest] * 20 * sysClkRateGet() / PERIOD_FAST);
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                slowest = max_pos_of_n(pos_cur, n, OVERSHOOT);
                                len_remain = pos_cur[slowest] * 20;
                                len_pass = max_len_nega - len_remain;
                                if (len_remain > PLAN_LEN_AMPR) {
                                        for (i = 0; i < n; i++) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (pos_cur[i] < pos_cur[slowest])
                                                        tx[i].data.cmd.vel = 0;
                                                else
                                                        tx[i].data.cmd.vel =
                                                                (s16)(pos_cur[slowest] * 20 * sysClkRateGet() / PERIOD_FAST
                                                                      - plan_vel(abs(vel_cur[i]), len_pass, len_remain, PERIOD_FAST)
                                                                      - pos_cur[slowest] * 20 * sysClkRateGet() / PERIOD_FAST);
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                                tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                                msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                        }
                                } else {
                                        for (i = 0; i < n; i++) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_AMPR;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                tx[i].data.cmd.vel = 0x3322;
                                                tx[i].data.cmd.ampr = -50;
                                                tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                                tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                                msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                        }
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

int judge_filter(int *ok, int *err, int value, int min, int max, int ctr)
{
        if (value >= min && value < max) {
                *err = 0;
                if (*ok < ctr)
                        (*ok)++;
        } else {
                *ok = 0;
                if (*err < ctr)
                        (*err)++;
        }
        if (*ok == ctr)
                return 1;
        else if (*err == ctr)
                return -1;
        else
                return 0;
}

int plan_vel(int vel_cur, int len_pass, int len_remain, int period)
{
        int len_stop = 0;
        int vel_plan = 0;
        int delta = MAX_ACC * period / sysClkRateGet();
        if (vel_cur >= PLAN_VEL_LOW)
                len_stop = (int)((vel_cur * vel_cur - PLAN_VEL_LOW * PLAN_VEL_LOW) / 2.0 / MAX_ACC + PLAN_LEN_LOW + 0.5);
        else
                len_stop = 0;
        if (len_remain > len_stop) {
                if (len_pass < PLAN_LEN_LOW) {
                        vel_plan = PLAN_VEL_LOW;
                } else {
                        if (vel_cur < PLAN_VEL_HIGH - delta)
                                vel_plan = vel_cur + delta;
                        else
                                vel_plan = PLAN_VEL_HIGH;
                }
        } else {
                if (len_remain > PLAN_LEN_LOW) {
                        if (vel_cur > PLAN_VEL_LOW + delta)
                                vel_plan = vel_cur - delta;
                        else
                                vel_plan = PLAN_VEL_LOW;
                } else {
                        if (len_remain > 0)
                                vel_plan = PLAN_VEL_LOW;
                        else
                                vel_plan = 0;
                }
        }
        return vel_plan;
}

int max_pos_of_n(int pos_cur[], int n, int overshoot)
{
        int tmp;
        int i;
        int j = 0;
        for (i = 0; i < n; i++) {
                if (pos_cur[j] < pos_cur[i] - overshoot)
                        j = i;
        }
        return j;
}

int min_pos_of_n(int pos_cur[], int n, int overshoot)
{
        int tmp;
        int i;
        int j = 0;
        for (i = 0; i < n; i++) {
                if (pos_cur[j] > pos_cur[i] + overshoot)
                        j = i;
        }
        return j;
}

struct frame_can *can_cllst_init(struct frame_can buf[], int len)
{
        int i;
        for (i = 0; i < len - 1; i++) {
                bzero((char *)&buf[i], sizeof(buf[i]));
                buf[i].next = &buf[i + 1];
        }
        bzero((char *)&buf[i], sizeof(buf[i]));
        buf[i].next = &buf[0];
        return buf;
}
