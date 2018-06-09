#include "addr.h"
#include "define.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200
#define PERIOD_FAST 8

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

#define VEL_PLAN_LOW  100  /* 1mm/s */
#define VEL_PLAN_HIGH 1000 /* 10mm/s */
#define LEN_PLAN_LOW  1000 /* 10mm */
#define LEN_PLAN_MOM  300  /* 3mm */

extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swh;

int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
s16 plan_vel(int vel_cur, int delta, int len_pass, int len_remain);
int max_of_n(int *buf, int n);
int min_of_n(int *buf, int n);
struct frame_can *can_cllst_init(struct frame_can buf[], int len);

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

void t_swh(void) /* Task: SWing arm of Horizontal */
{
        int period = PERIOD_SLOW;
        int delay = period;
        u32 prev;
        int len;
        int tmp[sizeof(struct frame_can)];
        struct main cmd;
        struct main state;
        struct main state_old;
        FRAME_RX can;
        struct frame_can rx[4][3][16];
        FRAME_RX *p[4][3];
        FRAME_TX tx[4];
        int verify = CMD_IDLE;
        int has_received[4] = {0};
        int n = 1;
        int i;
        int j;
        int max_form = 3;
        int addr[4] = {ADDR_SWH0, ADDR_SWH1, ADDR_SWH2, ADDR_SWH3};
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
        int result_pos = 0;
        int result_vel = 0;
        int result_ampr = 0;
        int result_running = 0;
        int result_sync_01 = 0;
        int result_sync_23 = 0;
        int result_sync_0123 = 0;
        int sub_01 = 0;
        int sub_23 = 0;
        int sub_0123 = 0;
        int async = 0;
        int len_remain = 0;
        int len_pass = 0;
        int max_len_posi = MAX_LEN;
        int max_len_nega = MAX_LEN;
        int len_passed_posi = 0;
        int len_passed_nega = 0;
        for (i = 0; i < n; i++) {
                for (j = 0; j < max_form; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], 16);
        }
        for (;;) {
                prev = tickGet();
                if (delay < 0 || delay > period)
                        delay = 0;
                len = msgQReceive(msg_swh, (char *)&tmp, sizeof(tmp), delay);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)&tmp;
                        switch (verify) {
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
                        delay -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = *(FRAME_RX *)&tmp;
                        switch (can.src) {
                        case ADDR_SWH0:
                                i = 0;
                                break;
                        case ADDR_SWH1:
                                i = 1;
                                break;
                        case ADDR_SWH2:
                                i = 2;
                                break;
                        case ADDR_SWH3:
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
                                p[i][j]->data.state.pos = can.data.state.pos;
                                p[i][j]->data.state.vel = can.data.state.vel;
                                p[i][j]->data.state.ampr = can.data.state.ampr;
                                p[i][j]->data.state.fault = can.data.state.fault;
                                p[i][j]->data.state.io = can.data.state.io;
                                cur_vel[i] = can.data.state.vel;
                                sum_pos[i] += p[i][j]->data.state.pos;
                                sum_vel[i] += p[i][j]->data.state.vel;
                                sum_ampr[i] += p[i][j]->data.state.ampr;
                                avg_pos[i] = sum_pos[i] / 16;
                                avg_vel[i] = sum_vel[i] / 16;
                                avg_ampr[i] = sum_ampr[i] / 16;
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
                                result_pos = judge_filter(&ctr_ok_pos[i], &ctr_err_pos[i], avg_pos[i], min_pos[i], max_pos[i], 10);
                                result_vel = judge_filter(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], 10);
                                result_ampr = judge_filter(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], 10);
                                result_running = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, 10);
                                if (result_pos == -1)
                                        result[i] |= RESULT_FAULT_POS;
                                else if (result_pos == 1)
                                        result[i] &= ~RESULT_FAULT_POS;
                                if (result_vel == -1)
                                        result[i] |= RESULT_FAULT_VEL;
                                else if (result_vel == 1)
                                        result[i] &= ~RESULT_FAULT_VEL;
                                if (result_ampr == -1)
                                        result[i] |= RESULT_FAULT_AMPR;
                                else if (result_ampr == 1)
                                        result[i] &= ~RESULT_FAULT_AMPR;
                                if (result_running == -1)
                                        result[i] |= RESULT_RUNNING;
                                else if (result_running == 1)
                                        result[i] &= ~RESULT_RUNNING;
                        default:
                                break;
                        }
                        sub_01 = avg_pos[0] - avg_pos[1];
                        sub_23 = avg_pos[2] - avg_pos[3];
                        sub_0123 = (avg_pos[0] - avg_pos[3] + avg_pos[1] - avg_pos[2]) / 2;
                        result_sync_01 = judge_filter(&ctr_ok_sync_01, &ctr_err_sync_01, sub_01, -err_sync_01, err_sync_01, 10);
                        result_sync_23 = judge_filter(&ctr_ok_sync_23, &ctr_err_sync_23, sub_23, -err_sync_23, err_sync_23, 10);
                        result_sync_0123 = judge_filter(&ctr_ok_sync_0123, &ctr_err_sync_0123, sub_0123, -err_sync_0123, err_sync_0123, 10);
                        if (result_sync_01 == -1) {
                                result[0] |= RESULT_FAULT_SYNC;
                                result[1] |= RESULT_FAULT_SYNC;
                        } else if (result_sync_01 == 1) {
                                result[0] &= ~RESULT_FAULT_SYNC;
                                result[1] &= ~RESULT_FAULT_SYNC;
                        }
                        if (result_sync_23 == -1) {
                                result[2] |= RESULT_FAULT_SYNC;
                                result[3] |= RESULT_FAULT_SYNC;
                        } else if (result_sync_23 == 1) {
                                result[2] &= ~RESULT_FAULT_SYNC;
                                result[3] &= ~RESULT_FAULT_SYNC;
                        }
                        if (result_sync_0123 == -1) {
                                result[0] |= RESULT_FAULT_SYNC;
                                result[1] |= RESULT_FAULT_SYNC;
                                result[2] |= RESULT_FAULT_SYNC;
                                result[3] |= RESULT_FAULT_SYNC;
                        } else if (result_sync_0123 == 1) {
                                result[0] &= ~RESULT_FAULT_SYNC;
                                result[1] &= ~RESULT_FAULT_SYNC;
                                result[2] &= ~RESULT_FAULT_SYNC;
                                result[3] &= ~RESULT_FAULT_SYNC;
                        }
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                max_len_posi = MAX_LEN - min_of_n(avg_pos, n) * 20;
                                max_len_nega = max_of_n(avg_pos, n) * 20;
                                len_passed_posi = min_of_n(avg_pos, n) * 20;
                                len_passed_nega = MAX_LEN - max_of_n(avg_pos, n) * 20;
                                printf("%8d  %8d  %8d  %8d\n", max_len_posi, max_len_nega, len_passed_posi, len_passed_nega);
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                len_pass = min_of_n(avg_pos, n) * 20 - len_passed_posi;
                                len_remain = max_len_posi - len_pass;
                                printf("%8d  %8d  ", len_pass, len_remain);
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                len_remain = max_of_n(avg_pos, n) * 20;
                                len_pass = max_len_nega - len_remain;
                                printf("%8d  %8d  ", len_pass, len_remain);
                                break;
                        default:
                                break;
                        }
                        delay -= tickGet() - prev;
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
                        for (i = 0; i < n; i++) {
                                if (result[i] & UNMASK_RESULT_FAULT)
                                        break;
                        }
                        if (i != n) {
                                state.type = TASK_NOTIFY_SWH | TASK_STATE_FAULT;
                                state.data = 0;
                                if (state_old.type != state.type)
                                        msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_URGENT);
                                state_old = state;
                                verify = verify & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_NOTIFY_SWH | TASK_STATE_OK;
                                state.data = 0;
                                if (state_old.type != state.type)
                                        msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_URGENT);
                                state_old = state;
                        }
#endif
                        switch (verify) {
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0;
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        if ((result[i] & UNMASK_RESULT_RUNNING) == 0)
                                                tx[i].data.cmd.enable = 0x3C;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                for (i = 0; i < n; i++) {
                                        if (tx[i].data.cmd.enable != 0x3C)
                                                break;
                                }
                                if (i == 4)
                                        delay = PERIOD_SLOW;
                                else
                                        delay = PERIOD_FAST;
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = plan_vel(abs(cur_vel[i]), 50, len_pass, len_remain);
                                        printf("%d\n", tx[i].data.cmd.vel);
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        tx[i].data.cmd.enable = 0xC3;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                delay = PERIOD_FAST;
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = -plan_vel(abs(cur_vel[i]), 50, len_pass, len_remain);
                                        printf("%d\n", tx[i].data.cmd.vel);
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        tx[i].data.cmd.enable = 0xC3;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                delay = PERIOD_FAST;
                                break;
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                delay = PERIOD_FAST;
                                break;
                        case CMD_IDLE:
                        default:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0x5C;
                                        tx[i].prio = 0x0C;
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
                                delay = PERIOD_SLOW;
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

s16 plan_vel(int vel_cur, int delta, int len_pass, int len_remain)
{
        int len_stop = 0;
        s16 vel_plan = 0;
        if (vel_cur > VEL_PLAN_LOW)
                len_stop = (int)((vel_cur * vel_cur - VEL_PLAN_LOW * VEL_PLAN_LOW) / 2.0 / MAX_ACC + LEN_PLAN_LOW + 0.5);
        else
                len_stop = 0;
 /*       printf("%8d %8d %8d\n", len_remain, len_stop, vel_cur);*/
        if (len_remain > len_stop) {
                if (len_pass < LEN_PLAN_LOW) {
                        vel_plan = VEL_PLAN_LOW;
                } else {
                        if (vel_cur < VEL_PLAN_HIGH - delta)
                                vel_plan = vel_cur + delta;
                        else
                                vel_plan = VEL_PLAN_HIGH;
                }
        } else {
                if (len_remain > LEN_PLAN_LOW) {
                        if (vel_cur > VEL_PLAN_LOW + delta)
                                vel_plan = vel_cur - delta;
                        else
                                vel_plan = VEL_PLAN_LOW;
                } else {
                        if (len_remain > 0)
                                vel_plan = VEL_PLAN_LOW;
                        else
                                vel_plan = 0;
                }
        }
        return vel_plan;
}

int max_of_n(int *buf, int n)
{
        if (n > 1)
                return max(*buf, max_of_n(buf + 1, n - 1));
        return *buf;
}

int min_of_n(int *buf, int n)
{
        if (n > 1)
                return min(*buf, min_of_n(buf + 1, n - 1));
        return *buf;
}

struct frame_can *can_cllst_init(struct frame_can buf[], int len)
{
        int i = 0;
        for (i = 0; i < len - 1; i++) {
                bzero((char *)&buf[i], sizeof(buf[i]));
                buf[i].next = &buf[i + 1];
        }
        bzero((char *)&buf[i], sizeof(buf[i]));
        buf[i].next = &buf[0];
        return buf;
}
