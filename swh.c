#include "addr.h"
#include "define.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

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

#define VEL_PLAN_SLOW 100  /* 1mm/s */
#define VEL_PLAN_FAST 1000 /* 10mm/s */
#define LEN_PLAN_SLOW 2000 /* 20mm */
#define LEN_PLAN_MOM  300  /* 3mm */

extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swh;

void judge_filter(int bit, int *result, int *ok, int *err, int value, int min, int max, int ctr);
int plan_vel(int vel_cur, int acc, int len_pass, int len_remain);
int max_of_n(int *buf, int n);
int min_of_n(int *buf, int n);
struct frame_can *can_cllst_init(struct frame_can buf[], int len);

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

void t_swh(void) /* Task: SWing arm of Horizontal */
{
        int period = 8;
        int delay = period;
        u32 prev;
        int len;
        int tmp[20];
        struct main cmd;
        struct main state;
        struct main state_old;
        FRAME_RX can;
        struct frame_can rx[4][3][16];
        FRAME_RX *p[4][3];
        FRAME_TX tx[4];
        int verify = CMD_IDLE;
        int has_received[4] = {0};
        int n = 4;
        int i;
        int j;
        int max_form = 3;
        int addr[4] = {ADDR_SWH0, ADDR_SWH1, ADDR_SWH2, ADDR_SWH3};
        int cable[4] = {0, 0, 1, 1};
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
        int sub_01 = 0;
        int sub_23 = 0;
        int sub_0123 = 0;
        int async = 0;
        int slowest = 0;
        int len_remain = 0;
        int len_pass = 0;
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
                case 8:
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
                case 4:
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
                        j = remap_fomr_index(can.form);
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
                                judge_filter(RESULT_FAULT_POS, &result[i], &ctr_ok_pos[i], &ctr_err_pos[i], avg_pos[i], min_pos[i], max_pos[i], 10);
                                judge_filter(RESULT_FAULT_VEL, &result[i], &ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], 10);
                                judge_filter(RESULT_FAULT_AMPR, &result[i], &ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], 10);
                                judge_filter(RESULT_RUNNING, &result[i], &ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, 10);
                        default:
                                break;
                        }
                        sub_01 = avg_pos[0] - avg_pos[1];
                        sub_23 = avg_pos[2] - avg_pos[3];
                        sub_0123 = (avg_pos[0] - avg_pos[3] + avg_pos[1] - avg_pos[2]) / 2;
                        judge_filter(1, &async, &ctr_ok_sync_01, &ctr_err_sync_01, sub_01, -err_sync_01, err_sync_01, 10);
                        judge_filter(2, &async, &ctr_ok_sync_23, &ctr_err_sync_23, sub_23, -err_sync_23, err_sync_23, 10);
                        judge_filter(4, &async, &ctr_ok_sync_0123, &ctr_err_sync_0123, sub_0123, -err_sync_0123, err_sync_0123, 10);
                        if (async) {
                                for (i = 0; i < n; i++)
                                        result[i] |= RESULT_FAULT_SYNC;
                        } else {
                                for (i = 0; i < n; i++)
                                        result[i] &= ~RESULT_FAULT_SYNC;
                        }
                        switch (verify) {
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                slowest = min_of_n(avg_pos, n) * 20;
                                len_remain = 20000 - slowest;
                                len_pass = slowest;
                                break;
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                slowest = max_of_n(avg_pos, n) * 20;
                                len_remain = slowest;
                                len_pass = 20000 - slowest;
                                break;
                        case CMD_IDLE:
                        case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
                        default:
                                break;
                        }
                        delay -= tickGet() - prev;
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
                        for (i = 0; i < n; i++) {
                                switch (verify) {
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP:
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
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = plan_vel(avg_vel[i], 1, len_pass, len_remain);
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        tx[i].data.cmd.enable = 0xC3;
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                        break;
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                        break;
                                case CMD_IDLE:
                                default:
                                        tx[i].tsc = tickGet();
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
                                        break;
                                }
                        }
                        delay = period;
                        break;
                }
        }
}

void judge_filter(int bit, int *result, int *ok, int *err, int value, int min, int max, int ctr)
{
        if (value >= min && value < max) {
                *err = 0;
                if (*ok < ctr)
                        *ok++;
        } else {
                *ok = 0;
                if (*err < ctr)
                        *err++;
        }
        if (*ok == ctr)
                *result &= ~bit;
        else if (*err == ctr)
                *result |= bit;
}

int plan_vel(int vel_cur, int acc, int len_pass, int len_remain)
{
        int len_stop = 0;
        int vel_plan = 0;
        if (vel_cur > VEL_PLAN_SLOW)
                len_stop = (vel_cur * vel_cur - VEL_PLAN_SLOW * VEL_PLAN_SLOW) /
                           2 / acc / 25 + LEN_PLAN_SLOW;
        else
                len_stop = 0;
        if (len_remain > len_stop) {
                if (len_pass > LEN_PLAN_SLOW) {
                        if (vel_cur < VEL_PLAN_FAST)
                                vel_plan = vel_cur + acc;
                        else
                                vel_plan = VEL_PLAN_FAST;
                } else {
                        vel_plan = VEL_PLAN_SLOW;
                }
        } else {
                if (len_remain > LEN_PLAN_SLOW) {
                        if (vel_cur > VEL_PLAN_SLOW)
                                vel_plan = vel_cur - acc;
                        else
                                vel_plan = VEL_PLAN_SLOW;
                } else {
                        if (len_remain > 0)
                                vel_plan = VEL_PLAN_SLOW;
                        else
                                vel_plan = 0;
                }
        }
        return vel_plan;
}

int max_of_n(int *buf, int n)
{
        if (n)
                return max(*buf, max_of_n(buf + 1, n - 1));
        return *buf;
}

int min_of_n(int *buf, int n)
{
        if (n)
                return min(*buf, min_of_n(buf + 1, n - 1));
        return *buf;
}

struct frame_can *can_cllst_init(struct frame_can buf[], int len)
{
        int i = 0;
        for (i = 0; i < len - 1; i++)
                buf[i].next = &buf[i + 1];
        buf[i].next = &buf[0];
        return buf;
}
