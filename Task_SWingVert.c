#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200
#define PERIOD_FAST 20

#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_IO      0x000000FF
#define UNMASK_RESULT_FAULT   0x0000FF00
#define UNMASK_RESULT_RUNNING 0x00FF0000
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_LIMIT    0x00000400
#define RESULT_FAULT_POS      0x00000800
#define RESULT_FAULT_VEL      0x00001000
#define RESULT_FAULT_AMPR     0x00002000
#define RESULT_FAULT_SYNC     0x00004000
#define RESULT_FAULT_COMM     0x00008000
#define RESULT_RUNNING        0x00FF0000

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swv;

extern void plan(int *vel, int *len_pass, int period, int len_low, int len_acc, int len_high, int vel_low, int vel_high);
extern int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
extern struct frame_can *can_cllst_init(struct frame_can buf[], int len);

const static int n = 4;
const static int max_form = 3;
const static int addr[4] = {J1939_ADDR_SWV0, J1939_ADDR_SWV1, J1939_ADDR_SWV2, J1939_ADDR_SWV3};
const static int cable[4] = {0, 0, 1, 1};
const static int zero_pos[4] = {0, 0, 0, 0};
const static int limit_posi[4] = {40000, 40000, 40000, 40000};
const static int limit_nega[4] = {500, 500, 500, 500};
const static int ampr_pos_posi[4] = {40000, 40000, 40000, 40000};
const static int ampr_value_posi[4] = {50, 50, 50, 50};
const static int ampr_pos_nega[4] = {500, 500, 500, 500};
const static int ampr_value_nega[4] = {50, 50, 50, 50};
const static int min_pos[4] = {0, 0, 0, 0};
const static int min_vel[4] = {-1500, -1500, -1500, -1500};
const static int min_ampr[4] = {-2000, -2000, -2000, -2000};
const static int max_pos[4] = {1000, 1000, 1000, 1000};
const static int max_vel[4] = {1500, 1500, 1500, 1500};
const static int max_ampr[4] = {2000, 2000, 2000, 2000};
const static int safe_pos[4] = {20000, 20000, 20000, 20000};
const static int err_sync = 10000;
const static int plan_len_low[4] = {1000, 1000, 1000, 1000};
const static int plan_len_acc[4] = {4000, 4000, 4000, 4000};
const static int plan_len_high[4] = {10000, 10000, 10000, 10000};
const static int plan_vel_low[4] = {100, 100, 100, 100};
const static int plan_vel_high[4] = {1000, 1000, 1000, 1000};
const static int plan_vel_medium[4] = {500, 500, 500, 500};

static int period = PERIOD_SLOW;
static u32 prev;
static int len;
static u8 tmp[sizeof(struct frame_can)];
static struct main cmd;
static struct main state;
static struct main state_old;
static struct frame_can can;
static struct frame_can rx[4][3][MAX_LEN_CLLST];
static FRAME_RX *p[4][3];
static FRAME_TX tx[4];
static int verify = CMD_IDLE;
static int has_received[4];
static int cur_pos[4];
static int cur_vel[4];
static int cur_ampr[4];
static int sum_pos[4];
static int sum_vel[4];
static int sum_ampr[4];
static int avg_pos[4];
static int avg_vel[4];
static int avg_ampr[4];
static int old_fault[4];
static int old_io[4];
static int ctr_ok_pos[4];
static int ctr_ok_vel[4];
static int ctr_ok_ampr[4];
static int ctr_ok_sync;
static int ctr_ok_stop[4];
static int ctr_err_pos[4];
static int ctr_err_vel[4];
static int ctr_err_ampr[4];
static int ctr_err_sync;
static int ctr_err_stop[4];
static int ctr_fault[4];
static int ctr_io[4];
static int ctr_comm[4];
static int result[4];
static int tmp_pos;
static int tmp_vel;
static int tmp_ampr;
static int tmp_running;
static int tmp_sync;
static int sub;
static int plan_vel[4];
static int plan_len_pass[4];
static int plan_len_posi[4];
static int plan_len_nega[4];
static int plan_len_low_posi[4];
static int plan_len_low_nega[4];
static int plan_len_acc_posi[4];
static int plan_len_acc_nega[4];
static int plan_len_high_posi[4];
static int plan_len_high_nega[4];
static int i;
static int j;

void t_swv(void) /* Task: SWing leg of Vertical */
{
        for (i = 0; i < n; i++) {
                for (j = 0; j < max_form; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_swv, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                switch (cmd.type) {
                                case CMD_IDLE:
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd.type;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
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
                        case J1939_ADDR_SWV0:
                                i = 0;
                                break;
                        case J1939_ADDR_SWV1:
                                i = 1;
                                break;
                        case J1939_ADDR_SWV2:
                                i = 2;
                                break;
                        case J1939_ADDR_SWV3:
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
                                old_fault[i] = p[i][j]->data.state.fault;
                                old_io[i] = p[i][j]->data.state.io;
                                p[i][j]->data.state.pos = ((FRAME_RX *)&can)->data.state.pos * 20;
                                p[i][j]->data.state.vel = ((FRAME_RX *)&can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                cur_pos[i] = p[i][j]->data.state.pos;
                                cur_vel[i] = p[i][j]->data.state.vel;
                                cur_ampr[i] = p[i][j]->data.state.ampr;
                                sum_pos[i] += p[i][j]->data.state.pos;
                                sum_vel[i] += p[i][j]->data.state.vel;
                                sum_ampr[i] += p[i][j]->data.state.ampr;
                                avg_pos[i] = sum_pos[i] / MAX_LEN_CLLST;
                                avg_vel[i] = sum_vel[i] / MAX_LEN_CLLST;
                                avg_ampr[i] = sum_ampr[i] / MAX_LEN_CLLST;
                                if (old_fault[i] == p[i][j]->data.state.fault) {
                                        if (ctr_fault[i] < MAX_LEN_CLLST)
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
                                if (old_io[i] == p[i][j]->data.state.io) {
                                        if (ctr_io[i] < 10)
                                                ctr_io[i]++;
                                } else {
                                        ctr_io[i] = 0;
                                }
                                if (ctr_io[i] > 5)
                                        result[i] = result[i] & ~UNMASK_RESULT_IO | p[i][j]->data.state.io;
                                if (avg_pos[i] > limit_posi[i] && (result[i] & 0x0000000C) != 0x00000004
                                    || avg_pos[i] < limit_nega[i] && (result[i] & 0x0000000C) != 0x00000008
                                    || avg_pos[i] > limit_nega[i] + 500 && avg_pos[i] < limit_posi[i] - 500 && (result[i] & 0x0000000C) != 0x0000000C
                                    || avg_pos[i] >= limit_posi[i] - 500 && avg_pos[i] <= limit_posi[i] + 500 && result[i] & 0x00000008
                                    || avg_pos[i] >= limit_nega[i] - 500 && avg_pos[i] <= limit_nega[i] + 500 && result[i] & 0x00000004)
                                        result[i] |= RESULT_FAULT_LIMIT;
                                else
                                        result[i] &= ~RESULT_FAULT_LIMIT;
                                tmp_pos = judge_filter(&ctr_ok_pos[i], &ctr_err_pos[i], avg_pos[i], min_pos[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_vel = judge_filter(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], MAX_LEN_CLLST);
                                tmp_ampr = judge_filter(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_running = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, MAX_LEN_CLLST);
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
                        sub = avg_pos[max_pos_of_n(avg_pos, n)] - avg_pos[min_pos_of_n(avg_pos, n)];
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
                        for (i = 0; i < n; i++) {
                                if (has_received[i]) {
                                        has_received[i] = 0;
                                        if (ctr_comm[i] < 0)
                                                ctr_comm[i] = 0;
                                        if (ctr_comm[i] < MAX_LEN_CLLST)
                                                ctr_comm[i]++;
                                        if (ctr_comm[i] == MAX_LEN_CLLST)
                                                result[i] &= ~RESULT_FAULT_COMM;
                                } else {
                                        if (ctr_comm[i] > 0)
                                                ctr_comm[i] = 0;
                                        if (ctr_comm[i] > -MAX_LEN_CLLST)
                                                ctr_comm[i]--;
                                        if (ctr_comm[i] == -MAX_LEN_CLLST)
                                                result[i] |= RESULT_FAULT_COMM;
                                }
                        }
                        state.type = TASK_NOTIFY_SWV;
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
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                for (i = 0; i < n; i++) {
                                        plan_vel[i] = 0;
                                        plan_len_pass[i] = 0;
                                        plan_len_posi[i] = plan_len_low[i] * 2 + plan_len_acc[i] * 2 + plan_len_high[i] - cur_pos[i];
                                        plan_len_nega[i] = cur_pos[i];
                                        if (plan_len_posi[i] < 0) {
                                                plan_len_low_posi[i] = 0;
                                                plan_len_acc_posi[i] = 0;
                                                plan_len_high_posi[i] = 0;
                                        } else if (plan_len_posi[i] < plan_len_low[i] * 2) {
                                                plan_len_low_posi[i] = plan_len_posi[i] / 2;
                                                plan_len_acc_posi[i] = 0;
                                                plan_len_high_posi[i] = 0;
                                        } else if (plan_len_posi[i] < plan_len_low[i] * 2 + plan_len_acc[i] * 2) {
                                                plan_len_low_posi[i] = plan_len_low[i];
                                                plan_len_acc_posi[i] = plan_len_posi[i] / 2 - plan_len_low[i];
                                                plan_len_high_posi[i] = 0;
                                        } else {
                                                plan_len_low_posi[i] = plan_len_low[i];
                                                plan_len_acc_posi[i] = plan_len_acc[i];
                                                plan_len_high_posi[i] = plan_len_posi[i] - plan_len_low[i] * 2 - plan_len_acc[i] * 2;
                                        }
                                        if (plan_len_nega[i] < 0) {
                                                plan_len_low_nega[i] = 0;
                                                plan_len_acc_nega[i] = 0;
                                                plan_len_high_nega[i] = 0;
                                        } else if (plan_len_nega[i] < plan_len_low[i] * 2) {
                                                plan_len_low_nega[i] = plan_len_nega[i] / 2;
                                                plan_len_acc_nega[i] = 0;
                                                plan_len_high_nega[i] = 0;
                                        } else if (plan_len_nega[i] < plan_len_low[i] * 2 + plan_len_acc[i] * 2) {
                                                plan_len_low_nega[i] = plan_len_low[i];
                                                plan_len_acc_nega[i] = plan_len_nega[i] / 2 - plan_len_low[i];
                                                plan_len_high_nega[i] = 0;
                                        } else {
                                                plan_len_low_nega[i] = plan_len_low[i];
                                                plan_len_acc_nega[i] = plan_len_acc[i];
                                                plan_len_high_nega[i] = plan_len_nega[i] - plan_len_low[i] * 2 - plan_len_acc[i] * 2;
                                        }
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
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
#if 0
                                        if (avg_pos[i] > ampr_pos_posi[i] && avg_pos[i] < len_low + len_acc + len_high + len_acc + len_low) {
                                                tx[i].form = J1939_FORM_SERVO_AMPR;
                                                tx[i].data.cmd.vel = 0x3322;
                                                tx[i].data.cmd.ampr = (s16)ampr_value_posi[i];
                                        } else {
#endif
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                plan(&plan_vel[i], &plan_len_pass[i], PERIOD_FAST,
                                                     plan_len_low_posi[i], plan_len_acc_posi[i], plan_len_high_posi[i],
                                                     plan_vel_low[i], plan_vel_high[i]);
                                                tx[i].data.cmd.vel = (s16)plan_vel[i];
                                                tx[i].data.cmd.ampr = 1000;
#if 0
                                        }
#endif
                                        msgQSend(msg_can[cable[i]], (char *)&tx[i], sizeof(tx[i]), NO_WAIT, MSG_PRI_URGENT);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
#if 0
                                        if (avg_pos[i] < ampr_pos_nega[i] && avg_pos[i] > zero_pos[i]) {
                                                tx[i].form = J1939_FORM_SERVO_AMPR;
                                                tx[i].data.cmd.vel = 0x3322;
                                                tx[i].data.cmd.ampr = -(s16)ampr_value_nega[i];
                                        } else {
#endif
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                plan(&plan_vel[i], &plan_len_pass[i], PERIOD_FAST,
                                                     plan_len_low_nega[i], plan_len_acc_nega[i], plan_len_high_nega[i],
                                                     plan_vel_low[i], plan_vel_high[i]);
                                                tx[i].data.cmd.vel = -(s16)plan_vel[i];
                                                tx[i].data.cmd.ampr = 1000;
#if 0
                                        }
#endif
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
