#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#include <math.h>

#define PERIOD_SLOW 200
#define PERIOD_FAST 20

#define MAX_NUM_DEV   4
#define MAX_NUM_FORM  3
#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_IO     0x000000FF
#define UNMASK_RESULT_FAULT  0x0000FF00
#define UNMASK_RESULT_MISC   0xFFFF0000
#define RESULT_FAULT_GENERAL 0x00000100
#define RESULT_FAULT_SERIOUS 0x00000200
#define RESULT_FAULT_IO      0x00000400
#define RESULT_FAULT_POS     0x00000800
#define RESULT_FAULT_VEL     0x00001000
#define RESULT_FAULT_AMPR    0x00002000
#define RESULT_FAULT_SYNC    0x00004000
#define RESULT_FAULT_COMM    0x00008000
#define RESULT_ZERO          0x00010000
#define RESULT_DEST          0x00020000
#define RESULT_STOP          0x00040000
#define RESULT_SAFE          0x00080000
#define RESULT_LOAD          0x00100000

#define PI 3.141592653589793
#define AA 1000000
#define BB 1000000

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int remap_form_index(u8 form);
int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
void plan(int *vel, int len_total, int *len_pass, struct plan *plan_len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period);
int max_pos_of_n(int pos[], int n);
int min_pos_of_n(int pos[], int n);
void lvl_posi(int delta[], int a, int b, int data);
void lvl_nega(int delta[], int a, int b, int data);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_rse;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int addr[MAX_NUM_DEV] = {J1939_ADDR_RSE0, J1939_ADDR_RSE1, J1939_ADDR_RSE2, J1939_ADDR_RSE3};
const static int cable[MAX_NUM_DEV] = {1, 1, 1, 1};
const static int io_pos_zero[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int io_pos_dest[MAX_NUM_DEV] = {20000, 20000, 20000, 20000};
const static int min_pos[MAX_NUM_DEV] = {-1000, -1000, -1000, -1000};
const static int max_pos[MAX_NUM_DEV] = {36000, 36000, 36000, 36000};
const static int min_vel[MAX_NUM_DEV] = {-1500, -1500, -1500, -1500};
const static int max_vel[MAX_NUM_DEV] = {1500, 1500, 1500, 1500};
const static int min_ampr[MAX_NUM_DEV] = {0, 0, 0, 0};
const static int max_ampr[MAX_NUM_DEV] = {200, 200, 200, 200};
const static int pos_zero[MAX_NUM_DEV] = {500, 500, 500, 500};
const static int pos_dest[MAX_NUM_DEV] = {20000, 20000, 20000, 20000};
const static int pos_safe[MAX_NUM_DEV] = {10000, 10000, 10000, 10000};
const static int ampr_zero[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int ampr_dest[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int ampr_load[MAX_NUM_DEV] = {150, 150, 150, 150};
const static int err_sync = 1000;
const static struct plan max_plan_len[MAX_NUM_DEV] = {
        {1000, 4000, 10000},
        {1000, 4000, 10000},
        {1000, 4000, 10000},
        {1000, 4000, 10000}
};
const static int plan_vel_low[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int plan_vel_high[MAX_NUM_DEV] = {1000, 1000, 1000, 1000};
const static int plan_vel_medium[MAX_NUM_DEV] = {500, 500, 500, 500};

static int period = PERIOD_SLOW;
static u32 prev;
static int len;
static u8 tmp[sizeof(struct frame_can)];
static struct main cmd;
static struct main verify = {CMD_IDLE, 0};
static struct main state;
static struct main old_state;
static struct frame_can can;
static struct frame_can rx[MAX_NUM_DEV][MAX_NUM_FORM][MAX_LEN_CLLST];
static FRAME_RX *p[MAX_NUM_DEV][MAX_NUM_FORM];
static FRAME_TX tx[MAX_NUM_DEV];
static int has_received[MAX_NUM_DEV];
static int cur_pos[MAX_NUM_DEV];
static int cur_vel[MAX_NUM_DEV];
static int cur_ampr[MAX_NUM_DEV];
static int sum_pos[MAX_NUM_DEV];
static int sum_vel[MAX_NUM_DEV];
static int sum_ampr[MAX_NUM_DEV];
static int avg_pos[MAX_NUM_DEV];
static int avg_vel[MAX_NUM_DEV];
static int avg_ampr[MAX_NUM_DEV];
static int old_fault[MAX_NUM_DEV];
static int old_io[MAX_NUM_DEV];
static int ctr_ok_pos[MAX_NUM_DEV];
static int ctr_ok_vel[MAX_NUM_DEV];
static int ctr_ok_ampr[MAX_NUM_DEV];
static int ctr_ok_zero[MAX_NUM_DEV];
static int ctr_ok_dest[MAX_NUM_DEV];
static int ctr_ok_stop[MAX_NUM_DEV];
static int ctr_ok_safe[MAX_NUM_DEV];
static int ctr_ok_load[MAX_NUM_DEV];
static int ctr_ok_sync;
static int ctr_err_pos[MAX_NUM_DEV];
static int ctr_err_vel[MAX_NUM_DEV];
static int ctr_err_ampr[MAX_NUM_DEV];
static int ctr_err_zero[MAX_NUM_DEV];
static int ctr_err_dest[MAX_NUM_DEV];
static int ctr_err_stop[MAX_NUM_DEV];
static int ctr_err_safe[MAX_NUM_DEV];
static int ctr_err_load[MAX_NUM_DEV];
static int ctr_err_sync;
static int ctr_fault[MAX_NUM_DEV];
static int ctr_io[MAX_NUM_DEV];
static int ctr_comm[MAX_NUM_DEV];
static int result[MAX_NUM_DEV];
static int tmp_pos;
static int tmp_vel;
static int tmp_ampr;
static int tmp_zero;
static int tmp_dest;
static int tmp_stop;
static int tmp_safe;
static int tmp_load;
static int tmp_sync;
static int all_zero;
static int all_dest;
static int all_safe;
static int num_load;
static int any_fault;
static int sub;
static int plan_vel[MAX_NUM_DEV];
static int plan_len_pass[MAX_NUM_DEV];
static int plan_len_posi[MAX_NUM_DEV];
static int plan_len_nega[MAX_NUM_DEV];
static struct plan plan_len[MAX_NUM_DEV];
static struct plan ext_plan_len[MAX_NUM_DEV];
static int delta_posi[MAX_NUM_DEV];
static int delta_nega[MAX_NUM_DEV];
static int i;
static int j;

void t_rse(void) /* Task: RaiSE arm */
{
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_rse, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        if ((cmd.type & UNMASK_TASK_NOTIFY) == TASK_NOTIFY_LVL) {
                                if ((cmd.type & UNMASK_TASK_STATE) == TASK_STATE_RUNNING) {
                                        if (num_load > 2) {
                                                lvl_posi(delta_posi, AA, BB, cmd.data);
                                                lvl_nega(delta_nega, AA, BB, cmd.data);
                                        } else {
                                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                                        delta_posi[i] = 0;
                                                        delta_nega[i] = 0;
                                                }
                                        }
                                } else {
                                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                                delta_posi[i] = 0;
                                                delta_nega[i] = 0;
                                        }
                                }
                        } else {
                                switch (verify.type) {
                                case CMD_IDLE:
                                case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        switch (cmd.type) {
                                        case CMD_IDLE:
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_POSI:
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_POSI:
                                        switch (cmd.type) {
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_POSI:
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                        switch (cmd.type) {
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                        switch (cmd.type) {
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                        switch (cmd.type) {
                                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                default:
                                        break;
                                }
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = *(struct frame_can *)tmp;
                        switch (can.src) {
                        case J1939_ADDR_RSE0:
                                i = 0;
                                break;
                        case J1939_ADDR_RSE1:
                                i = 1;
                                break;
                        case J1939_ADDR_RSE2:
                                i = 2;
                                break;
                        case J1939_ADDR_RSE3:
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
                                sum_pos[i] -= (int)(p[i][j]->data.state.pos) * 20;
                                sum_vel[i] -= p[i][j]->data.state.vel;
                                sum_ampr[i] -= abs(p[i][j]->data.state.ampr);
                                old_fault[i] = p[i][j]->data.state.fault;
                                old_io[i] = p[i][j]->data.state.io;
                                p[i][j]->data.state.pos = ((FRAME_RX *)&can)->data.state.pos;
                                p[i][j]->data.state.vel = ((FRAME_RX *)&can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                cur_pos[i] = (int)(p[i][j]->data.state.pos) * 20;
                                cur_vel[i] = p[i][j]->data.state.vel;
                                cur_ampr[i] = abs(p[i][j]->data.state.ampr);
                                sum_pos[i] += (int)(p[i][j]->data.state.pos) * 20;
                                sum_vel[i] += p[i][j]->data.state.vel;
                                sum_ampr[i] += abs(p[i][j]->data.state.ampr);
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
                                case J1939_FAULT_NORMAL:
                                case J1939_FAULT_WARN:
                                        if (ctr_fault[i] < 5)
                                                break;
                                        result[i] &= ~RESULT_FAULT_GENERAL;
                                        result[i] &= ~RESULT_FAULT_SERIOUS;
                                        break;
                                case J1939_FAULT_GENERAL:
                                        if (ctr_fault[i] < 3)
                                                break;
                                        result[i] |= RESULT_FAULT_GENERAL;
                                        break;
                                case J1939_FAULT_SERIOUS:
                                        result[i] |= RESULT_FAULT_SERIOUS;
                                        break;
                                default:
                                        break;
                                }
                                if (old_io[i] == p[i][j]->data.state.io) {
                                        if (ctr_io[i] < MAX_LEN_CLLST)
                                                ctr_io[i]++;
                                } else {
                                        ctr_io[i] = 0;
                                }
                                if (ctr_io[i] > 5)
                                        result[i] = result[i] & ~UNMASK_RESULT_IO | p[i][j]->data.state.io;
#if 0
                                if (avg_pos[i] > io_pos_dest[i] + 500 && (result[i] & 0x0000000C) != 0x00000004
                                    || avg_pos[i] < io_pos_zero[i] - 500 && (result[i] & 0x0000000C) != 0x00000008
                                    || avg_pos[i] > io_pos_zero[i] + 500 && avg_pos[i] < io_pos_dest[i] - 500 && (result[i] & 0x0000000C) != 0x0000000C
                                    || avg_pos[i] >= io_pos_dest[i] - 500 && avg_pos[i] <= io_pos_dest[i] + 500 && result[i] & 0x00000008
                                    || avg_pos[i] >= io_pos_zero[i] - 500 && avg_pos[i] <= io_pos_zero[i] + 500 && result[i] & 0x00000004)
                                        result[i] |= RESULT_FAULT_IO;
                                else
                                        result[i] &= ~RESULT_FAULT_IO;
#endif
                                tmp_pos = judge_filter(&ctr_ok_pos[i], &ctr_err_pos[i], avg_pos[i], min_pos[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_vel = judge_filter(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], MAX_LEN_CLLST);
                                tmp_ampr = judge_filter(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_zero = judge_filter(&ctr_ok_zero[i], &ctr_err_zero[i], avg_pos[i], min_pos[i], pos_zero[i] + delta_nega[i], MAX_LEN_CLLST);
                                tmp_dest = judge_filter(&ctr_ok_dest[i], &ctr_err_dest[i], avg_pos[i], pos_dest[i] + delta_posi[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_stop = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, MAX_LEN_CLLST);
                                tmp_safe = judge_filter(&ctr_ok_safe[i], &ctr_err_safe[i], avg_pos[i], pos_safe[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_load = judge_filter(&ctr_ok_load[i], &ctr_err_load[i], avg_ampr[i], ampr_load[i], max_ampr[i], MAX_LEN_CLLST);
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
                                if (tmp_zero == 1)
                                        result[i] |= RESULT_ZERO;
                                else if (tmp_zero == -1)
                                        result[i] &= ~RESULT_ZERO;
                                if (tmp_dest == 1)
                                        result[i] |= RESULT_DEST;
                                else if (tmp_dest == -1)
                                        result[i] &= ~RESULT_DEST;
                                if (tmp_stop == 1)
                                        result[i] |= RESULT_STOP;
                                else if (tmp_stop == -1)
                                        result[i] &= ~RESULT_STOP;
                                if (tmp_safe == 1)
                                        result[i] |= RESULT_SAFE;
                                else if (tmp_safe == -1)
                                        result[i] &= ~RESULT_SAFE;
                                if (tmp_load == 1)
                                        result[i] |= RESULT_LOAD;
                                else if (tmp_load == -1)
                                        result[i] &= ~RESULT_LOAD;
                                break;
                        default:
                                break;
                        }
                        all_zero = 0;
                        all_dest = 0;
                        all_safe = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                all_zero &= result[i];
                                all_dest &= result[i];
                                all_safe &= result[i];
                        }
                        all_zero &= RESULT_ZERO;
                        all_dest &= RESULT_DEST;
                        all_safe &= RESULT_SAFE;
                        num_load = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                if (result[i] & RESULT_LOAD)
                                        num_load++;
                        }
                        sub = avg_pos[max_pos_of_n(avg_pos, MAX_NUM_DEV)] - avg_pos[min_pos_of_n(avg_pos, MAX_NUM_DEV)];
                        tmp_sync = judge_filter(&ctr_ok_sync, &ctr_err_sync, sub, -err_sync, err_sync, MAX_LEN_CLLST);
#if 0
                        if (tmp_sync == -1) {
                                for (i = 0; i < MAX_NUM_DEV; i++)
                                        result[i] |= RESULT_FAULT_SYNC;
                        } else if (tmp_sync == 1) {
                                for (i = 0; i < MAX_NUM_DEV; i++)
                                        result[i] &= ~RESULT_FAULT_SYNC;
                        }
#endif
                        period -= tickGet() - prev;
                        break;
                default:
                        for (i = 0; i < MAX_NUM_DEV; i++) {
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
                        any_fault = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++)
                                any_fault |= result[i];
                        if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_AUTO)
                                any_fault &= UNMASK_RESULT_FAULT;
                        else if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_MANUAL)
                                any_fault = any_fault & UNMASK_RESULT_FAULT & ~RESULT_FAULT_SYNC;
                        if (any_fault) {
                                state.type = TASK_STATE_FAULT;
                                verify.type = verify.type & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (all_zero)
                                        state.type = TASK_STATE_ZERO;
                                else if (all_safe)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= TASK_NOTIFY_RSE;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        plan_vel[i] = 0;
                                        plan_len_pass[i] = 0;
                                        plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                        plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0;
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        if (result[i] & RESULT_STOP)
                                                tx[i].data.cmd.enable = J1939_SERVO_DISABLE;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (tx[i].data.cmd.enable != J1939_SERVO_DISABLE)
                                                break;
                                }
                                if (i == MAX_NUM_DEV)
                                        period = PERIOD_SLOW;
                                else
                                        period = PERIOD_FAST;
                                break;
                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_POSI:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (result[i] & RESULT_DEST) {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len_posi[i] = 0;
                                        } else {
                                                ext_plan_len[i] = max_plan_len[i];
                                                ext_plan_len[i].high += delta_posi[i];
                                                plan(&plan_vel[i], plan_len_posi[i] + delta_posi[i], &plan_len_pass[i],
                                                     &plan_len[i], ext_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                tx[i].data.cmd.vel = (s16)plan_vel[i];
                                        }
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (result[i] & RESULT_DEST) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len_posi[i] = 0;
                                                } else {
                                                        plan(&plan_vel[i], plan_len_posi[i], &plan_len_pass[i],
                                                             &plan_len[i], max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                        tx[i].data.cmd.vel = (s16)plan_vel[i];
                                                }
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                                tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        } else {
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
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        }
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (result[i] & RESULT_ZERO) {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len_nega[i] = 0;
                                        } else {
                                                ext_plan_len[i] = max_plan_len[i];
                                                ext_plan_len[i].high += delta_nega[i];
                                                plan(&plan_vel[i], plan_len_nega[i] + delta_nega[i], &plan_len_pass[i],
                                                     &plan_len[i], ext_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                tx[i].data.cmd.vel = -(s16)plan_vel[i];
                                        }
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (result[i] & RESULT_ZERO) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len_nega[i] = 0;
                                                } else {
                                                        plan(&plan_vel[i], plan_len_nega[i], &plan_len_pass[i],
                                                             &plan_len[i], max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                        tx[i].data.cmd.vel = -(s16)plan_vel[i];
                                                }
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                                tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        } else {
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
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        }
                                }
                                period = PERIOD_FAST;
                                break;
                        default:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
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
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}

void lvl_posi(int delta[], int a, int b, int data)
{
        double x, y;
        x = ((s16)(data & 0x0000FFFF) * 0.001 + 0.1) * PI / 180;
        y = ((s16)((data & 0xFFFF0000) >> 16) * 0.001 + 0.1) * PI / 180;
        if (x >= 0 && y >= 0) {
                delta[0] = 0;
                delta[1] = sin(x) * b * 0.01;
                delta[2] = (sin(x) * b + sin(x) * a) * 0.01;
                delta[3] = sin(y) * a * 0.01;
        } else if (x >= 0 && y < 0) {
                delta[0] = -sin(y) * a * 0.01;
                delta[1] = (sin(x) * b - sin(y) * a) * 0.01;
                delta[2] = sin(x) * b * 0.01;
                delta[3] = 0;
        } else if (x < 0 && y >= 0) {
                delta[0] = -sin(x) * b * 0.01;
                delta[1] = 0;
                delta[2] = sin(y) * a * 0.01;
                delta[3] = (-sin(x) * b + sin(y) * a) * 0.01;
        } else {
                delta[0] = (-sin(x) * b - sin(y) * a) * 0.01;
                delta[1] = -sin(y) * a * 0.01;
                delta[2] = 0;
                delta[3] = -sin(x) * b * 0.01;
        }
}

void lvl_nega(int delta[], int a, int b, int data)
{
        double x, y;
        x = ((s16)(data & 0x0000FFFF) * 0.001 + 0.1) * PI / 180;
        y = ((s16)((data & 0xFFFF0000) >> 16) * 0.001 + 0.1) * PI / 180;
        if (x >= 0 && y >= 0) {
                delta[0] = -(sin(x) * b + sin(x) * a) * 0.01;
                delta[1] = -sin(x) * a * 0.01;
                delta[2] = 0;
                delta[3] = -sin(x) * b  * 0.01;
        } else if (x >= 0 && y < 0) {
                delta[0] = -sin(x) * b * 0.01;
                delta[1] = 0;
                delta[2] = sin(y) * a * 0.01;
                delta[3] = (sin(y) * a - sin(x) * b) * 0.01;
        } else if (x < 0 && y >= 0) {
                delta[0] = -sin(y) * a * 0.01;
                delta[1] = (sin(x) * b - sin(y) * a) * 0.01;
                delta[2] = sin(x) * b * 0.01;
                delta[3] = 0;
        } else {
                delta[0] = 0;
                delta[1] = sin(x) * b * 0.01;
                delta[2] = (sin(x) * b + sin(y) * a) * 0.01;
                delta[3] = sin(y) * a * 0.01;
        }
}
