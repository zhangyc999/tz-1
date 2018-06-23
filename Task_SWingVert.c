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
#define UNMASK_RESULT_MISC    0xFFFF0000
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_IO       0x00000400
#define RESULT_FAULT_POS      0x00000800
#define RESULT_FAULT_VEL      0x00001000
#define RESULT_FAULT_AMPR     0x00002000
#define RESULT_FAULT_SYNC     0x00004000
#define RESULT_FAULT_COMM     0x00008000
#define RESULT_SAFE           0x00010000
#define RESULT_ZERO           0x00020000
#define RESULT_MID            0x00040000
#define RESULT_DEST           0x00080000
#define RESULT_STOP           0x00100000
#define RESULT_LOAD           0x00200000

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swv;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

extern void plan(int *vel, int *len_total, int *len_pass, struct plan *plan_len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period);
extern int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
extern struct frame_can *can_cllst_init(struct frame_can buf[], int len);

const static int n = 4;
const static int max_form = 3;
const static int addr[4] = {J1939_ADDR_SWV0, J1939_ADDR_SWV1, J1939_ADDR_SWV2, J1939_ADDR_SWV3};
const static int cable[4] = {0, 0, 1, 1};
const static int io_pos_zero[4] = {100, 100, 100, 100};
const static int io_pos_mid[4] = {10000, 10000, 10000, 10000};
const static int io_pos_dest[4] = {20000, 20000, 20000, 20000};
const static int min_pos[4] = {-1000, -1000, -1000, -1000};
const static int max_pos[4] = {52000, 52000, 52000, 52000};
const static int min_vel[4] = {-1500, -1500, -1500, -1500};
const static int max_vel[4] = {1500, 1500, 1500, 1500};
const static int min_ampr[4] = {0, 0, 0, 0};
const static int max_ampr[4] = {200, 200, 200, 200};
const static int safe_pos[4] = {10000, 10000, 10000, 10000};
const static int stop_pos_zero[4] = {500, 500, 500, 500};
const static int stop_pos_mid[4] = {10000, 10000, 10000, 10000};
const static int stop_pos_dest[4] = {20000, 20000, 20000, 20000};
const static int stop_ampr_zero[4] = {100, 100, 100, 100};
const static int stop_ampr_mid[4] = {100, 100, 100, 100};
const static int stop_ampr_dest[4] = {100, 100, 100, 100};
const static int load_ampr[4] = {150, 150, 150, 150};
const static int err_sync_01 = 500;
const static int err_sync_23 = 500;
const static int err_sync_0123 = 1000;
const static struct plan max_plan_len[4] = {
        {22000, 4000, 40000},
        {22000, 4000, 40000},
        {22000, 4000, 40000},
        {22000, 4000, 40000}
};
const static int plan_vel_low[4] = {50, 50, 50, 50};
const static int plan_vel_high[4] = {1000, 1000, 1000, 1000};
const static int plan_vel_medium[4] = {500, 500, 500, 500};

static int period = PERIOD_SLOW;
static u32 prev;
static int len;
static u8 tmp[sizeof(struct frame_can)];
static struct main cmd;
static struct main verify = {CMD_IDLE, 0};
static struct main state;
static struct main old_state;
static struct frame_can can;
static struct frame_can rx[4][3][MAX_LEN_CLLST];
static FRAME_RX *p[4][3];
static FRAME_TX tx[4];
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
static int ctr_ok_safe[4];
static int ctr_ok_zero[4];
static int ctr_ok_mid[4];
static int ctr_ok_dest[4];
static int ctr_ok_stop[4];
static int ctr_ok_load[4];
static int ctr_ok_sync_01;
static int ctr_ok_sync_23;
static int ctr_ok_sync_0123;
static int ctr_err_pos[4];
static int ctr_err_vel[4];
static int ctr_err_ampr[4];
static int ctr_err_safe[4];
static int ctr_err_zero[4];
static int ctr_err_mid[4];
static int ctr_err_dest[4];
static int ctr_err_stop[4];
static int ctr_err_load[4];
static int ctr_err_sync_01;
static int ctr_err_sync_23;
static int ctr_err_sync_0123;
static int ctr_fault[4];
static int ctr_io[4];
static int ctr_comm[4];
static int result[4];
static int tmp_pos;
static int tmp_vel;
static int tmp_ampr;
static int tmp_safe;
static int tmp_zero;
static int tmp_mid;
static int tmp_dest;
static int tmp_stop;
static int tmp_load;
static int tmp_sync_01;
static int tmp_sync_23;
static int tmp_sync_0123;
static int all_safe;
static int all_zero;
static int all_mid;
static int all_dest;
static int all_load;
static int all_unload;
static int any_fault;
static int sub_01;
static int sub_23;
static int sub_0123;
static int plan_vel[4];
static int plan_len_pass[4];
static int plan_len_posi[4];
static int plan_len_nega[4];
static struct plan plan_len[4];
static int segement;
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
                        switch (verify.type) {
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
                                        verify = cmd;
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
                                        verify = cmd;
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
                                        verify = cmd;
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
                                        verify = cmd;
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
                                        verify = cmd;
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
                                tmp_safe = judge_filter(&ctr_ok_safe[i], &ctr_err_safe[i], avg_pos[i], safe_pos[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_zero = judge_filter(&ctr_ok_zero[i], &ctr_err_zero[i], avg_pos[i], min_pos[i], stop_pos_zero[i], MAX_LEN_CLLST);
                                tmp_mid = judge_filter(&ctr_ok_mid[i], &ctr_err_mid[i], avg_pos[i], stop_pos_mid[i] - 5000, stop_pos_mid[i] + 5000, MAX_LEN_CLLST);
                                tmp_dest = judge_filter(&ctr_ok_dest[i], &ctr_err_dest[i], avg_pos[i], stop_pos_dest[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_stop = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, MAX_LEN_CLLST);
                                tmp_load = judge_filter(&ctr_ok_load[i], &ctr_err_load[i], avg_ampr[i], load_ampr[i], max_ampr[i], MAX_LEN_CLLST);
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
                                if (tmp_safe == 1)
                                        result[i] |= RESULT_SAFE;
                                else if (tmp_safe == -1)
                                        result[i] &= ~RESULT_SAFE;
                                if (tmp_zero == 1)
                                        result[i] |= RESULT_ZERO;
                                else if (tmp_zero == -1)
                                        result[i] &= ~RESULT_ZERO;
                                if (tmp_mid == 1)
                                        result[i] |= RESULT_MID;
                                else if (tmp_mid == -1)
                                        result[i] &= ~RESULT_MID;
                                if (tmp_dest == 1)
                                        result[i] |= RESULT_DEST;
                                else if (tmp_dest == -1)
                                        result[i] &= ~RESULT_DEST;
                                if (tmp_stop == 1)
                                        result[i] |= RESULT_STOP;
                                else if (tmp_stop == -1)
                                        result[i] &= ~RESULT_STOP;
                                if (tmp_load == 1)
                                        result[i] |= RESULT_LOAD;
                                else if (tmp_load == -1)
                                        result[i] &= ~RESULT_LOAD;
                                break;
                        default:
                                break;
                        }
                        all_safe = result[0] & result[1] & result[2] & result[3] & RESULT_SAFE;
                        all_zero = result[0] & result[1] & result[2] & result[3] & RESULT_ZERO;
                        all_mid = result[0] & result[1] & result[2] & result[3] & RESULT_MID;
                        all_dest = result[0] & result[1] & result[2] & result[3] & RESULT_DEST;
                        all_load = result[0] & result[1] & result[2] & result[3] & RESULT_LOAD;
                        all_unload = (result[0] | result[1] | result[2] | result[3]) & RESULT_LOAD;
                        sub_01 = avg_pos[0] - avg_pos[1];
                        sub_23 = avg_pos[2] - avg_pos[3];
                        sub_0123 = (avg_pos[0] - avg_pos[3] + avg_pos[1] - avg_pos[2]) / 2;
                        tmp_sync_01 = judge_filter(&ctr_ok_sync_01, &ctr_err_sync_01, sub_01, -err_sync_01, err_sync_01, MAX_LEN_CLLST);
                        tmp_sync_23 = judge_filter(&ctr_ok_sync_23, &ctr_err_sync_23, sub_23, -err_sync_23, err_sync_23, MAX_LEN_CLLST);
                        tmp_sync_0123 = judge_filter(&ctr_ok_sync_0123, &ctr_err_sync_0123, sub_0123, -err_sync_0123, err_sync_0123, MAX_LEN_CLLST);
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
                        if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_AUTO)
                                any_fault = (result[0] | result[1] | result[2] | result[3]) & UNMASK_RESULT_FAULT;
                        else if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_MANUAL)
                                any_fault = (result[0] | result[1] | result[2] | result[3]) & UNMASK_RESULT_FAULT & ~RESULT_FAULT_SYNC;
                        if (any_fault) {
                                state.type = TASK_STATE_FAULT;
                                verify.type = verify.type & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (all_zero)
                                        state.type = TASK_STATE_ZERO;
                                else if (all_dest)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= TASK_NOTIFY_SWV;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                for (i = 0; i < n; i++) {
                                        plan_vel[i] = 0;
                                        plan_len_pass[i] = 0;
                                        plan_len_posi[i] = stop_pos_dest[i] - cur_pos[i];
                                        plan_len_nega[i] = cur_pos[i];
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
                                for (i = 0; i < n; i++) {
                                        if (tx[i].data.cmd.enable != J1939_SERVO_DISABLE)
                                                break;
                                }
                                if (i == n)
                                        period = PERIOD_SLOW;
                                else
                                        period = PERIOD_FAST;
                                break;
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_POSI:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (avg_ampr[i] > stop_ampr_dest[i]) {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len_posi[i] = 0;
                                        } else {
                                                plan(&plan_vel[i], &plan_len_posi[i], &plan_len_pass[i],
                                                     &plan_len[i], max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
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
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                for (i = 0; i < n; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (avg_ampr[i] > stop_ampr_dest[i]) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len_posi[i] = 0;
                                                } else {
                                                        plan(&plan_vel[i], &plan_len_posi[i], &plan_len_pass[i],
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
                        case CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (avg_ampr[i] > stop_ampr_zero[i]) {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len_nega[i] = 0;
                                        } else {
                                                plan(&plan_vel[i], &plan_len_nega[i], &plan_len_pass[i],
                                                     &plan_len[i], max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
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
                        case CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                for (i = 0; i < n; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (avg_ampr[i] > stop_ampr_zero[i]) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len_nega[i] = 0;
                                                } else {
                                                        plan(&plan_vel[i], &plan_len_nega[i], &plan_len_pass[i],
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
