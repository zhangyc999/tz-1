#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#include <math.h>

#define PI 3.141592653589793
#define AA 1000000
#define BB 1000000

#define MSG    msg_rse
#define CMD    CMD_ACT_RSE
#define NOTIFY TASK_NOTIFY_RSE

#define PERIOD_SLOW 200
#define PERIOD_FAST 10

#define PRIO_SLOW 90
#define PRIO_FAST 40

#define MAX_NUM_DEV   4
#define MAX_NUM_FORM  1
#define MAX_LEN_CLLST 3

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
#define RESULT_STOP          0x00010000
#define RESULT_ZERO          0x00020000
#define RESULT_DEST          0x00040000
#define RESULT_MID           0x00080000
#define RESULT_LOAD          0x00100000

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int filter_judge(int *ok, int *err, int value, int min, int max, int ctr);
void plan(int *vel, int *len_pass, int len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period);
int max_of_n(int buf[], int n);
int min_of_n(int buf[], int n);
void lvl_posi(int delta[], int a, int b, int data);
void lvl_nega(int delta[], int a, int b, int data);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID MSG;
extern RING_ID rng_can_slow[];
extern RING_ID rng_can_fast[];
extern RING_ID rng_result;
extern SEM_ID sem_can[];
extern SEM_ID sem_result;

const static int addr[MAX_NUM_DEV] = {
        J1939_ADDR_RSE0, J1939_ADDR_RSE1, J1939_ADDR_RSE2, J1939_ADDR_RSE3
};
const static int cable[MAX_NUM_DEV] = {1, 1, 1, 1};
const static int sign[MAX_NUM_DEV] = {1, 1, 1, 1};
const static int io_pos_zero[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int io_pos_dest[MAX_NUM_DEV] = {20000, 20000, 20000, 20000};
const static int min_pos[MAX_NUM_DEV] = {-1000, -1000, -1000, -1000};
const static int max_pos[MAX_NUM_DEV] = {195000, 195000, 195000, 195000};
const static int min_vel[MAX_NUM_DEV] = {-1100, -1100, -1100, -1100};
const static int max_vel[MAX_NUM_DEV] = {1100, 1100, 1100, 1100};
const static int min_ampr[MAX_NUM_DEV] = {0, 0, 0, 0};
const static int max_ampr[MAX_NUM_DEV] = {200, 200, 200, 200};
const static int pos_zero[MAX_NUM_DEV] = {500, 500, 500, 500};
const static int pos_dest[MAX_NUM_DEV] = {194500, 194500, 194500, 194500};
const static int ampr_load[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int err_sync = 1000;
const static struct plan plan_len_auto[MAX_NUM_DEV] = {
        {2000, 4000, 182500},
        {2000, 4000, 182500},
        {2000, 4000, 182500},
        {2000, 4000, 182500}
};
const static struct plan plan_len_manual[MAX_NUM_DEV] = {
        {2000, 8000, 174500},
        {2000, 8000, 174500},
        {2000, 8000, 174500},
        {2000, 8000, 174500}
};
const static struct plan plan_len_repair[MAX_NUM_DEV] = {
        {2000, 8000, 600000},
        {2000, 8000, 600000},
        {2000, 8000, 600000},
        {2000, 8000, 600000}
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
static int ctr_ok_stop[MAX_NUM_DEV];
static int ctr_ok_zero[MAX_NUM_DEV];
static int ctr_ok_dest[MAX_NUM_DEV];
static int ctr_ok_load[MAX_NUM_DEV];
static int ctr_ok_sync;
static int ctr_err_pos[MAX_NUM_DEV];
static int ctr_err_vel[MAX_NUM_DEV];
static int ctr_err_ampr[MAX_NUM_DEV];
static int ctr_err_stop[MAX_NUM_DEV];
static int ctr_err_zero[MAX_NUM_DEV];
static int ctr_err_dest[MAX_NUM_DEV];
static int ctr_err_load[MAX_NUM_DEV];
static int ctr_err_sync;
static int ctr_fault[MAX_NUM_DEV];
static int ctr_io[MAX_NUM_DEV];
static int ctr_comm[MAX_NUM_DEV];
static int tmp_pos[MAX_NUM_DEV];
static int tmp_vel[MAX_NUM_DEV];
static int tmp_ampr[MAX_NUM_DEV];
static int tmp_stop[MAX_NUM_DEV];
static int tmp_zero[MAX_NUM_DEV];
static int tmp_dest[MAX_NUM_DEV];
static int tmp_load[MAX_NUM_DEV];
static int sub;
static int tmp_sync;
static int result[MAX_NUM_DEV];
static int all_stop;
static int all_zero;
static int all_dest;
static int num_load;
static int any_fault;
static int plan_vel[MAX_NUM_DEV];
static int plan_len_pass[MAX_NUM_DEV];
static int plan_len_posi[MAX_NUM_DEV];
static int plan_len_nega[MAX_NUM_DEV];
static int plan_len[MAX_NUM_DEV];
static struct plan max_plan_len[MAX_NUM_DEV];
static int dir[MAX_NUM_DEV];
static int delta_posi[MAX_NUM_DEV];
static int delta_nega[MAX_NUM_DEV];
static int i;
static int j;

void t_rse(void) /* Task: RaiSE arm */
{
        RING_ID rng_can[2] = {rng_can_slow[0], rng_can_slow[1]};
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
                tx[i].data.cmd.enable = 0x3C;
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(MSG, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        if ((cmd.type & UNMASK_TASK_NOTIFY) == TASK_NOTIFY_LVL) {
                                if ((cmd.type & UNMASK_TASK_STATE) == TASK_STATE_RUNNING && num_load > 2) {
                                        lvl_posi(delta_posi, AA, BB, cmd.data);
                                        lvl_nega(delta_nega, AA, BB, cmd.data);
                                } else {
                                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                                delta_posi[i] = 0;
                                                delta_nega[i] = 0;
                                        }
                                }
                        } else {
                                switch (verify.type) {
                                case CMD_IDLE:
                                case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        switch (cmd.type) {
                                        case CMD_IDLE:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_POSI | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_REPAIR:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                                        switch (cmd.type) {
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                        switch (cmd.type) {
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD | CMD_DIR_POSI | CMD_MODE_REPAIR:
                                        switch (cmd.type) {
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_POSI | CMD_MODE_REPAIR:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                        switch (cmd.type) {
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
                                        switch (cmd.type) {
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case CMD | CMD_DIR_NEGA| CMD_MODE_REPAIR:
                                        switch (cmd.type) {
                                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                        case CMD | CMD_DIR_STOP | CMD_MODE_REPAIR:
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_REPAIR:
                                                verify = cmd;
                                                break;
                                        default:
                                                break;
                                        }
                                default:
                                        break;
                                }
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = *(struct frame_can *)tmp;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                if (can.src == addr[i])
                                        break;
                        }
                        has_received[i] = 1;
                        switch (can.form) {
                        case 0xC6:
                                j = 0;
                                p[i][j] = p[i][j]->next;
                                sum_pos[i] -= (int)p[i][j]->data.state.pos * 20;
                                sum_vel[i] -= p[i][j]->data.state.vel;
                                sum_ampr[i] -= abs(p[i][j]->data.state.ampr);
                                old_fault[i] = p[i][j]->data.state.fault;
                                old_io[i] = p[i][j]->data.state.io;
                                p[i][j]->data.state.pos = sign[i] * ((FRAME_RX *)&can)->data.state.pos;
                                p[i][j]->data.state.vel = sign[i] * ((FRAME_RX *)&can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                cur_pos[i] = (int)p[i][j]->data.state.pos * 20;
                                cur_vel[i] = p[i][j]->data.state.vel;
                                cur_ampr[i] = abs(p[i][j]->data.state.ampr);
                                sum_pos[i] += (int)p[i][j]->data.state.pos * 20;
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
                                case 0x00:
                                case 0x03:
                                        if (ctr_fault[i] < 2)
                                                break;
                                        result[i] &= ~RESULT_FAULT_GENERAL;
                                        result[i] &= ~RESULT_FAULT_SERIOUS;
                                        break;
                                case 0x0C:
                                        if (ctr_fault[i] < 1)
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
                                        if (ctr_io[i] < MAX_LEN_CLLST)
                                                ctr_io[i]++;
                                } else {
                                        ctr_io[i] = 0;
                                }
                                if (ctr_io[i] > 5)
                                        result[i] = result[i] & ~UNMASK_RESULT_IO | p[i][j]->data.state.io;
                                tmp_pos[i] = filter_judge(&ctr_ok_pos[i], &ctr_err_pos[i], avg_pos[i], min_pos[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_vel[i] = filter_judge(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], MAX_LEN_CLLST);
                                tmp_ampr[i] = filter_judge(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_stop[i] = filter_judge(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, MAX_LEN_CLLST);
                                tmp_zero[i] = filter_judge(&ctr_ok_zero[i], &ctr_err_zero[i], cur_pos[i], min_pos[i] - 600000, pos_zero[i] + delta_nega[i], MAX_LEN_CLLST);
                                tmp_dest[i] = filter_judge(&ctr_ok_dest[i], &ctr_err_dest[i], cur_pos[i], pos_dest[i] + delta_posi[i], max_pos[i] + 600000, MAX_LEN_CLLST);
                                tmp_load[i] = filter_judge(&ctr_ok_load[i], &ctr_err_load[i], cur_ampr[i], ampr_load[i], max_ampr[i] + 600000, MAX_LEN_CLLST);
#if 0
                                if (avg_pos[i] < io_pos_zero[i] - 500 && (result[i] & 0x00000003) != 0x00000002
                                    || avg_pos[i] > io_pos_dest[i] + 500 && (result[i] & 0x00000003) != 0x00000001
                                    || avg_pos[i] > io_pos_zero[i] + 500 && avg_pos[i] < io_pos_dest[i] - 500 && result[i] & 0x00000003
                                    || avg_pos[i] >= io_pos_zero[i] - 500 && avg_pos[i] <= io_pos_zero[i] + 500 && result[i] & 0x00000002
                                    || avg_pos[i] >= io_pos_dest[i] - 500 && avg_pos[i] <= io_pos_dest[i] + 500 && result[i] & 0x00000001)
                                        result[i] |= RESULT_FAULT_IO;
                                else
                                        result[i] &= ~RESULT_FAULT_IO;
#endif
                                if (tmp_pos[i] == -1)
                                        result[i] |= RESULT_FAULT_POS;
                                else if (tmp_pos[i] == 1)
                                        result[i] &= ~RESULT_FAULT_POS;
                                if (tmp_vel[i] == -1)
                                        result[i] |= RESULT_FAULT_VEL;
                                else if (tmp_vel[i] == 1)
                                        result[i] &= ~RESULT_FAULT_VEL;
                                if (tmp_ampr[i] == -1)
                                        result[i] |= RESULT_FAULT_AMPR;
                                else if (tmp_ampr[i] == 1)
                                        result[i] &= ~RESULT_FAULT_AMPR;
                                if (tmp_stop[i] == 1)
                                        result[i] |= RESULT_STOP;
                                else if (tmp_stop[i] == -1)
                                        result[i] &= ~RESULT_STOP;
                                if (tmp_zero[i] == 1)
                                        result[i] |= RESULT_ZERO;
                                else if (tmp_zero[i] == -1)
                                        result[i] &= ~RESULT_ZERO;
                                if (tmp_dest[i] == 1)
                                        result[i] |= RESULT_DEST;
                                else if (tmp_dest[i] == -1)
                                        result[i] &= ~RESULT_DEST;
                                if (tmp_load[i] == 1)
                                        result[i] |= RESULT_LOAD;
                                else if (tmp_load[i] == -1)
                                        result[i] &= ~RESULT_LOAD;
                                break;
                        default:
                                break;
                        }
                        all_stop = 0;
                        all_zero = 0;
                        all_dest = 0;
                        num_load = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                all_stop &= result[i];
                                all_zero &= result[i];
                                all_dest &= result[i];
                                if (result[i] & RESULT_LOAD)
                                        num_load++;
                        }
                        all_stop &= RESULT_STOP;
                        all_zero &= RESULT_ZERO;
                        all_dest &= RESULT_DEST;
                        sub = max_of_n(avg_pos, MAX_NUM_DEV) - min_of_n(avg_pos, MAX_NUM_DEV);
                        tmp_sync = filter_judge(&ctr_ok_sync, &ctr_err_sync, sub, -err_sync, err_sync, MAX_LEN_CLLST);
                        if (tmp_sync == -1) {
                                for (i = 0; i < MAX_NUM_DEV; i++)
                                        result[i] |= RESULT_FAULT_SYNC;
                        } else if (tmp_sync == 1) {
                                for (i = 0; i < MAX_NUM_DEV; i++)
                                        result[i] &= ~RESULT_FAULT_SYNC;
                        }
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
                        if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_REPAIR)
                                any_fault = any_fault & UNMASK_RESULT_FAULT & ~RESULT_FAULT_SYNC;
                        else
                                any_fault &= UNMASK_RESULT_FAULT;
                        if (any_fault) {
                                state.type = TASK_STATE_FAULT;
                                if ((verify.type & UNMASK_CMD_ACT) == CMD)
                                        verify.type = verify.type & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (all_zero)
                                        state.type = TASK_STATE_ZERO;
                                else if (all_dest)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= NOTIFY;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        semTake(sem_result, WAIT_FOREVER);
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                rngBufPut(rng_result, (char *)&addr[i], sizeof(addr[i]));
                                rngBufPut(rng_result, (char *)&result[i], sizeof(result[i]));
                        }
                        semGive(sem_result);
                        switch (verify.type) {
                        case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                        case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                        case CMD | CMD_DIR_POSI | CMD_MODE_REPAIR:
                        case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                        case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
                        case CMD | CMD_DIR_NEGA | CMD_MODE_REPAIR:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        switch (verify.type & UNMASK_CMD_DIR) {
                                        case CMD_DIR_POSI:
                                                dir[i] = 1;
                                                plan_len[i] = plan_len_posi[i] + delta_posi[i];
                                                break;
                                        case CMD_DIR_NEGA:
                                                dir[i] = -1;
                                                plan_len[i] = plan_len_nega[i] + delta_nega[i];
                                                break;
                                        default:
                                                dir[i] = 0;
                                                plan_len[i] = 0;
                                                break;
                                        }
                                        switch (verify.type & UNMASK_CMD_MODE) {
                                        case CMD_MODE_AUTO:
                                                max_plan_len[i] = plan_len_auto[i];
                                                break;
                                        case CMD_MODE_MANUAL:
                                                max_plan_len[i] = plan_len_manual[i];
                                                break;
                                        case CMD_MODE_REPAIR:
                                                max_plan_len[i] = plan_len_repair[i];
                                                plan_len[i] = max_plan_len[i].low * 2 + max_plan_len[i].acc * 2 + max_plan_len[i].high;
                                                break;
                                        default:
                                                break;
                                        }
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (verify.data & 1 << i) {
                                                if ((verify.type & UNMASK_CMD_DIR) == CMD_DIR_POSI && result[i] & RESULT_DEST ||
                                                    (verify.type & UNMASK_CMD_DIR) == CMD_DIR_NEGA && result[i] & RESULT_ZERO) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len[i] = 0;
                                                        plan_len_pass[i] = 0;
                                                        plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                        plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                                } else {
                                                        plan(&plan_vel[i], &plan_len_pass[i], plan_len[i],
                                                             max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                        tx[i].data.cmd.vel = dir[i] * sign[i] * (s16)plan_vel[i];
                                                }
                                        } else {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len[i] = 0;
                                                plan_len_pass[i] = 0;
                                                plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                        }
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        tx[i].data.cmd.enable = 0xC3;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                rng_can[0] = rng_can_fast[0];
                                rng_can[1] = rng_can_fast[1];
                                taskPrioritySet(taskIdSelf(), PRIO_FAST);
                                period = PERIOD_FAST;
                                break;
                        default:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        plan_vel[i] = 0;
                                        plan_len_pass[i] = 0;
                                        plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                        plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0;
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                if (all_stop == 0) {
                                        rng_can[0] = rng_can_slow[0];
                                        rng_can[1] = rng_can_slow[1];
                                        taskPrioritySet(taskIdSelf(), PRIO_SLOW);
                                        period = PERIOD_SLOW;
                                } else {
                                        rng_can[0] = rng_can_fast[0];
                                        rng_can[1] = rng_can_fast[1];
                                        taskPrioritySet(taskIdSelf(), PRIO_FAST);
                                        period = PERIOD_FAST;
                                }
                                break;
                        }
                        break;
                }
        }
}

void lvl_posi(int delta[], int a, int b, int data)
{
        double x, y;
        int tmp[4];
        x = ((s16)(data & 0x0000FFFF) * 0.001 + 0.1) * PI / 180;
        y = ((s16)((data & 0xFFFF0000) >> 16) * 0.001 + 0.1) * PI / 180;
        if (x >= 0 && y >= 0) {
                tmp[0] = 0;
                tmp[1] = sin(x) * b * 0.01;
                tmp[2] = (sin(x) * b + sin(x) * a) * 0.01;
                tmp[3] = sin(y) * a * 0.01;
        } else if (x >= 0 && y < 0) {
                tmp[0] = -sin(y) * a * 0.01;
                tmp[1] = (sin(x) * b - sin(y) * a) * 0.01;
                tmp[2] = sin(x) * b * 0.01;
                tmp[3] = 0;
        } else if (x < 0 && y >= 0) {
                tmp[0] = -sin(x) * b * 0.01;
                tmp[1] = 0;
                tmp[2] = sin(y) * a * 0.01;
                tmp[3] = (-sin(x) * b + sin(y) * a) * 0.01;
        } else {
                tmp[0] = (-sin(x) * b - sin(y) * a) * 0.01;
                tmp[1] = -sin(y) * a * 0.01;
                tmp[2] = 0;
                tmp[3] = -sin(x) * b * 0.01;
        }
        delta[0] = tmp[0] - max_of_n(tmp, 4);
        delta[1] = tmp[1] - max_of_n(tmp, 4);
        delta[2] = tmp[2] - max_of_n(tmp, 4);
        delta[3] = tmp[3] - max_of_n(tmp, 4);
}

void lvl_nega(int delta[], int a, int b, int data)
{
        double x, y;
        int tmp[4];
        x = ((s16)(data & 0x0000FFFF) * 0.001 + 0.1) * PI / 180;
        y = ((s16)((data & 0xFFFF0000) >> 16) * 0.001 + 0.1) * PI / 180;
        if (x >= 0 && y >= 0) {
                tmp[0] = -(sin(x) * b + sin(x) * a) * 0.01;
                tmp[1] = -sin(x) * a * 0.01;
                tmp[2] = 0;
                tmp[3] = -sin(x) * b  * 0.01;
        } else if (x >= 0 && y < 0) {
                tmp[0] = -sin(x) * b * 0.01;
                tmp[1] = 0;
                tmp[2] = sin(y) * a * 0.01;
                tmp[3] = (sin(y) * a - sin(x) * b) * 0.01;
        } else if (x < 0 && y >= 0) {
                tmp[0] = -sin(y) * a * 0.01;
                tmp[1] = (sin(x) * b - sin(y) * a) * 0.01;
                tmp[2] = sin(x) * b * 0.01;
                tmp[3] = 0;
        } else {
                tmp[0] = 0;
                tmp[1] = sin(x) * b * 0.01;
                tmp[2] = (sin(x) * b + sin(y) * a) * 0.01;
                tmp[3] = sin(y) * a * 0.01;
        }
        delta[0] = tmp[0] - min_of_n(tmp, 4);
        delta[1] = tmp[1] - min_of_n(tmp, 4);
        delta[2] = tmp[2] - min_of_n(tmp, 4);
        delta[3] = tmp[3] - min_of_n(tmp, 4);
}
