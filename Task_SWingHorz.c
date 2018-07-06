#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define MSG    msg_swh
#define CMD    CMD_ACT_SWH
#define NOTIFY TASK_NOTIFY_SWH

#define PERIOD_SLOW 200
#define PERIOD_FAST 8

#define MAX_NUM_DEV   4
#define MAX_NUM_FORM  1
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
#define RESULT_STOP          0x01000000
#define RESULT_ZERO          0x02000000
#define RESULT_DEST          0x04000000
#define RESULT_LOAD          0x08000000
#define RESULT_SAFE          0x10000000
#define RESULT_PART_POSI(x)  (0x00010000 << x)
#define RESULT_PART_NEGA(x)  (0x00100000 << x)

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int filter_judge(int *ok, int *err, int value, int min, int max, int ctr);
void plan(int *vel, int *len_pass, int len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID MSG;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int addr[MAX_NUM_DEV] = {
        J1939_ADDR_SWH0, J1939_ADDR_SWH1, J1939_ADDR_SWH2, J1939_ADDR_SWH3
};
const static int cable[MAX_NUM_DEV] = {0, 0, 1, 1};
const static int sign[MAX_NUM_DEV] = {1, 1, 1, 1};
const static int io_pos_zero[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int io_pos_dest[MAX_NUM_DEV] = {20000, 20000, 20000, 20000};
const static int min_pos[MAX_NUM_DEV] = {-1000, -1000, -1000, -1000};
const static int max_pos[MAX_NUM_DEV] = {42000, 42000, 42000, 42000};
const static int min_vel[MAX_NUM_DEV] = {-1500, -1500, -1500, -1500};
const static int max_vel[MAX_NUM_DEV] = {1500, 1500, 1500, 1500};
const static int min_ampr[MAX_NUM_DEV] = {0, 0, 0, 0};
const static int max_ampr[MAX_NUM_DEV] = {200, 200, 200, 200};
const static int pos_zero[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int pos_dest[MAX_NUM_DEV] = {40000, 40000, 40000, 40000}; /* 42000 */
const static int ampr_load[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int pos_safe[MAX_NUM_DEV] = {10000, 10000, 10000, 10000};
const static int err_sync_01 = 500;
const static int err_sync_23 = 500;
const static int err_sync = 1000;
const static struct plan max_plan_len[MAX_NUM_DEV] = {
        {1000, 4000, 30000},
        {1000, 4000, 30000},
        {1000, 4000, 30000},
        {1000, 4000, 30000}
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
static struct main brake = {CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_LIGHT, 0};
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
static int ctr_ok_safe[MAX_NUM_DEV];
static int ctr_ok_sync_01;
static int ctr_ok_sync_23;
static int ctr_ok_sync;
static int ctr_err_pos[MAX_NUM_DEV];
static int ctr_err_vel[MAX_NUM_DEV];
static int ctr_err_ampr[MAX_NUM_DEV];
static int ctr_err_stop[MAX_NUM_DEV];
static int ctr_err_zero[MAX_NUM_DEV];
static int ctr_err_dest[MAX_NUM_DEV];
static int ctr_err_load[MAX_NUM_DEV];
static int ctr_err_safe[MAX_NUM_DEV];
static int ctr_err_sync_01;
static int ctr_err_sync_23;
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
static int tmp_safe[MAX_NUM_DEV];
static int sub_01;
static int sub_23;
static int sub;
static int tmp_sync_01;
static int tmp_sync_23;
static int tmp_sync;
static int result[MAX_NUM_DEV];
static int all_zero;
static int all_dest;
static int all_safe;
static int any_fault;
static int plan_vel[MAX_NUM_DEV];
static int plan_len_pass[MAX_NUM_DEV];
static int plan_len_posi[MAX_NUM_DEV];
static int plan_len_nega[MAX_NUM_DEV];
static int i;
static int j;

void t_swh(void) /* Task: SWing arm of Horizontal */
{
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(MSG, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify.type) {
                        case CMD_IDLE:
                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                switch (cmd.type) {
                                case CMD_IDLE:
                                case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
                                case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                                case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
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
                                case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                switch (cmd.type) {
                                case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                                case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
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
                                case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
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
                                tmp_zero[i] = filter_judge(&ctr_ok_zero[i], &ctr_err_zero[i], avg_pos[i], min_pos[i], pos_zero[i], MAX_LEN_CLLST);
                                tmp_dest[i] = filter_judge(&ctr_ok_dest[i], &ctr_err_dest[i], avg_pos[i], pos_dest[i], max_pos[i], MAX_LEN_CLLST);
                                tmp_load[i] = filter_judge(&ctr_ok_load[i], &ctr_err_load[i], avg_ampr[i], ampr_load[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_safe[i] = filter_judge(&ctr_ok_safe[i], &ctr_err_safe[i], avg_pos[i], pos_safe[i], max_pos[i], MAX_LEN_CLLST);
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
                                if (tmp_safe[i] == 1)
                                        result[i] |= RESULT_SAFE;
                                else if (tmp_safe[i] == -1)
                                        result[i] &= ~RESULT_SAFE;
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
                        sub_01 = avg_pos[0] - avg_pos[1];
                        sub_23 = avg_pos[2] - avg_pos[3];
                        sub = (avg_pos[0] - avg_pos[3] + avg_pos[1] - avg_pos[2]) / 2;
                        tmp_sync_01 = filter_judge(&ctr_ok_sync_01, &ctr_err_sync_01, sub_01, -err_sync_01, err_sync_01, MAX_LEN_CLLST);
                        tmp_sync_23 = filter_judge(&ctr_ok_sync_23, &ctr_err_sync_23, sub_23, -err_sync_23, err_sync_23, MAX_LEN_CLLST);
                        tmp_sync = filter_judge(&ctr_ok_sync, &ctr_err_sync, sub, -err_sync, err_sync, MAX_LEN_CLLST);
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
                        if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_AUTO)
                                any_fault &= UNMASK_RESULT_FAULT;
                        else if ((verify.type & UNMASK_CMD_MODE) == CMD_MODE_MANUAL)
                                any_fault = any_fault & UNMASK_RESULT_FAULT & ~RESULT_FAULT_SYNC;
                        if (any_fault) {
                                state.type = TASK_STATE_FAULT;
                                if (verify.type & UNMASK_CMD_ACT == CMD)
                                        verify.type = verify.type & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (all_zero)
                                        state.type = TASK_STATE_ZERO;
                                else if (all_safe)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= NOTIFY;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD | CMD_DIR_STOP | CMD_MODE_AUTO:
                        case CMD | CMD_DIR_STOP | CMD_MODE_MANUAL:
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
                                        if (result[i] & RESULT_STOP)
                                                tx[i].data.cmd.enable = 0x3C;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (tx[i].data.cmd.enable != 0x3C)
                                                break;
                                }
                                if (i == MAX_NUM_DEV)
                                        period = PERIOD_SLOW;
                                else
                                        period = PERIOD_FAST;
                                break;
                        case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (result[i] & RESULT_DEST) {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len_posi[i] = 0;
                                        } else {
                                                plan(&plan_vel[i], &plan_len_pass[i], plan_len_posi[i],
                                                     max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                tx[i].data.cmd.vel = sign[i] * (s16)plan_vel[i];
                                        }
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        tx[i].data.cmd.enable = 0xC3;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        if (result[i] & RESULT_ZERO && result[i] & RESULT_LOAD) {
                                                tx[i].data.cmd.vel = 0;
                                                plan_len_nega[i] = 0;
                                        } else {
                                                plan(&plan_vel[i], &plan_len_pass[i], plan_len_nega[i],
                                                     max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                tx[i].data.cmd.vel = -sign[i] * (s16)plan_vel[i];
                                        }
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        tx[i].data.cmd.enable = 0xC3;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].src = J1939_ADDR_MAIN;
                                                tx[i].dest = addr[i];
                                                tx[i].form = 0xA5;
                                                tx[i].prio = 0x08;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (result[i] & RESULT_DEST) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len_posi[i] = 0;
                                                } else {
                                                        plan(&plan_vel[i], &plan_len_pass[i], plan_len_posi[i],
                                                             max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                        tx[i].data.cmd.vel = sign[i] * (s16)plan_vel[i];
                                                }
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = 0x9A;
                                                tx[i].data.cmd.enable = 0xC3;
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        } else {
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
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].src = J1939_ADDR_MAIN;
                                                tx[i].dest = addr[i];
                                                tx[i].form = 0xA5;
                                                tx[i].prio = 0x08;
                                                tx[i].data.cmd.pos = 0x1100;
                                                if (result[i] & RESULT_ZERO && result[i] & RESULT_LOAD) {
                                                        tx[i].data.cmd.vel = 0;
                                                        plan_len_nega[i] = 0;
                                                } else {
                                                        plan(&plan_vel[i], &plan_len_pass[i], plan_len_nega[i],
                                                             max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                        tx[i].data.cmd.vel = -sign[i] * (s16)plan_vel[i];
                                                }
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = 0x9A;
                                                tx[i].data.cmd.enable = 0xC3;
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        } else {
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
                                }
                                period = PERIOD_FAST;
                                break;
                        default:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA5;
                                        tx[i].prio = 0x08;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0;
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = 0x9A;
                                        if (result[i] & RESULT_STOP)
                                                tx[i].data.cmd.enable = 0x3C;
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

void plan(int *vel, int *len_pass, int len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period)
{
        struct plan tmp;
        int acc = 0;
        if (len <= 0) {
                *vel = 0;
                return;
        }
        if (len < max_plan_len.low * 2) {
                tmp.low = len / 2;
                tmp.acc = 0;
                tmp.high = 0;
        } else if (len < max_plan_len.low * 2 + max_plan_len.acc * 2) {
                tmp.low = max_plan_len.low;
                tmp.acc = len / 2 - tmp.low;
                tmp.high = 0;
        } else {
                tmp.low = max_plan_len.low;
                tmp.acc = max_plan_len.acc;
                tmp.high = len - max_plan_len.low * 2 - max_plan_len.acc * 2;
        }
        if (max_plan_len.acc)
                acc = (plan_vel_high - plan_vel_low) * (plan_vel_high + plan_vel_low) * period / max_plan_len.acc / 2 / sysClkRateGet();
        else
                acc = 0;
        if (*len_pass < tmp.low) {
                *vel = plan_vel_low;
        } else if (*len_pass < tmp.low + tmp.acc) {
                if (*vel < plan_vel_high - acc)
                        *vel += acc;
        } else if (*len_pass < tmp.low + tmp.acc + tmp.high) {
                *vel = plan_vel_high;
        } else if (*len_pass < tmp.low + tmp.acc * 2 + tmp.high) {
                if (*vel > plan_vel_low + acc)
                        *vel -= acc;
        } else if (*len_pass < tmp.low * 2 + tmp.acc * 2 + tmp.high) {
                *vel = plan_vel_low;
        }
        *len_pass += *vel * period / sysClkRateGet();
}

int filter_judge(int *ok, int *err, int value, int min, int max, int ctr)
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

int max_index_of_n(int buf[], int n)
{
        int tmp;
        int i;
        int j = 0;
        for (i = 0; i < n; i++) {
                if (buf[j] < buf[i])
                        j = i;
        }
        return j;
}

int min_index_of_n(int buf[], int n)
{
        int tmp;
        int i;
        int j = 0;
        for (i = 0; i < n; i++) {
                if (buf[j] > buf[i])
                        j = i;
        }
        return j;
}

int max_of_n(int buf[], int n)
{
        if (n > 1)
                return max(*buf, max_of_n(buf + 1, n - 1));
        return *buf;
}

int min_of_n(int buf[], int n)
{
        if (n > 1)
                return min(*buf, min_of_n(buf + 1, n - 1));
        return *buf;
}
