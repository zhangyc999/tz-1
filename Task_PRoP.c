#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#include <math.h>

#define PI 3.141592653589793
#define AA 1000000
#define BB 1000000

#define MSG    msg_prp
#define CMD    CMD_ACT_PRP
#define NOTIFY TASK_NOTIFY_PRP

#define PERIOD_SLOW 200
#define PERIOD_FAST 10

#define PRIO_SLOW 90
#define PRIO_FAST 40

#define MAX_NUM_DEV   4
#define MAX_NUM_FORM  4
#define MAX_LEN_CLLST 4

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
void plan(double *vel, double *len_pass, int len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period);
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
        J1939_ADDR_PRP0, J1939_ADDR_PRP1, J1939_ADDR_PRP2, J1939_ADDR_PRP3
};
const static int cable[MAX_NUM_DEV] = {0, 0, 1, 1};
const static int sign[MAX_NUM_DEV] = {1, 1, 1, 1};
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
const static int pos_mid_posi[MAX_NUM_DEV] = {10000, 10000, 10000, 10000};
const static int pos_mid_nega[MAX_NUM_DEV] = {10000, 10000, 10000, 10000};
const static int ampr_load[MAX_NUM_DEV] = {150, 150, 150, 150};
const static int err_sync = 1000;
const static struct plan plan_len_auto[MAX_NUM_DEV] = {
        {1000, 4000, 10000},
        {1000, 4000, 10000},
        {1000, 4000, 10000},
        {1000, 4000, 10000}
};
const static struct plan plan_len_manual[MAX_NUM_DEV] = {
        {1000, 8000, 2000},
        {1000, 8000, 2000},
        {1000, 8000, 2000},
        {1000, 8000, 2000}
};
const static struct plan plan_len_repair[MAX_NUM_DEV] = {
        {1000, 8000, 40000},
        {1000, 8000, 40000},
        {1000, 8000, 40000},
        {1000, 8000, 40000}
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
static FRAME_TX tx[MAX_NUM_DEV][MAX_NUM_FORM];
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
static int result[MAX_NUM_DEV] = {RESULT_STOP, RESULT_STOP, RESULT_STOP, RESULT_STOP};
static int all_stop = RESULT_STOP;
static int all_zero;
static int all_dest;
static int num_load;
static int any_fault;
static double plan_vel[MAX_NUM_DEV];
static double plan_len_pass[MAX_NUM_DEV];
static int plan_len_posi[MAX_NUM_DEV];
static int plan_len_nega[MAX_NUM_DEV];
static int plan_len[MAX_NUM_DEV];
static struct plan max_plan_len[MAX_NUM_DEV];
static int dir[MAX_NUM_DEV];
static int delta_posi[MAX_NUM_DEV];
static int delta_nega[MAX_NUM_DEV];
static int segement;
static int i;
static int j;

void t_prp(void) /* Task: PRoP */
{
        RING_ID rng_can[2] = {rng_can_slow[0], rng_can_slow[1]};
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
                tx[i][0].src = J1939_ADDR_MAIN;
                tx[i][0].dest = addr[i];
                tx[i][0].form = 0x5C;
                tx[i][0].prio = 0x0C;
                tx[i][0].data.query[0] = 0x00;
                tx[i][0].data.query[1] = 0x11;
                tx[i][0].data.query[2] = 0x22;
                tx[i][0].data.query[3] = 0x33;
                tx[i][0].data.query[4] = 0x44;
                tx[i][0].data.query[5] = 0x55;
                tx[i][0].data.query[6] = 0x66;
                tx[i][0].data.query[7] = 0x77;
                tx[i][1].src = J1939_ADDR_MAIN;
                tx[i][1].dest = addr[i];
                tx[i][1].form = 0xA5;
                tx[i][1].prio = 0x08;
                tx[i][1].data.cmd.pos = 0x1100;
                tx[i][1].data.cmd.vel = 0;
                tx[i][1].data.cmd.ampr = 1000;
                tx[i][1].data.cmd.exec = 0x9A;
                tx[i][1].data.cmd.enable = 0x3C;
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
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_AUTO:
                                                if (all_stop)
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
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_MANUAL:
                                                if (all_stop)
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
                                        case CMD | CMD_DIR_NEGA | CMD_MODE_REPAIR:
                                                if (all_stop)
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
                                        case CMD | CMD_DIR_POSI | CMD_MODE_AUTO:
                                                if (all_stop)
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
                                        case CMD | CMD_DIR_POSI | CMD_MODE_MANUAL:
                                                if (all_stop)
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
                                        case CMD | CMD_DIR_POSI | CMD_MODE_REPAIR:
                                                if (all_stop)
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
                                tmp_stop[i] = filter_judge(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -3, 3, MAX_LEN_CLLST);
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
                        all_stop = RESULT_STOP & result[0] & result[1] & result[2] & result[3];
                        all_zero = RESULT_ZERO & result[0] & result[1] & result[2] & result[3];
                        all_dest = RESULT_DEST & result[0] & result[1] & result[2] & result[3];
                        num_load = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                if (result[i] & RESULT_LOAD)
                                        num_load++;
                        }
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
                        if ((verify.type & UNMASK_CMD_MODE) != CMD_MODE_REPAIR) {
                                for (i = 0; i < MAX_NUM_DEV; i++)
                                        any_fault |= result[i];
                                any_fault &= UNMASK_RESULT_FAULT;
                        }
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
                        if ((verify.type & UNMASK_CMD_ACT) != CMD) {
                                if (num_load > 2)
                                        segement = 1;
                                else
                                        segement = 0;
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        j = 0;
                                        plan_vel[i] = 0;
                                        plan_len_pass[i] = 0;
                                        if (segement == 1) {
                                                plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                plan_len_nega[i] = cur_pos[i] - pos_mid_nega[i];
                                        } else {
                                                plan_len_posi[i] = pos_mid_posi[i] - cur_pos[i];
                                                plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                        }
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i][j], sizeof(tx[i][j]));
                                        semGive(sem_can[cable[i]]);
                                }
                                rng_can[0] = rng_can_slow[0];
                                rng_can[1] = rng_can_slow[1];
                                taskPrioritySet(taskIdSelf(), PRIO_SLOW);
                                period = PERIOD_SLOW;
                        } else if ((verify.type & UNMASK_CMD_DIR) == CMD_DIR_STOP) {
                                if (num_load > 2)
                                        segement = 1;
                                else
                                        segement = 0;
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        j = 1;
                                        plan_vel[i] = 0;
                                        plan_len_pass[i] = 0;
                                        if (segement == 1) {
                                                plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                plan_len_nega[i] = cur_pos[i] - pos_mid_nega[i];
                                        } else {
                                                plan_len_posi[i] = pos_mid_posi[i] - cur_pos[i];
                                                plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                        }
                                        tx[i][j].data.cmd.vel = 0;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i][j], sizeof(tx[i][j]));
                                        semGive(sem_can[cable[i]]);
                                }
                                if (all_stop) {
                                        rng_can[0] = rng_can_slow[0];
                                        rng_can[1] = rng_can_slow[1];
                                        taskPrioritySet(taskIdSelf(), PRIO_SLOW);
                                        period = PERIOD_SLOW;
#if 0
                                        for (i = 0; i < MAX_NUM_DEV; i++)
                                                tx[i][j].data.cmd.enable = 0x3C;
#endif
                                } else {
                                        rng_can[0] = rng_can_fast[0];
                                        rng_can[1] = rng_can_fast[1];
                                        taskPrioritySet(taskIdSelf(), PRIO_FAST);
                                        period = PERIOD_FAST;
                                }
                        } else {
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        j = 1;
                                        switch (verify.type & UNMASK_CMD_DIR) {
                                        case CMD_DIR_POSI:
                                                dir[i] = 1;
                                                if (segement == 0) {
                                                        if (num_load > 2) {
                                                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                                                        plan_len_pass[i] = 0;
                                                                        plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                                }
                                                                segement = 1;
                                                        }
                                                }
                                                plan_len[i] = plan_len_posi[i] + delta_posi[i];
                                                break;
                                        case CMD_DIR_NEGA:
                                                dir[i] = -1;
                                                if (segement == 1) {
                                                        if (num_load < 3) {
                                                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                                                        plan_len_pass[i] = 0;
                                                                        plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                                                }
                                                                segement = 0;
                                                        }
                                                }
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
                                                switch (verify.type & UNMASK_CMD_DIR) {
                                                case CMD_DIR_POSI:
                                                        plan_len[i] = pos_dest[i] - cur_pos[i];
                                                        break;
                                                case CMD_DIR_NEGA:
                                                        plan_len[i] = cur_pos[i] - pos_zero[i];
                                                        break;
                                                default:
                                                        plan_len[i] = 0;
                                                        break;
                                                }
                                                break;
                                        case CMD_MODE_REPAIR:
                                                max_plan_len[i] = plan_len_repair[i];
                                                plan_len[i] = max_plan_len[i].low * 2 + max_plan_len[i].acc * 2 + max_plan_len[i].high;
                                                break;
                                        default:
                                                break;
                                        }
                                        if (verify.data & 1 << i) {
                                                if ((verify.type & UNMASK_CMD_DIR) == CMD_DIR_POSI && result[i] & RESULT_DEST ||
                                                    (verify.type & UNMASK_CMD_DIR) == CMD_DIR_NEGA && result[i] & RESULT_ZERO) {
                                                        tx[i][j].data.cmd.vel = 0;
                                                        plan_len[i] = 0;
                                                        plan_len_pass[i] = 0;
                                                        if (segement == 1) {
                                                                plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                                plan_len_nega[i] = cur_pos[i] - pos_mid_nega[i];
                                                        } else {
                                                                plan_len_posi[i] = pos_mid_posi[i] - cur_pos[i];
                                                                plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                                        }
                                                } else {
                                                        plan(&plan_vel[i], &plan_len_pass[i], plan_len[i],
                                                             max_plan_len[i], plan_vel_low[i], plan_vel_high[i], PERIOD_FAST);
                                                        tx[i][j].data.cmd.vel = dir[i] * sign[i] * (s16)plan_vel[i];
                                                }
                                        } else {
                                                tx[i][j].data.cmd.vel = 0;
                                                plan_len[i] = 0;
                                                plan_len_pass[i] = 0;
                                                if (segement == 1) {
                                                        plan_len_posi[i] = pos_dest[i] - cur_pos[i];
                                                        plan_len_nega[i] = cur_pos[i] - pos_mid_nega[i];
                                                } else {
                                                        plan_len_posi[i] = pos_mid_posi[i] - cur_pos[i];
                                                        plan_len_nega[i] = cur_pos[i] - pos_zero[i];
                                                }
                                        }
                                        tx[i][j].data.cmd.enable = 0xC3;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i][j], sizeof(tx[i][j]));
                                        semGive(sem_can[cable[i]]);
                                }
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (tx[i][j].data.cmd.vel != 0)
                                                break;
                                }
                                if (all_stop && i == MAX_NUM_DEV) {
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
                        }
                        break;
                }
        }
}
