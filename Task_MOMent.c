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

extern RING_ID rng_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_mom;

extern void plan(int *vel, int *len_pass, int period, int len_low, int len_acc, int len_high, int vel_low, int vel_high);
extern int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
extern struct frame_can *can_cllst_init(struct frame_can buf[], int len);

const static int n = 4;
const static int max_form = 3;
const static int addr[4] = {J1939_ADDR_MOM0, J1939_ADDR_MOM1, J1939_ADDR_MOM2, J1939_ADDR_MOM3};
const static int cable[4] = {1, 1, 1, 1};
const static int ampr_value[4] = {50, 50, 50, 50};
const static int min_vel[4] = {-1500, -1500, -1500, -1500};
const static int max_vel[4] = {1500, 1500, 1500, 1500};
const static int min_ampr[4] = {-2000, -2000, -2000, -2000};
const static int max_ampr[4] = {2000, 2000, 2000, 2000};
const static int safe_ampr[4] = {100, 100, 100, 100};

static int period = PERIOD_SLOW;
static u32 prev;
static int len;
static u8 tmp[sizeof(struct frame_can)];
static struct main cmd;
static struct main state;
static struct main old_state;
static struct frame_can can;
static struct frame_can rx[4][3][MAX_LEN_CLLST];
static FRAME_RX *p[4][3];
static FRAME_TX tx[4];
static int verify = CMD_IDLE;
static int has_received[4];
static int cur_vel[4];
static int cur_ampr[4];
static int sum_vel[4];
static int sum_ampr[4];
static int avg_vel[4];
static int avg_ampr[4];
static int old_fault[4];
static int old_io[4];
static int ctr_ok_vel[4];
static int ctr_ok_ampr[4];
static int ctr_ok_sync;
static int ctr_ok_stop[4];
static int ctr_err_vel[4];
static int ctr_err_ampr[4];
static int ctr_err_sync;
static int ctr_err_stop[4];
static int ctr_fault[4];
static int ctr_io[4];
static int ctr_comm[4];
static int result[4];
static int tmp_vel;
static int tmp_ampr;
static int tmp_running;
static int i;
static int j;

void t_mom(void) /* Task: constant MOMent arm */
{
        for (i = 0; i < n; i++) {
                for (j = 0; j < max_form; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_mom, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify & UNMASK_CMD_ACT) {
                        case CMD_IDLE:
                        case CMD_ACT_MOM:
                                verify = cmd.type;
                                break;
                        default:
                                break;
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = *(struct frame_can *)tmp;
                        switch (can.src) {
                        case J1939_ADDR_MOM0:
                                i = 0;
                                break;
                        case J1939_ADDR_MOM1:
                                i = 1;
                                break;
                        case J1939_ADDR_MOM2:
                                i = 2;
                                break;
                        case J1939_ADDR_MOM3:
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
                                sum_vel[i] -= p[i][j]->data.state.vel;
                                sum_ampr[i] -= p[i][j]->data.state.ampr;
                                old_fault[i] = p[i][j]->data.state.fault;
                                old_io[i] = p[i][j]->data.state.io;
                                p[i][j]->data.state.vel = ((FRAME_RX *)&can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                cur_vel[i] = p[i][j]->data.state.vel;
                                cur_ampr[i] = p[i][j]->data.state.ampr;
                                sum_vel[i] += p[i][j]->data.state.vel;
                                sum_ampr[i] += p[i][j]->data.state.ampr;
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
                                        if (ctr_io[i] < 10)
                                                ctr_io[i]++;
                                } else {
                                        ctr_io[i] = 0;
                                }
                                if (ctr_io[i] > 5)
                                        result[i] = result[i] & ~UNMASK_RESULT_IO | p[i][j]->data.state.io;
                                tmp_vel = judge_filter(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], MAX_LEN_CLLST);
                                tmp_ampr = judge_filter(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_running = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, MAX_LEN_CLLST);
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
                        state.type = TASK_NOTIFY_MOM;
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
                                if (avg_ampr[i] < safe_ampr[i])
                                        break;
                        }
                        if (i != n)
                                state.type |= TASK_STATE_LOCK;
                        else
                                state.type |= TASK_STATE_UNLOCK;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify & UNMASK_CMD_ACT) {
                        case CMD_ACT_MOM:
                                for (i = 0; i < n; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_AMPR;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0x3322;
                                        if (avg_ampr[i] > ampr_value[i] + 5)
                                                tx[i].data.cmd.ampr = (s16)(avg_ampr[i] - 5);
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
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
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                }
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}
