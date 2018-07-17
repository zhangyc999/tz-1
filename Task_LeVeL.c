#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD 200

#define MAX_NUM_DEV   2
#define MAX_NUM_FORM  1
#define MAX_LEN_CLLST 4

#define UNMASK_RESULT_FAULT  0x0000FF00
#define UNMASK_RESULT_MISC   0xFFFF0000
#define RESULT_FAULT_GENERAL 0x00000100
#define RESULT_FAULT_SERIOUS 0x00000200
#define RESULT_FAULT_COMM    0x00008000
#define RESULT_X             0x00010000
#define RESULT_Y             0x00020000

typedef struct frame_lvl_rx FRAME_RX;
typedef struct frame_lvl_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int filter_judge(int *ok, int *err, int value, int min, int max, int ctr);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_lvl;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_prp;
extern RING_ID rng_can_slow[];
extern SEM_ID sem_can[];

const static int addr[MAX_NUM_DEV] = {
        J1939_ADDR_LVL0, J1939_ADDR_LVL1
};
const static int cable[MAX_NUM_DEV] = {0, 0};
const static int min_x[MAX_NUM_DEV] = {-1000, -1000};
const static int max_x[MAX_NUM_DEV] = {1000, 1000};
const static int min_y[MAX_NUM_DEV] = {-1000, -1000};
const static int max_y[MAX_NUM_DEV] = {1000, 1000};

static int period = PERIOD;
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
static int cur_x[MAX_NUM_DEV];
static int cur_y[MAX_NUM_DEV];
static int sum_x[MAX_NUM_DEV];
static int sum_y[MAX_NUM_DEV];
static int avg_x[MAX_NUM_DEV];
static int avg_y[MAX_NUM_DEV];
static int old_fault[MAX_NUM_DEV];
static int ctr_ok_x[MAX_NUM_DEV];
static int ctr_ok_y[MAX_NUM_DEV];
static int ctr_err_x[MAX_NUM_DEV];
static int ctr_err_y[MAX_NUM_DEV];
static int ctr_fault[MAX_NUM_DEV];
static int ctr_comm[MAX_NUM_DEV];
static int tmp_x[MAX_NUM_DEV];
static int tmp_y[MAX_NUM_DEV];
static int result[MAX_NUM_DEV];
static int both_fault;
static int use;
static int i;
static int j;

void t_lvl(void) /* Task: LeVeL tilt sensor */
{
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD)
                        period = 0;
                len = msgQReceive(msg_lvl, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
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
                        case 0xC3:
                                j = 0;
                                p[i][j] = p[i][j]->next;
                                sum_x[i] -= p[i][j]->data.state.x;
                                sum_y[i] -= p[i][j]->data.state.y;
                                old_fault[i] = p[i][j]->data.state.fault;
                                p[i][j]->data.state.x = ((FRAME_RX *)&can)->data.state.x;
                                p[i][j]->data.state.y = ((FRAME_RX *)&can)->data.state.y;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                cur_x[i] = p[i][j]->data.state.x;
                                cur_y[i] = p[i][j]->data.state.y;
                                sum_x[i] += p[i][j]->data.state.x;
                                sum_y[i] += p[i][j]->data.state.y;
                                avg_x[i] = sum_x[i] / MAX_LEN_CLLST;
                                avg_y[i] = sum_y[i] / MAX_LEN_CLLST;
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
                                tmp_x[i] = filter_judge(&ctr_ok_x[i], &ctr_err_x[i], avg_x[i], min_x[i], max_x[i], MAX_LEN_CLLST);
                                tmp_y[i] = filter_judge(&ctr_ok_y[i], &ctr_err_y[i], avg_y[i], min_y[i], max_y[i], MAX_LEN_CLLST);
                                if (tmp_x[i] == 1)
                                        result[i] |= RESULT_X;
                                else if (tmp_x[i] == -1)
                                        result[i] &= ~RESULT_X;
                                if (tmp_y[i] == 1)
                                        result[i] |= RESULT_Y;
                                else if (tmp_y[i] == -1)
                                        result[i] &= ~RESULT_Y;
                                break;
                        default:
                                break;
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
                        both_fault = (result[0] & result[1]) & UNMASK_RESULT_FAULT;
                        if (both_fault) {
                                state.type = TASK_STATE_FAULT;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (result[0] & UNMASK_RESULT_FAULT)
                                        use = 1;
                                else
                                        use = 0;
                                if (result[use] & RESULT_X && result[use] & RESULT_Y)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= TASK_NOTIFY_LVL;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        state.data = (int)(avg_x[use]) & 0x0000FFFF | (int)avg_y[use] << 16;
                        if (old_state.type != state.type || old_state.data != state.data) {
                                msgQSend(msg_rse, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_prp, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        }
                        old_state = state;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                tx[i].src = J1939_ADDR_MAIN;
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
                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                rngBufPut(rng_can_slow[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                semGive(sem_can[cable[i]]);
                        }
                        period = PERIOD;
                        break;
                }
        }
}
