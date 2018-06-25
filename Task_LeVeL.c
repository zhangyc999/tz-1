#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200

#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_FAULT   0x0000FF00
#define UNMASK_RESULT_MISC    0xFFFF0000
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_COMM     0x00008000
#define RESULT_X_ZERO         0x00010000
#define RESULT_Y_ZERO         0x00020000
#define RESULT_OUT_X          0x00040000
#define RESULT_OUT_Y          0x00080000

typedef struct frame_lvl_rx FRAME_RX;
typedef struct frame_lvl_tx FRAME_TX;

int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int remap_form_index(u8 form);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_lvl;
extern MSG_Q_ID msg_rse;
extern MSG_Q_ID msg_prp;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int n = 2;
const static int max_form = 3;
const static int addr[2] = {J1939_ADDR_LVL0, J1939_ADDR_LVL1};
const static int cable[2] = {0, 0};
const static int min_x[2] = {-2000, -2000};
const static int max_x[2] = {2000, 2000};
const static int min_y[2] = {-2000, -2000};
const static int max_y[2] = {2000, 2000};
const static int x_zero[0] = {0, 0};
const static int y_zero[2] = {0, 0};

static int period = PERIOD_SLOW;
static u32 prev;
static int len;
static u8 tmp[sizeof(struct frame_can)];
static struct main cmd;
static struct main verify = {CMD_IDLE, 0};
static struct main state;
static struct main old_state;
static struct frame_can can;
static struct frame_can rx[2][3][MAX_LEN_CLLST];
static FRAME_RX *p[2][3];
static FRAME_TX tx[2];
static int has_received[2];
static int cur_x[2];
static int cur_y[4];
static int sum_x[2];
static int sum_y[2];
static int avg_x[2];
static int avg_y[2];
static int old_fault[2];
static int ctr_ok_x[2];
static int ctr_ok_y[2];
static int ctr_ok_x_zero[2];
static int ctr_ok_y_zero[2];
static int ctr_err_x[2];
static int ctr_err_y[2];
static int ctr_err_x_zero[2];
static int ctr_err_y_zero[2];
static int ctr_fault[2];
static int ctr_comm[2];
static int result[2];
static int tmp_x;
static int tmp_y;
static int tmp_x_zero;
static int tmp_y_zero;
static int all_fault;
static int use;
static int i;
static int j;

void t_lvl(void) /* Task: LeVeL tilt sensor */
{
        for (i = 0; i < n; i++) {
                for (j = 0; j < max_form; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_lvl, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = *(struct frame_can *)tmp;
                        switch (can.src) {
                        case J1939_ADDR_LVL0:
                                i = 0;
                                break;
                        case J1939_ADDR_LVL1:
                                i = 1;
                                break;
                        default:
                                break;
                        }
                        has_received[i] = 1;
                        j = remap_form_index(can.form);
                        switch (j) {
                        case 0:
                                break;
                        case 1:
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
                                tmp_x = judge_filter(&ctr_ok_x[i], &ctr_err_x[i], avg_x[i], min_x[i], max_x[i], MAX_LEN_CLLST);
                                tmp_y = judge_filter(&ctr_ok_y[i], &ctr_err_y[i], avg_y[i], min_y[i], max_y[i], MAX_LEN_CLLST);
                                tmp_x_zero = judge_filter(&ctr_ok_x_zero[i], &ctr_err_x_zero[i], avg_x[i], min_x[i], x_zero[i], MAX_LEN_CLLST);
                                tmp_y_zero = judge_filter(&ctr_ok_y_zero[i], &ctr_err_y_zero[i], avg_y[i], min_y[i], y_zero[i], MAX_LEN_CLLST);
                                if (tmp_x == -1)
                                        result[i] |= RESULT_OUT_X;
                                else if (tmp_x == 1)
                                        result[i] &= ~RESULT_OUT_X;
                                if (tmp_y == -1)
                                        result[i] |= RESULT_OUT_Y;
                                else if (tmp_y == 1)
                                        result[i] &= ~RESULT_OUT_Y;
                                if (tmp_x_zero == 1)
                                        result[i] |= RESULT_X_ZERO;
                                else if (tmp_x_zero == -1)
                                        result[i] &= ~RESULT_X_ZERO;
                                if (tmp_y_zero == 1)
                                        result[i] |= RESULT_Y_ZERO;
                                else if (tmp_y_zero == -1)
                                        result[i] &= ~RESULT_Y_ZERO;
                                break;
                        case 2:
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
                        all_fault = (result[0] & result[1]) & UNMASK_RESULT_FAULT;
                        if (all_fault) {
                                state.type = TASK_STATE_FAULT;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (result[0] & UNMASK_RESULT_FAULT)
                                        use = 1;
                                else
                                        use = 0;
                                if (result[use] & RESULT_X_ZERO && result[use] & RESULT_Y_ZERO)
                                        state.type = TASK_STATE_ZERO;
                        }
                        state.type |= TASK_NOTIFY_LVL;
                        if (all_fault)
                                state.data = 0;
                        else
                                state.data = avg_x[use] | avg_y[use] << 16;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_rse, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        msgQSend(msg_prp, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
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
        }
}
