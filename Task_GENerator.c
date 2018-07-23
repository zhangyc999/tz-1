#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD 200

#define MAX_NUM_DEV   2
#define MAX_NUM_FORM  4
#define MAX_LEN_CLLST 4

#define UNMASK_RESULT_FAULT  0x0000FF00
#define UNMASK_RESULT_MISC   0xFFFF0000
#define RESULT_FAULT_GENERAL 0x00000100
#define RESULT_FAULT_SERIOUS 0x00000200
#define RESULT_FAULT_VOLT    0x00000400
#define RESULT_FAULT_AMPR    0x00000800
#define RESULT_FAULT_RPM     0x00001000
#define RESULT_FAULT_TEMP    0x00002000
#define RESULT_FAULT_COMM    0x00008000

typedef struct frame_gen_rx FRAME_RX;
typedef struct frame_gen_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int filter_judge(int *ok, int *err, int value, int min, int max, int ctr);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_gen;
extern RING_ID rng_can_slow[];
extern SEM_ID sem_can_slow[];

const static int addr[MAX_NUM_DEV] = {
        J1939_ADDR_GEND, J1939_ADDR_GENS
};
const static int cable[MAX_NUM_DEV] = {1, 1};
const static int min_volt[MAX_NUM_DEV] = {0, 0};
const static int max_volt[MAX_NUM_DEV] = {5400, 5400};
const static int min_ampr[MAX_NUM_DEV] = {0, 0};
const static int max_ampr[MAX_NUM_DEV] = {2000, 2000};
const static int min_rpm[MAX_NUM_DEV] = {0, 0};
const static int max_rpm[MAX_NUM_DEV] = {1000, 1000};
const static int min_temp[MAX_NUM_DEV] = {-80, -80};
const static int max_temp[MAX_NUM_DEV] = {100, 100};

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
static FRAME_TX tx[MAX_NUM_DEV][MAX_NUM_FORM];
static int has_received[MAX_NUM_DEV];
static int cur_volt[MAX_NUM_DEV];
static int cur_ampr[MAX_NUM_DEV];
static int cur_rpm[MAX_NUM_DEV];
static int cur_temp[MAX_NUM_DEV];
static int sum_volt[MAX_NUM_DEV];
static int sum_ampr[MAX_NUM_DEV];
static int sum_rpm[MAX_NUM_DEV];
static int sum_temp[MAX_NUM_DEV];
static int avg_volt[MAX_NUM_DEV];
static int avg_ampr[MAX_NUM_DEV];
static int avg_rpm[MAX_NUM_DEV];
static int avg_temp[MAX_NUM_DEV];
static int old_fault[MAX_NUM_DEV];
static int ctr_ok_volt[MAX_NUM_DEV];
static int ctr_ok_ampr[MAX_NUM_DEV];
static int ctr_ok_rpm[MAX_NUM_DEV];
static int ctr_ok_temp[MAX_NUM_DEV];
static int ctr_err_volt[MAX_NUM_DEV];
static int ctr_err_ampr[MAX_NUM_DEV];
static int ctr_err_rpm[MAX_NUM_DEV];
static int ctr_err_temp[MAX_NUM_DEV];
static int ctr_fault[MAX_NUM_DEV];
static int ctr_comm[MAX_NUM_DEV];
static int tmp_volt[MAX_NUM_DEV];
static int tmp_ampr[MAX_NUM_DEV];
static int tmp_rpm[MAX_NUM_DEV];
static int tmp_temp[MAX_NUM_DEV];
static int result[MAX_NUM_DEV];
static int any_fault;
static int i;
static int j;

void t_gen(void) /* Task: GENerator */
{
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
                tx[i][0].src = J1939_ADDR_MAIN;
                tx[i][0].dest = addr[i];
                tx[i][0].form = 0x6E;
                tx[i][0].prio = 0x08;
                tx[i][0].data.cmd.type = 0x11;
                tx[i][0].data.cmd.enable = 0xBB;
                tx[i][0].data.cmd.toggle = 0x00;
                tx[i][0].data.cmd.protect = 0xAA;
                tx[i][0].data.cmd.res0 = 0x00;
                tx[i][0].data.cmd.res1 = 0x00;
                tx[i][0].data.cmd.res2 = 0x00;
                tx[i][0].data.cmd.res3 = 0x00;
                tx[i][1].src = J1939_ADDR_MAIN;
                tx[i][1].dest = addr[i];
                tx[i][1].form = 0x6E;
                tx[i][1].prio = 0x08;
                tx[i][1].data.cmd.type = 0x12;
                tx[i][1].data.cmd.enable = 0xBB;
                tx[i][1].data.cmd.toggle = 0x00;
                tx[i][1].data.cmd.protect = 0xAA;
                tx[i][1].data.cmd.res0 = 0x00;
                tx[i][1].data.cmd.res1 = 0x00;
                tx[i][1].data.cmd.res2 = 0x00;
                tx[i][1].data.cmd.res3 = 0x00;
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD)
                        period = 0;
                len = msgQReceive(msg_gen, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify.type) {
                        case CMD_IDLE:
                        case CMD_ACT_GEND | CMD_DIR_POSI:
                        case CMD_ACT_GEND | CMD_DIR_NEGA:
                        case CMD_ACT_GENS | CMD_DIR_POSI:
                        case CMD_ACT_GENS | CMD_DIR_NEGA:
                                verify = cmd;
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
                        case 0x96:
                                j = 0;
                                p[i][j] = p[i][j]->next;
                                old_fault[i] = p[i][j]->data.misc.fault;
                                p[i][j]->data.misc.fault = ((FRAME_RX *)&can)->data.misc.fault;
                                if (old_fault[i] == p[i][j]->data.misc.fault) {
                                        if (ctr_fault[i] < MAX_LEN_CLLST)
                                                ctr_fault[i]++;
                                } else {
                                        ctr_fault[i] = 0;
                                }
                                if (ctr_fault[i] > 1) {
                                        if (p[i][j]->data.misc.fault) {
                                                result[i] |= RESULT_FAULT_GENERAL;
                                                result[i] |= RESULT_FAULT_SERIOUS;
                                        } else {
                                                result[i] &= ~RESULT_FAULT_GENERAL;
                                                result[i] &= ~RESULT_FAULT_SERIOUS;
                                        }
                                }
                                break;
                        case 0x97:
                                j = 1;
                                p[i][j] = p[i][j]->next;
                                sum_volt[i] -= p[i][j]->data.state.volt;
                                sum_ampr[i] -= p[i][j]->data.state.ampr;
                                sum_rpm[i] -= p[i][j]->data.state.rpm;
                                sum_temp[i] -= p[i][j]->data.state.temp;
                                p[i][j]->data.state.volt = ((FRAME_RX *)&can)->data.state.volt;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.rpm = ((FRAME_RX *)&can)->data.state.rpm;
                                p[i][j]->data.state.temp = ((FRAME_RX *)&can)->data.state.temp;
                                cur_volt[i] = p[i][j]->data.state.volt;
                                cur_ampr[i] = p[i][j]->data.state.ampr;
                                cur_rpm[i] = p[i][j]->data.state.rpm;
                                cur_temp[i] = p[i][j]->data.state.temp;
                                sum_volt[i] += p[i][j]->data.state.volt;
                                sum_ampr[i] += p[i][j]->data.state.ampr;
                                sum_rpm[i] += p[i][j]->data.state.rpm;
                                sum_temp[i] += p[i][j]->data.state.temp;
                                avg_volt[i] = sum_volt[i] / MAX_LEN_CLLST;
                                avg_ampr[i] = sum_ampr[i] / MAX_LEN_CLLST;
                                avg_rpm[i] = sum_rpm[i] / MAX_LEN_CLLST;
                                avg_temp[i] = sum_temp[i] / MAX_LEN_CLLST;
                                tmp_volt[i] = filter_judge(&ctr_ok_volt[i], &ctr_err_volt[i], avg_volt[i], min_volt[i], max_volt[i], MAX_LEN_CLLST);
                                tmp_ampr[i] = filter_judge(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_rpm[i] = filter_judge(&ctr_ok_rpm[i], &ctr_err_rpm[i], avg_rpm[i], min_rpm[i], max_rpm[i], MAX_LEN_CLLST);
                                tmp_temp[i] = filter_judge(&ctr_ok_temp[i], &ctr_err_temp[i], avg_temp[i], min_temp[i], max_temp[i], MAX_LEN_CLLST);
                                if (tmp_volt[i] == -1)
                                        result[i] |= RESULT_FAULT_VOLT;
                                else if (tmp_volt[i] == 1)
                                        result[i] &= ~RESULT_FAULT_VOLT;
                                if (tmp_ampr[i] == -1)
                                        result[i] |= RESULT_FAULT_AMPR;
                                else if (tmp_ampr[i] == -1)
                                        result[i] &= ~RESULT_FAULT_AMPR;
                                if (tmp_rpm[i] == -1)
                                        result[i] |= RESULT_FAULT_RPM;
                                else if (tmp_rpm[i] == 1)
                                        result[i] &= ~RESULT_FAULT_RPM;
                                if (tmp_temp[i] == -1)
                                        result[i] |= RESULT_FAULT_TEMP;
                                else if (tmp_temp[i] == -1)
                                        result[i] &= ~RESULT_FAULT_TEMP;
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
                        any_fault = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++)
                                any_fault |= result[i];
                        any_fault &= UNMASK_RESULT_FAULT;
                        if (any_fault)
                                state.type = TASK_STATE_FAULT;
                        else
                                state.type = TASK_STATE_RUNNING;
                        state.type |= TASK_NOTIFY_GEN;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        if ((verify.type & UNMASK_CMD_ACT) != CMD_ACT_GEND) {
                                i = 0;
                                j = 0;
                        } else {
                                i = 0;
                                j = 1;
                                switch (verify.type & UNMASK_CMD_DIR) {
                                case CMD_DIR_POSI:
                                        tx[i][j].data.cmd.toggle = 0x55;
                                        break;
                                case CMD_DIR_NEGA:
                                        tx[i][j].data.cmd.toggle = 0x00;
                                        break;
                                default:
                                        break;
                                }
                        }
                        semTake(sem_can_slow[cable[i]], WAIT_FOREVER);
                        rngBufPut(rng_can_slow[cable[i]], (char *)&tx[i][j], sizeof(tx[i][j]));
                        semGive(sem_can_slow[cable[i]]);
                        if ((verify.type & UNMASK_CMD_ACT) != CMD_ACT_GENS) {
                                i = 1;
                                j = 0;
                        } else {
                                i = 1;
                                j = 1;
                                switch (verify.type & UNMASK_CMD_DIR) {
                                case CMD_DIR_POSI:
                                        tx[i][j].data.cmd.toggle = 0x55;
                                        break;
                                case CMD_DIR_NEGA:
                                        tx[i][j].data.cmd.toggle = 0x00;
                                        break;
                                default:
                                        break;
                                }
                        }
                        semTake(sem_can_slow[cable[i]], WAIT_FOREVER);
                        rngBufPut(rng_can_slow[cable[i]], (char *)&tx[i][j], sizeof(tx[i][j]));
                        semGive(sem_can_slow[cable[i]]);
                        period = PERIOD;
                        break;
                }
        }
}
