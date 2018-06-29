#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200

#define MAX_NUM_FORM  3
#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_FAULT   0x0000FF00
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_VOLT_24  0x00000400
#define RESULT_FAULT_AMPR_24  0x00000800
#define RESULT_FAULT_VOLT_500 0x00001000
#define RESULT_FAULT_AMPR_500 0x00002000
#define RESULT_FAULT_COMM     0x00008000

typedef struct frame_psu_rx FRAME_RX;
typedef struct frame_psu_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int remap_form_index(u8 form);
int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
int psu_delay(int cur, int old);
u8 check_xor(u8 *buf, int n);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_psu;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int min_volt_24 = 2000;
const static int max_volt_24 = 2800;
const static int min_ampr_24 = 0;
const static int max_ampr_24 = 2000;
const static int min_volt_500 = 0;
const static int max_volt_500 = 4800;
const static int min_ampr_500 = 5400;
const static int max_ampr_500 = 1000;

static int period = PERIOD_SLOW;
static u32 prev;
static int len;
static u8 tmp[sizeof(struct frame_can)];
static struct main cmd;
static struct main verify = {CMD_IDLE, 0};
static struct main state;
static struct main old_state;
static struct frame_can can;
static struct frame_can rx[MAX_NUM_FORM][MAX_LEN_CLLST];
static FRAME_RX *p[MAX_NUM_FORM];
static FRAME_TX tx;
static int has_received;
static int cur_volt_24;
static int cur_ampr_24;
static int cur_volt_500;
static int cur_ampr_500;
static int sum_volt_24;
static int sum_ampr_24;
static int sum_volt_500;
static int sum_ampr_500;
static int avg_volt_24;
static int avg_ampr_24;
static int avg_volt_500;
static int avg_ampr_500;
static int old_fault;
static int old_cmd;
static int ctr_ok_volt_24;
static int ctr_ok_ampr_24;
static int ctr_ok_volt_500;
static int ctr_ok_ampr_500;
static int ctr_err_volt_24;
static int ctr_err_ampr_24;
static int ctr_err_volt_500;
static int ctr_err_ampr_500;
static int ctr_fault;
static int ctr_comm;
static int result;
static int tmp_volt_24;
static int tmp_ampr_24;
static int tmp_volt_500;
static int tmp_ampr_500;
static int any_fault;
static int i;

void t_psu(void) /* Task: Power Supply Unit */
{
        for (i = 0; i < MAX_NUM_FORM; i++)
                p[i] = (FRAME_RX *)can_cllst_init(rx[i], MAX_LEN_CLLST);
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_psu, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify.type) {
                        case CMD_IDLE:
                        case CMD_ACT_PSU_24 | CMD_DIR_POSI:
                        case CMD_ACT_PSU_24 | CMD_DIR_NEGA:
                        case CMD_ACT_PSU_500 | CMD_DIR_POSI:
                        case CMD_ACT_PSU_500 | CMD_DIR_NEGA:
                                verify = cmd;
                                break;
                        default:
                                break;
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = *(struct frame_can *)tmp;
                        has_received = 1;
                        i = remap_form_index(can.form);
                        switch (i) {
                        case 0:
                                p[i] = p[i]->next;
                                old_fault = p[i]->data.io.fault;
                                p[i]->data.io.fault = ((FRAME_RX *)&can)->data.io.fault;
                                if (old_fault == p[i]->data.io.fault) {
                                        if (ctr_fault < MAX_LEN_CLLST)
                                                ctr_fault++;
                                } else {
                                        ctr_fault = 0;
                                }
                                switch (p[i]->data.io.fault) {
                                case J1939_FAULT_NORMAL:
                                case J1939_FAULT_WARN:
                                        if (ctr_fault < 5)
                                                break;
                                        result &= ~RESULT_FAULT_GENERAL;
                                        result &= ~RESULT_FAULT_SERIOUS;
                                        break;
                                case J1939_FAULT_GENERAL:
                                        if (ctr_fault < 3)
                                                break;
                                        result |= RESULT_FAULT_GENERAL;
                                        break;
                                case J1939_FAULT_SERIOUS:
                                        result |= RESULT_FAULT_SERIOUS;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case 1:
                                p[i] = p[i]->next;
                                sum_volt_24 -= p[i]->data.state.volt_24;
                                sum_ampr_24 -= p[i]->data.state.ampr_24;
                                sum_volt_500 -= p[i]->data.state.volt_500;
                                sum_ampr_500 -= p[i]->data.state.ampr_500;
                                p[i]->data.state.volt_24 = ((FRAME_RX *)&can)->data.state.volt_24;
                                p[i]->data.state.ampr_24 = ((FRAME_RX *)&can)->data.state.ampr_24;
                                p[i]->data.state.volt_500 = ((FRAME_RX *)&can)->data.state.volt_500;
                                p[i]->data.state.ampr_500 = ((FRAME_RX *)&can)->data.state.ampr_500;
                                cur_volt_24 = p[i]->data.state.volt_24;
                                cur_ampr_24 = p[i]->data.state.ampr_24;
                                cur_volt_500 = p[i]->data.state.volt_500;
                                cur_ampr_500 = p[i]->data.state.ampr_500;
                                sum_volt_24 += p[i]->data.state.volt_24;
                                sum_ampr_24 += p[i]->data.state.ampr_24;
                                sum_volt_500 += p[i]->data.state.volt_500;
                                sum_ampr_500 += p[i]->data.state.ampr_500;
                                avg_volt_24 = sum_volt_24 / MAX_LEN_CLLST;
                                avg_ampr_24 = sum_ampr_24 / MAX_LEN_CLLST;
                                avg_volt_500 = sum_volt_500 / MAX_LEN_CLLST;
                                avg_ampr_500 = sum_ampr_500 / MAX_LEN_CLLST;
                                tmp_volt_24 = judge_filter(&ctr_ok_volt_24, &ctr_err_volt_24, avg_volt_24, min_volt_24, max_volt_24, MAX_LEN_CLLST);
                                tmp_ampr_24 = judge_filter(&ctr_ok_ampr_24, &ctr_err_ampr_24, avg_ampr_24, min_ampr_24, max_ampr_24, MAX_LEN_CLLST);
                                tmp_volt_500 = judge_filter(&ctr_ok_volt_500, &ctr_err_volt_500, avg_volt_500, min_volt_500, max_volt_500, MAX_LEN_CLLST);
                                tmp_ampr_500 = judge_filter(&ctr_ok_ampr_500, &ctr_err_ampr_500, avg_ampr_500, min_ampr_500, max_ampr_500, MAX_LEN_CLLST);
                                if (tmp_volt_24 == -1)
                                        result |= RESULT_FAULT_VOLT_24;
                                else if (tmp_volt_24 == 1)
                                        result &= ~RESULT_FAULT_VOLT_24;
                                if (tmp_ampr_24 == -1)
                                        result |= RESULT_FAULT_AMPR_24;
                                else if (tmp_ampr_24 == -1)
                                        result &= ~RESULT_FAULT_AMPR_24;
                                if (tmp_volt_500 == -1)
                                        result |= RESULT_FAULT_VOLT_500;
                                else if (tmp_volt_500 == 1)
                                        result &= ~RESULT_FAULT_VOLT_500;
                                if (tmp_ampr_500 == -1)
                                        result |= RESULT_FAULT_AMPR_500;
                                else if (tmp_ampr_500 == -1)
                                        result &= ~RESULT_FAULT_AMPR_500;
                                break;
                        case 2:
                                break;
                        default:
                                break;
                        }
                        period -= tickGet() - prev;
                        break;
                default:
                        if (has_received) {
                                has_received = 0;
                                if (ctr_comm < 0)
                                        ctr_comm = 0;
                                if (ctr_comm < MAX_LEN_CLLST)
                                        ctr_comm++;
                                if (ctr_comm == MAX_LEN_CLLST)
                                        result &= ~RESULT_FAULT_COMM;
                        } else {
                                if (ctr_comm > 0)
                                        ctr_comm = 0;
                                if (ctr_comm > -MAX_LEN_CLLST)
                                        ctr_comm--;
                                if (ctr_comm == -MAX_LEN_CLLST)
                                        result |= RESULT_FAULT_COMM;
                        }
                        any_fault = result & UNMASK_RESULT_FAULT;
                        if (any_fault)
                                state.type = TASK_STATE_FAULT;
                        else
                                state.type = TASK_STATE_RUNNING;
                        state.type = TASK_NOTIFY_PSU;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD_ACT_PSU_24 | CMD_DIR_POSI:
                                tx.src = J1939_ADDR_MAIN;
                                tx.dest = J1939_ADDR_PSU;
                                tx.form = J1939_FORM_PSU_CTRL;
                                tx.prio = J1939_PRIO_PSU_CTRL;
                                tx.data.io.v24 = psu_delay(verify.data, old_cmd);
                                old_cmd = tx.data.io.v24;
                                tx.data.io.res = 0x66;
                                tx.data.io.xor = check_xor((u8 *)&tx.data.io.v24, 7);
                                semTake(sem_can[0], WAIT_FOREVER);
                                rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
                                semGive(sem_can[0]);
                                period = PERIOD_SLOW;
                                break;
                        case CMD_ACT_PSU_24 | CMD_DIR_NEGA:
                                tx.src = J1939_ADDR_MAIN;
                                tx.dest = J1939_ADDR_PSU;
                                tx.form = J1939_FORM_PSU_CTRL;
                                tx.prio = J1939_PRIO_PSU_CTRL;
                                tx.data.io.v24 = 0;
                                tx.data.io.res = 0x66;
                                tx.data.io.xor = check_xor((u8 *)&tx.data.io.v24, 7);
                                semTake(sem_can[0], WAIT_FOREVER);
                                rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
                                semGive(sem_can[0]);
                                period = PERIOD_SLOW;
                                break;
                        case CMD_ACT_PSU_500 | CMD_DIR_POSI:
                                tx.src = J1939_ADDR_MAIN;
                                tx.dest = J1939_ADDR_PSU;
                                tx.form = J1939_FORM_PSU_CTRL;
                                tx.prio = J1939_PRIO_PSU_CTRL;
                                tx.data.io.v500 = verify.data;
                                tx.data.io.res = 0x66;
                                tx.data.io.xor = check_xor((u8 *)&tx.data.io.v24, 7);
                                semTake(sem_can[0], WAIT_FOREVER);
                                rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
                                semGive(sem_can[0]);
                                period = PERIOD_SLOW;
                                break;
                        case CMD_ACT_PSU_500 | CMD_DIR_NEGA:
                                tx.src = J1939_ADDR_MAIN;
                                tx.dest = J1939_ADDR_PSU;
                                tx.form = J1939_FORM_PSU_CTRL;
                                tx.prio = J1939_PRIO_PSU_CTRL;
                                tx.data.io.v500 = 0;
                                tx.data.io.res = 0x66;
                                tx.data.io.xor = check_xor((u8 *)&tx.data.io.v24, 7);
                                semTake(sem_can[0], WAIT_FOREVER);
                                rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
                                semGive(sem_can[0]);
                                period = PERIOD_SLOW;
                                break;
                        default:
                                tx.src = J1939_ADDR_MAIN;
                                tx.dest = J1939_ADDR_PSU;
                                tx.form = J1939_FORM_QUERY;
                                tx.prio = J1939_PRIO_QUERY;
                                tx.data.query[0] = 0x00;
                                tx.data.query[1] = 0x11;
                                tx.data.query[2] = 0x22;
                                tx.data.query[3] = 0x33;
                                tx.data.query[4] = 0x44;
                                tx.data.query[5] = 0x55;
                                tx.data.query[6] = 0x66;
                                tx.data.query[7] = 0x77;
                                semTake(sem_can[0], WAIT_FOREVER);
                                rngBufPut(rng_can[0], (char *)&tx, sizeof(tx));
                                semGive(sem_can[0]);
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}

int psu_delay(int cur, int old)
{
        int tmp;
        int i;
        if (cur == old)
                return cur;
        tmp = cur ^ old;
        for (i = 0; i < 32; i++) {
                if (tmp & 1 << i)
                        break;
        }
        if (cur & 1 << i)
                old |= 1 << i;
        else
                old &= ~(1 << i);
        return old;
}
