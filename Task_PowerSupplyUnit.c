#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD 20

#define MAX_NUM_DEV   1
#define MAX_NUM_FORM  2
#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_FAULT   0x0000FF00
#define UNMASK_RESULT_MISC    0xFFFF0000
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
int filter_judge(int *ok, int *err, int value, int min, int max, int ctr);
int psu_delay(int cur, int old);
u8 check_xor(u8 *buf, int n);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_psu;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int addr[MAX_NUM_DEV] = {
        J1939_ADDR_PSU
};
const static int cable[MAX_NUM_DEV] = {0};
const static int min_volt_24[MAX_NUM_DEV] = {2000};
const static int max_volt_24[MAX_NUM_DEV] = {2800};
const static int min_ampr_24[MAX_NUM_DEV] = {0};
const static int max_ampr_24[MAX_NUM_DEV] = {2000};
const static int min_volt_500[MAX_NUM_DEV] = {4600};
const static int max_volt_500[MAX_NUM_DEV] = {5400};
const static int min_ampr_500[MAX_NUM_DEV] = {0};
const static int max_ampr_500[MAX_NUM_DEV] = {1000};

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
static int cur_volt_24[MAX_NUM_DEV];
static int cur_ampr_24[MAX_NUM_DEV];
static int cur_volt_500[MAX_NUM_DEV];
static int cur_ampr_500[MAX_NUM_DEV];
static int sum_volt_24[MAX_NUM_DEV];
static int sum_ampr_24[MAX_NUM_DEV];
static int sum_volt_500[MAX_NUM_DEV];
static int sum_ampr_500[MAX_NUM_DEV];
static int avg_volt_24[MAX_NUM_DEV];
static int avg_ampr_24[MAX_NUM_DEV];
static int avg_volt_500[MAX_NUM_DEV];
static int avg_ampr_500[MAX_NUM_DEV];
static int old_fault[MAX_NUM_DEV];
static int old_cmd[MAX_NUM_DEV];
static int ctr_ok_volt_24[MAX_NUM_DEV];
static int ctr_ok_ampr_24[MAX_NUM_DEV];
static int ctr_ok_volt_500[MAX_NUM_DEV];
static int ctr_ok_ampr_500[MAX_NUM_DEV];
static int ctr_err_volt_24[MAX_NUM_DEV];
static int ctr_err_ampr_24[MAX_NUM_DEV];
static int ctr_err_volt_500[MAX_NUM_DEV];
static int ctr_err_ampr_500[MAX_NUM_DEV];
static int ctr_fault[MAX_NUM_DEV];
static int ctr_comm[MAX_NUM_DEV];
static int tmp_volt_24[MAX_NUM_DEV];
static int tmp_ampr_24[MAX_NUM_DEV];
static int tmp_volt_500[MAX_NUM_DEV];
static int tmp_ampr_500[MAX_NUM_DEV];
static int result[MAX_NUM_DEV];
static int any_fault;
static int i;
static int j;

void t_psu(void) /* Task: Power Supply Unit */
{
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
                        p[i][j] = (FRAME_RX *)can_cllst_init(rx[i][j], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD)
                        period = 0;
                len = msgQReceive(msg_psu, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify.type) {
                        case CMD_IDLE:
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_BRAKE:
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_BRAKE:
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_LIGHT:
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_LIGHT:
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_24:
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_24:
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_500:
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_500:
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
                        case 0xC0:
                                j = 0;
                                p[i][j] = p[i][j]->next;
                                old_fault[i] = p[i][j]->data.io.fault;
                                p[i][j]->data.io.fault = ((FRAME_RX *)&can)->data.io.fault;
                                if (old_fault[i] == p[i][j]->data.io.fault) {
                                        if (ctr_fault[i] < MAX_LEN_CLLST)
                                                ctr_fault[i]++;
                                } else {
                                        ctr_fault[i] = 0;
                                }
                                switch (p[i][j]->data.io.fault) {
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
                                break;
                        case 0xC3:
                                j = 1;
                                p[i][j] = p[i][j]->next;
                                sum_volt_24[i] -= p[i][j]->data.state.volt_24;
                                sum_ampr_24[i] -= p[i][j]->data.state.ampr_24;
                                sum_volt_500[i] -= p[i][j]->data.state.volt_500;
                                sum_ampr_500[i] -= p[i][j]->data.state.ampr_500;
                                p[i][j]->data.state.volt_24 = ((FRAME_RX *)&can)->data.state.volt_24;
                                p[i][j]->data.state.ampr_24 = ((FRAME_RX *)&can)->data.state.ampr_24;
                                p[i][j]->data.state.volt_500 = ((FRAME_RX *)&can)->data.state.volt_500;
                                p[i][j]->data.state.ampr_500 = ((FRAME_RX *)&can)->data.state.ampr_500;
                                cur_volt_24[i] = p[i][j]->data.state.volt_24;
                                cur_ampr_24[i] = p[i][j]->data.state.ampr_24;
                                cur_volt_500[i] = p[i][j]->data.state.volt_500;
                                cur_ampr_500[i] = p[i][j]->data.state.ampr_500;
                                sum_volt_24[i] += p[i][j]->data.state.volt_24;
                                sum_ampr_24[i] += p[i][j]->data.state.ampr_24;
                                sum_volt_500[i] += p[i][j]->data.state.volt_500;
                                sum_ampr_500[i] += p[i][j]->data.state.ampr_500;
                                avg_volt_24[i] = sum_volt_24[i] / MAX_LEN_CLLST;
                                avg_ampr_24[i] = sum_ampr_24[i] / MAX_LEN_CLLST;
                                avg_volt_500[i] = sum_volt_500[i] / MAX_LEN_CLLST;
                                avg_ampr_500[i] = sum_ampr_500[i] / MAX_LEN_CLLST;
                                tmp_volt_24[i] = filter_judge(&ctr_ok_volt_24[i], &ctr_err_volt_24[i], avg_volt_24[i], min_volt_24[i], max_volt_24[i], MAX_LEN_CLLST);
                                tmp_ampr_24[i] = filter_judge(&ctr_ok_ampr_24[i], &ctr_err_ampr_24[i], avg_ampr_24[i], min_ampr_24[i], max_ampr_24[i], MAX_LEN_CLLST);
                                tmp_volt_500[i] = filter_judge(&ctr_ok_volt_500[i], &ctr_err_volt_500[i], avg_volt_500[i], min_volt_500[i], max_volt_500[i], MAX_LEN_CLLST);
                                tmp_ampr_500[i] = filter_judge(&ctr_ok_ampr_500[i], &ctr_err_ampr_500[i], avg_ampr_500[i], min_ampr_500[i], max_ampr_500[i], MAX_LEN_CLLST);
                                if (tmp_volt_24[i] == -1)
                                        result[i] |= RESULT_FAULT_VOLT_24;
                                else if (tmp_volt_24[i] == 1)
                                        result[i] &= ~RESULT_FAULT_VOLT_24;
                                if (tmp_ampr_24[i] == -1)
                                        result[i] |= RESULT_FAULT_AMPR_24;
                                else if (tmp_ampr_24[i] == -1)
                                        result[i] &= ~RESULT_FAULT_AMPR_24;
                                if (tmp_volt_500[i] == -1)
                                        result[i] |= RESULT_FAULT_VOLT_500;
                                else if (tmp_volt_500[i] == 1)
                                        result[i] &= ~RESULT_FAULT_VOLT_500;
                                if (tmp_ampr_500[i] == -1)
                                        result[i] |= RESULT_FAULT_AMPR_500;
                                else if (tmp_ampr_500[i] == -1)
                                        result[i] &= ~RESULT_FAULT_AMPR_500;
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
                        state.type |= TASK_NOTIFY_PSU;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_BRAKE:
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_BRAKE:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.io.brake = (u8)verify.data;
                                        tx[i].data.io.res = 0x66;
                                        tx[i].data.io.xor = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[i], WAIT_FOREVER);
                                        rngBufPut(rng_can[i], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[i]);
                                }
                                period = PERIOD;
                                break;
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_LIGHT:
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_LIGHT:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.io.light = (u8)verify.data;
                                        tx[i].data.io.res = 0x66;
                                        tx[i].data.io.xor = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD;
                                break;
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_24:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.io.v24 = (u16)psu_delay(verify.data, old_cmd[i]);
                                        old_cmd[i] = tx[i].data.io.v24;
                                        tx[i].data.io.res = 0x66;
                                        tx[i].data.io.xor = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD;
                                break;
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_24:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.io.v24 = 0;
                                        tx[i].data.io.res = 0x66;
                                        tx[i].data.io.xor = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD;
                                break;
                        case CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_500:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.io.v500 = (u16)verify.data;
                                        tx[i].data.io.res = 0x66;
                                        tx[i].data.io.xor = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD;
                                break;
                        case CMD_ACT_PSU | CMD_DIR_NEGA | CMD_PSU_500:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.io.v500 = 0;
                                        tx[i].data.io.res = 0x66;
                                        tx[i].data.io.xor = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD;
                                break;
                        default:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].src = J1939_ADDR_MAIN;
                                        tx[i].dest = addr[i];
                                        tx[i].form = 0xA0;
                                        tx[i].prio = 0x08;
                                        tx[i].data.query[0] = 0x00;
                                        tx[i].data.query[1] = 0x00;
                                        tx[i].data.query[2] = 0x00;
                                        tx[i].data.query[3] = 0x00;
                                        tx[i].data.query[4] = 0x00;
                                        tx[i].data.query[5] = 0x00;
                                        tx[i].data.query[6] = 0x66;
                                        tx[i].data.query[7] = check_xor((u8 *)&tx[i].data.io.brake, 7);
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD;
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
