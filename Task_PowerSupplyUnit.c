#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD 50

#define MAX_NUM_DEV   1
#define MAX_NUM_FORM  4
#define MAX_LEN_CLLST 4

#define UNMASK_RESULT_IO      0x000000FF
#define UNMASK_RESULT_FAULT   0x0000FF00
#define UNMASK_RESULT_MISC    0xFFFF0000
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_VOLT_24  0x00000400
#define RESULT_FAULT_AMPR_24  0x00000800
#define RESULT_FAULT_VOLT_500 0x00001000
#define RESULT_FAULT_AMPR_500 0x00002000
#define RESULT_FAULT_COMM     0x00008000
#define RESULT_IO_BRAKE       0x00000001
#define RESULT_IO_LIGHT       0x00000002
#define RESULT_IO_24          0x00000004
#define RESULT_IO_500         0x00000008

typedef struct frame_psu_rx FRAME_RX;
typedef struct frame_psu_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int filter_judge(int *ok, int *err, int value, int min, int max, int ctr);
int psu_delay(int cur, int old);
u8 check_xor(u8 *buf, int n);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_psu;
extern RING_ID rng_can_slow[];
extern SEM_ID sem_can_slow[];

const static int addr = J1939_ADDR_PSU;
const static int cable = 0;
const static int min_volt_24 = 2000;
const static int max_volt_24 = 2800;
const static int min_ampr_24 = 0;
const static int max_ampr_24 = 2000;
const static int min_volt_500 = 4600;
const static int max_volt_500 = 5400;
const static int min_ampr_500 = 0;
const static int max_ampr_500 = 1000;

static int period = PERIOD;
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
static FRAME_TX tx[MAX_NUM_FORM];
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
static int old_io_brake;
static int old_io_light;
static int old_io_24;
static int old_io_500;
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
static int ctr_io_brake;
static int ctr_io_light;
static int ctr_io_24;
static int ctr_io_500;
static int ctr_fault;
static int ctr_comm;
static int tmp_volt_24;
static int tmp_ampr_24;
static int tmp_volt_500;
static int tmp_ampr_500;
static int result;
static int any_fault;
static int j;

void t_psu(void) /* Task: Power Supply Unit */
{
        for (j = 0; j < MAX_NUM_FORM; j++)
                p[j] = (FRAME_RX *)can_cllst_init(rx[j], MAX_LEN_CLLST);
        tx[0].src = J1939_ADDR_MAIN;
        tx[0].dest = addr;
        tx[0].form = 0x5C;
        tx[0].prio = 0x0C;
        tx[0].data.query[0] = 0x00;
        tx[0].data.query[1] = 0x11;
        tx[0].data.query[2] = 0x22;
        tx[0].data.query[3] = 0x33;
        tx[0].data.query[4] = 0x44;
        tx[0].data.query[5] = 0x55;
        tx[0].data.query[6] = 0x66;
        tx[0].data.query[7] = 0x77;
        tx[1].src = J1939_ADDR_MAIN;
        tx[1].dest = addr;
        tx[1].form = 0xA0;
        tx[1].prio = 0x08;
        tx[1].data.io.brake = 0x00;
        tx[1].data.io.light = 0x00;
        tx[1].data.io.v24 = 0x0000;
        tx[1].data.io.v500 = 0x0000;
        tx[1].data.io.res = 0x66;
        tx[1].data.io.xor = 0x00;
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
                        has_received = 1;
                        switch (can.form) {
                        case 0xC0:
                                j = 0;
                                p[j] = p[j]->next;
                                old_io_brake = p[j]->data.io.brake;
                                old_io_light = p[j]->data.io.light;
                                old_io_24 = p[j]->data.io.v24;
                                old_io_500 = p[j]->data.io.v500;
                                old_fault = p[j]->data.io.fault;
                                p[j]->data.io.brake = ((FRAME_RX *)&can)->data.io.brake;
                                p[j]->data.io.light = ((FRAME_RX *)&can)->data.io.light;
                                p[j]->data.io.v24 = ((FRAME_RX *)&can)->data.io.v24;
                                p[j]->data.io.v500 = ((FRAME_RX *)&can)->data.io.v500;
                                p[j]->data.io.fault = ((FRAME_RX *)&can)->data.io.fault;
                                if (old_io_brake == p[j]->data.io.brake && old_io_brake == cmd.data && (cmd.type & UNMASK_CMD_PSU) == CMD_PSU_BRAKE) {
                                        if (ctr_io_brake < MAX_LEN_CLLST)
                                                ctr_io_brake++;
                                } else {
                                        ctr_io_brake = 0;
                                }
                                if (old_io_light == p[j]->data.io.light && old_io_light == cmd.data && (cmd.type & UNMASK_CMD_PSU) == CMD_PSU_LIGHT) {
                                        if (ctr_io_light < MAX_LEN_CLLST)
                                                ctr_io_light++;
                                } else {
                                        ctr_io_light = 0;
                                }
                                if (old_io_24 == p[j]->data.io.v24 && old_io_24 == cmd.data && (cmd.type & UNMASK_CMD_PSU) == CMD_PSU_24) {
                                        if (ctr_io_24 < MAX_LEN_CLLST)
                                                ctr_io_24++;
                                } else {
                                        ctr_io_24 = 0;
                                }
                                if (old_io_500 == p[j]->data.io.v500 && old_io_500 == cmd.data && (cmd.type & UNMASK_CMD_PSU) == CMD_PSU_500) {
                                        if (ctr_io_500 < MAX_LEN_CLLST)
                                                ctr_io_500++;
                                } else {
                                        ctr_io_500 = 0;
                                }
                                if (ctr_io_brake < MAX_LEN_CLLST)
                                        result |= RESULT_IO_BRAKE;
                                else
                                        result &= ~RESULT_IO_BRAKE;
                                if (ctr_io_light < MAX_LEN_CLLST)
                                        result |= RESULT_IO_LIGHT;
                                else
                                        result &= ~RESULT_IO_LIGHT;
                                if (ctr_io_24 < MAX_LEN_CLLST)
                                        result |= RESULT_IO_24;
                                else
                                        result &= ~RESULT_IO_24;
                                if (ctr_io_500 < MAX_LEN_CLLST)
                                        result |= RESULT_IO_500;
                                else
                                        result &= ~RESULT_IO_500;
                                if (old_fault == p[j]->data.io.fault) {
                                        if (ctr_fault < MAX_LEN_CLLST)
                                                ctr_fault++;
                                } else {
                                        ctr_fault = 0;
                                }
                                switch (p[j]->data.io.fault) {
                                case 0x00:
                                case 0x0C:
                                        if (ctr_fault < 2)
                                                break;
                                        result &= ~RESULT_FAULT_GENERAL;
                                        result &= ~RESULT_FAULT_SERIOUS;
                                        break;
                                case 0x03:
                                        if (ctr_fault < 1)
                                                break;
                                        result |= RESULT_FAULT_GENERAL;
                                        break;
                                case 0xF0:
                                        result |= RESULT_FAULT_SERIOUS;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case 0xC3:
                                j = 1;
                                p[j] = p[j]->next;
                                sum_volt_24 -= p[j]->data.state.volt_24;
                                sum_ampr_24 -= p[j]->data.state.ampr_24;
                                sum_volt_500 -= p[j]->data.state.volt_500;
                                sum_ampr_500 -= p[j]->data.state.ampr_500;
                                p[j]->data.state.volt_24 = ((FRAME_RX *)&can)->data.state.volt_24;
                                p[j]->data.state.ampr_24 = ((FRAME_RX *)&can)->data.state.ampr_24;
                                p[j]->data.state.volt_500 = ((FRAME_RX *)&can)->data.state.volt_500;
                                p[j]->data.state.ampr_500 = ((FRAME_RX *)&can)->data.state.ampr_500;
                                cur_volt_24 = p[j]->data.state.volt_24;
                                cur_ampr_24 = p[j]->data.state.ampr_24;
                                cur_volt_500 = p[j]->data.state.volt_500;
                                cur_ampr_500 = p[j]->data.state.ampr_500;
                                sum_volt_24 += p[j]->data.state.volt_24;
                                sum_ampr_24 += p[j]->data.state.ampr_24;
                                sum_volt_500 += p[j]->data.state.volt_500;
                                sum_ampr_500 += p[j]->data.state.ampr_500;
                                avg_volt_24 = sum_volt_24 / MAX_LEN_CLLST;
                                avg_ampr_24 = sum_ampr_24 / MAX_LEN_CLLST;
                                avg_volt_500 = sum_volt_500 / MAX_LEN_CLLST;
                                avg_ampr_500 = sum_ampr_500 / MAX_LEN_CLLST;
                                tmp_volt_24 = filter_judge(&ctr_ok_volt_24, &ctr_err_volt_24, avg_volt_24, min_volt_24, max_volt_24, MAX_LEN_CLLST);
                                tmp_ampr_24 = filter_judge(&ctr_ok_ampr_24, &ctr_err_ampr_24, avg_ampr_24, min_ampr_24, max_ampr_24, MAX_LEN_CLLST);
                                tmp_volt_500 = filter_judge(&ctr_ok_volt_500, &ctr_err_volt_500, avg_volt_500, min_volt_500, max_volt_500, MAX_LEN_CLLST);
                                tmp_ampr_500 = filter_judge(&ctr_ok_ampr_500, &ctr_err_ampr_500, avg_ampr_500, min_ampr_500, max_ampr_500, MAX_LEN_CLLST);
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
                        if (any_fault) {
                                state.type = TASK_STATE_FAULT;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if ((result & UNMASK_RESULT_IO) == 0)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= TASK_NOTIFY_PSU;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        if ((verify.type & UNMASK_CMD_ACT) != CMD_ACT_PSU) {
                                j = 0;
                        } else if ((verify.type & UNMASK_CMD_PSU) == CMD_PSU_BRAKE) {
                                j = 1;
                                tx[j].data.io.brake = (u8)verify.data;
                                tx[j].data.io.xor = check_xor((u8 *)&tx[j].data.io.brake, 7);
                        } else if ((verify.type & UNMASK_CMD_PSU) == CMD_PSU_LIGHT) {
                                j = 1;
                                tx[j].data.io.light = (u8)verify.data;
                                tx[j].data.io.xor = check_xor((u8 *)&tx[j].data.io.brake, 7);
                        } else if ((verify.type & UNMASK_CMD_PSU) == CMD_PSU_24) {
                                j = 1;
                                switch (verify.type & UNMASK_CMD_DIR) {
                                case CMD_DIR_POSI:
                                        tx[j].data.io.v24 = (u16)psu_delay(verify.data, old_cmd);
                                        break;
                                case CMD_DIR_NEGA:
                                        tx[j].data.io.v24 = 0;
                                        break;
                                default:
                                        break;
                                }
                                old_cmd = tx[j].data.io.v24;
                                tx[j].data.io.xor = check_xor((u8 *)&tx[j].data.io.brake, 7);
                        } else if ((verify.type & UNMASK_CMD_PSU) == CMD_PSU_500) {
                                j = 1;
                                switch (verify.type & UNMASK_CMD_DIR) {
                                case CMD_DIR_POSI:
                                        tx[j].data.io.v500 = (u16)verify.data;
                                        break;
                                case CMD_DIR_NEGA:
                                        tx[j].data.io.v500 = 0;
                                        break;
                                default:
                                        break;
                                }
                                tx[j].data.io.xor = check_xor((u8 *)&tx[j].data.io.brake, 7);
                        }
                        semTake(sem_can_slow[cable], WAIT_FOREVER);
                        rngBufPut(rng_can_slow[cable], (char *)&tx[j], sizeof(tx[j]));
                        semGive(sem_can_slow[cable]);
                        period = PERIOD;
                        verify.type = CMD_IDLE;
                        verify.data = 0;
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

u8 check_xor(u8 *buf, int n)
{
        if (n > 1)
                return *buf ^ check_xor(buf + 1, n - 1);
        return *buf;
}
