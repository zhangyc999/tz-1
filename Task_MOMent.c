#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200
#define PERIOD_FAST 20

#define MAX_NUM_DEV   4
#define MAX_NUM_FORM  3
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
#define RESULT_FAULT_COMM    0x00008000
#define RESULT_STOP          0x00040000
#define RESULT_LOAD          0x00100000

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int remap_form_index(u8 form);
int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_mom;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int addr[MAX_NUM_DEV] = {J1939_ADDR_MOM0, J1939_ADDR_MOM1, J1939_ADDR_MOM2, J1939_ADDR_MOM3};
const static int cable[MAX_NUM_DEV] = {1, 1, 1, 1};
const static int min_vel[MAX_NUM_DEV] = {-1500, -1500, -1500, -1500};
const static int max_vel[MAX_NUM_DEV] = {1500, 1500, 1500, 1500};
const static int min_ampr[MAX_NUM_DEV] = {0, 0, 0, 0};
const static int max_ampr[MAX_NUM_DEV] = {100, 100, 100, 100};
const static int ampr_load[MAX_NUM_DEV] = {50, 50, 50, 50};
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
static int cur_vel[MAX_NUM_DEV];
static int cur_ampr[MAX_NUM_DEV];
static int sum_vel[MAX_NUM_DEV];
static int sum_ampr[MAX_NUM_DEV];
static int avg_vel[MAX_NUM_DEV];
static int avg_ampr[MAX_NUM_DEV];
static int old_fault[MAX_NUM_DEV];
static int old_io[MAX_NUM_DEV];
static int ctr_ok_vel[MAX_NUM_DEV];
static int ctr_ok_ampr[MAX_NUM_DEV];
static int ctr_ok_stop[MAX_NUM_DEV];
static int ctr_ok_load[MAX_NUM_DEV];
static int ctr_err_vel[MAX_NUM_DEV];
static int ctr_err_ampr[MAX_NUM_DEV];
static int ctr_err_stop[MAX_NUM_DEV];
static int ctr_err_load[MAX_NUM_DEV];
static int ctr_fault[MAX_NUM_DEV];
static int ctr_io[MAX_NUM_DEV];
static int ctr_comm[MAX_NUM_DEV];
static int result[MAX_NUM_DEV];
static int tmp_vel;
static int tmp_ampr;
static int tmp_stop;
static int tmp_load;
static int num_load;
static int any_fault;
static int i;
static int j;

void t_mom(void) /* Task: constant MOMent electric machinery */
{
        for (i = 0; i < MAX_NUM_DEV; i++) {
                for (j = 0; j < MAX_NUM_FORM; j++)
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
                        switch (verify.type) {
                        case CMD_IDLE:
                        case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                switch (cmd.type) {
                                case CMD_IDLE:
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
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
                                sum_ampr[i] -= abs(p[i][j]->data.state.ampr);
                                old_fault[i] = p[i][j]->data.state.fault;
                                old_io[i] = p[i][j]->data.state.io;
                                p[i][j]->data.state.vel = ((FRAME_RX *)&can)->data.state.vel;
                                p[i][j]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i][j]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i][j]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                cur_vel[i] = p[i][j]->data.state.vel;
                                cur_ampr[i] = abs(p[i][j]->data.state.ampr);
                                sum_vel[i] += p[i][j]->data.state.vel;
                                sum_ampr[i] += abs(p[i][j]->data.state.ampr);
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
                                        if (ctr_io[i] < MAX_LEN_CLLST)
                                                ctr_io[i]++;
                                } else {
                                        ctr_io[i] = 0;
                                }
                                if (ctr_io[i] > 5)
                                        result[i] = result[i] & ~UNMASK_RESULT_IO | p[i][j]->data.state.io;
                                tmp_vel = judge_filter(&ctr_ok_vel[i], &ctr_err_vel[i], avg_vel[i], min_vel[i], max_vel[i], MAX_LEN_CLLST);
                                tmp_ampr = judge_filter(&ctr_ok_ampr[i], &ctr_err_ampr[i], avg_ampr[i], min_ampr[i], max_ampr[i], MAX_LEN_CLLST);
                                tmp_stop = judge_filter(&ctr_ok_stop[i], &ctr_err_stop[i], avg_vel[i], -5, 5, MAX_LEN_CLLST);
                                tmp_load = judge_filter(&ctr_ok_load[i], &ctr_err_load[i], avg_ampr[i], ampr_load[i], max_ampr[i], MAX_LEN_CLLST);
                                if (tmp_vel == -1)
                                        result[i] |= RESULT_FAULT_VEL;
                                else if (tmp_vel == 1)
                                        result[i] &= ~RESULT_FAULT_VEL;
                                if (tmp_ampr == -1)
                                        result[i] |= RESULT_FAULT_AMPR;
                                else if (tmp_ampr == 1)
                                        result[i] &= ~RESULT_FAULT_AMPR;
                                if (tmp_stop == 1)
                                        result[i] |= RESULT_STOP;
                                else if (tmp_stop == -1)
                                        result[i] &= ~RESULT_STOP;
                                if (tmp_load == 1)
                                        result[i] |= RESULT_LOAD;
                                else if (tmp_load == -1)
                                        result[i] &= ~RESULT_LOAD;
                                break;
                        default:
                                break;
                        }
                        num_load = 0;
                        for (i = 0; i < MAX_NUM_DEV; i++) {
                                if (result[i] & RESULT_LOAD)
                                        num_load++;
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
                        any_fault = (result[0] | result[1] | result[2] | result[3]) & UNMASK_RESULT_FAULT;
                        if (any_fault) {
                                state.type = TASK_STATE_FAULT;
                                verify.type = verify.type & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (num_load > 3)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= TASK_NOTIFY_MOM;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_VEL;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0;
                                        tx[i].data.cmd.ampr = 1000;
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        if (result[i] & RESULT_STOP)
                                                tx[i].data.cmd.enable = J1939_SERVO_DISABLE;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (tx[i].data.cmd.enable != J1939_SERVO_DISABLE)
                                                break;
                                }
                                if (i == MAX_NUM_DEV)
                                        period = PERIOD_SLOW;
                                else
                                        period = PERIOD_FAST;
                                break;
                        case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_POSI:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_AMPR;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0x3322;
                                        if (avg_ampr[i] < ampr_load[i] - 5)
                                                tx[i].data.cmd.ampr = -(s16)(avg_ampr[i] + 5);
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                tx[i].data.cmd.vel = (s16)plan_vel_medium[i];
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                                tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        } else {
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
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_MOM | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        tx[i].dest = addr[i];
                                        tx[i].form = J1939_FORM_SERVO_AMPR;
                                        tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                        tx[i].data.cmd.pos = 0x1100;
                                        tx[i].data.cmd.vel = 0x3322;
                                        if (avg_ampr[i] < ampr_load[i] - 5)
                                                tx[i].data.cmd.ampr = -(s16)(avg_ampr[i] + 5);
                                        tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                        tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                        semTake(sem_can[cable[i]], WAIT_FOREVER);
                                        rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                        semGive(sem_can[cable[i]]);
                                }
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_MOM | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
                                        if (verify.data & 1 << i) {
                                                tx[i].dest = addr[i];
                                                tx[i].form = J1939_FORM_SERVO_VEL;
                                                tx[i].prio = J1939_PRIO_SERVO_CTRL;
                                                tx[i].data.cmd.pos = 0x1100;
                                                tx[i].data.cmd.vel = -(s16)plan_vel_medium[i];
                                                tx[i].data.cmd.ampr = 1000;
                                                tx[i].data.cmd.exec = J1939_SERVO_ASYNC;
                                                tx[i].data.cmd.enable = J1939_SERVO_ENABLE;
                                                semTake(sem_can[cable[i]], WAIT_FOREVER);
                                                rngBufPut(rng_can[cable[i]], (char *)&tx[i], sizeof(tx[i]));
                                                semGive(sem_can[cable[i]]);
                                        } else {
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
                                }
                                period = PERIOD_FAST;
                                break;
                        default:
                                for (i = 0; i < MAX_NUM_DEV; i++) {
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
                        break;
                }
        }
}
