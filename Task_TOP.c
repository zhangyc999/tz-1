#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200
#define PERIOD_FAST 20

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
#define RESULT_FAULT_SYNC    0x00004000
#define RESULT_FAULT_COMM    0x00008000
#define RESULT_ZERO          0x00010000
#define RESULT_DEST          0x00020000
#define RESULT_STOP          0x00040000

typedef struct frame_cyl_rx FRAME_RX;
typedef struct frame_cyl_tx FRAME_TX;

struct frame_can *can_cllst_init(struct frame_can buf[], int len);
int remap_form_index(u8 form);
int judge_filter(int *ok, int *err, int value, int min, int max, int ctr);
void plan(int *vel, int len_total, int *len_pass, struct plan *plan_len, struct plan max_plan_len, int plan_vel_low, int plan_vel_high, int period);

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_top;
extern RING_ID rng_can[];
extern SEM_ID sem_can[];

const static int io_pos_zero = 100;
const static int io_pos_dest = 20000;
const static int min_pos = -1000;
const static int max_pos = 36000;
const static int min_vel = -1500;
const static int max_vel = 1500;
const static int min_ampr = 0;
const static int max_ampr = 200;
const static int pos_zero = 500;
const static int pos_dest = 20000;
const static int ampr_zero = 100;
const static int ampr_dest = 100;
const static int ampr_load = 150;
const static struct plan max_plan_len = {1000, 4000, 10000};
const static int plan_vel_low = 100;
const static int plan_vel_high = 1000;
const static int plan_vel_medium = 500;

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
static int cur_pos;
static int cur_vel;
static int cur_ampr;
static int sum_pos;
static int sum_vel;
static int sum_ampr;
static int avg_pos;
static int avg_vel;
static int avg_ampr;
static int old_fault;
static int old_io;
static int ctr_ok_pos;
static int ctr_ok_vel;
static int ctr_ok_ampr;
static int ctr_ok_zero;
static int ctr_ok_dest;
static int ctr_ok_stop;
static int ctr_err_pos;
static int ctr_err_vel;
static int ctr_err_ampr;
static int ctr_err_zero;
static int ctr_err_dest;
static int ctr_err_stop;
static int ctr_fault;
static int ctr_io;
static int ctr_comm;
static int result;
static int tmp_pos;
static int tmp_vel;
static int tmp_ampr;
static int tmp_zero;
static int tmp_dest;
static int tmp_stop;
static int plan_vel;
static int plan_len_pass;
static int plan_len_posi;
static int plan_len_nega;
static struct plan plan_len;
static int i;

void t_top(void) /* Task: TOP lengthwise electric machinery */
{
        for (i = 0; i < MAX_NUM_FORM; i++)
                p[i] = (FRAME_RX *)can_cllst_init(rx[i], MAX_LEN_CLLST);
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_top, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = *(struct main *)tmp;
                        switch (verify.type) {
                        case CMD_IDLE:
                        case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                switch (cmd.type) {
                                case CMD_IDLE:
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_POSI:
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                switch (cmd.type) {
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                        verify = cmd;
                                        break;
                                default:
                                        break;
                                }
                                break;
                        case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                switch (cmd.type) {
                                case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
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
                        has_received = 1;
                        i = remap_form_index(can.form);
                        switch (i) {
                        case 0:
                        case 1:
                                break;
                        case 2:
                                p[i] = p[i]->next;
                                sum_pos -= (int)(p[i]->data.state.pos) * 20;
                                sum_vel -= p[i]->data.state.vel;
                                sum_ampr -= abs(p[i]->data.state.ampr);
                                old_fault = p[i]->data.state.fault;
                                old_io = p[i]->data.state.io;
                                p[i]->data.state.pos = ((FRAME_RX *)&can)->data.state.pos;
                                p[i]->data.state.vel = ((FRAME_RX *)&can)->data.state.vel;
                                p[i]->data.state.ampr = ((FRAME_RX *)&can)->data.state.ampr;
                                p[i]->data.state.fault = ((FRAME_RX *)&can)->data.state.fault;
                                p[i]->data.state.io = ((FRAME_RX *)&can)->data.state.io;
                                cur_pos = (int)(p[i]->data.state.pos) * 20;
                                cur_vel = p[i]->data.state.vel;
                                cur_ampr = abs(p[i]->data.state.ampr);
                                sum_pos += (int)(p[i]->data.state.pos) * 20;
                                sum_vel += p[i]->data.state.vel;
                                sum_ampr += abs(p[i]->data.state.ampr);
                                avg_pos = sum_pos / MAX_LEN_CLLST;
                                avg_vel = sum_vel / MAX_LEN_CLLST;
                                avg_ampr = sum_ampr / MAX_LEN_CLLST;
                                if (old_fault == p[i]->data.state.fault) {
                                        if (ctr_fault < MAX_LEN_CLLST)
                                                ctr_fault++;
                                } else {
                                        ctr_fault = 0;
                                }
                                switch (p[i]->data.state.fault) {
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
                                if (old_io == p[i]->data.state.io) {
                                        if (ctr_io < MAX_LEN_CLLST)
                                                ctr_io++;
                                } else {
                                        ctr_io = 0;
                                }
                                if (ctr_io > 5)
                                        result = result & ~UNMASK_RESULT_IO | p[i]->data.state.io;
#if 0
                                if (avg_pos > io_pos_dest + 500 && (result & 0x0000000C) != 0x00000004
                                    || avg_pos < io_pos_zero - 500 && (result & 0x0000000C) != 0x00000008
                                    || avg_pos > io_pos_zero + 500 && avg_pos < io_pos_dest - 500 && (result & 0x0000000C) != 0x0000000C
                                    || avg_pos >= io_pos_dest - 500 && avg_pos <= io_pos_dest + 500 && result & 0x00000008
                                    || avg_pos >= io_pos_zero - 500 && avg_pos <= io_pos_zero + 500 && result & 0x00000004)
                                        result |= RESULT_FAULT_IO;
                                else
                                        result &= ~RESULT_FAULT_IO;
#endif
                                tmp_pos = judge_filter(&ctr_ok_pos, &ctr_err_pos, avg_pos, min_pos, max_pos, MAX_LEN_CLLST);
                                tmp_vel = judge_filter(&ctr_ok_vel, &ctr_err_vel, avg_vel, min_vel, max_vel, MAX_LEN_CLLST);
                                tmp_ampr = judge_filter(&ctr_ok_ampr, &ctr_err_ampr, avg_ampr, min_ampr, max_ampr, MAX_LEN_CLLST);
                                tmp_zero = judge_filter(&ctr_ok_zero, &ctr_err_zero, avg_pos, min_pos, pos_zero, MAX_LEN_CLLST);
                                tmp_dest = judge_filter(&ctr_ok_dest, &ctr_err_dest, avg_pos, pos_dest, max_pos, MAX_LEN_CLLST);
                                tmp_stop = judge_filter(&ctr_ok_stop, &ctr_err_stop, avg_vel, -5, 5, MAX_LEN_CLLST);
                                if (tmp_pos == -1)
                                        result |= RESULT_FAULT_POS;
                                else if (tmp_pos == 1)
                                        result &= ~RESULT_FAULT_POS;
                                if (tmp_vel == -1)
                                        result |= RESULT_FAULT_VEL;
                                else if (tmp_vel == 1)
                                        result &= ~RESULT_FAULT_VEL;
                                if (tmp_ampr == -1)
                                        result |= RESULT_FAULT_AMPR;
                                else if (tmp_ampr == 1)
                                        result &= ~RESULT_FAULT_AMPR;
                                if (tmp_zero == 1)
                                        result |= RESULT_ZERO;
                                else if (tmp_zero == -1)
                                        result &= ~RESULT_ZERO;
                                if (tmp_dest == 1)
                                        result |= RESULT_DEST;
                                else if (tmp_dest == -1)
                                        result &= ~RESULT_DEST;
                                if (tmp_stop == 1)
                                        result |= RESULT_STOP;
                                else if (tmp_stop == -1)
                                        result &= ~RESULT_STOP;
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
                        if (result & UNMASK_RESULT_FAULT) {
                                state.type = TASK_STATE_FAULT;
                                verify.type = verify.type & ~UNMASK_CMD_DIR | CMD_DIR_STOP;
                        } else {
                                state.type = TASK_STATE_RUNNING;
                                if (result & RESULT_ZERO)
                                        state.type = TASK_STATE_ZERO;
                                else if (result & RESULT_DEST)
                                        state.type = TASK_STATE_DEST;
                        }
                        state.type |= TASK_NOTIFY_TOP;
                        state.data = 0;
                        if (old_state.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_NORMAL);
                        old_state = state;
                        switch (verify.type) {
                        case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_STOP:
                        case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_STOP:
                                plan_vel = 0;
                                plan_len_pass = 0;
                                plan_len_posi = pos_dest - cur_pos;
                                plan_len_nega = cur_pos - pos_zero;
                                tx.dest = J1939_ADDR_TOP;
                                tx.form = J1939_FORM_SERVO_VEL;
                                tx.prio = J1939_PRIO_SERVO_CTRL;
                                tx.data.cmd.pos = 0x1100;
                                tx.data.cmd.vel = 0;
                                tx.data.cmd.ampr = 1000;
                                tx.data.cmd.exec = J1939_SERVO_ASYNC;
                                if (result & RESULT_STOP)
                                        tx.data.cmd.enable = J1939_SERVO_DISABLE;
                                semTake(sem_can[1], WAIT_FOREVER);
                                rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
                                semGive(sem_can[1]);
                                if (tx.data.cmd.enable == J1939_SERVO_DISABLE)
                                        period = PERIOD_SLOW;
                                else
                                        period = PERIOD_FAST;
                                break;
                        case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_POSI:
                        case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_POSI:
                                tx.dest = J1939_ADDR_TOP;
                                tx.form = J1939_FORM_SERVO_VEL;
                                tx.prio = J1939_PRIO_SERVO_CTRL;
                                tx.data.cmd.pos = 0x1100;
                                if (result & RESULT_DEST) {
                                        tx.data.cmd.vel = 0;
                                        plan_len_posi = 0;
                                } else {
                                        plan(&plan_vel, plan_len_posi, &plan_len_pass,
                                             &plan_len, max_plan_len, plan_vel_low, plan_vel_high, PERIOD_FAST);
                                        tx.data.cmd.vel = (s16)plan_vel;
                                }
                                tx.data.cmd.ampr = 1000;
                                tx.data.cmd.exec = J1939_SERVO_ASYNC;
                                tx.data.cmd.enable = J1939_SERVO_ENABLE;
                                semTake(sem_can[1], WAIT_FOREVER);
                                rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
                                semGive(sem_can[1]);
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_TOP | CMD_MODE_AUTO | CMD_DIR_NEGA:
                        case CMD_ACT_TOP | CMD_MODE_MANUAL | CMD_DIR_NEGA:
                                tx.dest = J1939_ADDR_TOP;
                                tx.form = J1939_FORM_SERVO_VEL;
                                tx.prio = J1939_PRIO_SERVO_CTRL;
                                tx.data.cmd.pos = 0x1100;
                                if (result & RESULT_ZERO) {
                                        tx.data.cmd.vel = 0;
                                        plan_len_nega = 0;
                                } else {
                                        plan(&plan_vel, plan_len_nega, &plan_len_pass,
                                             &plan_len, max_plan_len, plan_vel_low, plan_vel_high, PERIOD_FAST);
                                        tx.data.cmd.vel = -(s16)plan_vel;
                                }
                                tx.data.cmd.ampr = 1000;
                                tx.data.cmd.exec = J1939_SERVO_ASYNC;
                                tx.data.cmd.enable = J1939_SERVO_ENABLE;
                                semTake(sem_can[1], WAIT_FOREVER);
                                rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
                                semGive(sem_can[1]);
                                period = PERIOD_FAST;
                                break;
                        default:
                                tx.dest = J1939_ADDR_TOP;
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
                                semTake(sem_can[1], WAIT_FOREVER);
                                rngBufPut(rng_can[1], (char *)&tx, sizeof(tx));
                                semGive(sem_can[1]);
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}
