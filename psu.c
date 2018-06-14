#include "define.h"
#include "j1939.h"
#include "struct.h"
#include "type.h"
#include "vx.h"

#define PERIOD_SLOW 200
#define PERIOD_FAST 20

#define MAX_LEN_CLLST 16

#define UNMASK_RESULT_FAULT   0x0000FF00
#define RESULT_FAULT_GENERAL  0x00000100
#define RESULT_FAULT_SERIOUS  0x00000200
#define RESULT_FAULT_VOLT_24  0x00000400
#define RESULT_FAULT_AMPR_24  0x00000800
#define RESULT_FAULT_VOLT_500 0x00001000
#define RESULT_FAULT_AMPR_500 0x00002000
#define RESULT_FAULT_COMM     0x00004000

typedef struct frame_psu_rx FRAME_RX;
typedef struct frame_psu_tx FRAME_TX;

extern u8 check_xor(u8 *buf, int n);
extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_psu;

void t_psu(void)
{
        int period = PERIOD_SLOW;
        u32 prev;
        int len;
        int tmp[sizeof(struct frame_can)];
        struct main *cmd;
        struct main state;
        struct main state_old;
        struct frame_can *can;
        struct frame_can rx[3][MAX_LEN_CLLST];
        FRAME_RX *p[3];
        FRAME_TX tx;
        int verify = CMD_IDLE;
        int has_received = 0;
        int i;
        int max_form = 3;
        u32 sum_volt_24  = 0;
        u32 sum_volt_500 = 0;
        u32 sum_ampr_24  = 0;
        u32 sum_ampr_500 = 0;
        u32 avg_volt_24  = 0;
        u32 avg_volt_500 = 0;
        u32 avg_ampr_24  = 0;
        u32 avg_ampr_500 = 0;
        int fault_old = 0;
        u32 min_volt_24  = 20;
        u32 min_ampr_24  = 0;
        u32 min_volt_500 = 460;
        u32 min_ampr_500 = 0;
        u32 max_volt_24  = 28;
        u32 max_ampr_24  = 10;
        u32 max_volt_500 = 540;
        u32 max_ampr_500 = 1;
        int ctr_ok_volt_24  = 0;
        int ctr_ok_ampr_24  = 0;
        int ctr_ok_volt_500 = 0;
        int ctr_ok_ampr_500 = 0;
        int ctr_err_volt_24  = 0;
        int ctr_err_ampr_24  = 0;
        int ctr_err_volt_500 = 0;
        int ctr_err_ampr_500 = 0;
        int ctr_fault = 0;
        int ctr_comm = 0;
        int result = 0;
        int tmp_volt_24;
        int tmp_ampr_24;
        int tmp_volt_500;
        int tmp_ampr_500;
        for (i = 0; i < max_form; i++)
                p[i] = (FRAME_RX *)can_cllst_init(rx[2][i], MAX_LEN_CLLST);
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_psu, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = (struct main *)&tmp;
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_PSU_24 | CMD_DIR_POSI:
                        case CMD_ACT_PSU_24 | CMD_DIR_NEGA:
                                verify = cmd->type;
                                break;
                        default:
                                break;
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = (struct frame_can *)&tmp;
                        has_received = 1;
                        i = remap_form_index(can->form);
                        switch (i) {
                        case 0:
                                p[i] = p[i]->next;
                                fault_old = p[i]->data.io.fault;
                                p[i]->data.io.fault = ((FRAME_RX *)can)->data.io.fault;
                                if (fault_old == p[i]->data.io.fault) {
                                        if (ctr_fault < 10)
                                                ctr_fault++;
                                } else {
                                        ctr_fault = 0;
                                }
                                switch (p[i]->data.io.fault) {
                                case 0x00:
                                case 0x03:
                                        if (ctr_fault < 5)
                                                break;
                                        result &= ~RESULT_FAULT_GENERAL;
                                        result &= ~RESULT_FAULT_SERIOUS;
                                        break;
                                case 0x0C:
                                        if (ctr_fault < 3)
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
                        case 1:
                                p[i] = p[i]->next;
                                sum_volt_24 -= p[i]->data.state.volt_24;
                                sum_ampr_24 -= p[i]->data.state.ampr_24;
                                sum_volt_500 -= p[i]->data.state.volt_500;
                                sum_ampr_500 -= p[i]->data.state.ampr_500;
                                p[i]->data.state.volt_24 = ((FRAME_RX *)can)->data.state.volt_24;
                                p[i]->data.state.ampr_24 = ((FRAME_RX *)can)->data.state.ampr_24;
                                p[i]->data.state.volt_500 = ((FRAME_RX *)can)->data.state.volt_500;
                                p[i]->data.state.ampr_500 = ((FRAME_RX *)can)->data.state.ampr_500;
                                sum_volt_24 += p[i]->data.state.volt_24;
                                sum_ampr_24 += p[i]->data.state.ampr_24;
                                sum_volt_500 += p[i]->data.state.volt_500;
                                sum_ampr_500 += p[i]->data.state.ampr_500;
                                avg_volt_24 = sum_volt_24 / MAX_LEN_CLLST;
                                avg_ampr_24 = sum_ampr_24 / MAX_LEN_CLLST;
                                avg_volt_500 = sum_volt_500 / MAX_LEN_CLLST;
                                avg_ampr_500 = sum_ampr_500 / MAX_LEN_CLLST;
                                tmp_volt_24 = judge_filter(&ctr_ok_volt_24, &ctr_err_volt_24, avg_volt_24, min_volt_24, max_volt_24, 10);
                                tmp_ampr_24 = judge_filter(&ctr_ok_ampr_24, &ctr_err_ampr_24, avg_ampr_24, min_ampr_24, max_ampr_24, 10);
                                tmp_volt_500 = judge_filter(&ctr_ok_volt_500, &ctr_err_volt_500, avg_volt_500, min_volt_500, max_volt_500, 10);
                                tmp_ampr_500 = judge_filter(&ctr_ok_ampr_500, &ctr_err_ampr_500, avg_ampr_500, min_ampr_500, max_ampr_500, 10);
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
                                if (ctr_comm < 10)
                                        ctr_comm++;
                                if (ctr_comm == 10)
                                        result &= ~RESULT_FAULT_COMM;
                        } else {
                                if (ctr_comm > 0)
                                        ctr_comm = 0;
                                if (ctr_comm > -10)
                                        ctr_comm--;
                                if (ctr_comm == -10)
                                        result |= RESULT_FAULT_COMM;
                        }
                        state.type = TASK_NOTIFY_PSU;
                        state.data = 0;
                        if (result & UNMASK_RESULT_FAULT)
                                state.type |= TASK_STATE_FAULT;
                        else
                                state.type |= TASK_STATE_OK;
                        if (state_old.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_URGENT);
                        state_old = state;
                        switch (verify) {
                        case CMD_ACT_PSU_24 | CMD_DIR_POSI:
                        case CMD_ACT_PSU_24 | CMD_DIR_NEGA:
                                tx.dest = J1939_ADDR_PSU;
                                tx.form = J1939_FORM_PSU_CTRL;
                                tx.prio = J1939_PRIO_PSU_CTRL;
                                tx.data.io.v24 = 0; /* WQ */
                                tx.data.io.v500 = 0; /* WQ */
                                tx.data.io.res = 0x66;
                                tx.data.io.xor = check_xor((u8 *)&tx.data.io.v24, 7);
                                msgQSend(msg_can[0], (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_URGENT);
                                period = PERIOD_FAST;
                                break;
                        default:
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
                                msgQSend(msg_can[0], (char *)&tx, sizeof(tx), NO_WAIT, MSG_PRI_NORMAL);
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}

int psu_delay(struct main *cmd, int cur)
{
        int tmp;
        int i;
        tmp = cmd->data ^ cur;
        for (i = 0; i < sizeof(tmp) - 1; i++) {
                if (tmp & 1 << i)
                        break;
        }
        if (cmd->type == (CMD_ACT_PSU_24 | CMD_DIR_POSI))
                cur |= 1 << i;
        else if (cmd->type == (CMD_ACT_PSU_24 | CMD_DIR_NEGA))
                cur &= ~(1 << i);
        return cur;
}
