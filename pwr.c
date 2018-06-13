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

extern u8 check_xor(u8 *buf, int n);
extern MSG_Q_ID msg_can[];
extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_pwr;

void t_pwr(void)
{
        int period = PERIOD_SLOW;
        u32 prev;
        int len;
        int tmp[sizeof(struct frame_can)];
        struct main *cmd;
        struct main state;
        struct main state_old;
        struct frame_can *can;
        struct frame_can rx[3][3][MAX_LEN_CLLST];
        struct frame_gend_rx *p_gend[3];
        struct frame_gens_rx *p_gens[3];
        struct frame_psu_rx *p_psu[3];
        struct frame_gend_tx tx_gend;
        struct frame_gens_tx tx_gens;
        struct frame_psu_tx tx_psu;
        int verify = CMD_IDLE;
        int has_received_psu = 0;
        int n = 3;
        int i;
        int max_form = 3;
        u32 sum_psu_volt_24  = 0;
        u32 sum_psu_volt_500 = 0;
        u32 sum_psu_ampr_24  = 0;
        u32 sum_psu_ampr_500 = 0;
        u32 avg_psu_volt_24  = 0;
        u32 avg_psu_volt_500 = 0;
        u32 avg_psu_ampr_24  = 0;
        u32 avg_psu_ampr_500 = 0;
        int fault_old_psu = 0;
        u32 min_psu_volt_24  = 20;
        u32 max_psu_volt_24  = 28;
        u32 min_psu_volt_500 = 460;
        u32 max_psu_volt_500 = 540;
        u32 min_psu_ampr_24  = 0;
        u32 max_psu_ampr_24  = 10;
        u32 min_psu_ampr_500 = 0;
        u32 max_psu_ampr_500 = 1;
        int ctr_ok_psu_volt_24  = 0;
        int ctr_ok_psu_ampr_24  = 0;
        int ctr_ok_psu_volt_500 = 0;
        int ctr_ok_psu_ampr_500 = 0;
        int ctr_err_psu_volt_24  = 0;
        int ctr_err_psu_ampr_24  = 0;
        int ctr_err_psu_volt_500 = 0;
        int ctr_err_psu_ampr_500 = 0;
        int ctr_fault_psu = 0;
        int ctr_comm_psu = 0;
        int result_psu = 0;
        int tmp_psu_volt_24;
        int tmp_psu_ampr_24;
        int tmp_psu_volt_500;
        int tmp_psu_ampr_500;
        for (i = 0; i < max_form; i++) {
                p_gend[i] = (struct frame_gend_rx *)can_cllst_init(rx[0][i], MAX_LEN_CLLST);
                p_gens[i] = (struct frame_gens_rx *)can_cllst_init(rx[1][i], MAX_LEN_CLLST);
                p_psu[i] = (struct frame_psu_rx *)can_cllst_init(rx[2][i], MAX_LEN_CLLST);
        }
        for (;;) {
                prev = tickGet();
                if (period < 0 || period > PERIOD_SLOW)
                        period = 0;
                len = msgQReceive(msg_pwr, (char *)&tmp, sizeof(tmp), period);
                switch (len) {
                case sizeof(struct main):
                        cmd = (struct main *)&tmp;
                        switch (verify) {
                        case CMD_IDLE:
                        case CMD_ACT_GEND | CMD_DIR_POSI:
                        case CMD_ACT_GEND | CMD_DIR_NEGA:
                        case CMD_ACT_GENS | CMD_DIR_POSI:
                        case CMD_ACT_GENS | CMD_DIR_NEGA:
                        case CMD_ACT_PSU | CMD_DIR_POSI:
                        case CMD_ACT_PSU | CMD_DIR_NEGA:
                                verify = cmd->type;
                                break;
                        default:
                                break;
                        }
                        period -= tickGet() - prev;
                        break;
                case sizeof(struct frame_can):
                        can = (struct frame_can *)&tmp;
                        switch (can->src) {
                        case J1939_ADDR_GEND:
                        case J1939_ADDR_GENS:
                                break;
                        case J1939_ADDR_PSU:
                                has_received_psu = 1;
                                i = remap_form_index(can->form);
                                switch (i) {
                                case 0:
                                        if (fault_old_psu == p_psu[i]->data.io.fault) {
                                                if (ctr_fault_psu < 10)
                                                        ctr_fault_psu++;
                                        } else {
                                                ctr_fault_psu = 0;
                                        }
                                        switch (p_psu[i]->data.io.fault) {
                                        case 0x00:
                                        case 0x03:
                                                if (ctr_fault_psu < 5)
                                                        break;
                                                result_psu &= ~RESULT_FAULT_GENERAL;
                                                result_psu &= ~RESULT_FAULT_SERIOUS;
                                                break;
                                        case 0x0C:
                                                if (ctr_fault_psu < 3)
                                                        break;
                                                result_psu |= RESULT_FAULT_GENERAL;
                                                break;
                                        case 0xF0:
                                                result_psu |= RESULT_FAULT_SERIOUS;
                                                break;
                                        default:
                                                break;
                                        }
                                        break;
                                case 1:
                                        p_psu[i] = p_psu[i]->next;
                                        sum_psu_volt_24 -= p_psu[i]->data.state.volt_24;
                                        sum_psu_ampr_24 -= p_psu[i]->data.state.ampr_24;
                                        sum_psu_volt_500 -= p_psu[i]->data.state.volt_500;
                                        sum_psu_ampr_500 -= p_psu[i]->data.state.ampr_500;
                                        p_psu[i]->data.state.volt_24 = ((struct frame_psu_rx *)can)->data.state.volt_24;
                                        p_psu[i]->data.state.ampr_24 = ((struct frame_psu_rx *)can)->data.state.ampr_24;
                                        p_psu[i]->data.state.volt_500 = ((struct frame_psu_rx *)can)->data.state.volt_500;
                                        p_psu[i]->data.state.ampr_500 = ((struct frame_psu_rx *)can)->data.state.ampr_500;
                                        sum_psu_volt_24 += p_psu[i]->data.state.volt_24;
                                        sum_psu_ampr_24 += p_psu[i]->data.state.ampr_24;
                                        sum_psu_volt_500 += p_psu[i]->data.state.volt_500;
                                        sum_psu_ampr_500 += p_psu[i]->data.state.ampr_500;
                                        avg_psu_volt_24 = sum_psu_volt_24 / MAX_LEN_CLLST;
                                        avg_psu_ampr_24 = sum_psu_ampr_24 / MAX_LEN_CLLST;
                                        avg_psu_volt_500 = sum_psu_volt_500 / MAX_LEN_CLLST;
                                        avg_psu_ampr_500 = sum_psu_ampr_500 / MAX_LEN_CLLST;
                                        tmp_psu_volt_24 = judge_filter(&ctr_ok_psu_volt_24, &ctr_err_psu_volt_24, avg_psu_volt_24, min_psu_volt_24, max_psu_volt_24, 10);
                                        tmp_psu_ampr_24 = judge_filter(&ctr_ok_psu_ampr_24, &ctr_err_psu_ampr_24, avg_psu_ampr_24, min_psu_ampr_24, max_psu_ampr_24, 10);
                                        tmp_psu_volt_500 = judge_filter(&ctr_ok_psu_volt_500, &ctr_err_psu_volt_500, avg_psu_volt_500, min_psu_volt_500, max_psu_volt_500, 10);
                                        tmp_psu_ampr_500 = judge_filter(&ctr_ok_psu_ampr_500, &ctr_err_psu_ampr_500, avg_psu_ampr_500, min_psu_ampr_500, max_psu_ampr_500, 10);
                                        if (tmp_psu_volt_24 == -1)
                                                result_psu |= RESULT_FAULT_VOLT_24;
                                        else if (tmp_psu_volt_24 == 1)
                                                result_psu &= ~RESULT_FAULT_VOLT_24;
                                        if (tmp_psu_ampr_24 == -1)
                                                result_psu |= RESULT_FAULT_AMPR_24;
                                        else if (tmp_psu_ampr_24 == -1)
                                                result_psu &= ~RESULT_FAULT_AMPR_24;
                                        if (tmp_psu_volt_500 == -1)
                                                result_psu |= RESULT_FAULT_VOLT_500;
                                        else if (tmp_psu_volt_500 == 1)
                                                result_psu &= ~RESULT_FAULT_VOLT_500;
                                        if (tmp_psu_ampr_500 == -1)
                                                result_psu |= RESULT_FAULT_AMPR_500;
                                        else if (tmp_psu_ampr_500 == -1)
                                                result_psu &= ~RESULT_FAULT_AMPR_500;
                                        break;
                                case 2:
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
                default:
                        if (has_received_psu) {
                                has_received_psu = 0;
                                if (ctr_comm_psu < 0)
                                        ctr_comm_psu = 0;
                                if (ctr_comm_psu < 10)
                                        ctr_comm_psu++;
                                if (ctr_comm_psu == 10)
                                        result_psu &= ~RESULT_FAULT_COMM;
                        } else {
                                if (ctr_comm_psu > 0)
                                        ctr_comm_psu = 0;
                                if (ctr_comm_psu > -10)
                                        ctr_comm_psu--;
                                if (ctr_comm_psu == -10)
                                        result_psu |= RESULT_FAULT_COMM;
                        }
                        state.type = TASK_NOTIFY_PWR;
                        state.data = 0;
                        if (result_psu & UNMASK_RESULT_FAULT)
                                state.type |= TASK_STATE_FAULT;
                        else
                                state.type |= TASK_STATE_OK;
                        if (state_old.type != state.type)
                                msgQSend(msg_main, (char *)&state, sizeof(state), NO_WAIT, MSG_PRI_URGENT);
                        state_old = state;
                        switch (verify) {
                        case CMD_ACT_GEND | CMD_DIR_POSI:
                        case CMD_ACT_GEND | CMD_DIR_NEGA:
                        case CMD_ACT_GENS | CMD_DIR_POSI:
                        case CMD_ACT_GENS | CMD_DIR_NEGA:
                                period = PERIOD_FAST;
                                break;
                        case CMD_ACT_PSU | CMD_DIR_POSI:
                        case CMD_ACT_PSU | CMD_DIR_NEGA:
                                tx_gend.dest = J1939_ADDR_GEND;
                                tx_gend.form = J1939_FORM_QUERY;
                                tx_gend.prio = J1939_PRIO_QUERY;
                                tx_gend.data.query[0] = 0x00;
                                tx_gend.data.query[1] = 0x11;
                                tx_gend.data.query[2] = 0x22;
                                tx_gend.data.query[3] = 0x33;
                                tx_gend.data.query[4] = 0x44;
                                tx_gend.data.query[5] = 0x55;
                                tx_gend.data.query[6] = 0x66;
                                tx_gend.data.query[7] = 0x77;
                                tx_gens.dest = J1939_ADDR_GENS;
                                tx_gens.form = J1939_FORM_QUERY;
                                tx_gens.prio = J1939_PRIO_QUERY;
                                tx_gens.data.query[0] = 0x00;
                                tx_gens.data.query[1] = 0x11;
                                tx_gens.data.query[2] = 0x22;
                                tx_gens.data.query[3] = 0x33;
                                tx_gens.data.query[4] = 0x44;
                                tx_gens.data.query[5] = 0x55;
                                tx_gens.data.query[6] = 0x66;
                                tx_gens.data.query[7] = 0x77;
                                tx_psu.dest = J1939_ADDR_PSU;
                                tx_psu.form = J1939_FORM_PSU_CTRL;
                                tx_psu.prio = J1939_PRIO_PSU_CTRL;
                                tx_psu.data.io.v24 = 0; /* WQ */
                                tx_psu.data.io.v500 = 0; /* WQ */
                                tx_psu.data.io.res = 0x66;
                                tx_psu.data.io.xor = check_xor((u8 *)&tx_psu.data.io.v24, 7);
                                msgQSend(msg_can[1], (char *)&tx_gend, sizeof(tx_gend), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_can[1], (char *)&tx_gens, sizeof(tx_gens), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_can[0], (char *)&tx_psu, sizeof(tx_psu), NO_WAIT, MSG_PRI_URGENT);
                                period = PERIOD_FAST;
                                break;
                        default:
                                tx_gend.dest = J1939_ADDR_GEND;
                                tx_gend.form = J1939_FORM_QUERY;
                                tx_gend.prio = J1939_PRIO_QUERY;
                                tx_gend.data.query[0] = 0x00;
                                tx_gend.data.query[1] = 0x11;
                                tx_gend.data.query[2] = 0x22;
                                tx_gend.data.query[3] = 0x33;
                                tx_gend.data.query[4] = 0x44;
                                tx_gend.data.query[5] = 0x55;
                                tx_gend.data.query[6] = 0x66;
                                tx_gend.data.query[7] = 0x77;
                                tx_gens.dest = J1939_ADDR_GENS;
                                tx_gens.form = J1939_FORM_QUERY;
                                tx_gens.prio = J1939_PRIO_QUERY;
                                tx_gens.data.query[0] = 0x00;
                                tx_gens.data.query[1] = 0x11;
                                tx_gens.data.query[2] = 0x22;
                                tx_gens.data.query[3] = 0x33;
                                tx_gens.data.query[4] = 0x44;
                                tx_gens.data.query[5] = 0x55;
                                tx_gens.data.query[6] = 0x66;
                                tx_gens.data.query[7] = 0x77;
                                tx_psu.dest = J1939_ADDR_PSU;
                                tx_psu.form = J1939_FORM_QUERY;
                                tx_psu.prio = J1939_PRIO_QUERY;
                                tx_psu.data.query[0] = 0x00;
                                tx_psu.data.query[1] = 0x11;
                                tx_psu.data.query[2] = 0x22;
                                tx_psu.data.query[3] = 0x33;
                                tx_psu.data.query[4] = 0x44;
                                tx_psu.data.query[5] = 0x55;
                                tx_psu.data.query[6] = 0x66;
                                tx_psu.data.query[7] = 0x77;
                                msgQSend(msg_can[1], (char *)&tx_gend, sizeof(tx_gend), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_can[1], (char *)&tx_gens, sizeof(tx_gens), NO_WAIT, MSG_PRI_NORMAL);
                                msgQSend(msg_can[0], (char *)&tx_psu, sizeof(tx_psu), NO_WAIT, MSG_PRI_NORMAL);
                                period = PERIOD_SLOW;
                                break;
                        }
                        break;
                }
        }
}

int psu_delay(struct main *cmd, int state)
{
        int tmp;
        int i;
        tmp = cmd->data ^ state;
        for (i = 0; i < 32; i++) {
                if (tmp & (0x00000001 << i))
                        break;
        }
        if (cmd->type == (CMD_ACT_PSU | CMD_DIR_POSI))
                state |= (1 << i);
        else if (cmd->type == (CMD_ACT_PSU | CMD_DIR_NEGA))
                state &= ~(1 << i);
        return state;
}
