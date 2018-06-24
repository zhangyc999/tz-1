#include "define.h"
#include "struct.h"
#include "vx.h"

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_psu;

void psu_24(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU_24 | CMD_DIR_POSI;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void psu_500(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU_500 | CMD_DIR_POSI;
        udp.cmd.data = data;
        msgQSend(msg_psu, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swh(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhp(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_POSI;
        } else {
                udp.cmd.type = CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_POSI;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhn(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_SWH | CMD_MODE_AUTO | CMD_DIR_NEGA;
        } else {
                udp.cmd.type = CMD_ACT_SWH | CMD_MODE_MANUAL | CMD_DIR_NEGA;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rse(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsep(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_POSI;
        } else {
                udp.cmd.type = CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_POSI;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsen(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_RSE | CMD_MODE_AUTO | CMD_DIR_NEGA;
        } else {
                udp.cmd.type = CMD_ACT_RSE | CMD_MODE_MANUAL | CMD_DIR_NEGA;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swv(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvp(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_POSI;
        } else {
                udp.cmd.type = CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_POSI;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvn(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_SWV | CMD_MODE_AUTO | CMD_DIR_NEGA;
        } else {
                udp.cmd.type = CMD_ACT_SWV | CMD_MODE_MANUAL | CMD_DIR_NEGA;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpp(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_POSI;
        } else {
                udp.cmd.type = CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_POSI;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpn(int data)
{
        struct frame_udp_rx udp;
        if (data == 0) {
                udp.cmd.type = CMD_ACT_PRP | CMD_MODE_AUTO | CMD_DIR_NEGA;
        } else {
                udp.cmd.type = CMD_ACT_PRP | CMD_MODE_MANUAL | CMD_DIR_NEGA;
                udp.cmd.data = data;
        }
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}
