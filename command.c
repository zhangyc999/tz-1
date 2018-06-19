#include "define.h"
#include "struct.h"
#include "vx.h"

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swh;

void psu(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU_24;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swh(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhn(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rse(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsep(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsen(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swv(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvn(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpn(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void x(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_X | CMD_DIR_STOP;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void xp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_X | CMD_MODE_MANUAL | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void xn(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_X | CMD_MODE_MANUAL | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}
