#include "struct.h"
#include "vx.h"

struct udp_cmd {
        u16 head;
        u8 res0;
        u8 res1;
        struct main cmd;
        u8 res2;
        u8 res3;
        u8 res4;
        u8 check;
};

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_swh;

void psu(int data)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC001002;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swh(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0A2001;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhp(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC002001;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhn(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0C2001;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rse(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0A2002;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsep(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC002002;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsen(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0C2002;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swv(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0A2003;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvp(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC002003;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvn(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0C2003;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prp(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0A2004;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpp(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC002004;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpn(void)
{
        struct udp_cmd udp;
        udp.cmd.type = 0xEC0C2004;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}
