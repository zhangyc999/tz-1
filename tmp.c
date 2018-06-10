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

void tmp(int type)
{
        struct udp_cmd udp;
        udp.cmd.type = type;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}
