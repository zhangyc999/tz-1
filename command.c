#include "define.h"
#include "struct.h"
#include "vx.h"

extern MSG_Q_ID msg_main;
extern MSG_Q_ID msg_psu;

void gendp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_GEND | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void gendn(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_GEND | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void gensp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_GENS | CMD_DIR_POSI;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void gensn(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_GENS | CMD_DIR_NEGA;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void psu_brake(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_BRAKE;
        udp.cmd.data = data;
        msgQSend(msg_psu, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void psu_light(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_LIGHT;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void psu_24(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_24;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void psu_500(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PSU | CMD_DIR_POSI | CMD_PSU_500;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void mom(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_MOM | CMD_DIR_STOP | CMD_MODE_MANUAL;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void momp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_MOM | CMD_DIR_POSI | CMD_MODE_MANUAL;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void momn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_MOM | CMD_DIR_NEGA | CMD_MODE_MANUAL;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swh(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swhn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWH | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rse(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsep(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void rsen(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_RSE | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swv(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void swvn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SWV | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prp(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void prpn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_PRP | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdt(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDT | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdtp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDT | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdtn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDT | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sds(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDS | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdsp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDS | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdsn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDS | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdfb(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDFB | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdfbp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDFB | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void sdfbn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_SDFB | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void x(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_X | CMD_DIR_STOP | CMD_MODE_MANUAL;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void xp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_X | CMD_DIR_POSI | CMD_MODE_MANUAL;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void xn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_X | CMD_DIR_NEGA | CMD_MODE_MANUAL;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void y(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_Y | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void yp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_Y | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void yn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_Y | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void z(void)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_Z | CMD_DIR_STOP | CMD_MODE_AUTO;
        udp.cmd.data = 0;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void zp(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_Z | CMD_DIR_POSI | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void zn(int data)
{
        struct frame_udp_rx udp;
        udp.cmd.type = CMD_ACT_Z | CMD_DIR_NEGA | CMD_MODE_AUTO;
        udp.cmd.data = data;
        msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
}

void myswh(void)
{
        struct frame_udp_rx udp;
        for (;;) {
                udp.cmd.type = CMD_ACT_SWH | CMD_DIR_POSI | CMD_MODE_AUTO;
                udp.cmd.data = 0x0005000F;
                msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(sysClkRateGet() * 80);
                udp.cmd.type = CMD_ACT_SWH | CMD_DIR_NEGA | CMD_MODE_AUTO;
                udp.cmd.data = 0x0000000F;
                msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(sysClkRateGet() * 80);
        }
}

void myswv(void)
{
        struct frame_udp_rx udp;
        for (;;) {
                udp.cmd.type = CMD_ACT_SWV | CMD_DIR_POSI | CMD_MODE_AUTO;
                udp.cmd.data = 0x0000000D;
                msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(sysClkRateGet() * 35);
                udp.cmd.type = CMD_ACT_SWV | CMD_DIR_NEGA | CMD_MODE_AUTO;
                udp.cmd.data = 0x0000000D;
                msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(sysClkRateGet() * 35);
        }
}

void myx(void)
{
        struct frame_udp_rx udp;
        for (;;) {
                udp.cmd.type = CMD_ACT_X | CMD_DIR_POSI | CMD_MODE_MANUAL;
                udp.cmd.data = 0x00000003;
                msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(sysClkRateGet() * 160);
                udp.cmd.type = CMD_ACT_X | CMD_DIR_NEGA | CMD_MODE_MANUAL;
                udp.cmd.data = 0x00000003;
                msgQSend(msg_main, (char *)&udp.cmd, sizeof(udp.cmd), NO_WAIT, MSG_PRI_NORMAL);
                taskDelay(sysClkRateGet() * 160);
        }
}
